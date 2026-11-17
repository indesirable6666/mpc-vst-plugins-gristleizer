/* EMX-1 Virtual v0.4 (v0.3 + 11 types d'oscillateur, accords, drive et BPF+, ruban à 8 zones, slider à retour au centre) : 14 parties (5 synthés + 9 batteries), oscillateurs (Wave/Dual/Sync/Ring) ou samples, filtre,
 * LFO, séquenceur 16 pas avec enregistrement, 3 processeurs d'effets (16 effets de l'EMX-1), saturation « Valve ».
 * Les réglages affichés sont ceux de la partie sélectionnée, comme sur la machine. Les WAV ne sont ouverts qu'en lecture. */
#define _GNU_SOURCE
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>
#include "../../wrapper/engine.h"
#include "../fx.h"

#ifndef MEDIA_ROOT
#define MEDIA_ROOT "/media"
#endif
#define NP 14
#define NSYN 5
#define NVMAX 8
#define SR 44100.0
#define MAXF 2000
#define PLEN 512
#define MAX_DEPTH 6
#define MAX_BYTES (24u * 1024u * 1024u)

typedef struct { float *d; int frames, ch, rate; } smp_t;

typedef struct {
    int stage, note, src, gate;
    float vel, env;
    double pos, inc;
    float ph, ph2, ph3, oinc, nz, fic1[2], fic2[2];
    unsigned rnd;
    int base;
    smp_t *smp;
    unsigned age;
    float fenv, ic1[2], ic2[2], a1, a2, a3;
    unsigned cnt;
} voice_t;

typedef struct {
    int source, root, note, loop, ftype, maxv, sample, want;
    int otype, lshape, ldest, fxsel;
    float level, pan, attack_ms, release_ms, cutoff, reso, fenv_amt, fdecay_ms;
    float oedit1, oedit2, lspeed, ldepth, fxsend;
    int arp, ascale, atype, arate, acenter;      /* arpégiateur */
    float apos, agate;                           /* « slider » (-100..100) et durée de note (%) */
    int nh, nm, aidx, chord, chtype, spring;
    float fdrive, fa1[2], fa2[2], fa3[2], fk;
    uint64_t alast;
    uint8_t held[8], memo[8];
    double aclk;
    unsigned arnd;
    unsigned steps;
    unsigned char snote[16];
    float lph, pf, cm, am, panm;        /* LFO : phase, hauteur, coupure, volume, panoramique */
    _Atomic(smp_t *) cur;
    int req, req_id, done_id;
    char name[96], info[160];
    voice_t v[NVMAX];
} part_t;

typedef struct {
    part_t p[NP];
    int sel, run, rec, step, stepn, rib;
    float bend;
    uint64_t fcount;
    float tempo, swing, master, tube, fxret;
    int fxchain, fxon[3], fxtype[3];
    float fxe1[3], fxe2[3];
    fx_t *fx[3];
    int fxcur[3];
    double clk;
    unsigned clock, ctl;
    uint64_t chg;
    int chg_step, chg_run;
    atomic_uint epoch;
    atomic_int quit;
    pthread_t th;
    int th_started;
    pthread_mutex_t mx;
    pthread_cond_t cv;
    char (*paths)[PLEN];
    int nfiles, scanning, truncated;
    char scaninfo[128];
} emx_t;

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

/* ---------- lecture des WAV ---------- */
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static float conv(const uint8_t *p, int fmt, int bits) {
    float x;
    if (fmt == 3) { uint32_t u = rd32(p); memcpy(&x, &u, 4); if (!isfinite(x)) x = 0.0f; }
    else if (bits == 8) x = ((int)p[0] - 128) / 128.0f;
    else if (bits == 16) x = (int16_t)rd16(p) / 32768.0f;
    else if (bits == 24) { int32_t v = (int32_t)(p[0] | (p[1] << 8) | (p[2] << 16)); if (v & 0x800000) v |= ~0xFFFFFF; x = v / 8388608.0f; }
    else x = (int32_t)rd32(p) / 2147483648.0f;
    return clampf(x, -1.0f, 1.0f);
}

static smp_t *wav_load(const char *path, char *err, int elen) {
    FILE *f = fopen(path, "rb");
    if (!f) { snprintf(err, elen, "fichier illisible"); return NULL; }
    fseek(f, 0, SEEK_END); long fsz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t h[12];
    if (fsz < 44 || fsz > 1500000000L || fread(h, 1, 12, f) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) {
        snprintf(err, elen, "pas un fichier WAV valide"); fclose(f); return NULL;
    }
    int fmt = 0, ch = 0, bits = 0, have_fmt = 0;
    uint32_t rate = 0, dsize = 0;
    long dpos = -1;
    for (;;) {
        uint8_t c[8];
        if (fread(c, 1, 8, f) != 8) break;
        uint32_t sz = rd32(c + 4);
        long pos = ftell(f);
        if (!memcmp(c, "fmt ", 4)) {
            uint8_t b[40];
            uint32_t n = sz < 40 ? sz : 40;
            if (n < 16 || fread(b, 1, n, f) != n) break;
            fmt = rd16(b); ch = rd16(b + 2); rate = rd32(b + 4); bits = rd16(b + 14);
            if (fmt == 0xFFFE && n >= 26) fmt = rd16(b + 24);
            have_fmt = 1;
        } else if (!memcmp(c, "data", 4)) { dpos = pos; dsize = sz; break; }
        if (sz > (uint32_t)fsz) break;
        int64_t next = (int64_t)pos + (int64_t)sz + (int64_t)(sz & 1);   /* 64 bits : pas de débordement sur ARM 32 bits */
        if (next > (int64_t)fsz || fseek(f, (long)next, SEEK_SET)) break;
    }
    if (!have_fmt || dpos < 0) { snprintf(err, elen, "WAV incomplet"); fclose(f); return NULL; }
    if (!((fmt == 1 && (bits == 8 || bits == 16 || bits == 24 || bits == 32)) || (fmt == 3 && bits == 32))) {
        snprintf(err, elen, "format non pris en charge (%d bits)", bits); fclose(f); return NULL;
    }
    if (ch < 1 || rate < 1000 || rate > 384000) { snprintf(err, elen, "WAV invalide"); fclose(f); return NULL; }
    long avail = fsz - dpos;
    if (dsize == 0xFFFFFFFFu || (long)dsize > avail) dsize = (uint32_t)avail;
    int bytes = bits / 8;
    uint32_t block = (uint32_t)ch * (uint32_t)bytes;
    uint32_t frames = dsize / block;
    int och = ch >= 2 ? 2 : 1;
    if (frames < 1) { snprintf(err, elen, "sample vide"); fclose(f); return NULL; }
    if ((uint64_t)frames * (uint64_t)och * 4u > MAX_BYTES) { snprintf(err, elen, "trop gros (plus de 96 Mo en memoire)"); fclose(f); return NULL; }
    float *d = malloc((size_t)frames * (size_t)och * sizeof(float));
    enum { CHF = 8192 };
    uint8_t *buf = malloc((size_t)CHF * block);
    if (!d || !buf) { free(d); free(buf); fclose(f); snprintf(err, elen, "memoire insuffisante"); return NULL; }
    fseek(f, dpos, SEEK_SET);
    uint32_t done = 0;
    while (done < frames) {
        uint32_t want = frames - done < CHF ? frames - done : CHF;
        size_t got = fread(buf, block, want, f);
        if (got == 0) break;
        for (size_t i = 0; i < got; i++)
            for (int c2 = 0; c2 < och; c2++)
                d[(size_t)(done + i) * och + c2] = conv(buf + i * block + (size_t)c2 * bytes, fmt, bits);
        done += (uint32_t)got;
    }
    free(buf); fclose(f);
    if (done < 1) { free(d); snprintf(err, elen, "sample vide"); return NULL; }
    smp_t *s = malloc(sizeof *s);
    if (!s) { free(d); snprintf(err, elen, "memoire insuffisante"); return NULL; }
    s->d = d; s->frames = (int)done; s->ch = och; s->rate = (int)rate;
    return s;
}

/* ---------- recherche des fichiers ---------- */
static int is_wav(const char *n) { size_t L = strlen(n); return L > 4 && !strcasecmp(n + L - 4, ".wav"); }
static int skip_dir(const char *n) {
    static const char *S[] = {"lost+found", "Expansions", "Demos", "Synths", "System Volume Information", "$RECYCLE.BIN", NULL};
    for (int i = 0; S[i]; i++) if (!strcmp(n, S[i])) return 1;
    return 0;
}
static void scan_dir(emx_t *e, const char *dir, int depth) {
    if (depth > MAX_DEPTH) return;
    DIR *d = opendir(dir);
    if (!d) return;
    struct dirent *de;
    while (!atomic_load(&e->quit) && (de = readdir(d))) {
        if (de->d_name[0] == '.') continue;
        char p[PLEN];
        if (snprintf(p, sizeof p, "%s/%s", dir, de->d_name) >= (int)sizeof p) continue;
        struct stat st;
        if (stat(p, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) { if (!skip_dir(de->d_name)) scan_dir(e, p, depth + 1); }
        else if (S_ISREG(st.st_mode) && is_wav(de->d_name)) {
            pthread_mutex_lock(&e->mx);
            if (e->nfiles < MAXF) {
                strcpy(e->paths[e->nfiles], p);
                e->nfiles++;
                if (e->nfiles % 10 == 0) snprintf(e->scaninfo, sizeof e->scaninfo, "Recherche des WAV... %d fichiers", e->nfiles);
            } else e->truncated = 1;
            pthread_mutex_unlock(&e->mx);
        }
    }
    closedir(d);
}
static void scan_all(emx_t *e) {
    DIR *d = opendir(MEDIA_ROOT);
    if (!d) return;
    struct dirent *de;
    while (!atomic_load(&e->quit) && (de = readdir(d))) {
        const char *n = de->d_name;
        if (n[0] == '.' || !strcmp(n, "az01-internal") || !strcmp(n, "EFI") || !strcmp(n, "secure-media")) continue;
        char p[PLEN];
        if (snprintf(p, sizeof p, "%s/%s", MEDIA_ROOT, n) >= (int)sizeof p) continue;
        struct stat st;
        if (lstat(p, &st) != 0 || !S_ISDIR(st.st_mode)) continue;   /* les liens (acvs-synths...) sont ignorés */
        scan_dir(e, p, 0);
    }
    closedir(d);
}
static int cmp_paths(const void *a, const void *b) { return strcasecmp((const char *)a, (const char *)b); }

/* "dossier/fichier.wav", tronqué par le début sans couper un caractère UTF-8 */
static void make_name(const char *path, char *out, int n) {
    const char *file = strrchr(path, '/');
    const char *start = file ? file + 1 : path;
    if (file) { const char *q = file; while (q > path && q[-1] != '/') q--; start = q; }
    size_t L = strlen(start);
    if ((int)L > n - 1) { start += L - (size_t)(n - 1); while (((unsigned char)*start & 0xC0) == 0x80) start++; }
    size_t m = strlen(start);
    if (m > (size_t)(n - 1)) m = (size_t)(n - 1);
    memcpy(out, start, m);
    out[m] = 0;
}


static void free_smp(smp_t *s) { if (s) { free(s->d); free(s); } }

static void *worker(void *arg) {
    emx_t *e = arg;
    scan_all(e);
    pthread_mutex_lock(&e->mx);
    qsort(e->paths, (size_t)e->nfiles, PLEN, cmp_paths);
    e->scanning = 0;
    for (;;) {
        int pi = -1;
        while (!atomic_load(&e->quit)) {
            for (int k = 0; k < NP; k++) if (e->p[k].req_id != e->p[k].done_id) { pi = k; break; }
            if (pi >= 0) break;
            pthread_cond_wait(&e->cv, &e->mx);
        }
        if (atomic_load(&e->quit)) break;
        part_t *pt = &e->p[pi];
        int id = pt->req_id, idx = pt->req, n = e->nfiles;
        if (idx < 0) {                                  /* numéro 0 : aucun sample */
            smp_t *old = atomic_exchange(&pt->cur, NULL);
            snprintf(pt->name, sizeof pt->name, "(aucun sample)");
            snprintf(pt->info, sizeof pt->info, "choisir un numero de sample");
            pt->done_id = id;
            if (old) {
                pthread_mutex_unlock(&e->mx);
                unsigned ep = atomic_load(&e->epoch);
                for (int k = 0; k < 200 && !atomic_load(&e->quit); k++) {
                    if ((unsigned)(atomic_load(&e->epoch) - ep) >= 2u) break;
                    usleep(10000);
                }
                free_smp(old);
                pthread_mutex_lock(&e->mx);
            }
            continue;
        }
        if (n == 0) { snprintf(pt->info, sizeof pt->info, "Aucun fichier WAV trouve sous %s", MEDIA_ROOT); pt->done_id = id; continue; }
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        char path[PLEN], nm[96];
        strcpy(path, e->paths[idx]);
        make_name(path, nm, 61);
        snprintf(pt->name, sizeof pt->name, "%s", nm);
        snprintf(pt->info, sizeof pt->info, "%d/%d   chargement...", idx + 1, n);
        pthread_mutex_unlock(&e->mx);
        char err[96] = "";
        smp_t *s = wav_load(path, err, sizeof err);
        pthread_mutex_lock(&e->mx);
        if (s) {
            smp_t *old = atomic_exchange(&pt->cur, s);
            snprintf(pt->info, sizeof pt->info, "%d/%d   %.1f kHz   %s   %.2f s%s", idx + 1, n, s->rate / 1000.0,
                     s->ch == 1 ? "mono" : "stereo", (double)s->frames / s->rate, e->truncated ? "   (liste limitee)" : "");
            pt->done_id = id;
            if (old) {
                pthread_mutex_unlock(&e->mx);
                unsigned ep = atomic_load(&e->epoch);   /* attendre que le fil audio ait quitté l'ancien sample */
                for (int k = 0; k < 200 && !atomic_load(&e->quit); k++) {
                    if ((unsigned)(atomic_load(&e->epoch) - ep) >= 2u) break;
                    usleep(10000);
                }
                free_smp(old);
                pthread_mutex_lock(&e->mx);
            }
        } else {
            snprintf(pt->info, sizeof pt->info, "%d/%d   erreur : %s", idx + 1, n, err);
            pt->done_id = id;
        }
    }
    pthread_mutex_unlock(&e->mx);
    return NULL;
}

static const char *PNAME[NP] = {"SYNTH 1", "SYNTH 2", "SYNTH 3", "SYNTH 4", "SYNTH 5", "DRUM 1", "DRUM 2", "DRUM 3", "DRUM 4", "DRUM 5", "DRUM 6A", "DRUM 6B", "DRUM 7A", "DRUM 7B"};
static const char *PSHORT[NP] = {"S1", "S2", "S3", "S4", "S5", "D1", "D2", "D3", "D4", "D5", "D6A", "D6B", "D7A", "D7B"};
static const char *SRC[] = {"Smp", "Saw", "Sqr", "Sin"};
static const char *FT[] = {"LP", "HP", "BP", "BP+"};
static const char *OT[] = {"Wave", "Dual", "Sync", "Ring", "Chord", "Cross", "Additive", "Noise", "Formant", "Unison", "VPM"};
static const char *LS[] = {"Sin", "Tri", "Saw", "Sqr"};
static const char *LD[] = {"Pitch", "Cutoff", "Amp", "Pan"};
static const char *FS[] = {"Off", "FX1", "FX2", "FX3"};
/* réglages de la partie sélectionnée, dans l'ordre des bits de e->chg */
#define NSK 64
static const char *SKEYS[NSK] = {"source", "sample", "root", "note", "level", "pan", "loop", "attack", "release", "ftype", "cutoff", "reso", "fenv", "fdecay",
    "otype", "oedit1", "oedit2", "lshape", "lspeed", "ldepth", "ldest", "fxsel", "fxsend",
    "t01", "t02", "t03", "t04", "t05", "t06", "t07", "t08", "t09", "t10", "t11", "t12", "t13", "t14", "t15", "t16",
    "arp", "arp_pos", "arp_gate", "arp_center", "arp_scale", "arp_type", "arp_rate",
    "pa01", "pa02", "pa03", "pa04", "pa05", "pa06", "pa07", "pa08", "pa09", "pa10", "pa11", "pa12", "pa13", "pa14",
    "fdrive", "chord", "chtype", "arp_spring"};
static const char *AT[] = {"Up", "Down", "UpDn", "Rand"};
static const char *AR[] = {"8th", "16th", "32nd"};
typedef struct { const char *name; int n; int8_t iv[4]; } chord_t;
static const chord_t CHORDS[8] = {{"Maj", 3, {0, 4, 7, 0}}, {"min", 3, {0, 3, 7, 0}}, {"Maj7", 4, {0, 4, 7, 11}}, {"min7", 4, {0, 3, 7, 10}},
                                  {"sus4", 3, {0, 5, 7, 0}}, {"dim", 3, {0, 3, 6, 0}}, {"aug", 3, {0, 4, 8, 0}}, {"5th", 3, {0, 7, 12, 0}}};
static float TSIN[4096], CH_R[8][3];
static void init_tables(void) {
    for (int i = 0; i < 4096; i++) TSIN[i] = sinf(6.2831853f * (float)i / 4096.0f);
    for (int c = 0; c < 8; c++) for (int k = 0; k < 3; k++) CH_R[c][k] = exp2f(CHORDS[c].iv[k] / 12.0f);
}
/* gammes de l'arpégiateur (liste ARPEGGIO SCALE de l'EMX-1, tonique do) */
typedef struct { const char *name; int n; uint8_t iv[12]; } scale_t;
static const scale_t SCALES[31] = {
    {"Chroma", 12, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}}, {"Ionian", 7, {0, 2, 4, 5, 7, 9, 11}}, {"Dorian", 7, {0, 2, 3, 5, 7, 9, 10}},
    {"Phrygi", 7, {0, 1, 3, 5, 7, 8, 10}}, {"Lydian", 7, {0, 2, 4, 6, 7, 9, 11}}, {"MixLyd", 7, {0, 2, 4, 5, 7, 9, 10}},
    {"Aeolia", 7, {0, 2, 3, 5, 7, 8, 10}}, {"Locria", 7, {0, 1, 3, 5, 6, 8, 10}}, {"MBlues", 6, {0, 3, 4, 7, 9, 10}},
    {"mBlues", 6, {0, 3, 5, 6, 7, 10}}, {"Dim", 8, {0, 2, 3, 5, 6, 8, 9, 11}}, {"ComDim", 8, {0, 1, 3, 4, 6, 7, 9, 10}},
    {"MPenta", 5, {0, 2, 4, 7, 9}}, {"mPenta", 5, {0, 3, 5, 7, 10}}, {"Raga1", 7, {0, 1, 4, 5, 7, 8, 11}},
    {"Raga2", 7, {0, 1, 4, 6, 7, 9, 11}}, {"Raga3", 7, {0, 1, 3, 6, 7, 8, 11}}, {"Spansh", 8, {0, 1, 3, 4, 5, 7, 8, 10}},
    {"Gypsy", 7, {0, 2, 3, 6, 7, 8, 11}}, {"Arabia", 7, {0, 2, 4, 5, 6, 8, 10}}, {"Egypt", 5, {0, 2, 5, 7, 10}},
    {"Hawaii", 5, {0, 2, 3, 7, 9}}, {"Pelog", 5, {0, 1, 3, 7, 8}}, {"Japan", 5, {0, 1, 5, 7, 8}}, {"Ryukyu", 5, {0, 4, 5, 7, 11}},
    {"Whole", 6, {0, 2, 4, 6, 8, 10}}, {"M3rd", 3, {0, 4, 8}}, {"m3rd", 4, {0, 3, 6, 9}}, {"4th", 3, {0, 5, 10}},
    {"5th", 2, {0, 7}}, {"Octave", 1, {0}}};

/* déplace une note de « deg » degrés dans la gamme (la note est d'abord ramenée à la note de la gamme la plus proche) */
static int scale_shift(int note, int sc, int deg) {
    const scale_t *s = &SCALES[sc < 0 ? 0 : (sc > 30 ? 30 : sc)];
    int oct = note / 12, pc = note % 12, idx = 0, best = 99;
    for (int k = 0; k < s->n; k++) { int d = abs(s->iv[k] - pc); if (d < best) { best = d; idx = k; } }
    if (12 - pc < best) { idx = 0; oct++; }
    int ni = idx + deg, wraps = ni >= 0 ? ni / s->n : -((-ni + s->n - 1) / s->n);
    int r = ni - wraps * s->n, out = (oct + wraps) * 12 + s->iv[r];
    return out < 0 ? 0 : (out > 127 ? 127 : out);
}
#define ALLCHG (~0ULL)

static void part_init(part_t *pt, int p) {
    memset(pt, 0, sizeof *pt);
    int syn = p < NSYN;
    pt->source = syn ? 1 : 0;
    pt->maxv = syn ? NVMAX : 2;
    pt->root = pt->note = 60;
    pt->level = 80.0f; pt->attack_ms = 2.0f; pt->release_ms = syn ? 200.0f : 80.0f;
    pt->cutoff = 100.0f; pt->fdecay_ms = 300.0f; pt->oedit2 = 50.0f; pt->lspeed = 20.0f;
    pt->pf = 1.0f; pt->am = 1.0f;
    pt->spring = 1; pt->fk = 0.2f; pt->agate = 60.0f; pt->acenter = 60; pt->arate = 1; pt->arnd = 12345u + (unsigned)p;
    atomic_init(&pt->cur, NULL);
    snprintf(pt->name, sizeof pt->name, "(aucun sample)");
    snprintf(pt->info, sizeof pt->info, "choisir un numero de sample (0 = aucun)");
}

static void *create(const char *d) {
    (void)d;
    emx_t *e = calloc(1, sizeof *e);
    if (!e) return NULL;
    init_tables();
    e->paths = calloc(MAXF, PLEN);
    for (int p = 0; p < NP; p++) part_init(&e->p[p], p);
    for (int u = 0; u < 3; u++) { e->fx[u] = fx_new(); e->fxe1[u] = e->fxe2[u] = 50.0f; e->fxcur[u] = -1; }
    e->tempo = 120.0f; e->master = 80.0f; e->fxret = 100.0f;
    e->scanning = 1;
    snprintf(e->scaninfo, sizeof e->scaninfo, "Recherche des WAV...");
    atomic_init(&e->epoch, 0); atomic_init(&e->quit, 0);
    pthread_mutex_init(&e->mx, NULL);
    pthread_cond_init(&e->cv, NULL);
    if (!e->paths || pthread_create(&e->th, NULL, worker, e) != 0) e->scanning = 0; else e->th_started = 1;
    return e;
}
static void destroy(void *i) {
    emx_t *e = i;
    if (!e) return;
    atomic_store(&e->quit, 1);
    pthread_mutex_lock(&e->mx); pthread_cond_broadcast(&e->cv); pthread_mutex_unlock(&e->mx);
    if (e->th_started) pthread_join(e->th, NULL);
    for (int p = 0; p < NP; p++) free_smp(atomic_load(&e->p[p].cur));
    for (int u = 0; u < 3; u++) fx_free(e->fx[u]);
    free(e->paths);
    pthread_mutex_destroy(&e->mx); pthread_cond_destroy(&e->cv);
    free(e);
}

/* correction PolyBLEP : adoucit les arêtes des formes d'onde pour limiter le repliement */
static float blep(float t, float dt) {
    if (t < dt) { t /= dt; return t + t - t * t - 1.0f; }
    if (t > 1.0f - dt) { t = (t - 1.0f) / dt; return t * t + t + t + 1.0f; }
    return 0.0f;
}
static float osc(int src, float p, float dt) {
    if (src == 3) return sinf(6.2831853f * p);
    if (src == 2) {
        float p2 = p + 0.5f; if (p2 >= 1.0f) p2 -= 1.0f;
        return (p < 0.5f ? 1.0f : -1.0f) + blep(p, dt) - blep(p2, dt);
    }
    return 2.0f * p - 1.0f - blep(p, dt);
}

/* filtre à variable d'état (TPT) par voix */
static inline float svf(voice_t *v, int c, float x, int ft, float k) {
    float v3 = x - v->ic2[c];
    float v1 = v->a1 * v->ic1[c] + v->a2 * v3;
    float v2 = v->ic2[c] + v->a2 * v->ic1[c] + v->a3 * v3;
    v->ic1[c] = 2.0f * v1 - v->ic1[c];
    v->ic2[c] = 2.0f * v2 - v->ic2[c];
    return ft == 1 ? x - k * v1 - v2 : (ft == 2 ? k * v1 : (ft == 3 ? 0.8f * v1 : v2));
}

static inline float wrap1(float p) { return p - floorf(p); }
static inline float fsin(float p) { return TSIN[(int)(wrap1(p) * 4096.0f) & 4095]; }
/* oscillateur d'une voix : 11 types (Wave, Dual, Sync, Ring, Chord, Cross, Additive, Noise, Formant, Unison, VPM) */
static inline float voice_osc(voice_t *v, const part_t *pt, float pf) {
    float dt = v->oinc * pf, x;
    float e1 = pt->oedit1 * 0.01f, e2 = pt->oedit2 * 0.01f, ratio = 1.0f + pt->oedit1 * 0.07f;
    switch (pt->otype) {
    case 1: {
        float det = 1.0f + e1 * 0.03f;
        x = osc(v->src, v->ph, dt) * (1.0f - e2) + osc(v->src, v->ph2, dt * det) * e2;
        v->ph2 += dt * det; while (v->ph2 >= 1.0f) v->ph2 -= 1.0f;
        break; }
    case 2:
        x = osc(v->src, v->ph2, dt * ratio);
        v->ph2 += dt * ratio; while (v->ph2 >= 1.0f) v->ph2 -= 1.0f;
        break;
    case 3:
        x = osc(v->src, v->ph, dt) * ((1.0f - e2) + e2 * osc(3, v->ph2, dt * ratio));
        v->ph2 += dt * ratio; while (v->ph2 >= 1.0f) v->ph2 -= 1.0f;
        break;
    case 4: {
        const float *r = CH_R[pt->chtype];
        x = (osc(v->src, v->ph, dt) + osc(v->src, v->ph2, dt * r[1]) + osc(v->src, v->ph3, dt * r[2])) * 0.45f;
        v->ph2 += dt * r[1]; while (v->ph2 >= 1.0f) v->ph2 -= 1.0f;
        v->ph3 += dt * r[2]; while (v->ph3 >= 1.0f) v->ph3 -= 1.0f;
        break; }
    case 5:
        x = osc(v->src, wrap1(v->ph + fsin(v->ph2) * e2 * 0.5f), dt);
        v->ph2 += dt * ratio; while (v->ph2 >= 1.0f) v->ph2 -= 1.0f;
        break;
    case 6: {
        int nh = 1 + (int)(e1 * 7.0f);
        float a = 1.0f, sum = 0.0f, norm = 0.0f, dec = 0.4f + e2 * 0.55f;
        for (int k = 1; k <= nh; k++) { sum += fsin(v->ph * (float)k) * a; norm += a; a *= dec; }
        x = sum / norm;
        break; }
    case 7: {
        v->rnd = v->rnd * 1664525u + 1013904223u;
        float n = (float)(v->rnd >> 8) * (1.0f / 8388608.0f) - 1.0f, al = 0.02f + e1 * e1 * 0.98f;
        v->nz += (n - v->nz) * al;
        x = v->nz * (1.0f + 3.0f * (1.0f - al)) * (1.0f - e2) + osc(v->src, v->ph, dt) * e2;
        break; }
    case 9: {
        float det = e1 * 0.02f;
        x = (osc(v->src, v->ph, dt) + osc(v->src, v->ph2, dt * (1.0f + det)) + osc(v->src, v->ph3, dt * (1.0f - det))) * 0.5f;
        v->ph2 += dt * (1.0f + det); while (v->ph2 >= 1.0f) v->ph2 -= 1.0f;
        v->ph3 += dt * (1.0f - det); while (v->ph3 >= 1.0f) v->ph3 -= 1.0f;
        break; }
    case 10:
        x = fsin(v->ph + e2 * 3.0f * fsin(v->ph2));
        v->ph2 += dt * ratio; while (v->ph2 >= 1.0f) v->ph2 -= 1.0f;
        break;
    default:
        x = osc(v->src, v->ph, dt);
    }
    v->ph += dt;
    if (v->ph >= 1.0f) { v->ph -= 1.0f; if (pt->otype == 2) v->ph2 = 0.0f; }
    return x;
}

/* filtre de formants (type Formant) : deux passe-bande, coefficients calculés par partie */
static inline float formant(voice_t *v, const part_t *pt, float x) {
    float out = 0.0f;
    for (int b = 0; b < 2; b++) {
        float v3 = x - v->fic2[b], v1 = pt->fa1[b] * v->fic1[b] + pt->fa2[b] * v3, v2 = v->fic2[b] + pt->fa2[b] * v->fic1[b] + pt->fa3[b] * v3;
        v->fic1[b] = 2.0f * v1 - v->fic1[b]; v->fic2[b] = 2.0f * v2 - v->fic2[b];
        out += pt->fk * v1;
    }
    return out * 1.5f;
}
static void formant_update(part_t *pt, float sr) {
    static const float FF[5][2] = {{800, 1150}, {400, 1600}, {350, 1700}, {450, 800}, {325, 700}};
    float pos = pt->oedit1 * 0.04f; int i = (int)pos; if (i > 3) i = 3;
    float fr = pos - (float)i;
    pt->fk = 0.4f - pt->oedit2 * 0.003f;
    for (int b = 0; b < 2; b++) {
        float hz = FF[i][b] * (1.0f - fr) + FF[i + 1][b] * fr, g = tanf(3.14159265f * hz / sr);
        pt->fa1[b] = 1.0f / (1.0f + g * (g + pt->fk)); pt->fa2[b] = g * pt->fa1[b]; pt->fa3[b] = g * pt->fa2[b];
    }
}

static void note_on(emx_t *e, int p, int note, int vel, int gate, int base) {
    part_t *pt = &e->p[p];
    smp_t *s = atomic_load(&pt->cur);
    if (pt->source == 0 && !s) return;
    voice_t *t = NULL;
    int k, mv = pt->maxv;
    for (k = 0; k < mv; k++) if (pt->v[k].stage && pt->v[k].note == note) { t = &pt->v[k]; break; }
    if (!t) for (k = 0; k < mv; k++) if (!pt->v[k].stage) { t = &pt->v[k]; break; }
    if (!t) {
        float best = 2.0f;
        for (k = 0; k < mv; k++) if (pt->v[k].stage == 3 && pt->v[k].env < best) { best = pt->v[k].env; t = &pt->v[k]; }
    }
    if (!t) {
        unsigned oldest = 0xFFFFFFFFu;
        for (k = 0; k < mv; k++) if (pt->v[k].age < oldest) { oldest = pt->v[k].age; t = &pt->v[k]; }
    }
    t->note = note; t->base = base;
    t->vel = vel / 127.0f;
    t->src = pt->source;
    t->ph3 = 0.0f; t->nz = 0.0f; t->fic1[0] = t->fic1[1] = t->fic2[0] = t->fic2[1] = 0.0f; t->rnd = (unsigned)(e->clock * 2654435761u + 12345u);
    if (t->src == 0) { t->inc = pow(2.0, (note - pt->root) / 12.0) * s->rate / SR; t->pos = 0.0; t->smp = s; }
    else { t->smp = NULL; t->oinc = 440.0f * powf(2.0f, (note - 69) / 12.0f) / (float)SR; t->ph = t->ph2 = 0.0f; }
    t->env = 0.0f; t->fenv = 1.0f; t->cnt = 0; t->gate = gate;
    t->ic1[0] = t->ic1[1] = t->ic2[0] = t->ic2[1] = 0.0f;
    t->stage = 1;
    t->age = ++e->clock;
}
static void note_off(emx_t *e, int p, int note) {
    part_t *pt = &e->p[p];
    for (int k = 0; k < NVMAX; k++)
        if (pt->v[k].stage && pt->v[k].stage != 3 && pt->v[k].base == note && pt->v[k].gate < 0) pt->v[k].stage = 3;
}
/* joue une note, ou un accord si CHORD est actif */
static void play(emx_t *e, int p, int note, int vel, int gate) {
    part_t *pt = &e->p[p];
    if (!pt->chord) { note_on(e, p, note, vel, gate, note); return; }
    const chord_t *c = &CHORDS[pt->chtype];
    for (int k = 0; k < c->n; k++) { int n2 = note + c->iv[k]; if (n2 <= 127) note_on(e, p, n2, vel, gate, note); }
}
static void all_off(emx_t *e) {
    for (int p = 0; p < NP; p++) for (int k = 0; k < NVMAX; k++) if (e->p[p].v[k].stage) e->p[p].v[k].stage = 3;
}
static void held_add(part_t *pt, int note) {
    for (int k = 0; k < pt->nh; k++) if (pt->held[k] == note) return;
    if (pt->nh < 8) { int k = pt->nh++; while (k > 0 && pt->held[k - 1] > note) { pt->held[k] = pt->held[k - 1]; k--; } pt->held[k] = (uint8_t)note; }
    memcpy(pt->memo, pt->held, sizeof pt->memo); pt->nm = pt->nh;      /* dernier accord tenu, rejoué par le ruban */
}
static void held_remove(part_t *pt, int note) {
    for (int k = 0; k < pt->nh; k++) if (pt->held[k] == note) { for (int j = k; j < pt->nh - 1; j++) pt->held[j] = pt->held[j + 1]; pt->nh--; break; }
}
static void midi(void *i, const uint8_t *m, int n) {
    emx_t *e = i;
    if (n < 2) return;
    int st = m[0] & 0xF0;
    part_t *pt = &e->p[e->sel];
    if (st == 0x90 && n >= 3 && m[2] > 0) {
        held_add(pt, m[1]);
        if (pt->arp) return;                             /* l'arpégiateur joue à la place du pad */
        play(e, e->sel, m[1], m[2], -1);
        if (e->rec && e->run) {                          /* enregistrement : le pas le plus proche */
            double steplen = SR * 60.0 / ((double)e->tempo * 4.0);
            int s = e->clk < steplen * 0.5 ? (e->step + 1) & 15 : e->step;
            pt->steps |= 1u << s; pt->snote[s] = m[1];
            e->chg |= 1ULL << (23 + s);
        }
    }
    else if (st == 0x80 || (st == 0x90 && n >= 3 && m[2] == 0)) {
        held_remove(pt, m[1]);
        if (!pt->arp) note_off(e, e->sel, m[1]);
    }
    else if (st == 0xE0 && n >= 3) {                     /* molette pitch : slider de l'arpégiateur, sinon inflexion de hauteur */
        float b = (float)(((m[2] << 7) | m[1]) - 8192) / 8192.0f;
        if (pt->arp || e->rib) { pt->apos = clampf(b * 100.0f, -100.0f, 100.0f); pt->alast = e->fcount; e->chg |= 1ULL << 40; }
        else e->bend = b;
    }
    else if (st == 0xB0 && n >= 3 && (m[1] == 123 || m[1] == 120)) { all_off(e); for (int p = 0; p < NP; p++) e->p[p].nh = 0; }
}

static int idx_of(const char *v, int max) { int x = atoi(v); return x < 0 ? 0 : (x > max ? max : x); }

static void request_sample(emx_t *e, part_t *pt, int idx) {
    pthread_mutex_lock(&e->mx);
    pt->req = idx; pt->req_id++; pt->want = idx >= 0;
    pthread_cond_signal(&e->cv);
    pthread_mutex_unlock(&e->mx);
}

static void transport(emx_t *e, int run) {
    if (run && !e->run) { e->stepn = 0; e->clk = 0.0; }
    if (!run && e->run) all_off(e);
    if (run != e->run) e->chg_run = 1;
    e->run = run;
}

static int state_parse(emx_t *e, const char *s);

/* Les listes d'options arrivent sous forme d'index ("0", "1"...), pas de noms. */
static void set_param(void *i, const char *k, const char *v) {
    emx_t *e = i;
    part_t *pt = &e->p[e->sel];
    float f = (float)atof(v);
    if (!strcmp(k, "state")) { state_parse(e, v); return; }
    if (!strcmp(k, "part")) { int x = idx_of(v, NP - 1); if (x != e->sel) { e->sel = x; e->chg = ALLCHG; } }
    else if (!strcmp(k, "run")) transport(e, v[0] == '1');
    else if (!strcmp(k, "play")) { if (f > 0.5f) transport(e, 1); }
    else if (!strcmp(k, "stop")) { if (f > 0.5f) transport(e, 0); }
    else if (!strcmp(k, "rec")) e->rec = v[0] == '1';
    else if (!strcmp(k, "tempo")) e->tempo = clampf(f, 60.0f, 200.0f);
    else if (!strcmp(k, "swing")) e->swing = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "master")) e->master = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "tube")) e->tube = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "fxchain")) e->fxchain = v[0] == '1';
    else if (!strcmp(k, "fxret")) e->fxret = clampf(f, 0.0f, 100.0f);
    else if (k[0] == 'f' && k[1] == 'x' && k[2] >= '1' && k[2] <= '3' && k[3]) {
        int u = k[2] - '1';
        if (!strcmp(k + 3, "on")) e->fxon[u] = v[0] == '1';
        else if (!strcmp(k + 3, "type")) e->fxtype[u] = idx_of(v, FXN - 1);
        else if (!strcmp(k + 3, "e1")) e->fxe1[u] = clampf(f, 0.0f, 100.0f);
        else if (!strcmp(k + 3, "e2")) e->fxe2[u] = clampf(f, 0.0f, 100.0f);
    }
    else if (!strcmp(k, "source")) pt->source = idx_of(v, 3);
    else if (!strcmp(k, "sample")) { int x = atoi(v); if (x < 0) x = 0; if (x != pt->sample) { pt->sample = x; request_sample(e, pt, x - 1); } }
    else if (!strcmp(k, "root")) pt->root = idx_of(v, 127);
    else if (!strcmp(k, "note")) pt->note = idx_of(v, 127);
    else if (!strcmp(k, "level")) pt->level = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "pan")) pt->pan = clampf(f, -100.0f, 100.0f);
    else if (!strcmp(k, "loop")) pt->loop = v[0] == '1';
    else if (!strcmp(k, "attack")) pt->attack_ms = clampf(f, 1.0f, 2000.0f);
    else if (!strcmp(k, "release")) pt->release_ms = clampf(f, 5.0f, 3000.0f);
    else if (!strcmp(k, "ftype")) pt->ftype = idx_of(v, 3);
    else if (!strcmp(k, "cutoff")) pt->cutoff = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "reso")) pt->reso = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "fenv")) pt->fenv_amt = clampf(f, -100.0f, 100.0f);
    else if (!strcmp(k, "fdecay")) pt->fdecay_ms = clampf(f, 10.0f, 3000.0f);
    else if (!strcmp(k, "otype")) pt->otype = idx_of(v, 10);
    else if (!strcmp(k, "oedit1")) pt->oedit1 = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "oedit2")) pt->oedit2 = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "lshape")) pt->lshape = idx_of(v, 3);
    else if (!strcmp(k, "lspeed")) pt->lspeed = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "ldepth")) pt->ldepth = clampf(f, -100.0f, 100.0f);
    else if (!strcmp(k, "ldest")) pt->ldest = idx_of(v, 3);
    else if (!strcmp(k, "fxsel")) pt->fxsel = idx_of(v, 3);
    else if (!strcmp(k, "fxsend")) pt->fxsend = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "arp")) { int on = v[0] == '1'; if (on != pt->arp) { pt->arp = on; pt->aclk = 0.0; pt->aidx = 0; } }
    else if (!strcmp(k, "arp_pos")) { pt->apos = clampf(f, -100.0f, 100.0f); pt->alast = e->fcount; }
    else if (!strcmp(k, "arp_gate")) pt->agate = clampf(f, 5.0f, 100.0f);
    else if (!strcmp(k, "arp_center")) pt->acenter = idx_of(v, 127);
    else if (!strcmp(k, "arp_scale")) pt->ascale = idx_of(v, 30);
    else if (!strcmp(k, "arp_type")) pt->atype = idx_of(v, 3);
    else if (!strcmp(k, "arp_rate")) pt->arate = idx_of(v, 2);
    else if (!strcmp(k, "rib")) e->rib = f > 0.5f ? 8 : 0;
    else if (k[0] == 'r' && k[1] == 'i' && k[2] == 'b' && k[3] >= '1' && k[3] <= '8' && !k[4]) { int sg = k[3] - '0'; if (f > 0.5f) e->rib = sg; else if (e->rib == sg) e->rib = 0; }
    else if (!strcmp(k, "fdrive")) pt->fdrive = clampf(f, 0.0f, 100.0f);
    else if (!strcmp(k, "chord")) pt->chord = v[0] == '1';
    else if (!strcmp(k, "chtype")) pt->chtype = idx_of(v, 7);
    else if (!strcmp(k, "arp_spring")) pt->spring = v[0] == '1';
    else if (k[0] == 'p' && k[1] == 'a' && isdigit((unsigned char)k[2]) && isdigit((unsigned char)k[3])) {
        int n = atoi(k + 2) - 1;
        if (n >= 0 && n < NP) {
            if (v[0] == '1') { if (n != e->sel) { e->sel = n; e->chg = ALLCHG; } }
            else if (n == e->sel) e->chg |= 1ULL << (46 + n);     /* le pad choisi reste allumé */
        }
    }
    else if (k[0] == 't' && isdigit((unsigned char)k[1]) && isdigit((unsigned char)k[2])) {
        int n = atoi(k + 1) - 1;
        if (n >= 0 && n < 16) {
            if (v[0] == '1') { pt->steps |= 1u << n; if (!pt->snote[n]) pt->snote[n] = (unsigned char)pt->note; }
            else { pt->steps &= ~(1u << n); pt->snote[n] = 0; }
        }
    }
}

static int wnum(char *b, int n, int *o, double x) { if (*o >= n) return 0; *o += snprintf(b + *o, (size_t)(n - *o), "%g;", x); return 1; }
static int state_write(emx_t *e, char *b, int n) {
    int o = snprintf(b, (size_t)n, "EMX1v4;");
    wnum(b, n, &o, e->sel); wnum(b, n, &o, e->tempo); wnum(b, n, &o, e->swing); wnum(b, n, &o, e->master); wnum(b, n, &o, e->tube);
    wnum(b, n, &o, e->fxchain); wnum(b, n, &o, e->fxret);
    for (int u = 0; u < 3; u++) { wnum(b, n, &o, e->fxon[u]); wnum(b, n, &o, e->fxtype[u]); wnum(b, n, &o, e->fxe1[u]); wnum(b, n, &o, e->fxe2[u]); }
    for (int p = 0; p < NP; p++) {
        part_t *q = &e->p[p];
        double v[24] = {q->source, q->sample, q->root, q->note, q->loop, q->ftype, q->otype, q->ldest, q->lshape, q->fxsel, q->level, q->pan,
                        q->attack_ms, q->release_ms, q->cutoff, q->reso, q->fenv_amt, q->fdecay_ms, q->oedit1, q->oedit2, q->lspeed, q->ldepth, q->fxsend, q->steps};
        for (int k = 0; k < 24; k++) wnum(b, n, &o, v[k]);
        for (int k = 0; k < 16; k++) wnum(b, n, &o, q->snote[k]);
        { double a[11] = {q->arp, q->apos, q->agate, q->acenter, q->ascale, q->atype, q->arate, q->fdrive, q->chord, q->chtype, q->spring}; for (int k = 0; k < 11; k++) wnum(b, n, &o, a[k]); }
    }
    return o;
}
static int rnum(const char **c, double *v) { char *end; *v = strtod(*c, &end); if (end == *c) return 0; *c = end; if (**c == ';') (*c)++; return 1; }
static int state_parse(emx_t *e, const char *s) {
    if (strncmp(s, "EMX1v4;", 7)) return 0;
    const char *c = s + 7;
    double g[7];
    for (int k = 0; k < 7; k++) if (!rnum(&c, &g[k])) return 0;
    double fxv[12];
    for (int k = 0; k < 12; k++) if (!rnum(&c, &fxv[k])) return 0;
    e->sel = (int)clampf((float)g[0], 0, NP - 1); e->tempo = clampf((float)g[1], 60, 200); e->swing = clampf((float)g[2], 0, 100);
    e->master = clampf((float)g[3], 0, 100); e->tube = clampf((float)g[4], 0, 100); e->fxchain = g[5] != 0; e->fxret = clampf((float)g[6], 0, 100);
    for (int u = 0; u < 3; u++) { e->fxon[u] = fxv[u * 4] != 0; e->fxtype[u] = (int)clampf((float)fxv[u * 4 + 1], 0, FXN - 1); e->fxe1[u] = clampf((float)fxv[u * 4 + 2], 0, 100); e->fxe2[u] = clampf((float)fxv[u * 4 + 3], 0, 100); }
    for (int p = 0; p < NP; p++) {
        double v[24], sn[16];
        for (int k = 0; k < 24; k++) if (!rnum(&c, &v[k])) return 0;
        for (int k = 0; k < 16; k++) if (!rnum(&c, &sn[k])) return 0;
        double ar[11];
        for (int k = 0; k < 11; k++) if (!rnum(&c, &ar[k])) return 0;
        part_t *q = &e->p[p];
        q->source = (int)clampf((float)v[0], 0, 3); q->root = (int)clampf((float)v[2], 0, 127); q->note = (int)clampf((float)v[3], 0, 127);
        q->loop = v[4] != 0; q->ftype = (int)clampf((float)v[5], 0, 3); q->otype = (int)clampf((float)v[6], 0, 10); q->ldest = (int)clampf((float)v[7], 0, 3);
        q->lshape = (int)clampf((float)v[8], 0, 3); q->fxsel = (int)clampf((float)v[9], 0, 3);
        q->level = clampf((float)v[10], 0, 100); q->pan = clampf((float)v[11], -100, 100); q->attack_ms = clampf((float)v[12], 1, 2000);
        q->release_ms = clampf((float)v[13], 5, 3000); q->cutoff = clampf((float)v[14], 0, 100); q->reso = clampf((float)v[15], 0, 100);
        q->fenv_amt = clampf((float)v[16], -100, 100); q->fdecay_ms = clampf((float)v[17], 10, 3000); q->oedit1 = clampf((float)v[18], 0, 100);
        q->oedit2 = clampf((float)v[19], 0, 100); q->lspeed = clampf((float)v[20], 0, 100); q->ldepth = clampf((float)v[21], -100, 100);
        q->fxsend = clampf((float)v[22], 0, 100); q->steps = (unsigned)v[23] & 0xFFFFu;
        for (int k = 0; k < 16; k++) q->snote[k] = (unsigned char)((int)sn[k] & 127);
        q->sample = v[1] < 0 ? 0 : (int)v[1];
        q->arp = ar[0] != 0; q->apos = clampf((float)ar[1], -100, 100); q->agate = clampf((float)ar[2], 5, 100); q->acenter = (int)clampf((float)ar[3], 0, 127);
        q->ascale = (int)clampf((float)ar[4], 0, 30); q->atype = (int)clampf((float)ar[5], 0, 3); q->arate = (int)clampf((float)ar[6], 0, 2);
        q->fdrive = clampf((float)ar[7], 0, 100); q->chord = ar[8] != 0; q->chtype = (int)clampf((float)ar[9], 0, 7); q->spring = ar[10] != 0;
        if (q->sample > 0 && v[1] != 0) request_sample(e, q, q->sample - 1);
    }
    e->chg = ALLCHG;
    return 1;
}

static int get_param(void *i, const char *k, char *b, int n) {
    emx_t *e = i;
    part_t *pt = &e->p[e->sel];
    if (!strcmp(k, "state")) { char tmp[4096]; int o = state_write(e, tmp, (int)sizeof tmp); if (o <= 0 || o >= (int)sizeof tmp - 8 || o + 1 > n) return 0; memcpy(b, tmp, (size_t)o + 1); return o; }
    if (!strcmp(k, "part")) return snprintf(b, n, "%s", PSHORT[e->sel]);
    if (!strcmp(k, "run")) return snprintf(b, n, "%s", e->run ? "on" : "off");
    if (!strcmp(k, "rec")) return snprintf(b, n, "%s", e->rec ? "on" : "off");
    if (!strcmp(k, "play") || !strcmp(k, "stop")) return snprintf(b, n, "0");
    if (!strcmp(k, "step")) return snprintf(b, n, "s%02d", e->step + 1);
    if (!strcmp(k, "tempo")) return snprintf(b, n, "%.0f", e->tempo);
    if (!strcmp(k, "swing")) return snprintf(b, n, "%.0f", e->swing);
    if (!strcmp(k, "master")) return snprintf(b, n, "%.0f", e->master);
    if (!strcmp(k, "tube")) return snprintf(b, n, "%.0f", e->tube);
    if (!strcmp(k, "fxchain")) return snprintf(b, n, "%s", e->fxchain ? "on" : "off");
    if (!strcmp(k, "fxret")) return snprintf(b, n, "%.0f", e->fxret);
    if (k[0] == 'f' && k[1] == 'x' && k[2] >= '1' && k[2] <= '3' && k[3]) {
        int u = k[2] - '1';
        if (!strcmp(k + 3, "on")) return snprintf(b, n, "%s", e->fxon[u] ? "on" : "off");
        if (!strcmp(k + 3, "type")) return snprintf(b, n, "%s", FX_NAMES[e->fxtype[u]]);
        if (!strcmp(k + 3, "e1")) return snprintf(b, n, "%.0f", e->fxe1[u]);
        if (!strcmp(k + 3, "e2")) return snprintf(b, n, "%.0f", e->fxe2[u]);
    }
    if (!strcmp(k, "part_name")) return snprintf(b, n, "%s", PNAME[e->sel]);
    if (!strcmp(k, "source")) return snprintf(b, n, "%s", SRC[pt->source]);
    if (!strcmp(k, "sample")) return snprintf(b, n, "%d", pt->sample);
    if (!strcmp(k, "root")) return snprintf(b, n, "%d", pt->root);
    if (!strcmp(k, "note")) return snprintf(b, n, "%d", pt->note);
    if (!strcmp(k, "level")) return snprintf(b, n, "%.0f", pt->level);
    if (!strcmp(k, "pan")) return snprintf(b, n, "%.0f", pt->pan);
    if (!strcmp(k, "loop")) return snprintf(b, n, "%s", pt->loop ? "on" : "off");
    if (!strcmp(k, "attack")) return snprintf(b, n, "%.0f", pt->attack_ms);
    if (!strcmp(k, "release")) return snprintf(b, n, "%.0f", pt->release_ms);
    if (!strcmp(k, "ftype")) return snprintf(b, n, "%s", FT[pt->ftype]);
    if (!strcmp(k, "cutoff")) return snprintf(b, n, "%.0f", pt->cutoff);
    if (!strcmp(k, "reso")) return snprintf(b, n, "%.0f", pt->reso);
    if (!strcmp(k, "fenv")) return snprintf(b, n, "%.0f", pt->fenv_amt);
    if (!strcmp(k, "fdecay")) return snprintf(b, n, "%.0f", pt->fdecay_ms);
    if (!strcmp(k, "otype")) return snprintf(b, n, "%s", OT[pt->otype]);
    if (!strcmp(k, "oedit1")) return snprintf(b, n, "%.0f", pt->oedit1);
    if (!strcmp(k, "oedit2")) return snprintf(b, n, "%.0f", pt->oedit2);
    if (!strcmp(k, "lshape")) return snprintf(b, n, "%s", LS[pt->lshape]);
    if (!strcmp(k, "lspeed")) return snprintf(b, n, "%.0f", pt->lspeed);
    if (!strcmp(k, "ldepth")) return snprintf(b, n, "%.0f", pt->ldepth);
    if (!strcmp(k, "ldest")) return snprintf(b, n, "%s", LD[pt->ldest]);
    if (!strcmp(k, "fxsel")) return snprintf(b, n, "%s", FS[pt->fxsel]);
    if (!strcmp(k, "fxsend")) return snprintf(b, n, "%.0f", pt->fxsend);
    if (!strcmp(k, "arp")) return snprintf(b, n, "%s", pt->arp ? "on" : "off");
    if (!strcmp(k, "arp_pos")) return snprintf(b, n, "%.0f", pt->apos);
    if (!strcmp(k, "arp_gate")) return snprintf(b, n, "%.0f", pt->agate);
    if (!strcmp(k, "arp_center")) return snprintf(b, n, "%d", pt->acenter);
    if (!strcmp(k, "arp_scale")) return snprintf(b, n, "%s", SCALES[pt->ascale].name);
    if (!strcmp(k, "arp_type")) return snprintf(b, n, "%s", AT[pt->atype]);
    if (!strcmp(k, "arp_rate")) return snprintf(b, n, "%s", AR[pt->arate]);
    if (!strcmp(k, "rib") || (k[0] == 'r' && k[1] == 'i' && k[2] == 'b' && k[3] >= '1' && k[3] <= '8' && !k[4])) return snprintf(b, n, "0");
    if (!strcmp(k, "fdrive")) return snprintf(b, n, "%.0f", pt->fdrive);
    if (!strcmp(k, "chord")) return snprintf(b, n, "%s", pt->chord ? "on" : "off");
    if (!strcmp(k, "chtype")) return snprintf(b, n, "%s", CHORDS[pt->chtype].name);
    if (!strcmp(k, "arp_spring")) return snprintf(b, n, "%s", pt->spring ? "on" : "off");
    if (k[0] == 'p' && k[1] == 'a' && isdigit((unsigned char)k[2]) && isdigit((unsigned char)k[3])) return snprintf(b, n, "%s", atoi(k + 2) - 1 == e->sel ? "on" : "off");
    if (k[0] == 't' && isdigit((unsigned char)k[1]) && isdigit((unsigned char)k[2])) {
        int s = atoi(k + 1) - 1;
        if (s >= 0 && s < 16) return snprintf(b, n, "%s", (pt->steps >> s) & 1u ? "on" : "off");
    }
    if (!strcmp(k, "sample_name") || !strcmp(k, "sample_info")) {
        pthread_mutex_lock(&e->mx);
        int r;
        if (k[7] == 'n') r = snprintf(b, n, "%s", e->scanning ? "..." : pt->name);
        else r = snprintf(b, n, "%s", e->scanning ? e->scaninfo : pt->info);
        pthread_mutex_unlock(&e->mx);
        return r;
    }
    return 0;
}

int mpc_engine_changed(void *inst, char *key, int len) {
    emx_t *e = inst;
    if (e->chg_step) { e->chg_step = 0; snprintf(key, (size_t)len, "step"); return 1; }
    if (e->chg_run) { e->chg_run = 0; snprintf(key, (size_t)len, "run"); return 1; }
    if (e->chg) { int b = __builtin_ctzll(e->chg); e->chg &= ~(1ULL << b); snprintf(key, (size_t)len, "%s", SKEYS[b]); return 1; }
    return 0;
}

static void fire_step(emx_t *e, double steplen) {
    int s = e->stepn;
    for (int p = 0; p < NP; p++) {
        part_t *pt = &e->p[p];
        if (!((pt->steps >> s) & 1u)) continue;
        int gate = (pt->source != 0 || pt->loop) ? (int)(steplen * 0.9) : -1;
        play(e, p, pt->snote[s] ? pt->snote[s] : pt->note, 100, gate);
    }
    e->step = s; e->chg_step = 1;
    e->stepn = (s + 1) & 15;
}

/* un pas de l'arpégiateur de la partie p : alen = durée du pas en échantillons */
static void arp_step(emx_t *e, int p, double alen) {
    part_t *pt = &e->p[p];
    uint8_t set[8];
    int n = 0;
    if (pt->arp && pt->nh > 0) { memcpy(set, pt->held, (size_t)pt->nh); n = pt->nh; }
    else if (e->rib && p == e->sel) {
        if (pt->nh > 0) { memcpy(set, pt->held, (size_t)pt->nh); n = pt->nh; }
        else if (pt->nm > 0) { memcpy(set, pt->memo, (size_t)pt->nm); n = pt->nm; }
        else { set[0] = (uint8_t)pt->note; n = 1; }
    }
    if (n == 0) return;
    int i;
    switch (pt->atype) {
    case 1: i = n - 1 - (pt->aidx % n); break;
    case 2: { int cyc = n > 1 ? 2 * n - 2 : 1, k = pt->aidx % cyc; i = k < n ? k : cyc - k; break; }
    case 3: pt->arnd = pt->arnd * 1664525u + 1013904223u; i = (int)((pt->arnd >> 16) % (unsigned)n); break;
    default: i = pt->aidx % n;
    }
    pt->aidx++;
    int note = scale_shift(set[i] + (pt->acenter - 60), pt->ascale, (int)lroundf(pt->apos * 0.12f));
    float gp = (e->rib > 0 && p == e->sel) ? 10.0f + (float)(e->rib - 1) * 12.857f : pt->agate;   /* ruban : gauche = haché, droite = long */
    int gate = (int)(alen * gp * 0.01);
    play(e, p, note, 100, gate > 1 ? gate : 1);
}

static void render(void *inst, int16_t *o, int f) {
    emx_t *e = inst;
    const float srf = (float)SR;
    float att[NP], rel[NP], fdec[NP], gl[NP], gr[NP], bo[NP], mo[NP], qk[NP], dg[NP], dn[NP];
    smp_t *cs[NP];
    for (int p = 0; p < NP; p++) {
        part_t *pt = &e->p[p];
        cs[p] = atomic_load(&pt->cur);
        att[p] = 1.0f / (pt->attack_ms * 0.001f * srf);
        rel[p] = 1.0f / (pt->release_ms * 0.001f * srf);
        fdec[p] = 1.0f / (pt->fdecay_ms * 0.001f * srf);
        float x = (pt->pan * 0.01f + 1.0f) * 0.78539816f, lv = pt->level * 0.01f * 0.5f * 1.41421356f;
        gl[p] = lv * cosf(x); gr[p] = lv * sinf(x);
        bo[p] = pt->cutoff * 0.1f; mo[p] = pt->fenv_amt * 0.01f * 8.0f;
        qk[p] = 2.0f - 1.9f * (pt->reso * 0.01f);
        dg[p] = 1.0f + pt->fdrive * 0.08f; dn[p] = 1.0f / tanhf(dg[p]);
        for (int k = 0; k < NVMAX; k++) if (pt->v[k].stage && pt->v[k].src == 0 && pt->v[k].smp != cs[p]) pt->v[k].stage = 0;
    }
    for (int u = 0; u < 3; u++) {                        /* réglages des 3 processeurs d'effets */
        fx_t *fx = e->fx[u];
        if (!fx) continue;
        if (e->fxcur[u] != e->fxtype[u]) { fx_reset(fx); e->fxcur[u] = e->fxtype[u]; }
        fx->type = e->fxtype[u]; fx->e1 = e->fxe1[u] * 0.01f; fx->e2 = e->fxe2[u] * 0.01f; fx->bpm = e->tempo;
    }
    double steplen = SR * 60.0 / ((double)e->tempo * 4.0);
    double sw = e->swing * 0.01 * 0.33;
    float mg = e->master * 0.01f, fret = e->fxret * 0.01f, bendf = exp2f(e->bend * (2.0f / 12.0f));
    float t = e->tube * 0.01f, tg = 1.0f + e->tube * 0.06f, tn = 1.0f / tanhf(tg), tc = 1.0f / (1.0f + 0.8f * t);
    for (int n = 0; n < f; n++) {
        e->fcount++;
        if (e->run) {
            e->clk -= 1.0;
            if (e->clk <= 0.0) {
                int cur = e->stepn;
                fire_step(e, steplen);
                e->clk += steplen * ((cur & 1) ? (1.0 - sw) : (1.0 + sw));
            }
        }
        for (int p = 0; p < NP; p++) {                   /* arpégiateurs */
            part_t *pa = &e->p[p];
            int act = pa->arp ? pa->nh > 0 : (e->rib && p == e->sel);
            if (!act) { pa->aclk = 0.0; pa->aidx = 0; continue; }
            pa->aclk -= 1.0;
            if (pa->aclk <= 0.0) { double alen = steplen * (pa->arate == 0 ? 2.0 : (pa->arate == 2 ? 0.5 : 1.0)); arp_step(e, p, alen); pa->aclk += alen; }
        }
        if ((e->ctl++ & 15u) == 0) {                     /* LFO de chaque partie, 1 échantillon sur 16 */
            for (int p = 0; p < NP; p++) {
                part_t *pt = &e->p[p];
                if (pt->otype == 8) formant_update(pt, srf);
                if (pt->spring && pt->apos != 0.0f && e->fcount - pt->alast > 22050 && !(e->rib && p == e->sel)) {   /* le slider revient au centre */
                    pt->apos *= 0.8f; if (fabsf(pt->apos) < 1.0f) pt->apos = 0.0f;
                    if (p == e->sel) e->chg |= 1ULL << 40;
                }
                if (pt->ldepth == 0.0f) { pt->pf = 1.0f; pt->cm = 0.0f; pt->am = 1.0f; pt->panm = 0.0f; continue; }
                float hz = 0.05f + pt->lspeed * pt->lspeed * 0.0025f;       /* 0.05 .. 25 Hz */
                pt->lph += 16.0f * hz / srf; if (pt->lph >= 1.0f) pt->lph -= 1.0f;
                float ph = pt->lph, lv;
                switch (pt->lshape) { case 1: lv = ph < 0.5f ? 4.0f * ph - 1.0f : 3.0f - 4.0f * ph; break; case 2: lv = 2.0f * ph - 1.0f; break; case 3: lv = ph < 0.5f ? 1.0f : -1.0f; break; default: lv = sinf(6.2831853f * ph); }
                float d = pt->ldepth * 0.01f * lv;
                pt->pf = pt->ldest == 0 ? exp2f(d) : 1.0f;
                pt->cm = pt->ldest == 1 ? d * 5.0f : 0.0f;
                pt->am = pt->ldest == 2 ? 1.0f - fabsf(pt->ldepth * 0.01f) * 0.5f * (1.0f - lv) : 1.0f;
                pt->panm = pt->ldest == 3 ? d * 100.0f : 0.0f;
                if (pt->ldest == 3) {
                    float x = (clampf(pt->pan + pt->panm, -100.0f, 100.0f) * 0.01f + 1.0f) * 0.78539816f, lvl = pt->level * 0.01f * 0.5f * 1.41421356f;
                    gl[p] = lvl * cosf(x); gr[p] = lvl * sinf(x);
                }
            }
        }
        float L = 0.0f, R = 0.0f, bus[3][2] = {{0, 0}, {0, 0}, {0, 0}};
        for (int p = 0; p < NP; p++) {
            part_t *pt = &e->p[p];
            float pl = 0.0f, pr = 0.0f, pf = pt->pf * ((pt->arp || (e->rib && p == e->sel)) ? 1.0f : bendf);
            int ft = pt->ftype;
            for (int k = 0; k < pt->maxv; k++) {
                voice_t *v = &pt->v[k];
                if (!v->stage) continue;
                if (v->gate > 0 && --v->gate == 0 && v->stage != 3) v->stage = 3;
                if (v->stage == 1) { v->env += att[p]; if (v->env >= 1.0f) { v->env = 1.0f; v->stage = 2; } }
                else if (v->stage == 3) { v->env -= rel[p]; if (v->env <= 0.0f) { v->env = 0.0f; v->stage = 0; continue; } }
                float x0, x1;
                if (v->src == 0) {
                    const smp_t *m = v->smp;
                    int i0 = (int)v->pos, i1 = i0 + 1;
                    float fr = (float)(v->pos - i0);
                    if (i1 >= m->frames) i1 = pt->loop ? 0 : m->frames - 1;
                    if (m->ch == 1) { x0 = x1 = m->d[i0] + (m->d[i1] - m->d[i0]) * fr; }
                    else {
                        x0 = m->d[2 * i0] + (m->d[2 * i1] - m->d[2 * i0]) * fr;
                        x1 = m->d[2 * i0 + 1] + (m->d[2 * i1 + 1] - m->d[2 * i0 + 1]) * fr;
                    }
                    v->pos += v->inc * pf;
                    if (v->pos >= m->frames) {
                        if (pt->loop) { do v->pos -= m->frames; while (v->pos >= m->frames); }
                        else v->stage = 0;
                    }
                } else { x0 = voice_osc(v, pt, pf); if (pt->otype == 8) x0 = formant(v, pt, x0); x1 = x0; }
                if ((v->cnt++ & 15u) == 0) {
                    float hz = 20.0f * exp2f(bo[p] + pt->cm + mo[p] * v->fenv);
                    if (hz > 18000.0f) hz = 18000.0f;
                    if (hz < 20.0f) hz = 20.0f;
                    float g = tanf(3.14159265f * hz / srf);
                    v->a1 = 1.0f / (1.0f + g * (g + qk[p])); v->a2 = g * v->a1; v->a3 = g * v->a2;
                }
                v->fenv -= fdec[p]; if (v->fenv < 0.0f) v->fenv = 0.0f;
                if (dg[p] > 1.0f) { x0 = tanhf(x0 * dg[p]) * dn[p]; x1 = tanhf(x1 * dg[p]) * dn[p]; }
                float a = v->env * v->vel * pt->am;
                pl += svf(v, 0, x0, ft, qk[p]) * a;
                pr += svf(v, 1, x1, ft, qk[p]) * a;
            }
            pl *= gl[p]; pr *= gr[p];
            L += pl; R += pr;
            if (pt->fxsel > 0 && pt->fxsend > 0.0f) { float s = pt->fxsend * 0.01f; bus[pt->fxsel - 1][0] += pl * s; bus[pt->fxsel - 1][1] += pr * s; }
        }
        float pvl = 0.0f, pvr = 0.0f;
        for (int u = 0; u < 3; u++) {                    /* effets : FX1 -> FX2 -> FX3 en chaîne, ou en parallèle */
            if (!e->fxon[u] || !e->fx[u]) { pvl = pvr = 0.0f; continue; }
            float il = bus[u][0] + (e->fxchain ? pvl : 0.0f), ir = bus[u][1] + (e->fxchain ? pvr : 0.0f);
            fx_run(e->fx[u], &il, &ir, srf);
            L += il * fret; R += ir * fret;
            pvl = il; pvr = ir;
        }
        if (t > 0.0f) {                                  /* saturation « Valve » avec compensation de volume */
            L = (L + t * (tanhf(tg * L) * tn - L)) * tc;
            R = (R + t * (tanhf(tg * R) * tn - R)) * tc;
        }
        o[2 * n] = (int16_t)(clampf(L * mg, -1.0f, 1.0f) * 32767.0f);
        o[2 * n + 1] = (int16_t)(clampf(R * mg, -1.0f, 1.0f) * 32767.0f);
    }
    atomic_fetch_add(&e->epoch, 1u);
}

static const mpc_engine_t ENGINE = { create, destroy, midi, set_param, get_param, render, NULL, NULL };
const mpc_engine_t *mpc_engine(void) { return &ENGINE; }
