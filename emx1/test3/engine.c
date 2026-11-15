/* EMX-1 Test 3 : lecteur de samples (WAV de la MPC), 8 voix, chargement asynchrone.
 * - un fil de travail cherche les WAV sous /media puis charge le sample demandé en mémoire ;
 * - le fil audio ne lit jamais de fichier : il joue le sample déjà en mémoire ;
 * - les fichiers de l'utilisateur ne sont ouverts qu'en lecture seule. */
#define _GNU_SOURCE
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

#ifndef MEDIA_ROOT
#define MEDIA_ROOT "/media"
#endif
#define NV 8
#define SR 44100.0
#define MAXF 2000                       /* nombre maximum de fichiers listés */
#define PLEN 512                        /* longueur maximum d'un chemin */
#define MAX_DEPTH 6                     /* profondeur de recherche dans les dossiers */
#define MAX_BYTES (96u * 1024u * 1024u) /* plafond mémoire pour un sample */

typedef struct { float *d; int frames, ch, rate; } smp_t;

typedef struct {
    int stage;                          /* 0 libre, 1 attaque, 2 tenue, 3 relâchement */
    int note;
    float vel, env;
    double pos, inc;
    smp_t *smp;
    unsigned age;
} voice_t;

typedef struct {
    voice_t v[NV];
    float level, attack_ms, release_ms;
    int root, loop;
    unsigned clock;
    _Atomic(smp_t *) cur;               /* sample en cours, lu par le fil audio */
    atomic_uint epoch;                  /* compte les appels à render() */
    atomic_int quit;
    pthread_t th;
    int th_started;
    pthread_mutex_t mx;                 /* protège ce qui suit, jamais pris par render() */
    pthread_cond_t cv;
    int req, req_id, done_id;
    char (*paths)[PLEN];
    int nfiles, scanning, truncated;
    char name[96], info[160];
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
                if (e->nfiles % 10 == 0) snprintf(e->info, sizeof e->info, "Recherche des WAV... %d fichiers", e->nfiles);
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

static void *worker(void *arg) {
    emx_t *e = arg;
    scan_all(e);
    pthread_mutex_lock(&e->mx);
    qsort(e->paths, (size_t)e->nfiles, PLEN, cmp_paths);
    e->scanning = 0;
    if (e->nfiles == 0) snprintf(e->info, sizeof e->info, "Aucun fichier WAV trouve sous %s", MEDIA_ROOT);
    for (;;) {
        while (!atomic_load(&e->quit) && e->req_id == e->done_id) pthread_cond_wait(&e->cv, &e->mx);
        if (atomic_load(&e->quit)) break;
        int id = e->req_id, idx = e->req, n = e->nfiles;
        if (n == 0) { e->done_id = id; continue; }
        if (idx < 0) idx = 0;
        if (idx >= n) idx = n - 1;
        char path[PLEN], nm[96];
        strcpy(path, e->paths[idx]);
        make_name(path, nm, 61);
        snprintf(e->name, sizeof e->name, "%s", nm);
        snprintf(e->info, sizeof e->info, "%d/%d   chargement...", idx + 1, n);
        pthread_mutex_unlock(&e->mx);
        char err[96] = "";
        smp_t *s = wav_load(path, err, sizeof err);
        pthread_mutex_lock(&e->mx);
        if (s) {
            smp_t *old = atomic_exchange(&e->cur, s);
            snprintf(e->info, sizeof e->info, "%d/%d   %.1f kHz   %s   %.2f s%s", idx + 1, n, s->rate / 1000.0,
                     s->ch == 1 ? "mono" : "stereo", (double)s->frames / s->rate, e->truncated ? "   (liste limitee)" : "");
            e->done_id = id;
            if (old) {
                pthread_mutex_unlock(&e->mx);
                unsigned ep = atomic_load(&e->epoch);   /* attendre que le fil audio ait quitté l'ancien sample */
                for (int k = 0; k < 200 && !atomic_load(&e->quit); k++) {
                    if ((unsigned)(atomic_load(&e->epoch) - ep) >= 2u) break;
                    usleep(10000);
                }
                free(old->d); free(old);
                pthread_mutex_lock(&e->mx);
            }
        } else {
            snprintf(e->info, sizeof e->info, "%d/%d   erreur : %s", idx + 1, n, err);
            e->done_id = id;
        }
    }
    pthread_mutex_unlock(&e->mx);
    return NULL;
}

/* ---------- moteur ---------- */
static void *create(const char *d) {
    (void)d;
    emx_t *e = calloc(1, sizeof *e);
    if (!e) return NULL;
    e->paths = calloc(MAXF, PLEN);
    e->level = 80.0f; e->attack_ms = 2.0f; e->release_ms = 200.0f; e->root = 60; e->loop = 0;
    e->scanning = 1; e->req = 0; e->req_id = 1; e->done_id = 0;
    snprintf(e->name, sizeof e->name, "...");
    snprintf(e->info, sizeof e->info, "Recherche des WAV...");
    atomic_init(&e->cur, NULL); atomic_init(&e->epoch, 0); atomic_init(&e->quit, 0);
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
    smp_t *s = atomic_load(&e->cur);
    if (s) { free(s->d); free(s); }
    free(e->paths);
    pthread_mutex_destroy(&e->mx); pthread_cond_destroy(&e->cv);
    free(e);
}

static void note_on(emx_t *e, int note, int vel) {
    smp_t *s = atomic_load(&e->cur);
    if (!s) return;
    voice_t *t = NULL;
    int k;
    for (k = 0; k < NV; k++) if (e->v[k].stage && e->v[k].note == note) { t = &e->v[k]; break; }
    if (!t) for (k = 0; k < NV; k++) if (!e->v[k].stage) { t = &e->v[k]; break; }
    if (!t) {
        float best = 2.0f;
        for (k = 0; k < NV; k++) if (e->v[k].stage == 3 && e->v[k].env < best) { best = e->v[k].env; t = &e->v[k]; }
    }
    if (!t) {
        unsigned oldest = 0xFFFFFFFFu;
        for (k = 0; k < NV; k++) if (e->v[k].age < oldest) { oldest = e->v[k].age; t = &e->v[k]; }
    }
    t->note = note;
    t->vel = vel / 127.0f;
    t->inc = pow(2.0, (note - e->root) / 12.0) * s->rate / SR;
    t->pos = 0.0; t->env = 0.0f;
    t->smp = s;
    t->stage = 1;
    t->age = ++e->clock;
}
static void note_off(emx_t *e, int note) {
    for (int k = 0; k < NV; k++)
        if (e->v[k].stage && e->v[k].stage != 3 && e->v[k].note == note) e->v[k].stage = 3;
}
static void midi(void *i, const uint8_t *m, int n) {
    emx_t *e = i;
    if (n < 2) return;
    int st = m[0] & 0xF0;
    if (st == 0x90 && n >= 3 && m[2] > 0) note_on(e, m[1], m[2]);
    else if (st == 0x80 || (st == 0x90 && n >= 3 && m[2] == 0)) note_off(e, m[1]);
    else if (st == 0xB0 && n >= 3 && (m[1] == 123 || m[1] == 120))
        for (int k = 0; k < NV; k++) if (e->v[k].stage) e->v[k].stage = 3;
}

/* Les listes d'options arrivent sous forme d'index ("0", "1"...), pas de noms. */
static void set_param(void *i, const char *k, const char *v) {
    emx_t *e = i;
    if (!strcmp(k, "sample")) {
        int idx = atoi(v);
        if (idx < 0) idx = 0;
        pthread_mutex_lock(&e->mx);
        if (idx != e->req) { e->req = idx; e->req_id++; pthread_cond_signal(&e->cv); }
        pthread_mutex_unlock(&e->mx);
    }
    else if (!strcmp(k, "root")) { int r = atoi(v); e->root = r < 0 ? 0 : (r > 127 ? 127 : r); }
    else if (!strcmp(k, "level")) e->level = clampf((float)atof(v), 0.0f, 100.0f);
    else if (!strcmp(k, "attack")) e->attack_ms = clampf((float)atof(v), 1.0f, 2000.0f);
    else if (!strcmp(k, "release")) e->release_ms = clampf((float)atof(v), 5.0f, 3000.0f);
    else if (!strcmp(k, "loop")) e->loop = (v[0] == '1' || !strcmp(v, "on")) ? 1 : 0;
}
static int get_param(void *i, const char *k, char *b, int n) {
    emx_t *e = i;
    if (!strcmp(k, "sample")) { pthread_mutex_lock(&e->mx); int r = e->req; pthread_mutex_unlock(&e->mx); return snprintf(b, n, "%d", r); }
    if (!strcmp(k, "root")) return snprintf(b, n, "%d", e->root);
    if (!strcmp(k, "level")) return snprintf(b, n, "%.1f", e->level);
    if (!strcmp(k, "attack")) return snprintf(b, n, "%.1f", e->attack_ms);
    if (!strcmp(k, "release")) return snprintf(b, n, "%.1f", e->release_ms);
    if (!strcmp(k, "loop")) return snprintf(b, n, "%s", e->loop ? "on" : "off");
    if (!strcmp(k, "sample_name") || !strcmp(k, "sample_info")) {
        pthread_mutex_lock(&e->mx);
        int r = snprintf(b, n, "%s", k[7] == 'n' ? e->name : e->info);
        pthread_mutex_unlock(&e->mx);
        return r;
    }
    return 0;
}

static void render(void *i, int16_t *o, int f) {
    emx_t *e = i;
    smp_t *s = atomic_load(&e->cur);
    float att = 1.0f / (e->attack_ms * 0.001f * (float)SR);
    float rel = 1.0f / (e->release_ms * 0.001f * (float)SR);
    float gain = e->level * 0.01f * 0.5f;
    for (int k = 0; k < NV; k++) if (e->v[k].stage && e->v[k].smp != s) e->v[k].stage = 0;   /* sample remplacé */
    for (int n = 0; n < f; n++) {
        float l = 0.0f, r = 0.0f;
        for (int k = 0; k < NV; k++) {
            voice_t *v = &e->v[k];
            if (!v->stage) continue;
            if (v->stage == 1) { v->env += att; if (v->env >= 1.0f) { v->env = 1.0f; v->stage = 2; } }
            else if (v->stage == 3) { v->env -= rel; if (v->env <= 0.0f) { v->env = 0.0f; v->stage = 0; continue; } }
            const smp_t *m = v->smp;
            int i0 = (int)v->pos, i1 = i0 + 1;
            float fr = (float)(v->pos - i0);
            if (i1 >= m->frames) i1 = e->loop ? 0 : m->frames - 1;
            float a = v->env * v->vel;
            if (m->ch == 1) { float x = (m->d[i0] + (m->d[i1] - m->d[i0]) * fr) * a; l += x; r += x; }
            else {
                l += (m->d[2 * i0] + (m->d[2 * i1] - m->d[2 * i0]) * fr) * a;
                r += (m->d[2 * i0 + 1] + (m->d[2 * i1 + 1] - m->d[2 * i0 + 1]) * fr) * a;
            }
            v->pos += v->inc;
            if (v->pos >= m->frames) {
                if (e->loop) { do v->pos -= m->frames; while (v->pos >= m->frames); }
                else v->stage = 0;
            }
        }
        o[2 * n] = (int16_t)(clampf(l * gain, -1.0f, 1.0f) * 32767.0f);
        o[2 * n + 1] = (int16_t)(clampf(r * gain, -1.0f, 1.0f) * 32767.0f);
    }
    atomic_fetch_add(&e->epoch, 1u);
}

static const mpc_engine_t ENGINE = { create, destroy, midi, set_param, get_param, render, NULL, NULL };
const mpc_engine_t *mpc_engine(void) { return &ENGINE; }
