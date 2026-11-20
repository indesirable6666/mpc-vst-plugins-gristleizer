/* DJ Mixxx-like pour MPC Live II : 2 platines (WAV + MP3 lus en streaming depuis une clé USB, un disque ou la carte SD),
 * bibliothèque (navigation dans /media), pitch, sync BPM, cue, hotcues, boucles, EQ 3 bandes à tuer, filtre, crossfader,
 * 2 unités d'effets (8 effets) routables sur chaque platine et sur la sortie master.
 * Mémoire : 1,8 Mo de tampon par platine (10 s de son) ; aucune piste n'est chargée en entier. */
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
#include "../../wrapper/engine.h"
#include "decoder.inc"

#define SR 44100.0f
#define ND 2
#define RING 441000                     /* 10 s de trames stéréo int16 = 1,76 Mo */
#define AHEAD 132300                    /* 3 s décodées d'avance */
#define HIST 132300                     /* 3 s d'historique derrière la tête de lecture */
#define MAXLOOP 264600                  /* boucle de 6 s au plus */
#define CHUNK 2048
#define ECHOLEN 80000
#ifndef MEDIA_ROOT
#define MEDIA_ROOT "/media"
#endif

typedef struct { const char *key; int kind; /* 0 nombre, 1 option, 2 texte, 3 impulsion */ int nopt; const char *const *opt; float mn, mx, df; } pdef_t;
#include "popts.inc"
static const pdef_t PD[] = {
#include "pdefs.inc"
};
#include "pidx.inc"
#define DP(d, k) (DB + (d) * DK + (k))

typedef struct { char name[256]; int isdir; } lent_t;
typedef struct { lent_t *e; int n; } llist_t;

typedef struct {
    pthread_t th, tha; int th_ok, tha_ok;
    _Atomic int reqgen, anareq;
    char reqpath[512], anapath[512];
    int16_t *ring;
    _Atomic int wstart, wend, seekreq, ready, rpos, keep, loaded_gen, failed;
    int total, sr, ch;
    char title[2][64]; _Atomic int titlei;
    _Atomic int bpm_milli, beat_off, analyzing;
    char path[512];                     /* piste chargée, pour la sauvegarde */
    /* côté audio / interface */
    double pos; int playing, cue, hot[4], loop_on, loop_in, loop_out, waiting;
    float vu, ramp[6];                  /* gain, bas, médium, aigu, fader, crossfader : valeurs lissées */
    float eq[2][16], flt[2][2];
    double taps[8]; int ntap;
} deck_t;

typedef struct {
    float *buf[2]; int w; float dcur, ph[2], st[2][8], k[8], held[2]; unsigned cnt;
    float cb[2][8][1700]; int ci[2][8]; float cl[2][8];
    float ab[2][4][600]; int ai[2][4];
    int type;
} fxi_t;

typedef struct {
    float v[NPAR];
    deck_t d[ND];
    fxi_t *fx[2][3];                    /* unité, source (A, B, master) */
    atomic_bool quit;
    pthread_t lth; int lth_ok;
    llist_t lst[2]; _Atomic int lcur, lreq, lready; char reqdir[512], dir[512];
    int cursor, scroll;
    uint64_t chg[2];
    unsigned blocks;
    float xf_prev[ND]; void *targs; unsigned since;
} mx_t;

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float fnz(float x) { return fabsf(x) < 1e-15f ? 0.0f : x; }
static inline float tnh(float x) { if (x > 3.0f) return 1.0f; if (x < -3.0f) return -1.0f; float x2 = x * x; return x * (27.0f + x2) / (27.0f + 9.0f * x2); }
static void mark(mx_t *m, int i) { m->chg[i >> 6] |= 1ULL << (i & 63); }

/* ---------- bibliothèque : lecture des dossiers dans un fil à part ---------- */
static int lent_cmp(const void *a, const void *b) {
    const lent_t *x = a, *y = b;
    if (x->isdir != y->isdir) return y->isdir - x->isdir;
    return strcasecmp(x->name, y->name);
}
static void lib_scan(mx_t *m, const char *dir, llist_t *out) {
    free(out->e); out->e = NULL; out->n = 0;
    DIR *dp = opendir(dir);
    if (!dp) return;
    int cap = 256; lent_t *e = malloc(sizeof(lent_t) * (size_t)cap); int n = 0;
    struct dirent *de;
    int root = !strcmp(dir, MEDIA_ROOT);
    while (e && (de = readdir(dp))) {
        const char *nm = de->d_name;
        if (nm[0] == '.' || !strcmp(nm, "lost+found") || !strcmp(nm, "System Volume Information") || !strcmp(nm, "$RECYCLE.BIN")) continue;
        if (root && (!strcmp(nm, "EFI") || !strcmp(nm, "az01-internal") || !strncmp(nm, "acvs-", 5))) continue;
        char full[1024]; snprintf(full, sizeof full, "%s/%s", dir, nm);
        int isdir = de->d_type == DT_DIR;
        if (de->d_type == DT_UNKNOWN || de->d_type == DT_LNK) { struct stat st; isdir = stat(full, &st) == 0 && S_ISDIR(st.st_mode); }
        if (!isdir && !is_audio_name(nm)) continue;
        if (n >= 3000) break;
        if (n >= cap) { cap *= 2; lent_t *t = realloc(e, sizeof(lent_t) * (size_t)cap); if (!t) break; e = t; }
        snprintf(e[n].name, sizeof e[n].name, "%s", nm); e[n].isdir = isdir; n++;
        if (atomic_load(&m->quit)) break;
    }
    closedir(dp);
    if (e && n > 1) qsort(e, (size_t)n, sizeof(lent_t), lent_cmp);
    out->e = e; out->n = n;
}
static void *lib_thread(void *arg) {
    mx_t *m = arg; int seen = 0;
    while (!atomic_load(&m->quit)) {
        int rq = atomic_load(&m->lreq);
        if (rq != seen) {
            seen = rq; char dir[512]; snprintf(dir, sizeof dir, "%s", m->reqdir);
            int nxt = 1 - atomic_load(&m->lcur);
            lib_scan(m, dir, &m->lst[nxt]);
            atomic_store(&m->lcur, nxt); atomic_store(&m->lready, seen);
        } else usleep(15000);
    }
    return NULL;
}
static void lib_go(mx_t *m, const char *dir) {
    snprintf(m->dir, sizeof m->dir, "%s", dir); snprintf(m->reqdir, sizeof m->reqdir, "%s", dir);
    m->cursor = -1; m->scroll = 0; atomic_fetch_add(&m->lreq, 1);
}

/* ---------- analyse BPM (fil à part, sur 60 s de la piste, sans la charger en entier) ---------- */
static int analyze(dec_t *dc, int *bpm_milli, int *beat_off, _Atomic int *cancel_gen, int gen) {
    int sr = dc->sr, T = dc->total;
    int s0 = T > 100 * sr ? 20 * sr : 0, len = T - s0 < 60 * sr ? T - s0 : 60 * sr;
    if (len < 8 * sr) return -1;
    int hop = sr / 200, N = len / hop;
    float *lo = calloc((size_t)N + 8, 4), *hi = calloc((size_t)N + 8, 4), *on = calloc((size_t)N + 8, 4);
    int16_t *buf = malloc(4096 * 4);
    if (!lo || !hi || !on || !buf) { free(lo); free(hi); free(on); free(buf); return -1; }
    dec_seek(dc, s0);
    float lp = 0.0f, prev = 0.0f, a = 1.0f - expf(-6.2831853f * 150.0f / (float)sr);
    int k = 0, cnt = 0, got = 0; double elo = 0, ehi = 0;
    while (k < N && (got = dec_read(dc, buf, 4096)) > 0) {
        if (atomic_load(cancel_gen) != gen) { free(lo); free(hi); free(on); free(buf); return -1; }
        for (int i = 0; i < got && k < N; i++) {
            float x = (buf[2 * i] + buf[2 * i + 1]) * (0.5f / 32768.0f);
            lp += a * (x - lp); float d = x - prev; prev = x;
            elo += (double)lp * lp; ehi += (double)d * d;
            if (++cnt >= hop) { lo[k] = sqrtf((float)(elo / hop)); hi[k] = sqrtf((float)(ehi / hop)); k++; cnt = 0; elo = ehi = 0; }
        }
    }
    free(buf);
    N = k;
    if (N < 1600) { free(lo); free(hi); free(on); return -1; }
    for (int i = 1; i < N; i++) { float d1 = lo[i] - lo[i - 1], d2 = hi[i] - hi[i - 1]; on[i] = (d1 > 0 ? d1 : 0) + 0.5f * (d2 > 0 ? d2 * 4.0f : 0); }
    /* retire la moyenne locale (1 s) pour ne garder que les pics */
    float *ps = calloc((size_t)N + 1, 4);
    if (!ps) { free(lo); free(hi); free(on); return -1; }
    for (int i = 0; i < N; i++) ps[i + 1] = ps[i] + on[i];
    for (int i = 0; i < N; i++) { int a0 = i - 100 < 0 ? 0 : i - 100, b0 = i + 100 > N ? N : i + 100; float mean = (ps[b0] - ps[a0]) / (float)(b0 - a0); on[i] = on[i] - mean > 0 ? on[i] - mean : 0; }
    free(ps);
    int Lmin = 200 * 60 / 175, Lmax = 200 * 60 / 70;
    double bestS = -1; int bestL = Lmin;
    static double ac[400];
    for (int L = Lmin / 2; L <= 2 * Lmax + 2 && L < 390; L++) { double s = 0; for (int i = 0; i + L < N; i++) s += (double)on[i] * on[i + L]; ac[L] = s / (double)(N - L); }
    for (int L = Lmin; L <= Lmax; L++) { double s = ac[L] + 0.5 * ac[2 * L] + 0.5 * ac[L / 2 > 0 ? L / 2 : 1]; if (s > bestS) { bestS = s; bestL = L; } }
    double L0 = bestL;
    if (bestL > Lmin && bestL < Lmax) { double y0 = ac[bestL - 1], y1 = ac[bestL], y2 = ac[bestL + 1], den = y0 - 2 * y1 + y2; if (den < 0) L0 += 0.5 * (y0 - y2) / den; }
    double bpm0 = 12000.0 / L0, bb = bpm0, bphase = 0, bscore = -1;
    for (double b = bpm0 * 0.985; b <= bpm0 * 1.015; b += 0.02) {
        double P = 12000.0 / b;
        for (double ph = 0; ph < P; ph += 1.0) {
            double s = 0; int c = 0;
            for (double t = ph; t < N - 1; t += P, c++) { int i = (int)t; double f = t - i; s += on[i] * (1 - f) + on[i + 1] * f; }
            s /= (c ? c : 1);
            if (s > bscore) { bscore = s; bb = b; bphase = ph; }
        }
    }
    while (bb < 78.0) bb *= 2.0;
    while (bb > 170.0) bb /= 2.0;
    double period = (double)sr * 60.0 / bb, off = fmod((double)s0 + bphase * hop, period);
    *bpm_milli = (int)(bb * 1000.0 + 0.5); *beat_off = (int)off;
    free(lo); free(hi); free(on);
    return 0;
}
static void *ana_thread(void *arg) {
    struct { mx_t *m; deck_t *d; } *a = arg; mx_t *m = a->m; deck_t *d = a->d; int seen = 0;
    while (!atomic_load(&m->quit)) {
        int rq = atomic_load(&d->anareq);
        if (rq == seen) { usleep(20000); continue; }
        seen = rq; char path[512]; snprintf(path, sizeof path, "%s", d->anapath);
        atomic_store(&d->analyzing, 1);
        dec_t *dc = dec_open(path); int bpm = 0, off = 0;
        if (dc) { if (analyze(dc, &bpm, &off, &d->anareq, seen) == 0 && atomic_load(&d->anareq) == seen) { atomic_store(&d->beat_off, off); atomic_store(&d->bpm_milli, bpm); } dec_close(dc); }
        atomic_store(&d->analyzing, 0);
    }
    return NULL;
}

/* ---------- fil de lecture d'une platine : charge le fichier et garde le tampon rempli ---------- */
static void *deck_thread(void *arg) {
    struct { mx_t *m; deck_t *d; } *a = arg; mx_t *m = a->m; deck_t *d = a->d;
    dec_t *dc = NULL; int gen = 0; int16_t *tmp = malloc(CHUNK * 4);
    while (!atomic_load(&m->quit) && tmp) {
        int rg = atomic_load(&d->reqgen);
        if (rg != gen) {
            gen = rg; char path[512]; snprintf(path, sizeof path, "%s", d->reqpath);
            atomic_store(&d->ready, 0); atomic_store(&d->failed, 0);
            if (dc) { dec_close(dc); dc = NULL; }
            dc = dec_open(path);
            int ti = 1 - atomic_load(&d->titlei);
            if (!dc) { snprintf(d->title[ti], sizeof d->title[ti], "(cannot open file)"); atomic_store(&d->titlei, ti); atomic_store(&d->failed, 1); atomic_store(&d->loaded_gen, gen); continue; }
            const char *bn = strrchr(path, '/'); bn = bn ? bn + 1 : path;
            snprintf(d->title[ti], sizeof d->title[ti], "%s", bn); char *dot = strrchr(d->title[ti], '.'); if (dot) *dot = 0;
            d->total = dc->total; d->sr = dc->sr; d->ch = dc->ch;
            atomic_store(&d->bpm_milli, 0); atomic_store(&d->beat_off, 0);
            atomic_store(&d->wstart, 0); atomic_store(&d->wend, 0); atomic_store(&d->seekreq, -1); atomic_store(&d->keep, -1); atomic_store(&d->rpos, 0);
            atomic_store(&d->titlei, ti); atomic_store(&d->ready, 1); atomic_store(&d->loaded_gen, gen);
            snprintf(d->anapath, sizeof d->anapath, "%s", path); atomic_fetch_add(&d->anareq, 1);
            continue;
        }
        if (!dc || !atomic_load(&d->ready)) { usleep(5000); continue; }
        int sk = atomic_exchange(&d->seekreq, -1);
        if (sk >= 0) {
            int ws = atomic_load(&d->wstart), we = atomic_load(&d->wend);
            if (!(sk >= ws && sk <= we)) { atomic_store(&d->wend, sk); atomic_store(&d->wstart, sk); dec_seek(dc, sk); }
        }
        int rp = atomic_load(&d->rpos), ws = atomic_load(&d->wstart), we = atomic_load(&d->wend), keep = atomic_load(&d->keep);
        if (we < rp + AHEAD && we < d->total && dc->cur == we) {
            int n = d->total - we < CHUNK ? d->total - we : CHUNK;
            int low = keep >= 0 ? keep : rp - HIST; if (low > rp) low = rp; if (low < 0) low = 0;
            int nws = we + n - RING; if (nws < low) nws = low; if (nws < ws) nws = ws; if (nws > rp) nws = rp;
            if (nws > ws) atomic_store(&d->wstart, nws);
            int got = dec_read(dc, tmp, n);
            if (got <= 0) { d->total = we; usleep(20000); continue; }
            for (int i = 0; i < got; i++) { int s = (we + i) % RING; d->ring[2 * s] = tmp[2 * i]; d->ring[2 * s + 1] = tmp[2 * i + 1]; }
            atomic_store_explicit(&d->wend, we + got, memory_order_release);
        } else if (dc->cur != we) { dec_seek(dc, we); } else usleep(3000);
    }
    free(tmp); if (dc) dec_close(dc);
    return NULL;
}

/* ---------- DSP : égaliseur à 3 bandes (croisements Linkwitz-Riley 4e ordre à 250 Hz et 2,5 kHz) et filtre ---------- */
typedef struct { float b0, b1, b2, a1, a2; } bq_t;
static bq_t LP1, HP1, LP2, HP2;
static void bq_make(bq_t *q, int hp, float fc) {
    float w = 6.2831853f * fc / SR, cs = cosf(w), al = sinf(w) * 0.70710678f, a0 = 1.0f + al;
    if (hp) { q->b0 = (1 + cs) * 0.5f / a0; q->b1 = -(1 + cs) / a0; q->b2 = q->b0; }
    else { q->b0 = (1 - cs) * 0.5f / a0; q->b1 = (1 - cs) / a0; q->b2 = q->b0; }
    q->a1 = -2.0f * cs / a0; q->a2 = (1.0f - al) / a0;
}
static inline float bq_run(const bq_t *q, float x, float *z) {
    float y = q->b0 * x + z[0];
    z[0] = fnz(q->b1 * x - q->a1 * y + z[1]); z[1] = fnz(q->b2 * x - q->a2 * y);
    return y;
}
static inline float eq3(float *z, float x, float gl, float gm, float gh) {
    float lo = bq_run(&LP1, bq_run(&LP1, x, z), z + 2);
    float rest = bq_run(&HP1, bq_run(&HP1, x, z + 4), z + 6);
    float mid = bq_run(&LP2, bq_run(&LP2, rest, z + 8), z + 10);
    float hi = bq_run(&HP2, bq_run(&HP2, rest, z + 12), z + 14);
    return lo * gl + mid * gm + hi * gh;
}
static inline float svf(float *z, float x, float a1, float a2, float a3, float k, int hp) {
    float v3 = x - z[1], v1 = a1 * z[0] + a2 * v3, v2 = z[1] + a2 * z[0] + a3 * v3;
    z[0] = fnz(2.0f * v1 - z[0]); z[1] = fnz(2.0f * v2 - z[1]);
    return hp ? x - k * v1 - v2 : v2;
}
static float eqg(float v, int kill) { if (kill) return 0.0f; return v <= 50.0f ? (v / 50.0f) * (v / 50.0f) : 1.0f + (v - 50.0f) / 50.0f; }
static float xgain(int curve, int orient, float x) {
    if (orient == 1) return 1.0f;
    float p = orient == 0 ? x : 1.0f - x;
    if (curve == 0) return p < 0.5f ? 1.0f : 2.0f * (1.0f - p);
    if (curve == 1) return cosf(p * 1.5707963f);
    return p < 0.9f ? 1.0f : (1.0f - p) / 0.1f;
}

/* ---------- effets (8 types) : une instance par unité et par source ---------- */
static const float ECB[8] = {0.125f, 0.25f, 0.375f, 0.5f, 0.75f, 1.0f, 1.5f, 2.0f};
static const char *ECN[8] = {"1/8", "1/4", "3/8", "1/2", "3/4", "1", "1.5", "2"};
static const int CMB[8] = {1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};
static const int APS[4] = {556, 441, 341, 225};
static fxi_t *fxi_new(void) {
    fxi_t *f = calloc(1, sizeof *f);
    if (!f) return NULL;
    f->buf[0] = calloc(ECHOLEN, 4); f->buf[1] = calloc(ECHOLEN, 4);
    if (!f->buf[0] || !f->buf[1]) { free(f->buf[0]); free(f->buf[1]); free(f); return NULL; }
    f->type = -1;
    return f;
}
static void fxi_free(fxi_t *f) { if (f) { free(f->buf[0]); free(f->buf[1]); free(f); } }
static void fxi_reset(fxi_t *f) {
    memset(f->buf[0], 0, ECHOLEN * 4); memset(f->buf[1], 0, ECHOLEN * 4);
    memset(f->st, 0, sizeof f->st); memset(f->cb, 0, sizeof f->cb); memset(f->ci, 0, sizeof f->ci); memset(f->cl, 0, sizeof f->cl);
    memset(f->ab, 0, sizeof f->ab); memset(f->ai, 0, sizeof f->ai); memset(f->held, 0, sizeof f->held);
    f->w = 0; f->ph[0] = 0.0f; f->ph[1] = 0.25f; f->cnt = 0;
}
static inline float rdi(const float *b, int w, float d) {
    if (d < 1.0f) d = 1.0f;
    if (d > (float)(ECHOLEN - 3)) d = (float)(ECHOLEN - 3);
    float p = (float)w - d; while (p < 0.0f) p += (float)ECHOLEN;
    int i = (int)p; if (i >= ECHOLEN) i = ECHOLEN - 1;
    float fr = p - (float)i; int j = i + 1; if (j >= ECHOLEN) j = 0;
    return b[i] + (b[j] - b[i]) * fr;
}
static void fx_prep(fxi_t *f, int type, float p1, float p2, float bpm) {
    if (f->type != type) { f->type = type; fxi_reset(f); f->dcur = 0.0f; }
    float u1 = p1 * 0.01f, u2 = p2 * 0.01f, *k = f->k;
    if (bpm < 40.0f) bpm = 120.0f;
    switch (type) {
    case 0: k[0] = ECB[(int)(u1 * 7.99f)] * 60.0f / bpm * SR; if (k[0] > ECHOLEN - 8) k[0] = ECHOLEN - 8; k[1] = 0.9f * u2; if (f->dcur < 1.0f) f->dcur = k[0]; break;
    case 1: k[0] = 0.70f + 0.28f * u1; k[1] = 0.1f + 0.6f * u2; break;
    case 2: k[0] = 0.05f * exp2f(u1 * 7.0f) / SR; k[1] = 0.2f + 0.6f * u2; break;
    case 3: k[0] = 0.05f * exp2f(u1 * 7.0f) / SR; k[1] = 1.0f + 4.0f * u2; k[2] = 0.3f + 0.5f * u2; break;
    case 4: k[0] = (float)(1 + (int)(u2 * 31.0f)); k[1] = exp2f(15.0f - (float)(int)(u1 * 13.0f)); break;
    case 5: { float g = tanf(3.14159265f * clampf(20.0f * exp2f(u1 * 10.0f), 20.0f, 18000.0f) / SR); k[0] = g / (1.0f + g); k[1] = u2 * 3.9f; break; }
    case 6: k[0] = 0.1f * exp2f(u1 * 6.3f) / SR; k[1] = u2; break;
    default: k[0] = 0.5f * exp2f(u1 * 5.3f) / SR; k[1] = u2; break;
    }
}
static void fx_run(fxi_t *f, float wet, float *pl, float *pr) {
    float L = *pl, R = *pr, oL = L, oR = R; const float *k = f->k; int w = f->w;
    switch (f->type) {
    case 0: {
        f->dcur += (k[0] - f->dcur) * 0.0004f;
        float dl = rdi(f->buf[0], w, f->dcur), dr = rdi(f->buf[1], w, f->dcur);
        f->st[0][0] = fnz(f->st[0][0] + (dl - f->st[0][0]) * 0.45f); f->st[1][0] = fnz(f->st[1][0] + (dr - f->st[1][0]) * 0.45f);
        f->buf[0][w] = fnz(L + f->st[0][0] * k[1]); f->buf[1][w] = fnz(R + f->st[1][0] * k[1]);
        oL = L + dl * wet; oR = R + dr * wet; break; }
    case 1: {
        float in = (L + R) * 0.015f, s[2] = {0.0f, 0.0f};
        for (int ch = 0; ch < 2; ch++) {
            for (int c = 0; c < 8; c++) {
                int len = CMB[c] + (ch ? 23 : 0), ix = f->ci[ch][c]; float y = f->cb[ch][c][ix];
                f->cl[ch][c] = fnz(y * (1.0f - k[1]) + f->cl[ch][c] * k[1]); f->cb[ch][c][ix] = fnz(in + f->cl[ch][c] * k[0]);
                if (++ix >= len) ix = 0; f->ci[ch][c] = ix; s[ch] += y;
            }
            for (int a = 0; a < 4; a++) {
                int len = APS[a] + (ch ? 23 : 0), ix = f->ai[ch][a]; float bf = f->ab[ch][a][ix], o = -s[ch] + bf;
                f->ab[ch][a][ix] = fnz(s[ch] + bf * 0.5f); s[ch] = o; if (++ix >= len) ix = 0; f->ai[ch][a] = ix;
            }
        }
        oL = L + s[0] * wet * 2.5f; oR = R + s[1] * wet * 2.5f; break; }
    case 2: {
        float in[2] = {L, R}, out[2];
        for (int ch = 0; ch < 2; ch++) {
            float lf = sinf(6.2831853f * f->ph[ch]), d = (0.3f + 1.35f * (1.0f + lf)) * 0.001f * SR;
            float dl = rdi(f->buf[ch], w, d);
            f->buf[ch][w] = fnz(in[ch] + dl * k[1]);
            out[ch] = in[ch] * (1.0f - wet) + (in[ch] + dl) * 0.707f * wet;
            f->ph[ch] += k[0]; if (f->ph[ch] >= 1.0f) f->ph[ch] -= 1.0f;
        }
        oL = out[0]; oR = out[1]; break; }
    case 3: {
        float in[2] = {L, R}, out[2];
        if ((f->cnt++ & 7u) == 0) for (int ch = 0; ch < 2; ch++) {
            float lf = 0.5f + 0.5f * sinf(6.2831853f * f->ph[ch]);
            float t = tanf(3.14159265f * fminf(200.0f * exp2f(lf * k[1]), 12000.0f) / SR);
            f->st[ch][7] = (t - 1.0f) / (t + 1.0f);
        }
        for (int ch = 0; ch < 2; ch++) {
            float a = f->st[ch][7], x = in[ch] + f->st[ch][4] * k[2];
            for (int s = 0; s < 4; s++) { float y = a * x + f->st[ch][s]; f->st[ch][s] = fnz(x - a * y); x = y; }
            f->st[ch][4] = fnz(x);
            out[ch] = in[ch] * (1.0f - wet) + (in[ch] + x) * 0.5f * wet;
            f->ph[ch] += k[0]; if (f->ph[ch] >= 1.0f) f->ph[ch] -= 1.0f;
        }
        oL = out[0]; oR = out[1]; break; }
    case 4: {
        if (f->cnt++ % (unsigned)k[0] == 0) { f->held[0] = L; f->held[1] = R; }
        float cl = roundf(f->held[0] * k[1]) / k[1], cr = roundf(f->held[1] * k[1]) / k[1];
        oL = L * (1.0f - wet) + cl * wet; oR = R * (1.0f - wet) + cr * wet; break; }
    case 5: {
        float in[2] = {L, R}, out[2], G = k[0], kk = k[1], G2 = G * G, G4 = G2 * G2;
        for (int ch = 0; ch < 2; ch++) {
            float *s = f->st[ch], b1 = (1 - G) * s[0], b2 = (1 - G) * s[1], b3 = (1 - G) * s[2], b4 = (1 - G) * s[3];
            float S = G2 * G * b1 + G2 * b2 + G * b3 + b4, u = tnh((in[ch] - kk * S) / (1.0f + kk * G4)), y = u;
            for (int q = 0; q < 4; q++) { float vv = (y - s[q]) * G; y = vv + s[q]; s[q] = fnz(y + vv); }
            out[ch] = in[ch] * (1.0f - wet) + y * (1.0f + 0.3f * kk) * wet;
        }
        oL = out[0]; oR = out[1]; break; }
    case 6: {
        float th = (0.5f + 0.5f * k[1] * sinf(6.2831853f * f->ph[0])) * 1.5707963f;
        f->ph[0] += k[0]; if (f->ph[0] >= 1.0f) f->ph[0] -= 1.0f;
        oL = L * (1.0f - wet) + L * cosf(th) * 1.414f * wet; oR = R * (1.0f - wet) + R * sinf(th) * 1.414f * wet; break; }
    default: {
        float g = 1.0f - k[1] * (0.5f - 0.5f * sinf(6.2831853f * f->ph[0]));
        f->ph[0] += k[0]; if (f->ph[0] >= 1.0f) f->ph[0] -= 1.0f;
        oL = L * (1.0f - wet) + L * g * wet; oR = R * (1.0f - wet) + R * g * wet; break; }
    }
    if (++w >= ECHOLEN) w = 0;
    f->w = w;
    if (!isfinite(oL) || !isfinite(oR) || fabsf(oL) > 64.0f || fabsf(oR) > 64.0f) { f->type = -1; oL = oR = 0.0f; }
    *pl = oL; *pr = oR;
}
static void fx_desc(mx_t *m, int u, char *b, int n) {
    int t = (int)m->v[FB + u * FK + F_TYPE], p1 = (int)m->v[FB + u * FK + F_P1], p2 = (int)m->v[FB + u * FK + F_P2];
    static const char *NM[8] = {"Echo", "Reverb", "Flanger", "Phaser", "Bitcrusher", "Moog Filter", "Autopan", "Tremolo"};
    static const char *A[8] = {"Delay", "Size", "Rate", "Rate", "Bits", "Cutoff", "Rate", "Rate"};
    static const char *B[8] = {"Feedback", "Damp", "Feedback", "Depth", "Rate div", "Resonance", "Depth", "Depth"};
    if (t == 0) snprintf(b, (size_t)n, "%s: Delay %s beat, Feedback %d%%", NM[t], ECN[(int)(p1 * 0.01f * 7.99f)], p2);
    else snprintf(b, (size_t)n, "%s: %s %d%%, %s %d%%", NM[t], A[t], p1, B[t], p2);
}

/* ---------- actions ---------- */
static float pitchfrac(mx_t *m, int d) { static const float R[3] = {0.08f, 0.16f, 0.5f}; return m->v[DP(d, D_PITCH)] * 0.01f * R[(int)m->v[DP(d, D_PRANGE)]]; }
static float eff_bpm(mx_t *m, int d) { int b = atomic_load(&m->d[d].bpm_milli); return b ? (float)b * 0.001f * (1.0f + pitchfrac(m, d)) : 0.0f; }
static void set_play(mx_t *m, int d, int p) {
    deck_t *dk = &m->d[d];
    if (p && !atomic_load(&dk->ready)) p = 0;
    dk->playing = p; m->v[DP(d, D_PLAY)] = (float)p; mark(m, DP(d, D_PLAY));
}
static void loop_apply(mx_t *m, int d) {
    deck_t *dk = &m->d[d];
    if (m->v[DP(d, D_LOOP)] < 0.5f) { dk->loop_on = 0; atomic_store(&dk->keep, -1); return; }
    int b = atomic_load(&dk->bpm_milli);
    if (!b || !dk->sr || !atomic_load(&dk->ready)) { m->v[DP(d, D_LOOP)] = 0; mark(m, DP(d, D_LOOP)); return; }
    static const float LB[5] = {0.5f, 1, 2, 4, 8};
    double beat = (double)dk->sr * 60.0 / ((double)b * 0.001);
    int len = (int)(beat * LB[(int)m->v[DP(d, D_LSIZE)]]); if (len > MAXLOOP) len = MAXLOOP; if (len < 64) len = 64;
    int in = (int)dk->pos, off = atomic_load(&dk->beat_off);
    if (!dk->loop_on) { double kk = floor(((double)in - off) / beat + 0.5); in = (int)((double)off + kk * beat); if (in < 0) in = 0; } else in = dk->loop_in;
    dk->loop_in = in; dk->loop_out = in + len; dk->loop_on = 1; atomic_store(&dk->keep, in);
}
static void do_tap(mx_t *m, int d) {
    deck_t *dk = &m->d[d]; struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    double t = (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
    if (dk->ntap > 0 && t - dk->taps[dk->ntap - 1] > 2.5) dk->ntap = 0;
    if (dk->ntap == 8) { memmove(dk->taps, dk->taps + 1, 7 * sizeof(double)); dk->ntap = 7; }
    dk->taps[dk->ntap++] = t;
    if (dk->ntap >= 3 && atomic_load(&dk->ready) && dk->sr > 0) {
        double avg = (dk->taps[dk->ntap - 1] - dk->taps[0]) / (double)(dk->ntap - 1), bpm = 60.0 / avg;
        while (bpm < 78.0) bpm *= 2.0;
        while (bpm > 170.0) bpm /= 2.0;
        bpm /= 1.0 + (double)pitchfrac(m, d);                 /* le tempo tapé est celui qu'on entend : on retire le pitch */
        double period = (double)dk->sr * 60.0 / bpm;
        atomic_store(&dk->beat_off, (int)fmod(dk->pos, period)); atomic_store(&dk->bpm_milli, (int)(bpm * 1000.0 + 0.5));
    }
}
static void do_sync(mx_t *m, int d) {
    int o = 1 - d; deck_t *dk = &m->d[d], *ot = &m->d[o];
    float bt = (float)atomic_load(&dk->bpm_milli) * 0.001f, bo = eff_bpm(m, o);
    if (bt < 1.0f || bo < 1.0f) return;
    static const float R[3] = {0.08f, 0.16f, 0.5f};
    float r = R[(int)m->v[DP(d, D_PRANGE)]], pf = bo / bt - 1.0f;
    m->v[DP(d, D_PITCH)] = clampf(pf / r * 100.0f, -100.0f, 100.0f); mark(m, DP(d, D_PITCH));
    if (ot->playing && dk->playing && dk->sr && ot->sr) {
        double pT = (double)dk->sr * 60.0 / bt, pO = (double)ot->sr * 60.0 / ((double)atomic_load(&ot->bpm_milli) * 0.001);
        double phT = fmod((dk->pos - atomic_load(&dk->beat_off)) / pT + 1000.0, 1.0), phO = fmod((ot->pos - atomic_load(&ot->beat_off)) / pO + 1000.0, 1.0);
        double dl = phO - phT; if (dl > 0.5) dl -= 1.0; if (dl < -0.5) dl += 1.0;
        dk->pos += dl * pT;
    }
}
static void deck_load(mx_t *m, int d, const char *path) {
    deck_t *dk = &m->d[d];
    dk->playing = 0; dk->pos = 0; dk->cue = 0; dk->loop_on = 0; dk->waiting = 0;
    for (int i = 0; i < 4; i++) dk->hot[i] = -1;
    atomic_store(&dk->ready, 0); atomic_store(&dk->keep, -1);
    m->v[DP(d, D_PLAY)] = 0; m->v[DP(d, D_LOOP)] = 0; m->v[DP(d, D_PITCH)] = 0;
    snprintf(dk->path, sizeof dk->path, "%s", path); snprintf(dk->reqpath, sizeof dk->reqpath, "%s", path);
    atomic_fetch_add(&dk->reqgen, 1);
    mark(m, DP(d, D_PLAY)); mark(m, DP(d, D_LOOP)); mark(m, DP(d, D_POS)); mark(m, DP(d, D_PITCH));
}
static int cur_path(mx_t *m, char *out, size_t n) {
    llist_t *L = &m->lst[atomic_load(&m->lcur)];
    if (m->cursor < 0 || m->cursor >= L->n || L->e[m->cursor].isdir) return 0;
    snprintf(out, n, "%s/%s", m->dir, L->e[m->cursor].name);
    return 1;
}
static void lib_clamp(mx_t *m) {
    llist_t *L = &m->lst[atomic_load(&m->lcur)];
    int mx = L->n - 8; if (mx < 0) mx = 0;
    m->scroll = m->scroll < 0 ? 0 : (m->scroll > mx ? mx : m->scroll);
    m->v[G_LIB_SCROLL] = (float)m->scroll; mark(m, G_LIB_SCROLL);
}
static void act(mx_t *m, int i) {
    if (i >= G_LIB_TAP0 && i < G_LIB_TAP0 + 8) {
        llist_t *L = &m->lst[atomic_load(&m->lcur)]; int ix = m->scroll + (i - G_LIB_TAP0);
        if (ix >= 0 && ix < L->n) {
            if (L->e[ix].isdir) { char nd[512]; snprintf(nd, sizeof nd, "%s/%s", m->dir, L->e[ix].name); lib_go(m, nd); m->v[G_LIB_SCROLL] = 0; mark(m, G_LIB_SCROLL); }
            else m->cursor = ix;
        }
    } else if (i == G_LIB_UP) {
        if (strcmp(m->dir, MEDIA_ROOT)) { char nd[512]; snprintf(nd, sizeof nd, "%s", m->dir); char *sl = strrchr(nd, '/'); if (sl && sl != nd) *sl = 0; if (strlen(nd) < strlen(MEDIA_ROOT)) snprintf(nd, sizeof nd, "%s", MEDIA_ROOT); lib_go(m, nd); m->v[G_LIB_SCROLL] = 0; mark(m, G_LIB_SCROLL); }
    } else if (i == G_LIB_PGUP) { m->scroll -= 8; lib_clamp(m); }
    else if (i == G_LIB_PGDN) { m->scroll += 8; lib_clamp(m); }
    else if (i == G_LIB_LOADA || i == G_LIB_LOADB) { char p[512]; if (cur_path(m, p, sizeof p)) deck_load(m, i == G_LIB_LOADA ? 0 : 1, p); }
    else if (i >= DB && i < DB + ND * DK) {
        int d = (i - DB) / DK, k = (i - DB) % DK; deck_t *dk = &m->d[d];
        if (k == D_CUE) { if (dk->playing) { dk->pos = dk->cue; set_play(m, d, 0); } else dk->cue = (int)dk->pos; }
        else if (k >= D_HC1 && k <= D_HC4) {
            int j = k - D_HC1;
            if (m->v[DP(d, D_HCCLR)] > 0.5f) dk->hot[j] = -1; else if (dk->hot[j] < 0) dk->hot[j] = (int)dk->pos; else dk->pos = dk->hot[j];
        }
        else if (k == D_SYNC) do_sync(m, d);
        else if (k == D_TAP) do_tap(m, d);
        else if (k == D_NUDM) dk->pos -= 0.010 * (dk->sr ? dk->sr : 44100);
        else if (k == D_NUDP) dk->pos += 0.010 * (dk->sr ? dk->sr : 44100);
        if (dk->pos < 0) dk->pos = 0;
    }
}

/* ---------- interface moteur ---------- */
static int keyidx(const char *k) { for (int i = 0; i < NPAR; i++) if (!strcmp(PD[i].key, k)) return i; return -1; }
static void *create(const char *dir) {
    (void)dir;
    mx_t *m = calloc(1, sizeof *m);
    if (!m) return NULL;
    for (int i = 0; i < NPAR; i++) m->v[i] = PD[i].df;
    static int once; if (!once) { once = 1; bq_make(&LP1, 0, 250.0f); bq_make(&HP1, 1, 250.0f); bq_make(&LP2, 0, 2500.0f); bq_make(&HP2, 1, 2500.0f); }
    struct targ { mx_t *m; deck_t *d; } *ta = calloc(ND * 2, sizeof *ta);
    if (!ta) { free(m); return NULL; }
    m->targs = ta;
    int ok = 1;
    for (int d = 0; d < ND; d++) {
        deck_t *dk = &m->d[d];
        dk->ring = calloc((size_t)RING * 2, sizeof(int16_t));
        atomic_init(&dk->seekreq, -1); atomic_init(&dk->keep, -1);
        for (int j = 0; j < 4; j++) dk->hot[j] = -1;
        for (int j = 0; j < 5; j++) dk->ramp[j] = 1.0f;
        snprintf(dk->title[0], sizeof dk->title[0], "(no track)");
        for (int u = 0; u < 2; u++) { m->fx[u][d] = fxi_new(); if (!m->fx[u][d]) ok = 0; }
        if (!dk->ring) ok = 0;
    }
    for (int u = 0; u < 2; u++) { m->fx[u][2] = fxi_new(); if (!m->fx[u][2]) ok = 0; }
    if (!ok) goto fail;
    for (int u = 0; u < 2; u++) for (int sidx = 0; sidx < 3; sidx++) fxi_reset(m->fx[u][sidx]);   /* touche la mémoire une fois pour toutes : pas d'à-coup à la première activation */
    lib_go(m, MEDIA_ROOT);
    if (pthread_create(&m->lth, NULL, lib_thread, m)) goto fail;
    m->lth_ok = 1;
    for (int d = 0; d < ND; d++) {
        ta[d].m = m; ta[d].d = &m->d[d]; ta[ND + d].m = m; ta[ND + d].d = &m->d[d];
        m->d[d].th_ok = pthread_create(&m->d[d].th, NULL, deck_thread, &ta[d]) == 0;
        m->d[d].tha_ok = pthread_create(&m->d[d].tha, NULL, ana_thread, &ta[ND + d]) == 0;
    }
    return m;
fail:
    atomic_store(&m->quit, 1);
    if (m->lth_ok) pthread_join(m->lth, NULL);
    for (int d = 0; d < ND; d++) free(m->d[d].ring);
    for (int u = 0; u < 2; u++) for (int s = 0; s < 3; s++) fxi_free(m->fx[u][s]);
    free(ta); free(m);
    return NULL;
}
static void destroy(void *i) {
    mx_t *m = i; if (!m) return;
    atomic_store(&m->quit, 1);
    if (m->lth_ok) pthread_join(m->lth, NULL);
    for (int d = 0; d < ND; d++) { if (m->d[d].th_ok) pthread_join(m->d[d].th, NULL); if (m->d[d].tha_ok) pthread_join(m->d[d].tha, NULL); free(m->d[d].ring); }
    for (int u = 0; u < 2; u++) for (int s = 0; s < 3; s++) fxi_free(m->fx[u][s]);
    for (int l = 0; l < 2; l++) free(m->lst[l].e);
    free(m->targs); free(m);
}
static void midi(void *i, const uint8_t *msg, int n) {
    mx_t *m = i;
    if (n < 3 || (msg[0] & 0xF0) != 0x90 || msg[2] == 0) return;
    int nt = msg[1];
    if (nt >= 36 && nt <= 39) act(m, DP(0, D_HC1 + nt - 36));
    else if (nt >= 40 && nt <= 43) act(m, DP(1, D_HC1 + nt - 40));
    else if (nt == 44 || nt == 45) set_play(m, nt - 44, !m->d[nt - 44].playing);
    else if (nt == 46 || nt == 47) act(m, DP(nt - 46, D_CUE));
    else if (nt == 48 || nt == 49) act(m, DP(nt - 48, D_SYNC));
}
static void set_param(void *inst, const char *k, const char *v) {
    mx_t *m = inst;
    if (!strcmp(k, "state")) {
        if (strncmp(v, "MXS1;", 5)) return;
        const char *c = v + 5;
        for (int i = 0; i < NPAR && *c && *c != '|'; i++) {
            if (PD[i].kind > 1 || i == DP(0, D_POS) || i == DP(1, D_POS) || i == DP(0, D_PLAY) || i == DP(1, D_PLAY) || i == G_LIB_SCROLL) continue;
            char *e; float x = strtof(c, &e); if (e == c) break;
            m->v[i] = clampf(x, PD[i].mn, PD[i].mx); c = *e == ';' ? e + 1 : e;
        }
        for (int d = 0; d < ND; d++) { char tag[8]; snprintf(tag, sizeof tag, "|%c=", 'A' + d); const char *p = strstr(v, tag); if (p) { char path[512]; size_t q = 0; p += 3; while (*p && *p != '|' && q < sizeof path - 1) path[q++] = *p++; path[q] = 0; if (q) deck_load(m, d, path); } }
        for (int i = 0; i < NPAR; i++) if (PD[i].kind <= 1) mark(m, i);
        return;
    }
    int i = keyidx(k);
    if (i < 0 || PD[i].kind == 2) return;
    if (PD[i].kind == 3) { if (atof(v) > 0.5) act(m, i); return; }
    float x;
    if (PD[i].kind == 1 && (v[0] < '0' || v[0] > '9')) { x = 0; for (int o = 0; o < PD[i].nopt; o++) if (!strcmp(PD[i].opt[o], v)) x = (float)o; }
    else x = (float)atof(v);
    x = clampf(x, PD[i].mn, PD[i].mx);
    if (PD[i].kind == 1 || PD[i].mx - PD[i].mn >= 1.5f) { if (i != DP(0, D_PITCH) && i != DP(1, D_PITCH)) x = floorf(x + 0.5f); }
    if (i == G_LIB_SCROLL) { m->v[i] = x; m->scroll = (int)x; lib_clamp(m); return; }
    if (i >= DB && i < DB + ND * DK) {
        int d = (i - DB) / DK, kk = (i - DB) % DK; deck_t *dk = &m->d[d];
        if (kk == D_POS) {
            if (atomic_load(&dk->ready) && dk->total > 0) {
                int cur = (int)(dk->pos / (double)dk->total * 1000.0 + 0.5);
                if ((int)x != cur) dk->pos = (double)x / 1000.0 * (double)dk->total;
            }
            return;
        }
        m->v[i] = x;
        if (kk == D_PLAY) set_play(m, d, x > 0.5f);
        else if (kk == D_LOOP || kk == D_LSIZE) loop_apply(m, d);
        return;
    }
    m->v[i] = x;
    if (i >= FB && i < FB + 2 * FK) { int u = (i - FB) / FK, kk = (i - FB) % FK; if (kk == F_RA || kk == F_RB || kk == F_RM) { fxi_t *f = m->fx[u][kk - F_RA]; if (f) f->type = -1; } }
}
static void ftime(char *b, int n, double sec) { if (sec < 0) sec = 0; int s = (int)sec; snprintf(b, (size_t)n, "%02d:%02d", s / 60, s % 60); }
static int get_param(void *inst, const char *k, char *b, int n) {
    mx_t *m = inst;
    if (!strcmp(k, "state")) {
        char tmp[1536]; int o = snprintf(tmp, sizeof tmp, "MXS1;");
        for (int i = 0; i < NPAR; i++) {
            if (PD[i].kind > 1 || i == DP(0, D_POS) || i == DP(1, D_POS) || i == DP(0, D_PLAY) || i == DP(1, D_PLAY) || i == G_LIB_SCROLL) continue;
            o += snprintf(tmp + o, sizeof tmp - (size_t)o, "%g;", m->v[i]);
        }
        for (int d = 0; d < ND; d++) if (m->d[d].path[0]) o += snprintf(tmp + o, sizeof tmp - (size_t)o, "|%c=%s", 'A' + d, m->d[d].path);
        if (o <= 0 || o >= (int)sizeof tmp || o + 1 > n) return 0;
        memcpy(b, tmp, (size_t)o + 1); return o;
    }
    int i = keyidx(k);
    if (i < 0) {
        size_t L = strlen(k);
        if (L > 3 && !strcmp(k + L - 3, "_on")) return snprintf(b, (size_t)n, "0");
        return 0;
    }
    if (PD[i].kind == 3) return snprintf(b, (size_t)n, "0");
    if (PD[i].kind == 1) return snprintf(b, (size_t)n, "%s", PD[i].opt[(int)m->v[i]]);
    if (PD[i].kind == 0) {
        if (i >= DB && i < DB + ND * DK && (i - DB) % DK == D_POS) { deck_t *dk = &m->d[(i - DB) / DK]; return snprintf(b, (size_t)n, "%d", dk->total > 0 ? (int)(dk->pos / (double)dk->total * 1000.0 + 0.5) : 0); }
        return snprintf(b, (size_t)n, "%d", (int)floorf(m->v[i] + 0.5f));
    }
    /* textes */
    if (i == G_LIB_DIR) { size_t L = strlen(m->dir); return snprintf(b, (size_t)n, "%s%s", L > 66 ? "..." : "", L > 66 ? m->dir + L - 63 : m->dir); }
    if (i == G_LIB_INFO) {
        llist_t *Ls = &m->lst[atomic_load(&m->lcur)];
        return snprintf(b, (size_t)n, "%s  %d item(s)%s", atomic_load(&m->lready) ? "" : "reading...", Ls->n, HAVE_MP3 ? "  WAV + MP3" : "  WAV only (minimp3.h missing)");
    }
    if (i >= G_LIB_ROW0 && i < G_LIB_ROW0 + 8) {
        llist_t *Ls = &m->lst[atomic_load(&m->lcur)]; int ix = m->scroll + (i - G_LIB_ROW0);
        if (!atomic_load(&m->lready)) return snprintf(b, (size_t)n, "%s", i == G_LIB_ROW0 ? "  reading..." : " ");
        if (ix >= Ls->n) return snprintf(b, (size_t)n, "%s", (i == G_LIB_ROW0 && Ls->n == 0) ? "  (empty folder)" : " ");
        char nm[84]; snprintf(nm, sizeof nm, "%s", Ls->e[ix].name);
        return snprintf(b, (size_t)n, "%s%s%s", ix == m->cursor ? "> " : "  ", Ls->e[ix].isdir ? "[+] " : "", nm);
    }
    if (i >= DB && i < DB + ND * DK) {
        int d = (i - DB) / DK, kk = (i - DB) % DK; deck_t *dk = &m->d[d];
        int rdy = atomic_load(&dk->ready);
        if (kk == D_TITLE) return snprintf(b, (size_t)n, "%s", (rdy || atomic_load(&dk->failed) || atomic_load(&dk->reqgen)) ? dk->title[atomic_load(&dk->titlei)] : "(no track)");
        if (kk == D_TIME) { if (!rdy || !dk->sr) return snprintf(b, (size_t)n, "--:--  --:--"); char a[16], c[16]; ftime(a, 16, dk->pos / dk->sr); ftime(c, 16, (dk->total - dk->pos) / dk->sr); return snprintf(b, (size_t)n, "%s  -%s", a, c); }
        if (kk == D_BPM) {
            int bm = atomic_load(&dk->bpm_milli); float pf = pitchfrac(m, d) * 100.0f;
            if (!rdy) return snprintf(b, (size_t)n, " ");
            if (!bm) return snprintf(b, (size_t)n, "%s  %+.1f%%", atomic_load(&dk->analyzing) ? "analysing..." : "-- BPM", pf);
            return snprintf(b, (size_t)n, "%.1f BPM  %+.1f%%", (float)bm * 0.001f * (1.0f + pf * 0.01f), pf);
        }
        if (kk == D_VU) { int len = (int)(sqrtf(clampf(dk->vu, 0.0f, 1.0f)) * 16.0f); char t[24]; for (int q = 0; q < 16; q++) t[q] = q < len ? '|' : '.'; t[16] = 0; return snprintf(b, (size_t)n, "%s", t); }
    }
    if (i >= FB && i < FB + 2 * FK && (i - FB) % FK == F_DESC) { char t[96]; fx_desc(m, (i - FB) / FK, t, sizeof t); return snprintf(b, (size_t)n, "%s", t); }
    return snprintf(b, (size_t)n, " ");
}
int mpc_engine_changed(void *inst, char *key, int len) {
    mx_t *m = inst;
    for (int w = 0; w < 2; w++) if (m->chg[w]) {
        int b = __builtin_ctzll(m->chg[w]); m->chg[w] &= ~(1ULL << b);
        snprintf(key, (size_t)len, "%s", PD[w * 64 + b].key); return 1;
    }
    return 0;
}

/* ---------- audio ---------- */
static void render_chunk(mx_t *m, int16_t *out, int n) {
    float DL[ND][256], DR[ND][256], mixL[256], mixR[256];
    memset(mixL, 0, sizeof mixL); memset(mixR, 0, sizeof mixR);
    int curve = (int)m->v[G_XCURVE]; float xf = m->v[G_XFADE] * 0.01f;
    float bp[ND]; for (int d = 0; d < ND; d++) bp[d] = eff_bpm(m, d);
    for (int d = 0; d < ND; d++) {
        deck_t *dk = &m->d[d];
        memset(DL[d], 0, sizeof DL[d]); memset(DR[d], 0, sizeof DR[d]);
        int ready = atomic_load(&dk->ready);
        int ws = atomic_load(&dk->wstart), we = atomic_load_explicit(&dk->wend, memory_order_acquire);
        if (ready) {
            int ip0 = (int)dk->pos;
            if (ip0 < ws || ip0 > we + 11025) { if (!dk->waiting) { atomic_store(&dk->seekreq, ip0); dk->waiting = 1; } }
            else if (ip0 >= ws && ip0 < we) dk->waiting = 0;
        }
        if (ready && dk->playing && dk->sr > 0) {
            float rate = (float)dk->sr / SR * (1.0f + pitchfrac(m, d));
            for (int i = 0; i < n; i++) {
                if (dk->loop_on) while (dk->pos >= dk->loop_out) dk->pos -= (double)(dk->loop_out - dk->loop_in);
                int ip = (int)dk->pos;
                if (ip < ws || ip + 1 >= we) continue;                    /* tampon pas prêt : silence, la position attend */
                float fr = (float)(dk->pos - ip);
                int i0 = ip - 1 >= ws ? ip - 1 : ip, i3 = ip + 2 < we ? ip + 2 : ip + 1;
                const int16_t *r = dk->ring;
                int a0 = (i0 % RING) * 2, a1 = (ip % RING) * 2, a2 = ((ip + 1) % RING) * 2, a3 = (i3 % RING) * 2;
                for (int c = 0; c < 2; c++) {
                    float p0 = r[a0 + c], p1 = r[a1 + c], p2 = r[a2 + c], p3 = r[a3 + c];
                    float y = p1 + 0.5f * fr * (p2 - p0 + fr * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3 + fr * (3.0f * (p1 - p2) + p3 - p0)));
                    (c ? DR[d] : DL[d])[i] = y * (1.0f / 32768.0f);
                }
                dk->pos += rate;
                if (dk->pos >= (double)dk->total - 2.0) { dk->pos = (double)dk->total - 2.0; set_play(m, d, 0); break; }
            }
        }
        atomic_store(&dk->rpos, (int)dk->pos);
    }
    for (int d = 0; d < ND; d++) {
        deck_t *dk = &m->d[d];
        int routed[2] = {m->v[FB + F_RA + d] > 0.5f, m->v[FB + FK + F_RA + d] > 0.5f};
        if (!dk->playing && !routed[0] && !routed[1] && dk->vu < 1e-4f) { dk->vu = 0.0f; continue; }
        float tg[5], cur[5], inc[5], pk = 0.0f;
        float trim = m->v[DP(d, D_GAIN)] / 50.0f;
        tg[0] = trim * trim;
        tg[1] = eqg(m->v[DP(d, D_LOW)], m->v[DP(d, D_KL)] > 0.5f); tg[2] = eqg(m->v[DP(d, D_MID)], m->v[DP(d, D_KM)] > 0.5f); tg[3] = eqg(m->v[DP(d, D_HI)], m->v[DP(d, D_KH)] > 0.5f);
        tg[4] = m->v[DP(d, D_FADER)] * 0.01f * xgain(curve, (int)m->v[DP(d, D_ORIENT)], xf);
        for (int q = 0; q < 5; q++) { cur[q] = dk->ramp[q]; inc[q] = (tg[q] - cur[q]) / (float)n; }
        float kv = m->v[DP(d, D_FILTER)]; int fon = fabsf(kv - 50.0f) > 2.0f, fhp = kv > 50.0f;
        float fa1 = 0, fa2 = 0, fa3 = 0, fk = 1.1f;
        if (fon) {
            float fc = fhp ? 20.0f * powf(400.0f, (kv - 50.0f) / 50.0f) : 20000.0f * powf(0.004f, (50.0f - kv) / 50.0f);
            float g = tanf(3.14159265f * clampf(fc, 20.0f, 19000.0f) / SR);
            fa1 = 1.0f / (1.0f + g * (g + fk)); fa2 = g * fa1; fa3 = g * fa2;
        }
        int u_on[2]; float wet[2];
        for (int u = 0; u < 2; u++) {
            u_on[u] = routed[u]; int b = FB + u * FK; wet[u] = m->v[b + F_WET] * 0.01f;
            if (u_on[u]) fx_prep(m->fx[u][d], (int)m->v[b + F_TYPE], m->v[b + F_P1], m->v[b + F_P2], bp[d] > 0 ? bp[d] : (bp[1 - d] > 0 ? bp[1 - d] : 120.0f));
        }
        for (int i = 0; i < n; i++) {
            for (int q = 0; q < 5; q++) cur[q] += inc[q];
            float l = DL[d][i] * cur[0], r = DR[d][i] * cur[0];
            l = eq3(dk->eq[0], l, cur[1], cur[2], cur[3]); r = eq3(dk->eq[1], r, cur[1], cur[2], cur[3]);
            if (fon) { l = svf(dk->flt[0], l, fa1, fa2, fa3, fk, fhp); r = svf(dk->flt[1], r, fa1, fa2, fa3, fk, fhp); }
            for (int u = 0; u < 2; u++) if (u_on[u]) fx_run(m->fx[u][d], wet[u], &l, &r);
            float al = fabsf(l), ar = fabsf(r);
            if (al > pk) pk = al;
            if (ar > pk) pk = ar;
            mixL[i] += l * cur[4]; mixR[i] += r * cur[4];
        }
        for (int q = 0; q < 5; q++) dk->ramp[q] = tg[q];
        dk->vu = pk > dk->vu * 0.9f ? pk : dk->vu * 0.9f;
    }
    int mon[2] = {m->v[FB + F_RM] > 0.5f, m->v[FB + FK + F_RM] > 0.5f};
    float bpm = bp[0] > 0 && bp[1] > 0 ? 0.5f * (bp[0] + bp[1]) : (bp[0] > 0 ? bp[0] : (bp[1] > 0 ? bp[1] : 120.0f)), mw[2];
    for (int u = 0; u < 2; u++) { int b = FB + u * FK; mw[u] = m->v[b + F_WET] * 0.01f; if (mon[u]) fx_prep(m->fx[u][2], (int)m->v[b + F_TYPE], m->v[b + F_P1], m->v[b + F_P2], bpm); }
    float mg0 = m->xf_prev[0], mg1 = m->v[G_MASTER] * 0.01f, mi = (mg1 - mg0) / (float)n;
    for (int i = 0; i < n; i++) {
        float l = mixL[i], r = mixR[i]; mg0 += mi;
        for (int u = 0; u < 2; u++) if (mon[u]) fx_run(m->fx[u][2], mw[u], &l, &r);
        out[2 * i] = (int16_t)(clampf(tnh(l * mg0), -1.0f, 1.0f) * 32767.0f); out[2 * i + 1] = (int16_t)(clampf(tnh(r * mg0), -1.0f, 1.0f) * 32767.0f);
    }
    m->xf_prev[0] = mg1;
}
static void lib_hl(mx_t *m) {
    llist_t *L = &m->lst[atomic_load(&m->lcur)]; size_t dl = strlen(m->dir);
    for (int d = 0; d < ND; d++) {
        int row = -1; const char *p = m->d[d].path;
        if (p[0] && !strncmp(p, m->dir, dl) && p[dl] == '/' && !strchr(p + dl + 1, '/')) {
            const char *nm = p + dl + 1;
            for (int i = 0; i < L->n; i++) if (!L->e[i].isdir && !strcmp(L->e[i].name, nm)) { if (i >= m->scroll && i < m->scroll + 8) row = i - m->scroll; break; }
        }
        float nv = (float)(row + 1); int ix = G_LIB_HA + d;
        if (m->v[ix] != nv) { m->v[ix] = nv; mark(m, ix); }
    }
}
static void render(void *inst, int16_t *out, int frames) {
    mx_t *m = inst;
    for (int done = 0; done < frames;) {
        int n = frames - done < 256 ? frames - done : 256;
        render_chunk(m, out + 2 * done, n); done += n;
    }
    m->since += (unsigned)frames;
    if (m->since >= 4410) { m->since = 0; lib_hl(m); for (int d = 0; d < ND; d++) if (m->d[d].playing) mark(m, DP(d, D_POS)); }
}
static const mpc_engine_t ENGINE = { create, destroy, midi, set_param, get_param, render, NULL, NULL };
const mpc_engine_t *mpc_engine(void) { return &ENGINE; }
