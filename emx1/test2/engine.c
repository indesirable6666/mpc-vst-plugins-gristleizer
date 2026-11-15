/* EMX-1 Test 2 : synthé polyphonique de test, 8 voix, sans filtre. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../wrapper/engine.h"

#define NV 8
#define SR 44100.0f

typedef struct {
    int stage;              /* 0 libre, 1 attaque, 2 tenue, 3 relâchement */
    int note;
    float vel, phase, inc, env;
    unsigned age;
} voice_t;

typedef struct {
    voice_t v[NV];
    int wave;               /* 0 saw, 1 square, 2 sine */
    float level, attack_ms, release_ms;
    unsigned clock;
} emx_t;

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void *create(const char *d) {
    (void)d;
    emx_t *e = calloc(1, sizeof *e);
    e->level = 70.0f; e->attack_ms = 5.0f; e->release_ms = 300.0f; e->wave = 0;
    return e;
}
static void destroy(void *i) { free(i); }

static void note_on(emx_t *e, int note, int vel) {
    voice_t *t = NULL;
    int k;
    /* 1. la même note rejouée reprend sa voix */
    for (k = 0; k < NV; k++) if (e->v[k].stage && e->v[k].note == note) { t = &e->v[k]; break; }
    /* 2. sinon une voix libre */
    if (!t) for (k = 0; k < NV; k++) if (!e->v[k].stage) { t = &e->v[k]; break; }
    /* 3. sinon la voix en relâchement la plus basse */
    if (!t) {
        float best = 2.0f;
        for (k = 0; k < NV; k++) if (e->v[k].stage == 3 && e->v[k].env < best) { best = e->v[k].env; t = &e->v[k]; }
    }
    /* 4. sinon la voix la plus ancienne */
    if (!t) {
        unsigned oldest = 0xFFFFFFFFu;
        for (k = 0; k < NV; k++) if (e->v[k].age < oldest) { oldest = e->v[k].age; t = &e->v[k]; }
    }
    t->note = note;
    t->vel = vel / 127.0f;
    t->inc = 440.0f * powf(2.0f, (note - 69) / 12.0f) / SR;
    if (!t->stage) { t->phase = 0.0f; t->env = 0.0f; }
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

static void set_param(void *i, const char *k, const char *v) {
    emx_t *e = i;
    if (!strcmp(k, "level")) e->level = clampf((float)atof(v), 0.0f, 100.0f);
    else if (!strcmp(k, "attack")) e->attack_ms = clampf((float)atof(v), 1.0f, 2000.0f);
    else if (!strcmp(k, "release")) e->release_ms = clampf((float)atof(v), 5.0f, 3000.0f);
    else if (!strcmp(k, "wave")) e->wave = (v[0] >= '0' && v[0] <= '2') ? v[0] - '0' : (!strcmp(v, "Square") ? 1 : (!strcmp(v, "Sine") ? 2 : 0));
}
static int get_param(void *i, const char *k, char *b, int n) {
    emx_t *e = i;
    if (!strcmp(k, "level")) return snprintf(b, n, "%.1f", e->level);
    if (!strcmp(k, "attack")) return snprintf(b, n, "%.1f", e->attack_ms);
    if (!strcmp(k, "release")) return snprintf(b, n, "%.1f", e->release_ms);
    if (!strcmp(k, "wave")) return snprintf(b, n, "%s", e->wave == 1 ? "Square" : (e->wave == 2 ? "Sine" : "Saw"));
    return 0;
}

/* correction PolyBLEP : adoucit les arêtes des formes d'onde pour limiter le repliement */
static float blep(float t, float dt) {
    if (t < dt) { t /= dt; return t + t - t * t - 1.0f; }
    if (t > 1.0f - dt) { t = (t - 1.0f) / dt; return t * t + t + t + 1.0f; }
    return 0.0f;
}
static float osc(int wave, float p, float dt) {
    if (wave == 2) return sinf(6.2831853f * p);
    if (wave == 1) {
        float p2 = p + 0.5f; if (p2 >= 1.0f) p2 -= 1.0f;
        return (p < 0.5f ? 1.0f : -1.0f) + blep(p, dt) - blep(p2, dt);
    }
    return 2.0f * p - 1.0f - blep(p, dt);
}

static void render(void *i, int16_t *o, int f) {
    emx_t *e = i;
    float att = 1.0f / (e->attack_ms * 0.001f * SR);
    float rel = 1.0f / (e->release_ms * 0.001f * SR);
    float gain = e->level * 0.01f * 0.2f;
    for (int n = 0; n < f; n++) {
        float s = 0.0f;
        for (int k = 0; k < NV; k++) {
            voice_t *v = &e->v[k];
            if (!v->stage) continue;
            if (v->stage == 1) { v->env += att; if (v->env >= 1.0f) { v->env = 1.0f; v->stage = 2; } }
            else if (v->stage == 3) { v->env -= rel; if (v->env <= 0.0f) { v->env = 0.0f; v->stage = 0; continue; } }
            s += osc(e->wave, v->phase, v->inc) * v->env * v->vel;
            v->phase += v->inc;
            if (v->phase >= 1.0f) v->phase -= 1.0f;
        }
        s = clampf(s * gain, -1.0f, 1.0f);
        int16_t q = (int16_t)(s * 32767.0f);
        o[2 * n] = q;
        o[2 * n + 1] = q;
    }
}

static const mpc_engine_t ENGINE = { create, destroy, midi, set_param, get_param, render, NULL, NULL };
const mpc_engine_t *mpc_engine(void) { return &ENGINE; }
