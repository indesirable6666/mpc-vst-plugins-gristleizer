/* EMX-1 Test 1 : chenillard de 16 pas, sans son. Le moteur avance seul et prévient l'écran. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../wrapper/engine.h"

typedef struct { int step, running, tempo, changed; double acc; } emx_t;

static void *create(const char *d) {
    (void)d;
    emx_t *e = calloc(1, sizeof *e);
    e->tempo = 120; e->running = 1; e->changed = 1;
    return e;
}
static void destroy(void *i) { free(i); }
static void midi(void *i, const uint8_t *m, int n) { (void)i; (void)m; (void)n; }
static void set_param(void *i, const char *k, const char *v) {
    emx_t *e = i;
    if (!strcmp(k, "tempo")) {
        int t = atoi(v);
        e->tempo = t < 60 ? 60 : (t > 200 ? 200 : t);
    } else if (!strcmp(k, "run")) {
        e->running = !strcmp(v, "running");
    }
}
static int get_param(void *i, const char *k, char *b, int n) {
    emx_t *e = i;
    if (!strcmp(k, "tempo")) return snprintf(b, n, "%d.0", e->tempo);
    if (!strcmp(k, "run")) return snprintf(b, n, "%s", e->running ? "running" : "paused");
    if (!strcmp(k, "step")) return snprintf(b, n, "s%02d", e->step + 1);
    return 0;
}
static void render(void *i, int16_t *o, int f) {
    emx_t *e = i;
    memset(o, 0, (size_t)f * 4);
    if (!e->running) return;
    double spf = 44100.0 * 60.0 / (e->tempo * 4.0);   /* images audio par double-croche */
    e->acc += f;
    while (e->acc >= spf) {
        e->acc -= spf;
        e->step = (e->step + 1) & 15;
        e->changed = 1;
    }
}
int mpc_engine_changed(void *inst, char *key, int len) {
    emx_t *e = inst;
    if (!e->changed) return 0;
    e->changed = 0;
    snprintf(key, len, "%s", "step");
    return 1;
}
static const mpc_engine_t ENGINE = { create, destroy, midi, set_param, get_param, render, NULL };
const mpc_engine_t *mpc_engine(void) { return &ENGINE; }
