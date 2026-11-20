/* ---------- décodeurs en streaming : WAV (tous formats PCM/float) et MP3 (minimp3) ---------- */
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#if defined(__has_include)
#if __has_include("minimp3.h")
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
#define HAVE_MP3 1
#endif
#endif
#ifndef HAVE_MP3
#define HAVE_MP3 0
#endif

typedef struct {
    int fd, kind;                       /* 1 = WAV, 2 = MP3 */
    int sr, ch, total, cur;             /* total et cur en trames */
    /* WAV */
    int bits, isfloat, align; off_t data0; int64_t datalen;
    /* MP3 */
    uint32_t *foff; int nf, spf;
    off_t fpos; unsigned char *mbuf; int mlen, moff, meof, skip;
    int16_t *pcm; int pn, po;
#if HAVE_MP3
    mp3dec_t mp3;
#endif
} dec_t;

static const char *path_ext(const char *p) { const char *d = strrchr(p, '.'); return d ? d + 1 : ""; }
static int is_audio_name(const char *n) {
    const char *e = path_ext(n);
    if (!strcasecmp(e, "wav")) return 1;
#if HAVE_MP3
    if (!strcasecmp(e, "mp3")) return 2;
#endif
    return 0;
}
static uint32_t rd32(const unsigned char *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static int rd16(const unsigned char *p) { return p[0] | (p[1] << 8); }

static int wav_open(dec_t *d) {
    unsigned char h[12], c[8];
    if (pread(d->fd, h, 12, 0) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) return -1;
    off_t o = 12; int have_fmt = 0;
    for (int n = 0; n < 64; n++) {
        if (pread(d->fd, c, 8, o) != 8) return -1;
        uint32_t sz = rd32(c + 4);
        if (!memcmp(c, "fmt ", 4) && sz >= 16) {
            unsigned char f[40]; memset(f, 0, sizeof f);
            if (pread(d->fd, f, sz < 40 ? sz : 40, o + 8) < 16) return -1;
            int tag = rd16(f);
            if (tag == 0xFFFE && sz >= 26) tag = rd16(f + 24);
            d->ch = rd16(f + 2); d->sr = (int)rd32(f + 4); d->bits = rd16(f + 14);
            d->isfloat = tag == 3;
            if ((tag != 1 && tag != 3) || d->ch < 1 || d->ch > 8 || d->sr < 8000 || d->sr > 192000) return -1;
            if (!d->isfloat && d->bits != 8 && d->bits != 16 && d->bits != 24 && d->bits != 32) return -1;
            if (d->isfloat && d->bits != 32) return -1;
            d->align = d->ch * (d->bits / 8); have_fmt = 1;
        } else if (!memcmp(c, "data", 4) && have_fmt) {
            struct stat st; fstat(d->fd, &st);
            d->data0 = o + 8; d->datalen = sz;
            if (d->data0 + d->datalen > st.st_size || sz == 0xFFFFFFFFu) d->datalen = st.st_size - d->data0;
            d->total = (int)(d->datalen / d->align);
            return d->total > 0 ? 0 : -1;
        }
        o += 8 + sz + (sz & 1);
    }
    return -1;
}
static int wav_read(dec_t *d, int16_t *dst, int max) {
    unsigned char buf[8192 * 4];
    int per = (int)(sizeof buf) / d->align; if (per > max) per = max;
    if (d->cur >= d->total) return 0;
    if (per > d->total - d->cur) per = d->total - d->cur;
    ssize_t got = pread(d->fd, buf, (size_t)per * (size_t)d->align, d->data0 + (off_t)d->cur * d->align);
    if (got <= 0) return 0;
    int n = (int)(got / d->align), B = d->bits / 8;
    for (int i = 0; i < n; i++) {
        float s[2];
        for (int c = 0; c < 2; c++) {
            const unsigned char *p = buf + (size_t)i * (size_t)d->align + (size_t)(c < d->ch ? c : 0) * (size_t)B;
            float v;
            if (d->isfloat) { float f; memcpy(&f, p, 4); v = f * 32767.0f; }
            else if (B == 1) v = (float)((int)p[0] - 128) * 256.0f;
            else if (B == 2) v = (float)(int16_t)(p[0] | (p[1] << 8));
            else if (B == 3) v = (float)((int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24) >> 16);
            else v = (float)((int32_t)rd32(p) >> 16);
            s[c] = v > 32767.0f ? 32767.0f : (v < -32768.0f ? -32768.0f : v);
        }
        dst[2 * i] = (int16_t)s[0]; dst[2 * i + 1] = (int16_t)s[1];
    }
    d->cur += n;
    return n;
}

#if HAVE_MP3
static const int BR1[16] = {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0};
static const int BR2[16] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0};
static const int SR1[4] = {44100, 48000, 32000, 0};
/* taille d'une trame MP3 layer III d'après son en-tête ; 0 si l'en-tête n'est pas valide */
static int mp3_hdr(const unsigned char *h, int *sr, int *ch, int *spf) {
    if (h[0] != 0xFF || (h[1] & 0xE0) != 0xE0) return 0;
    int ver = (h[1] >> 3) & 3, layer = (h[1] >> 1) & 3, bri = h[2] >> 4, sri = (h[2] >> 2) & 3, pad = (h[2] >> 1) & 1;
    if (ver == 1 || layer != 1 || bri == 0 || bri == 15 || sri == 3) return 0;
    int mpeg1 = ver == 3, s = SR1[sri] >> (ver == 3 ? 0 : (ver == 2 ? 1 : 2));
    int br = (mpeg1 ? BR1[bri] : BR2[bri]) * 1000;
    int fb = (mpeg1 ? 144 : 72) * br / s + pad;
    if (fb < 24) return 0;
    if (sr) *sr = s;
    if (ch) *ch = ((h[3] >> 6) == 3) ? 1 : 2;
    if (spf) *spf = mpeg1 ? 1152 : 576;
    return fb;
}
static int mp3_index(dec_t *d) {
    struct stat st; fstat(d->fd, &st);
    off_t size = st.st_size, pos = 0;
    unsigned char id[10];
    if (pread(d->fd, id, 10, 0) == 10 && !memcmp(id, "ID3", 3)) pos = 10 + ((id[6] << 21) | (id[7] << 14) | (id[8] << 7) | id[9]);
    int cap = 4096; d->foff = malloc((size_t)cap * 4); if (!d->foff) return -1;
    enum { CH = 1 << 18 };
    unsigned char *buf = malloc(CH + 8); if (!buf) return -1;
    off_t base = pos; int bl = 0, bi = 0, sr0 = 0, ch0 = 0, spf0 = 0;
    for (;;) {
        if (bl - bi < 4096 && base + bl < size) {         /* recharge */
            memmove(buf, buf + bi, (size_t)(bl - bi)); base += bi; bl -= bi; bi = 0;
            ssize_t g = pread(d->fd, buf + bl, (size_t)(CH - bl), base + bl);
            if (g > 0) bl += (int)g;
        }
        if (bl - bi < 4) break;
        int sr, ch, spf, fb = mp3_hdr(buf + bi, &sr, &ch, &spf);
        int ok = fb > 0 && (!sr0 || (sr == sr0 && spf == spf0));
        if (ok && bi + fb + 4 <= bl) ok = mp3_hdr(buf + bi + fb, NULL, NULL, NULL) > 0;
        else if (ok && base + bi + fb < size) ok = 0;
        if (ok) {
            if (!sr0) { sr0 = sr; ch0 = ch; spf0 = spf; }
            if (d->nf >= cap) { if (cap >= 262144) break; cap *= 2; uint32_t *n = realloc(d->foff, (size_t)cap * 4); if (!n) break; d->foff = n; }
            d->foff[d->nf++] = (uint32_t)(base + bi); bi += fb;
        } else bi++;
        if (base + bi >= size) break;
    }
    free(buf);
    if (d->nf < 4 || !sr0) return -1;
    d->sr = sr0; d->ch = ch0; d->spf = spf0; d->total = d->nf * spf0;
    return 0;
}
static void mp3_seek(dec_t *d, int frame) {
    int f = frame / d->spf, start = f > 6 ? f - 6 : 0;
    if (start >= d->nf) start = d->nf - 1;
    mp3dec_init(&d->mp3);
    d->fpos = d->foff[start]; d->mlen = d->moff = d->meof = 0; d->pn = d->po = 0;
    d->skip = frame - start * d->spf;      /* trames à jeter : amorçage + décalage dans la trame */
    d->cur = frame;
}
static int mp3_read(dec_t *d, int16_t *dst, int max) {
    int out = 0;
    while (out < max) {
        if (d->po < d->pn) {
            int n = d->pn - d->po; if (n > max - out) n = max - out;
            memcpy(dst + 2 * out, d->pcm + 2 * d->po, (size_t)n * 4); d->po += n; out += n; continue;
        }
        if (d->mlen - d->moff < 8192 && !d->meof) {
            memmove(d->mbuf, d->mbuf + d->moff, (size_t)(d->mlen - d->moff)); d->mlen -= d->moff; d->moff = 0;
            ssize_t g = pread(d->fd, d->mbuf + d->mlen, (size_t)(65536 - d->mlen), d->fpos);
            if (g > 0) { d->mlen += (int)g; d->fpos += g; } else d->meof = 1;
        }
        if (d->mlen - d->moff < 4) break;
        mp3dec_frame_info_t info; memset(&info, 0, sizeof info);
        int16_t tmp[MINIMP3_MAX_SAMPLES_PER_FRAME];
        int n = mp3dec_decode_frame(&d->mp3, d->mbuf + d->moff, d->mlen - d->moff, tmp, &info);
        if (info.frame_bytes <= 0) { if (d->meof) break; d->moff++; continue; }
        d->moff += info.frame_bytes;
        if (n <= 0) continue;
        int ch = info.channels > 0 ? info.channels : 1, from = 0;
        if (d->skip > 0) { from = d->skip < n ? d->skip : n; d->skip -= from; }
        int m = n - from;
        for (int i = 0; i < m; i++) {
            int16_t l = tmp[(from + i) * ch], r = ch > 1 ? tmp[(from + i) * ch + 1] : l;
            d->pcm[2 * i] = l; d->pcm[2 * i + 1] = r;
        }
        d->pn = m; d->po = 0;
    }
    d->cur += out;
    return out;
}
#endif

static void dec_close(dec_t *d) {
    if (!d) return;
    if (d->fd >= 0) close(d->fd);
    free(d->foff); free(d->mbuf); free(d->pcm); free(d);
}
static dec_t *dec_open(const char *path) {
    dec_t *d = calloc(1, sizeof *d);
    if (!d) return NULL;
    d->fd = open(path, O_RDONLY);
    if (d->fd < 0) { free(d); return NULL; }
    int k = is_audio_name(path), rc = -1;
    if (k == 1) { d->kind = 1; rc = wav_open(d); }
#if HAVE_MP3
    else if (k == 2) {
        d->kind = 2; d->mbuf = malloc(65536 + 16); d->pcm = malloc(sizeof(int16_t) * MINIMP3_MAX_SAMPLES_PER_FRAME);
        if (d->mbuf && d->pcm && mp3_index(d) == 0) { rc = 0; mp3_seek(d, 0); }
    }
#endif
    if (rc != 0) { dec_close(d); return NULL; }
    return d;
}
static void dec_seek(dec_t *d, int frame) {
    if (frame < 0) frame = 0;
    if (frame > d->total) frame = d->total;
    if (d->kind == 1) d->cur = frame;
#if HAVE_MP3
    else mp3_seek(d, frame);
#endif
}
static int dec_read(dec_t *d, int16_t *dst, int max) {
    if (d->kind == 1) return wav_read(d, dst, max);
#if HAVE_MP3
    return mp3_read(d, dst, max);
#else
    return 0;
#endif
}
