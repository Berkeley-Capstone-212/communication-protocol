/*
 * swarm.c - see swarm.h. Wire format and rationale: README.md.
 */
#include "swarm.h"

#include <math.h>
#include <string.h>

#ifdef SW_FLOAT
#define RSQRT sqrtf
#define RSIN sinf
#define RCOS cosf
#define RATAN2 atan2f
#define RFLOOR floorf
#define RCEIL ceilf
#else
#define RSQRT sqrt
#define RSIN sin
#define RCOS cos
#define RATAN2 atan2
#define RFLOOR floor
#define RCEIL ceil
#endif

/* compile-time limits: a negative array size stops the build */
typedef char sw_chk_log[(SW_LOG & (SW_LOG - 1)) == 0 ? 1 : -1];
typedef char sw_chk_task[SW_MAX_TASK >= 2 && SW_MAX_TASK <= 0xFFFF ? 1 : -1];
typedef char sw_chk_u8[SW_MAX_NODES <= 255 && SW_MAX_NBR <= 255 &&
                       SW_FANOUT <= 255 && SW_OP_MAX <= 255 &&
                       SW_CTX_MAX <= 255 ? 1 : -1];
typedef char sw_chk_mtu[SW_FRAME_MIN <= 0xFFFF ? 1 : -1];

#define DIRTY 1
#define FB_ROOT 1   /* link root[] holds the peer's advertised root */
#define FB_HEARD 2  /* peer has sent us a unicast, so it answers probes */
#define UNKNOWN ((sw_real)1e9)
#define U32MAX ((sw_u32)0xFFFFFFFFUL)

/* ---- little-endian codec ---------------------------------------------- */

static void put16(sw_u8 *p, unsigned v)
{
    p[0] = (sw_u8)v;
    p[1] = (sw_u8)(v >> 8);
}

static void put32(sw_u8 *p, sw_u32 v)
{
    p[0] = (sw_u8)v;
    p[1] = (sw_u8)(v >> 8);
    p[2] = (sw_u8)(v >> 16);
    p[3] = (sw_u8)(v >> 24);
}

static unsigned get16(const sw_u8 *p)
{
    return (unsigned)p[0] | (unsigned)p[1] << 8;
}

static sw_u32 get32(const sw_u8 *p)
{
    return (sw_u32)p[0] | (sw_u32)p[1] << 8 | (sw_u32)p[2] << 16 |
           (sw_u32)p[3] << 24;
}

/* two's complement decode without implementation-defined conversions */
static long sx16(unsigned v)
{
    return v & 0x8000U ? (long)v - 0x10000L : (long)v;
}

static long sx32(sw_u32 v)
{
    return v & 0x80000000UL ? -(long)(~v & 0x7FFFFFFFUL) - 1 : (long)v;
}

/* ---- BLAKE2s (RFC 7693), one-shot, 32-bit arithmetic only -------------- */

static const sw_u32 b2_iv[8] = {
    0x6A09E667UL, 0xBB67AE85UL, 0x3C6EF372UL, 0xA54FF53AUL,
    0x510E527FUL, 0x9B05688CUL, 0x1F83D9ABUL, 0x5BE0CD19UL
};

static const sw_u8 b2_sigma[10][16] = {
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
    { 14, 10, 4, 8, 9, 15, 13, 6, 1, 12, 0, 2, 11, 7, 5, 3 },
    { 11, 8, 12, 0, 5, 2, 15, 13, 10, 14, 3, 6, 7, 1, 9, 4 },
    { 7, 9, 3, 1, 13, 12, 11, 14, 2, 6, 5, 10, 4, 0, 15, 8 },
    { 9, 0, 5, 7, 2, 4, 10, 15, 14, 1, 11, 12, 6, 8, 3, 13 },
    { 2, 12, 6, 10, 0, 11, 8, 3, 4, 13, 7, 5, 15, 14, 1, 9 },
    { 12, 5, 1, 15, 14, 13, 4, 10, 0, 7, 6, 3, 9, 2, 8, 11 },
    { 13, 11, 7, 14, 12, 1, 3, 9, 5, 0, 15, 4, 8, 6, 2, 10 },
    { 6, 15, 14, 9, 11, 3, 0, 8, 12, 2, 13, 7, 1, 4, 10, 5 },
    { 10, 2, 8, 4, 7, 6, 1, 5, 15, 11, 9, 14, 3, 12, 13, 0 }
};

static const sw_u8 b2_g[8][4] = {
    { 0, 4, 8, 12 }, { 1, 5, 9, 13 }, { 2, 6, 10, 14 }, { 3, 7, 11, 15 },
    { 0, 5, 10, 15 }, { 1, 6, 11, 12 }, { 2, 7, 8, 13 }, { 3, 4, 9, 14 }
};

#define ROTR(x, n) ((sw_u32)((x) >> (n)) | (sw_u32)((x) << (32 - (n))))

static void b2_compress(sw_u32 *h, const sw_u8 *blk, sw_u32 t, int last)
{
    sw_u32 v[16], m[16];
    unsigned i, r;

    for (i = 0; i < 16; i++)
        m[i] = get32(blk + 4 * i);
    for (i = 0; i < 8; i++) {
        v[i] = h[i];
        v[i + 8] = b2_iv[i];
    }
    v[12] ^= t;
    if (last)
        v[14] = ~v[14];
    for (r = 0; r < 10; r++) {
        for (i = 0; i < 8; i++) {
            const sw_u8 *g = b2_g[i];
            sw_u32 x = m[b2_sigma[r][2 * i]], y = m[b2_sigma[r][2 * i + 1]];
            v[g[0]] = v[g[0]] + v[g[1]] + x;
            v[g[3]] = ROTR(v[g[3]] ^ v[g[0]], 16);
            v[g[2]] = v[g[2]] + v[g[3]];
            v[g[1]] = ROTR(v[g[1]] ^ v[g[2]], 12);
            v[g[0]] = v[g[0]] + v[g[1]] + y;
            v[g[3]] = ROTR(v[g[3]] ^ v[g[0]], 8);
            v[g[2]] = v[g[2]] + v[g[3]];
            v[g[1]] = ROTR(v[g[1]] ^ v[g[2]], 7);
        }
    }
    for (i = 0; i < 8; i++)
        h[i] ^= v[i] ^ v[i + 8];
}

/* in must be < 4 GiB, so the high counter word stays zero */
static void b2s(sw_u8 *out, unsigned outlen, const sw_u8 *key, unsigned klen,
                const sw_u8 *in, unsigned len)
{
    sw_u32 h[8], t = 0;
    sw_u8 blk[64];
    unsigned i;

    for (i = 0; i < 8; i++)
        h[i] = b2_iv[i];
    h[0] ^= 0x01010000UL ^ ((sw_u32)klen << 8) ^ (sw_u32)outlen;
    if (klen) {
        memset(blk, 0, 64);
        memcpy(blk, key, klen);
        t = 64;
        b2_compress(h, blk, t, len == 0);
    }
    for (; len > 64; in += 64, len -= 64) {
        t += 64;
        b2_compress(h, in, t, 0);
    }
    if (len || !klen) {
        memset(blk, 0, 64);
        if (len)
            memcpy(blk, in, len);
        t += len;
        b2_compress(h, blk, t, 1);
    }
    for (i = 0; i < outlen; i++)
        out[i] = (sw_u8)(h[i >> 2] >> (8 * (i & 3)));
}

/* ---- SE_2(3) tangent coordinates ---------------------------------------
 * A navigation state X = (R, v, p) is sent as Log(X) = (theta, nu, rho)
 * with v = J(theta) nu, p = J(theta) rho and J the left Jacobian of SO(3):
 *   J     = I + a [theta]x + b [theta]x^2,  a = (1-cos t)/t^2,
 *                                           b = (t - sin t)/t^3
 *   J^-1  = I - 1/2 [theta]x + c [theta]x^2, c = 1/t^2 - cot(t/2)/(2t)
 * Everything downstream (prediction, gating) is plain R^3 algebra. */

static void cross(const sw_real *a, const sw_real *b, sw_real *o)
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

/* o = x + a (w x x) + b (w x (w x x)) */
static void jmul(const sw_real *w, sw_real a, sw_real b, const sw_real *x,
                 sw_real *o)
{
    sw_real t[3], u[3];
    unsigned i;

    cross(w, x, t);
    cross(w, t, u);
    for (i = 0; i < 3; i++)
        o[i] = x[i] + a * t[i] + b * u[i];
}

static sw_real norm3(const sw_real *x)
{
    return RSQRT(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}

/* rebuild the decoded cache; rounding (half a wire unit) and arithmetic
 * error are folded into the sigmas so the estimate stays consistent
 * (never overconfident) */
static void ctx_cache(sw_ctx *c)
{
    sw_real th[3], rho[3], nu[3], t, t2, a, b;
    unsigned i;

    for (i = 0; i < 3; i++) {
        th[i] = (sw_real)c->th[i] * (sw_real)1e-4;
        rho[i] = (sw_real)c->rho[i] * (sw_real)1e-3;
        nu[i] = (sw_real)c->nu[i] * (sw_real)1e-2;
    }
    t = norm3(th);
    t2 = t * t;
    if (t < (sw_real)0.1) {
        a = (sw_real)0.5 - t2 / 24 + t2 * t2 / 720;
        b = (sw_real)1 / 6 - t2 / 120 + t2 * t2 / 5040;
    } else {
        a = (1 - RCOS(t)) / t2;
        b = (t - RSIN(t)) / (t2 * t);
    }
    jmul(th, a, b, rho, c->p);
    jmul(th, a, b, nu, c->v);
    c->sp = c->sig[2] == 0xFFFF ? UNKNOWN :
            (sw_real)c->sig[2] * (sw_real)0.01 + (sw_real)1e-6 * norm3(rho) +
            (sw_real)1e-3;
    c->sv = c->sig[1] == 0xFFFF ? UNKNOWN :
            (sw_real)c->sig[1] * (sw_real)0.01 + (sw_real)1e-6 * norm3(nu) +
            (sw_real)1e-2;
}

static sw_u32 age(sw_u32 stamp, sw_u32 now)
{
    return stamp >= now ? 0 : now - stamp;
}

/* constant-velocity prediction to now; returns position 1-sigma */
static sw_real predict(const sw_ctx *c, sw_u32 now, sw_real *p)
{
    sw_real dt = (sw_real)age(c->stamp, now) * (sw_real)1e-3, w;
    unsigned i;

    for (i = 0; i < 3; i++)
        p[i] = c->p[i] + c->v[i] * dt;
    w = (sw_real)SW_ACC * dt * dt / 2;
    return RSQRT(c->sp * c->sp + c->sv * dt * c->sv * dt + w * w);
}

/* directed edge a -> b of the reach graph: b within a's radio range */
static int reach(const sw_ctx *a, const sw_ctx *b, sw_u32 now)
{
    sw_real pa[3], pb[3], d[3], sa, sb, g;
    unsigned i;

    sa = predict(a, now, pa);
    sb = predict(b, now, pb);
    for (i = 0; i < 3; i++)
        d[i] = pa[i] - pb[i];
    g = (sw_real)a->range + (sw_real)SW_GATE_K * RSQRT(sa * sa + sb * sb);
    return d[0] * d[0] + d[1] * d[1] + d[2] * d[2] <= g * g;
}

/* ---- context wire ------------------------------------------------------ */

static unsigned ctx_size(const sw_ctx *c)
{
    return SW_CTX_LEN + c->op_len + c->ctx_len;
}

static sw_u8 *ctx_put(sw_u8 *w, const sw_ctx *c, unsigned hops)
{
    unsigned i;

    put16(w, c->id);
    w[2] = (sw_u8)hops;
    put32(w + 3, c->stamp);
    put16(w + 7, c->range);
    put16(w + 9, c->mtu);
    w += 11;
    for (i = 0; i < 3; i++) {
        put16(w + 2 * i, (sw_u16)c->th[i]);
        put16(w + 6 + 2 * i, (sw_u16)c->nu[i]);
        put32(w + 12 + 4 * i, (sw_u32)c->rho[i]);
        put16(w + 24 + 2 * i, c->sig[i]);
    }
    w += 30;
    *w++ = c->op_len;
    memcpy(w, c->op, c->op_len);
    w += c->op_len;
    *w++ = c->ctx_len;
    memcpy(w, c->ctx, c->ctx_len);
    return w + c->ctx_len;
}

/* returns NULL on malformed input; c->hops is the wire value */
static const sw_u8 *ctx_get(sw_ctx *c, const sw_u8 *p, const sw_u8 *e)
{
    unsigned i;
    long th, nu;
    sw_u32 rho;

    if (e - p < SW_CTX_LEN)
        return 0;
    c->id = (sw_u16)get16(p);
    c->hops = p[2];
    c->stamp = get32(p + 3);
    c->range = (sw_u16)get16(p + 7);
    c->mtu = (sw_u16)get16(p + 9);
    if (c->id == SW_BCAST || c->mtu < SW_FRAME_MIN)
        return 0;
    p += 11;
    for (i = 0; i < 3; i++) {
        /* accept exactly what an honest sender can produce */
        th = sx16(get16(p + 2 * i));
        nu = sx16(get16(p + 6 + 2 * i));
        rho = get32(p + 12 + 4 * i);
        if (th < -31416L || th > 31416L || nu < -32767L ||
            (rho > 2000000000UL && rho < 2294967296UL))
            return 0;
        c->th[i] = (sw_i16)th;
        c->nu[i] = (sw_i16)nu;
        c->rho[i] = (sw_i32)sx32(rho);
        c->sig[i] = (sw_u16)get16(p + 24 + 2 * i);
    }
    p += 30;
    c->op_len = *p++;
    if (c->op_len > SW_OP_MAX || e - p < (long)c->op_len + 1)
        return 0;
    memcpy(c->op, p, c->op_len);
    p += c->op_len;
    c->ctx_len = *p++;
    if (c->ctx_len > SW_CTX_MAX || e - p < (long)c->ctx_len)
        return 0;
    memcpy(c->ctx, p, c->ctx_len);
    ctx_cache(c);
    return p + c->ctx_len;
}

static sw_ctx *peer_find(sw_node *n, unsigned id)
{
    unsigned i;

    for (i = 0; i < n->npeer; i++)
        if (n->peer[i].id == id)
            return &n->peer[i];
    return 0;
}

/* keep the freshest copy per origin; when full, displace the farthest and
 * then oldest entry, so the local neighbourhood is never evicted by
 * far-away hearsay */
static void peer_merge(sw_node *n, const sw_ctx *c, sw_u32 now)
{
    sw_ctx *o;
    unsigned i, h = c->hops < 255 ? c->hops + 1U : 255U;

    if (c->id == n->self.id || age(c->stamp, now) > SW_TTL_MS ||
        (c->stamp > now && c->stamp - now > SW_SKEW_MS))
        return;
    o = peer_find(n, c->id);
    if (o) {
        if (c->stamp < o->stamp || (c->stamp == o->stamp && h >= o->hops))
            return;
    } else if (n->npeer < SW_MAX_NODES) {
        o = &n->peer[n->npeer++];
    } else {
        o = &n->peer[0];
        for (i = 1; i < SW_MAX_NODES; i++)
            if (n->peer[i].hops > o->hops ||
                (n->peer[i].hops == o->hops && n->peer[i].stamp < o->stamp))
                o = &n->peer[i];
        if (h > o->hops || (h == o->hops && c->stamp <= o->stamp))
            return;
    }
    *o = *c;
    o->hops = (sw_u8)h;
}

/* ---- goal tree --------------------------------------------------------- */

static void rec_put(sw_u8 *w, const sw_rec *r)
{
    w[0] = r->st;
    put16(w + 1, r->owner);
    put16(w + 3, r->cost);
    put32(w + 5, r->ver);
}

static int rec_get(sw_rec *r, const sw_u8 *p)
{
    r->st = p[0];
    r->owner = (sw_u16)get16(p + 1);
    r->cost = (sw_u16)get16(p + 3);
    r->ver = get32(p + 5);
    return r->st <= SW_DONE;
}

/* total order: done first, then version, state, lower cost, lower owner.
 * Its maximum is a join-semilattice, hence a state-based CRDT. */
static int rec_gt(const sw_rec *a, const sw_rec *b)
{
    int ad = a->st == SW_DONE, bd = b->st == SW_DONE;

    if (ad != bd)
        return ad;
    if (a->ver != b->ver)
        return a->ver > b->ver;
    if (a->st != b->st)
        return a->st > b->st;
    if (a->cost != b->cost)
        return a->cost < b->cost;
    return a->owner < b->owner;
}

/* dirty marks are closed upward, so stop at the first marked ancestor */
static void mark(sw_node *n, unsigned i)
{
    while (!(n->tflag[i] & DIRTY)) {
        n->tflag[i] |= DIRTY;
        if (i == 0)
            break;
        i = n->parent[i];
    }
}

static void set_rec(sw_node *n, unsigned leaf, const sw_rec *r, unsigned src)
{
    unsigned k = (unsigned)(n->loghead & (SW_LOG - 1));

    n->rec[leaf] = *r;
    mark(n, leaf);
    n->logleaf[k] = (sw_u16)leaf;
    n->logsrc[k] = (sw_u16)src;
    n->loghead++;
}

static void merge_rec(sw_node *n, unsigned leaf, const sw_rec *r, unsigned src)
{
    if (rec_gt(r, &n->rec[leaf]))
        set_rec(n, leaf, r, src);
}

/* children carry larger breadth-first indices than parents, so a single
 * descending pass rebuilds every dirty hash bottom-up without recursion */
static void rehash(sw_node *n)
{
    unsigned i = n->ntask, k, done;
    sw_u8 b[2 + SW_REC_LEN];

    while (i-- > 0) {
        if (!(n->tflag[i] & DIRTY))
            continue;
        if (n->cnt[i]) {
            b2s(n->hash[i], SW_HASH, 0, 0, n->hash[n->first[i]],
                n->cnt[i] * SW_HASH);
            done = SW_COMPLETE;
            for (k = 0; k < n->cnt[i]; k++)
                done &= n->tflag[n->first[i] + k];
        } else {
            put16(b, i);
            rec_put(b + 2, &n->rec[i]);
            b2s(n->hash[i], SW_HASH, 0, 0, b, sizeof b);
            done = n->rec[i].st == SW_DONE ? SW_COMPLETE : 0;
        }
        n->tflag[i] = (sw_u8)done;
    }
}

static unsigned probe_len(const sw_node *n, unsigned P)
{
    unsigned c, len = 2;

    for (c = n->first[P]; c < n->first[P] + n->cnt[P]; c++)
        len += n->cnt[c] ? SW_HASH : SW_REC_LEN;
    return len;
}

/* a probe describes node P: hash of each internal child, record of each
 * leaf child */
static sw_u8 *probe_put(const sw_node *n, unsigned P, sw_u8 *w)
{
    unsigned c;

    put16(w, P);
    w += 2;
    for (c = n->first[P]; c < n->first[P] + n->cnt[P]; c++) {
        if (n->cnt[c]) {
            memcpy(w, n->hash[c], SW_HASH);
            w += SW_HASH;
        } else {
            rec_put(w, &n->rec[c]);
            w += SW_REC_LEN;
        }
    }
    return w;
}

/* append once to a FANOUT-sized queue; overflow is dropped and found
 * again by a later descent */
static void enqueue(sw_u16 *q, sw_u8 *nq, unsigned v)
{
    unsigned k;

    for (k = 0; k < *nq; k++)
        if (q[k] == v)
            return;
    if (*nq < SW_FANOUT)
        q[(*nq)++] = (sw_u16)v;
}

/* merge the peer's view of P; queue our leaves that beat theirs, and every
 * internal child whose hash differs, to be described back */
static const sw_u8 *probe_get(sw_node *n, sw_link *l, const sw_u8 *p,
                              const sw_u8 *e)
{
    unsigned P, c;
    sw_rec r;

    if (e - p < 2)
        return 0;
    P = get16(p);
    p += 2;
    if (P >= n->ntask || !n->cnt[P])
        return 0;
    for (c = n->first[P]; c < n->first[P] + n->cnt[P]; c++) {
        if (n->cnt[c]) {
            if (e - p < SW_HASH)
                return 0;
            if (memcmp(p, n->hash[c], SW_HASH))
                enqueue(l->probe, &l->nprobe, c);
            p += SW_HASH;
            continue;
        }
        if (e - p < SW_REC_LEN || !rec_get(&r, p))
            return 0;
        p += SW_REC_LEN;
        merge_rec(n, c, &r, l->id);
        if (rec_gt(&n->rec[c], &r))
            enqueue(l->push, &l->npush, c);
    }
    return p;
}

/* fallback for links that never answer: cycle through nodes with at least
 * one leaf child, since only those carry records */
static unsigned sweep_next(const sw_node *n, unsigned i)
{
    unsigned k, c;

    for (k = 0; k < n->ntask; k++) {
        i = (i + 1) % n->ntask;
        for (c = n->first[i]; c < n->first[i] + n->cnt[i]; c++)
            if (!n->cnt[c])
                return i;
    }
    return 0;
}

static sw_link *link_get(sw_node *n, unsigned id, sw_u32 now)
{
    sw_link *l = 0;
    unsigned i;

    for (i = 0; i < n->nlink; i++)
        if (n->link[i].id == id) {
            l = &n->link[i];
            break;
        }
    if (!l) {
        if (n->nlink < SW_MAX_NBR) {
            l = &n->link[n->nlink++];
        } else {
            l = &n->link[0];
            for (i = 1; i < SW_MAX_NBR; i++)
                if (n->link[i].seen < l->seen)
                    l = &n->link[i];
        }
        memset(l, 0, sizeof *l);
        l->id = (sw_u16)id;
        /* a new neighbour gets the recent changes still in the ring */
        l->logpos = n->loghead > SW_LOG ? n->loghead - SW_LOG : 0;
    }
    l->seen = now;
    return l;
}

/* ---- API --------------------------------------------------------------- */

int sw_init(sw_node *n, sw_u16 id, sw_u16 range, sw_u16 mtu,
            const sw_u8 *cnt, unsigned ntask, const sw_u8 *key, unsigned klen)
{
    unsigned i, k, next = 1;
    sw_u8 m[SW_HASH + 3], mid[4];

    if (!n || !cnt || ntask < 2 || ntask > SW_MAX_TASK || !cnt[0] ||
        mtu < SW_FRAME_MIN || klen > SW_KEY_MAX || (klen && !key) ||
        id == SW_BCAST)
        return SW_EARG;
    memset(n, 0, sizeof *n);
    /* breadth-first: node i exists only if an earlier node claimed it */
    for (i = 0; i < ntask; i++) {
        if ((i && i >= next) || cnt[i] > SW_FANOUT || next + cnt[i] > ntask)
            return SW_EARG;
        n->cnt[i] = cnt[i];
        n->first[i] = (sw_u16)next;
        for (k = 0; k < cnt[i]; k++)
            n->parent[next + k] = (sw_u16)i;
        next += cnt[i];
        n->tflag[i] = DIRTY;
    }
    if (next != ntask)
        return SW_EARG;
    n->ntask = (sw_u16)ntask;
    if (klen)
        memcpy(n->key, key, klen);
    n->klen = (sw_u8)klen;
    /* bind tree shape and wire-shaping build options: mismatched firmware
     * is rejected as another mission instead of as malformed frames */
    b2s(m, SW_HASH, 0, 0, cnt, ntask);
    m[SW_HASH] = SW_OP_MAX;
    m[SW_HASH + 1] = SW_CTX_MAX;
    m[SW_HASH + 2] = SW_FANOUT;
    b2s(mid, 4, key, klen, m, sizeof m);
    n->mission = get32(mid);
    n->self.id = id;
    n->self.range = range;
    n->self.mtu = mtu;
    for (i = 0; i < 3; i++)
        n->self.sig[i] = 0xFFFF;
    ctx_cache(&n->self);
    rehash(n);
    return SW_OK;
}

static int isreal(const sw_real *x, unsigned k)
{
    while (k--)
        if (x[k] != x[k] || x[k] - x[k] != 0)
            return 0;
    return 1;
}

/* quantise to the wire unit, or fail if it does not fit; lim must be exact
 * in sw_real so the conversion to long cannot overflow */
static int quant(sw_real x, sw_real scale, long lim, long *o)
{
    x = RFLOOR(x * scale + (sw_real)0.5);
    if (!(x >= (sw_real)-lim && x <= (sw_real)lim)) /* also rejects NaN */
        return 0;
    *o = (long)x;
    return 1;
}

/* sigmas round up, unknown or huge saturates to 0xFFFF */
static sw_u16 qsig(sw_real s, sw_real scale)
{
    s = RCEIL(s * scale);
    return s >= (sw_real)0xFFFF ? (sw_u16)0xFFFF : (sw_u16)s;
}

int sw_set_self(sw_node *n, const sw_state *s, sw_u32 now)
{
    sw_real q[4], th[3], rho[3], nu[3], qn, vs, t, c, k;
    long a[9];
    unsigned i;

    if (!n || !s || !isreal(s->q, 4) || !isreal(s->v, 3) ||
        !isreal(s->p, 3) || !isreal(s->sig, 3) ||
        s->sig[0] < 0 || s->sig[1] < 0 || s->sig[2] < 0 ||
        s->op_len > SW_OP_MAX || s->ctx_len > SW_CTX_MAX ||
        (s->op_len && !s->op) || (s->ctx_len && !s->ctx))
        return SW_EARG;
    qn = RSQRT(s->q[0] * s->q[0] + s->q[1] * s->q[1] + s->q[2] * s->q[2] +
               s->q[3] * s->q[3]);
    if (qn < (sw_real)0.9 || qn > (sw_real)1.1)
        return SW_EARG;
    /* pick the hemisphere w >= 0 so |theta| <= pi */
    for (i = 0; i < 4; i++)
        q[i] = (s->q[0] < 0 ? -s->q[i] : s->q[i]) / qn;
    vs = norm3(q + 1);
    t = 2 * RATAN2(vs, q[0]);
    k = vs > (sw_real)1e-12 ? t / vs : 2;
    /* Quantise theta first and invert J at the quantised value: the
     * receiver's J(theta_q) rho then reproduces p to the mm, instead of
     * carrying a |p| * 1e-4 error from the rotation rounding. */
    for (i = 0; i < 3; i++) {
        if (!quant(q[i + 1] * k, (sw_real)1e4, 31416L, &a[i]))
            return SW_EARG;
        th[i] = (sw_real)a[i] * (sw_real)1e-4;
    }
    t = norm3(th);
    if (t < (sw_real)0.5)
        c = (sw_real)1 / 12 + t * t / 720 + t * t * t * t / 30240;
    else
        c = 1 / (t * t) - RCOS(t / 2) / RSIN(t / 2) / (2 * t);
    jmul(th, (sw_real)-0.5, c, s->p, rho);
    jmul(th, (sw_real)-0.5, c, s->v, nu);
    for (i = 0; i < 3; i++)
        if (!quant(nu[i], (sw_real)100, 32767L, &a[3 + i]) ||
            !quant(rho[i], (sw_real)1000, 2000000000L, &a[6 + i]))
            return SW_EARG;
    for (i = 0; i < 3; i++) {
        n->self.th[i] = (sw_i16)a[i];
        n->self.nu[i] = (sw_i16)a[3 + i];
        n->self.rho[i] = (sw_i32)a[6 + i];
    }
    n->self.sig[0] = qsig(s->sig[0], (sw_real)1e4);
    n->self.sig[1] = qsig(s->sig[1], (sw_real)100);
    n->self.sig[2] = qsig(s->sig[2], (sw_real)100);
    n->self.op_len = (sw_u8)s->op_len;
    if (s->op_len)
        memcpy(n->self.op, s->op, s->op_len);
    n->self.ctx_len = (sw_u8)s->ctx_len;
    if (s->ctx_len)
        memcpy(n->self.ctx, s->ctx, s->ctx_len);
    n->self.stamp = now;
    ctx_cache(&n->self);
    return SW_OK;
}

int sw_task(sw_node *n, unsigned leaf, unsigned st, sw_u16 cost, sw_u32 now)
{
    const sw_rec *c;
    sw_rec r;
    sw_u32 base = now;

    if (!n || leaf >= n->ntask || n->cnt[leaf] || st > SW_DONE)
        return SW_EARG;
    c = &n->rec[leaf];
    if (c->st == SW_DONE)
        return SW_EDONE;
    if (st == SW_CLAIMED) {
        if (c->st == SW_CLAIMED && c->owner != n->self.id && c->ver > now &&
            cost >= c->cost)
            return SW_EBUSY;
        if (now > U32MAX - SW_LEASE_MS)
            return SW_ETIME;
        base = now + SW_LEASE_MS;
    } else if (st == SW_OPEN &&
               (c->st != SW_CLAIMED || c->owner != n->self.id)) {
        return SW_EARG;
    }
    /* strictly above what we have seen: our action dominates it. DONE
     * wins at any version, so it may saturate. */
    if (c->ver == U32MAX) {
        if (st != SW_DONE)
            return SW_ETIME;
        r.ver = U32MAX;
    } else {
        r.ver = c->ver + 1 > base ? c->ver + 1 : base;
    }
    r.st = (sw_u8)st;
    r.owner = n->self.id;
    r.cost = cost;
    set_rec(n, leaf, &r, n->self.id);
    return SW_OK;
}

unsigned sw_tick(sw_node *n, sw_u32 now, sw_u16 *nbr, unsigned max)
{
    unsigned i, k = 0;

    if (!n || (max && !nbr))
        return 0;
    for (i = 0; i < n->npeer;)
        if (age(n->peer[i].stamp, now) > SW_TTL_MS)
            n->peer[i] = n->peer[--n->npeer];
        else
            i++;
    for (i = 0; i < n->ntask; i++)
        if (!n->cnt[i] && n->rec[i].st == SW_CLAIMED &&
            n->rec[i].owner == n->self.id &&
            (n->rec[i].ver <= now || n->rec[i].ver - now < SW_LEASE_MS / 2))
            sw_task(n, i, SW_CLAIMED, n->rec[i].cost, now);
    rehash(n);
    for (i = 0; i < n->npeer && k < max; i++)
        if (reach(&n->self, &n->peer[i], now))
            nbr[k++] = n->peer[i].id;
    return k;
}

int sw_encode(sw_node *n, sw_u16 dst, sw_u32 now, sw_u8 *buf, unsigned cap)
{
    const sw_ctx *d;
    sw_link *l = 0;
    sw_u8 *w, *end, *nrec, *nctx;
    unsigned lim, P, s0 = 0, i, k, cur, fresh;

    if (!n || !buf || dst == n->self.id)
        return SW_EARG;
    lim = cap < n->self.mtu ? cap : n->self.mtu;
    if (lim > (unsigned)INT_MAX)
        lim = (unsigned)INT_MAX;
    if (dst != SW_BCAST) {
        d = peer_find(n, dst);
        if (!d || age(d->stamp, now) > SW_TTL_MS || !reach(&n->self, d, now))
            return SW_EREACH;
        if (d->mtu < lim)
            lim = d->mtu;
    }
    if (lim < SW_FRAME_MIN)
        return SW_ESHORT;
    if (dst != SW_BCAST)
        l = link_get(n, dst, now);
    rehash(n);
    end = buf + lim - SW_MAC;
    buf[0] = SW_VERSION;
    buf[1] = 0;
    put32(buf + 2, n->mission);
    put16(buf + 6, n->self.id);
    put16(buf + 8, dst);
    memcpy(buf + 10, n->hash[0], SW_HASH);
    w = ctx_put(buf + SW_HDR_LEN, &n->self, 0);
    if (l && !((l->fb & FB_ROOT) && !memcmp(l->root, n->hash[0], SW_HASH))) {
        /* roots differ or unknown: describe every subtree it showed to
         * differ (restarting at the goal when none is pending), or sweep
         * when it has not answered recently. The first probe always fits
         * (SW_FRAME_MIN); more are added while one record still fits. */
        fresh = (l->fb & FB_HEARD) && age(l->heard, now) <= SW_TTL_MS;
        if (fresh && !l->nprobe)
            l->probe[l->nprobe++] = 0;
        for (k = 0; k < 255; k++) {
            if (fresh) {
                if (k == l->nprobe)
                    break;
                P = l->probe[k];
            } else {
                P = sweep_next(n, l->sweep);
                if (!k)
                    s0 = P;
                else if (P == s0)
                    break;
            }
            if (k && (unsigned)(end - w) < probe_len(n, P) + 4 + SW_REC_LEN)
                break;
            w = probe_put(n, P, w);
            if (!fresh)
                l->sweep = (sw_u16)P;
        }
        buf[1] = (sw_u8)k;
        if (fresh) {
            l->nprobe = (sw_u8)(l->nprobe - k);
            memmove(l->probe, l->probe + k, l->nprobe * sizeof l->probe[0]);
        }
    }
    nrec = w++;
    *nrec = 0;
    if (l) {
        for (k = 0; k < l->npush && (unsigned)(end - w) >= 3 + SW_REC_LEN;
             k++) {
            put16(w, l->push[k]);
            rec_put(w + 2, &n->rec[l->push[k]]);
            w += 2 + SW_REC_LEN;
            ++*nrec;
        }
        l->npush = (sw_u8)(l->npush - k);
        memmove(l->push, l->push + k, l->npush * sizeof l->push[0]);
        /* lagging past the ring: the Merkle probe repairs the gap */
        if (n->loghead - l->logpos > SW_LOG)
            l->logpos = n->loghead - SW_LOG;
        for (; l->logpos != n->loghead && *nrec < 255; l->logpos++) {
            k = (unsigned)(l->logpos & (SW_LOG - 1));
            if (n->logsrc[k] == dst)
                continue;
            if ((unsigned)(end - w) < 3 + SW_REC_LEN)
                break;
            put16(w, n->logleaf[k]);
            rec_put(w + 2, &n->rec[n->logleaf[k]]);
            w += 2 + SW_REC_LEN;
            ++*nrec;
        }
    }
    nctx = w++;
    *nctx = 0;
    if (l && n->npeer) {
        cur = l->ctxcur % n->npeer;
        for (k = 0; k < n->npeer && *nctx < 255; k++) {
            i = (cur + k) % n->npeer;
            if (n->peer[i].id == dst || age(n->peer[i].stamp, now) > SW_TTL_MS)
                continue;
            if ((unsigned)(end - w) < ctx_size(&n->peer[i]))
                break;
            w = ctx_put(w, &n->peer[i], n->peer[i].hops);
            ++*nctx;
        }
        l->ctxcur = (sw_u8)((cur + k) % n->npeer);
    }
    b2s(w, SW_MAC, n->key, n->klen, buf, (unsigned)(w - buf));
    return (int)(w + SW_MAC - buf);
}

int sw_decode(sw_node *n, const sw_u8 *buf, unsigned len, sw_u32 now)
{
    const sw_u8 *p, *e;
    sw_u8 mac[SW_MAC], diff = 0;
    sw_link *l;
    sw_ctx c;
    sw_rec r;
    unsigned src, dst, k, i;

    if (!n || !buf)
        return SW_EARG;
    if (len < SW_HDR_LEN + SW_CTX_LEN + 2 + SW_MAC)
        return SW_ESHORT;
    if (buf[0] != SW_VERSION)
        return SW_EVER;
    if (get32(buf + 2) != n->mission)
        return SW_EMISSION;
    src = get16(buf + 6);
    dst = get16(buf + 8);
    if (src == n->self.id || src == SW_BCAST ||
        (dst != n->self.id && dst != SW_BCAST))
        return SW_EADDR;
    e = buf + len - SW_MAC;
    b2s(mac, SW_MAC, n->key, n->klen, buf, len - SW_MAC);
    for (k = 0; k < SW_MAC; k++)
        diff |= (sw_u8)(mac[k] ^ e[k]);
    if (diff)
        return SW_EAUTH;
    if (dst == SW_BCAST && buf[1])
        return SW_EFIELD;
    p = ctx_get(&c, buf + SW_HDR_LEN, e);
    if (!p || c.id != src || c.hops)
        return SW_EFIELD;
    /* bounded age both ways: an old frame's sigma has grown enough to pass
     * any gate, so replays older than the TTL must not get that far */
    if ((c.stamp > now && c.stamp - now > SW_SKEW_MS) ||
        age(c.stamp, now) > SW_TTL_MS)
        return SW_ESTALE;
    /* inbound only along an edge src -> us of the reach graph */
    if (!reach(&c, &n->self, now))
        return SW_EREACH;
    /* every element below is checked before it is merged; merges are
     * monotone, so a frame rejected half-way leaves a valid state */
    peer_merge(n, &c, now);
    l = link_get(n, src, now);
    l->fb |= FB_ROOT;
    memcpy(l->root, buf + 10, SW_HASH);
    /* only a unicast to us proves it answers probes; a beacon must not
     * mark the link fresh and suppress the sweep */
    if (dst == n->self.id) {
        l->fb |= FB_HEARD;
        l->heard = now;
    }
    rehash(n);
    /* the probe queue persists across frames: clearing it on each arrival
     * lets beacons or unequal send rates livelock the descent */
    for (k = buf[1]; k; k--)
        if (!(p = probe_get(n, l, p, e)))
            return SW_EFIELD;
    if (p >= e)
        return SW_EFIELD;
    for (k = *p++; k; k--) {
        if (e - p < 2 + SW_REC_LEN)
            return SW_EFIELD;
        i = get16(p);
        if (i >= n->ntask || n->cnt[i] || !rec_get(&r, p + 2))
            return SW_EFIELD;
        merge_rec(n, i, &r, src);
        p += 2 + SW_REC_LEN;
    }
    if (p >= e)
        return SW_EFIELD;
    for (k = *p++; k; k--) {
        if (!(p = ctx_get(&c, p, e)))
            return SW_EFIELD;
        peer_merge(n, &c, now);
    }
    return p == e ? SW_OK : SW_EFIELD;
}
