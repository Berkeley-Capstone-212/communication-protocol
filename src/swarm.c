/*
 * swarm.c - see swarm.h. Wire format and rationale: README.md.
 */
#include "swarm.h"

#include <float.h>
#include <math.h>
#include <string.h>

#ifdef SW_FLOAT
#define RSQRT sqrtf
#define RSIN sinf
#define RCOS cosf
#define RATAN2 atan2f
#define RFLOOR floorf
#define RCEIL ceilf
#define RMAX FLT_MAX
#else
#define RSQRT sqrt
#define RSIN sin
#define RCOS cos
#define RATAN2 atan2
#define RFLOOR floor
#define RCEIL ceil
#define RMAX DBL_MAX
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

/* constant-velocity prediction to now; returns the position variance */
static sw_real predict(const sw_ctx *c, sw_u32 now, sw_real *p)
{
    sw_u32 ms = age(c->stamp, now);
    sw_real dt = (sw_real)ms * (sw_real)1e-3, w;
    unsigned i;

    for (i = 0; i < 3; i++)
        p[i] = c->p[i] + c->v[i] * dt;
    w = (sw_real)SW_ACC * dt * dt / 2;
    return c->sp * c->sp + c->sv * dt * c->sv * dt + w * w;
}

/* Directed edge a -> b of the reach graph: b within a's range, widened by
 * K sigma of the combined uncertainty. The caller predicts a once; a
 * square root is only taken beyond the nominal range. */
static int edge(const sw_real *pa, sw_real va, unsigned range,
                const sw_ctx *b, sw_u32 now)
{
    sw_real pb[3], vb, d2 = 0, r = (sw_real)range, x;
    unsigned i;

    vb = predict(b, now, pb);
    for (i = 0; i < 3; i++) {
        x = pa[i] - pb[i];
        d2 += x * x;
    }
    if (d2 <= r * r)
        return 1;
    x = RSQRT(d2) - r;
    return x * x <= (sw_real)(SW_GATE_K * SW_GATE_K) * (va + vb);
}

/* ---- id index: ids[] ascending, slot[k] is the table slot of ids[k] ----
 * Lookups are a binary search over a few cache lines of 16-bit ids; table
 * entries never move on insert, so only bytes are shifted. */

static unsigned ix_pos(const sw_u16 *ids, unsigned n, unsigned id)
{
    unsigned lo = 0, hi = n, mid;

    while (lo < hi) {
        mid = (lo + hi) >> 1;
        if (ids[mid] < id)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

static int ix_find(const sw_u16 *ids, const sw_u8 *slot, unsigned n,
                   unsigned id)
{
    unsigned k = ix_pos(ids, n, id);

    return k < n && ids[k] == id ? slot[k] : -1;
}

/* n is the entry count before the change */
static void ix_add(sw_u16 *ids, sw_u8 *slot, unsigned n, unsigned id,
                   unsigned s)
{
    unsigned k = ix_pos(ids, n, id);

    memmove(ids + k + 1, ids + k, (n - k) * sizeof *ids);
    memmove(slot + k + 1, slot + k, n - k);
    ids[k] = (sw_u16)id;
    slot[k] = (sw_u8)s;
}

static void ix_del(sw_u16 *ids, sw_u8 *slot, unsigned n, unsigned id)
{
    unsigned k = ix_pos(ids, n, id);

    memmove(ids + k, ids + k + 1, (n - k - 1) * sizeof *ids);
    memmove(slot + k, slot + k + 1, n - k - 1);
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

/* returns NULL on malformed input; c->hops is the wire value. The decoded
 * cache is left to the caller, so duplicates cost no trigonometry. */
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
    return p + c->ctx_len;
}

static sw_ctx *peer_find(sw_node *n, unsigned id)
{
    int s = ix_find(n->pid, n->pix, n->npeer, id);

    return s < 0 ? 0 : &n->peer[s];
}

/* keep the freshest copy per origin; when full, displace the farthest and
 * then oldest entry, so the local neighbourhood is never evicted by
 * far-away hearsay */
static void peer_merge(sw_node *n, const sw_ctx *c, sw_u32 now)
{
    sw_ctx *o;
    int s;
    unsigned i, w, h = c->hops < 255 ? c->hops + 1U : 255U;

    if (c->id == n->self.id || age(c->stamp, now) > SW_TTL_MS ||
        (c->stamp > now && c->stamp - now > SW_SKEW_MS))
        return;
    s = ix_find(n->pid, n->pix, n->npeer, c->id);
    if (s >= 0) {
        o = &n->peer[s];
        if (c->stamp < o->stamp || (c->stamp == o->stamp && h >= o->hops))
            return;
        if (c->stamp == o->stamp) { /* same estimate by a shorter route */
            o->hops = (sw_u8)h;
            return;
        }
    } else if (n->npeer < SW_MAX_NODES) {
        ix_add(n->pid, n->pix, n->npeer, c->id, n->npeer);
        o = &n->peer[n->npeer];
        n->npeer++;
    } else {
        w = 0;
        for (i = 1; i < SW_MAX_NODES; i++)
            if (n->peer[i].hops > n->peer[w].hops ||
                (n->peer[i].hops == n->peer[w].hops &&
                 n->peer[i].stamp < n->peer[w].stamp))
                w = i;
        o = &n->peer[w];
        if (h > o->hops || (h == o->hops && c->stamp <= o->stamp))
            return;
        ix_del(n->pid, n->pix, SW_MAX_NODES, o->id);
        ix_add(n->pid, n->pix, SW_MAX_NODES - 1, c->id, w);
    }
    *o = *c;
    o->hops = (sw_u8)h;
    ctx_cache(o);
}

/* forget slot i: the last entry moves into it, so only its index entry
 * needs repointing */
static void peer_del(sw_node *n, unsigned i)
{
    ix_del(n->pid, n->pix, n->npeer, n->peer[i].id);
    n->npeer--;
    if (i == n->npeer)
        return;
    n->peer[i] = n->peer[n->npeer];
    n->pix[ix_pos(n->pid, n->npeer, n->peer[i].id)] = (sw_u8)i;
}

/* ---- goal tree --------------------------------------------------------- */

static void rec_make(sw_u8 *r, unsigned st, unsigned owner, unsigned cost,
                     sw_u32 ver)
{
    r[0] = (sw_u8)st;
    put16(r + 1, owner);
    put16(r + 3, cost);
    put32(r + 5, ver);
}

/* total order on wire-form records: done first, then version, state, lower
 * cost, lower owner. Its maximum is a join-semilattice (state-based CRDT). */
static int rec_gt(const sw_u8 *a, const sw_u8 *b)
{
    sw_u32 va = get32(a + 5), vb = get32(b + 5);
    unsigned ca, cb;

    if ((a[0] == SW_DONE) != (b[0] == SW_DONE))
        return a[0] == SW_DONE;
    if (va != vb)
        return va > vb;
    if (a[0] != b[0])
        return a[0] > b[0];
    ca = get16(a + 3);
    cb = get16(b + 3);
    if (ca != cb)
        return ca < cb;
    return get16(a + 1) < get16(b + 1);
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

static void set_rec(sw_node *n, unsigned leaf, const sw_u8 *r, unsigned src)
{
    unsigned k = (unsigned)(n->loghead & (SW_LOG - 1));

    memcpy(n->rec[leaf], r, SW_REC_LEN);
    mark(n, leaf);
    n->logleaf[k] = (sw_u16)leaf;
    n->logsrc[k] = (sw_u16)src;
    n->loghead++;
}

static void merge_rec(sw_node *n, unsigned leaf, const sw_u8 *r, unsigned src)
{
    if (rec_gt(r, n->rec[leaf]))
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
            memcpy(b + 2, n->rec[i], SW_REC_LEN);
            b2s(n->hash[i], SW_HASH, 0, 0, b, sizeof b);
            done = n->rec[i][0] == SW_DONE ? SW_COMPLETE : 0;
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
            memcpy(w, n->rec[c], SW_REC_LEN);
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
 * internal child whose hash differs, to be described back (when the peer
 * holds a link slot) */
static const sw_u8 *probe_get(sw_node *n, sw_link *l, unsigned src,
                              const sw_u8 *p, const sw_u8 *e)
{
    unsigned P, c;

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
            if (l && memcmp(p, n->hash[c], SW_HASH))
                enqueue(l->probe, &l->nprobe, c);
            p += SW_HASH;
            continue;
        }
        if (e - p < SW_REC_LEN || p[0] > SW_DONE)
            return 0;
        merge_rec(n, c, p, src);
        if (l && rec_gt(n->rec[c], p))
            enqueue(l->push, &l->npush, c);
        p += SW_REC_LEN;
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

/* NULL when every slot is younger than SW_SLOT_MS and still in use */
static sw_link *link_get(sw_node *n, unsigned id, sw_u32 now)
{
    sw_link *l;
    int s = ix_find(n->lid, n->lix, n->nlink, id);
    unsigned i, w;
    sw_u32 pos;

    if (s >= 0) {
        l = &n->link[s];
        /* back after an outage: what it told us before is stale */
        if (age(l->seen, now) > SW_TTL_MS) {
            l->fb = 0;
            l->nprobe = 0;
            l->npush = 0;
        }
    } else {
        /* a new neighbour gets the recent changes still in the ring; a
         * rotated-in one does not, as slots may turn over every frame and
         * the Merkle descent repairs whatever it missed */
        pos = n->loghead > SW_LOG ? n->loghead - SW_LOG : 0;
        if (n->nlink < SW_MAX_NBR) {
            w = n->nlink;
            ix_add(n->lid, n->lix, w, id, w);
            n->nlink++;
        } else {
            pos = n->loghead;
            /* a slot goes to whoever needs it: first a link out of touch,
             * then one already in sync with us (it only needs the change
             * log, which a new link replays), else the oldest slot once it
             * has served its quantum (first-in-first-out rotation) */
            w = 0;
            for (i = 1; i < SW_MAX_NBR; i++)
                if (n->link[i].seen < n->link[w].seen)
                    w = i;
            if (age(n->link[w].seen, now) <= SW_TTL_MS) {
                for (w = 0; w < SW_MAX_NBR; w++)
                    if ((n->link[w].fb & FB_ROOT) &&
                        !memcmp(n->link[w].root, n->hash[0], SW_HASH))
                        break;
                if (w == SW_MAX_NBR) {
                    w = 0;
                    for (i = 1; i < SW_MAX_NBR; i++)
                        if (n->link[i].since < n->link[w].since)
                            w = i;
                    if (age(n->link[w].since, now) < SW_SLOT_MS)
                        return 0;
                }
            }
            ix_del(n->lid, n->lix, SW_MAX_NBR, n->link[w].id);
            ix_add(n->lid, n->lix, SW_MAX_NBR - 1, id, w);
        }
        l = &n->link[w];
        memset(l, 0, sizeof *l);
        l->id = (sw_u16)id;
        l->since = now;
        l->logpos = pos;
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
        mtu < SW_FRAME_MIN || (klen && !key) || id == SW_BCAST)
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
    /* any key length: derive a fixed-size key for the tag */
    b2s(n->key, SW_KEY, 0, 0, key, klen);
    /* bind tree shape and wire-shaping build options: mismatched firmware
     * is rejected as another mission instead of as malformed frames */
    b2s(m, SW_HASH, 0, 0, cnt, ntask);
    m[SW_HASH] = SW_OP_MAX;
    m[SW_HASH + 1] = SW_CTX_MAX;
    m[SW_HASH + 2] = SW_FANOUT;
    b2s(mid, 4, n->key, SW_KEY, m, sizeof m);
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

/* finite check with ordered comparisons only: NaN and +-inf fail both */
static int isreal(const sw_real *x, unsigned k)
{
    while (k--)
        if (!(x[k] >= -RMAX && x[k] <= RMAX))
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

int sw_task(sw_node *n, unsigned leaf, unsigned st, sw_u16 cost,
            sw_u32 lease, sw_u32 now)
{
    const sw_u8 *c;
    sw_u8 r[SW_REC_LEN];
    sw_u32 ver, base = now;

    if (!n || leaf >= n->ntask || n->cnt[leaf] || st > SW_DONE)
        return SW_EARG;
    c = n->rec[leaf];
    ver = get32(c + 5);
    if (c[0] == SW_DONE)
        return SW_EDONE;
    if (st == SW_CLAIMED) {
        if (!lease)
            lease = SW_LEASE_MS;
        if (c[0] == SW_CLAIMED && get16(c + 1) != n->self.id && ver > now &&
            cost >= get16(c + 3))
            return SW_EBUSY;
        if (now > U32MAX - lease)
            return SW_ETIME;
        base = now + lease;
    } else if (st == SW_OPEN &&
               (c[0] != SW_CLAIMED || get16(c + 1) != n->self.id)) {
        return SW_EARG;
    }
    /* strictly above what we have seen: our action dominates it. DONE
     * wins at any version, so it may saturate. */
    if (ver == U32MAX) {
        if (st != SW_DONE)
            return SW_ETIME;
    } else {
        ver = ver + 1 > base ? ver + 1 : base;
    }
    rec_make(r, st, n->self.id, cost, ver);
    set_rec(n, leaf, r, n->self.id);
    return SW_OK;
}

int sw_leaf(const sw_node *n, unsigned leaf, sw_rec *out)
{
    const sw_u8 *r;

    if (!n || !out || leaf >= n->ntask || n->cnt[leaf])
        return SW_EARG;
    r = n->rec[leaf];
    out->st = r[0];
    out->owner = (sw_u16)get16(r + 1);
    out->cost = (sw_u16)get16(r + 3);
    out->ver = get32(r + 5);
    return SW_OK;
}

unsigned sw_tick(sw_node *n, sw_u32 now, sw_u16 *nbr, unsigned max)
{
    const sw_u8 *r;
    const sw_ctx *c;
    sw_real ps[3], vs;
    sw_u32 ver;
    unsigned i, k = 0, off;

    if (!n || (max && !nbr))
        return 0;
    for (i = 0; i < n->npeer;)
        if (age(n->peer[i].stamp, now) > SW_TTL_MS)
            peer_del(n, i);
        else
            i++;
    for (i = 0; i < n->ntask; i++) {
        r = n->rec[i];
        ver = get32(r + 5);
        if (!n->cnt[i] && r[0] == SW_CLAIMED && get16(r + 1) == n->self.id &&
            (ver <= now || ver - now < SW_LEASE_MS / 2))
            sw_task(n, i, SW_CLAIMED, (sw_u16)get16(r + 3), 0, now);
    }
    rehash(n);
    /* id order from a start that moves pseudo-randomly with time, so with
     * more neighbours than link slots none is starved of a slot; our own
     * position is predicted once */
    off = (unsigned)((sw_u32)(now * 2654435761UL) >> 16);
    vs = predict(&n->self, now, ps);
    for (i = 0; i < n->npeer && k < max; i++) {
        c = &n->peer[n->pix[(off + i) % n->npeer]];
        if (edge(ps, vs, n->self.range, c, now))
            nbr[k++] = c->id;
    }
    return k;
}

int sw_encode(sw_node *n, sw_u16 dst, sw_u32 now, sw_u8 *buf, unsigned cap)
{
    const sw_ctx *d;
    sw_link *l = 0;
    sw_u8 *w, *end, *nrec, *nctx;
    sw_real ps[3], vs;
    unsigned lim, P, s0 = 0, i, k, cur, fresh;

    if (!n || !buf || dst == n->self.id)
        return SW_EARG;
    lim = cap < n->self.mtu ? cap : n->self.mtu;
    if (lim > (unsigned)INT_MAX)
        lim = (unsigned)INT_MAX;
    if (dst != SW_BCAST) {
        d = peer_find(n, dst);
        if (!d || age(d->stamp, now) > SW_TTL_MS)
            return SW_EREACH;
        vs = predict(&n->self, now, ps);
        if (!edge(ps, vs, n->self.range, d, now))
            return SW_EREACH;
        if (d->mtu < lim)
            lim = d->mtu;
    }
    if (lim < SW_FRAME_MIN)
        return SW_ESHORT;
    if (dst != SW_BCAST && !(l = link_get(n, dst, now)))
        return SW_ESLOT;
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
            memcpy(w + 2, n->rec[l->push[k]], SW_REC_LEN);
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
            memcpy(w + 2, n->rec[n->logleaf[k]], SW_REC_LEN);
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
    b2s(w, SW_MAC, n->key, SW_KEY, buf, (unsigned)(w - buf));
    return (int)(w + SW_MAC - buf);
}

int sw_decode(sw_node *n, const sw_u8 *buf, unsigned len, sw_u32 now)
{
    const sw_u8 *p, *e;
    sw_u8 mac[SW_MAC], diff = 0;
    sw_link *l;
    sw_ctx c;
    sw_real pc[3], vc;
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
    b2s(mac, SW_MAC, n->key, SW_KEY, buf, len - SW_MAC);
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
    ctx_cache(&c);
    vc = predict(&c, now, pc);
    if (!edge(pc, vc, c.range, &n->self, now))
        return SW_EREACH;
    /* every element below is checked before it is merged; merges are
     * monotone, so a frame rejected half-way leaves a valid state */
    peer_merge(n, &c, now);
    /* without a free slot the frame is still merged, only no feedback is
     * kept for the sender */
    l = link_get(n, src, now);
    if (l) {
        l->fb |= FB_ROOT;
        memcpy(l->root, buf + 10, SW_HASH);
        /* only a unicast to us proves it answers probes; a beacon must not
         * mark the link fresh and suppress the sweep */
        if (dst == n->self.id) {
            l->fb |= FB_HEARD;
            l->heard = now;
        }
    }
    rehash(n);
    /* the probe queue persists across frames: clearing it on each arrival
     * lets beacons or unequal send rates livelock the descent */
    for (k = buf[1]; k; k--)
        if (!(p = probe_get(n, l, src, p, e)))
            return SW_EFIELD;
    if (p >= e)
        return SW_EFIELD;
    for (k = *p++; k; k--) {
        if (e - p < 2 + SW_REC_LEN)
            return SW_EFIELD;
        i = get16(p);
        if (i >= n->ntask || n->cnt[i] || p[2] > SW_DONE)
            return SW_EFIELD;
        merge_rec(n, i, p + 2, src);
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
