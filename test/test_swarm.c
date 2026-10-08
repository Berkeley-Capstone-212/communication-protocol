/*
 * test_swarm.c - unit, fuzz and swarm simulation tests. Includes the source
 * directly to reach internal functions.
 */
#include "../src/swarm.c"

#include <stdio.h>
#include <stdlib.h>

static int fails;

#define CHECK(c)                                                         \
    do {                                                                 \
        if (!(c)) {                                                      \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);          \
            fails++;                                                     \
        }                                                                \
    } while (0)

static sw_u32 rng = 12345;

static sw_u32 rnd(void)
{
    rng = rng * 1103515245UL + 12345UL;
    return (rng >> 8) & 0xFFFFFFUL;
}

static double frnd(double lo, double hi)
{
    sw_u32 r = rnd();

    return lo + (hi - lo) * (double)r / (double)0xFFFFFF;
}

static int hexeq(const sw_u8 *b, const char *hex)
{
    unsigned i, v;

    for (i = 0; hex[2 * i]; i++) {
        sscanf(hex + 2 * i, "%2x", &v);
        if (b[i] != v)
            return 0;
    }
    return 1;
}

static void test_blake2s(void)
{
    sw_u8 k[32], d[129], o[32];
    unsigned i;

    for (i = 0; i < 129; i++)
        d[i] = (sw_u8)i;
    for (i = 0; i < 32; i++)
        k[i] = (sw_u8)i;
    b2s(o, 32, 0, 0, (const sw_u8 *)"abc", 3);
    CHECK(hexeq(o, "508c5e8c327c14e2e1a72ba34eeb452f37458b209ed63a294d999b4c86675982"));
    b2s(o, 32, 0, 0, d, 0);
    CHECK(hexeq(o, "69217a3079908094e11121d042354a7c1f55b6482ca1a51e1b250dfd1ed0eef9"));
    b2s(o, 8, (const sw_u8 *)"k", 1, (const sw_u8 *)"abc", 3);
    CHECK(hexeq(o, "dc16049ccdf930f7"));
    b2s(o, 8, k, 32, d, 0);
    CHECK(hexeq(o, "3a23e83d1d585785"));
    b2s(o, 8, k, 32, d, 64);
    CHECK(hexeq(o, "8013e78702c80e7b"));
    b2s(o, 8, k, 32, d, 65);
    CHECK(hexeq(o, "322e9aa4f43f1ee0"));
    b2s(o, 8, 0, 0, d, 128);
    CHECK(hexeq(o, "48f021896f099170"));
    b2s(o, 4, 0, 0, d, 129);
    CHECK(hexeq(o, "0380107f"));
}

static void rand_rec(sw_u8 *r)
{
    rec_make(r, rnd() % 3, rnd() % 3, rnd() % 3, rnd() % 4);
}

static const sw_u8 *rmax(const sw_u8 *a, const sw_u8 *b)
{
    return rec_gt(b, a) ? b : a;
}

static int req(const sw_u8 *a, const sw_u8 *b)
{
    return !memcmp(a, b, SW_REC_LEN);
}

/* merge must be a join: total, antisymmetric, associative, commutative */
static void test_lattice(void)
{
    sw_u8 a[SW_REC_LEN], b[SW_REC_LEN], c[SW_REC_LEN];
    unsigned i;

    for (i = 0; i < 20000; i++) {
        rand_rec(a);
        rand_rec(b);
        rand_rec(c);
        CHECK(!(rec_gt(a, b) && rec_gt(b, a)));
        CHECK(req(a, b) || rec_gt(a, b) || rec_gt(b, a));
        CHECK(req(rmax(a, b), rmax(b, a)));
        CHECK(req(rmax(a, rmax(b, c)), rmax(rmax(a, b), c)));
        CHECK(req(rmax(a, a), a));
    }
}

static const sw_u8 tree[] = {
    4, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
#define NTASK ((unsigned)sizeof tree)
static const sw_u8 deep[] = { /* 1-2-4-8-64 */
    2, 2, 2, 2, 2, 2, 2, 8, 8, 8, 8, 8, 8, 8, 8,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
static const sw_u8 key[] = "mission-key-0001";

static void state_at(sw_state *s, const double *p, const double *v,
                     double yaw)
{
    double c = cos(yaw / 2), sn = sin(yaw / 2);
    unsigned i;

    memset(s, 0, sizeof *s);
    s->q[0] = (sw_real)c;
    s->q[3] = (sw_real)sn;
    for (i = 0; i < 3; i++) {
        s->p[i] = (sw_real)p[i];
        s->v[i] = (sw_real)v[i];
    }
    s->sig[0] = (sw_real)0.01;
    s->sig[1] = (sw_real)0.1;
    s->sig[2] = (sw_real)2.0;
}

/* Log on the sender, Exp on the receiver must reproduce p and v */
static void test_lie(void)
{
    static sw_node n;
    sw_state s;
    double ax[3], an, ang, worst = 0, e, x, zero = 0;
    unsigned i, j;

    CHECK(sw_init(&n, 1, 500, 255, tree, NTASK, key, sizeof key) == SW_OK);
    memset(&s, 0, sizeof s);
    for (i = 0; i < 5000; i++) {
        for (j = 0; j < 3; j++) {
            ax[j] = frnd(-1, 1);
            x = frnd(-50000, 50000);
            s.p[j] = (sw_real)x;
            x = frnd(-60, 60);
            s.v[j] = (sw_real)x;
            s.sig[j] = 0;
        }
        an = sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]) + 1e-9;
        /* cover tiny, generic and near-pi rotations */
        ang = i % 3 == 0 ? frnd(0, 1e-6) : i % 3 == 1 ? frnd(0, 3.1415)
                                                      : 3.14159265 - frnd(0, 1e-5);
        if (i % 7 == 0)
            ang = -ang; /* w < 0 hemisphere */
        x = cos(ang / 2);
        s.q[0] = (sw_real)x;
        x = sin(ang / 2);
        for (j = 0; j < 3; j++)
            s.q[j + 1] = (sw_real)(x * ax[j] / an);
        CHECK(sw_set_self(&n, &s, 1000) == SW_OK);
        for (j = 0; j < 3; j++) {
            e = fabs((double)(n.self.p[j] - s.p[j]));
            CHECK(e <= (double)n.self.sp);
            if (e > worst)
                worst = e;
            CHECK(fabs((double)(n.self.v[j] - s.v[j])) <= (double)n.self.sv);
        }
    }
    printf("lie: worst position round-trip error %.4f m at 50 km\n", worst);
    CHECK(worst < (sizeof(sw_real) == sizeof(double) ? 0.005 : 0.05));
    s.p[0] = 1e7; /* 10 000 km: does not fit the wire */
    CHECK(sw_set_self(&n, &s, 1000) == SW_EARG);
    /* finite but overflowing inside J^-1: must fail cleanly, not UB */
    s.p[0] = (sw_real)(sizeof(sw_real) == sizeof(double) ? 1e300 : 3e38);
    CHECK(sw_set_self(&n, &s, 1000) == SW_EARG);
    s.p[0] = 0;
    s.v[1] = (sw_real)(zero / zero);
    CHECK(sw_set_self(&n, &s, 1000) == SW_EARG);
}

static void test_init(void)
{
    static sw_node n;
    static const sw_u8 orphan[] = { 1, 0, 1, 0 };   /* node 2 has no parent */
    static const sw_u8 extra[] = { 2, 0 };          /* child 2 missing */
    static const sw_u8 leafroot[] = { 0, 0 };
    sw_u8 wide[SW_FANOUT + 2];

    CHECK(sw_init(&n, 1, 500, 255, orphan, sizeof orphan, 0, 0) == SW_EARG);
    CHECK(sw_init(&n, 1, 500, 255, extra, sizeof extra, 0, 0) == SW_EARG);
    CHECK(sw_init(&n, 1, 500, 255, leafroot, sizeof leafroot, 0, 0) == SW_EARG);
    memset(wide, 0, sizeof wide);
    wide[0] = SW_FANOUT + 1;
    CHECK(sw_init(&n, 1, 500, 255, wide, sizeof wide, 0, 0) == SW_EARG);
    CHECK(sw_init(&n, 1, 500, SW_FRAME_MIN - 1, tree, NTASK, 0, 0) == SW_EARG);
    CHECK(sw_init(&n, SW_BCAST, 500, 255, tree, NTASK, 0, 0) == SW_EARG);
    CHECK(sw_init(&n, 1, 500, 255, tree, NTASK, 0, 1) == SW_EARG);
    CHECK(sw_init(&n, 1, 500, 255, tree, NTASK, 0, 0) == SW_OK);
    CHECK(!SW_GOAL_DONE(&n));
}

static sw_rec leafv(const sw_node *n, unsigned leaf)
{
    sw_rec r;

    memset(&r, 0, sizeof r);
    CHECK(sw_leaf(n, leaf, &r) == SW_OK);
    return r;
}

/* both id indexes sorted, unique and pointing at the right slots */
static int index_ok(const sw_node *n)
{
    unsigned k;

    for (k = 0; k < n->npeer; k++)
        if (n->pix[k] >= n->npeer || n->peer[n->pix[k]].id != n->pid[k] ||
            (k && n->pid[k - 1] >= n->pid[k]))
            return 0;
    for (k = 0; k < n->nlink; k++)
        if (n->lix[k] >= n->nlink || n->link[n->lix[k]].id != n->lid[k] ||
            (k && n->lid[k - 1] >= n->lid[k]))
            return 0;
    return 1;
}

/* random arrivals, refreshes, evictions and expiry against the index */
static void test_index(void)
{
    static sw_node a;
    sw_ctx c;
    sw_u32 now = 20000;
    unsigned i, bad = 0;

    CHECK(sw_init(&a, 1, 500, 255, tree, NTASK, key, sizeof key) == SW_OK);
    c = a.self;
    for (i = 0; i < 50000; i++) {
        c.id = (sw_u16)(2 + rnd() % 200);
        c.hops = (sw_u8)(rnd() % 4);
        c.stamp = now - rnd() % SW_TTL_MS;
        peer_merge(&a, &c, now);
        link_get(&a, 2 + rnd() % 200, now);
        if (i % 97 == 0)
            sw_tick(&a, now, 0, 0);
        now += rnd() % 50;
        bad += !index_ok(&a);
    }
    CHECK(bad == 0);
    CHECK(a.npeer == SW_MAX_NODES && a.nlink == SW_MAX_NBR);
    now += SW_TTL_MS + 1;
    sw_tick(&a, now, 0, 0);
    CHECK(a.npeer == 0 && index_ok(&a));
}

static void pair_t(sw_node *a, sw_node *b, double dist, const sw_u8 *t,
                   unsigned nt)
{
    static const double v0[3] = { 0, 0, 0 };
    double pa[3] = { 0, 0, 50 }, pb[3] = { 0, 0, 50 };
    sw_state s;

    pb[0] = dist;
    CHECK(sw_init(a, 1, 500, 255, t, nt, key, sizeof key) == SW_OK);
    CHECK(sw_init(b, 2, 300, 255, t, nt, key, sizeof key) == SW_OK);
    state_at(&s, pa, v0, 0);
    sw_set_self(a, &s, 1000);
    state_at(&s, pb, v0, 1);
    sw_set_self(b, &s, 1000);
}

static void pair(sw_node *a, sw_node *b, double dist)
{
    pair_t(a, b, dist, tree, NTASK);
}

/* tag, mission, addressing, directed reach and beacon discovery */
static void test_frames(void)
{
    static sw_node a, b, x;
    static const sw_u8 other[] = { 2, 0, 0 };
    sw_u8 f[255];
    int len;

    pair(&a, &b, 400); /* a reaches b (500 m), b cannot reach a (300 m) */
    CHECK(sw_encode(&a, 2, 1000, f, sizeof f) == SW_EREACH); /* unknown */
    len = sw_encode(&a, SW_BCAST, 1000, f, sizeof f);
    CHECK(len > 0 && sw_decode(&b, f, (unsigned)len, 1000) == SW_OK);
    len = sw_encode(&b, SW_BCAST, 1000, f, sizeof f);
    CHECK(len > 0 && sw_decode(&a, f, (unsigned)len, 1000) == SW_EREACH);
    CHECK(sw_encode(&b, 1, 1000, f, sizeof f) == SW_EREACH);

    pair(&a, &b, 200); /* both directions */
    len = sw_encode(&b, SW_BCAST, 1000, f, sizeof f);
    CHECK(sw_decode(&a, f, (unsigned)len, 1000) == SW_OK);
    CHECK(sw_encode(&a, 2, 1000, f, 100) == SW_ESHORT);
    CHECK(sw_task(&a, 20, SW_CLAIMED, 7, 0, 1000) == SW_OK);
    len = sw_encode(&a, 2, 1000, f, sizeof f);
    CHECK(len > 0 && len <= 255);
    CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_OK);
    CHECK(leafv(&b, 20).st == SW_CLAIMED && leafv(&b, 20).owner == 1);
    CHECK(sw_task(&b, 20, SW_CLAIMED, 9, 0, 1100) == SW_EBUSY);
    CHECK(sw_task(&b, 20, SW_CLAIMED, 3, 0, 1100) == SW_OK); /* outbid */
    CHECK(sw_task(&b, 20, SW_OPEN, 0, 0, 1200) == SW_OK);
    CHECK(sw_task(&b, 20, SW_OPEN, 0, 0, 1200) == SW_EARG);
    CHECK(sw_task(&b, 4, SW_CLAIMED, 1, 0, 1200) == SW_EARG); /* not a leaf */
    CHECK(sw_leaf(&b, 4, 0) == SW_EARG);
    CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_OK); /* replay: harmless */
    CHECK(leafv(&b, 20).st == SW_OPEN);
    CHECK(sw_task(&b, 22, SW_CLAIMED, 1, 300000, 1200) == SW_OK);
    CHECK(leafv(&b, 22).ver == 301200); /* explicit lease */

    f[30] ^= 1;
    CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_EAUTH);
    f[30] ^= 1;
    CHECK(sw_decode(&b, f, (unsigned)len - 1, 1000) == SW_EAUTH);
    CHECK(sw_decode(&a, f, (unsigned)len, 1000) == SW_EADDR); /* own frame */
    CHECK(sw_decode(&b, f, 20, 1000) == SW_ESHORT);
    CHECK(sw_init(&x, 3, 500, 255, other, sizeof other, key, sizeof key) == SW_OK);
    CHECK(sw_decode(&x, f, (unsigned)len, 1000) == SW_EMISSION);
    CHECK(sw_init(&x, 2, 500, 255, tree, NTASK, (const sw_u8 *)"x", 1) == SW_OK);
    CHECK(sw_decode(&x, f, (unsigned)len, 1000) == SW_EMISSION);
    a.self.stamp = 900000;
    len = sw_encode(&a, 2, 1000, f, sizeof f);
    CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_ESTALE);

    pair(&a, &b, 200); /* replay beyond the TTL must not pass the gate */
    len = sw_encode(&a, SW_BCAST, 1000, f, sizeof f);
    CHECK(sw_decode(&b, f, (unsigned)len, 1000 + SW_TTL_MS + 1) == SW_ESTALE);
    rec_make(b.rec[21], SW_CLAIMED, 1, 0, U32MAX); /* version space exhausted */
    CHECK(sw_task(&b, 21, SW_CLAIMED, 0, 0, 1000) == SW_EBUSY);
    CHECK(sw_task(&b, 21, SW_DONE, 0, 0, 1000) == SW_OK); /* DONE still wins */
    CHECK(leafv(&b, 21).st == SW_DONE && leafv(&b, 21).ver == U32MAX);
}

static unsigned leaves_set(sw_node *n, unsigned from, unsigned cnt,
                           unsigned st)
{
    unsigned i, ok = 0;

    for (i = from; i < from + cnt; i++)
        ok += sw_task(n, i, st, (sw_u16)i, 0, 1000) == SW_OK;
    return ok;
}

static int same_root(sw_node *a, sw_node *b)
{
    rehash(a);
    rehash(b);
    return !memcmp(a->hash[0], b->hash[0], SW_HASH);
}

/* Two-way Merkle repair of changes older than the link and the change-log
 * ring. Per round a sends ka unicasts (and optionally a beacon), b one.
 * Beacons and unequal rates once livelocked the descent. */
static unsigned heal(const sw_u8 *t, unsigned nt, unsigned a0, unsigned b0,
                     unsigned ka, int beacon)
{
    static sw_node a, b;
    sw_u8 f[255];
    unsigned r, j;
    int len;

    pair_t(&a, &b, 200, t, nt);
    CHECK(leaves_set(&a, a0, 40, SW_DONE) == 40);
    CHECK(leaves_set(&b, b0, 6, SW_CLAIMED) == 6);
    len = sw_encode(&a, SW_BCAST, 1000, f, sizeof f);
    CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_OK);
    len = sw_encode(&b, SW_BCAST, 1000, f, sizeof f);
    CHECK(sw_decode(&a, f, (unsigned)len, 1000) == SW_OK);
    for (r = 0; r < 100 && !same_root(&a, &b); r++) {
        for (j = 0; j < ka; j++) {
            len = sw_encode(&a, 2, 1000, f, sizeof f);
            CHECK(len > 0 && sw_decode(&b, f, (unsigned)len, 1000) == SW_OK);
        }
        if (beacon) {
            len = sw_encode(&a, SW_BCAST, 1000, f, sizeof f);
            CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_OK);
        }
        len = sw_encode(&b, 1, 1000, f, sizeof f);
        CHECK(len > 0 && sw_decode(&a, f, (unsigned)len, 1000) == SW_OK);
    }
    CHECK(same_root(&a, &b));
    CHECK(leafv(&b, a0).st == SW_DONE && leafv(&a, b0).st == SW_CLAIMED);
    return r;
}

/* Merkle repair over a two-way link (descent, several schedules and tree
 * depths) and a one-way link (sweep) */
static void test_antientropy(void)
{
    static sw_node a, b;
    sw_u8 f[255];
    unsigned r;
    int len;

    printf("anti-entropy: two-way rounds 1:1=%u beacon=%u 2:1=%u; "
           "deep tree 1:1=%u beacon=%u 2:1=%u\n",
           heal(tree, NTASK, 17, 57, 1, 0), heal(tree, NTASK, 17, 57, 1, 1),
           heal(tree, NTASK, 17, 57, 2, 0),
           heal(deep, sizeof deep, 15, 70, 1, 0),
           heal(deep, sizeof deep, 15, 70, 1, 1),
           heal(deep, sizeof deep, 15, 70, 2, 0));

    pair(&a, &b, 400); /* a reaches b, b cannot reach a */
    CHECK(leaves_set(&a, 17, 48, SW_DONE) == 48);
    peer_merge(&a, &b.self, 1000); /* b's context relayed by a third drone */
    for (r = 0; r < 60 && !same_root(&a, &b); r++) {
        len = sw_encode(&a, 2, 1000, f, sizeof f);
        CHECK(len > 0);
        CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_OK);
    }
    printf("anti-entropy: one-way link converged in %u frames\n", r);
    CHECK(same_root(&a, &b) && SW_GOAL_DONE(&b));
}

/* mutate valid frames, re-tag so the parser sees them, decode must never
 * crash, overrun, or break invariants (run under ASan/UBSan) */
static void test_fuzz(void)
{
    static sw_node a, b, c;
    sw_u8 f[255], g[255];
    unsigned i, j, k, len, cut, ok = 0;
    int r;

    pair(&a, &b, 200);
    r = sw_encode(&b, SW_BCAST, 1000, f, sizeof f);
    sw_decode(&a, f, (unsigned)r, 1000);
    for (i = 30; i < 50; i++)
        sw_task(&a, i, i % 2 ? SW_DONE : SW_CLAIMED, (sw_u16)i, 0, 1000);
    r = sw_encode(&a, 2, 1000, f, sizeof f);
    CHECK(r > 0);
    len = (unsigned)r;
    for (i = 0; i < 200000; i++) {
        memcpy(g, f, len);
        cut = len;
        if (i % 5 == 0)
            cut = SW_HDR_LEN + SW_MAC + rnd() % (len - SW_HDR_LEN - SW_MAC);
        else if (i % 5 == 1)
            for (k = SW_HDR_LEN; k < len; k++)
                g[k] = (sw_u8)rnd();
        for (k = rnd() % 4 + 1; k; k--)
            g[rnd() % (cut - SW_MAC)] = (sw_u8)rnd();
        if (i & 1) { /* keep header valid so the body parser is reached */
            memcpy(g, f, 10);
        }
        b2s(g + cut - SW_MAC, SW_MAC, b.key, SW_KEY, g, cut - SW_MAC);
        c = b;
        if (sw_decode(&c, g, cut, 1000) == SW_OK)
            ok++;
        CHECK(c.npeer <= SW_MAX_NODES && c.nlink <= SW_MAX_NBR);
        CHECK(index_ok(&c));
        for (k = 0; k < NTASK; k++)
            CHECK(c.rec[k][0] <= SW_DONE);
        for (k = 0; k < c.nlink; k++) {
            CHECK(c.link[k].npush <= SW_FANOUT && c.link[k].nprobe <= SW_FANOUT);
            for (j = 0; j < c.link[k].nprobe; j++)
                CHECK(c.link[k].probe[j] < NTASK && c.cnt[c.link[k].probe[j]]);
        }
    }
    printf("fuzz: 200000 mutated frames, %u accepted, no faults\n", ok);
}

/* ---- swarm simulation ------------------------------------------------- */

#define TICK 100
#define MAXD 16
#define MAXT 20000 /* tick limit */

#define QMAX 4

typedef struct {
    sw_node n;
    double p[3], v[3];
    int q[QMAX];    /* own claims, head is being worked on */
    int nq;
    int left;       /* work ticks left on the head; < 0: dead */
} drone;

typedef struct {
    double spacing;  /* grid spacing, m */
    double gap;      /* x offset of a second cluster; 0 = one grid */
    double drift;    /* max random speed, m/s */
    double lsp;      /* leaf spacing along x, m */
    sw_u32 lease;    /* claim lease, ms; 0 = default */
    unsigned nd;     /* drones */
    unsigned cols;   /* grid columns per cluster */
    unsigned work;   /* ticks per task */
    unsigned queue;  /* claims a drone keeps ahead (1 = only the current) */
    unsigned on;     /* cross-cluster link up for `on` ticks ... */
    unsigned off;    /* ... then down for `off` ticks; 0 = always up */
    int kill;        /* tick at which a claim holder dies; < 0 = none */
} simcfg;

typedef struct {
    unsigned conv, t, frames, bytes, lost, waste, dup, hops, heal, reconn;
    unsigned victim;  /* leaf held by the drone that died, 0 = none */
    sw_u32 vexp;      /* its lease expiry as the others knew it */
} simres;

static drone sw[MAXD];
static unsigned doers[NTASK];
static sw_u8 joined[NTASK][SW_REC_LEN];

static double dist(const double *a, const double *b)
{
    double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static unsigned cluster(const simcfg *cf, unsigned i)
{
    return cf->gap > 0 && i >= cf->nd / 2;
}

/* the radio: a frame from s reaches d only within s's true range, and
 * across clusters only while the scheduled link is up */
static void deliver(const simcfg *cf, simres *rs, unsigned s, unsigned d,
                    const sw_u8 *f, unsigned len, unsigned t, sw_u32 now)
{
    int r;

    if (sw[d].left < 0 || dist(sw[s].p, sw[d].p) > sw[s].n.self.range)
        return;
    if (cf->off && cluster(cf, s) != cluster(cf, d) &&
        t % (cf->on + cf->off) >= cf->on)
        return;
    if (rnd() % 10 == 0) {
        rs->lost++;
        return;
    }
    r = sw_decode(&sw[d].n, f, len, now);
    CHECK(r == SW_OK || r == SW_EREACH);
}

/* every live drone holds at least the join of all views at reconnection */
static int healed(const simcfg *cf)
{
    unsigned i, k;

    for (i = 0; i < cf->nd; i++)
        for (k = 0; k < NTASK; k++)
            if (sw[i].left >= 0 && !sw[i].n.cnt[k] &&
                rec_gt(joined[k], sw[i].n.rec[k]))
                return 0;
    return 1;
}

/* planner: drop claims lost to others, work the head task, keep
 * cf->queue claims ahead (nearest free leaf first) */
static void plan(const simcfg *cf, simres *rs, drone *d, sw_u32 now)
{
    sw_node *n = &d->n;
    sw_rec rc;
    double lp[3], c, bc;
    unsigned k, j;
    int best;

    for (j = 0; j < (unsigned)d->nq;) {
        rc = leafv(n, (unsigned)d->q[j]);
        if (rc.st != SW_DONE && rc.owner == n->self.id) {
            j++;
            continue;
        }
        if (j == 0 && d->left > 0) { /* work spent on a task someone won */
            rs->waste += cf->work - (unsigned)d->left;
            d->left = 0;
        }
        d->nq--;
        memmove(d->q + j, d->q + j + 1, (unsigned)d->nq * sizeof d->q[0]);
    }
    if (d->nq && d->left > 0 && --d->left == 0) {
        rc = leafv(n, (unsigned)d->q[0]);
        CHECK(sw_task(n, (unsigned)d->q[0], SW_DONE, rc.cost, 0, now) == SW_OK);
        doers[d->q[0]]++;
        d->nq--;
        memmove(d->q, d->q + 1, (unsigned)d->nq * sizeof d->q[0]);
    }
    while ((unsigned)d->nq < cf->queue) {
        best = -1;
        bc = 1e30;
        for (k = 0; k < NTASK; k++) {
            if (n->cnt[k])
                continue;
            rc = leafv(n, k);
            if (rc.st == SW_DONE || (rc.st == SW_CLAIMED && rc.ver > now))
                continue;
            lp[0] = (double)(k % 8) * cf->lsp;
            lp[1] = (double)(k / 8) * 150.0;
            lp[2] = 50;
            c = dist(lp, d->p);
            if (c < bc) {
                bc = c;
                best = (int)k;
            }
        }
        if (best < 0 ||
            sw_task(n, (unsigned)best, SW_CLAIMED,
                    (sw_u16)(bc < 65535 ? bc : 65535), cf->lease,
                    now) != SW_OK)
            break;
        d->q[d->nq++] = best;
    }
    if (d->nq && d->left == 0)
        d->left = (int)cf->work;
}

static void sim(const simcfg *cf, simres *rs)
{
    static const unsigned mtus[4] = { SW_FRAME_MIN, 255, 512, 1400 };
    sw_u16 nb[SW_MAX_NODES];
    sw_u8 f[1400];
    sw_state s;
    sw_rec rc;
    sw_u32 now = 1000;
    unsigned i, j, k, t, m, x, cl, t0 = 0, healing = 0;
    unsigned half = cf->nd > 1 ? cf->nd / 2 : 1;
    int r;

    memset(rs, 0, sizeof *rs);
    memset(doers, 0, sizeof doers);
    for (i = 0; i < cf->nd; i++) {
        cl = cluster(cf, i);
        x = cf->gap > 0 ? i % half : i;
        sw[i].p[0] = (double)(x % cf->cols) * cf->spacing + (cl ? cf->gap : 0);
        sw[i].p[1] = (double)(x / cf->cols) * cf->spacing;
        sw[i].p[2] = 50;
        sw[i].v[0] = frnd(-cf->drift, cf->drift);
        sw[i].v[1] = frnd(-cf->drift, cf->drift);
        sw[i].v[2] = 0;
        sw[i].nq = 0;
        sw[i].left = 0;
        /* heterogeneous capacity: radio reach and frame size */
        CHECK(sw_init(&sw[i].n, (sw_u16)(100 + i), (sw_u16)(380 + 40 * (i % 6)),
                      (sw_u16)mtus[i % 4], tree, NTASK, key, sizeof key) == SW_OK);
    }
    for (t = 0; t < MAXT && !rs->conv; t++, now += TICK) {
        for (i = 0; i < cf->nd; i++) {
            if (sw[i].left < 0)
                continue;
            for (j = 0; j < 3; j++)
                sw[i].p[j] += sw[i].v[j] * TICK / 1000.0;
            state_at(&s, sw[i].p, sw[i].v, 0.3 * i);
            CHECK(sw_set_self(&sw[i].n, &s, now) == SW_OK);
        }
        if (cf->kill >= 0 && t >= (unsigned)cf->kill && !rs->victim) {
            /* a drone dies holding a claim that drone 0 already knows */
            for (i = 1; i < cf->nd; i++) {
                if (!sw[i].nq)
                    continue;
                rc = leafv(&sw[0].n, (unsigned)sw[i].q[0]);
                if (rc.st == SW_CLAIMED && rc.owner == sw[i].n.self.id)
                    break;
            }
            if (i < cf->nd) {
                sw[i].left = -1;
                rs->victim = (unsigned)sw[i].q[0];
                rs->vexp = rc.ver;
            }
        }
        if (cf->off && t && t % (cf->on + cf->off) == 0) {
            /* reconnection: snapshot the join of every view */
            for (k = 0; k < NTASK; k++) {
                memcpy(joined[k], sw[0].n.rec[k], SW_REC_LEN);
                for (i = 1; i < cf->nd; i++)
                    if (sw[i].left >= 0 && rec_gt(sw[i].n.rec[k], joined[k]))
                        memcpy(joined[k], sw[i].n.rec[k], SW_REC_LEN);
            }
            healing = 1;
            t0 = t;
        }
        for (i = 0; i < cf->nd; i++) {
            if (sw[i].left < 0)
                continue;
            m = sw_tick(&sw[i].n, now, nb, SW_MAX_NODES);
            if (t % 10 == 0 || !m) {
                r = sw_encode(&sw[i].n, SW_BCAST, now, f, sizeof f);
                CHECK(r > 0);
                for (j = 0; j < cf->nd; j++)
                    if (j != i)
                        deliver(cf, rs, i, j, f, (unsigned)r, t, now);
            }
            for (k = 0; k < m; k++) {
                r = sw_encode(&sw[i].n, nb[k], now, f, sizeof f);
                CHECK(r > 0 || r == SW_ESLOT);
                if (r <= 0)
                    continue;
                rs->frames++;
                rs->bytes += (unsigned)r;
                j = nb[k] - 100U;
                CHECK(j < cf->nd && r <= (int)sw[j].n.self.mtu);
                deliver(cf, rs, i, j, f, (unsigned)r, t, now);
            }
        }
        if (healing && healed(cf)) {
            healing = 0;
            rs->reconn++;
            if (t - t0 > rs->heal)
                rs->heal = t - t0;
        }
        for (i = 0; i < cf->nd; i++)
            if (sw[i].left >= 0)
                plan(cf, rs, &sw[i], now);
        rs->conv = 1;
        for (i = 0; i < cf->nd; i++)
            if (sw[i].left >= 0 && (!SW_GOAL_DONE(&sw[i].n) ||
                                    memcmp(sw[i].n.hash[0], sw[0].n.hash[0],
                                           SW_HASH)))
                rs->conv = 0;
    }
    rs->t = t;
    for (k = 0; k < NTASK; k++)
        if (doers[k] > 1)
            rs->dup += doers[k] - 1;
    for (i = 0; i < sw[0].n.npeer; i++)
        if (sw[0].n.peer[i].hops > 1)
            rs->hops++;
    for (i = 0; i < cf->nd; i++)
        CHECK(index_ok(&sw[i].n));
}

/* one moving grid, packet loss, mixed ranges and MTUs, a drone dies */
static void test_swarm(void)
{
    simcfg cf;
    simres rs;

    memset(&cf, 0, sizeof cf);
    cf.nd = 16;
    cf.cols = 4;
    cf.spacing = 350;
    cf.drift = 1;
    cf.lsp = 150;
    cf.work = 20;
    cf.queue = 1;
    cf.kill = 25;
    sim(&cf, &rs);
    printf("swarm: %u drones, converged=%u after %.1f s, frames=%u avg=%u B, "
           "lost=%u, work lost to conflicts %.1f s, duplicate completions=%u, "
           "drone 0 knows %u peers (%u multi-hop)\n",
           cf.nd, rs.conv, (double)rs.t * TICK / 1000.0, rs.frames,
           rs.frames ? rs.bytes / rs.frames : 0U, rs.lost,
           (double)rs.waste * TICK / 1000.0, rs.dup,
           (unsigned)sw[0].n.npeer, rs.hops);
    CHECK(rs.conv && rs.victim);
    /* taken over only once the dead drone's lease ran out */
    CHECK(leafv(&sw[0].n, rs.victim).st == SW_DONE &&
          leafv(&sw[0].n, rs.victim).ver > rs.vexp);
    CHECK(rs.hops > 0 || SW_MAX_NODES < cf.nd);
    CHECK(rs.dup <= 2);
}

/* two clusters whose radio link is up 2 min, down 4 min, up again, over
 * and over; tasks take 200 s, so the mission spans several outages. Every
 * reconnection must heal within seconds. Nothing claimed during an outage
 * can reach the other side, so a planner that claims the work it will do
 * during an outage ahead of time, with a lease that outlasts it, must see
 * no duplicate work at all. */
static void test_dynamic(void)
{
    static const sw_u32 lease[2] = { 0, 900000 };
    static const unsigned queue[2] = { 1, 3 };
    simcfg cf;
    simres rs[2];
    unsigned i;

    memset(&cf, 0, sizeof cf);
    cf.nd = 12;
    cf.cols = 3;
    cf.spacing = 250;
    cf.gap = 900;
    cf.lsp = 200;
    cf.work = 2000;
    cf.on = 1200;
    cf.off = 2400;
    cf.kill = -1;
    for (i = 0; i < 2; i++) {
        cf.lease = lease[i];
        cf.queue = queue[i];
        sim(&cf, &rs[i]);
        printf("dynamic: lease %3lu s, claims ahead %u: converged=%u after "
               "%.1f s, reconnections=%u, worst heal %.1f s, duplicate "
               "completions=%u, work lost to conflicts %.1f s\n",
               (unsigned long)((lease[i] ? lease[i] : SW_LEASE_MS) / 1000),
               queue[i], rs[i].conv, (double)rs[i].t * TICK / 1000.0,
               rs[i].reconn, (double)rs[i].heal * TICK / 1000.0, rs[i].dup,
               (double)rs[i].waste * TICK / 1000.0);
        CHECK(rs[i].conv && rs[i].reconn >= 2 && rs[i].heal <= 30);
    }
    CHECK(rs[1].dup == 0 && rs[1].waste < cf.work);
}

int main(void)
{
    test_blake2s();
    test_lattice();
    test_lie();
    test_init();
    test_index();
    test_frames();
    test_antientropy();
    test_fuzz();
    test_swarm();
    test_dynamic();
    printf(fails ? "FAILED: %d\n" : "all tests passed\n", fails);
    return fails != 0;
}
