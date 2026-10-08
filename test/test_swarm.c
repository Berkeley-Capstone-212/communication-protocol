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
    return lo + (hi - lo) * (double)rnd() / (double)0xFFFFFF;
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

static void rand_rec(sw_rec *r)
{
    r->st = (sw_u8)(rnd() % 3);
    r->ver = rnd() % 4;
    r->owner = (sw_u16)(rnd() % 3);
    r->cost = (sw_u16)(rnd() % 3);
}

static const sw_rec *rmax(const sw_rec *a, const sw_rec *b)
{
    return rec_gt(b, a) ? b : a;
}

static int req(const sw_rec *a, const sw_rec *b)
{
    return a->st == b->st && a->ver == b->ver && a->owner == b->owner &&
           a->cost == b->cost;
}

/* merge must be a join: total, antisymmetric, associative, commutative */
static void test_lattice(void)
{
    sw_rec a, b, c;
    unsigned i;

    for (i = 0; i < 20000; i++) {
        rand_rec(&a);
        rand_rec(&b);
        rand_rec(&c);
        CHECK(!(rec_gt(&a, &b) && rec_gt(&b, &a)));
        CHECK(req(&a, &b) || rec_gt(&a, &b) || rec_gt(&b, &a));
        CHECK(req(rmax(&a, &b), rmax(&b, &a)));
        CHECK(req(rmax(&a, rmax(&b, &c)), rmax(rmax(&a, &b), &c)));
        CHECK(req(rmax(&a, &a), &a));
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
    unsigned i;

    memset(s, 0, sizeof *s);
    s->q[0] = cos(yaw / 2);
    s->q[3] = sin(yaw / 2);
    for (i = 0; i < 3; i++) {
        s->p[i] = (sw_real)p[i];
        s->v[i] = (sw_real)v[i];
    }
    s->sig[0] = 0.01;
    s->sig[1] = 0.1;
    s->sig[2] = 2.0;
}

/* Log on the sender, Exp on the receiver must reproduce p and v */
static void test_lie(void)
{
    static sw_node n;
    sw_state s;
    double ax[3], an, ang, worst = 0, e, zero = 0;
    unsigned i, j;

    CHECK(sw_init(&n, 1, 500, 255, tree, NTASK, key, sizeof key) == SW_OK);
    memset(&s, 0, sizeof s);
    for (i = 0; i < 5000; i++) {
        for (j = 0; j < 3; j++) {
            ax[j] = frnd(-1, 1);
            s.p[j] = frnd(-50000, 50000);
            s.v[j] = frnd(-60, 60);
            s.sig[j] = 0;
        }
        an = sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]) + 1e-9;
        /* cover tiny, generic and near-pi rotations */
        ang = i % 3 == 0 ? frnd(0, 1e-6) : i % 3 == 1 ? frnd(0, 3.1415)
                                                      : 3.14159265 - frnd(0, 1e-5);
        if (i % 7 == 0)
            ang = -ang; /* w < 0 hemisphere */
        s.q[0] = cos(ang / 2);
        for (j = 0; j < 3; j++)
            s.q[j + 1] = sin(ang / 2) * ax[j] / an;
        CHECK(sw_set_self(&n, &s, 1000) == SW_OK);
        for (j = 0; j < 3; j++) {
            e = fabs(n.self.p[j] - s.p[j]);
            CHECK(e <= n.self.sp);
            if (e > worst)
                worst = e;
            CHECK(fabs(n.self.v[j] - s.v[j]) <= n.self.sv);
        }
    }
    printf("lie: worst position round-trip error %.4f m at 50 km\n", worst);
    CHECK(worst < (sizeof(sw_real) == sizeof(double) ? 0.005 : 0.05));
    s.p[0] = 1e7; /* 10 000 km: does not fit the wire */
    CHECK(sw_set_self(&n, &s, 1000) == SW_EARG);
    /* finite but overflowing inside J^-1: must fail cleanly, not UB */
    s.p[0] = sizeof(sw_real) == sizeof(double) ? 1e300 : 3e38;
    CHECK(sw_set_self(&n, &s, 1000) == SW_EARG);
    s.p[0] = 0;
    s.v[1] = zero / zero;
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
    CHECK(sw_init(&n, 1, 500, 255, tree, NTASK, 0, 0) == SW_OK);
    CHECK(!SW_GOAL_DONE(&n));
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
    CHECK(sw_task(&a, 20, SW_CLAIMED, 7, 1000) == SW_OK);
    len = sw_encode(&a, 2, 1000, f, sizeof f);
    CHECK(len > 0 && len <= 255);
    CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_OK);
    CHECK(b.rec[20].st == SW_CLAIMED && b.rec[20].owner == 1);
    CHECK(sw_task(&b, 20, SW_CLAIMED, 9, 1100) == SW_EBUSY);
    CHECK(sw_task(&b, 20, SW_CLAIMED, 3, 1100) == SW_OK); /* outbid */
    CHECK(sw_task(&b, 20, SW_OPEN, 0, 1200) == SW_OK);
    CHECK(sw_task(&b, 20, SW_OPEN, 0, 1200) == SW_EARG);
    CHECK(sw_task(&b, 4, SW_CLAIMED, 1, 1200) == SW_EARG); /* not a leaf */
    CHECK(sw_decode(&b, f, (unsigned)len, 1000) == SW_OK); /* replay: harmless */
    CHECK(b.rec[20].st == SW_OPEN);

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
    b.rec[21].st = SW_CLAIMED; /* version space exhausted: DONE still wins */
    b.rec[21].ver = U32MAX;
    CHECK(sw_task(&b, 21, SW_CLAIMED, 0, 1000) == SW_EBUSY);
    CHECK(sw_task(&b, 21, SW_DONE, 0, 1000) == SW_OK && b.rec[21].ver == U32MAX);
}

static unsigned leaves_set(sw_node *n, unsigned from, unsigned cnt,
                           unsigned st)
{
    unsigned i, ok = 0;

    for (i = from; i < from + cnt; i++)
        ok += sw_task(n, i, st, (sw_u16)i, 1000) == SW_OK;
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
    CHECK(b.rec[a0].st == SW_DONE && a.rec[b0].st == SW_CLAIMED);
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
        sw_task(&a, i, i % 2 ? SW_DONE : SW_CLAIMED, (sw_u16)i, 1000);
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
        b2s(g + cut - SW_MAC, SW_MAC, key, sizeof key, g, cut - SW_MAC);
        c = b;
        if (sw_decode(&c, g, cut, 1000) == SW_OK)
            ok++;
        CHECK(c.npeer <= SW_MAX_NODES && c.nlink <= SW_MAX_NBR);
        for (k = 0; k < NTASK; k++)
            CHECK(c.rec[k].st <= SW_DONE);
        for (k = 0; k < c.nlink; k++) {
            CHECK(c.link[k].npush <= SW_FANOUT && c.link[k].nprobe <= SW_FANOUT);
            for (j = 0; j < c.link[k].nprobe; j++)
                CHECK(c.link[k].probe[j] < NTASK && c.cnt[c.link[k].probe[j]]);
        }
    }
    printf("fuzz: 200000 mutated frames, %u accepted, no faults\n", ok);
}

/* ---- swarm simulation ------------------------------------------------- */

#define N 16
#define TICK 100
#define WORK 20

typedef struct {
    sw_node n;
    double p[3], v[3];
    int alive, leaf, left;
} drone;

static drone sw[N];
static unsigned doers[NTASK];

static double dist(const double *a, const double *b)
{
    double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static void leaf_pos(unsigned leaf, double *p)
{
    p[0] = (double)(leaf % 8) * 150.0;
    p[1] = (double)(leaf / 8) * 150.0;
    p[2] = 50;
}

/* the radio: a frame from s reaches d only within s's true range */
static void deliver(unsigned s, unsigned d, const sw_u8 *f, unsigned len,
                    sw_u32 now, unsigned *lost)
{
    int r;

    if (!sw[d].alive || dist(sw[s].p, sw[d].p) > sw[s].n.self.range)
        return;
    if (rnd() % 10 == 0) {
        ++*lost;
        return;
    }
    r = sw_decode(&sw[d].n, f, len, now);
    CHECK(r == SW_OK || r == SW_EREACH);
}

static void test_swarm(void)
{
    static const unsigned mtus[4] = { 180, 255, 512, 1400 };
    sw_u16 nb[SW_MAX_NODES];
    sw_u8 f[1400];
    sw_state s;
    double lp[3];
    sw_u32 now = 1000;
    unsigned i, j, k, t, m, lost = 0, frames = 0, bytes = 0, aband = 0;
    unsigned dup = 0, hops = 0, conv = 0, victim_leaf = 0;
    int r, best;
    double c, bc;

    for (i = 0; i < N; i++) {
        sw[i].p[0] = (double)(i % 4) * 350.0;
        sw[i].p[1] = (double)(i / 4) * 350.0;
        sw[i].p[2] = 50;
        sw[i].v[0] = frnd(-1, 1);
        sw[i].v[1] = frnd(-1, 1);
        sw[i].v[2] = 0;
        sw[i].alive = 1;
        sw[i].leaf = -1;
        /* heterogeneous capacity: radio reach and frame size */
        CHECK(sw_init(&sw[i].n, (sw_u16)(100 + i), (sw_u16)(380 + 40 * (i % 6)),
                      (sw_u16)mtus[i % 4], tree, NTASK, key, sizeof key) == SW_OK);
    }
    for (t = 0; t < 6000 && !conv; t++, now += TICK) {
        for (i = 0; i < N; i++) {
            if (!sw[i].alive)
                continue;
            for (j = 0; j < 3; j++)
                sw[i].p[j] += sw[i].v[j] * TICK / 1000.0;
            state_at(&s, sw[i].p, sw[i].v, 0.3 * i);
            CHECK(sw_set_self(&sw[i].n, &s, now) == SW_OK);
        }
        if (t == 25) { /* a drone dies while holding a claim */
            for (i = 1; i < N && sw[i].leaf < 0; i++)
                ;
            CHECK(i < N);
            if (i < N) {
                sw[i].alive = 0;
                victim_leaf = (unsigned)sw[i].leaf;
            }
        }
        for (i = 0; i < N; i++) {
            if (!sw[i].alive)
                continue;
            m = sw_tick(&sw[i].n, now, nb, SW_MAX_NODES);
            if (t % 10 == 0 || !m) {
                r = sw_encode(&sw[i].n, SW_BCAST, now, f, sizeof f);
                CHECK(r > 0);
                for (j = 0; j < N; j++)
                    if (j != i)
                        deliver(i, j, f, (unsigned)r, now, &lost);
            }
            for (k = 0; k < m; k++) {
                r = sw_encode(&sw[i].n, nb[k], now, f, sizeof f);
                CHECK(r > 0);
                if (r <= 0)
                    continue;
                frames++;
                bytes += (unsigned)r;
                j = nb[k] - 100U;
                CHECK(j < N && r <= (int)sw[j].n.self.mtu);
                deliver(i, j, f, (unsigned)r, now, &lost);
            }
        }
        /* each drone: keep, finish, or pick the cheapest free leaf */
        for (i = 0; i < N; i++) {
            sw_node *n = &sw[i].n;
            if (!sw[i].alive)
                continue;
            if (sw[i].leaf >= 0) {
                sw_rec *rc = &n->rec[sw[i].leaf];
                if (rc->st == SW_DONE || rc->owner != n->self.id) {
                    aband++;
                    sw[i].leaf = -1;
                } else if (--sw[i].left == 0) {
                    CHECK(sw_task(n, (unsigned)sw[i].leaf, SW_DONE, rc->cost, now) == SW_OK);
                    doers[sw[i].leaf]++;
                    sw[i].leaf = -1;
                }
                continue;
            }
            best = -1;
            bc = 1e30;
            for (k = 0; k < NTASK; k++) {
                if (n->cnt[k] || n->rec[k].st == SW_DONE ||
                    (n->rec[k].st == SW_CLAIMED && n->rec[k].ver > now))
                    continue;
                leaf_pos(k, lp);
                c = dist(lp, sw[i].p);
                if (c < bc) {
                    bc = c;
                    best = (int)k;
                }
            }
            if (best >= 0 &&
                sw_task(n, (unsigned)best, SW_CLAIMED, (sw_u16)(bc < 65535 ? bc : 65535), now) == SW_OK) {
                sw[i].leaf = best;
                sw[i].left = WORK;
            }
        }
        conv = 1;
        for (i = 0; i < N; i++)
            if (sw[i].alive && (!SW_GOAL_DONE(&sw[i].n) ||
                                memcmp(sw[i].n.hash[0], sw[0].n.hash[0], SW_HASH)))
                conv = 0;
    }
    for (k = 0; k < NTASK; k++)
        if (doers[k] > 1)
            dup += doers[k] - 1;
    for (i = 0; i < sw[0].n.npeer; i++)
        if (sw[0].n.peer[i].hops > 1)
            hops++;
    printf("swarm: %u drones, converged=%u after %.1f s, frames=%u avg=%u B, "
           "lost=%u, abandoned=%u, duplicate completions=%u, "
           "drone 0 knows %u peers (%u multi-hop)\n",
           N, conv, (double)t * TICK / 1000.0, frames, frames ? bytes / frames : 0,
           lost, aband, dup, sw[0].n.npeer, hops);
    CHECK(conv);
    CHECK(sw[0].n.rec[victim_leaf].st == SW_DONE);
    CHECK(hops > 0 || SW_MAX_NODES < N);
    CHECK(dup <= 2);
}

int main(void)
{
    test_blake2s();
    test_lattice();
    test_lie();
    test_init();
    test_frames();
    test_antientropy();
    test_fuzz();
    test_swarm();
    printf(fails ? "FAILED: %d\n" : "all tests passed\n", fails);
    return fails != 0;
}
