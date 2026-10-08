/*
 * swarm.h - decentralised context gossip and goal-tree agreement for UAV swarms.
 *
 * ISO C90 (also valid C++98). No heap, no globals, no OS calls: all state is
 * in the caller-owned sw_node, so memory use is fixed at compile time and
 * several nodes can live in one process (simulation) without interference.
 *
 * Time: every `now` is milliseconds since the mission epoch on a
 * GNSS-disciplined clock shared by the swarm. 32 bits => missions up to
 * 49 days. Values are compared as plain unsigned integers.
 */
#ifndef SWARM_H
#define SWARM_H

#include <limits.h>

#ifdef __cplusplus
extern "C" {
#endif

#if CHAR_BIT != 8
#error "swarm: 8-bit bytes required"
#endif
#if USHRT_MAX != 0xFFFF
#error "swarm: 16-bit unsigned short required"
#endif
typedef unsigned char sw_u8;
typedef unsigned short sw_u16;
typedef short sw_i16;
#if UINT_MAX == 0xFFFFFFFF
typedef unsigned int sw_u32;
typedef int sw_i32;
#elif ULONG_MAX == 0xFFFFFFFF
typedef unsigned long sw_u32;
typedef long sw_i32;
#else
#error "swarm: no 32-bit integer type"
#endif

/* SW_FLOAT selects single precision (needs a C99 libm: sqrtf, sinf, ...). */
#ifdef SW_FLOAT
typedef float sw_real;
#else
typedef double sw_real;
#endif

/* ---- capacity, override with -D ------------------------------------- */
#ifndef SW_MAX_NODES    /* other drones remembered, any hop distance (<=255) */
#define SW_MAX_NODES 32
#endif
#ifndef SW_MAX_NBR      /* links with per-neighbour sync state (<=255) */
#define SW_MAX_NBR 12
#endif
#ifndef SW_MAX_TASK     /* goal-tree nodes: root + sub-goals + leaves */
#define SW_MAX_TASK 128
#endif
#ifndef SW_FANOUT       /* max children of one tree node (<=255) */
#define SW_FANOUT 8
#endif
#ifndef SW_LOG          /* change-log ring, power of two */
#define SW_LOG 32
#endif
#ifndef SW_OP_MAX       /* operation-info placeholder bytes (<=255) */
#define SW_OP_MAX 8
#endif
#ifndef SW_CTX_MAX      /* context placeholder bytes (<=255) */
#define SW_CTX_MAX 16
#endif

/* ---- timing and gating ---------------------------------------------- */
#ifndef SW_TTL_MS       /* context older than this is forgotten */
#define SW_TTL_MS 10000UL
#endif
#ifndef SW_LEASE_MS     /* claim lease; owner renews at half-life */
#define SW_LEASE_MS 30000UL
#endif
#ifndef SW_SKEW_MS      /* tolerated clock skew for future stamps */
#define SW_SKEW_MS 1000UL
#endif
#ifndef SW_GATE_K       /* reachability gate in standard deviations */
#define SW_GATE_K 3
#endif
#ifndef SW_ACC          /* 1-sigma unmodelled acceleration, m/s^2 */
#define SW_ACC 2.0
#endif

/* ---- wire constants -------------------------------------------------- */
#define SW_VERSION 1
#define SW_BCAST 0xFFFF   /* destination of a discovery beacon */
#define SW_HASH 8         /* Merkle node hash bytes */
#define SW_MAC 8          /* frame tag bytes */
#define SW_KEY_MAX 32
#define SW_HDR_LEN 18
#define SW_CTX_LEN 43     /* fixed part of one context on the wire */
#define SW_REC_LEN 9
/* Smallest MTU that always fits the own context, the largest probe and one
 * record, so every frame makes sync progress. */
#define SW_FRAME_MIN (SW_HDR_LEN + SW_CTX_LEN + SW_OP_MAX + SW_CTX_MAX + \
                      2 + SW_FANOUT * SW_REC_LEN + 2 + SW_REC_LEN + 2 + SW_MAC)

/* leaf states, ordered: a later state of equal version dominates */
#define SW_OPEN 0
#define SW_CLAIMED 1
#define SW_DONE 2

/* results; negative is failure */
#define SW_OK 0
#define SW_EARG (-1)      /* bad argument or tree */
#define SW_ESHORT (-2)    /* buffer or frame too small */
#define SW_EVER (-3)      /* unknown protocol version */
#define SW_EMISSION (-4)  /* other mission */
#define SW_EADDR (-5)     /* not addressed to us, or from our own id */
#define SW_EAUTH (-6)     /* tag mismatch */
#define SW_EFIELD (-7)    /* malformed field */
#define SW_ESTALE (-8)    /* sender stamp in the future */
#define SW_EREACH (-9)    /* peer is not a neighbour on the reach graph */
#define SW_EBUSY (-10)    /* leaf held by a cheaper live claim */
#define SW_EDONE (-11)    /* leaf already done */
#define SW_ETIME (-12)    /* version space exhausted */

/* Own estimator output, from an EqF / error-state filter on SE_2(3). */
typedef struct {
    sw_real q[4];     /* attitude, Hamilton (w,x,y,z), body -> mission */
    sw_real v[3];     /* velocity in mission frame, m/s */
    sw_real p[3];     /* position in mission frame (ENU at anchor), m */
    sw_real sig[3];   /* conservative 1-sigma: attitude rad, vel m/s, pos m */
    const sw_u8 *op;  /* operation info placeholder */
    unsigned op_len;
    const sw_u8 *ctx; /* context placeholder */
    unsigned ctx_len;
} sw_state;

/* One drone's context. Tangent coordinates are kept quantised exactly as
 * received so relaying never adds error; p, v, sp, sv are a decoded cache. */
typedef struct {
    sw_u32 stamp;     /* origin time of the estimate */
    sw_i32 rho[3];    /* J(theta)^-1 p, mm */
    sw_i16 th[3];     /* so(3) part, 1e-4 rad */
    sw_i16 nu[3];     /* J(theta)^-1 v, cm/s */
    sw_u16 sig[3];    /* 1e-4 rad, cm/s, cm; 0xFFFF = unknown */
    sw_u16 id;
    sw_u16 range;     /* radio reach, m */
    sw_u16 mtu;       /* largest frame it accepts */
    sw_u8 hops;       /* wire: relays so far; stored: hop distance (1 = direct) */
    sw_u8 op_len;
    sw_u8 ctx_len;
    sw_u8 op[SW_OP_MAX];
    sw_u8 ctx[SW_CTX_MAX];
    sw_real p[3], v[3], sp, sv;
} sw_ctx;

/* Replicated leaf record. Records form a total order (see sw_task); merge
 * keeps the maximum, so replicas converge in any delivery order. */
typedef struct {
    sw_u32 ver;       /* claim: lease expiry; otherwise action time */
    sw_u16 owner;
    sw_u16 cost;
    sw_u8 st;
} sw_rec;

typedef struct {
    sw_u32 heard;     /* last frame from this neighbour */
    sw_u32 seen;      /* last use, for eviction */
    sw_u32 logpos;    /* change-log position already sent */
    sw_u16 id;
    sw_u16 sweep;     /* fallback cursor when it never answers */
    sw_u16 probe[SW_FANOUT]; /* tree nodes to describe to it next */
    sw_u16 push[SW_FANOUT];  /* leaves where our record beats its own */
    sw_u8 nprobe;
    sw_u8 npush;
    sw_u8 ctxcur;     /* relay round-robin */
    sw_u8 fb;         /* bit0: root[] valid, bit1: it has unicast to us */
    sw_u8 root[SW_HASH];
} sw_link;

/* Treat as read-only outside the API, except rec[] / tflag[] for queries. */
typedef struct {
    sw_ctx self;
    sw_ctx peer[SW_MAX_NODES];
    sw_link link[SW_MAX_NBR];
    sw_rec rec[SW_MAX_TASK];
    sw_u8 hash[SW_MAX_TASK][SW_HASH];
    sw_u16 first[SW_MAX_TASK];
    sw_u16 parent[SW_MAX_TASK];
    sw_u8 cnt[SW_MAX_TASK];
    sw_u8 tflag[SW_MAX_TASK];     /* bit1: subtree complete */
    sw_u16 logleaf[SW_LOG];
    sw_u16 logsrc[SW_LOG];
    sw_u32 loghead;
    sw_u32 mission;
    sw_u16 ntask;
    sw_u8 npeer;
    sw_u8 nlink;
    sw_u8 key[SW_KEY_MAX];
    sw_u8 klen;
} sw_node;

/* tflag[] and SW_GOAL_DONE are current after sw_tick (or sw_encode) */
#define SW_COMPLETE 2
#define SW_GOAL_DONE(n) (((n)->tflag[0] & SW_COMPLETE) != 0)

/* Goal tree in breadth-first order: cnt[i] = number of children of node i,
 * 0 for a leaf. Node 0 is the goal and must have children. Identical on
 * every drone; it and the key define the mission. */
int sw_init(sw_node *n, sw_u16 id, sw_u16 range, sw_u16 mtu,
            const sw_u8 *cnt, unsigned ntask, const sw_u8 *key, unsigned klen);
/* Call at least every SW_TTL_MS (with unknown sigma if there is no fix):
 * frames whose sender context is older are rejected as stale. */
int sw_set_self(sw_node *n, const sw_state *s, sw_u32 now);
/* Local leaf action: SW_CLAIMED (bid `cost`, lower wins), SW_OPEN (release
 * own claim) or SW_DONE. */
int sw_task(sw_node *n, unsigned leaf, unsigned st, sw_u16 cost, sw_u32 now);
/* Maintenance; writes current out-neighbours to nbr, returns how many. */
unsigned sw_tick(sw_node *n, sw_u32 now, sw_u16 *nbr, unsigned max);
/* Frame for one out-neighbour, or SW_BCAST beacon. Returns length or error. */
int sw_encode(sw_node *n, sw_u16 dst, sw_u32 now, sw_u8 *buf, unsigned cap);
int sw_decode(sw_node *n, const sw_u8 *buf, unsigned len, sw_u32 now);

#ifdef __cplusplus
}
#endif
#endif
