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
/* Placeholder sizes (<=255). The defaults fill sw_ctx exactly, so bytes
 * that would otherwise be alignment padding are usable. */
#ifndef SW_OP_MAX       /* operation-info placeholder bytes */
#define SW_OP_MAX 8
#endif
#ifndef SW_CTX_MAX      /* context placeholder bytes */
#define SW_CTX_MAX 21
#endif

/* ---- timing and gating ---------------------------------------------- */
#ifndef SW_TTL_MS       /* context older than this is forgotten */
#define SW_TTL_MS 10000UL
#endif
#ifndef SW_LEASE_MS     /* default claim lease; owner renews at half-life */
#define SW_LEASE_MS 30000UL
#endif
#ifndef SW_SLOT_MS      /* least time a link slot is held before rotating */
#define SW_SLOT_MS 5000UL
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
#define SW_KEY 32         /* derived mission key bytes */
#define SW_HDR_LEN 18
#define SW_CTX_LEN 43     /* fixed part of one context on the wire */
#define SW_REC_LEN 9      /* st u8, owner u16, cost u16, ver u32 */
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
#define SW_ESTALE (-8)    /* sender context too old or in the future */
#define SW_EREACH (-9)    /* peer is not a neighbour on the reach graph */
#define SW_EBUSY (-10)    /* leaf held by a cheaper live claim */
#define SW_EDONE (-11)    /* leaf already done */
#define SW_ETIME (-12)    /* version space exhausted */
#define SW_ESLOT (-13)    /* all link slots held; served in rotation, retry */

/* Own estimator output, from an EqF / error-state filter on SE_2(3). */
typedef struct {
    const sw_u8 *op;  /* operation info placeholder */
    const sw_u8 *ctx; /* context placeholder */
    sw_real q[4];     /* attitude, Hamilton (w,x,y,z), body -> mission */
    sw_real v[3];     /* velocity in mission frame, m/s */
    sw_real p[3];     /* position in mission frame (ENU at anchor), m */
    sw_real sig[3];   /* conservative 1-sigma: attitude rad, vel m/s, pos m */
    unsigned op_len;
    unsigned ctx_len;
} sw_state;

/* One drone's context. Tangent coordinates are kept quantised exactly as
 * received so relaying never adds error; p, v, sp, sv are a decoded cache.
 * Members are ordered by alignment so the default build has no padding. */
typedef struct {
    sw_real p[3], v[3], sp, sv;
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
} sw_ctx;

/* Decoded view of a leaf record (sw_leaf). Records form a total order;
 * merge keeps the maximum, so replicas converge in any delivery order. */
typedef struct {
    sw_u32 ver;       /* claim: lease expiry; otherwise action time */
    sw_u16 owner;
    sw_u16 cost;
    unsigned st;
} sw_rec;

/* Per-neighbour sync state. With more neighbours than SW_MAX_NBR, slots
 * rotate first-in-first-out after SW_SLOT_MS, so each neighbour gets a
 * stable window instead of every link being evicted on every frame. */
typedef struct {
    sw_u32 heard;     /* last unicast from this neighbour */
    sw_u32 seen;      /* last use; an older one means it was out of touch */
    sw_u32 since;     /* slot taken */
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

/* Treat as read-only outside the API, except tflag[] for queries. Peers and
 * links are found by id through a sorted index (pid/lid -> pix/lix). */
typedef struct {
    sw_ctx self;
    sw_ctx peer[SW_MAX_NODES];
    sw_link link[SW_MAX_NBR];
    sw_u32 loghead;
    sw_u32 mission;
    sw_u16 first[SW_MAX_TASK];
    sw_u16 parent[SW_MAX_TASK];
    sw_u16 logleaf[SW_LOG];
    sw_u16 logsrc[SW_LOG];
    sw_u16 pid[SW_MAX_NODES];     /* peer ids, ascending */
    sw_u16 lid[SW_MAX_NBR];       /* link ids, ascending */
    sw_u16 ntask;
    sw_u8 rec[SW_MAX_TASK][SW_REC_LEN]; /* leaf records in wire form */
    sw_u8 hash[SW_MAX_TASK][SW_HASH];
    sw_u8 cnt[SW_MAX_TASK];
    sw_u8 tflag[SW_MAX_TASK];     /* bit1: subtree complete */
    sw_u8 pix[SW_MAX_NODES];      /* peer slot of pid[k] */
    sw_u8 lix[SW_MAX_NBR];        /* link slot of lid[k] */
    sw_u8 key[SW_KEY];
    sw_u8 npeer;
    sw_u8 nlink;
} sw_node;

/* tflag[] and SW_GOAL_DONE are current after sw_tick (or sw_encode) */
#define SW_COMPLETE 2
#define SW_GOAL_DONE(n) (((n)->tflag[0] & SW_COMPLETE) != 0)

/* Goal tree in breadth-first order: cnt[i] = number of children of node i,
 * 0 for a leaf. Node 0 is the goal and must have children. Identical on
 * every drone; it and the key (any length) define the mission. */
int sw_init(sw_node *n, sw_u16 id, sw_u16 range, sw_u16 mtu,
            const sw_u8 *cnt, unsigned ntask, const sw_u8 *key, unsigned klen);
/* Call at least every SW_TTL_MS (with unknown sigma if there is no fix):
 * frames whose sender context is older are rejected as stale. */
int sw_set_self(sw_node *n, const sw_state *s, sw_u32 now);
/* Local leaf action: SW_CLAIMED (bid `cost`, lower wins; `lease` ms, 0 for
 * SW_LEASE_MS), SW_OPEN (release own claim) or SW_DONE. A lease longer
 * than the expected link outage keeps others from taking over the task
 * while this drone is out of touch; re-claim to extend it. */
int sw_task(sw_node *n, unsigned leaf, unsigned st, sw_u16 cost,
            sw_u32 lease, sw_u32 now);
int sw_leaf(const sw_node *n, unsigned leaf, sw_rec *out);
/* Maintenance; writes current out-neighbours to nbr (rotating order, so
 * every neighbour is first in turn), returns how many. */
unsigned sw_tick(sw_node *n, sw_u32 now, sw_u16 *nbr, unsigned max);
/* Frame for one out-neighbour, or SW_BCAST beacon. Returns length or error. */
int sw_encode(sw_node *n, sw_u16 dst, sw_u32 now, sw_u8 *buf, unsigned cap);
int sw_decode(sw_node *n, const sw_u8 *buf, unsigned len, sw_u32 now);

#ifdef __cplusplus
}
#endif
#endif
