# swarm: decentralised communication protocol for UAV swarms

Think of a crew spread across a field with no radio dispatcher. Each person
talks only to the people near enough to hear them. They pass along what they
heard from others, and they keep a shared checklist of the job. When two
people find that their checklists differ, they compare section by section
and fix only the differing lines. No one is in charge, yet everyone ends up
with the same checklist, and nobody does a job that someone else has already
taken.

This library implements that idea for drones:

| Crew analogy | Protocol |
|---|---|
| who can hear whom | directed **reach graph**: edge a→b iff b is inside a's radio range |
| "what I'm doing / where I am" | **context**: pose in Lie-algebra coordinates, uncertainty, capacity, op info, context placeholder |
| repeating what you heard | outbound frame to neighbour d carries **every known context except d's own** |
| the job checklist | **goal tree**: the root is the goal, internal nodes are sub-goals, leaves are tasks |
| "I'll take that one" | leaf **claim** with a lease; conflicts are resolved by a deterministic total order |
| comparing checklists | **Merkle hash** per tree node; the root hash travels in every frame |

Language: ISO C90, which also compiles as C++98. You can call it from C# through P/Invoke
on the `sw_*` functions. It uses no heap, no globals and no OS calls. With the
default limits the code is about 9.5 KB and each node needs 8.7 KB of RAM
(`sw_node`, or 7.5 KB with `SW_FLOAT`).

```
include/swarm.h     API, wire constants, compile-time limits
src/swarm.c         implementation (single translation unit)
test/test_swarm.c   unit, fuzz and swarm-simulation tests
```

## Build

```
make            # build/libswarm.a  (-std=c89 -pedantic -Werror, -Os, section GC ready)
make test       # 3 configs under ASan/UBSan: default, tiny tables, single precision
make cxx        # same source as C++98
make size
```

Link with `-lm`. For the smallest firmware, link with `-Wl,--gc-sections`
(GNU) or `-Wl,-dead_strip` (Apple). `SW_FLOAT` switches all real arithmetic
to `float`. It needs `sqrtf`/`sinf`/`cosf`/`atan2f` from a C99 libm. Without it,
the code uses only C90 `double` math.

## Model

### Reach graph and capacity

Every drone advertises its **capacity**: radio `range` (m) and `mtu` (the
largest frame it accepts). Ranges differ, so the graph is **directed**:

```
a -> b   iff   |p_a(now) - p_b(now)| <= range_a + K * sqrt(s_a^2 + s_b^2)
```

Here `p(now) = p + v*dt` is a constant-velocity prediction from the context
stamp. The position sigma grows as `s^2 = s_p^2 + (s_v dt)^2 + (A dt^2/2)^2`,
with K = `SW_GATE_K` and A = `SW_ACC`.

- **Inbound**: `sw_decode` accepts a frame only along an edge src→us. A
  frame from a drone that cannot reach us is rejected with `SW_EREACH`.
  A sender context older than `SW_TTL_MS`, or further than `SW_SKEW_MS`
  in the future, is rejected with `SW_ESTALE`; otherwise an old frame's
  grown sigma would pass any gate. Each drone must therefore call
  `sw_set_self` at least every `SW_TTL_MS`, with unknown sigma if it has
  no fix.
- **Outbound**: `sw_tick` returns the current out-neighbours, and `sw_encode`
  refuses any destination that is not one. A frame is never larger than
  min(own mtu, destination mtu, buffer).
- **Discovery**: a `SW_BCAST` beacon carries only the sender's context, so
  drones can learn each other's positions before any edge is known.

### Context

Each context carries: id, hop count, origin stamp, range, mtu, pose, sigmas,
an **operation info placeholder** (`SW_OP_MAX` bytes) and a **context
placeholder** (`SW_CTX_MAX` bytes). Both placeholders are opaque to the
protocol and only length-checked. Until their contents are decided, we
recommend a self-describing TLV layout (`type u8, len u8, value`) so fields
can be added without a protocol version bump.

Relays keep the origin's quantised integers bit-exact, so forwarding never
adds error. Each drone keeps the freshest copy per origin (by stamp, then by
fewest hops). Copies older than `SW_TTL_MS` are forgotten. When the table is
full, the farthest and then oldest entry is evicted, so hearsay from far away
never pushes out the local neighbourhood.

### Pose in Lie-algebra coordinates (SE_2(3))

The navigation state X = (R, v, p) is an extended pose. It goes on the wire as
its logarithm, a 9-vector in the Lie algebra:

```
theta = Log(R)                     |theta| <= pi   (quaternion hemisphere w >= 0)
nu    = J(theta)^-1 v
rho   = J(theta)^-1 p

J     = I + a[θ]x + b[θ]x²      a = (1-cos t)/t²,  b = (t-sin t)/t³
J^-1  = I - ½[θ]x + c[θ]x²      c = 1/t² - cot(t/2)/(2t)
```

The receiver restores `p = J(theta) rho` and `v = J(theta) nu`. From that
point on, prediction, distance and gating are plain R³ linear algebra. Small
angles use Taylor series (`t < 0.1` for J, `t < 0.5` for J⁻¹), so there is no
cancellation in either precision.

Quantisation: θ uses 1e-4 rad (i16), ν uses cm/s (i16), ρ uses mm (i32,
|ρ| ≤ 2000 km). θ is quantised **first**, and J⁻¹ is evaluated at the
quantised θ. As a result, the receiver's Exp reproduces p to within the mm
rounding. Without this step, the SE(3) coupling turns 1e-4 rad of rounding
into metres of position error far from the anchor. Measured: 0.7 mm (double)
and 20 mm (float) worst case at 50 km.

Uncertainty is sent as three conservative isotropic 1-sigma values
(√trace of the attitude, velocity and position blocks). These are always
rounded **up**, and 0xFFFF means unknown. The receiver adds rounding and
arithmetic error on top. The estimate stays consistent: it is never
overconfident.

### Goal tree and claims

The tree is given in breadth-first order as `cnt[i]` = number of children of
node i (0 for a leaf). Every drone loads the identical tree. The 32-bit
**mission id** binds the tree shape, the key and the wire-shaping build
options, so a drone on another mission or with mismatched firmware is
rejected cleanly.

Each leaf holds a replicated record `(st, ver, owner, cost)`. Records are
ordered lexicographically by

```
(st == DONE, ver, st, -cost, -owner)
```

and merging keeps the maximum. That makes it a join-semilattice, i.e. a
state-based CRDT: merge is commutative, associative and idempotent. Replicas
therefore converge regardless of delivery order, loss, duplication or
replay.

- **claim**: `ver = max(now + SW_LEASE_MS, ver + 1)`, so `ver` is the lease
  expiry. A live claim by another drone can only be taken with a strictly
  lower cost (an auction). The owner renews at half-life inside `sw_tick`.
  If the owner dies, the lease expires and the leaf becomes claimable.
- **release**: allowed only for the owner. It writes `OPEN` with `ver + 1`.
- **done**: absorbing. It dominates everything.
- Two concurrent claims resolve deterministically. The loser sees
  `rec[leaf].owner != self` and stops.

Hashes: `leaf = H(index ‖ record)` (11 bytes) and
`node = H(child hashes)` (a multiple of 8 bytes, so it never collides in
length with a leaf input). H is BLAKE2s truncated to 8 bytes.
`tflag[i] & SW_COMPLETE` marks a subtree as done, and `SW_GOAL_DONE(n)` does
the same for the whole goal. Equal root hashes between two drones mean they
agree on the entire checklist.

### Synchronisation

There are three paths, cheapest first:

1. **Change log**: every record change goes into a ring (`SW_LOG`). Each
   link has a cursor and receives the changes it hasn't seen, except changes
   that came from that neighbour. Fresh claims flood within one frame per
   hop. A new link starts at the oldest entry still in the ring.
2. **Merkle descent**: when a neighbour's last advertised root differs from
   ours, we send **probes**. A probe for node P contains P's child hashes,
   with full records for leaf children. The receiver merges those records.
   It queues back the leaves where its own record wins, and the internal
   children whose hash differs. The two sides alternate one tree level per
   frame and descend into every mismatched branch at once. The queue
   persists across frames, so beacons and unequal send rates cannot reset
   the descent.
3. **Sweep**: on a link with no fresh feedback, we cycle through the nodes
   that have leaf children and fill any spare MTU. "No fresh feedback"
   means no unicast from that neighbour within the TTL; beacons don't
   count. A typical case is a one-way edge caused by unequal ranges.

Measured: 46 divergent leaves are repaired in 3 to 5 round trips over a
two-way link. That holds for both tested trees (depth 3 and depth 4), with
or without beacons between unicasts, and at 1:1 or 2:1 send rates. Over a
one-way link, 48 leaves are repaired in 3 frames.

## Wire format (little-endian, version 1)

Frame:

| bytes | field |
|---|---|
| 1 | version = 1 |
| 1 | number of probes (0 in a beacon) |
| 4 | mission id |
| 2 | src id |
| 2 | dst id (`0xFFFF` = beacon) |
| 8 | sender's root hash |
| 43+ | sender's context, hops = 0 |
| per probe | node u16, then per child: hash (8) if internal, record (9) if leaf |
| 1 + 11·n | record count, then (leaf u16, record) |
| 1 + … | context count, then relayed contexts (every known one except dst's) |
| 8 | BLAKE2s-64 tag, keyed with the mission key, over everything above |

Context (43 bytes plus the placeholders):

| bytes | field |
|---|---|
| 2 | id |
| 1 | hops (relays so far) |
| 4 | stamp, ms since mission epoch |
| 2 | range, m |
| 2 | mtu |
| 3×2 | θ, 1e-4 rad |
| 3×2 | ν = J⁻¹v, cm/s |
| 3×4 | ρ = J⁻¹p, mm |
| 3×2 | σ attitude 1e-4 rad, velocity cm/s, position cm (0xFFFF unknown) |
| 1 + n | op info length, bytes |
| 1 + n | context length, bytes |

Record (9 bytes): `st u8, owner u16, cost u16, ver u32`.

`SW_FRAME_MIN` (180 bytes by default) is the smallest MTU that always fits
the own context, the largest probe and one record, so every frame makes
progress. Mesh radios with a 255-byte payload work as-is.

## Use

```c
static sw_node node;                         /* 8.7 KB, no heap */
sw_init(&node, my_id, range_m, mtu, cnt, ntask, key, keylen);

for (;;) {                                   /* e.g. 10 Hz */
    sw_set_self(&node, &estimator_output, now);
    m = sw_tick(&node, now, nbr, MAXN);      /* maintenance + out-neighbours */
    for (i = 0; i < m; i++)
        if ((len = sw_encode(&node, nbr[i], now, buf, sizeof buf)) > 0)
            radio_send(nbr[i], buf, len);
    if (m == 0 || beacon_due)
        radio_send_all(buf, sw_encode(&node, SW_BCAST, now, buf, sizeof buf));
    while (radio_recv(buf, &len))
        sw_decode(&node, buf, len, now);
    /* planner: sw_task(&node, leaf, SW_CLAIMED, cost, now) -> work ->
       sw_task(&node, leaf, SW_DONE, cost, now); give up when
       node.rec[leaf].owner != my_id */
}
```

`now` is milliseconds since the mission epoch on the GNSS-disciplined clock
that all drones share.

## How the three papers shape the protocol

- **Scheiber, Fornasier, Brommer, Weiss, "Revisiting multi-GNSS Navigation
  for UAVs – An Equivariant Filtering Approach"** (ICAR 2023,
  arXiv:2310.10597). GNSS-INS navigation is filtered on a symmetry group of
  the extended-pose state. We send that state as Lie-algebra coordinates, so
  every receiver computes in a vector space, and we use GNSS time as the
  swarm's shared clock for stamps and leases.
- **Brommer, Fornasier, Steinbrener, Weiss, "Sensor Model Identification via
  Simultaneous Model Selection and State Variable Determination"**
  (arXiv:2506.11263). Which state variables, calibration states and
  reference frames a drone carries is decided at runtime. That is why the
  context field is an opaque, bounded, self-describing placeholder rather
  than a fixed schema.
- **Raab, Weiss, Fornasier, Brommer, Ibrahim, "Consistent Pose Estimation of
  Unmanned Ground Vehicles through Terrain-Aided Multi-Sensor Fusion on
  Geometric Manifolds"** (IROS 2025, arXiv:2508.14661). Minimal manifold
  parameterisation and filter consistency. We use minimal tangent
  coordinates (9 numbers for SE_2(3)) and keep the shared uncertainty
  consistent: rounded up, inflated by quantisation, prediction time and
  unmodelled acceleration, and gated at Kσ.

## Risk register

These were found by iterating build → sanitiser tests → fuzz → simulation →
independent adversarial review. Every review finding has a regression test.
For the descent livelock, the test was confirmed to fail with the old code.

| Risk | Mitigation | Evidence |
|---|---|---|
| Out-of-memory, fragmentation | No heap. All state is in `sw_node`, sized at compile time. Limits are checked by negative-array typedefs. No recursion (Merkle rebuild is one descending pass) | tiny-table config passes |
| Malformed or hostile fields (bad lengths, indices, states, θ range, mtu, NaN/Inf input) | Every field is validated before use. The tag is checked before parsing. Local input is checked for finite values, unit quaternion and representable range | 200k mutated, re-tagged frames per config under ASan/UBSan |
| Forged frames (for example a fake "goal done") | Keyed BLAKE2s tag. The mission id binds key, tree and build options | `SW_EAUTH` / `SW_EMISSION` tests |
| Replay, reordering, duplicates | CRDT merge is idempotent and stamps only move forward | replay test |
| Frame rejected halfway | Merges are monotone, so an applied prefix is still a valid state | fuzz invariants |
| Duplicate work | Total-order conflict resolution, claims flood via the change log, losers detect ownership change | simulation: 0 duplicate completions, about 30 early abandons |
| Drone dies holding a task | Lease with half-life renewal | simulation: leaf taken over 1 ms after lease expiry |
| Partition heal, log overflow | Merkle descent with a persistent per-link probe queue | 3 to 5 rounds for 46 divergent leaves, across schedules and tree depths |
| Descent livelock from beacons or unequal send rates (review finding) | The queue is no longer cleared on arrival; only unicasts mark a link fresh | `heal()` schedules; fails if the old behaviour is restored |
| Replayed old frames passing the gate (review finding) | Sender context age is bounded by the TTL | replay-after-TTL test |
| Overflow to NaN inside J⁻¹ on huge finite input (review finding) | NaN-safe range check in quantisation | 1e300 / 3e38 input test |
| Asymmetric ranges (one-way edges) | Sweep fallback | 3 frames |
| More drones or neighbours than the tables hold | Farthest-then-oldest eviction for contexts; LRU for links, and a new link replays the ring | `SW_MAX_NODES=6, SW_MAX_NBR=3, SW_LOG=4` converges |
| SE(3) quantisation coupling | θ quantised first, J⁻¹ evaluated at the quantised θ, residual folded into σ | 0.7 mm at 50 km; error ≤ reported σ always |
| Overconfident gating | Sigmas rounded up, unknown saturates, inflated with age | Lie tests |
| Clock skew, wrap | Future stamps beyond `SW_SKEW_MS` rejected; 49-day horizon; `SW_ETIME` on version exhaustion | stale-stamp test |
| Endianness, alignment, int width | Byte-wise LE codec, no struct casts, types chosen from `limits.h` with `#error` | C89 / C++98 / float builds |

### Known limits (not yet covered)

- **Replay inside the TTL window.** An outsider can replay a captured frame
  for up to `SW_TTL_MS`. That can't corrupt records or contexts, because
  merges only move forward, but it can keep a link looking fresh. Strict
  protection would need a per-link monotonic sequence in the header.
- **GNSS-denied time.** Leases and TTLs assume a shared clock. Without GNSS
  the replicas still converge, but lease fairness degrades with drift.
- **Duplicate drone ids** are not detectable in general. `SW_EADDR` only
  catches a frame carrying our own id.
- **Insiders.** The tag authenticates membership, not honesty. A drone that
  holds the key can lie about its pose or mark tasks done.
- **Airtime.** Per-neighbour unicast costs one frame per neighbour per
  tick. Tune the tick rate to the radio budget.
- **Static goal tree.** Re-planning means a new mission id.
- **Outbidding** can ping-pong if cost estimates oscillate. Add hysteresis
  in the planner.
- **Attitude sigma** is carried for consumers but not used by the protocol
  itself.
