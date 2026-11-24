"""Python bindings for swarm, a decentralised communication protocol for UAV
swarms. The protocol itself is the C library in this repository; see
README.md for the model and include/swarm.h for the C API.

Each method calls one ``sw_*`` function. As in C, every ``now`` is
milliseconds since the mission epoch on the clock the swarm shares, as an
unsigned 32-bit int. A negative C result raises :class:`SwarmError`, whose
``code`` is that result (``EREACH``, ``ESLOT``, ...).

Example::

    import swarm_protocol as sw

    tree = [2, 0, 0]                      # goal with two leaf tasks
    a = sw.Node(1, range=500, mtu=255, tree=tree, key=b"mission key")
    a.set_self(now, p=(0.0, 0.0, 50.0), sig=(0.01, 0.1, 2.0))
    for dst in a.tick(now):
        radio_send(dst, a.encode(dst, now))
"""

from __future__ import annotations

import math
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple, Union

from . import _swarm

__version__ = "0.1.0"

__all__ = [
    "Node", "Context", "Record", "SwarmError",
    "OPEN", "CLAIMED", "DONE", "BCAST", "VERSION", "FRAME_MIN",
    "MAX_NODES", "MAX_NBR", "MAX_TASK", "FANOUT", "LOG", "OP_MAX", "CTX_MAX",
    "TTL_MS", "LEASE_MS", "SLOT_MS", "SKEW_MS", "NODE_BYTES",
    "OK", "EARG", "ESHORT", "EVER", "EMISSION", "EADDR", "EAUTH", "EFIELD",
    "ESTALE", "EREACH", "EBUSY", "EDONE", "ETIME", "ESLOT",
]

# Values come from the compiled library, so they always match its build.
OPEN: int = _swarm.OPEN
CLAIMED: int = _swarm.CLAIMED
DONE: int = _swarm.DONE
BCAST: int = _swarm.BCAST          # destination id of a discovery beacon
VERSION: int = _swarm.VERSION      # wire protocol version
FRAME_MIN: int = _swarm.FRAME_MIN  # smallest usable mtu
MAX_NODES: int = _swarm.MAX_NODES  # other drones remembered
MAX_NBR: int = _swarm.MAX_NBR      # link slots
MAX_TASK: int = _swarm.MAX_TASK    # goal-tree nodes
FANOUT: int = _swarm.FANOUT        # children per tree node
LOG: int = _swarm.LOG              # change-log ring size
OP_MAX: int = _swarm.OP_MAX        # operation-info placeholder bytes
CTX_MAX: int = _swarm.CTX_MAX      # context placeholder bytes
TTL_MS: int = _swarm.TTL_MS
LEASE_MS: int = _swarm.LEASE_MS
SLOT_MS: int = _swarm.SLOT_MS
SKEW_MS: int = _swarm.SKEW_MS
NODE_BYTES: int = _swarm.NODE_BYTES  # sizeof(sw_node)

OK: int = _swarm.OK
EARG: int = _swarm.EARG
ESHORT: int = _swarm.ESHORT
EVER: int = _swarm.EVER
EMISSION: int = _swarm.EMISSION
EADDR: int = _swarm.EADDR
EAUTH: int = _swarm.EAUTH
EFIELD: int = _swarm.EFIELD
ESTALE: int = _swarm.ESTALE
EREACH: int = _swarm.EREACH
EBUSY: int = _swarm.EBUSY
EDONE: int = _swarm.EDONE
ETIME: int = _swarm.ETIME
ESLOT: int = _swarm.ESLOT

_ERRORS = {
    EARG: ("EARG", "bad argument or tree"),
    ESHORT: ("ESHORT", "buffer or frame too small"),
    EVER: ("EVER", "unknown protocol version"),
    EMISSION: ("EMISSION", "other mission"),
    EADDR: ("EADDR", "not addressed to us, or from our own id"),
    EAUTH: ("EAUTH", "tag mismatch"),
    EFIELD: ("EFIELD", "malformed field"),
    ESTALE: ("ESTALE", "sender context too old or in the future"),
    EREACH: ("EREACH", "peer is not a neighbour on the reach graph"),
    EBUSY: ("EBUSY", "leaf held by a cheaper live claim"),
    EDONE: ("EDONE", "leaf already done"),
    ETIME: ("ETIME", "version space exhausted"),
    ESLOT: ("ESLOT", "all link slots held; retry next tick"),
}

# sw_set_self rejects inf; any sigma this large is sent as "unknown"
_UNKNOWN = 1e9

Vec3 = Tuple[float, float, float]
Bytes = Union[bytes, bytearray, memoryview]


class SwarmError(Exception):
    """A swarm call failed. ``code`` is the negative C result, ``name`` its
    short name (``"EREACH"``, ``"ESLOT"``, ...).

    Receivers drop frames routinely (out of range, other mission, stale), so
    catch this around :meth:`Node.decode`. On :meth:`Node.encode`,
    ``ESLOT`` means "retry next tick".
    """

    def __init__(self, code: int) -> None:
        super().__init__(code)
        self.code = code
        self.name = _ERRORS.get(code, ("E?", ""))[0]

    def __str__(self) -> str:
        name, text = _ERRORS.get(self.code, ("E?", "unknown error"))
        return f"{name} ({self.code}): {text}"


def _check(rc: int) -> None:
    if rc < 0:
        raise SwarmError(rc)


class Record(NamedTuple):
    """Replicated record of one leaf task."""

    state: int  #: OPEN, CLAIMED or DONE
    owner: int  #: id of the drone whose action this is
    cost: int   #: claim bid; lower wins
    ver: int    #: claim: lease expiry (ms); otherwise time of the action


class Context(NamedTuple):
    """One drone's context as the swarm sees it (decoded from the wire)."""

    id: int
    hops: int     #: hop distance, 1 = heard directly; 0 for our own context
    stamp: int    #: time of the estimate, ms since mission epoch
    range: int    #: radio reach, m
    mtu: int      #: largest frame it accepts
    p: Vec3       #: position in the mission frame (ENU at anchor), m
    v: Vec3       #: velocity in the mission frame, m/s
    theta: Vec3   #: attitude as rotation vector Log(R), body -> mission, rad
    sigma_att: float  #: conservative 1-sigma, rad (inf if unknown)
    sigma_vel: float  #: conservative 1-sigma, m/s (inf if unknown)
    sigma_pos: float  #: conservative 1-sigma, m (inf if unknown)
    op: bytes     #: operation-info placeholder
    ctx: bytes    #: context placeholder

    @property
    def q(self) -> Tuple[float, float, float, float]:
        """Attitude as a Hamilton quaternion (w, x, y, z)."""
        t = math.sqrt(sum(x * x for x in self.theta))
        if t < 1e-12:
            return (1.0, 0.0, 0.0, 0.0)
        s = math.sin(t / 2) / t
        return (math.cos(t / 2), self.theta[0] * s, self.theta[1] * s,
                self.theta[2] * s)

    def predict(self, now: int) -> Vec3:
        """Position at ``now`` by constant velocity, as the protocol does."""
        dt = max(0, now - self.stamp) * 1e-3
        return (self.p[0] + self.v[0] * dt, self.p[1] + self.v[1] * dt,
                self.p[2] + self.v[2] * dt)


def _ctx(t: tuple) -> Context:
    return Context(*t)


def _sig(x: Optional[float]) -> float:
    if x is None or not math.isfinite(x) or x > _UNKNOWN:
        return _UNKNOWN
    return float(x)


class Node:
    """One drone's protocol state (``sw_node``, fixed size, no heap).

    ``tree`` is the goal tree in breadth-first order: ``tree[i]`` is the
    number of children of node ``i``, 0 for a leaf. Node 0 is the goal. The
    tree and ``key`` define the mission and must be identical on every drone.
    """

    __slots__ = ("_n", "_tree")

    def __init__(self, id: int, range: int, mtu: int,
                 tree: Union[Sequence[int], bytes], key: Union[Bytes, str]):
        if isinstance(key, str):
            key = key.encode()
        self._tree = bytes(tree)
        self._n = _swarm.Node()
        _check(self._n.init(id, range, mtu, self._tree, bytes(key)))

    def __repr__(self) -> str:
        c = self.context
        return (f"<swarm_protocol.Node id={c.id} range={c.range} "
                f"peers={len(self._n.peers())} tasks={len(self._tree)}>")

    # ---- own state ------------------------------------------------------

    def set_self(self, now: int, p: Sequence[float],
                 v: Sequence[float] = (0.0, 0.0, 0.0),
                 q: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
                 sig: Optional[Sequence[Optional[float]]] = None,
                 op: Bytes = b"", ctx: Bytes = b"") -> None:
        """Publish this drone's estimate. Call at least every ``TTL_MS``.

        ``p``, ``v``: position (m) and velocity (m/s) in the mission frame.
        ``q``: attitude quaternion (w, x, y, z), body -> mission.
        ``sig``: conservative 1-sigma (attitude rad, velocity m/s, position
        m); ``None`` or a ``None`` entry means unknown, e.g. no GNSS fix.
        ``op``, ``ctx``: placeholders, at most ``OP_MAX`` / ``CTX_MAX`` bytes.
        """
        s = (_UNKNOWN,) * 3 if sig is None else tuple(_sig(x) for x in sig)
        _check(self._n.set_self(tuple(q), tuple(v), tuple(p), s, bytes(op),
                                bytes(ctx), now))

    @property
    def context(self) -> Context:
        """Our own context as it goes on the wire."""
        return _ctx(self._n.context())

    @property
    def id(self) -> int:
        return self.context.id

    @property
    def mission(self) -> int:
        """32-bit mission id derived from the key, tree and build options."""
        return self._n.mission()

    # ---- the loop -------------------------------------------------------

    def tick(self, now: int) -> List[int]:
        """Maintenance; returns the ids of current out-neighbours, in a
        rotating order. Call once per cycle, before encoding."""
        return self._n.tick(now)

    def encode(self, dst: int, now: int, cap: Optional[int] = None) -> bytes:
        """Frame for out-neighbour ``dst``, at most ``cap`` bytes (default:
        the smaller mtu). Raises ``ESLOT`` when no link slot is free (retry
        next tick) and ``EREACH`` when ``dst`` is not an out-neighbour."""
        out = self._n.encode(dst, now, 0xFFFF if cap is None else cap)
        if isinstance(out, int):
            raise SwarmError(out)
        return out

    def beacon(self, now: int) -> bytes:
        """Discovery beacon for everyone in range (``encode(BCAST, now)``)."""
        return self.encode(BCAST, now)

    def decode(self, frame: Bytes, now: int) -> None:
        """Merge a received frame. Raises SwarmError if it is rejected."""
        _check(self._n.decode(bytes(frame), now))

    # ---- tasks ----------------------------------------------------------

    def claim(self, leaf: int, now: int, cost: int = 0, lease: int = 0) -> None:
        """Claim a leaf task with bid ``cost`` (lower wins) for ``lease`` ms
        (0: ``LEASE_MS``). Raises ``EBUSY`` if a cheaper live claim holds it
        and ``EDONE`` if it is finished. Claim again to extend the lease."""
        _check(self._n.task(leaf, CLAIMED, cost, lease, now))

    def release(self, leaf: int, now: int) -> None:
        """Give up our own claim on a leaf."""
        _check(self._n.task(leaf, OPEN, 0, 0, now))

    def done(self, leaf: int, now: int, cost: int = 0) -> None:
        """Mark a leaf task done (final)."""
        _check(self._n.task(leaf, DONE, cost, 0, now))

    def leaf(self, leaf: int) -> Record:
        """Current record of a leaf task."""
        rc, st, owner, cost, ver = self._n.leaf(leaf)
        _check(rc)
        return Record(st, owner, cost, ver)

    def records(self) -> Dict[int, Record]:
        """Records of all leaves, by leaf index."""
        return {i: self.leaf(i) for i in self.leaves}

    def complete(self, i: int) -> bool:
        """Whether subtree ``i`` is done; current after :meth:`tick`."""
        return self._n.complete(i)

    @property
    def goal_done(self) -> bool:
        """Whether the whole goal is done; current after :meth:`tick`."""
        return self._n.complete(0)

    @property
    def root_hash(self) -> bytes:
        """Merkle root of the goal tree; current after :meth:`tick`. Equal
        roots on two drones mean they agree on every record."""
        return self._n.root()

    # ---- tree shape -----------------------------------------------------

    @property
    def tree(self) -> Tuple[int, ...]:
        return tuple(self._tree)

    @property
    def leaves(self) -> List[int]:
        return [i for i, c in enumerate(self._tree) if c == 0]

    def children(self, i: int) -> range:
        first = 1 + sum(self._tree[:i])
        return range(first, first + self._tree[i])

    # ---- other drones ---------------------------------------------------

    def peers(self) -> List[Context]:
        """Every drone we know of, directly or through relays, by id."""
        return [_ctx(t) for t in self._n.peers()]
