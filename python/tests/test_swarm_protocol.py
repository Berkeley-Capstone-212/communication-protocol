"""Tests for the Python binding. Run: python -m unittest discover -s python/tests"""

import math
import pickle
import unittest

import swarm_protocol as sw

# same trees and key as test/test_swarm.c
TREE = [4, 3, 3, 3, 3] + [4] * 12 + [0] * 48
KEY = b"mission-key-0001\0"
SIG = (0.01, 0.1, 2.0)


def yaw_q(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def pair(dist, tree=TREE):
    """a (500 m) and b (300 m) at distance dist, as pair() in the C tests."""
    a = sw.Node(1, range=500, mtu=255, tree=tree, key=KEY)
    b = sw.Node(2, range=300, mtu=255, tree=tree, key=KEY)
    a.set_self(1000, p=(0, 0, 50), sig=SIG)
    b.set_self(1000, p=(dist, 0, 50), q=yaw_q(1), sig=SIG)
    return a, b


def code(fn, *args, **kw):
    try:
        fn(*args, **kw)
    except sw.SwarmError as e:
        return e.code
    return sw.OK


class Constants(unittest.TestCase):
    def test_values(self):
        self.assertEqual((sw.OPEN, sw.CLAIMED, sw.DONE), (0, 1, 2))
        self.assertEqual(sw.BCAST, 0xFFFF)
        self.assertEqual(sw.FRAME_MIN, 185)
        self.assertEqual(sw.MAX_NODES, 32)

    def test_error(self):
        e = sw.SwarmError(sw.EREACH)
        self.assertEqual((e.code, e.name), (sw.EREACH, "EREACH"))
        self.assertIn("reach graph", str(e))
        e2 = pickle.loads(pickle.dumps(e))
        self.assertEqual((e2.code, e2.name), (e.code, e.name))


class Init(unittest.TestCase):
    def test_bad_tree_or_mtu(self):
        self.assertEqual(code(sw.Node, 1, 500, 255, [0], KEY), sw.EARG)
        self.assertEqual(code(sw.Node, 1, 500, 255, [2, 0], KEY), sw.EARG)
        self.assertEqual(code(sw.Node, 1, 500, 100, TREE, KEY), sw.EARG)
        self.assertEqual(code(sw.Node, sw.BCAST, 500, 255, TREE, KEY), sw.EARG)
        self.assertEqual(code(sw.Node, 1, 500, 255, [1] + [0] * 999, KEY),
                         sw.EARG)

    def test_int_ranges(self):
        with self.assertRaises(OverflowError):
            sw.Node(70000, 500, 255, TREE, KEY)
        with self.assertRaises(OverflowError):
            sw.Node(-1, 500, 255, TREE, KEY)
        with self.assertRaises(TypeError):
            sw.Node(1.5, 500, 255, TREE, KEY)

    def test_str_key_and_tree_shape(self):
        n = sw.Node(7, 500, 255, TREE, "mission-key-0001\0")
        self.assertEqual(n.mission, sw.Node(1, 500, 255, TREE, KEY).mission)
        self.assertNotEqual(n.mission, sw.Node(1, 500, 255, TREE, b"x").mission)
        self.assertEqual(len(n.leaves), 48)
        self.assertEqual(list(n.children(0)), [1, 2, 3, 4])
        self.assertEqual(list(n.children(1)), [5, 6, 7])
        self.assertEqual(n.id, 7)
        self.assertIn("id=7", repr(n))


class Frames(unittest.TestCase):
    """Mirrors test_frames() in test/test_swarm.c."""

    def test_directed_reach(self):
        a, b = pair(400)  # a reaches b, b cannot reach a
        self.assertEqual(code(a.encode, 2, 1000), sw.EREACH)  # not known yet
        b.decode(a.beacon(1000), 1000)
        self.assertEqual(code(a.decode, b.beacon(1000), 1000), sw.EREACH)
        self.assertEqual(code(b.encode, 1, 1000), sw.EREACH)

    def test_claims_and_errors(self):
        a, b = pair(200)
        a.decode(b.beacon(1000), 1000)
        self.assertEqual(a.tick(1000), [2])
        self.assertEqual(code(a.encode, 2, 1000, cap=100), sw.ESHORT)
        a.claim(20, 1000, cost=7)
        f = a.encode(2, 1000)
        self.assertLessEqual(len(f), 255)
        b.decode(f, 1000)
        self.assertEqual(b.leaf(20)[:3], (sw.CLAIMED, 1, 7))
        self.assertEqual(code(b.claim, 20, 1100, cost=9), sw.EBUSY)
        b.claim(20, 1100, cost=3)  # outbid
        b.release(20, 1200)
        self.assertEqual(code(b.release, 20, 1200), sw.EARG)
        self.assertEqual(code(b.claim, 4, 1200), sw.EARG)  # not a leaf
        self.assertEqual(code(b.leaf, 4), sw.EARG)
        b.decode(f, 1000)  # replay: harmless
        self.assertEqual(b.leaf(20).state, sw.OPEN)
        b.claim(22, 1200, cost=1, lease=300000)
        self.assertEqual(b.leaf(22).ver, 301200)
        b.done(22, 1300)
        self.assertEqual(code(b.claim, 22, 1400), sw.EDONE)

        bad = bytearray(f)
        bad[30] ^= 1
        self.assertEqual(code(b.decode, bad, 1000), sw.EAUTH)
        self.assertEqual(code(b.decode, f[:-1], 1000), sw.EAUTH)
        self.assertEqual(code(a.decode, f, 1000), sw.EADDR)  # own frame
        self.assertEqual(code(b.decode, f[:20], 1000), sw.ESHORT)
        x = sw.Node(3, 500, 255, [2, 0, 0], KEY)
        self.assertEqual(code(x.decode, f, 1000), sw.EMISSION)
        self.assertEqual(code(b.decode, f, 1000 + sw.TTL_MS + 1), sw.ESTALE)

    def test_set_self_validation(self):
        a, _ = pair(200)
        with self.assertRaises(TypeError):
            a.set_self(1000, p=(0, 0))
        self.assertEqual(code(a.set_self, 1000, p=(0, 0, math.nan)), sw.EARG)
        self.assertEqual(code(a.set_self, 1000, p=(0, 0, 0), q=(2, 0, 0, 0)),
                         sw.EARG)
        self.assertEqual(code(a.set_self, 1000, p=(0, 0, 0),
                              op=bytes(sw.OP_MAX + 1)), sw.EARG)
        with self.assertRaises(OverflowError):
            a.set_self(-1, p=(0, 0, 0))


class Contexts(unittest.TestCase):
    def test_pose_round_trip(self):
        a = sw.Node(1, 60000, 255, TREE, KEY)
        b = sw.Node(2, 60000, 255, TREE, KEY)
        p, v = (41234.5678, -20000.25, 312.75), (12.34, -5.67, 0.89)
        a.set_self(5000, p=p, v=v, q=yaw_q(2.5), sig=SIG,
                   op=b"\x01\x02", ctx=b"hello")
        b.set_self(5000, p=(40000, -20000, 300), sig=(None, None, 1.0))
        b.decode(a.beacon(5000), 5000)
        (c,) = b.peers()
        self.assertEqual((c.id, c.hops, c.stamp, c.range), (1, 1, 5000, 60000))
        for i in range(3):
            self.assertAlmostEqual(c.p[i], p[i], delta=0.002)
            self.assertAlmostEqual(c.v[i], v[i], delta=0.02)
        self.assertAlmostEqual(c.theta[2], 2.5, delta=1e-4)
        self.assertAlmostEqual(c.q[3], math.sin(1.25), delta=1e-4)
        self.assertGreaterEqual(c.sigma_pos, 2.0)
        self.assertGreaterEqual(c.sigma_vel, 0.1)
        self.assertEqual((c.op, c.ctx), (b"\x01\x02", b"hello"))
        self.assertAlmostEqual(c.predict(6000)[0], c.p[0] + c.v[0], delta=1e-9)
        own = b.context
        self.assertEqual((own.id, own.hops), (2, 0))
        self.assertTrue(math.isinf(own.sigma_att))
        self.assertTrue(math.isinf(own.sigma_vel))
        self.assertGreaterEqual(own.sigma_pos, 1.0)


class Swarm(unittest.TestCase):
    """Six drones in a line, 150 m apart, 200 m radios: a multi-hop swarm."""

    def run_swarm(self, nodes, pos, t0, steps, dt=100):
        rng = {n.id: n.context.range for n in nodes}
        for step in range(steps):
            now = t0 + step * dt
            frames = []
            for n in nodes:
                n.set_self(now, p=pos[n.id], sig=(0.01, 0.1, 1.0))
            for n in nodes:
                nbrs = n.tick(now)
                for d in nbrs:
                    try:
                        frames.append((n.id, d, n.encode(d, now)))
                    except sw.SwarmError as e:
                        self.assertEqual(e.code, sw.ESLOT)
                if not nbrs or step % 10 == 0:
                    frames.append((n.id, sw.BCAST, n.beacon(now)))
            for src, dst, f in frames:
                for r in nodes:
                    if r.id == src or (dst != sw.BCAST and dst != r.id):
                        continue
                    if math.dist(pos[src], pos[r.id]) > rng[src]:
                        continue  # out of radio range
                    try:
                        r.decode(f, now)
                    except sw.SwarmError:
                        pass
        return t0 + steps * dt

    def test_converges(self):
        nodes = [sw.Node(i, 200, 255, TREE, KEY) for i in range(1, 7)]
        pos = {i: (150.0 * i, 0.0, 50.0) for i in range(1, 7)}
        leaves = nodes[0].leaves
        now = self.run_swarm(nodes, pos, 1000, 20)
        # every drone knows every other one, the far ones through relays
        far = {c.id: c.hops for c in nodes[0].peers()}
        self.assertEqual(sorted(far), [2, 3, 4, 5, 6])
        self.assertEqual(far[2], 1)
        self.assertGreater(far[6], 1)
        # each drone claims a disjoint pair and finishes one of them
        for k, n in enumerate(nodes):
            n.claim(leaves[2 * k], now, cost=k)
            n.claim(leaves[2 * k + 1], now, cost=k)
            n.done(leaves[2 * k], now)
        now = self.run_swarm(nodes, pos, now, 30)
        roots = {n.root_hash for n in nodes}
        self.assertEqual(len(roots), 1)
        recs = nodes[-1].records()
        for k, n in enumerate(nodes):
            self.assertEqual(recs[leaves[2 * k]].state, sw.DONE)
            self.assertEqual(recs[leaves[2 * k + 1]][:2], (sw.CLAIMED, n.id))
        self.assertFalse(nodes[0].goal_done)
        # finish everything from one drone: the goal completes everywhere
        for leaf in leaves:
            if nodes[0].leaf(leaf).state != sw.DONE:
                nodes[0].done(leaf, now)
        self.run_swarm(nodes, pos, now, 60)
        self.assertTrue(all(n.goal_done for n in nodes))


if __name__ == "__main__":
    unittest.main()
