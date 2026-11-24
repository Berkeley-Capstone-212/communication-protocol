/*
 * _swarm.c - CPython binding for the swarm library (include/swarm.h).
 *
 * Thin by design: each method calls one sw_* function and returns its
 * result code (or bytes / tuples on success). swarm_protocol/__init__.py
 * turns codes into SwarmError and tuples into named records. Built against
 * the stable ABI (Py_LIMITED_API) when the interpreter supports it.
 */
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <math.h>
#include <string.h>

#include "swarm.h"

typedef struct {
    PyObject_HEAD
    int ready;
    sw_node node;
} NodeObject;

/* ---- argument converters for "O&": ints with an explicit range -------- */

static int conv_uint(PyObject *o, unsigned long lim, unsigned long *out)
{
    PyObject *i = PyNumber_Index(o);
    unsigned long v;

    if (!i)
        return 0;
    v = PyLong_AsUnsignedLong(i);
    Py_DECREF(i);
    if (v == (unsigned long)-1 && PyErr_Occurred())
        return 0;
    if (v > lim) {
        PyErr_Format(PyExc_OverflowError, "%lu is out of range 0..%lu", v,
                     lim);
        return 0;
    }
    *out = v;
    return 1;
}

static int conv_u16(PyObject *o, void *p)
{
    unsigned long v;

    if (!conv_uint(o, 0xFFFFUL, &v))
        return 0;
    *(sw_u16 *)p = (sw_u16)v;
    return 1;
}

static int conv_u32(PyObject *o, void *p)
{
    unsigned long v;

    if (!conv_uint(o, 0xFFFFFFFFUL, &v))
        return 0;
    *(sw_u32 *)p = (sw_u32)v;
    return 1;
}

static int not_ready(NodeObject *self)
{
    if (self->ready)
        return 0;
    PyErr_SetString(PyExc_RuntimeError, "node is not initialised");
    return 1;
}

/* sigma in SI units; the wire's 0xFFFF (unknown) becomes +inf */
static double sigma(unsigned raw, double scale)
{
    return raw == 0xFFFF ? HUGE_VAL : (double)raw * scale;
}

static PyObject *ctx_tuple(const sw_ctx *c)
{
    return Py_BuildValue(
        "(IIkII(ddd)(ddd)(ddd)ddd" "y#y#)",
        (unsigned)c->id, (unsigned)c->hops, (unsigned long)c->stamp,
        (unsigned)c->range, (unsigned)c->mtu,
        (double)c->p[0], (double)c->p[1], (double)c->p[2],
        (double)c->v[0], (double)c->v[1], (double)c->v[2],
        c->th[0] * 1e-4, c->th[1] * 1e-4, c->th[2] * 1e-4,
        sigma(c->sig[0], 1e-4),
        c->sig[1] == 0xFFFF ? HUGE_VAL : (double)c->sv,
        c->sig[2] == 0xFFFF ? HUGE_VAL : (double)c->sp,
        (const char *)c->op, (Py_ssize_t)c->op_len,
        (const char *)c->ctx, (Py_ssize_t)c->ctx_len);
}

/* ---- methods ----------------------------------------------------------- */

static PyObject *node_init(NodeObject *self, PyObject *args)
{
    sw_u16 id, range, mtu;
    const char *cnt, *key;
    Py_ssize_t ncnt, nkey;
    int r;

    if (!PyArg_ParseTuple(args, "O&O&O&y#y#:init", conv_u16, &id, conv_u16,
                          &range, conv_u16, &mtu, &cnt, &ncnt, &key, &nkey))
        return NULL;
    if ((size_t)nkey > UINT_MAX) {
        PyErr_SetString(PyExc_ValueError, "key too long");
        return NULL;
    }
    self->ready = 0;
    if (ncnt > SW_MAX_TASK)
        return PyLong_FromLong(SW_EARG);
    r = sw_init(&self->node, id, range, mtu, (const sw_u8 *)cnt,
                (unsigned)ncnt, (const sw_u8 *)key, (unsigned)nkey);
    self->ready = r == SW_OK;
    return PyLong_FromLong(r);
}

static PyObject *node_set_self(NodeObject *self, PyObject *args)
{
    double q[4], v[3], p[3], sg[3];
    const char *op, *ctx;
    Py_ssize_t nop, nctx;
    sw_u32 now;
    sw_state s;
    unsigned i;

    if (not_ready(self) ||
        !PyArg_ParseTuple(args, "(dddd)(ddd)(ddd)(ddd)y#y#O&:set_self",
                          &q[0], &q[1], &q[2], &q[3], &v[0], &v[1], &v[2],
                          &p[0], &p[1], &p[2], &sg[0], &sg[1], &sg[2], &op,
                          &nop, &ctx, &nctx, conv_u32, &now))
        return NULL;
    if (nop > SW_OP_MAX || nctx > SW_CTX_MAX)
        return PyLong_FromLong(SW_EARG);
    memset(&s, 0, sizeof s);
    for (i = 0; i < 4; i++)
        s.q[i] = (sw_real)q[i];
    for (i = 0; i < 3; i++) {
        s.v[i] = (sw_real)v[i];
        s.p[i] = (sw_real)p[i];
        s.sig[i] = (sw_real)sg[i];
    }
    s.op = (const sw_u8 *)op;
    s.op_len = (unsigned)nop;
    s.ctx = (const sw_u8 *)ctx;
    s.ctx_len = (unsigned)nctx;
    return PyLong_FromLong(sw_set_self(&self->node, &s, now));
}

static PyObject *node_task(NodeObject *self, PyObject *args)
{
    sw_u16 leaf, st, cost;
    sw_u32 lease, now;

    if (not_ready(self) ||
        !PyArg_ParseTuple(args, "O&O&O&O&O&:task", conv_u16, &leaf, conv_u16,
                          &st, conv_u16, &cost, conv_u32, &lease, conv_u32,
                          &now))
        return NULL;
    return PyLong_FromLong(sw_task(&self->node, leaf, st, cost, lease, now));
}

static PyObject *node_leaf(NodeObject *self, PyObject *args)
{
    sw_u16 leaf;
    sw_rec r;
    int rc;

    if (not_ready(self) ||
        !PyArg_ParseTuple(args, "O&:leaf", conv_u16, &leaf))
        return NULL;
    memset(&r, 0, sizeof r);
    rc = sw_leaf(&self->node, leaf, &r);
    return Py_BuildValue("(iIIIk)", rc, r.st, (unsigned)r.owner,
                         (unsigned)r.cost, (unsigned long)r.ver);
}

static PyObject *node_tick(NodeObject *self, PyObject *args)
{
    sw_u16 nbr[SW_MAX_NODES];
    sw_u32 now;
    unsigned i, k;
    PyObject *list, *id;

    if (not_ready(self) || !PyArg_ParseTuple(args, "O&:tick", conv_u32, &now))
        return NULL;
    k = sw_tick(&self->node, now, nbr, SW_MAX_NODES);
    list = PyList_New((Py_ssize_t)k);
    if (!list)
        return NULL;
    for (i = 0; i < k; i++) {
        id = PyLong_FromLong((long)nbr[i]);
        if (!id) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, id);
    }
    return list;
}

static PyObject *node_encode(NodeObject *self, PyObject *args)
{
    sw_u16 dst;
    sw_u32 now, cap;
    sw_u8 *buf;
    unsigned lim;
    int r;
    PyObject *out;

    if (not_ready(self) ||
        !PyArg_ParseTuple(args, "O&O&O&:encode", conv_u16, &dst, conv_u32,
                          &now, conv_u32, &cap))
        return NULL;
    /* a frame never exceeds our own mtu, so that bounds the buffer */
    lim = cap < self->node.self.mtu ? (unsigned)cap : self->node.self.mtu;
    buf = (sw_u8 *)PyMem_Malloc(lim ? lim : 1);
    if (!buf)
        return PyErr_NoMemory();
    r = sw_encode(&self->node, dst, now, buf, lim);
    out = r < 0 ? PyLong_FromLong(r)
                : PyBytes_FromStringAndSize((const char *)buf, r);
    PyMem_Free(buf);
    return out;
}

static PyObject *node_decode(NodeObject *self, PyObject *args)
{
    const char *buf;
    Py_ssize_t len;
    sw_u32 now;

    if (not_ready(self) ||
        !PyArg_ParseTuple(args, "y#O&:decode", &buf, &len, conv_u32, &now))
        return NULL;
    if ((size_t)len > 0xFFFFUL) /* above any mtu */
        return PyLong_FromLong(SW_EFIELD);
    return PyLong_FromLong(
        sw_decode(&self->node, (const sw_u8 *)buf, (unsigned)len, now));
}

static PyObject *node_context(NodeObject *self, PyObject *noargs)
{
    (void)noargs;
    if (not_ready(self))
        return NULL;
    return ctx_tuple(&self->node.self);
}

/* known drones in ascending id order */
static PyObject *node_peers(NodeObject *self, PyObject *noargs)
{
    const sw_node *n = &self->node;
    PyObject *list, *t;
    unsigned k;

    (void)noargs;
    if (not_ready(self))
        return NULL;
    list = PyList_New((Py_ssize_t)n->npeer);
    if (!list)
        return NULL;
    for (k = 0; k < n->npeer; k++) {
        t = ctx_tuple(&n->peer[n->pix[k]]);
        if (!t) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)k, t);
    }
    return list;
}

static PyObject *node_root(NodeObject *self, PyObject *noargs)
{
    (void)noargs;
    if (not_ready(self))
        return NULL;
    return PyBytes_FromStringAndSize((const char *)self->node.hash[0],
                                     SW_HASH);
}

static PyObject *node_complete(NodeObject *self, PyObject *args)
{
    sw_u16 i;

    if (not_ready(self) ||
        !PyArg_ParseTuple(args, "O&:complete", conv_u16, &i))
        return NULL;
    if (i >= self->node.ntask) {
        PyErr_SetString(PyExc_IndexError, "tree node index out of range");
        return NULL;
    }
    return PyBool_FromLong((self->node.tflag[i] & SW_COMPLETE) != 0);
}

static PyObject *node_mission(NodeObject *self, PyObject *noargs)
{
    (void)noargs;
    if (not_ready(self))
        return NULL;
    return PyLong_FromUnsignedLong((unsigned long)self->node.mission);
}

static PyMethodDef node_methods[] = {
    {"init", (PyCFunction)node_init, METH_VARARGS,
     "init(id, range, mtu, cnt, key) -> code"},
    {"set_self", (PyCFunction)node_set_self, METH_VARARGS,
     "set_self(q, v, p, sig, op, ctx, now) -> code"},
    {"task", (PyCFunction)node_task, METH_VARARGS,
     "task(leaf, st, cost, lease, now) -> code"},
    {"leaf", (PyCFunction)node_leaf, METH_VARARGS,
     "leaf(leaf) -> (code, st, owner, cost, ver)"},
    {"tick", (PyCFunction)node_tick, METH_VARARGS,
     "tick(now) -> [neighbour ids]"},
    {"encode", (PyCFunction)node_encode, METH_VARARGS,
     "encode(dst, now, cap) -> bytes | code"},
    {"decode", (PyCFunction)node_decode, METH_VARARGS,
     "decode(frame, now) -> code"},
    {"context", (PyCFunction)node_context, METH_NOARGS,
     "context() -> own context tuple"},
    {"peers", (PyCFunction)node_peers, METH_NOARGS,
     "peers() -> [context tuple], ascending id"},
    {"root", (PyCFunction)node_root, METH_NOARGS,
     "root() -> goal-tree root hash"},
    {"complete", (PyCFunction)node_complete, METH_VARARGS,
     "complete(i) -> subtree i is done"},
    {"mission", (PyCFunction)node_mission, METH_NOARGS,
     "mission() -> mission id"},
    {NULL, NULL, 0, NULL}
};

static void node_dealloc(PyObject *self)
{
    PyTypeObject *tp = Py_TYPE(self);
    freefunc free_fn = (freefunc)PyType_GetSlot(tp, Py_tp_free);

    free_fn(self);
    Py_DECREF(tp);
}

static PyType_Slot node_slots[] = {
    {Py_tp_dealloc, (void *)node_dealloc},
    {Py_tp_methods, node_methods},
    {Py_tp_new, (void *)PyType_GenericNew},
    {Py_tp_doc, (void *)"Raw sw_node; use swarm_protocol.Node instead."},
    {0, NULL}
};

static PyType_Spec node_spec = {
    "swarm_protocol._swarm.Node", (int)sizeof(NodeObject), 0,
    Py_TPFLAGS_DEFAULT, node_slots
};

static struct PyModuleDef module_def = {
    PyModuleDef_HEAD_INIT, "_swarm",
    "C core of swarm_protocol; use the swarm_protocol package.", -1,
    NULL, NULL, NULL, NULL, NULL
};

#define ADD_INT(m, name, value) \
    if (PyModule_AddIntConstant(m, name, (long)(value)) < 0) goto fail

PyMODINIT_FUNC PyInit__swarm(void)
{
    PyObject *m = PyModule_Create(&module_def), *t;

    if (!m)
        return NULL;
    t = PyType_FromSpec(&node_spec);
    if (!t || PyModule_AddObjectRef(m, "Node", t) < 0) {
        Py_XDECREF(t);
        goto fail;
    }
    Py_DECREF(t);
    ADD_INT(m, "VERSION", SW_VERSION);
    ADD_INT(m, "BCAST", SW_BCAST);
    ADD_INT(m, "HASH", SW_HASH);
    ADD_INT(m, "FRAME_MIN", SW_FRAME_MIN);
    ADD_INT(m, "MAX_NODES", SW_MAX_NODES);
    ADD_INT(m, "MAX_NBR", SW_MAX_NBR);
    ADD_INT(m, "MAX_TASK", SW_MAX_TASK);
    ADD_INT(m, "FANOUT", SW_FANOUT);
    ADD_INT(m, "LOG", SW_LOG);
    ADD_INT(m, "OP_MAX", SW_OP_MAX);
    ADD_INT(m, "CTX_MAX", SW_CTX_MAX);
    ADD_INT(m, "TTL_MS", SW_TTL_MS);
    ADD_INT(m, "LEASE_MS", SW_LEASE_MS);
    ADD_INT(m, "SLOT_MS", SW_SLOT_MS);
    ADD_INT(m, "SKEW_MS", SW_SKEW_MS);
    ADD_INT(m, "NODE_BYTES", sizeof(sw_node));
    ADD_INT(m, "OPEN", SW_OPEN);
    ADD_INT(m, "CLAIMED", SW_CLAIMED);
    ADD_INT(m, "DONE", SW_DONE);
    ADD_INT(m, "OK", SW_OK);
    ADD_INT(m, "EARG", SW_EARG);
    ADD_INT(m, "ESHORT", SW_ESHORT);
    ADD_INT(m, "EVER", SW_EVER);
    ADD_INT(m, "EMISSION", SW_EMISSION);
    ADD_INT(m, "EADDR", SW_EADDR);
    ADD_INT(m, "EAUTH", SW_EAUTH);
    ADD_INT(m, "EFIELD", SW_EFIELD);
    ADD_INT(m, "ESTALE", SW_ESTALE);
    ADD_INT(m, "EREACH", SW_EREACH);
    ADD_INT(m, "EBUSY", SW_EBUSY);
    ADD_INT(m, "EDONE", SW_EDONE);
    ADD_INT(m, "ETIME", SW_ETIME);
    ADD_INT(m, "ESLOT", SW_ESLOT);
    return m;
fail:
    Py_DECREF(m);
    return NULL;
}
