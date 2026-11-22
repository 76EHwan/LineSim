#!/usr/bin/env python3
"""
race_line.py - corridor-limited racing line prototype for the 4th run.

The track centre line comes straight from the track JSON (straights + R 0.25 arcs).
The sensor bar may leave the line by up to +-W inside the viewer's +-150 mm corridor.
  1. offset n(s) of the sensor bar on a coarse grid, minimum-curvature QP with box
     constraints |n| <= W (FISTA, projected), ends pinned to the line near start / goal.
  2. body path = trailer of the bar path (no-slip), body curvature -> friction-circle
     speed profile (a_lat = base^2 / R_body_ref, accel / decel, vmax) -> lap time.
Only numpy.

Usage: tools/.venv/Scripts/python.exe tools/race_line.py --track tracks/finals_1_5x_championship.json
"""
import argparse
import json
import math
import re

import numpy as np

SENSOR_X = 0.13213


def load_track(path):
    txt = open(path, encoding="utf-8").read()
    txt = re.sub(r"//[^\n]*", "", txt)
    return json.loads(txt)


def centerline(tr, ds):
    """Dense centre line (x, y, th, kappa) every ds metres around the closed track."""
    xs, ys, ths, ks = [], [], [], []
    x = y = th = 0.0
    o = tr.get("origin")
    if o:
        x, y, th = o[0], o[1], math.radians(o[2])
    carry = 0.0
    for sg in tr["segments"]:
        if "straight" in sg:
            L, k = sg["straight"], 0.0
        else:
            R = sg.get("radius", 0.25)
            L = abs(sg["arc"]) * math.pi / 180 * R
            k = (1 if sg["arc"] >= 0 else -1) / R
        u = carry
        while u < L - 1e-12:
            # pose at distance u into this segment
            if k == 0:
                px, py, pt = x + u * math.cos(th), y + u * math.sin(th), th
            else:
                pt = th + k * u
                px = x + (math.sin(pt) - math.sin(th)) / k
                py = y - (math.cos(pt) - math.cos(th)) / k
            xs.append(px); ys.append(py); ths.append(pt); ks.append(k)
            u += ds
        carry = u - L
        if k == 0:
            x, y = x + L * math.cos(th), y + L * math.sin(th)
        else:
            nt = th + k * L
            x, y = x + (math.sin(nt) - math.sin(th)) / k, y - (math.cos(nt) - math.cos(th)) / k
            th = nt
    s = np.arange(len(xs)) * ds
    return s, np.c_[xs, ys], np.unwrap(np.array(ths)), np.array(ks)


def lap_slice(s, C, th, k, start, goal, length, ds):
    """Centre line from start to goal (closed track, going once around)."""
    end = goal if goal > start else goal + length
    q = np.arange(start, end + 1e-9, ds)
    qq = np.mod(q, length)
    idx = np.searchsorted(s, qq, side="right") - 1
    idx = np.clip(idx, 0, len(s) - 1)
    # exact resample by integrating from the nearest grid point
    du = qq - s[idx]
    t0, k0 = th[idx], k[idx]
    tt = t0 + k0 * du
    with np.errstate(divide="ignore", invalid="ignore"):
        dx = np.where(np.abs(k0) > 1e-9, (np.sin(tt) - np.sin(t0)) / np.where(k0 == 0, 1, k0), du * np.cos(t0))
        dy = np.where(np.abs(k0) > 1e-9, -(np.cos(tt) - np.cos(t0)) / np.where(k0 == 0, 1, k0), du * np.sin(t0))
    P = C[idx] + np.c_[dx, dy]
    return q - start, P, np.unwrap(tt), k0


def min_curv(C, N, W, pin, iters=6000, w_len=0.0, reweight=200, n0=None):
    """argmin sum w_i |P[i-1]-2P[i]+P[i+1]|^2 (+ w_len * sum |P[i+1]-P[i]|^2), P = C + n N, |n| <= W.
    w_i = (ds0 / ds_i)^4 from the current iterate (every `reweight` steps), so the term is the
    true curvature^2 and the optimiser cannot hide curvature where inside points bunch up."""
    n = np.zeros(len(C)) if n0 is None else n0.copy()
    y = n.copy()
    t = 1.0
    ds0 = float(np.mean(np.hypot(*np.diff(C, axis=0).T)))
    w = np.ones(len(C) - 2)
    Lip = 16.0 + 4.0 * w_len
    for it in range(iters):
        if reweight and it % reweight == 0:
            P = C + n[:, None] * N
            dl = np.hypot(*np.diff(P, axis=0).T)
            dloc = 0.5 * (dl[1:] + dl[:-1])
            w = np.minimum((ds0 / np.maximum(dloc, 1e-6)) ** 4, 30.0)
            Lip = 16.0 * float(w.max()) + 4.0 * w_len
            y = n.copy()
            t = 1.0
        P = C + y[:, None] * N
        D2 = (P[:-2] - 2 * P[1:-1] + P[2:]) * w[:, None]
        g = np.zeros_like(P)
        g[:-2] += D2
        g[1:-1] -= 2 * D2
        g[2:] += D2
        if w_len > 0:
            D1 = P[1:] - P[:-1]
            g[:-1] -= w_len * D1
            g[1:] += w_len * D1
        gn = 2 * np.einsum("ij,ij->i", g, N)
        nn = np.clip(y - gn / (2 * Lip), -W, W)
        nn[pin] = 0.0
        t1 = 0.5 * (1 + math.sqrt(1 + 4 * t * t))
        y = nn + (t - 1) / t1 * (nn - n)
        n, t = nn, t1
    return n


def qp_terms(C, N, w):
    """f(n) = n'Hn + 2b'n + c for f = sum_i w_i |D2_i|^2 (pentadiagonal H)."""
    m = len(C)
    H = np.zeros((m, m))
    b = np.zeros(m)
    A = C[:-2] - 2 * C[1:-1] + C[2:]
    cf = (1.0, -2.0, 1.0)
    for i in range(m - 2):
        js = (i, i + 1, i + 2)
        for a_, ja in enumerate(js):
            b[ja] += w[i] * cf[a_] * float(A[i] @ N[ja])
            for b_, jb in enumerate(js):
                H[ja, jb] += w[i] * cf[a_] * cf[b_] * float(N[ja] @ N[jb])
    return H, b


def box_qp(H, b, lo, hi, x0=None, max_iter=60):
    """min x'Hx + 2b'x s.t. lo <= x <= hi (primal-dual active set)."""
    m = len(b)
    x = np.clip(np.zeros(m) if x0 is None else x0, lo, hi)
    fixed_lo = np.isclose(lo, hi)  # pinned
    for _ in range(max_iter):
        g = H @ x + b  # half gradient
        at_lo = (x <= lo + 1e-12) & (g > 0)
        at_hi = (x >= hi - 1e-12) & (g < 0)
        act = at_lo | at_hi | fixed_lo
        F = ~act
        xn = x.copy()
        xn[at_lo] = lo[at_lo]
        xn[at_hi] = hi[at_hi]
        if F.any():
            rhs = -(b[F] + H[np.ix_(F, act)] @ xn[act])
            xn[F] = np.linalg.solve(H[np.ix_(F, F)], rhs)
        viol = (xn < lo - 1e-9) | (xn > hi + 1e-9)
        xn = np.clip(xn, lo, hi)
        if not viol.any() and np.allclose(xn, x, atol=1e-7):
            x = xn
            break
        x = xn
    return x


def qp_terms_proj(C, N, Np, w, w_t=0.0):
    """f = sum_i w_i (D2_i . Np_i)^2 + w_t |D2_i|^2 : curvature along the path normal Np_i only."""
    m = len(C)
    H = np.zeros((m, m))
    b = np.zeros(m)
    A = C[:-2] - 2 * C[1:-1] + C[2:]
    cf = (1.0, -2.0, 1.0)
    for i in range(m - 2):
        js = (i, i + 1, i + 2)
        e = [cf[a_] * float(N[j] @ Np[i]) for a_, j in enumerate(js)]
        a0 = float(A[i] @ Np[i])
        for a_ in range(3):
            b[js[a_]] += w[i] * e[a_] * a0
            for b_ in range(3):
                H[js[a_], js[b_]] += w[i] * e[a_] * e[b_]
    if w_t > 0:
        H2, b2 = qp_terms(C, N, np.full(m - 2, w_t))
        H += H2
        b += b2
    return H, b


def penta_solve(e0, e1, e2, r):
    """Solve a symmetric pentadiagonal SPD system (diag e0, 1st e1, 2nd e2) by LDL^T."""
    m = len(e0)
    d = [0.0] * m
    l1 = [0.0] * m  # L[k, k-1]
    l2 = [0.0] * m  # L[k, k-2]
    for k in range(m):
        a2 = e2[k - 2] if k >= 2 else 0.0
        l2k = a2 / d[k - 2] if k >= 2 else 0.0
        a1 = e1[k - 1] if k >= 1 else 0.0
        l1k = (a1 - (l2k * d[k - 2] * l1[k - 1] if k >= 2 else 0.0)) / d[k - 1] if k >= 1 else 0.0
        dk = e0[k] - (l1k * l1k * d[k - 1] if k >= 1 else 0.0) - (l2k * l2k * d[k - 2] if k >= 2 else 0.0)
        l1[k], l2[k], d[k] = l1k, l2k, dk
    y = [0.0] * m
    for k in range(m):
        y[k] = r[k] - (l1[k] * y[k - 1] if k >= 1 else 0.0) - (l2[k] * y[k - 2] if k >= 2 else 0.0)
    x = [0.0] * m
    for k in range(m - 1, -1, -1):
        v = y[k] / d[k]
        if k + 1 < m:
            v -= l1[k + 1] * x[k + 1]
        if k + 2 < m:
            v -= l2[k + 2] * x[k + 2]
        x[k] = v
    return x


def box_qp_band(d0, d1, d2, b, lo, hi, x0, max_iter=80):
    """min x'Hx + 2b'x, lo <= x <= hi, H pentadiagonal (bands d0, d1, d2). Primal-dual active set."""
    m = len(b)

    def hx(x):
        y = d0 * x
        y[:-1] += d1 * x[1:]
        y[1:] += d1 * x[:-1]
        y[:-2] += d2 * x[2:]
        y[2:] += d2 * x[:-2]
        return y

    x = np.clip(x0, lo, hi)
    pinned = np.isclose(lo, hi)
    for _ in range(max_iter):
        g = hx(x) + b
        at_lo = pinned | ((x <= lo + 1e-12) & (g > 0))
        at_hi = (x >= hi - 1e-12) & (g < 0) & ~pinned
        act = at_lo | at_hi
        xa = np.where(at_lo, lo, np.where(at_hi, hi, 0.0))
        F = np.flatnonzero(~act)
        xn = xa.copy()
        if len(F):
            rhs = -(b + hx(xa))[F]
            gap1 = np.r_[np.diff(F), 99]
            gap2 = np.r_[F[2:] - F[:-2], 99, 99]
            e0 = d0[F]
            d1x = np.r_[d1, 0.0]
            d2x = np.r_[d2, 0.0, 0.0]
            e1 = np.where(gap1 == 1, d1x[F], np.where(gap1 == 2, d2x[F], 0.0))
            e2 = np.where(gap2 == 2, d2x[F], 0.0)
            xn[F] = penta_solve(e0.tolist(), e1.tolist(), e2.tolist(), rhs.tolist())
        viol = (xn < lo - 1e-9) | (xn > hi + 1e-9)
        xn = np.clip(xn, lo, hi)
        done = (not viol.any()) and np.allclose(xn, x, atol=1e-8)
        x = xn
        if done:
            break
    return x


def bands_proj(C, N, Np, w, w_t):
    """Bands of H and b for f = sum w_i (D2_i.Np_i)^2 + w_t |D2_i|^2 (vectorised)."""
    m = len(C)
    A = C[:-2] - 2 * C[1:-1] + C[2:]
    d0 = np.zeros(m); d1 = np.zeros(m - 1); d2 = np.zeros(m - 2); b = np.zeros(m)
    # projected term: residual r_i = a0 + e0 n_i + e1 n_{i+1} + e2 n_{i+2}
    e = [np.einsum("ij,ij->i", N[0:m - 2], Np), -2 * np.einsum("ij,ij->i", N[1:m - 1], Np),
         np.einsum("ij,ij->i", N[2:m], Np)]
    a0 = np.einsum("ij,ij->i", A, Np)
    for a_ in range(3):
        b[a_:m - 2 + a_] += w * e[a_] * a0
        d0[a_:m - 2 + a_] += w * e[a_] ** 2
    d1[0:m - 2] += w * e[0] * e[1]
    d1[1:m - 1] += w * e[1] * e[2]
    d2[0:m - 2] += w * e[0] * e[2]
    if w_t > 0:
        cf = (1.0, -2.0, 1.0)
        Nj = [N[0:m - 2], N[1:m - 1], N[2:m]]
        for a_ in range(3):
            b[a_:m - 2 + a_] += w_t * cf[a_] * np.einsum("ij,ij->i", A, Nj[a_])
            d0[a_:m - 2 + a_] += w_t * cf[a_] ** 2
        d1[0:m - 2] += w_t * cf[0] * cf[1] * np.einsum("ij,ij->i", Nj[0], Nj[1])
        d1[1:m - 1] += w_t * cf[1] * cf[2] * np.einsum("ij,ij->i", Nj[1], Nj[2])
        d2[0:m - 2] += w_t * cf[0] * cf[2] * np.einsum("ij,ij->i", Nj[0], Nj[2])
    return d0, d1, d2, b


def race_line_band(C, N, W, pin, outer=8, w_t=0.02, lo=None, hi=None):
    m = len(C)
    if lo is None:
        lo = np.full(m, -W); hi = np.full(m, W)
    lo = lo.copy(); hi = hi.copy()
    lo[pin] = hi[pin] = 0.0
    ds0 = float(np.mean(np.hypot(*np.diff(C, axis=0).T)))
    n = np.zeros(m)
    for _ in range(outer):
        P = C + n[:, None] * N
        T = np.gradient(P, axis=0)
        T /= np.linalg.norm(T, axis=1)[:, None]
        Np = np.c_[-T[:, 1], T[:, 0]][1:-1]
        dl = np.hypot(*np.diff(P, axis=0).T)
        dloc = np.maximum(0.5 * (dl[1:] + dl[:-1]), 0.2 * ds0)
        w = (ds0 / dloc) ** 3
        d0, d1, d2, b = bands_proj(C, N, Np, w, w_t)
        d0 = d0 + 1e-12
        n = box_qp_band(d0, d1, d2, b, lo, hi, n)
    return n


def min_curv_qp(C, N, W, pin, outer=8, w_len=0.0, w_t=0.02, lo=None, hi=None):
    m = len(C)
    if lo is None:
        lo = np.full(m, -W)
        hi = np.full(m, W)
    lo = lo.copy(); hi = hi.copy()
    lo[pin] = hi[pin] = 0.0
    ds0 = float(np.mean(np.hypot(*np.diff(C, axis=0).T)))
    n = np.zeros(m)
    for _ in range(outer):
        P = C + n[:, None] * N
        T = np.gradient(P, axis=0)
        T /= np.linalg.norm(T, axis=1)[:, None]
        Np = np.c_[-T[:, 1], T[:, 0]][1:-1]
        dl = np.hypot(*np.diff(P, axis=0).T)
        dloc = np.maximum(0.5 * (dl[1:] + dl[:-1]), 0.2 * ds0)
        w = (ds0 / dloc) ** 3  # sum kappa^2 dl
        H, b = qp_terms_proj(C, N, Np, w, w_t)
        if w_len > 0:
            # + w_len * sum |P[i+1]-P[i]|^2
            for i in range(m - 1):
                d = C[i + 1] - C[i]
                b[i] -= w_len * float(d @ N[i]); b[i + 1] += w_len * float(d @ N[i + 1])
                H[i, i] += w_len; H[i + 1, i + 1] += w_len
                c = w_len * float(N[i] @ N[i + 1]); H[i, i + 1] -= c; H[i + 1, i] -= c
        H += 1e-12 * np.eye(m)
        n = box_qp(H, b, lo, hi, n)
    return n


def resample(P, ds):
    seg = np.hypot(*np.diff(P, axis=0).T)
    u = np.r_[0, np.cumsum(seg)]
    q = np.arange(0, u[-1], ds)
    return np.c_[np.interp(q, u, P[:, 0]), np.interp(q, u, P[:, 1])]


def gauss1d(a, sigma):
    if sigma <= 0:
        return a
    r = int(3 * sigma + 1)
    x = np.arange(-r, r + 1)
    w = np.exp(-0.5 * (x / sigma) ** 2)
    w /= w.sum()
    return np.convolve(np.pad(a, r, mode="edge"), w, mode="valid")


def trailer(P, L):
    seg = np.diff(P, axis=0)
    dsp = np.hypot(seg[:, 0], seg[:, 1]).tolist()
    phi = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0])).tolist()
    th = [phi[0]]
    for i in range(len(dsp)):
        th.append(th[-1] + dsp[i] * math.sin(phi[i] - th[-1]) / L)
    th = np.array(th)
    Q = P - L * np.c_[np.cos(th), np.sin(th)]
    return Q, th


def lap_time(P, ds, a_lat, accel, decel, vmax, v_end=None):
    """No-slip body of a bar path P (uniform ds): lap time + profile."""
    Q, th = trailer(P, SENSOR_X)
    dq = np.hypot(*np.diff(Q, axis=0).T)
    kb = np.diff(th) / np.maximum(dq, 1e-9)
    kb = gauss1d(np.r_[kb[0], 0.5 * (kb[1:] + kb[:-1]), kb[-1]], 2.0)
    k = np.abs(kb)
    v = np.minimum(vmax, np.sqrt(a_lat / np.maximum(k, 1e-9)))
    v[0] = 0.0
    vl = v.tolist(); kl = k.tolist(); dl = dq.tolist()
    n = len(vl)
    for i in range(n - 1):
        al = vl[i] * vl[i] * kl[i]
        a = min(accel, math.sqrt(max(0.0, a_lat * a_lat - al * al)))
        vl[i + 1] = min(vl[i + 1], math.sqrt(vl[i] * vl[i] + 2 * a * dl[i]))
    if v_end is not None:
        vl[-1] = min(vl[-1], v_end)
    for i in range(n - 2, -1, -1):
        al = vl[i + 1] * vl[i + 1] * kl[i + 1]
        a = min(decel, math.sqrt(max(0.0, a_lat * a_lat - al * al)))
        vl[i] = min(vl[i], math.sqrt(vl[i + 1] * vl[i + 1] + 2 * a * dl[i]))
    v = np.array(vl)
    vm = 0.5 * (v[1:] + v[:-1])
    t = float(np.sum(dq / np.maximum(vm, 1e-3)))
    return t, v, kb, float(dq.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", required=True)
    ap.add_argument("--base", type=float, default=1.80)
    ap.add_argument("--accel", type=float, default=5.5)
    ap.add_argument("--decel", type=float, default=5.5)
    ap.add_argument("--vmax", type=float, default=7.0)
    ap.add_argument("--widths", default="0,0.04,0.075,0.10,0.12,0.14")
    ap.add_argument("--ds-opt", type=float, default=0.05)
    ap.add_argument("--iters", type=int, default=6000)
    ap.add_argument("--w-len", type=float, default=0.0)
    ap.add_argument("--outer", type=int, default=8)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()

    tr = load_track(a.track)
    ds = 0.01
    s, C, th, k = centerline(tr, ds)
    length = len(s) * ds
    start = tr.get("robot_start_s", tr["start_s"])
    q, Cl, thl, kl = lap_slice(s, C, th, k, start, tr["goal_s"], length, a.ds_opt)
    N = np.c_[-np.sin(thl), np.cos(thl)]
    pin = (q < 0.30) | (q > q[-1] - 0.30)
    # a_lat: the sim robot holds base speed on the line (bar R 0.25 -> body R 0.212)
    r_body = math.sqrt(0.25 ** 2 - SENSOR_X ** 2)
    a_lat = a.base ** 2 / r_body
    print(f"track {a.track}: length {length:.2f} m, lap {q[-1]:.2f} m, a_lat {a_lat:.1f} m/s^2")
    rows = []
    for W in [float(x) for x in a.widths.split(",")]:
        n = np.zeros(len(q)) if W == 0 else race_line_band(Cl, N, W, pin, outer=a.outer)
        P = resample(Cl + n[:, None] * N, ds)
        t, v, kb, dist = lap_time(P, ds, a_lat, a.accel, a.decel, a.vmax)
        blind = float(np.mean(np.abs(n) > 0.065))
        rows.append((W, t, dist, float(v.mean()), blind))
        print(f"W {W*1000:5.0f} mm: lap {t:6.2f} s  dist {dist:6.2f} m  vmean {v.mean():4.2f}  |n|>65mm {blind*100:4.1f}%")
        if a.csv and W == float(a.widths.split(",")[-1]):
            np.savetxt(a.csv, np.c_[q, n], delimiter=",", header="s,n", comments="")


if __name__ == "__main__":
    main()


# ----------------------------------------------------------------------------- Gauss-Newton (spacing-exact) version

def menger(P):
    """Signed three-point (circumcircle) curvature at interior points and local length."""
    a = P[1:-1] - P[:-2]
    b = P[2:] - P[1:-1]
    c = P[2:] - P[:-2]
    la, lb, lc = (np.linalg.norm(v, axis=1) for v in (a, b, c))
    cr = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    k = 2.0 * cr / np.maximum(la * lb * lc, 1e-12)
    return k, 0.5 * (la + lb)


def body_f(k, L, body):
    if not body:
        return k
    kk = np.clip(k, -0.95 / L, 0.95 / L)
    return kk / np.sqrt(1.0 - (L * kk) ** 2)


def gn_residual(C, N, n, L, body):
    P = C + n[:, None] * N
    k, dl = menger(P)
    return body_f(k, L, body) * np.sqrt(dl)


def race_line_gn(C, N, lo, hi, iters=25, L=SENSOR_X, body=True, mu0=1e-3, h=1e-4, verbose=False):
    m = len(C)
    n = np.clip(np.zeros(m), lo, hi)
    mu = mu0
    r = gn_residual(C, N, n, L, body)
    cost = float(r @ r)
    for it in range(iters):
        # Jacobian rows i (interior index i+1) w.r.t. n[i], n[i+1], n[i+2]
        J = np.zeros((m - 2, 3))
        for a_ in range(3):
            dn = np.zeros(m)
            dn[a_:m - 2 + a_] = 0.0
            # perturb every third point with the same offset pattern so the three stencil columns are separable
            for ph in range(3):
                d = np.zeros(m)
                idx = np.arange(ph, m, 3)
                d[idx] = h
                rp = gn_residual(C, N, n + d, L, body)
                # residual i depends on n[i], n[i+1], n[i+2]; column a_ = position of the perturbed point in the stencil
                rows = np.arange(m - 2)
                hit = (rows + a_) % 3 == ph
                J[hit, a_] = (rp[hit] - r[hit]) / h
        d0 = np.zeros(m); d1 = np.zeros(m - 1); d2 = np.zeros(m - 2); b = np.zeros(m)
        for a_ in range(3):
            d0[a_:m - 2 + a_] += J[:, a_] ** 2
            b[a_:m - 2 + a_] += J[:, a_] * r
        d1[0:m - 2] += J[:, 0] * J[:, 1]
        d1[1:m - 1] += J[:, 1] * J[:, 2]
        d2[0:m - 2] += J[:, 0] * J[:, 2]
        for _ in range(8):
            dd0 = d0 + mu * (1.0 + d0)
            step = box_qp_band(dd0, d1, d2, b, lo - n, hi - n, np.zeros(m))
            nn = np.clip(n + step, lo, hi)
            rn = gn_residual(C, N, nn, L, body)
            cn = float(rn @ rn)
            if cn < cost:
                n, r, cost = nn, rn, cn
                mu = max(mu * 0.3, 1e-6)
                break
            mu *= 5.0
        if verbose:
            print(f"  gn {it}: cost {cost:.4f} mu {mu:.1e}")
    return n
