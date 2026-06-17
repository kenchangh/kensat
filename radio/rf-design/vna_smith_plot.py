#!/usr/bin/env python3
"""
VNA + Component Tuning Smith Chart Visualizer
==============================================

Edit the VNA measurements and component values at the top of this file,
then re-run to see where each modeled impedance lands on the Smith chart
versus what the NanoVNA actually measured.

  python src/vna_smith_plot.py            # use saved calibration
  python src/vna_smith_plot.py --fit      # refit CAL_TX/CAL_RX to match VNA,
                                          #   then copy printed values back
                                          #   into the CAL_* blocks below.

Workflow:
  1. Capture VNA with the CURRENT BOM, paste Z values into VNA{} below.
  2. Run with --fit once. A 2-knob compensation (series L + shunt C) per
     branch is solved so the model exactly reproduces today's VNA.
  3. Paste the printed CAL_TX / CAL_RX values back into this file.
  4. Now edit C1..C7 / L2..L5 to model rework. The calibrated parasitics
     stay fixed and the predicted Smith points show where the board will
     actually land post-rework.
  5. After rework, re-measure and re-run --fit to refresh the calibration.

Topology mirrors src/matching_network_skrf.py (ideal lumped, no parasitics).
"""

import argparse

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--fit', action='store_true',
                    help='Refit CAL_TX/CAL_RX to match current VNA values')
parser.add_argument('--optimize', action='store_true',
                    help='Search in-stock C/L values for best 50 Ω match. '
                         'Phase A: sweep TX (C2,C3,L2,L3). Phase B: add ANT '
                         '(C4,C5,L4) only if TX-only cannot reach -15 dB. '
                         'Phase C: single-part RX tweak if >=3 dB improvement.')
parser.add_argument('-i', '--interactive', action='store_true',
                    help='Open a live matplotlib window and a REPL so you '
                         'can tweak component values and watch the chart '
                         'update in place')
args = parser.parse_args()

import matplotlib
if not args.interactive:
    matplotlib.use('Agg')   # headless render, save PNG only

import numpy as np
import skrf as rf
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

# Unit suffix shorthand for the REPL: 4.7p → 4.7e-12, 27n → 27e-9
_SUFFIX = {'p': 1e-12, 'n': 1e-9, 'u': 1e-6, 'f': 1e-15, '': 1.0}


# ============================================================
# EDIT ME — VNA measurements at F0
# ============================================================
F0_HZ = 437.2e6

VNA = {
    'TX': {'z': 34.1 - 6.93j, 's11_db': -13.76},  # NanoVNA: Rs=34.1Ω, Cs=52.5pF @ 437.2 MHz (post C2 33→27p)
    'RX': {'z': 57.8 -  2.1j, 's11_db': -22.53},  # IMG_8807
}


# ============================================================
# EDIT ME — Matching network component values (SI units)
# ============================================================
# --- TX path (PA → C1 → shunt C2 → L2 → shunt C3 → L3 → switch) ---
C1 = 47e-12    # TX series DC-block near PA
C2 = 27e-12
C3 = 22e-12
L2 = 27e-9
L3 = 6.8e-9

# --- RX path (LNA → shunt C7 → L5 → C6 → switch) ---
C6 = 100e-12   # RX series DC-block                   [was 39 pF]
C7 = 1.5e-12   # RX shunt near LNA
L5 = 47e-9     # RX series                            [was 56 nH]

# --- ANT / harmonic filter (switch common → shunt C5 → L4 → shunt C4 → SMA) ---
C4 = 12e-12    # ANT shunt near SMA
C5 = 12e-12    # ANT shunt near switch
L4 = 15e-9     # ANT series

# --- Chip-side helpers (not on matching traces; left for reference) ---
L1 = 120e-9    # PA DC choke to VDD (baked into TX load model, not cascade)
# C8 = C9 = 100 pF DC blocks — RF shorts, ignored in this model


# ============================================================
# In-stock 0805 rework parts (Digi-Key invoices 124185643 + 123180639)
# ============================================================
# Caps: Murata GQM2195 / Johanson QSCT251 (250 V C0G/NP0)
#       + Yageo CC0805CR / Kemet C0805 (50 V C0G/NP0)
STOCK_C_PF = [1.0, 1.2, 1.5, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0, 3.3, 3.9, 4.3,
              4.7, 5.1, 5.6, 6.8, 7.5, 8.2, 9.1, 10, 12, 15, 18, 22, 27,
              33, 39, 47, 56, 68, 100]
# Inductors: Delta 0805CS + Murata LQW2BAS series
STOCK_L_NH = [5.6, 6.8, 8.2, 10, 12, 15, 18, 22, 27, 33, 39, 43, 47, 56,
              68, 82, 100, 120]


# ============================================================
# CALIBRATION — parasitic compensation per branch
# ============================================================
# These absorb everything the lumped model doesn't capture (trace L/C, pad
# parasitics, via stubs, component self-resonance, switch parasitics, probe
# fixture offset). They are solved by --fit so the model exactly reproduces
# the VNA Z at F0. Once fit, EDIT the lines below with the printed values
# and they stay fixed while you tweak C1..L5 to model rework.
#
# Topology of each compensation cell (between SMA side and the rest of the
# chain): series inductance, then shunt capacitance to ground. Negative
# values are allowed and physically meaningful (they model a net-capacitive
# series parasitic or a net-inductive shunt).
CAL_TX = {'L_ser': +6.532e-9, 'C_sh': -99.791e-12}
CAL_RX = {'L_ser': +11.769e-9, 'C_sh': +10.037e-12}

# ============================================================
# Si4463 chip-side target impedances (from AN648 / AN643)
# ============================================================
Z0 = 50.0
omega0 = 2 * np.pi * F0_HZ

# TX — Class-E PA load (AN648 sec 4.2)
CSHUNT = 2.5e-12
LCHOKE = L1
RLOAD = 0.2815 / (omega0 * CSHUNT) * np.cos(np.radians(49.0524))  # ~26.9 Ω

def tx_load_z():
    Y = 1.0/RLOAD + 1j*omega0*CSHUNT + 1.0/(1j*omega0*LCHOKE)
    return 1.0 / Y

# RX — LNA single-ended equivalent (AN643 Table 1, 434 MHz)
RLNA_SE = 240.0
CLNA_SE = 1.94e-12

def rx_load_z():
    Y = 1.0/RLNA_SE + 1j*omega0*CLNA_SE
    return 1.0 / Y

# AS179-92LF SPDT switch
SW_RON = 1.5
SW_COFF = 0.3e-12


# ============================================================
# Build single-frequency network model
# ============================================================
freq = rf.Frequency(start=F0_HZ, stop=F0_HZ, npoints=1, unit='Hz')
line = rf.DefinedGammaZ0(frequency=freq, z0=Z0)


def series_Z(Z):
    """Two-port ABCD for a series impedance."""
    S = np.zeros((1, 2, 2), dtype=complex)
    denom = Z + 2*Z0
    S[0, 0, 0] = Z / denom
    S[0, 1, 1] = Z / denom
    S[0, 0, 1] = 2*Z0 / denom
    S[0, 1, 0] = 2*Z0 / denom
    return rf.Network(frequency=freq, s=S, z0=Z0)


def shunt_Y(Y):
    """Two-port ABCD for a shunt admittance."""
    S = np.zeros((1, 2, 2), dtype=complex)
    denom = 2 + Y*Z0
    S[0, 0, 0] = -Y*Z0 / denom
    S[0, 1, 1] = -Y*Z0 / denom
    S[0, 0, 1] = 2 / denom
    S[0, 1, 0] = 2 / denom
    return rf.Network(frequency=freq, s=S, z0=Z0)


def L_series(L):  return series_Z(1j*omega0*L)
def C_series(C):  return series_Z(1/(1j*omega0*C))
def C_shunt(C):   return shunt_Y(1j*omega0*C)
def R_series(R):  return series_Z(R + 0j)


def build_tx_chain():
    """Port 1 = switch side, Port 2 = PA side."""
    return (L_series(L3)
            ** C_shunt(C3)
            ** L_series(L2)
            ** C_shunt(C2)
            ** C_series(C1))


def build_rx_chain():
    """Port 1 = switch side, Port 2 = LNA side."""
    return (C_series(C6)
            ** L_series(L5)
            ** C_shunt(C7))


def build_ant_chain():
    """Port 1 = SMA side, Port 2 = switch common."""
    return (C_shunt(C5)
            ** L_series(L4)
            ** C_shunt(C4))


def comp_cell(cal):
    """Series L followed by shunt C. Supports zero / negative values."""
    Z_ser = 1j*omega0*cal['L_ser']
    Y_sh = 1j*omega0*cal['C_sh']
    return series_Z(Z_ser) ** shunt_Y(Y_sh)


def full_path_z(mode, cal_tx=None, cal_rx=None):
    """Return Z looking in from the SMA (port 1) with chip-side termination."""
    if cal_tx is None: cal_tx = CAL_TX
    if cal_rx is None: cal_rx = CAL_RX
    ant = build_ant_chain()
    sw_off = C_shunt(SW_COFF)
    sw_on = R_series(SW_RON)
    if mode == 'TX':
        chain = ant ** sw_off ** sw_on ** comp_cell(cal_tx) ** build_tx_chain()
        Z_load = tx_load_z()
    else:
        chain = ant ** sw_off ** sw_on ** comp_cell(cal_rx) ** build_rx_chain()
        Z_load = rx_load_z()
    S = chain.s[0]
    gamma_L = (Z_load - Z0) / (Z_load + Z0)
    gamma_in = S[0, 0] + S[0, 1]*S[1, 0]*gamma_L / (1 - S[1, 1]*gamma_L)
    return Z0 * (1 + gamma_in) / (1 - gamma_in), gamma_in


# ============================================================
# Calibration fit: solve CAL_TX/CAL_RX so model Z == VNA Z
# ============================================================
def fit_calibration():
    """Fit (L_ser, C_sh) per branch so predicted Z matches VNA Z exactly.
    Multi-start over several x0 to escape boundary traps — with tight bounds
    and a single init, the solver has been known to clamp at ±bound when the
    true optimum is on the opposite side of a non-convex residual surface."""
    def make_residuals(mode):
        vna_z = VNA[mode]['z']
        def res(x):
            cal = {'L_ser': x[0]*1e-9, 'C_sh': x[1]*1e-12}
            kwargs = {'cal_tx': cal} if mode == 'TX' else {'cal_rx': cal}
            z, _ = full_path_z(mode, **kwargs)
            return [z.real - vna_z.real, z.imag - vna_z.imag]
        return res
    # x is [L_nH, C_pF] — scaled units keep the solver well-conditioned.
    # Bounds widened from ±500/±100 to ±2000/±500 because real board parasitics
    # (especially shunt compensation for unmodeled chip-side Z) can exceed the
    # tight range after BOM changes.
    bounds = ([-2000, -500], [2000, 500])
    # Multi-start: cover the four quadrants + origin + a couple of off-diagonal
    # seeds. Keeps the fit finding the global min regardless of BOM state.
    x0_candidates = [
        [   1.0,    1.0],
        [ -50.0,  -10.0],
        [  50.0,   50.0],
        [ 100.0,  -50.0],
        [-100.0,   50.0],
        [-500.0, -100.0],
        [ 500.0,  100.0],
    ]
    def best_fit(mode):
        res_fn = make_residuals(mode)
        best = None
        for x0 in x0_candidates:
            try:
                sol = least_squares(res_fn, x0, bounds=bounds, xtol=1e-12)
                r = float(np.linalg.norm(sol.fun))
                if best is None or r < best[0]:
                    best = (r, sol)
            except Exception:
                continue
        return best[1]
    tx_sol = best_fit('TX')
    rx_sol = best_fit('RX')
    return (
        {'L_ser': tx_sol.x[0]*1e-9, 'C_sh': tx_sol.x[1]*1e-12},
        {'L_ser': rx_sol.x[0]*1e-9, 'C_sh': rx_sol.x[1]*1e-12},
    )


if args.fit:
    cal_tx_fit, cal_rx_fit = fit_calibration()
    CAL_TX = cal_tx_fit
    CAL_RX = cal_rx_fit
    print("\n=== Fitted calibration — paste back into CAL_TX / CAL_RX ===")
    print(f"CAL_TX = {{'L_ser': {CAL_TX['L_ser']*1e9:+.3f}e-9, "
          f"'C_sh': {CAL_TX['C_sh']*1e12:+.3f}e-12}}")
    print(f"CAL_RX = {{'L_ser': {CAL_RX['L_ser']*1e9:+.3f}e-9, "
          f"'C_sh': {CAL_RX['C_sh']*1e12:+.3f}e-12}}")
    print()


# ============================================================
# Optimizer: search in-stock values for best 50 Ω match
# ============================================================
# Fast pure-numpy ABCD evaluation, 100–1000× faster than rebuilding
# skrf Networks per candidate. The skrf-based full_path_z() is still
# the source of truth for rendering; these helpers exist only to make
# the optimizer tractable over 100k+ candidate BOMs.

def _abcd_ser(Z):
    """ABCD matrix for a series impedance. Z scalar or ndarray → (..., 2, 2)."""
    Z = np.asarray(Z, dtype=complex)
    M = np.empty(Z.shape + (2, 2), dtype=complex)
    M[..., 0, 0] = 1
    M[..., 0, 1] = Z
    M[..., 1, 0] = 0
    M[..., 1, 1] = 1
    return M


def _abcd_sh(Y):
    """ABCD matrix for a shunt admittance."""
    Y = np.asarray(Y, dtype=complex)
    M = np.empty(Y.shape + (2, 2), dtype=complex)
    M[..., 0, 0] = 1
    M[..., 0, 1] = 0
    M[..., 1, 0] = Y
    M[..., 1, 1] = 1
    return M


def _zin(M, ZL):
    """Z_in at port 1 given ABCD matrix M terminated in ZL."""
    A = M[..., 0, 0]
    B = M[..., 0, 1]
    C = M[..., 1, 0]
    D = M[..., 1, 1]
    return (A*ZL + B) / (C*ZL + D)


def _prefix_and_load(mode):
    """Fixed ABCD from SMA (port 1) to just before the variable chain,
    plus the chip-side load impedance. Everything here stays constant
    during the inner sweep, so compute once."""
    # ANT: C5_sh → L4_ser → C4_sh  (per build_ant_chain)
    M = (_abcd_sh(1j*omega0*C5)
         @ _abcd_ser(1j*omega0*L4)
         @ _abcd_sh(1j*omega0*C4))
    # Switch: Coff shunt, then Ron series
    M = M @ _abcd_sh(1j*omega0*SW_COFF)
    M = M @ _abcd_ser(SW_RON + 0j)
    # CAL cell: series L, then shunt C
    cal = CAL_TX if mode == 'TX' else CAL_RX
    M = M @ _abcd_ser(1j*omega0*cal['L_ser'])
    M = M @ _abcd_sh(1j*omega0*cal['C_sh'])
    ZL = tx_load_z() if mode == 'TX' else rx_load_z()
    return M, ZL


def _eval_both_ports(overrides):
    """Reference (slow) evaluator using the skrf-based full_path_z.
    Kept only for one-off verification and backward compatibility."""
    g = globals()
    saved = {k: g[k] for k in overrides}
    try:
        for k, v in overrides.items():
            g[k] = v
        _, gt = full_path_z('TX')
        _, gr = full_path_z('RX')
    finally:
        for k, v in saved.items():
            g[k] = v
    return gt, gr


def _gamma_tx_grid(C2a, C3a, L2a, L3a):
    """Vectorized TX gamma over a 4D grid of (C2, C3, L2, L3) values.
    Axis order of the returned array matches the input arrays' broadcast.
    Returns complex gamma with shape (len(C2a), len(C3a), len(L2a), len(L3a))."""
    prefix, ZL = _prefix_and_load('TX')
    M_L3 = _abcd_ser(1j*omega0*np.asarray(L3a).reshape(1, 1, 1, -1))
    M_C3 = _abcd_sh (1j*omega0*np.asarray(C3a).reshape(1, -1, 1, 1))
    M_L2 = _abcd_ser(1j*omega0*np.asarray(L2a).reshape(1, 1, -1, 1))
    M_C2 = _abcd_sh (1j*omega0*np.asarray(C2a).reshape(-1, 1, 1, 1))
    M_C1 = _abcd_ser(1/(1j*omega0*C1))
    M = prefix @ M_L3 @ M_C3 @ M_L2 @ M_C2 @ M_C1
    Zin = _zin(M, ZL)
    return (Zin - Z0) / (Zin + Z0)


def _gamma_rx_scalar(C6_val=None, C7_val=None, L5_val=None):
    """RX gamma for the current (or overridden) RX chain. Scalar complex."""
    c6 = C6 if C6_val is None else C6_val
    c7 = C7 if C7_val is None else C7_val
    l5 = L5 if L5_val is None else L5_val
    prefix, ZL = _prefix_and_load('RX')
    M_chain = (_abcd_ser(1/(1j*omega0*c6))
               @ _abcd_ser(1j*omega0*l5)
               @ _abcd_sh (1j*omega0*c7))
    M = prefix @ M_chain
    Zin = _zin(M, ZL)
    return (Zin - Z0) / (Zin + Z0)


def _gamma_rx_grid(C6a, C7a, L5a):
    """Vectorized RX gamma over (C6, C7, L5) grid."""
    prefix, ZL = _prefix_and_load('RX')
    M_C6 = _abcd_ser(1/(1j*omega0*np.asarray(C6a).reshape(-1, 1, 1)))
    M_L5 = _abcd_ser(1j*omega0*np.asarray(L5a).reshape(1, 1, -1))
    M_C7 = _abcd_sh (1j*omega0*np.asarray(C7a).reshape(1, -1, 1))
    M = prefix @ M_C6 @ M_L5 @ M_C7
    Zin = _zin(M, ZL)
    return (Zin - Z0) / (Zin + Z0)


def _fmt_cand(tag, over, gt, gr):
    parts = []
    for k in ('C2', 'C3', 'L2', 'L3', 'C4', 'C5', 'L4', 'C7', 'L5'):
        if k in over:
            unit = 'p' if k.startswith('C') else 'n'
            scale = 1e12 if k.startswith('C') else 1e9
            parts.append(f"{k}={over[k]*scale:>5.1f}{unit}")
    s11_tx = 20*np.log10(abs(gt))
    s11_rx = 20*np.log10(abs(gr))
    z_tx = Z0 * (1 + gt) / (1 - gt)
    return (f"  {tag}  " + "  ".join(parts)
            + f"   | TX {s11_tx:>6.2f}dB Z={z_tx.real:>5.1f}{z_tx.imag:+6.1f}j"
            + f"   RX {s11_rx:>6.2f}dB")


def optimize():
    import time
    t0 = time.time()
    print(f"\n=== Optimizer — F0 = {F0_HZ/1e6:.2f} MHz ===")
    print(f"  Search space: {len(STOCK_C_PF)} caps × {len(STOCK_L_NH)} inds "
          f"(0805, on hand)")

    C_arr = np.asarray(STOCK_C_PF) * 1e-12
    L_arr = np.asarray(STOCK_L_NH) * 1e-9

    # Baseline RX γ with current BOM — constant across a TX-only sweep.
    g_rx_base = _gamma_rx_scalar()
    rx_base_mag = abs(g_rx_base)
    print(f"  Baseline RX |Γ| = {rx_base_mag:.3f} "
          f"({20*np.log10(rx_base_mag):+.2f} dB)  — guardrail")

    # ---------- Phase A: TX sweep (C2, C3, L2, L3) ----------
    print("\n--- Phase A: TX sweep (C2, C3, L2, L3) ---")
    t_a = time.time()
    gamma_tx = _gamma_tx_grid(C_arr, C_arr, L_arr, L_arr)
    mag = np.abs(gamma_tx)
    # RX is invariant in Phase A — no guardrail filtering needed.
    flat_idx = np.argsort(mag.ravel())[:5]
    top_a = []
    for k in flat_idx:
        i2, i3, j2, j3 = np.unravel_index(k, mag.shape)
        over = {'C2': float(C_arr[i2]), 'C3': float(C_arr[i3]),
                'L2': float(L_arr[j2]), 'L3': float(L_arr[j3])}
        gt = complex(gamma_tx[i2, i3, j2, j3])
        top_a.append((abs(gt), over, gt, g_rx_base))
        print(_fmt_cand(f"#{len(top_a)}", over, gt, g_rx_base))
    print(f"  (Phase A: {mag.size:,} combos in {time.time()-t_a:.2f} s)")

    # ---------- Phase A': best SINGLE-component swap vs current BOM ----------
    print("\n--- Phase A': best single-component swap (vs current BOM) ---")
    cur = {'C2': C2, 'C3': C3, 'L2': L2, 'L3': L3}
    best_single = None  # (|gamma|, name, value, gamma)
    for name in ('C2', 'C3', 'L2', 'L3'):
        arr = C_arr if name.startswith('C') else L_arr
        best_v = None
        for v in arr:
            kw = dict(cur); kw[name] = float(v)
            g = _gamma_tx_grid(np.array([kw['C2']]), np.array([kw['C3']]),
                                np.array([kw['L2']]), np.array([kw['L3']]))
            gv = complex(g.ravel()[0])
            if best_v is None or abs(gv) < abs(best_v[1]):
                best_v = (float(v), gv)
        val, gt = best_v
        print(_fmt_cand(f"swap {name}", {name: val}, gt, g_rx_base))
        if best_single is None or abs(gt) < best_single[0]:
            best_single = (abs(gt), name, val, gt)

    bm, bn, bv, _ = best_single
    bunit, bscale = ('pF', 1e12) if bn.startswith('C') else ('nH', 1e9)
    cur_unit, cur_scale = bunit, bscale
    cur_val = cur[bn] * cur_scale
    print(f"\n  >>> Recommended single swap: {bn}  "
          f"{cur_val:.1f} {cur_unit} -> {bv*bscale:.1f} {bunit}  "
          f"(predicted TX S11 = {20*np.log10(bm):.2f} dB)")

    best_a = top_a[0]
    a_good = 20*np.log10(best_a[0]) <= -15.0

    # ---------- Phase B: ANT fallback ----------
    top_b = []
    if a_good:
        print("\n--- Phase B: skipped (Phase A already reaches ≤ -15 dB) ---")
    else:
        print("\n--- Phase B: ANT fallback (+C4, C5, L4) ---")
        t_b = time.time()
        # Strategy: outer loop over ANT (C4, C5, L4) on a coarse grid (every
        # 2nd stock value); inner TX sweep is vectorized. Then refine TX
        # over full stock at the winning ANT combo.
        C_coarse = C_arr[::2]
        L_coarse = L_arr[::2]
        g = globals()
        saved = {'C4': g['C4'], 'C5': g['C5'], 'L4': g['L4']}
        best_cost = np.inf
        best_over = None
        try:
            for c4 in C_coarse:
                for c5 in C_coarse:
                    for l4 in L_coarse:
                        g['C4'] = float(c4); g['C5'] = float(c5); g['L4'] = float(l4)
                        # Also RX can shift because ANT is shared.
                        gr = _gamma_rx_scalar()
                        gtx = _gamma_tx_grid(C_coarse, C_coarse, L_coarse, L_coarse)
                        cost = np.abs(gtx) + 0.5*abs(gr)
                        k = int(np.argmin(cost))
                        i2, i3, j2, j3 = np.unravel_index(k, cost.shape)
                        c = float(cost.flat[k])
                        if c < best_cost:
                            best_cost = c
                            best_over = {
                                'C2': float(C_coarse[i2]), 'C3': float(C_coarse[i3]),
                                'L2': float(L_coarse[j2]), 'L3': float(L_coarse[j3]),
                                'C4': float(c4), 'C5': float(c5), 'L4': float(l4),
                            }
            # Refine TX at the winning ANT, full stock.
            g['C4'] = best_over['C4']; g['C5'] = best_over['C5']; g['L4'] = best_over['L4']
            gr = _gamma_rx_scalar()
            gtx = _gamma_tx_grid(C_arr, C_arr, L_arr, L_arr)
            cost = np.abs(gtx) + 0.5*abs(gr)
            flat_idx = np.argsort(cost.ravel())[:3]
            for k in flat_idx:
                i2, i3, j2, j3 = np.unravel_index(k, cost.shape)
                over = dict(best_over,
                            C2=float(C_arr[i2]), C3=float(C_arr[i3]),
                            L2=float(L_arr[j2]), L3=float(L_arr[j3]))
                gt_val = complex(gtx[i2, i3, j2, j3])
                top_b.append((float(cost.flat[k]), over, gt_val, gr))
                print(_fmt_cand(f"#{len(top_b)}", over, gt_val, gr))
        finally:
            for k, v in saved.items():
                g[k] = v
        print(f"  (Phase B: vectorized coarse + refine in {time.time()-t_b:.2f} s)")

    # ---------- Phase C: RX single-part tweak ----------
    winner_over = (top_b[0][1] if top_b else best_a[1])
    t_c = time.time()
    g = globals()
    saved = {k: g[k] for k in ('C2', 'C3', 'L2', 'L3', 'C4', 'C5', 'L4')
             if k in winner_over}
    try:
        for k in saved:
            g[k] = winner_over[k]
        g_rx_after = _gamma_rx_scalar()
        rx_after_db = 20*np.log10(abs(g_rx_after))
        print(f"\n--- Phase C: RX single-part tweak "
              f"(current RX S11 at winner = {rx_after_db:+.2f} dB) ---")

        best_tweak = None
        # C7 sweep (vectorized)
        for val in C_arr:
            gr = _gamma_rx_scalar(C7_val=float(val))
            improv = rx_after_db - 20*np.log10(abs(gr))
            if best_tweak is None or improv > best_tweak[0]:
                best_tweak = (improv, 'C7', float(val), None, complex(gr))
        for val in L_arr:
            gr = _gamma_rx_scalar(L5_val=float(val))
            improv = rx_after_db - 20*np.log10(abs(gr))
            if improv > best_tweak[0]:
                best_tweak = (improv, 'L5', float(val), None, complex(gr))
    finally:
        for k, v in saved.items():
            g[k] = v

    if best_tweak and best_tweak[0] >= 3.0:
        improv, part, val, _, gr = best_tweak
        unit = 'pF' if part.startswith('C') else 'nH'
        scale = 1e12 if part.startswith('C') else 1e9
        print(f"  Tweak: {part} = {val*scale:.1f} {unit}  "
              f"(+{improv:.1f} dB RX improvement)")
        # Re-evaluate TX gamma at winner for display.
        gt_disp = top_b[0][2] if top_b else best_a[2]
        print(_fmt_cand("==>", dict(winner_over, **{part: val}), gt_disp, gr))
    else:
        improv = best_tweak[0] if best_tweak else 0
        print(f"  No tweak warranted (best single-part change = "
              f"{improv:+.1f} dB). Leave RX at C7={C7*1e12:.1f} pF, "
              f"L5={L5*1e9:.1f} nH.")
    print(f"  (Phase C: {time.time()-t_c:.2f} s)")

    print(f"\n  Total optimizer time: {time.time()-t0:.2f} s")

    # Apply the winner to in-memory globals so the final render previews
    # where the board would land after rework. This does NOT edit the file;
    # re-run without --optimize after committing the BOM change.
    final_over = dict(winner_over)
    if best_tweak and best_tweak[0] >= 3.0:
        final_over[best_tweak[1]] = best_tweak[2]
    g = globals()
    for k, v in final_over.items():
        g[k] = v
    parts = "  ".join(
        f"{k}={v*(1e12 if k.startswith('C') else 1e9):.1f}"
        f"{'p' if k.startswith('C') else 'n'}"
        for k, v in final_over.items())
    print(f"\n  Applying winner to in-memory BOM for preview render:")
    print(f"    {parts}")
    print("  Smith plot will show VNA (circles, current board) vs Model")
    print("  (squares, predicted post-rework). To commit, edit C2..L5 in")
    print("  the file to match, then re-run without --optimize.\n")


if args.optimize:
    optimize()


# ============================================================
# Compute / print / render
# ============================================================
OUT_PATH = 'src/vna_smith.png'

# Names of the editable knobs, in display order. Used by the REPL.
COMPONENTS = ['C1', 'C2', 'C3', 'C6', 'C7', 'C4', 'C5',
              'L2', 'L3', 'L4', 'L5']


def compute_results():
    out = {}
    for mode in ('TX', 'RX'):
        model_z, model_g = full_path_z(mode)
        vna_z = VNA[mode]['z']
        vna_g = (vna_z - Z0) / (vna_z + Z0)
        out[mode] = {
            'vna_z': vna_z, 'vna_g': vna_g,
            'vna_s11_db': 20*np.log10(abs(vna_g)),
            'model_z': model_z, 'model_g': model_g,
            'model_s11_db': 20*np.log10(abs(model_g)),
        }
    return out


def print_summary(results):
    print(f"\nVNA Smith Plot — F0 = {F0_HZ/1e6:.2f} MHz\n")
    print(f"  {'Mode':<6} {'VNA S11':>10} {'Model S11':>10}   "
          f"{'VNA Z':>18}   {'Model Z':>18}")
    print(f"  {'-'*72}")
    for mode in ('TX', 'RX'):
        r = results[mode]
        print(f"  {mode:<6} {r['vna_s11_db']:>8.2f}dB {r['model_s11_db']:>8.2f}dB   "
              f"{r['vna_z'].real:>7.1f}{r['vna_z'].imag:>+7.1f}j Ω   "
              f"{r['model_z'].real:>7.1f}{r['model_z'].imag:>+7.1f}j Ω")
    print()


# Persistent figure handles for interactive mode (None until first render).
_FIG = None
_AX = None


def _draw_into(ax, results):
    ax.clear()
    rf.plotting.smith(ax=ax, draw_labels=True, chart_type='z')
    colors = {'TX': 'tab:red', 'RX': 'tab:blue'}
    for mode in ('TX', 'RX'):
        r = results[mode]
        c = colors[mode]
        ax.plot(r['vna_g'].real, r['vna_g'].imag, 'o', color=c, markersize=14,
                markeredgecolor='black', markeredgewidth=1.2,
                label=f'{mode} VNA: {r["vna_s11_db"]:.1f} dB, '
                      f'{r["vna_z"].real:.0f}{r["vna_z"].imag:+.0f}j Ω')
        ax.plot(r['model_g'].real, r['model_g'].imag, 's',
                markerfacecolor='none', markeredgecolor=c, markersize=14,
                markeredgewidth=2,
                label=f'{mode} model: {r["model_s11_db"]:.1f} dB, '
                      f'{r["model_z"].real:.0f}{r["model_z"].imag:+.0f}j Ω')
    ax.plot(0, 0, 'k+', markersize=16, markeredgewidth=2, label='50 Ω')
    ax.set_title(f'Si4463 Matching Network — VNA vs Model @ {F0_HZ/1e6:.2f} MHz')
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), fontsize=9,
              frameon=True)


def render_once():
    global _FIG, _AX
    results = compute_results()
    print_summary(results)

    if args.interactive:
        if _FIG is None:
            plt.ion()
            _FIG, _AX = plt.subplots(figsize=(9, 9))
            _FIG.canvas.manager.set_window_title('VNA Smith Plot')
        _draw_into(_AX, results)
        _FIG.tight_layout()
        _FIG.canvas.draw_idle()
        _FIG.canvas.flush_events()
        # Also save PNG for record-keeping
        _FIG.savefig(OUT_PATH, dpi=150, bbox_inches='tight')
        print(f"Saved: {OUT_PATH}")
    else:
        fig, ax = plt.subplots(figsize=(9, 9))
        _draw_into(ax, results)
        fig.tight_layout()
        fig.savefig(OUT_PATH, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved: {OUT_PATH}")


# ============================================================
# Interactive REPL
# ============================================================
def _parse_value(s):
    """'4.7p' → 4.7e-12, '27n' → 27e-9, '1.5e-9' → 1.5e-9."""
    s = s.strip().lower().replace('ω', '').replace('f', '')
    if not s:
        raise ValueError("empty value")
    suffix = ''
    if s[-1] in _SUFFIX and not s[-1].isdigit() and s[-1] != '.':
        suffix = s[-1]
        s = s[:-1]
    return float(s) * _SUFFIX[suffix]


def _fmt(name, val):
    unit = 'pF' if name.startswith('C') else 'nH'
    scale = 1e12 if name.startswith('C') else 1e9
    return f"{val*scale:>8.2f} {unit}"


def _print_components():
    print("  Components:")
    for n in COMPONENTS:
        print(f"    {n:<4} = {_fmt(n, globals()[n])}")
    print(f"  CAL_TX = {{'L_ser': {CAL_TX['L_ser']*1e9:+.3f}e-9, "
          f"'C_sh': {CAL_TX['C_sh']*1e12:+.3f}e-12}}")
    print(f"  CAL_RX = {{'L_ser': {CAL_RX['L_ser']*1e9:+.3f}e-9, "
          f"'C_sh': {CAL_RX['C_sh']*1e12:+.3f}e-12}}")


def _print_help():
    print("""
  Commands:
    <name> <value>     Set a component, e.g.  C1 4.7p   or   L5 27n
    show               Print current component values + CAL
    fit                Refit CAL_TX/CAL_RX to match current VNA{}
    vna TX <z>         Update VNA TX impedance, e.g.  vna TX 9.1-38j
    vna RX <z>         Update VNA RX impedance
    plot               Re-render src/vna_smith.png with current values
    help               Show this help
    quit / q           Exit
""")


def interactive_loop():
    print("\n=== Interactive mode — type 'help' for commands ===")
    _print_components()
    while True:
        try:
            line = input("\nvna> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        cmd, *rest = line.split()
        cmd_lc = cmd.lower()

        if cmd_lc in ('q', 'quit', 'exit'):
            break
        if cmd_lc == 'help':
            _print_help()
            continue
        if cmd_lc == 'show':
            _print_components()
            continue
        if cmd_lc == 'plot':
            render_once()
            continue
        if cmd_lc == 'fit':
            tx, rx = fit_calibration()
            globals()['CAL_TX'] = tx
            globals()['CAL_RX'] = rx
            print("Refit done.")
            _print_components()
            render_once()
            continue
        if cmd_lc == 'vna' and len(rest) >= 2:
            mode = rest[0].upper()
            if mode not in VNA:
                print(f"  ! unknown VNA mode '{mode}'")
                continue
            try:
                z = complex(rest[1].replace('i', 'j'))
            except ValueError as e:
                print(f"  ! bad complex value: {e}")
                continue
            VNA[mode]['z'] = z
            gamma = (z - Z0) / (z + Z0)
            VNA[mode]['s11_db'] = 20*np.log10(abs(gamma))
            print(f"  VNA[{mode}] = {z}")
            continue

        # Otherwise: component assignment, "C1 4.7p" or "C1=4.7p"
        if '=' in line and len(rest) == 0:
            cmd, val_str = line.split('=', 1)
            cmd = cmd.strip()
            cmd_lc = cmd.lower()
            rest = [val_str]
        if cmd in COMPONENTS and rest:
            try:
                val = _parse_value(rest[0])
            except (ValueError, KeyError) as e:
                print(f"  ! bad value: {e}")
                continue
            globals()[cmd] = val
            print(f"  {cmd} = {_fmt(cmd, val)}")
            render_once()
            continue

        print(f"  ! unknown command: {line!r}  (try 'help')")


# ============================================================
# Main
# ============================================================
render_once()
if args.interactive:
    interactive_loop()
