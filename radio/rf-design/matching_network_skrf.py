#!/usr/bin/env python3
"""
scikit-rf Matching Network Optimizer for Si4463 at 437.08 MHz
==============================================================

Uses scikit-rf for S-parameter analysis with discrete grid search
over LCSC-available standard component values.

Includes PCB trace parasitic models (microstrip transmission lines)
extracted from KiCad layout to match real-world performance.

Usage:
  python src/matching_network_skrf.py            # parasitic-aware (default)
  python src/matching_network_skrf.py --ideal     # ideal lumped model (legacy)
"""

import argparse
import matplotlib
matplotlib.use('Agg')

import numpy as np
import skrf as rf
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='Si4463 Matching Network Optimizer')
parser.add_argument('--ideal', action='store_true',
                    help='Use ideal lumped model without PCB trace parasitics')
parser.add_argument('--vna', action='store_true',
                    help='Compare model predictions against VNA measurements')
args = parser.parse_args()
USE_PARASITICS = not args.ideal

# ============================================================
# DESIGN PARAMETERS
# ============================================================
f0_hz = 437.08e6
f0_str = '437.08MHz'
Z0 = 50.0

# Si4463 PA — Class-E switching amplifier model (AN648 sec 4.2)
CSHUNT = 2.5e-12                # PA drain-source capacitance (AN648 sec 4.2.3)
LCHOKE = 220e-9                 # DC feed inductor, 434 MHz band (AN648 Table 2)
omega0 = 2 * np.pi * f0_hz
RLOAD = 0.2815 / (omega0 * CSHUNT) * np.cos(np.radians(49.0524))  # ~26.9 Ω Class-E (AN648 Eq. 2)

# Si4463 RX LNA — single-ended equivalent (AN643 Table 1, 434 MHz)
RLNA_SE = 240.0      # 480 Ω differential / 2
CLNA_SE = 1.94e-12   # 0.97 pF differential * 2

# AS179-92LF SPDT RF Switch — datasheet typical specs at 400 MHz
SW_RON = 1.5           # on-throw series resistance (Ω)
SW_COFF = 0.3e-12      # off-throw capacitance to common (F)
SW_IL_DB = 0.5         # insertion loss (dB) — used for validation only


# ============================================================
# PCB STACKUP & MICROSTRIP TRACE GEOMETRY (from KiCad layout)
# ============================================================
# 2-layer FR4 board (JLCPCB standard)
PCB_H = 1.465e-3      # dielectric thickness, F.Cu to B.Cu (m)
PCB_ER = 4.6           # FR4 relative permittivity
PCB_TAND = 0.02        # FR4 loss tangent
PCB_T = 0.035e-3       # copper thickness (m)

# Trace segments between matching network components.
# Format: list of (width_mm, length_mm) for each segment between components.
# Measured from KiCad routed trace segments (rev3 layout, IC1 flipped to rot -90°).
#
# IC1 at (136.7, 88.5) rot -90°, U1 at (137.85, 79.725) rot 90°
# Flipped IC1: J2/J3 face UP toward U1, J1 faces DOWN toward SMA
# Net mapping: J2=TX throw (net 11), J3=RX throw (net 8), J1=ANT (net 12)
# TX crosses RX via routing around IC1 body (J2 at top-left, TX column at right)
#
# TX: 50Ω(switch) → L3 → C3(sh) → L2 → C2(sh) → C1 → PA
# Signal path: IC1 J2 → C8 → C3/L3 → L2 → C2 → C1/L1 → U1 TX
TX_TRACES_MM = [
    (0.4,  4.48),  # switch→C8 (3.52) + C8→L3 (0.96), C8 is RF short
    (0.4,  1.38),  # L3 → C3 (shunt branch)
    (0.4,  0.9),   # L2 → C2
    (0.4,  1.0),   # C1 → L2
    (0.4,  2.33),  # C1→L1 (0.825) + L1→TX pin (1.5), L1 taps DC feed
]
# RX: 50Ω(switch) → C6 → L5 → C7(sh) → LNA
# Signal path: IC1 J3 → C6 → L5 → C7 → U1 RXp
RX_TRACES_MM = [
    (0.4,  1.87),  # switch pin → C6
    (0.4,  0.75),  # C6 → L5
    (0.4,  0.95),  # L5 → C7 (shunt branch)
    (0.4,  1.0),   # L5 → U1 RXp
]
# ANT: 50Ω(switch) → C5(sh) → L4 → C4(sh) → 50Ω
# Signal path: IC1 J1 → C4 → L4 → C5 → 50Ω microstrip
ANT_TRACES_MM = [
    (0.4,  0.0),   # L4 pad directly on 50Ω microstrip (no trace)
    (0.4,  0.9),   # C5 → L4
    (0.4,  1.2),   # C4 → L4
    (0.4,  3.85),  # C4→C9 (0.8) + C9→switch (3.05), C9 is RF short
]


# ============================================================
# MICROSTRIP TRANSMISSION LINE MODEL (Hammerstad-Jensen)
# ============================================================
def microstrip_params(w, h, er, t=0.0):
    """
    Compute Z0 and effective epsilon for a microstrip line.

    Uses Hammerstad-Jensen closed-form equations.
    Reference: Hammerstad & Jensen, "Accurate Models for Microstrip
    Computer-Aided Design", IEEE-MTT Symposium, 1980.

    Parameters:
        w: trace width (m)
        h: substrate height (m)
        er: substrate relative permittivity
        t: conductor thickness (m), optional
    Returns:
        (Z0, ep_eff) tuple
    """
    # Effective width correction for finite conductor thickness
    if t > 0:
        w_eff = w + (t / np.pi) * (1 + np.log(2 * h / t))
    else:
        w_eff = w

    u = w_eff / h

    # Effective permittivity (Hammerstad-Jensen)
    a = 1 + (1.0 / 49.0) * np.log((u**4 + (u / 52.0)**2) / (u**4 + 0.432)) \
        + (1.0 / 18.7) * np.log(1 + (u / 18.1)**3)
    b_hj = 0.564 * ((er - 0.9) / (er + 3.0))**0.053
    ep_eff = (er + 1) / 2.0 + (er - 1) / 2.0 * (1 + 10.0 / u)**(-a * b_hj)

    # Characteristic impedance
    f_u = 6.0 + (2.0 * np.pi - 6.0) * np.exp(-(30.666 / u)**0.7528)
    Z0_ms = (60.0 / np.sqrt(ep_eff)) * np.log(f_u / u + np.sqrt(1 + (2.0 / u)**2))

    return Z0_ms, ep_eff


# ============================================================
# FREQUENCY SWEEP & MEDIA SETUP
# ============================================================
freq = rf.Frequency(start=100, stop=1500, npoints=1401, unit='MHz')
line = rf.media.DefinedGammaZ0(frequency=freq, z0=Z0)


def make_microstrip_line(w_mm, length_mm):
    """Create a scikit-rf 2-port Network for a microstrip trace segment.

    Builds S-parameters directly from ABCD with 50Ω port reference,
    avoiding z0_port mismatch when cascading with lumped elements.
    """
    w_m = w_mm * 1e-3
    d_m = length_mm * 1e-3
    z0_line, ep_eff = microstrip_params(w_m, PCB_H, PCB_ER, PCB_T)

    beta = 2 * np.pi * freq.f * np.sqrt(ep_eff) / 299792458.0
    theta = beta * d_m  # electrical length at each frequency

    # ABCD matrix for transmission line
    A = np.cos(theta)
    B = 1j * z0_line * np.sin(theta)
    C = 1j * np.sin(theta) / z0_line
    D = np.cos(theta)

    # Convert ABCD → S referenced to Z0 (50Ω)
    denom = A + B / Z0 + C * Z0 + D
    s = np.zeros((len(freq.f), 2, 2), dtype=complex)
    s[:, 0, 0] = (A + B / Z0 - C * Z0 - D) / denom
    s[:, 0, 1] = 2 * (A * D - B * C) / denom
    s[:, 1, 0] = 2.0 / denom
    s[:, 1, 1] = (-A + B / Z0 - C * Z0 + D) / denom

    return rf.Network(frequency=freq, s=s, z0=Z0)


# Pre-compute trace segment networks (constant, don't depend on component values)
if USE_PARASITICS:
    TX_SEGS = [make_microstrip_line(w, d) for w, d in TX_TRACES_MM]
    RX_SEGS = [make_microstrip_line(w, d) for w, d in RX_TRACES_MM]
    ANT_SEGS = [make_microstrip_line(w, d) for w, d in ANT_TRACES_MM]

    # Print microstrip parameters for reference
    print("Microstrip Trace Parameters:")
    for label, traces in [("TX", TX_TRACES_MM), ("RX", RX_TRACES_MM), ("ANT", ANT_TRACES_MM)]:
        for i, (w, d) in enumerate(traces):
            z0, ep = microstrip_params(w*1e-3, PCB_H, PCB_ER, PCB_T)
            theta_deg = 360 * d*1e-3 * f0_hz * np.sqrt(ep) / 299792458.0
            L_equiv = z0 * np.sin(np.radians(theta_deg)) / (2 * np.pi * f0_hz)
            print(f"  {label}[{i}]: w={w:.1f}mm  d={d:.1f}mm  "
                  f"Z0={z0:.1f}Ω  εeff={ep:.2f}  "
                  f"θ={theta_deg:.1f}°  L_equiv≈{L_equiv*1e9:.1f}nH")
    print()


# ============================================================
# LOAD IMPEDANCE (frequency-dependent, as arrays)
# ============================================================
omega_arr = 2 * np.pi * freq.f

def tx_load_z():
    """PA load impedance: RLOAD || CSHUNT || LCHOKE."""
    Y = 1.0/RLOAD + 1j*omega_arr*CSHUNT + 1.0/(1j*omega_arr*LCHOKE)
    return 1.0 / Y

def rx_load_z():
    """RX LNA impedance: RLNA_SE || CLNA_SE."""
    Y = 1.0/RLNA_SE + 1j*omega_arr*CLNA_SE
    return 1.0 / Y

TX_ZL = tx_load_z()   # shape (nfreq,)
RX_ZL = rx_load_z()
ANT_ZL = np.full_like(omega_arr, Z0, dtype=complex)  # 50 Ω


# ============================================================
# 2-PORT NETWORK BUILDERS (with optional trace parasitics)
# ============================================================
def build_tx_2port(L3, C3, L2, C2, C1):
    """TX matching chain as 2-port."""
    core = (line.inductor(L3)
            ** line.shunt_capacitor(C3)
            ** line.inductor(L2)
            ** line.shunt_capacitor(C2)
            ** line.capacitor(C1))
    if not USE_PARASITICS:
        return core
    return (TX_SEGS[0]
            ** line.inductor(L3)
            ** TX_SEGS[1]
            ** line.shunt_capacitor(C3)
            ** line.inductor(L2)
            ** TX_SEGS[2]
            ** line.shunt_capacitor(C2)
            ** TX_SEGS[3]
            ** line.capacitor(C1)
            ** TX_SEGS[4])


def build_rx_2port(C6, L5, C7):
    """RX matching chain as 2-port."""
    core = (line.capacitor(C6)
            ** line.inductor(L5)
            ** line.shunt_capacitor(C7))
    if not USE_PARASITICS:
        return core
    return (RX_SEGS[0]
            ** line.capacitor(C6)
            ** RX_SEGS[1]
            ** line.inductor(L5)
            ** RX_SEGS[2]
            ** line.shunt_capacitor(C7)
            ** RX_SEGS[3])


def build_ant_2port(C5, L4, C4):
    """Antenna matching chain as 2-port."""
    core = (line.shunt_capacitor(C5)
            ** line.inductor(L4)
            ** line.shunt_capacitor(C4))
    if not USE_PARASITICS:
        return core
    return (ANT_SEGS[0]
            ** line.shunt_capacitor(C5)
            ** ANT_SEGS[1]
            ** line.inductor(L4)
            ** ANT_SEGS[2]
            ** line.shunt_capacitor(C4)
            ** ANT_SEGS[3])


# ============================================================
# AS179-92LF SWITCH MODEL & FULL-PATH BUILDERS
# ============================================================
# The SPDT switch is modeled at the junction between ANT and TX/RX networks.
# On-throw: series resistance (SW_RON)
# Off-throw: shunt capacitance (SW_COFF) from common node to ground
# When TX is active: on-path to TX, off-shunt from RX throw
# When RX is active: on-path to RX, off-shunt from TX throw

def build_switch_on():
    """On-throw: series resistor (Ron)."""
    return line.resistor(SW_RON)

def build_switch_off_shunt():
    """Off-throw: shunt capacitance (Coff) loading the common node."""
    return line.shunt_capacitor(SW_COFF)


def build_full_tx_path(L3, C3, L2, C2, C1, C5, L4, C4):
    """Full path from SMA (50Ω) → ANT network → switch(TX on, RX off) → TX network → PA load.
    Returns 2-port; terminate port 2 with TX_ZL for loaded S11 at port 1."""
    ant = build_ant_2port(C5, L4, C4)
    sw_off = build_switch_off_shunt()
    sw_on = build_switch_on()
    tx = build_tx_2port(L3, C3, L2, C2, C1)
    return ant ** sw_off ** sw_on ** tx


def build_full_rx_path(C6, L5, C7, C5, L4, C4):
    """Full path from SMA (50Ω) → ANT network → switch(RX on, TX off) → RX network → LNA load.
    Returns 2-port; terminate port 2 with RX_ZL for loaded S11 at port 1."""
    ant = build_ant_2port(C5, L4, C4)
    sw_off = build_switch_off_shunt()
    sw_on = build_switch_on()
    rx = build_rx_2port(C6, L5, C7)
    return ant ** sw_off ** sw_on ** rx


# ============================================================
# LOADED S11 COMPUTATION
# ============================================================
def loaded_s11(ntw_2port, Z_load_arr):
    """
    Compute S11 of a 2-port network terminated with a frequency-dependent load.

    Γ_in = S11 + S12*S21*Γ_L / (1 - S22*Γ_L)

    Returns complex Γ_in array (nfreq,).
    """
    S = ntw_2port.s  # shape (nfreq, 2, 2)
    s11 = S[:, 0, 0]
    s12 = S[:, 0, 1]
    s21 = S[:, 1, 0]
    s22 = S[:, 1, 1]
    gamma_L = (Z_load_arr - Z0) / (Z_load_arr + Z0)
    return s11 + s12 * s21 * gamma_L / (1 - s22 * gamma_L)


def loaded_s11_db_at_f0(ntw_2port, Z_load_arr):
    """Return loaded |S11| in dB at f0."""
    gamma_in = loaded_s11(ntw_2port, Z_load_arr)
    idx = np.argmin(np.abs(freq.f - f0_hz))
    mag = np.abs(gamma_in[idx])
    return 20 * np.log10(max(mag, 1e-15))


def loaded_s11_mag_at_f0(ntw_2port, Z_load_arr):
    """Return loaded |S11| linear at f0."""
    gamma_in = loaded_s11(ntw_2port, Z_load_arr)
    idx = np.argmin(np.abs(freq.f - f0_hz))
    return np.abs(gamma_in[idx])


# ============================================================
# LCSC-AVAILABLE STANDARD COMPONENT VALUES (0805)
# ============================================================
# Capacitors: 0805 C0G/NP0, confirmed in-stock on LCSC
LCSC_CAP_PF = np.array([1.0, 1.2, 1.5, 1.8, 2.0, 2.2, 2.7, 3.0, 3.3, 3.9,
                         4.7, 5.6, 6.8, 8.2, 10, 12, 15, 18, 22, 27, 33, 39,
                         47, 56, 68, 82, 100])

# Inductors: 0805 Murata LQW18AN series, confirmed in-stock on LCSC
LCSC_IND_NH = np.array([3.3, 3.9, 4.7, 5.6, 6.8, 8.2, 10, 12, 15, 18, 22, 27,
                         33, 39, 47, 56, 68, 82, 100])


# ============================================================
# FAST COST FUNCTIONS (raw ABCD math, no scikit-rf overhead)
# ============================================================
# Evaluate at only the frequencies needed by the cost functions:
#   [0]   = 437.08 MHz    (f0 — primary target)
#   [1,2] = 430, 440 MHz  (band edges for worst-case S11)
#   [3]   = 874 MHz       (2f0 harmonic rejection)
#   [4]   = 1311 MHz      (3f0 harmonic rejection)
_cost_f = np.array([f0_hz, 430e6, 440e6, 2*f0_hz, 3*f0_hz])
_cost_jw = 1j * 2 * np.pi * _cost_f  # shape (5,)
_f0_idx = 0                    # index of f0 in _cost_f
_band_idx = np.array([1, 2])   # indices into _cost_f for band edges

# Load reflection coefficients at cost frequencies
_tx_zl_cost = 1.0 / (1.0/RLOAD + _cost_jw*CSHUNT + 1.0/(_cost_jw*LCHOKE))
_tx_gl_cost = (_tx_zl_cost - Z0) / (_tx_zl_cost + Z0)
_rx_zl_cost = 1.0 / (1.0/RLNA_SE + _cost_jw*CLNA_SE)
_rx_gl_cost = (_rx_zl_cost - Z0) / (_rx_zl_cost + Z0)
_ant_gl_cost = np.zeros(len(_cost_f), dtype=complex)  # 50 Ω load → Γ_L = 0

# Identity matrix at each cost frequency
_n_cost = len(_cost_f)
_I = np.broadcast_to(np.eye(2, dtype=complex), (_n_cost, 2, 2)).copy()


def _abcd_series_L(L_nH):
    """ABCD for series inductor. Returns shape (n_freq, 2, 2)."""
    Z = _cost_jw * (L_nH * 1e-9)
    M = _I.copy()
    M[:, 0, 1] = Z
    return M

def _abcd_series_C(C_pF):
    """ABCD for series capacitor. Returns shape (n_freq, 2, 2)."""
    Z = 1.0 / (_cost_jw * (C_pF * 1e-12))
    M = _I.copy()
    M[:, 0, 1] = Z
    return M

def _abcd_shunt_C(C_pF):
    """ABCD for shunt capacitor. Returns shape (n_freq, 2, 2)."""
    Y = _cost_jw * (C_pF * 1e-12)
    M = _I.copy()
    M[:, 1, 0] = Y
    return M

def _abcd_shunt_L(L_nH):
    """ABCD for shunt inductor. Returns shape (n_freq, 2, 2)."""
    Y = 1.0 / (_cost_jw * (L_nH * 1e-9))
    M = _I.copy()
    M[:, 1, 0] = Y
    return M

def _abcd_microstrip(w_mm, length_mm):
    """ABCD matrix for a microstrip trace segment at cost frequencies."""
    w_m = w_mm * 1e-3
    d_m = length_mm * 1e-3
    z0_line, ep_eff = microstrip_params(w_m, PCB_H, PCB_ER, PCB_T)

    beta = 2 * np.pi * _cost_f * np.sqrt(ep_eff) / 299792458.0
    theta = beta * d_m

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    M = np.zeros((len(_cost_f), 2, 2), dtype=complex)
    M[:, 0, 0] = cos_t
    M[:, 0, 1] = 1j * z0_line * sin_t
    M[:, 1, 0] = 1j * sin_t / z0_line
    M[:, 1, 1] = cos_t
    return M


def _cascade(*matrices):
    """Cascade ABCD matrices. Each shape (n_freq, 2, 2)."""
    result = matrices[0]
    for M in matrices[1:]:
        result = np.einsum('fij,fjk->fik', result, M)
    return result

def _abcd_to_loaded_s11(abcd, gamma_L):
    """Convert ABCD to S-params and compute loaded S11."""
    A, B, C, D = abcd[:, 0, 0], abcd[:, 0, 1], abcd[:, 1, 0], abcd[:, 1, 1]
    denom = A + B/Z0 + C*Z0 + D
    s11 = (A + B/Z0 - C*Z0 - D) / denom
    s12 = 2*(A*D - B*C) / denom
    s21 = 2.0 / denom
    s22 = (-A + B/Z0 - C*Z0 + D) / denom
    return s11 + s12 * s21 * gamma_L / (1 - s22 * gamma_L)

def _abcd_s21_mag(abcd, idx):
    """Get |S21| at a specific frequency index from ABCD."""
    A, B, C, D = abcd[idx, 0, 0], abcd[idx, 0, 1], abcd[idx, 1, 0], abcd[idx, 1, 1]
    return abs(2.0 / (A + B/Z0 + C*Z0 + D))


# Pre-compute constant trace ABCD matrices for grid search
if USE_PARASITICS:
    _TX_ABCD = [_abcd_microstrip(w, d) for w, d in TX_TRACES_MM]
    _RX_ABCD = [_abcd_microstrip(w, d) for w, d in RX_TRACES_MM]
    _ANT_ABCD = [_abcd_microstrip(w, d) for w, d in ANT_TRACES_MM]


# AS179 switch ABCD matrices (frequency-independent R and C models)
def _abcd_series_R(R):
    """ABCD for series resistor."""
    M = _I.copy()
    M[:, 0, 1] = R
    return M

def _abcd_shunt_Coff():
    """ABCD for off-throw shunt capacitance."""
    Y = _cost_jw * SW_COFF
    M = _I.copy()
    M[:, 1, 0] = Y
    return M

_SW_ON_ABCD = _abcd_series_R(SW_RON)
_SW_OFF_ABCD = _abcd_shunt_Coff()


def _build_ant_abcd(C5, L4, C4):
    """Build ANT network ABCD at cost frequencies."""
    if USE_PARASITICS:
        return _cascade(
            _ANT_ABCD[0],
            _abcd_shunt_C(C5),
            _ANT_ABCD[1],
            _abcd_series_L(L4),
            _ANT_ABCD[2],
            _abcd_shunt_C(C4),
            _ANT_ABCD[3],
        )
    else:
        return _cascade(_abcd_shunt_C(C5), _abcd_series_L(L4), _abcd_shunt_C(C4))

def _build_tx_abcd(L3, C3, L2, C2, C1):
    """Build TX network ABCD at cost frequencies."""
    if USE_PARASITICS:
        return _cascade(
            _TX_ABCD[0],
            _abcd_series_L(L3),
            _TX_ABCD[1],
            _abcd_shunt_C(C3),
            _abcd_series_L(L2),
            _TX_ABCD[2],
            _abcd_shunt_C(C2),
            _TX_ABCD[3],
            _abcd_series_C(C1),
            _TX_ABCD[4],
        )
    else:
        return _cascade(_abcd_series_L(L3), _abcd_shunt_C(C3),
                         _abcd_series_L(L2), _abcd_shunt_C(C2),
                         _abcd_series_C(C1))

def _build_rx_abcd(C6, L5, C7):
    """Build RX network ABCD at cost frequencies."""
    if USE_PARASITICS:
        return _cascade(
            _RX_ABCD[0],
            _abcd_series_C(C6),
            _RX_ABCD[1],
            _abcd_series_L(L5),
            _RX_ABCD[2],
            _abcd_shunt_C(C7),
            _RX_ABCD[3],
        )
    else:
        return _cascade(_abcd_series_C(C6), _abcd_series_L(L5), _abcd_shunt_C(C7))


def tx_cost_vals(L3, C3, L2, C2, C1):
    if USE_PARASITICS:
        abcd = _cascade(
            _TX_ABCD[0],
            _abcd_series_L(L3),
            _TX_ABCD[1],
            _abcd_shunt_C(C3),
            _abcd_series_L(L2),
            _TX_ABCD[2],
            _abcd_shunt_C(C2),
            _TX_ABCD[3],
            _abcd_series_C(C1),
            _TX_ABCD[4],
        )
    else:
        abcd = _cascade(_abcd_series_L(L3), _abcd_shunt_C(C3),
                         _abcd_series_L(L2), _abcd_shunt_C(C2),
                         _abcd_series_C(C1))
    gamma_in = _abcd_to_loaded_s11(abcd, _tx_gl_cost)
    # Primary: S11 at f0.  Secondary: band-edge worst case.  Tertiary: harmonic rejection.
    s11_f0 = np.abs(gamma_in[_f0_idx])
    s11_band = np.max(np.abs(gamma_in[_band_idx]))
    s21_2f0 = _abcd_s21_mag(abcd, 3)
    s21_3f0 = _abcd_s21_mag(abcd, 4)
    return s11_f0 + 0.3 * s11_band + 0.2 * s21_2f0 + 0.1 * s21_3f0


def rx_cost_vals(C6, L5, C7):
    if USE_PARASITICS:
        abcd = _cascade(
            _RX_ABCD[0],
            _abcd_series_C(C6),
            _RX_ABCD[1],
            _abcd_series_L(L5),
            _RX_ABCD[2],
            _abcd_shunt_C(C7),
            _RX_ABCD[3],
        )
    else:
        abcd = _cascade(_abcd_series_C(C6), _abcd_series_L(L5), _abcd_shunt_C(C7))
    gamma_in = _abcd_to_loaded_s11(abcd, _rx_gl_cost)
    s11_f0 = np.abs(gamma_in[_f0_idx])
    s11_band = np.max(np.abs(gamma_in[_band_idx]))
    return s11_f0 + 0.3 * s11_band


def ant_cost_vals(C5, L4, C4):
    if USE_PARASITICS:
        abcd = _cascade(
            _ANT_ABCD[0],
            _abcd_shunt_C(C5),
            _ANT_ABCD[1],
            _abcd_series_L(L4),
            _ANT_ABCD[2],
            _abcd_shunt_C(C4),
            _ANT_ABCD[3],
        )
    else:
        abcd = _cascade(_abcd_shunt_C(C5), _abcd_series_L(L4), _abcd_shunt_C(C4))
    gamma_in = _abcd_to_loaded_s11(abcd, _ant_gl_cost)
    s11_f0 = np.abs(gamma_in[_f0_idx])
    s11_band = np.max(np.abs(gamma_in[_band_idx]))
    s21_2f0 = _abcd_s21_mag(abcd, 3)
    return s11_f0 + 0.3 * s11_band + 0.15 * s21_2f0


# Full-resolution indices (for post-optimization reporting)
f_band_lo = 430e6
f_band_hi = 445e6
band_mask = (freq.f >= f_band_lo) & (freq.f <= f_band_hi)
idx_2f0 = np.argmin(np.abs(freq.f - 2*f0_hz))
idx_3f0 = np.argmin(np.abs(freq.f - 3*f0_hz))


# ============================================================
# DISCRETE GRID SEARCH OVER LCSC-AVAILABLE VALUES
# ============================================================
mode_str = "PARASITIC-AWARE (microstrip traces)" if USE_PARASITICS else "IDEAL (lumped only)"
print("=" * 70)
print(f" scikit-rf Matching Network Optimizer — Si4463 @ 437.08 MHz")
print(f" Mode: {mode_str}")
print(f" Constrained to LCSC-available standard component values (0805)")
print("=" * 70)

# --- TX ---
# Topology: 50Ω → L3 → C3(sh) → L2 → C2(sh) → C1 → PA
# Restrict search ranges to reasonable values for 437 MHz matching
print("\n--- TX Matching Optimization (discrete grid search) ---")
print("  Topology: 50Ω → L3 → C3(sh) → L2 → C2(sh) → C1 → PA")
print(f"  PA model: RLOAD={RLOAD:.1f}Ω || CSHUNT={CSHUNT*1e12:.1f}pF || LCHOKE={LCHOKE*1e9:.0f}nH")

tx_L_vals = LCSC_IND_NH[(LCSC_IND_NH >= 6.8) & (LCSC_IND_NH <= 47)]
tx_C_vals = LCSC_CAP_PF[(LCSC_CAP_PF >= 1.0) & (LCSC_CAP_PF <= 47)]

best_tx_cost = np.inf
best_tx = None
n_tx = len(tx_L_vals)**2 * len(tx_C_vals)**3
print(f"  Searching {n_tx:,} combinations...")

for L3 in tx_L_vals:
    for C3 in tx_C_vals:
        for L2 in tx_L_vals:
            for C2 in tx_C_vals:
                for C1 in tx_C_vals:
                    cost = tx_cost_vals(L3, C3, L2, C2, C1)
                    if cost < best_tx_cost:
                        best_tx_cost = cost
                        best_tx = (L3, C3, L2, C2, C1)

tx_opt = dict(L3=best_tx[0]*1e-9, C3=best_tx[1]*1e-12,
              L2=best_tx[2]*1e-9, C2=best_tx[3]*1e-12, C1=best_tx[4]*1e-12)
tx_s11 = loaded_s11_db_at_f0(build_tx_2port(**tx_opt), TX_ZL)
print(f"  Result: L3={best_tx[0]}nH  C3={best_tx[1]}pF  "
      f"L2={best_tx[2]}nH  C2={best_tx[3]}pF  C1={best_tx[4]}pF")
print(f"  S11 = {tx_s11:.1f} dB at {f0_str}")

# --- RX ---
print("\n--- RX Matching Optimization (discrete grid search) ---")
print("  Topology: 50Ω → C6 → L5 → C7(sh) → RX LNA")
print(f"  LNA: {RLNA_SE:.0f}Ω || {CLNA_SE*1e12:.2f}pF (SE)")

rx_C_vals = LCSC_CAP_PF[(LCSC_CAP_PF >= 1.0) & (LCSC_CAP_PF <= 47)]
rx_L_vals = LCSC_IND_NH[(LCSC_IND_NH >= 6.8) & (LCSC_IND_NH <= 68)]

best_rx_cost = np.inf
best_rx = None
n_rx = len(rx_C_vals)**2 * len(rx_L_vals)
print(f"  Searching {n_rx:,} combinations...")

for C6 in rx_C_vals:
    for L5 in rx_L_vals:
        for C7 in rx_C_vals:
            cost = rx_cost_vals(C6, L5, C7)
            if cost < best_rx_cost:
                best_rx_cost = cost
                best_rx = (C6, L5, C7)

rx_opt = dict(C6=best_rx[0]*1e-12, L5=best_rx[1]*1e-9, C7=best_rx[2]*1e-12)
rx_s11 = loaded_s11_db_at_f0(build_rx_2port(**rx_opt), RX_ZL)
print(f"  Result: C6={best_rx[0]}pF  L5={best_rx[1]}nH  C7={best_rx[2]}pF")
print(f"  S11 = {rx_s11:.1f} dB at {f0_str}")

# --- Antenna ---
print("\n--- Antenna Matching Optimization (discrete grid search) ---")
print("  Topology: 50Ω → C5(sh) → L4 → C4(sh) → 50Ω")

ant_C_vals = LCSC_CAP_PF[(LCSC_CAP_PF >= 1.0) & (LCSC_CAP_PF <= 33)]
ant_L_vals = LCSC_IND_NH[(LCSC_IND_NH >= 6.8) & (LCSC_IND_NH <= 56)]

best_ant_cost = np.inf
best_ant = None
n_ant = len(ant_C_vals)**2 * len(ant_L_vals)
print(f"  Searching {n_ant:,} combinations...")

for C5 in ant_C_vals:
    for L4 in ant_L_vals:
        for C4 in ant_C_vals:
            cost = ant_cost_vals(C5, L4, C4)
            if cost < best_ant_cost:
                best_ant_cost = cost
                best_ant = (C5, L4, C4)

ant_opt = dict(C5=best_ant[0]*1e-12, L4=best_ant[1]*1e-9, C4=best_ant[2]*1e-12)
ant_s11 = loaded_s11_db_at_f0(build_ant_2port(**ant_opt), ANT_ZL)
print(f"  Result: C5={best_ant[0]}pF  L4={best_ant[1]}nH  C4={best_ant[2]}pF")
print(f"  S11 = {ant_s11:.1f} dB at {f0_str}")


# ============================================================
# FULL-PATH OPTIMIZATION (ANT + AS179 switch + TX/RX)
# ============================================================
# The VNA measures from SMA through the full cascade. Optimize the
# full path S11 using iterative refinement:
#   1. Fix ANT at per-network optimum, optimize TX and RX for full-path S11
#   2. Fix TX/RX, re-optimize ANT for full-path S11
#   3. Repeat until convergence

print("\n" + "=" * 70)
print(" FULL-PATH OPTIMIZATION (SMA → ANT → AS179 → TX/RX → load)")
print("=" * 70)
print("  Iteratively optimizing all networks for full-path S11...")

# Current best from per-network optimization (in nH/pF for grid search)
fp_ant = [best_ant[0], best_ant[1], best_ant[2]]  # C5, L4, C4 in pF/nH/pF
fp_tx  = [best_tx[0], best_tx[1], best_tx[2], best_tx[3], best_tx[4]]  # L3,C3,L2,C2,C1
fp_rx  = [best_rx[0], best_rx[1], best_rx[2]]  # C6,L5,C7


def full_tx_cost(L3, C3, L2, C2, C1, C5, L4, C4):
    """Full-path TX S11: SMA → ANT → switch(TX on, RX off) → TX → PA."""
    ant_abcd = _build_ant_abcd(C5, L4, C4)
    tx_abcd = _build_tx_abcd(L3, C3, L2, C2, C1)
    full_abcd = _cascade(ant_abcd, _SW_OFF_ABCD, _SW_ON_ABCD, tx_abcd)
    gamma_in = _abcd_to_loaded_s11(full_abcd, _tx_gl_cost)
    s11_f0 = np.abs(gamma_in[_f0_idx])
    s11_band = np.max(np.abs(gamma_in[_band_idx]))
    s21_2f0 = _abcd_s21_mag(full_abcd, 3)
    s21_3f0 = _abcd_s21_mag(full_abcd, 4)
    return s11_f0 + 0.3 * s11_band + 0.2 * s21_2f0 + 0.1 * s21_3f0


def full_rx_cost(C6, L5, C7, C5, L4, C4):
    """Full-path RX S11: SMA → ANT → switch(RX on, TX off) → RX → LNA."""
    ant_abcd = _build_ant_abcd(C5, L4, C4)
    rx_abcd = _build_rx_abcd(C6, L5, C7)
    full_abcd = _cascade(ant_abcd, _SW_OFF_ABCD, _SW_ON_ABCD, rx_abcd)
    gamma_in = _abcd_to_loaded_s11(full_abcd, _rx_gl_cost)
    s11_f0 = np.abs(gamma_in[_f0_idx])
    s11_band = np.max(np.abs(gamma_in[_band_idx]))
    return s11_f0 + 0.3 * s11_band


N_ITER = 3
for iteration in range(N_ITER):
    print(f"\n  --- Iteration {iteration + 1}/{N_ITER} ---")

    # Step 1: Optimize TX given current ANT
    best_ftx_cost = np.inf
    best_ftx = None
    C5_cur, L4_cur, C4_cur = fp_ant
    for L3 in tx_L_vals:
        for C3 in tx_C_vals:
            for L2 in tx_L_vals:
                for C2 in tx_C_vals:
                    for C1 in tx_C_vals:
                        c = full_tx_cost(L3, C3, L2, C2, C1, C5_cur, L4_cur, C4_cur)
                        if c < best_ftx_cost:
                            best_ftx_cost = c
                            best_ftx = (L3, C3, L2, C2, C1)
    fp_tx = list(best_ftx)
    print(f"    TX: L3={fp_tx[0]}nH C3={fp_tx[1]}pF L2={fp_tx[2]}nH "
          f"C2={fp_tx[3]}pF C1={fp_tx[4]}pF")

    # Step 2: Optimize RX given current ANT
    best_frx_cost = np.inf
    best_frx = None
    for C6 in rx_C_vals:
        for L5 in rx_L_vals:
            for C7 in rx_C_vals:
                c = full_rx_cost(C6, L5, C7, C5_cur, L4_cur, C4_cur)
                if c < best_frx_cost:
                    best_frx_cost = c
                    best_frx = (C6, L5, C7)
    fp_rx = list(best_frx)
    print(f"    RX: C6={fp_rx[0]}pF L5={fp_rx[1]}nH C7={fp_rx[2]}pF")

    # Step 3: Re-optimize ANT given current TX and RX
    best_fant_cost = np.inf
    best_fant = None
    L3c, C3c, L2c, C2c, C1c = fp_tx
    C6c, L5c, C7c = fp_rx
    for C5 in ant_C_vals:
        for L4 in ant_L_vals:
            for C4 in ant_C_vals:
                cost_tx = full_tx_cost(L3c, C3c, L2c, C2c, C1c, C5, L4, C4)
                cost_rx = full_rx_cost(C6c, L5c, C7c, C5, L4, C4)
                # Joint cost: worst-case of TX and RX full-path S11
                c = max(cost_tx, cost_rx)
                if c < best_fant_cost:
                    best_fant_cost = c
                    best_fant = (C5, L4, C4)
    fp_ant = list(best_fant)
    print(f"    ANT: C5={fp_ant[0]}pF L4={fp_ant[1]}nH C4={fp_ant[2]}pF")

# Convert to SI units and compute final full-path S11
fp_tx_opt = dict(L3=fp_tx[0]*1e-9, C3=fp_tx[1]*1e-12,
                 L2=fp_tx[2]*1e-9, C2=fp_tx[3]*1e-12, C1=fp_tx[4]*1e-12)
fp_rx_opt = dict(C6=fp_rx[0]*1e-12, L5=fp_rx[1]*1e-9, C7=fp_rx[2]*1e-12)
fp_ant_opt = dict(C5=fp_ant[0]*1e-12, L4=fp_ant[1]*1e-9, C4=fp_ant[2]*1e-12)

fp_tx_ntwk = build_full_tx_path(**fp_tx_opt, **fp_ant_opt)
fp_rx_ntwk = build_full_rx_path(**fp_rx_opt, **fp_ant_opt)
fp_tx_s11 = loaded_s11_db_at_f0(fp_tx_ntwk, TX_ZL)
fp_rx_s11 = loaded_s11_db_at_f0(fp_rx_ntwk, RX_ZL)

print(f"\n  FULL-PATH OPTIMIZED RESULTS:")
print(f"    TX full-path S11 = {fp_tx_s11:.1f} dB at {f0_str}")
print(f"    RX full-path S11 = {fp_rx_s11:.1f} dB at {f0_str}")
print(f"\n    TX: L3={fp_tx[0]}nH  C3={fp_tx[1]}pF  L2={fp_tx[2]}nH  "
      f"C2={fp_tx[3]}pF  C1={fp_tx[4]}pF")
print(f"    RX: C6={fp_rx[0]}pF  L5={fp_rx[1]}nH  C7={fp_rx[2]}pF")
print(f"    ANT: C5={fp_ant[0]}pF  L4={fp_ant[1]}nH  C4={fp_ant[2]}pF")

# Show comparison: per-network vs full-path optimized
print(f"\n  COMPARISON: Per-network S11 vs Full-path S11")
print(f"  (Per-network S11 ignores switch + cascade effects)")
pn_tx_fp = loaded_s11_db_at_f0(
    build_full_tx_path(**tx_opt, **ant_opt), TX_ZL)
pn_rx_fp = loaded_s11_db_at_f0(
    build_full_rx_path(**rx_opt, **ant_opt), RX_ZL)
print(f"    Per-network values through full path:  TX={pn_tx_fp:.1f} dB  RX={pn_rx_fp:.1f} dB")
print(f"    Full-path optimized through full path:  TX={fp_tx_s11:.1f} dB  RX={fp_rx_s11:.1f} dB")


# ============================================================
# BUILD ALL 2-PORT NETWORKS FOR COMPARISON
# ============================================================
def build_set(name, vals):
    if name == 'TX':
        return build_tx_2port(vals['L3'], vals['C3'], vals['L2'], vals['C2'], vals['C1'])
    elif name == 'RX':
        return build_rx_2port(vals['C6'], vals['L5'], vals['C7'])
    else:
        return build_ant_2port(vals['C5'], vals['L4'], vals['C4'])

all_vals = {
    'TX':  tx_opt,
    'RX':  rx_opt,
    'ANT': ant_opt,
}

load_z = {'TX': TX_ZL, 'RX': RX_ZL, 'ANT': ANT_ZL}

networks_2p = {}
for net_name in ['TX', 'RX', 'ANT']:
    networks_2p[net_name] = build_set(net_name, all_vals[net_name])


# ============================================================
# DIAGNOSTIC: Show what the OLD ideal values look like with parasitics
# ============================================================
if USE_PARASITICS:
    print("\n" + "=" * 70)
    print(" DIAGNOSTIC: Old ideal values WITH trace parasitics")
    print("=" * 70)
    print("  (Shows what the old ideal component values actually produce on this PCB)")

    # Old ideal values (from the ideal optimizer)
    old_tx = dict(L3=22e-9, C3=10e-12, L2=27e-9, C2=5.6e-12, C1=33e-12)
    old_rx = dict(C6=47e-12, L5=39e-9, C7=1.0e-12)
    old_ant = dict(C5=12e-12, L4=16e-9, C4=12e-12)
    old_vals = {'TX': old_tx, 'RX': old_rx, 'ANT': old_ant}

    for net_name in ['TX', 'RX', 'ANT']:
        ntwk = build_set(net_name, old_vals[net_name])
        zl = load_z[net_name]
        gamma_in = loaded_s11(ntwk, zl)
        s11_db = 20 * np.log10(np.maximum(np.abs(gamma_in), 1e-15))

        # Find resonant frequency (minimum S11 in 200-500 MHz range)
        mask = (freq.f >= 200e6) & (freq.f <= 500e6)
        idx_min = np.argmin(s11_db[mask])
        f_res = freq.f[mask][idx_min]
        s11_at_res = s11_db[mask][idx_min]
        s11_at_f0 = loaded_s11_db_at_f0(ntwk, zl)

        print(f"\n  {net_name} (old ideal values + PCB parasitics):")
        vals = old_vals[net_name]
        for comp, v in vals.items():
            if v >= 1e-9:
                print(f"    {comp}: {v*1e9:.1f} nH")
            else:
                print(f"    {comp}: {v*1e12:.1f} pF")
        print(f"    Resonant freq: {f_res/1e6:.1f} MHz (S11 = {s11_at_res:.1f} dB)")
        print(f"    S11 at {f0_str}: {s11_at_f0:.1f} dB")
        if abs(f_res - f0_hz) > 20e6:
            print(f"    >>> SHIFTED {(f_res - f0_hz)/1e6:+.1f} MHz from target!")


# ============================================================
# VNA COMPARISON MODE
# ============================================================
if args.vna:
    print("\n" + "=" * 70)
    print(" VNA COMPARISON: Model vs Measured (current installed values)")
    print("=" * 70)

    # VNA measured data at 437.2 MHz (from NanoVNA screenshots)
    vna_f = 437.2e6
    vna_tx_s11_db = -3.78
    vna_tx_z = 10.8 + 4.3j
    vna_rx_s11_db = -8.15
    vna_rx_z = 67.1 - 46j
    vna_nomode_s11_db = -11.11
    vna_nomode_z = 35.9 - 20j

    # Model prediction with per-network optimized values through full path
    pn_tx_fp_s11 = loaded_s11_db_at_f0(
        build_full_tx_path(**tx_opt, **ant_opt), TX_ZL)
    pn_rx_fp_s11 = loaded_s11_db_at_f0(
        build_full_rx_path(**rx_opt, **ant_opt), RX_ZL)

    # Also compute model's predicted impedance at f0
    def full_path_z_at_f0(ntwk_2port, Z_load_arr):
        """Compute impedance at port 1 of a 2-port terminated by load."""
        gamma_in = loaded_s11(ntwk_2port, Z_load_arr)
        idx = np.argmin(np.abs(freq.f - f0_hz))
        g = gamma_in[idx]
        return Z0 * (1 + g) / (1 - g)

    model_tx_z = full_path_z_at_f0(
        build_full_tx_path(**tx_opt, **ant_opt), TX_ZL)
    model_rx_z = full_path_z_at_f0(
        build_full_rx_path(**rx_opt, **ant_opt), RX_ZL)

    print(f"\n  {'Mode':<12} {'VNA S11':>10} {'Model S11':>10} {'Delta':>8}")
    print(f"  {'-'*42}")
    print(f"  {'TX':<12} {vna_tx_s11_db:>8.1f}dB {pn_tx_fp_s11:>8.1f}dB "
          f"{pn_tx_fp_s11 - vna_tx_s11_db:>+6.1f}dB")
    print(f"  {'RX':<12} {vna_rx_s11_db:>8.1f}dB {pn_rx_fp_s11:>8.1f}dB "
          f"{pn_rx_fp_s11 - vna_rx_s11_db:>+6.1f}dB")

    print(f"\n  {'Mode':<12} {'VNA Z':>20} {'Model Z':>20}")
    print(f"  {'-'*54}")
    print(f"  {'TX':<12} {vna_tx_z.real:>7.1f}{vna_tx_z.imag:>+7.1f}j Ω"
          f"  {model_tx_z.real:>7.1f}{model_tx_z.imag:>+7.1f}j Ω")
    print(f"  {'RX':<12} {vna_rx_z.real:>7.1f}{vna_rx_z.imag:>+7.1f}j Ω"
          f"  {model_rx_z.real:>7.1f}{model_rx_z.imag:>+7.1f}j Ω")

    # Full-path optimized prediction
    print(f"\n  After full-path optimization:")
    print(f"    TX full-path S11 = {fp_tx_s11:.1f} dB")
    print(f"    RX full-path S11 = {fp_rx_s11:.1f} dB")

    model_fptx_z = full_path_z_at_f0(fp_tx_ntwk, TX_ZL)
    model_fprx_z = full_path_z_at_f0(fp_rx_ntwk, RX_ZL)
    print(f"    TX predicted Z = {model_fptx_z.real:.1f}{model_fptx_z.imag:+.1f}j Ω")
    print(f"    RX predicted Z = {model_fprx_z.real:.1f}{model_fprx_z.imag:+.1f}j Ω")


# ============================================================
# RESULTS TABLE
# ============================================================
print("\n" + "=" * 70)
print(f" OPTIMIZED COMPONENT VALUES ({mode_str})")
print("=" * 70)

# Component values
for net_name in ['TX', 'RX', 'ANT']:
    print(f"\n  {net_name} Network:")
    vals = all_vals[net_name]
    comps = list(vals.keys())
    print(f"    {'Comp':<6} {'Value':>12}")
    print(f"    {'-'*20}")
    for comp in comps:
        v = vals[comp]
        if v >= 1e-9:
            print(f"    {comp:<6} {v*1e9:>10.1f} nH")
        else:
            print(f"    {comp:<6} {v*1e12:>10.1f} pF")

# S11 at f0 — per-network (independent) optimization
print(f"\n  Per-network S11 at {f0_str} (independent optimization):")
print(f"    {'Net':<6} {'S11':>12}")
print(f"    {'-'*20}")
for net_name in ['TX', 'RX', 'ANT']:
    zl = load_z[net_name]
    s11_db = loaded_s11_db_at_f0(networks_2p[net_name], zl)
    print(f"    {net_name:<6} {s11_db:>10.1f} dB")

# Full-path optimized values
fp_all_vals = {
    'TX':  fp_tx_opt,
    'RX':  fp_rx_opt,
    'ANT': fp_ant_opt,
}
print(f"\n  FULL-PATH OPTIMIZED (recommended for installation):")
for net_name in ['TX', 'RX', 'ANT']:
    print(f"\n  {net_name} Network:")
    vals = fp_all_vals[net_name]
    comps = list(vals.keys())
    print(f"    {'Comp':<6} {'Value':>12}")
    print(f"    {'-'*20}")
    for comp in comps:
        v = vals[comp]
        if v >= 1e-9:
            print(f"    {comp:<6} {v*1e9:>10.1f} nH")
        else:
            print(f"    {comp:<6} {v*1e12:>10.1f} pF")

print(f"\n  Full-path S11 at {f0_str} (SMA → ANT → switch → TX/RX → load):")
print(f"    {'Mode':<6} {'S11':>12}")
print(f"    {'-'*20}")
print(f"    {'TX':<6} {fp_tx_s11:>10.1f} dB")
print(f"    {'RX':<6} {fp_rx_s11:>10.1f} dB")

# Find resonant frequencies
print(f"\n  Resonant frequencies (S11 minimum in 300-600 MHz):")
print(f"    {'Net':<6} {'f_res':>10} {'S11':>10} {'Δf':>10}")
print(f"    {'-'*38}")
for net_name in ['TX', 'RX', 'ANT']:
    zl = load_z[net_name]
    gamma_in = loaded_s11(networks_2p[net_name], zl)
    s11_db = 20 * np.log10(np.maximum(np.abs(gamma_in), 1e-15))
    mask = (freq.f >= 300e6) & (freq.f <= 600e6)
    idx_min = np.argmin(s11_db[mask])
    f_res = freq.f[mask][idx_min]
    s11_min = s11_db[mask][idx_min]
    delta = f_res - f0_hz
    print(f"    {net_name:<6} {f_res/1e6:>8.1f}MHz {s11_min:>8.1f}dB {delta/1e6:>+8.1f}MHz")

# Harmonic rejection (S21 of the 2-port filter at harmonics)
print(f"\n  TX Filter S21 at Harmonics (2-port, lower = more rejection):")
print(f"    {'Freq':<14} {'S21':>12}")
print(f"    {'-'*28}")
for label, fstr in [('f0 (437MHz)', f0_str), ('2f0 (874MHz)', '874MHz'), ('3f0 (1311MHz)', '1311MHz')]:
    ntw = networks_2p['TX']
    mag = np.abs(ntw[fstr].s[0, 1, 0])
    val_db = 20 * np.log10(max(mag, 1e-15))
    print(f"    {label:<14} {val_db:>10.1f} dB")


# ============================================================
# STEP-BY-STEP IMPEDANCE TRANSFORMATION AT f0
# ============================================================
w0 = 2 * np.pi * f0_hz
idx_f0 = np.argmin(np.abs(freq.f - f0_hz))

def _z_to_gamma(z):
    """Impedance to reflection coefficient (Z0-normalized)."""
    return (z - Z0) / (z + Z0)

def _add_series_z(z, z_elem):
    return z + z_elem

def _add_shunt_y(z, y_elem):
    return 1.0 / (1.0/z + y_elem)

def _add_trace(z, w_mm, length_mm):
    """Transform impedance through a microstrip transmission line segment."""
    w_m = w_mm * 1e-3
    d_m = length_mm * 1e-3
    z0_line, ep_eff = microstrip_params(w_m, PCB_H, PCB_ER, PCB_T)
    beta = 2 * np.pi * f0_hz * np.sqrt(ep_eff) / 299792458.0
    theta = beta * d_m
    return z0_line * (z * np.cos(theta) + 1j * z0_line * np.sin(theta)) / \
           (z0_line * np.cos(theta) + 1j * z * np.sin(theta))

def _arc_series(z_start, z_end, n_pts=80):
    """Arc along constant-resistance circle (series element)."""
    R = z_start.real
    X = np.linspace(z_start.imag, z_end.imag, n_pts)
    Z = R + 1j * X
    return (Z - Z0) / (Z + Z0)

def _arc_shunt(z_start, z_end, n_pts=80):
    """Arc along constant-conductance circle (shunt element)."""
    y_start, y_end = 1.0 / z_start, 1.0 / z_end
    G = y_start.real
    B = np.linspace(y_start.imag, y_end.imag, n_pts)
    Y = G + 1j * B
    Z = 1.0 / Y
    return (Z - Z0) / (Z + Z0)

def _arc_trace(z_start, z_end, w_mm, length_mm, n_pts=80):
    """Arc along transmission line transformation (circular arc on Smith chart)."""
    w_m = w_mm * 1e-3
    d_m = length_mm * 1e-3
    z0_line, ep_eff = microstrip_params(w_m, PCB_H, PCB_ER, PCB_T)
    beta = 2 * np.pi * f0_hz * np.sqrt(ep_eff) / 299792458.0
    theta_total = beta * d_m
    thetas = np.linspace(0, theta_total, n_pts)
    Z = z0_line * (z_start * np.cos(thetas) + 1j * z0_line * np.sin(thetas)) / \
        (z0_line * np.cos(thetas) + 1j * z_start * np.sin(thetas))
    return (Z - Z0) / (Z + Z0)


# Each step: (label, z_after, 'series'|'shunt'|'start'|'trace', trace_info)
def compute_tx_steps(vals):
    """TX: PA load ← C1 ← C2(sh) ← L2 ← C3(sh) ← L3 ← 50Ω source."""
    L3, C3, L2, C2, C1 = vals['L3'], vals['C3'], vals['L2'], vals['C2'], vals['C1']
    z_pa = TX_ZL[idx_f0]
    steps = [('PA load', z_pa, 'start', None)]
    z = z_pa
    if USE_PARASITICS:
        w, d = TX_TRACES_MM[4]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_series_z(z, 1.0/(1j*w0*C1));  steps.append(('+ C1 (series)', z, 'series', None))
    if USE_PARASITICS:
        w, d = TX_TRACES_MM[3]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_shunt_y(z, 1j*w0*C2);         steps.append(('+ C2 (shunt)',  z, 'shunt', None))
    if USE_PARASITICS:
        w, d = TX_TRACES_MM[2]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_series_z(z, 1j*w0*L2);        steps.append(('+ L2 (series)', z, 'series', None))
    if USE_PARASITICS:
        w, d = TX_TRACES_MM[1]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_shunt_y(z, 1j*w0*C3);         steps.append(('+ C3 (shunt)',  z, 'shunt', None))
    z = _add_series_z(z, 1j*w0*L3);        steps.append(('+ L3 (series)', z, 'series', None))
    if USE_PARASITICS:
        w, d = TX_TRACES_MM[0]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    return steps

def compute_rx_steps(vals):
    """RX: LNA load ← C7(sh) ← L5 ← C6 ← 50Ω source."""
    C6, L5, C7 = vals['C6'], vals['L5'], vals['C7']
    z_lna = RX_ZL[idx_f0]
    steps = [('LNA load', z_lna, 'start', None)]
    z = z_lna
    if USE_PARASITICS:
        w, d = RX_TRACES_MM[3]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_shunt_y(z, 1j*w0*C7);         steps.append(('+ C7 (shunt)',  z, 'shunt', None))
    if USE_PARASITICS:
        w, d = RX_TRACES_MM[2]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_series_z(z, 1j*w0*L5);        steps.append(('+ L5 (series)', z, 'series', None))
    if USE_PARASITICS:
        w, d = RX_TRACES_MM[1]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_series_z(z, 1.0/(1j*w0*C6));  steps.append(('+ C6 (series)', z, 'series', None))
    if USE_PARASITICS:
        w, d = RX_TRACES_MM[0]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    return steps

def compute_ant_steps(vals):
    """ANT: 50Ω load ← C4(sh) ← L4 ← C5(sh) ← 50Ω source."""
    C5, L4, C4 = vals['C5'], vals['L4'], vals['C4']
    z = complex(Z0)
    steps = [('50Ω load', z, 'start', None)]
    if USE_PARASITICS:
        w, d = ANT_TRACES_MM[3]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_shunt_y(z, 1j*w0*C4);         steps.append(('+ C4 (shunt)',  z, 'shunt', None))
    if USE_PARASITICS:
        w, d = ANT_TRACES_MM[2]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_series_z(z, 1j*w0*L4);        steps.append(('+ L4 (series)', z, 'series', None))
    if USE_PARASITICS:
        w, d = ANT_TRACES_MM[1]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    z = _add_shunt_y(z, 1j*w0*C5);         steps.append(('+ C5 (shunt)',  z, 'shunt', None))
    if USE_PARASITICS:
        w, d = ANT_TRACES_MM[0]
        z = _add_trace(z, w, d); steps.append((f'trace {d:.0f}mm', z, 'trace', (w, d)))
    return steps

smith_steps = {
    'TX':  compute_tx_steps(all_vals['TX']),
    'RX':  compute_rx_steps(all_vals['RX']),
    'ANT': compute_ant_steps(all_vals['ANT']),
}

# Print step-by-step transformation
print(f"\n  Impedance transformation at f₀ = {f0_hz/1e6:.2f} MHz:")
for net_name in ['TX', 'RX', 'ANT']:
    print(f"\n  {net_name}:")
    for label, z, stype, _ in smith_steps[net_name]:
        g = _z_to_gamma(z)
        print(f"    {label:<20}  Z = {z.real:7.1f}{z.imag:+7.1f}j Ω   |Γ| = {abs(g):.3f}")


# ============================================================
# PLOTS
# ============================================================
print("\nGenerating plots...")

fig, axes = plt.subplots(3, 3, figsize=(20, 18))
fig.suptitle(f'Si4463 Matching Networks — scikit-rf Optimized (LCSC)\n'
             f'f₀ = {f0_hz/1e6:.2f} MHz, Z₀ = {Z0:.0f} Ω — {mode_str}',
             fontsize=14, fontweight='bold')

color = '#E63946'
titles_map = {'TX': 'TX: PA → 50 Ω', 'RX': 'RX: 50 Ω → LNA', 'ANT': 'Antenna: 50 Ω → Switch'}

for row, net_name in enumerate(['TX', 'RX', 'ANT']):
    ntwk = networks_2p[net_name]
    zl = load_z[net_name]

    # Column 0: Loaded S11 (dB) vs frequency
    ax = axes[row, 0]
    gamma_in = loaded_s11(ntwk, zl)
    s11_db = 20 * np.log10(np.maximum(np.abs(gamma_in), 1e-15))
    ax.plot(freq.f / 1e6, s11_db, color=color, linewidth=1.5)
    ax.axvline(f0_hz / 1e6, color='gray', linestyle='--', alpha=0.5, label='f₀')
    ax.axhline(-10, color='orange', linestyle=':', alpha=0.4, label='-10 dB')
    ax.set_xlabel('Frequency (MHz)')
    ax.set_ylabel('S11 (dB)')
    ax.set_title(f'{titles_map[net_name]} — Loaded S11')
    ax.set_ylim(-50, 5)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Column 1: S21 (dB) of matching filter (2-port, 50Ω ref)
    ax = axes[row, 1]
    s21_db = ntwk.s_db[:, 1, 0]
    ax.plot(freq.f / 1e6, s21_db, color=color, linewidth=1.5)
    ax.axvline(f0_hz / 1e6, color='gray', linestyle='--', alpha=0.5, label='f₀')
    if net_name == 'TX':
        ax.axvline(2 * f0_hz / 1e6, color='gray', linestyle=':', alpha=0.3, label='2f₀')
        ax.axvline(3 * f0_hz / 1e6, color='gray', linestyle=':', alpha=0.3, label='3f₀')
    ax.set_xlabel('Frequency (MHz)')
    ax.set_ylabel('S21 (dB)')
    ax.set_title(f'{titles_map[net_name]} — Filter S21')
    ax.set_ylim(-60, 5)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Column 2: Smith chart with step-by-step impedance transformation
    ax = axes[row, 2]
    plt.sca(ax)
    ntwk.plot_s_smith(m=0, n=0, color=color, linewidth=1.5, alpha=0.25,
                      draw_labels=True, label='S11 vs freq')

    # Overlay step-by-step arcs
    steps = smith_steps[net_name]
    arc_colors = {'series': '#457B9D', 'shunt': '#E63946', 'trace': '#2D6A4F'}

    for i in range(1, len(steps)):
        label_prev, z_prev, _, _ = steps[i-1]
        label_curr, z_curr, stype, trace_info = steps[i]
        if stype == 'series':
            arc_gamma = _arc_series(z_prev, z_curr)
        elif stype == 'shunt':
            arc_gamma = _arc_shunt(z_prev, z_curr)
        elif stype == 'trace':
            w_mm, d_mm = trace_info
            arc_gamma = _arc_trace(z_prev, z_curr, w_mm, d_mm)
        else:
            continue
        c = arc_colors.get(stype, '#333333')
        ax.plot(arc_gamma.real, arc_gamma.imag, '-', color=c,
                linewidth=2.5, alpha=0.8, zorder=5)
        # Arrowhead at midpoint
        mid = len(arc_gamma) // 2
        ax.annotate('', xy=(arc_gamma[mid+1].real, arc_gamma[mid+1].imag),
                    xytext=(arc_gamma[mid-1].real, arc_gamma[mid-1].imag),
                    arrowprops=dict(arrowstyle='->', color=c, lw=2), zorder=6)

    # Plot numbered markers at each step
    for i, (label, z, stype, _) in enumerate(steps):
        g = _z_to_gamma(z)
        mc = '#2D6A4F' if i == 0 or i == len(steps)-1 else '#333333'
        ax.plot(g.real, g.imag, 'o', color=mc, markersize=7,
                zorder=10, markeredgecolor='white', markeredgewidth=1)
        ax.annotate(f'{i}: {label}',
                    xy=(g.real, g.imag), fontsize=5.5, fontweight='bold',
                    xytext=(6, 6), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.85,
                              ec='gray', lw=0.5),
                    zorder=11)

    # Legend for arc types
    from matplotlib.lines import Line2D
    handles = [Line2D([0], [0], color='#457B9D', lw=2.5, label='series (const R)'),
               Line2D([0], [0], color='#E63946', lw=2.5, label='shunt (const G)')]
    if USE_PARASITICS:
        handles.append(Line2D([0], [0], color='#2D6A4F', lw=2.5, label='microstrip trace'))
    ax.legend(handles=handles, fontsize=6, loc='lower right')
    ax.set_title(f'{titles_map[net_name]} — Smith Chart')

plt.tight_layout(rect=[0, 0, 1, 0.96])
out_path = 'src/matching_skrf.png'
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"Saved: {out_path}")
plt.show()

print("\nDone.")
