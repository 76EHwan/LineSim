#!/usr/bin/env python3
"""Fourier Transform (FFT) Spectral Analysis for LineSim Simulation Runs.

Analyzes dominant oscillation frequencies in:
- Yaw Rate (wz_dps)
- Lateral Tracking Error (lat_mm)
- Motor Differential Current / Steering Effort (iq_diff)
Across straights, curves, and full laps.
"""

import argparse
import csv
import math
import os
import sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from course import load, pieces

def compute_fft(signal, fs, window=True):
    """Computes single-sided FFT amplitude spectrum and PSD."""
    N = len(signal)
    if N < 16:
        return np.array([]), np.array([]), np.array([])
    
    # Remove DC component (mean)
    sig_detrend = signal - np.mean(signal)
    
    if window:
        win = np.hanning(N)
        win_sig = sig_detrend * win
        # Normalization factor for window
        scale = np.sqrt(1.0 / (np.sum(win**2) / N))
        win_sig *= scale
    else:
        win_sig = sig_detrend

    # Real FFT
    rfft_out = np.fft.rfft(win_sig)
    freqs = np.fft.rfftfreq(N, d=1.0 / fs)
    
    # Amplitude spectrum [units of signal]
    amps = 2.0 * np.abs(rfft_out) / N
    amps[0] /= 2.0  # DC
    
    # Power spectral density
    psd = (np.abs(rfft_out) ** 2) / (fs * N)
    if len(psd) > 1:
        psd[1:-1] *= 2.0
        
    return freqs, amps, psd

def find_peaks(freqs, amps, top_n=5, min_freq=0.2, max_freq=150.0):
    """Identifies prominent spectral peaks."""
    mask = (freqs >= min_freq) & (freqs <= max_freq)
    f_sub = freqs[mask]
    a_sub = amps[mask]
    if len(a_sub) < 3:
        return []

    # Local maxima
    peaks = []
    for i in range(1, len(a_sub) - 1):
        if a_sub[i] > a_sub[i-1] and a_sub[i] > a_sub[i+1]:
            peaks.append((f_sub[i], a_sub[i]))

    # Sort by descending amplitude
    peaks.sort(key=lambda x: x[1], reverse=True)
    return peaks[:top_n]

def calculate_band_energies(freqs, psd):
    """Categorizes spectral power into characteristic frequency bands."""
    bands = {
        "Track Following (0.2 - 2.0 Hz)": (0.2, 2.0),
        "Hunting / Weave (2.0 - 8.0 Hz)": (2.0, 8.0),
        "Steering Loop Dynamics (8.0 - 25.0 Hz)": (8.0, 25.0),
        "Sensor Noise / Vibration (25.0 - 100.0 Hz)": (25.0, 100.0),
        "High Frequency (> 100.0 Hz)": (100.0, freqs[-1] if len(freqs) > 0 else 500.0)
    }
    total_power = np.sum(psd[freqs >= 0.2]) + 1e-12
    res = {}
    for name, (f_lo, f_hi) in bands.items():
        m = (freqs >= f_lo) & (freqs < f_hi)
        p = np.sum(psd[m])
        res[name] = (p, 100.0 * p / total_power)
    return res

def analyze_csv(csv_path, track_path=None):
    if not os.path.exists(csv_path):
        print(f"Error: CSV file not found: {csv_path}")
        return

    rows = list(csv.DictReader(open(csv_path, "r", encoding="utf-8", errors="ignore")))
    if len(rows) < 100:
        print(f"Error: Not enough data points in {csv_path}")
        return

    t = np.array([float(r["t"]) for r in rows])
    wz = np.array([float(r["wz_dps"]) for r in rows])
    lat = np.array([float(r["lat_mm"]) for r in rows])
    v = np.array([float(r["v"]) for r in rows])
    
    # Motor differential current if available
    if "iq_l" in rows[0] and "iq_r" in rows[0]:
        iq_diff = np.array([float(r["iq_l"]) - float(r["iq_r"]) for r in rows])
    else:
        iq_diff = np.zeros_like(t)

    # Estimate sampling frequency
    dt = np.median(np.diff(t))
    fs = 1.0 / dt if dt > 0 else 1000.0

    print("=" * 80)
    print(f"  FOURIER SPECTRAL ANALYSIS: {os.path.basename(csv_path)}")
    print(f"  Duration: {t[-1] - t[0]:.2f} s | Samples: {len(t):,d} | Fs: {fs:.1f} Hz (Nyquist: {fs/2:.1f} Hz)")
    print(f"  Mean Speed: {np.mean(v):.2f} m/s | Max Speed: {np.max(v):.2f} m/s")
    print("=" * 80)

    # 1. Full Lap Analysis
    signals = [
        ("Yaw Rate (wz)", wz, "deg/s"),
        ("Lateral Error (lat)", lat, "mm"),
        ("Steering Current Diff", iq_diff, "A")
    ]

    for name, sig, unit in signals:
        if np.all(sig == 0):
            continue
        freqs, amps, psd = compute_fft(sig, fs)
        peaks = find_peaks(freqs, amps, top_n=5)
        bands = calculate_band_energies(freqs, psd)

        print(f"\n--- Signal: {name} (Overall Full-Lap FFT) ---")
        print(f"  RMS: {np.sqrt(np.mean((sig - np.mean(sig))**2)):.2f} {unit} | P-P: {np.ptp(sig):.2f} {unit}")
        print("  Top Dominant Frequencies:")
        for rank, (pk_f, pk_a) in enumerate(peaks, 1):
            bar = "#" * int(min(35, pk_a / (peaks[0][1] + 1e-9) * 35))
            print(f"    #{rank}: {pk_f:6.2f} Hz  |  Amplitude: {pk_a:7.3f} {unit}  [{bar}]")

        print("  Energy Distribution by Frequency Band:")
        for b_name, (bp, bp_pct) in bands.items():
            print(f"    - {b_name:38s}: {bp_pct:5.1f}%")

    # 2. Straight-Section Breakdown if track geometry is available
    if track_path and os.path.exists(track_path):
        j = load(track_path)
        o = j.get("origin", [0, 0, 0])
        ps, _, _ = pieces(j["segments"], o[0], o[1], math.radians(o[2]))
        s = np.array([float(r["s"]) for r in rows])

        # Extract straight pieces
        str_indices = []
        for p in ps:
            if p[0] == "S" and p[2] >= 0.30:  # Straight >= 0.3m
                s0 = p[1] + 0.10  # skip 10cm entry transient
                s1 = p[1] + p[2]
                idx = np.where((s >= s0) & (s <= s1) & (v > 1.0))[0]
                if len(idx) > 20:
                    str_indices.extend(idx)

        if len(str_indices) >= 50:
            wz_str = wz[str_indices]
            lat_str = lat[str_indices]
            f_wz, a_wz, p_wz = compute_fft(wz_str, fs)
            f_lat, a_lat, p_lat = compute_fft(lat_str, fs)

            pk_wz = find_peaks(f_wz, a_wz, top_n=3)
            pk_lat = find_peaks(f_lat, a_lat, top_n=3)

            print("\n" + "=" * 80)
            print("  STRAIGHT SECTIONS ONLY (Pure Weave / Hunting Oscillation Spectrum):")
            print(f"  Total Straight Samples: {len(str_indices)} ({len(str_indices)/fs:.2f} s)")
            print(f"  Straight Yaw Rate RMS: {np.std(wz_str):.2f} deg/s | Lat Error RMS: {np.std(lat_str):.2f} mm")
            print("  Dominant Yaw Weave Frequencies:")
            for rank, (pk_f, pk_a) in enumerate(pk_wz, 1):
                print(f"    #{rank}: {pk_f:6.2f} Hz  |  Amplitude: {pk_a:7.3f} deg/s")
            print("  Dominant Lateral Weave Frequencies:")
            for rank, (pk_f, pk_a) in enumerate(pk_lat, 1):
                print(f"    #{rank}: {pk_f:6.2f} Hz  |  Amplitude: {pk_a:7.3f} mm")
            print("=" * 80)

def main():
    parser = argparse.ArgumentParser(description="FFT Spectral Frequency Analyzer for LineSim")
    parser.add_argument("csv", nargs="?", default="runs/20261104_073139/drive4.csv", help="drive1.csv or drive4.csv path")
    parser.add_argument("--track", default=os.path.join(ROOT, "tracks", "zetin_finals_championship.json"), help="Track JSON path")
    args = parser.parse_args()

    analyze_csv(args.csv, args.track)

if __name__ == "__main__":
    main()
