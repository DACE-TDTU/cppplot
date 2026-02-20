#!/usr/bin/env python3
"""
Comprehensive Plotting Script for Robust Control Case Studies
Generates publication-quality figures from CSV data files.

Author: CppPlot Library
Usage: python plot_all_results.py
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path

# Configure matplotlib for publication quality
plt.rcParams.update({
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'figure.figsize': (10, 6),
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'lines.linewidth': 1.5,
    'axes.grid': True,
    'grid.alpha': 0.3
})

def plot_hinf_results():
    """Plot H-infinity motor control results"""
    csv_file = 'hinf_motor_data.csv'
    if not os.path.exists(csv_file):
        print(f"Warning: {csv_file} not found, skipping.")
        return
    
    df = pd.read_csv(csv_file)
    
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('H-infinity Motor Speed Control - Differential Drive Robot', 
                 fontsize=14, fontweight='bold')
    
    # Plot 1: Angular velocity
    ax1 = axes[0]
    ax1.plot(df['time'], df['omega'], 'b-', linewidth=2, label=r'$\omega$ (actual)')
    ax1.plot(df['time'], df['reference'], 'r--', linewidth=1.5, label=r'$\omega_{ref}$')
    ax1.fill_between(df['time'], df['omega'], df['reference'], alpha=0.1, color='blue')
    ax1.set_ylabel(r'$\omega$ (rad/s)')
    ax1.legend(loc='lower right')
    ax1.set_title('Angular Velocity Response')
    ax1.axhline(y=df['reference'].iloc[-1], color='k', linestyle=':', alpha=0.5)
    
    # Plot 2: Control signal
    ax2 = axes[1]
    ax2.plot(df['time'], df['control'], 'g-', linewidth=2)
    ax2.set_ylabel('u (V)')
    ax2.axhline(y=12, color='r', linestyle='--', alpha=0.5, label='Saturation (±12V)')
    ax2.axhline(y=-12, color='r', linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right')
    ax2.set_title('Control Signal')
    
    # Plot 3: Error
    ax3 = axes[2]
    ax3.plot(df['time'], df['error'], 'm-', linewidth=2)
    ax3.set_ylabel('e (rad/s)')
    ax3.set_xlabel('Time (s)')
    ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    # Add 2% settling band
    ss_ref = df['reference'].iloc[-1]
    ax3.axhline(y=0.02*ss_ref, color='orange', linestyle='--', alpha=0.5, label='±2% band')
    ax3.axhline(y=-0.02*ss_ref, color='orange', linestyle='--', alpha=0.5)
    ax3.legend(loc='upper right')
    ax3.set_title('Tracking Error')
    
    plt.tight_layout()
    plt.savefig('hinf_motor_response.png', dpi=300, bbox_inches='tight')
    plt.savefig('hinf_motor_response.pdf', bbox_inches='tight')
    print("Saved: hinf_motor_response.png, hinf_motor_response.pdf")
    
    # Print metrics
    print("\nH-infinity Control Performance Metrics:")
    print(f"  Final velocity:     {df['omega'].iloc[-1]:.4f} rad/s")
    print(f"  Reference:          {df['reference'].iloc[-1]:.4f} rad/s")
    print(f"  Steady-state error: {abs(df['error'].iloc[-1]):.4f} rad/s ({abs(df['error'].iloc[-1])/ss_ref*100:.2f}%)")
    
    omega_max = df['omega'].max()
    overshoot = max(0, (omega_max - ss_ref) / ss_ref * 100)
    print(f"  Overshoot:          {overshoot:.2f}%")
    
    # Find rise time (10% to 90%)
    t_10 = df[df['omega'] >= 0.1*ss_ref]['time'].iloc[0] if any(df['omega'] >= 0.1*ss_ref) else None
    t_90 = df[df['omega'] >= 0.9*ss_ref]['time'].iloc[0] if any(df['omega'] >= 0.9*ss_ref) else None
    if t_10 and t_90:
        print(f"  Rise time (10-90%): {(t_90-t_10)*1000:.1f} ms")
    
    plt.close()

def plot_case1_step_response():
    """Plot Case Study 1: Nominal PI Control step response"""
    csv_file = 'case1_step_response.csv'
    if not os.path.exists(csv_file):
        print(f"Warning: {csv_file} not found, skipping.")
        return
    
    df = pd.read_csv(csv_file)
    
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(df['time'], df['response'], 'b-', linewidth=2, label='PI Control Response')
    ax.axhline(y=10, color='r', linestyle='--', label='Reference (10 rad/s)')
    ax.axhline(y=10*0.98, color='orange', linestyle=':', alpha=0.7, label='±2% band')
    ax.axhline(y=10*1.02, color='orange', linestyle=':', alpha=0.7)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(r'$\omega$ (rad/s)')
    ax.set_title('Case Study 1: Nominal PI Control - Step Response')
    ax.legend(loc='lower right')
    
    plt.tight_layout()
    plt.savefig('case1_step_response.png', dpi=300, bbox_inches='tight')
    print("Saved: case1_step_response.png")
    plt.close()

def create_bode_plot():
    """Generate Bode plot for the motor transfer function"""
    # Motor parameters
    K = 6.667  # DC gain
    tau = 3.333  # Time constant (s)
    
    # Frequency range
    omega = np.logspace(-2, 3, 500)
    
    # Transfer function G(s) = K / (tau*s + 1)
    s = 1j * omega
    G = K / (tau * s + 1)
    
    # PI Controller: Kp = 4.26, Ki = 26.3 (example)
    Kp, Ki = 4.26, 26.3
    C = Kp + Ki / s
    
    # Loop transfer function
    L = G * C
    
    # Sensitivity and complementary sensitivity
    S = 1 / (1 + L)
    T = L / (1 + L)
    
    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # Bode plot - Magnitude
    ax1 = axes[0, 0]
    ax1.semilogx(omega, 20*np.log10(np.abs(G)), 'b-', label='Plant G(s)')
    ax1.semilogx(omega, 20*np.log10(np.abs(C)), 'g-', label='Controller C(s)')
    ax1.semilogx(omega, 20*np.log10(np.abs(L)), 'r-', label='Loop L(s)=G·C')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title('Bode Diagram - Magnitude')
    ax1.legend()
    ax1.set_xlim([0.01, 1000])
    
    # Bode plot - Phase
    ax2 = axes[1, 0]
    ax2.semilogx(omega, np.angle(G, deg=True), 'b-', label='Plant G(s)')
    ax2.semilogx(omega, np.angle(C, deg=True), 'g-', label='Controller C(s)')
    ax2.semilogx(omega, np.angle(L, deg=True), 'r-', label='Loop L(s)')
    ax2.axhline(y=-180, color='k', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Frequency (rad/s)')
    ax2.set_ylabel('Phase (degrees)')
    ax2.set_title('Bode Diagram - Phase')
    ax2.legend()
    ax2.set_xlim([0.01, 1000])
    
    # Sensitivity functions
    ax3 = axes[0, 1]
    ax3.semilogx(omega, 20*np.log10(np.abs(S)), 'b-', label='|S| Sensitivity')
    ax3.semilogx(omega, 20*np.log10(np.abs(T)), 'r-', label='|T| Comp. Sens.')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax3.axhline(y=6, color='orange', linestyle=':', label='M_s = 2 (6 dB)')
    ax3.set_ylabel('Magnitude (dB)')
    ax3.set_title('Sensitivity Functions')
    ax3.legend()
    ax3.set_xlim([0.01, 1000])
    
    # Nyquist-like: |S| + |T|
    ax4 = axes[1, 1]
    theta = np.linspace(0, 2*np.pi, 100)
    ax4.plot(np.cos(theta), np.sin(theta), 'k--', alpha=0.3, label='Unit circle')
    ax4.plot(np.real(L), np.imag(L), 'r-', linewidth=1.5, label='L(jω)')
    ax4.plot(-1, 0, 'kx', markersize=10, markeredgewidth=2, label='Critical point')
    ax4.set_xlabel('Real')
    ax4.set_ylabel('Imaginary')
    ax4.set_title('Nyquist Diagram')
    ax4.set_xlim([-3, 3])
    ax4.set_ylim([-3, 3])
    ax4.set_aspect('equal')
    ax4.legend(loc='lower right')
    ax4.axhline(y=0, color='k', alpha=0.3)
    ax4.axvline(x=0, color='k', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('bode_analysis.png', dpi=300, bbox_inches='tight')
    plt.savefig('bode_analysis.pdf', bbox_inches='tight')
    print("Saved: bode_analysis.png, bode_analysis.pdf")
    plt.close()

def create_robust_stability_plot():
    """Generate robust stability analysis plot"""
    omega = np.logspace(-2, 3, 500)
    
    # Motor parameters
    K = 6.667
    tau = 3.333
    s = 1j * omega
    G = K / (tau * s + 1)
    
    # PI Controller
    Kp, Ki = 5.0, 20.0
    C = Kp + Ki / s
    
    L = G * C
    T = L / (1 + L)
    
    # Uncertainty weight: W_delta(s) = (tau_w*s + r0) / (tau_w/r_inf*s + 1)
    r0 = 0.25
    r_inf = 1.5
    tau_w = 0.05
    W_delta = (tau_w * s + r0) / (tau_w/r_inf * s + 1)
    
    # Robust stability condition: |W_delta * T| < 1
    WdT = np.abs(W_delta * T)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Left: Individual magnitudes
    ax1 = axes[0]
    ax1.semilogx(omega, 20*np.log10(np.abs(T)), 'b-', linewidth=2, label='|T(jω)|')
    ax1.semilogx(omega, 20*np.log10(np.abs(W_delta)), 'g-', linewidth=2, label='|W_Δ(jω)|')
    ax1.semilogx(omega, 20*np.log10(WdT), 'r-', linewidth=2, label='|W_Δ·T|')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5, label='1 (0 dB)')
    ax1.set_xlabel('Frequency (rad/s)')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title('Robust Stability Analysis')
    ax1.legend()
    ax1.set_xlim([0.01, 1000])
    ax1.set_ylim([-40, 20])
    
    # Right: Robust stability test
    ax2 = axes[1]
    ax2.semilogx(omega, WdT, 'r-', linewidth=2, label='|W_Δ·T(jω)|')
    ax2.axhline(y=1, color='k', linestyle='--', linewidth=2, label='Stability boundary')
    ax2.fill_between(omega, WdT, 1, where=(WdT <= 1), color='green', alpha=0.2, label='Stable region')
    ax2.fill_between(omega, WdT, 1, where=(WdT > 1), color='red', alpha=0.2, label='Unstable region')
    ax2.set_xlabel('Frequency (rad/s)')
    ax2.set_ylabel('|W_Δ·T|')
    ax2.set_title('Robust Stability Test: ||W_Δ·T||_∞ < 1')
    ax2.legend()
    ax2.set_xlim([0.01, 1000])
    ax2.set_ylim([0, 1.5])
    
    max_WdT = np.max(WdT)
    idx = np.argmax(WdT)
    omega_crit = omega[idx]
    
    ax2.annotate(f'Max = {max_WdT:.3f}\nω = {omega_crit:.1f} rad/s',
                xy=(omega_crit, max_WdT), xytext=(omega_crit*5, max_WdT + 0.2),
                arrowprops=dict(arrowstyle='->', color='black'),
                fontsize=10)
    
    plt.tight_layout()
    plt.savefig('robust_stability.png', dpi=300, bbox_inches='tight')
    plt.savefig('robust_stability.pdf', bbox_inches='tight')
    print("Saved: robust_stability.png, robust_stability.pdf")
    
    print(f"\nRobust Stability Analysis:")
    print(f"  ||W_Δ·T||_∞ = {max_WdT:.4f}")
    print(f"  Critical frequency = {omega_crit:.2f} rad/s")
    if max_WdT < 1:
        print(f"  Status: ROBUSTLY STABLE (margin = {1/max_WdT:.2f})")
    else:
        print(f"  Status: NOT ROBUSTLY STABLE")
    
    plt.close()

def main():
    print("=" * 60)
    print("  Robust Control Case Study - Plotting Results")
    print("  Differential Drive Mobile Robot Motor Control")
    print("=" * 60)
    print()
    
    # Change to examples directory
    script_dir = Path(__file__).parent
    os.chdir(script_dir)
    
    # Generate all plots
    print("Generating H-infinity motor control plots...")
    plot_hinf_results()
    print()
    
    print("Generating Case Study 1 step response plot...")
    plot_case1_step_response()
    print()
    
    print("Generating Bode analysis plots...")
    create_bode_plot()
    print()
    
    print("Generating robust stability analysis plots...")
    create_robust_stability_plot()
    print()
    
    print("=" * 60)
    print("  All plots generated successfully!")
    print("  Check the examples folder for PNG/PDF files.")
    print("=" * 60)

if __name__ == '__main__':
    main()
