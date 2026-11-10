"""Fit each motor's feedforward model and PI gains from an identification log.

Usage: python3 tools/fit_motor.py <log.csv> <chart.png>

Prints the fit and also saves it next to the chart, as <chart>.txt.

The log comes from the firmware's identification test (kMode = Mode::kSysIdTest):
columns t_ms, phase, power, first_speed, second_speed. Phases 0 and 4 are slow
power ramps, 2 and 6 are power steps, the rest are pauses.

Model, as in WPILib SysId: power = kS * sign(v) + kV * v + kA * a
  - kS, kV: least-squares fit to the slow ramps, where acceleration is tiny.
  - kA: from the step response's time constant, tau = kA / kV, fitted as a
    first-order response with a dead time theta.

PI gains use IMC (lambda) tuning for that first-order-plus-dead-time model:
  kP = kA / (lambda + delay),  kI = kV / (lambda + delay)
"""
import sys

import matplotlib.pyplot as plt
import numpy as np

from chart_style import BLUE, INK, SURFACE, read_log_rows, style_axes

RAMP_PHASES = (0, 4)
STEP_PHASES = (2, 6)
MOVING = 0.05  # rev/s; slower samples are mostly static friction, so skipped
STEADY_MS = 500  # end of each step used as its steady speed
CONTROL_PERIOD_S = 0.02


def load(path):
    rows = read_log_rows(path, 5)
    if not rows:
        sys.exit(f"No identification data found in {path}")
    return np.array(rows)


def fit_ramps(data, col):
    rows = data[np.isin(data[:, 1], RAMP_PHASES)]
    rows = rows[np.abs(rows[:, col]) > MOVING]
    speed, power = rows[:, col], rows[:, 2]
    a = np.column_stack([np.sign(speed), speed])
    (k_s, k_v), *_ = np.linalg.lstsq(a, power, rcond=None)
    residual = power - a @ np.array([k_s, k_v])
    return k_s, k_v, speed, power, float(np.sqrt(np.mean(residual**2)))


def fit_step(data, phase, col):
    rows = data[data[:, 1] == phase]
    t = (rows[:, 0] - rows[0, 0]) / 1000.0
    v = rows[:, col]
    v_ss = float(np.mean(v[t >= t[-1] - STEADY_MS / 1000.0]))
    best = (np.inf, 0.0, 0.0)
    for theta in np.arange(0.0, 0.081, 0.001):
        for tau in np.arange(0.005, 0.401, 0.001):
            model = np.where(t > theta,
                             v_ss * (1 - np.exp(-(t - theta) / tau)), 0.0)
            err = float(np.sum((v - model) ** 2))
            if err < best[0]:
                best = (err, tau, theta)
    _, tau, theta = best
    return tau, theta, v_ss, t, v, float(rows[0, 2])


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    log_path, chart_path = sys.argv[1], sys.argv[2]
    data = load(log_path)
    report = []

    def say(line=""):
        print(line)
        report.append(line)

    fig, axes = plt.subplots(2, 3, figsize=(14, 8), facecolor=SURFACE)

    for row, (name, col) in enumerate((("First", 3), ("Second", 4))):
        k_s, k_v, speed, power, rms = fit_ramps(data, col)
        steps = [fit_step(data, phase, col) for phase in STEP_PHASES]
        tau = float(np.mean([s[0] for s in steps]))
        theta = float(np.mean([s[1] for s in steps]))
        k_a = k_v * tau
        # The controller also waits on average half a period before acting.
        delay = theta + CONTROL_PERIOD_S / 2

        say(f"\n{name} motor")
        say(f"  kS = {k_s:.4f} power      (static friction)")
        say(f"  kV = {k_v:.4f} power per rev/s   (ramp fit RMS error {rms:.4f})")
        say(f"  kA = {k_a:.4f} power per rev/s²  "
              f"(tau {tau * 1000:.0f} ms, dead time {theta * 1000:.0f} ms)")
        for (tau_i, theta_i, v_ss, _, _, p), phase in zip(steps, STEP_PHASES):
            predicted = (abs(p) - k_s) / k_v * np.sign(p)
            say(f"  step {phase}: power {p:+.2f}, tau {tau_i * 1000:.0f} ms, "
                  f"dead time {theta_i * 1000:.0f} ms, steady {v_ss:+.3f} rev/s "
                  f"(model predicts {predicted:+.3f})")
        say(f"  PI gains (IMC), loop delay {delay * 1000:.0f} ms:")
        for label, lam in (("tight, lambda = delay", delay),
                           ("moderate, lambda = 2 x delay", 2 * delay),
                           ("gentle, lambda = 4 x delay", 4 * delay)):
            say(f"    {label:30s} kP = {k_a / (lam + delay):.3f}, "
                  f"kI = {k_v / (lam + delay):.2f}")

        ax = axes[row, 0]
        ax.scatter(speed, power, s=6, color=BLUE, alpha=0.5, label="Measured")
        line_v = np.linspace(speed.min(), speed.max(), 200)
        ax.plot(line_v, k_s * np.sign(line_v) + k_v * line_v, color=INK,
                linewidth=1.5, label=f"Fit: kS {k_s:.3f}, kV {k_v:.3f}")
        ax.set_title(f"{name} motor: slow ramps", loc="left")
        ax.set_xlabel("Speed (rev/s)")
        ax.set_ylabel("Power (PWM fraction)")
        ax.legend(frameon=False, loc="upper left")

        for i, (tau_i, theta_i, v_ss, t, v, p) in enumerate(steps):
            ax = axes[row, 1 + i]
            model = np.where(t > theta_i,
                             v_ss * (1 - np.exp(-(t - theta_i) / tau_i)), 0.0)
            ax.plot(t, v, color=BLUE, linewidth=2, label="Measured")
            ax.plot(t, model, color=INK, linewidth=1.5, linestyle=(0, (4, 3)),
                    label=f"Fit: tau {tau_i * 1000:.0f} ms, "
                          f"dead time {theta_i * 1000:.0f} ms")
            direction = "forward" if p > 0 else "backward"
            ax.set_title(f"{name} motor: step to {p:+.1f} power, {direction}",
                         loc="left")
            ax.set_xlabel("Time since step (s)")
            ax.set_ylabel("Speed (rev/s)")
            ax.legend(frameon=False, loc="lower right" if p > 0 else "upper right")

    for ax in axes.flat:
        style_axes(ax, grid_axis="both")

    fig.tight_layout()
    fig.savefig(chart_path, dpi=130, facecolor=SURFACE)
    summary_path = chart_path.rsplit(".", 1)[0] + ".txt"
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(f"Fit of {log_path}\n" + "\n".join(report) + "\n")
    print(f"\nChart saved to {chart_path}, numbers to {summary_path}")


if __name__ == "__main__":
    main()
