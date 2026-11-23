"""Fit each motor's feedforward model and PI gains from an identification log.

Usage: python3 tools/fit_motor.py <log.csv> <chart.png>

Prints the fit and also saves it next to the chart, as <chart>.txt.

Model, as in WPILib SysId: power = kS * sign(v) + kV * v + kA * a. kS and kV
come from the slow ramps, kA from the steps' time constant; PI gains from IMC
(lambda) tuning of that first-order-plus-dead-time model.
"""
import sys
from collections import namedtuple

import matplotlib.pyplot as plt
import numpy as np

from chart_style import BLUE, INK, SURFACE, read_log_rows, style_axes

RAMP_PHASES = (0, 4)
STEP_PHASES = (2, 6)
MOVING = 0.05  # rev/s; slower samples are mostly static friction, so skipped
STEADY_MS = 500  # end of each step used as its steady speed
CONTROL_PERIOD_S = 0.02


StepFit = namedtuple("StepFit", "tau theta v_ss t v power")


def step_model(t, v_ss, tau, theta):
    """First order plus dead time: nothing until theta, then rise with tau."""
    return np.where(t > theta, v_ss * (1 - np.exp(-(t - theta) / tau)), 0.0)


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
            err = float(np.sum((v - step_model(t, v_ss, tau, theta)) ** 2))
            if err < best[0]:
                best = (err, tau, theta)
    _, tau, theta = best
    return StepFit(tau, theta, v_ss, t, v, float(rows[0, 2]))


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
        tau = float(np.mean([s.tau for s in steps]))
        theta = float(np.mean([s.theta for s in steps]))
        k_a = k_v * tau
        # The controller also waits on average half a period before acting.
        delay = theta + CONTROL_PERIOD_S / 2

        say(f"\n{name} motor")
        say(f"  kS = {k_s:.4f} power      (static friction)")
        say(f"  kV = {k_v:.4f} power per rev/s   (ramp fit RMS error {rms:.4f})")
        say(f"  kA = {k_a:.4f} power per rev/s²  "
              f"(tau {tau * 1000:.0f} ms, dead time {theta * 1000:.0f} ms)")
        for s, phase in zip(steps, STEP_PHASES):
            predicted = (abs(s.power) - k_s) / k_v * np.sign(s.power)
            say(f"  step {phase}: power {s.power:+.2f}, tau {s.tau * 1000:.0f} ms, "
                  f"dead time {s.theta * 1000:.0f} ms, steady {s.v_ss:+.3f} rev/s "
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

        for i, s in enumerate(steps):
            ax = axes[row, 1 + i]
            ax.plot(s.t, s.v, color=BLUE, linewidth=2, label="Measured")
            ax.plot(s.t, step_model(s.t, s.v_ss, s.tau, s.theta), color=INK,
                    linewidth=1.5, linestyle=(0, (4, 3)),
                    label=f"Fit: tau {s.tau * 1000:.0f} ms, "
                          f"dead time {s.theta * 1000:.0f} ms")
            is_forward = s.power > 0
            direction = "forward" if is_forward else "backward"
            ax.set_title(f"{name} motor: step to {s.power:+.1f} power, "
                         f"{direction}", loc="left")
            ax.set_xlabel("Time since step (s)")
            ax.set_ylabel("Speed (rev/s)")
            ax.legend(frameon=False,
                      loc="lower right" if is_forward else "upper right")

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
