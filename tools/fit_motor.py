"""Fit each motor's feedforward model and PI gains from an identification log.

Usage: python3 tools/fit_motor.py <log.csv> <chart.png>

Prints the fit and also saves it next to the chart, as <chart>.txt.

Model, as in WPILib SysId: power = kS * sign(v) + kV * v + kA * a. kS and kV
come from the slow ramps, kA from the steps' time constant; PI gains from IMC
(lambda) tuning of that first-order-plus-dead-time model.
"""
import pathlib
import sys
from collections import namedtuple

import matplotlib.pyplot as plt
import numpy as np

from chart_style import BLUE, INK
from log_reader import read_log_rows

# The identification log's columns.
T_MS_COL, PHASE_COL, POWER_COL, FIRST_SPEED_COL, SECOND_SPEED_COL = range(5)
COLUMN_COUNT = 5

RAMP_PHASES = (0, 4)
STEP_PHASES = (2, 6)
MOVING_RPS = 0.05  # slower samples are mostly static friction, so skipped
STEADY_MS = 500  # end of each step used as its steady speed


StepFit = namedtuple("StepFit", "tau theta v_ss t v power")


def step_model(t, v_ss, tau, theta):
    """First order plus dead time: nothing until theta, then rise with tau."""
    return np.where(t > theta, v_ss * (1 - np.exp(-(t - theta) / tau)), 0.0)


def fit_ramps(data, speed_col):
    rows = data[np.isin(data[:, PHASE_COL], RAMP_PHASES)]
    rows = rows[np.abs(rows[:, speed_col]) > MOVING_RPS]
    speed, power = rows[:, speed_col], rows[:, POWER_COL]
    a = np.column_stack([np.sign(speed), speed])
    (k_s, k_v), *_ = np.linalg.lstsq(a, power, rcond=None)
    residual = power - a @ np.array([k_s, k_v])
    return k_s, k_v, speed, power, float(np.sqrt(np.mean(residual**2)))


def fit_step(data, phase, speed_col):
    rows = data[data[:, PHASE_COL] == phase]
    t = (rows[:, T_MS_COL] - rows[0, T_MS_COL]) / 1000.0
    v = rows[:, speed_col]
    v_ss = float(np.mean(v[t >= t[-1] - STEADY_MS / 1000.0]))
    best = (np.inf, 0.0, 0.0)
    for theta in np.arange(0.0, 0.081, 0.001):
        for tau in np.arange(0.005, 0.401, 0.001):
            err = float(np.sum((v - step_model(t, v_ss, tau, theta)) ** 2))
            if err < best[0]:
                best = (err, tau, theta)
    _, tau, theta = best
    return StepFit(tau, theta, v_ss, t, v, float(rows[0, POWER_COL]))


MotorFit = namedtuple("MotorFit",
                      "k_s k_v k_a tau theta rms ramp_speed ramp_power steps")


def fit_model(data, speed_col):
    """kS and kV from the ramps; kA from the steps' mean time constant."""
    k_s, k_v, ramp_speed, ramp_power, rms = fit_ramps(data, speed_col)
    steps = [fit_step(data, phase, speed_col) for phase in STEP_PHASES]
    tau = float(np.mean([s.tau for s in steps]))
    theta = float(np.mean([s.theta for s in steps]))
    return MotorFit(k_s, k_v, k_v * tau, tau, theta, rms, ramp_speed,
                    ramp_power, steps)


def imc_pi_gains(k_v, k_a, delay_s, lambda_s):
    """(kP, kI) for a first-order-plus-dead-time motor, by IMC tuning."""
    return k_a / (lambda_s + delay_s), k_v / (lambda_s + delay_s)


def describe_fit(name, fit, control_period_s):
    """The fit and the PI gains it suggests, as report lines."""
    # The controller also waits on average half a period before acting.
    delay = fit.theta + control_period_s / 2
    lines = [
        f"\n{name} motor",
        f"  kS = {fit.k_s:.4f} power      (static friction)",
        f"  kV = {fit.k_v:.4f} power per rev/s   "
        f"(ramp fit RMS error {fit.rms:.4f})",
        f"  kA = {fit.k_a:.4f} power per rev/s²  "
        f"(tau {fit.tau * 1000:.0f} ms, dead time {fit.theta * 1000:.0f} ms)",
    ]
    for s, phase in zip(fit.steps, STEP_PHASES):
        predicted = (abs(s.power) - fit.k_s) / fit.k_v * np.sign(s.power)
        lines.append(
            f"  step {phase}: power {s.power:+.2f}, "
            f"tau {s.tau * 1000:.0f} ms, "
            f"dead time {s.theta * 1000:.0f} ms, steady {s.v_ss:+.3f} rev/s "
            f"(model predicts {predicted:+.3f})")
    lines.append(f"  PI gains (IMC), loop delay {delay * 1000:.0f} ms:")
    for label, lam in (("tight, lambda = delay", delay),
                       ("moderate, lambda = 2 x delay", 2 * delay),
                       ("gentle, lambda = 4 x delay", 4 * delay)):
        k_p, k_i = imc_pi_gains(fit.k_v, fit.k_a, delay, lam)
        lines.append(f"    {label:30s} kP = {k_p:.3f}, kI = {k_i:.2f}")
    return lines


def plot_ramps(ax, name, fit):
    ax.scatter(fit.ramp_speed, fit.ramp_power, s=6, color=BLUE, alpha=0.5,
               label="Measured")
    line_v = np.linspace(fit.ramp_speed.min(), fit.ramp_speed.max(), 200)
    ax.plot(line_v, fit.k_s * np.sign(line_v) + fit.k_v * line_v, color=INK,
            linewidth=1.5, label=f"Fit: kS {fit.k_s:.3f}, kV {fit.k_v:.3f}")
    ax.set_title(f"{name} motor: slow ramps", loc="left")
    ax.set_xlabel("Speed (rev/s)")
    ax.set_ylabel("Power (PWM fraction)")
    ax.legend(loc="upper left")


def plot_step(ax, name, step):
    ax.plot(step.t, step.v, color=BLUE, linewidth=2, label="Measured")
    ax.plot(step.t, step_model(step.t, step.v_ss, step.tau, step.theta),
            color=INK, linewidth=1.5, linestyle=(0, (4, 3)),
            label=f"Fit: tau {step.tau * 1000:.0f} ms, "
                  f"dead time {step.theta * 1000:.0f} ms")
    is_forward = step.power > 0
    direction = "forward" if is_forward else "backward"
    ax.set_title(f"{name} motor: step to {step.power:+.1f} power, "
                 f"{direction}", loc="left")
    ax.set_xlabel("Time since step (s)")
    ax.set_ylabel("Speed (rev/s)")
    ax.legend(loc="lower right" if is_forward else "upper right")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    log_path, chart_path = sys.argv[1], sys.argv[2]
    data = np.array(read_log_rows(log_path, COLUMN_COUNT))
    control_period_s = float(np.median(np.diff(data[:, T_MS_COL]))) / 1000.0

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    report = []
    for row, (name, speed_col) in enumerate((("First", FIRST_SPEED_COL),
                                             ("Second", SECOND_SPEED_COL))):
        fit = fit_model(data, speed_col)
        report += describe_fit(name, fit, control_period_s)
        plot_ramps(axes[row, 0], name, fit)
        for ax, step in zip(axes[row, 1:], fit.steps):
            plot_step(ax, name, step)
    for ax in axes.flat:
        ax.grid(True, axis="both")
    fig.tight_layout()
    fig.savefig(chart_path, dpi=130)

    print("\n".join(report))
    summary_path = pathlib.Path(chart_path).with_suffix(".txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(f"Fit of {log_path}\n" + "\n".join(report) + "\n")
    print(f"\nChart saved to {chart_path}, numbers to {summary_path}")


if __name__ == "__main__":
    main()
