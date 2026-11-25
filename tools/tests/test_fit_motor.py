import numpy as np
import pytest

from fit_motor import (COLUMN_COUNT, FIRST_SPEED_COL, PHASE_COL, POWER_COL,
                       RAMP_PHASES, STEP_PHASES, T_MS_COL, fit_ramps, fit_step,
                       step_model)

K_S = 0.012
K_V = 0.345


def log_rows(phase, t_ms, power, speed):
    """Identification log rows, with the first motor's speed."""
    rows = np.zeros((len(t_ms), COLUMN_COUNT))
    rows[:, T_MS_COL] = t_ms
    rows[:, PHASE_COL] = phase
    rows[:, POWER_COL] = power
    rows[:, FIRST_SPEED_COL] = speed
    return rows


def test_step_model_waits_for_the_dead_time_then_rises():
    t = np.array([0.0, 0.010, 0.060])
    v = step_model(t, v_ss=1.0, tau=0.050, theta=0.010)
    assert v[0] == 0.0
    assert v[1] == 0.0
    assert v[2] == pytest.approx(1 - np.exp(-1))


def test_ramps_give_back_static_friction_and_speed_gain():
    forward = np.linspace(0.1, 2.0, 50)
    speed = np.concatenate([forward, -forward])
    power = K_S * np.sign(speed) + K_V * speed
    data = log_rows(RAMP_PHASES[0], np.arange(100) * 20, power, speed)
    k_s, k_v, _, _, rms = fit_ramps(data, FIRST_SPEED_COL)
    assert k_s == pytest.approx(K_S)
    assert k_v == pytest.approx(K_V)
    assert rms == pytest.approx(0.0, abs=1e-12)


def test_ramps_skip_samples_too_slow_to_be_moving():
    speed = np.array([0.5, 1.0, 1.5, 0.01])
    power = K_S + K_V * speed
    power[-1] = 0.3  # static friction holding, not the model
    data = log_rows(RAMP_PHASES[0], np.arange(4) * 20, power, speed)
    k_s, k_v, *_ = fit_ramps(data, FIRST_SPEED_COL)
    assert k_s == pytest.approx(K_S)
    assert k_v == pytest.approx(K_V)


def test_step_gives_back_its_time_constant_and_dead_time():
    t_ms = np.arange(0, 3000, 20)
    speed = step_model(t_ms / 1000.0, v_ss=1.4, tau=0.050, theta=0.011)
    data = log_rows(STEP_PHASES[0], t_ms, 0.5, speed)
    fit = fit_step(data, STEP_PHASES[0], FIRST_SPEED_COL)
    assert fit.tau == pytest.approx(0.050, abs=0.002)
    assert fit.theta == pytest.approx(0.011, abs=0.002)
    assert fit.v_ss == pytest.approx(1.4)
    assert fit.power == 0.5
