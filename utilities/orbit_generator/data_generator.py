import concurrent.futures
from functools import partial
import time

import numpy as np
from numpy import typing as np_type
import pandas as pd

import newtonian_orbit


def build_table(
    timing_data: newtonian_orbit.DataTiming, mu, init_pos, init_vel, output_file
) -> pd.DataFrame:
    test_data = {}
    orbit = newtonian_orbit.Orbit(
        mu, initial_position=init_pos, initial_velocity=init_vel
    )
    for datum in newtonian_orbit.generate_data(orbit, timing_data):
        test_data[datum.timestamp_ms / 1000] = (*datum.position, *datum.velocity)

    col_head = [["position", "velocity"], ["x", "y", "z"]]
    col_index = pd.MultiIndex.from_product(col_head, names=["vector", "component"])

    df = pd.DataFrame.from_dict(test_data, orient="index", columns=col_index)
    df["rad"] = df.apply(lambda row: np.hypot(row.position.x, row.position.y), axis=1)
    name = f"ts_{int(1000*timing_data.timestep_ms)}"
    df.to_hdf(output_file, name)


def generate_true_data(radius, speed, time_stop) -> pd.DataFrame:
    # generate 12 orbits at 8sec*50hz=400 intervals per orbit
    _time_sec = 0
    _test_data = {}
    _angle = 0
    _angular_velocity = speed / radius
    _angle_delta = _angular_velocity * 0.02  # 20 ms timestep
    while _time_sec < time_stop:
        position = np.array([radius * np.cos(_angle), radius * np.sin(_angle), 0])
        velocity = np.array([-speed * np.sin(_angle), speed * np.cos(_angle), 0])
        _test_data[_time_sec] = (*position, *velocity)
        _angle = _angle + _angle_delta
        _time_sec = _time_sec + 0.02

    # add last data point, which should be back at 0
    # _test_data[_time_sec] = (*np.array([4, 0, 0]), *np.array([0, np.pi, 0]))

    col_head = [["position", "velocity"], ["x", "y", "z"]]
    col_index = pd.MultiIndex.from_product(col_head, names=["vector", "component"])

    df = pd.DataFrame.from_dict(_test_data, orient="index", columns=col_index)
    return df


def generate_timings(stop_time_sec):
    timestep_bases = [1, 2, 5]
    # timestep_magnitudes = [0.001, 0.01, 0.1, 1, 10]
    timestep_magnitudes = [0.01, 0.1, 1, 10]

    _timings = []
    _stop_time_ms = stop_time_sec * 1000
    for mag in timestep_magnitudes:
        for base in timestep_bases:
            timestep = mag * base
            timing = newtonian_orbit.DataTiming(
                timestep_ms=timestep, stop_time_ms=_stop_time_ms, output_timestep_ms=20
            )
            _timings.append(timing)
    return _timings


if __name__ == "__main__":
    output_file_name = "physical_circles.h5"

    # choose something semi-physical
    _mu = 1e12
    _rad = 1e6
    _p = np.array([_rad, 0, 0])
    _speed = np.sqrt(_mu / _rad)
    _v = np.array([0, _speed, 0])
    bound_generator = partial(
        build_table,
        mu=1e12,
        init_pos=_p,
        init_vel=_v,
        output_file=output_file_name,
    )

    _stop_time_sec = 100
    true_df = generate_true_data(_rad, _speed, _stop_time_sec)

    true_df.to_hdf(output_file_name, "true_circle")

    timings = generate_timings(_stop_time_sec)

    start = time.time()
    with concurrent.futures.ThreadPoolExecutor(14) as executor:
        executor.map(bound_generator, timings)
    stop = time.time()
    print(f"data generated - {stop-start} sec")
