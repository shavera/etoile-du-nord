import time

import numpy as np
from numpy import typing as np_type
import pandas as pd

import newtonian_orbit


class TestScenario:
    def __init__(
        self,
        mu: float,
        initial_position: np_type.ArrayLike,
        initial_velocity: np_type.ArrayLike,
    ):
        self.orbit = newtonian_orbit.Orbit(
            mu, initial_position=initial_position, initial_velocity=initial_velocity
        )

    def build_table(self, timing_data: newtonian_orbit.DataTiming) -> pd.DataFrame:
        test_data = {}
        for datum in newtonian_orbit.generate_data(self.orbit, timing_data):
            test_data[datum.timestamp_ms / 1000] = (*datum.position, *datum.velocity)

        col_head = [["position", "velocity"], ["x", "y", "z"]]
        col_index = pd.MultiIndex.from_product(col_head, names=["vector", "component"])

        df = pd.DataFrame.from_dict(test_data, orient="index", columns=col_index)
        return df


if __name__ == "__main__":
    scenario = TestScenario(1, np.array([1, 0, 0]), np.array([0, 1, 0]))
    timing = newtonian_orbit.DataTiming(
        timestep_ms=1, stop_time_ms=100000, output_timestep_ms=20
    )
    start = time.time()
    data = scenario.build_table(timing)
    data.to_hdf("sample.h5", "table")
    done = time.time()
    print(f"completed in {done-start}")

    print(pd.read_hdf("sample.h5", "table").head())
