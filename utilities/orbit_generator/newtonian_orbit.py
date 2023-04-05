from typing import Generator, NamedTuple, NoReturn

import numpy as np
from numpy import typing as np_type


class Orbit:
    def __init__(
        self,
        mu: float,
        initial_position: np_type.ArrayLike,
        initial_velocity: np_type.ArrayLike,
    ):
        self.mu = mu
        self.position = initial_position
        self.velocity = initial_velocity
        self.time_microsec = int(0)

    def advance_by_time(self, time_step: float) -> NoReturn:
        """
        Assume constant acceleration for the duration
        F=mu*m/r^2 -> a=(mu/r^2) in the radially inward direction
        (inward is inverse of position vector)
        x += (a*t^2)/2 + vt
        v += a*t

        :param time_step: time in milliseconds, but could be float to get microsecond precision if desired.
        :return: no return
        """
        r_squared = np.dot(self.position, self.position)
        a_mag = self.mu / r_squared
        r = np.sqrt(r_squared)
        accel = [-(a_mag / r)] * self.position
        time_sec = time_step / 1000
        self.position = (
            ((time_sec * time_sec) / 2) * accel
            + self.velocity * time_sec
            + self.position
        )
        self.velocity = self.velocity + time_sec * accel


class DataRow(NamedTuple):
    timestamp_ms: float
    position: np_type.ArrayLike
    velocity: np_type.ArrayLike


class DataTiming(NamedTuple):
    """
    Class to gather the timing parameters for the test

    :param timestep_ms: how much time the simulation should advance each iter (milliseconds)
    :param stop_time_ms: run the simulation for no more than this time (milliseconds)
    :param output_timestep_ms: try to time output to this length of time (milliseconds)
    """

    timestep_ms: float = (20,)
    stop_time_ms: float = (1e3,)
    output_timestep_ms: float = (20,)


def generate_data(orbit: Orbit, timing: DataTiming) -> Generator[DataRow, None, None]:
    """
    Walk through the orbit in time from t=0, to next moment past the stop time
    output the data at every step where the current time meets the next output
    time step or exceeds it by less than the next functional timestep;
    e.g. if timestep is 8 ms and output is 20 ms, then output 0, 24, 40, 64...
    ideally we'll just run this where they're evenly divisible and not have
    to worry about that.

    :param orbit: Orbit to advance physically
    :param timing: timing parameters needed for test
    :returns: (as a generator) the data element for each output time
    """
    current_time_ms = 0
    yield DataRow(current_time_ms, orbit.position, orbit.velocity)
    while current_time_ms < timing.stop_time_ms:
        orbit.advance_by_time(timing.timestep_ms)
        current_time_ms = current_time_ms + timing.timestep_ms
        if (current_time_ms % timing.output_timestep_ms) < timing.timestep_ms:
            yield DataRow(current_time_ms, orbit.position, orbit.velocity)
