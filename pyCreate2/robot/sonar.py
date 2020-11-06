"""
Module to interface a PING Sonar connected via GPIO
"""
from .gpio import Gpio
import time


class Sonar:
    """Class to use the PING Sonar

    This class assumes that the PING sonar is connected
    using a GPIO pin. It implements the protocol specified in
    https://www.parallax.com/sites/default/files/downloads/28015-PING-Documentation-v1.6.pdf
    using hardware interrupts.

    Args:
        pin: GPIO pin number where the sonar is connected to.
    """
    def __init__(self, pin):
        """Constructor.

        Args:
            pin (integer): GPIO pin number where the sonar is connected to.
        """
        self._gpio = Gpio(pin)

    def get_distance(self):
        while True:
            res = self._get_distance()
            if res is not None:
                return res

    def _get_distance(self):
        """Queries the current distance from the sonar.

        Starts a new measurement cycle (which takes up to 19 ms) and
        returns the measured distance in m.

        Returns:
            Distance in m. If there was an error, it returns 3.3 m.
        """

        # pulse SIG pin
        self._gpio.set_direction("low")
        # self._gpio.set_value(0)
        time.sleep(200 / 1e6) # 200 us delay before next measurement
        self._gpio.set_value(1)
        time.sleep(5 / 1e6) # 5 us Input trigger
        self._gpio.set_value(0)

        # configure as input and wait for interrupts
        self._gpio.set_direction("in")
        self._gpio.set_edge("both")
        # time.sleep(750 / 1e6)
        while True:
            res = self._gpio.wait_for_interrupt(2)
            # print(res)
            if res is None:
                return None
            if res[0] == "1":
                break
            # if self._gpio.wait_for_interrupt(2) is None:
                # return 3.3
        start = time.time()
        res = self._gpio.wait_for_interrupt(19)
        # print(res)
        if res is None or res[0] != "0":
            return None
        end = time.time()
        travel_time_in_s = end - start
        distance_in_m = travel_time_in_s * 1e6 / 5800
        self._gpio.set_direction("low")
        return distance_in_m
