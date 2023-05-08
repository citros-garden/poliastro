from astropy import time
from astropy import units as u

import numpy as np

from poliastro.bodies import Earth
from poliastro.twobody import Orbit
from poliastro.twobody.sampling import EpochBounds

def run(ros_node, apo_r, peri_r, start_t, finish_t):

        ros_node.get_logger().info(f"Starting simulation...")

        r_p = Earth.R + apo_r * u.km
        r_a = Earth.R + peri_r * u.km

        a_parking = (r_p + r_a) / 2
        ecc_parking = 1 - r_p / a_parking

        start_date = time.Time(start_t, scale="utc")
        end_date = time.Time(finish_t, scale="utc")

        parking = Orbit.from_classical(
        Earth,
        a_parking,
        ecc_parking,
        0 * u.deg,
        0 * u.deg,
        0 * u.deg,
        0 * u.deg,  # We don't mind
        start_date
        )

        ephem1, _ = parking.to_ephem(strategy=EpochBounds(min_epoch=start_date, max_epoch=end_date)).rv()

        # Preparing results for output
        orb_output = np.empty((len(ephem1)-1,3))
        for i in range(len(ephem1)-1):
            orb_output[i] = ephem1[i]

        ros_node.get_logger().info(f"Simulation finished successfully!")

        return (orb_output)