import numpy as np

from astropy.time import TimeDelta
from astropy import units as u

from poliastro.bodies import Earth
from poliastro.constants import rho0_earth, H0_earth

from poliastro.core.perturbations import (
    atmospheric_drag_exponential
)
from poliastro.core.propagation import func_twobody
from poliastro.twobody import Orbit
from poliastro.twobody.propagation import CowellPropagator
from poliastro.twobody.sampling import EpochsArray
from poliastro.twobody.events import LithobrakeEvent

def run(ros_node, earth_r, a, ecc, inc, raan, argp, nu, c_d, t_limit):

        ros_node.get_logger().info(f"Starting simulation...")

        R = earth_r # Defining Earth param

        # Defining orbit from imported values
        orbit = Orbit.from_classical(Earth,
                                a = a * u.km,
                                ecc = ecc * u.one,
                                inc = inc * u.deg,
                                raan = raan * u.deg,
                                argp = argp * u.deg,
                                nu = nu * u.deg
                                )

        # Parameters of a body
        C_D = c_d  # dimentionless (any value would do)
        A_over_m = ((np.pi / 4.0) * (u.m**2) / (100 * u.kg)).to_value(
        u.km**2 / u.kg
        )  # km^2/kg
        B = C_D * A_over_m

        # Parameters of the atmosphere
        rho0 = rho0_earth.to(u.kg / u.km**3).value  # kg/km^3
        H0 = H0_earth.to(u.km).value

        # Defining timeline
        tofs = TimeDelta(np.linspace(0 * u.h, 100000 * u.s, num=2000))

        # Atmospheric drag function
        def f(t0, state, k):
                du_kep = func_twobody(t0, state, k)
                ax, ay, az = atmospheric_drag_exponential(
                        t0,
                        state,
                        k,
                        R=R,
                        C_D=C_D,
                        A_over_m=A_over_m,
                        H0=H0,
                        rho0=rho0,
                )
                du_ad = np.array([0, 0, 0, ax, ay, az])

                return du_kep + du_ad


        rr, _ = orbit.to_ephem(
        EpochsArray(orbit.epoch + tofs, method=CowellPropagator(f=f)),
        ).rv()

        orbit = Orbit.from_classical(Earth,
                        a = a * u.km,
                        ecc = ecc * u.one,
                        inc = inc * u.deg,
                        raan = raan * u.deg,
                        argp = argp * u.deg,
                        nu = nu * u.deg
                        )

        tofs = TimeDelta(np.linspace(0 * u.h, t_limit * u.d, num=2000))
        
        # Catching event
        lithobrake_event = LithobrakeEvent(R)
        events = [lithobrake_event]

        rr, _ = orbit.to_ephem(
        EpochsArray(
                orbit.epoch + tofs, method=CowellPropagator(f=f, events=events)
        ),
        ).rv()

        # Preparing results for output
        orb_output = np.empty((len(rr)-1,3))
        for i in range(len(rr)-1):
            orb_output[i] = rr[i]

        ros_node.get_logger().info(f"Simulation finished successfully!")

        return (orb_output, lithobrake_event.last_t.to(u.d).value)