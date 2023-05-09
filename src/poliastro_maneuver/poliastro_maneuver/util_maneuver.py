from astropy import units as u
import numpy as np

from poliastro.maneuver import Maneuver
from poliastro.bodies import Earth
from poliastro.twobody import Orbit

def run(ros_node, r_init, r_final):

    ros_node.get_logger().info(f"Starting simulation...")

    orb_i = Orbit.circular(Earth, alt=r_init << u.km) #Setting initial orbit
    hoh = Maneuver.hohmann(orb_i, r_final << u.km) #Creating Hohmann maneuver

    orb_a, orb_f = orb_i.apply_maneuver(hoh, intermediate=True) #Executing maneuver

    #Getting orbit ephems
    rri, _ = orb_i.to_ephem().rv() 
    rra, _ = orb_a.to_ephem().rv()
    rrf, _ = orb_f.to_ephem().rv()

    #Preparing arrays
    orbi_output = np.empty((len(rri),3))
    orba_output = np.empty((len(rra),3))
    orbf_output = np.empty((len(rrf),3))

    for i in range(len(rri)):
        orbi_output[i] = rri[i]
        orba_output[i] = rra[i]
        orbf_output[i] = rrf[i]

    # Defining output array
    output = np.concatenate([orbi_output, orba_output, orbf_output])
    ros_node.get_logger().info(f"Simulation finished successfully!")

    return (output)