# General Information
This project contains three simple examples using Poliastro lib for Python 3:
1. Poliastro_simple_orbit. It returns vessel orbital coordinates around Earth between time bounds from input apoapsis and periapsis altitudes.
2. Poliastro_maneuver. This package provides three orbits for the Hohmann transition: an initial orbit, an intermediate orbit, and a final orbit. Takes the radius of the initial orbit and the radius of the final orbit as input.
3. Poliastro_atmo_drag. A simple example showing the effect of aerodynamic drag forces on an artificial satellite on low Earth orbit. Takes Earth diameter, drag coefficient, Keppler orbit parameters and maximum simulation time as inputs. The result is a plot of altitude by time and the flight time before hitting the surface.

# Installation
Clone the repository:
```bash 
#TODO
```
# Build
1. Open project root folder as a Docker container
2. Set parameters for simulation in src/<selected_example>/config/params.xml
3. Build ROS2 environment:
```bash 
colcon build
```

# Preparing FoxGlove Studio:
Each example has its unique ros2 publisher, so it's necessary to set Message paths in FoxGlove in advance:
1. For Poliastro_simple_orbit: '/poliastro_simple_orbit/state.data'. Set '/poliastro_simple_orbit/state.data[0]' as Message path in 'Series' tab and '/poliastro_simple_orbit/state.data[1]' as Message path in the 'X Axis' tab. This will provide a plot of orbit. 
2. For Poliastro_maneuver: '/Poliastro_maneuver/state.data'. Set '/poliastro_maneuver/state.data[0]' as Message path in 'Series' tab and '/Poliastro_maneuver/state.data[1]' as Message path in the 'X Axis' tab. This will provide full Hohmann maneuver including initial and final orbits.
3. For Poliastro_atmo_drag: '/Poliastro_atmo_drag/state.data'. Set '/Poliastro_atmo_drag/state.data[0]' as Message path in 'Series' tab and '/Poliastro_atmo_drag/state.data[1]' as Message path in the 'X Axis' tab. This will provide raw orbit data, and '/poliastro_atmo_drag/res.data' will provide total flight time before reaching surface. Although the best way to process simulation results is Citros notebook.

# Run
1. Prepare your FoxGlove studio (previous step).
2. Launch selected ROS2 package:
```bash 
ros2 launch <selected_example> launch.py
```
1. Watch the FoxGlove plot built from results!

# Citros usage
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is Citros! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

# Develope
instruction to further develop the simulation.

# Extras
Images / Videos from Foxglove