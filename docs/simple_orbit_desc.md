# Simple orbit example using Poliastro lib
## Description

The package returns vessel orbital coordinates around Earth between given time bounds from input apoapsis and periapsis altitudes using simple orbital mechanics provided by Poliastro lib under the hood.

The result is an ephemerides of orbit (actually a part of it between given time bounds) with zero right ascension of the ascending node, argument of the pericenter an true anomaly for simplicity. All the calculations defined in [util_simple_orbit.py file](/./src/poliastro_simple_orbit/poliastro_simple_orbit/util_simple_orbit.py).

Another important file is [poliastro_simple_orbit.py file](/./src/poliastro_simple_orbit/poliastro_simple_orbit/poliastro_simple_orbit.py). ROS 2 node is used to call simulation function, get the results and publish it via ROS topic. You also can set publishing frequency (```publish_freq```) in the [params.yaml config file](/./src/poliastro_simple_orbit/config/params.yaml) (more info about this file below).

## Usage
All the preparations described in the [main Readme file](/./README.md). This package allows you to set 4 parameters of the simulation:
1. Altitude of the apocenter ```apo_r``` above the Earth surface in km (default is 10000.00 km)
2. Altitude of pericenter ```peri_r``` above the Earth surface in km (default is 10000.00 km)
3. Start time ```start_t``` formated as ```YYYY-MM-DD HH:MM``` (default is  2022-01-01 00:00)
4. Finish time ```finish_t``` formated as ```YYYY-MM-DD HH:MM``` (default is  2022-01-01 05:00)

You can set all of the parameters in the [params.yaml config file](/./src/poliastro_simple_orbit/config/params.yaml) or just keep it default :)

5. Now you can start the simulation following the instructions in the [main Readme file](/./README.md)!

## Results
![png](/docs/img/simple_orbit.png "Maneuver example")