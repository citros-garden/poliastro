---
slug: Poliastro example using CITROS
title: Poliastro example using CITROS
authors: [gtep]
tags: [CITROS]
---

Blog posts support [Docusaurus Markdown features](https://docusaurus.io/docs/markdown-features), such as [MDX](https://mdxjs.com/).

## Example overview 🌐 

![jpg](img/img0.jpg "https://images.nasa.gov/details/0202375")

This project contains three simple examples using Poliastro lib for Python 3:
1. `Poliastro_simple_orbit`. It returns vessel orbital coordinates around Earth between time bounds from input apoapsis and periapsis altitudes. The result is an ephemerides of orbit (actually a part of it between given time bounds) with zero right ascension of the ascending node, argument of the pericenter and true anomaly for simplicity. 
2. `Poliastro_maneuver`. This package provides three orbits for the Hohmann transition: an initial orbit, an intermediate orbit, and a final orbit. Takes the radius of the initial orbit and the radius of the final orbit as input. You will get the ephemerides of these orbits, not the trajectory! 
3. `Poliastro_atmo_drag`. A simple example showing the effect of aerodynamic drag forces on an artificial satellite on low Earth orbit. Takes Earth diameter, drag coefficient, Keppler orbit parameters and maximum simulation time as inputs. The result is a plot of altitude by time and the flight time before hitting the surface.

You can find all information about used functions and mathematical explanation on the [Poliastro Website](https://docs.poliastro.space/en/stable/).


## Code overview
### Poliastro Atmospheric Drag simulation
This example consists of two main files:
1. ```poliastro_atmo_drag.py```. This Python file defines a ROS 2 node called poliastro_atmo_drag that simulates the dynamics of an orbiting object in near-Earth space with atmospheric drag. It publishes the object's state (position coordinates) at a specified frequency and, when the simulation is complete, publishes a result. The simulation parameters, such as orbit characteristics and drag coefficient, are declared as ROS 2 parameters and retrieved from the parameter server. The code initializes the ROS 2 runtime, creates the node, and keeps it running until the simulation is finished, at which point it shuts down the runtime.
2. ```util_atmo_drag.py```. The file wraps Poliastro functions and initializes an orbit based on classical orbital elements and sets up parameters for atmospheric drag, such as drag coefficient and atmospheric density. The simulation calculates the object's trajectory over a specified time interval, with the option to detect a potential collision with Earth's surface (lithobrake event). The function returns the object's position data and the time of the lithobrake event, logging a success message upon completion.

You can find more information about this example on the [Poliastro examples page](https://docs.poliastro.space/en/stable/examples/Natural%20and%20artificial%20perturbations.html).
The Poliastro Atmospheric Drag simulation has the following ROS parameters:

$$
\begin{array}{|c|c|}
\hline
\text{Parameter} & \text{Description} \\
\hline
earth\_r & \text{Earth radius} \\
a & \text{Semimajor axis} \\
ecc & \text{Eccentricity} \\
inc & \text{Inclination} \\
raan & \text{Right Ascension of the Ascending Node} \\
argp & \text{Argument of periapsis} \\
nu & \text{True anomaly} \\
c\_d & \text{Drag coefficient} \\
t\_limit & \text{Maximum simulation duration} \\
publish~freq & \text{frequency of publishing} \\
\hline
\end{array}
$$

The parameters usage explanation written in the [Usage section](#configuring-the-project-⚙️)

### Poliastro Maneuver simulation
This example consists of two main files:
1. ```poliastro_maneuver.py```. This Python file defines a ROS 2 node called poliastro_maneuver responsible for performing orbital maneuvers. It initializes the node and publishes the object's position data during the maneuver. Parameters for the initial and target orbit altitudes are declared and retrieved from the ROS 2 parameter server. The code invokes a simulation function to calculate the object's trajectory during the maneuver. It sets up a timer to periodically publish the object's state and completes the ROS 2 runtime once the maneuver is finished.

2. ```util_maneuver.py```. The file wraps a Poliastro simulation function that models a Hohmann orbital maneuver around Earth. It starts with an initial circular orbit at a given altitude and calculates a Hohmann maneuver to transfer to a different orbit with a specified final altitude. The code computes position data for the initial, intermediate, and final orbits and stores them in separate arrays. These arrays are then concatenated into a single output array. Finally, a success message is logged, and the function returns the concatenated position data.

You can find more information about this example on the [Poliastro Website](https://docs.poliastro.space/en/stable/quickstart.html).

This example has only 2 ROS parameters: ```r_init``` - initial orbital altitude, and ```r_final``` - final orbital altitude. The parameters usage explanation written in the [Usage section](#configuring-the-project-⚙️)

### Poliastro Simple Orbit simulation
This example consists of two main files:
1. ```poliastro_simple_orbit.py```. This Python code defines a ROS 2 node called poliastro_simple_orbit for simulating a basic orbit and publishing the object's position data during the orbit. It declares parameters for initial and final orbit radius, as well as the simulation's time range, retrieved from the ROS 2 parameter server. It sets up a timer to periodically publish the object's state data and logs the information. Finally, the ROS 2 runtime is initialized, the poliastro_simple_orbit node is executed, and the runtime is gracefully shut down upon completion.
2. ```util_simple_orbit.py```.  The file wraps a Poliastro simulation function to model a basic orbit around Earth. It calculates the orbit's position data for a parking orbit, defined by its apogee and perigee radii, over a specified time interval. The function uses the calculated parameters to create the parking orbit and samples its position within the given time bounds. The resulting position data is stored in an array and returned. A success message is logged at the end of the simulation.

You can find more information about this example on the [Poliastro Website](https://docs.poliastro.space/en/stable/quickstart.html).
The Poliastro Atmospheric Drag simulation has the following ROS parameters:

$$
\begin{array}{|c|c|}
\hline
\text{Parameter} & \text{Description} \\
\hline
apo\_r & \text{Apoapsis alitude} \\
peri\_r & \text{Periapsis alitude} \\
start\_t & \text{Start date and time} \\
finish\_t & \text{Final date and time} \\
publish~freq & \text{frequency of publishing} \\
\hline
\end{array}
$$

The parameters usage explanation written in the [Usage section](#configuring-the-project-⚙️)

## Local Usage 💻

All project installation and usage information also available in the project [GitHub page](https://github.com/citros-garden/poliastro).

### Installation 🔨
1. Docker engine. This project runs inside Docker container, and requires Docker Engine/Docker Desktop. Follow the instructions on [Docker official website](https://www.docker.com/get-started/).
2. To use Docker inside VS Code several extensions are required. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) extensions from Extensions tab on your left control panel.
3. Clone the repository:
```bash 
git clone git@github.com:citros-garden/poliastro.git
```

### Build 🏠
1. Open project root folder in VS Code.
2. Navigate to the lower-left corner of VS Code window and click on green mark.
3. Select "Reopen in container" option in the list on the top of the VS Code window. Wait a minute while Docker container is starting.
2. Open ```/src/{your-selected-example}/config/params.xml``` file to set parameters for simulation or just keep it default. Don't forget to save your changes!
3. Build ROS 2 environment:
```bash 
colcon build
```
4. Source the environment:
```bash 
source install/local_setup.bash
```

### Preparing FoxGlove Studio 🪄
FoxGlove Studio is a robotics visualization and debugging tool, which can connect to ROS topic and get the data publishing through it. We will use it to visualizate the results of our simulations.

First of all, you need to download it from the [official website](https://foxglove.dev/) and install following the instructions. 

Next step is connecting to your ROS node. To perform it, open FoxGlove Studio and select *__Open connection__* option, then select *__Rosbridge__* option. Check the *__WebSocket URL__* field on the right of the window, it should contain ```ws://localhost:9090```. Now we are almost ready to go!


OR

:::tip

You can use prepared layout: Go to the ```Layout``` tab on the top panel, then click on import_layout button and select the file from foxglove_layouts folder.

:::

![png](img/atmo_drag0.png "FoxGlove example")

:::tip

The best way to process simulation results is CITROS notebook 🍋 :)

:::


### Run 🚀
1. Go back to the VS Code.
2. Launch ROS 2 package:

For Simple Orbit Example:
```bash 
ros2 launch poliastro_simple_orbit poliastro_simple_orbit.launch.py
```
For Maneuver Example:
```bash 
ros2 launch poliastro_maneuver poliastro_maneuver.launch.py
```
For Atmo Drag Example:
```bash 
ros2 launch poliastro_atmo_drag poliastro_atmo_drag.launch.py
```
3. Watch the FoxGlove plot built from results!

OR

:::tip

You can use Visual Code Tasks: simply press ```Alt+T``` and select the task you want to build. This will build, source and launch the project automaticly.

:::

![gif](img/main.gif "FoxGlove example")

## CITROS usage 🛸
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is CITROS! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

### CITROS installation 🛫

First of all, to use all the powerfull CITROS features usage requires CITROS installation: follow the instructions on the CITROS CLI [GitHub page](https://github.com/lulav/citros_cli).

### Configuring the project ⚙️
After all the prerequisites done, we can start configuring our project. The starting point is the Lunar_Starship devcontainer loaded and running, CITROS CLI is installed and ready.
1. Initialize CITROS:
```bash 
citros init
```
Now you can see ```.citros``` folder in the explorer.

2. Configuring the setup. We need to set up the maximum perfomance available: timeout, CPU, GPU and Memory. To perform it, we need to define it in the ```.citros/simulations/simulation_turtlebot3.json```. The recommended setup is minimum 180 seconds timeout, 2 CPU, and 2048 MB of Memory. Don't forget to save the file!

3. Configuring the params setup. You can find default setup in ```.citros/parameter_setups/default_param_setup.json```. Check the [table](#code-overview) for the list of parameters available.

Don't forget to save the file!

### Syncing the project's setup 📡
Now we can sync our project settings with CITROS server:
```bash 
citros commit
citros push
```
### Running locally 🛋️
Since all the preparations done, we can launch it locally (your project should be built and sourced before that, check the instructions above):
```bash 
citros run -n 'poliastro' -m 'local test run'
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

### Uploading Docker image to the CITROS database and running in the cloud 🛰️
1. We need to build and push Docker container image to the CITROS server:
```bash 
citros docker-build-push
```

2. Finally, we can run it in the cloud! Simply add ```-r``` to the terminal command: 
```bash 
citros run -n 'poliastro' -m 'cloud test run' -r
```
Select the launch file (should be the only one here) by pressing ```Enter``` button. Now the simulation is running in the CITROS server, and it will upload results to the CITROS database automaticly.

### CITROS Web usage and data analysis 🌌
#### Launching project via CITROS Web
The best way to use all the innovative capabilities of CITROS is through it's Web interface. The following manual explains how to run this project in the cloud and how to process the simualtion results.
The starting point is CITROS main page, user is logged in and the project Docker image is built and pushed to the cloud (see the [manual](#uploading-docker-image-to-the-citros-database-and-running-in-the-cloud-🛰️) above).
1. Go to the ```Repositories``` page clicking on the tab on the top;
2. Find your project and open it;
3. Navigate to the ```Runs``` tab;
4. Click on the ```Run Simulation``` button on the right;
5. Now you can choose the project and the simulation setup from the droplists, set the number of repeats and how many simulations should run in parallel, type the Name of the run and the additional message. This window also shows the perfomance preset.
6. We are ready to go! Start the Batch with the button below.

The simualtion launched! Open the Run you just started in the list on ```Runs``` page to check how is it going. In this page you can find all the runs of this batch. The number of runs here equals to the number of runs you've set before.
Navigate to the Run by clicking on it in the table:
* The main part of this page is a simulation's log. Here you can find all the logging information from all levels: from your code logs up to the CITROS system information.
* The right part of the page provides additional information about Events: the main stages of the simulation run.

![png](img/citros0.png "CITROS example")

#### Working with integrated Jupiter Notebooks
CITROS Web provides powerfull data analisys package, which is comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data. To use it, the Jupiter Notebook support is built-in. 
Navigate to our project ```Code``` page, open the Notebooks folder and click on the notebook file. Here you can see the usual Jupiter editor's interface: you can add blocks of code or built-in Markdown engine, run and save notebook and control the Python kernel.

You can find all the data analisys package [here](https://citros.io/doc/docs_data_analysis).

![png](img/citros1.png "CITROS example")

## Extras
### FoxGlove examples

![png](img/maneuver.png "FoxGlove example")
