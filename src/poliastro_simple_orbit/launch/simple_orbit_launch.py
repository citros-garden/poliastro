import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

class bcolors:
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

def generate_launch_description():
    ld = LaunchDescription()
    poliastro_simple_orbit_config = os.path.join(
        get_package_share_directory('poliastro_simple_orbit'),
        'config',
        'params.yaml'
        )
    poliastro_simple_orbit=Node(
        package = 'poliastro_simple_orbit',
        name = 'poliastro_simple_orbit',
        executable = 'poliastro_simple_orbit',
        parameters = [poliastro_simple_orbit_config]
    )

    sys_shut_down = RegisterEventHandler(OnProcessExit(
    target_action=poliastro_simple_orbit,
    on_exit=[
                LogInfo(msg=(f'{bcolors.OKGREEN}The Scenario has ended!{bcolors.ENDC}')),
                EmitEvent(event=Shutdown(
                    reason='Finished'))
            ]		
    ))

    bridge_dir = get_package_share_directory('rosbridge_server')
    bridge_launch =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml')) 
    ld = LaunchDescription ([bridge_launch, poliastro_simple_orbit, sys_shut_down])
    return ld