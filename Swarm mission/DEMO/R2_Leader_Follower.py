import time
import datetime
import os
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.power_switch import PowerSwitch
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.swarm import Swarm, CachedCfFactory

# Define drone URIs and initial positions
drones = [
    'radio://0/80/2M/E7E7E7E701',
    'radio://0/80/2M/E7E7E7E702',
    'radio://0/80/2M/E7E7E7E703',
    'radio://0/80/2M/E7E7E7E704',
    'radio://0/80/2M/E7E7E7E70A',
    'radio://0/80/2M/E7E7E7E70B',
    'radio://0/80/2M/E7E7E7E70C',
]

psws = [PowerSwitch(drone) for drone in drones]

initialPos = [
    [-0.7, 0.7, 0], [0.7, 0.7, 0],
    [-1.0, 0, 0], [0, 0, 0], [1.0, 0, 0],
    [-0.7, -0.7, 0], [0.7, -0.7, 0],
]

moveDelta = [
    [0.0, 1.0, 0.0], 
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
]

missNo = int(input('Mission type: \n - [0]Blink \n - [1]In-position \n - [2]Line \n - [6]Up-down\n >>'))

arguments = {
    drones[0] : [0, missNo],
    drones[1] : [1, missNo],
    drones[2] : [2, missNo],
    drones[3] : [3, missNo],
    drones[4] : [4, missNo],
    drones[5] : [5, missNo],
    drones[6] : [6, missNo],
}

miss_type = ['NO', 'TR']
drone_ID = ['0A', '0B', '0C', '0D', '0E', '0F', '0G']

# Variables to hold leader's position
leader_x, leader_y, leader_z = 0.0, 0.0, 0.0
prev_leader_x, prev_leader_y, prev_leader_z = 0.0, 0.0, 0.0

follower_logconfs = {}
follower_positions = {}

def follower_position_callback(timestamp, data, logconf, uri):
    if uri not in follower_positions:
        follower_positions[uri] = {'x': [], 'y': [], 'z': []}

    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']

    follower_positions[uri]['x'].append(x)
    follower_positions[uri]['y'].append(y)
    follower_positions[uri]['z'].append(z)

# Log configuration for the leader drone
leader_logconf = LogConfig(name='Leader Position', period_in_ms=100)
leader_logconf.add_variable('stateEstimate.x', 'float')
leader_logconf.add_variable('stateEstimate.y', 'float')
leader_logconf.add_variable('stateEstimate.z', 'float')

# Callback function for updating leader's position
def leader_position_callback(timestamp, data, logconf):
    global leader_x, leader_y, leader_z
    global prev_leader_x, prev_leader_y, prev_leader_z

    leader_x = data['stateEstimate.x']
    leader_y = data['stateEstimate.y']
    leader_z = data['stateEstimate.z']

    delta_x = leader_x - prev_leader_x
    delta_y = leader_y - prev_leader_y
    delta_z = leader_z - prev_leader_z

    for follower_uri in follower_drones:
        sync_follower_movement(follower_uri, delta_x, delta_y, delta_z)

    prev_leader_x, prev_leader_y, prev_leader_z = leader_x, leader_y, leader_z

# Function to synchronize follower drone's movement with the leader
def sync_follower_movement(follower_uri, delta_x, delta_y, delta_z):
    with SyncCrazyflie(follower_uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        phlc = PositionHlCommander(scf)
        phlc.move_distance(delta_x, delta_y, delta_z)

    
# Function to execute the mission for each drone
def mission(scf: SyncCrazyflie, posNo, code):
    takeoff_height = 1.0

    phlc = PositionHlCommander(scf, 
                               x=initialPos[posNo][0], 
                               y=initialPos[posNo][1], 
                               z=initialPos[posNo][2])
    phlc.take_off(takeoff_height, 2.0)
    time.sleep(4)

    if scf.cf.link_uri == leader_drone and code == 2:
        phlc.move_distance(moveDelta[posNo][0], 
                           moveDelta[posNo][1], 
                           moveDelta[posNo][2])
        update_leader_position(phlc)

        delta_x, delta_y, delta_z = moveDelta[posNo]
        for follower_uri in follower_drones:
            sync_follower_movement(follower_uri, delta_x, delta_y, delta_z)

    phlc.land()
    
# Function to update leader drone's position
def update_leader_position(phlc):
    global leader_x, leader_y, leader_z
    leader_x, leader_y, leader_z = phlc.position()

# Main function
# Main function with follower drone log tracking
def main():
    global leader_drone, follower_drones
    cflib.crtp.init_drivers()

    # Define drone positions
    drone_positions = dict(zip(drones, initialPos))

    # Set leader and follower drones
    leader_drone = 'radio://0/80/2M/E7E7E7E704'
    follower_drones = [
        'radio://0/80/2M/E7E7E7E701',
        'radio://0/80/2M/E7E7E7E702',
        'radio://0/80/2M/E7E7E7E703',
        'radio://0/80/2M/E7E7E7E70A',
        'radio://0/80/2M/E7E7E7E70B',
        'radio://0/80/2M/E7E7E7E70C',
    ]

    # Log configurations for follower drones
    follower_logconfs = {uri: LogConfig(name=f'Follower Position {uri[-2:]}', period_in_ms=100) for uri in follower_drones}
    for logconf in follower_logconfs.values():
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')

    factory = CachedCfFactory(rw_cache='./cache')

    with SyncCrazyflie(leader_drone, cf=Crazyflie(rw_cache='./cache')) as leader_cf:
        # Setup leader log configuration and start
        leader_cf.cf.log.add_config(leader_logconf)
        leader_logconf.data_received_cb.add_callback(leader_position_callback)
        leader_logconf.start()

        # Setup follower log configurations and start
        for uri in follower_drones:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as follower_cf:
                follower_logconf = follower_logconfs[uri]
                follower_cf.cf.log.add_config(follower_logconf)
                follower_logconf.data_received_cb.add_callback(
                    lambda ts, data, logconf, uri=uri: follower_position_callback(ts, data, logconf, uri))
                follower_logconf.start()

        with Swarm(drones, factory=factory) as swarm:
            try:
                swarm.parallel_safe(mission, args_dict=arguments)
            except KeyboardInterrupt:
                print('EMERGENCY STOP TRIGGERED')
                for i in psws:
                    i.platform_power_down()
                    print('['+str(hex(i.address[4]))+']: SHUTDOWN')

if __name__ == '__main__':
    main()
