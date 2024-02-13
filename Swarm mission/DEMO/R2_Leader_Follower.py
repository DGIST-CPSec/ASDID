import time
import math
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig

# URI to the Crazyflie to connect to
leader_uri = 'radio://0/80/2M/E7E7E7E701'
follower_uris = [
    'radio://0/80/2M/E7E7E7E702',
    'radio://0/80/2M/E7E7E7E703',
    'radio://0/80/2M/E7E7E7E704'
]

# Initialize follower drone's positions
follower_positions = [{'x': 0, 'y': 0, 'z': 0} for _ in range(len(follower_uris))]
follower_phlcs = []  # List to store PositionHlCommander objects for followers

def initialize(scf):
    scf.cf.param.request_update_of_all_params()
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    print('[INIT]: kalman prediction reset')
    scf.cf.param.set_value('health.startPropTest', '1')  # propeller test before flight
    time.sleep(5)
    print('[INIT]: initialization complete')

def make_follower_position_callback(index):
    def follower_position_callback(timestamp, data, logconf):
        follower_positions[index]['x'] = data['stateEstimate.x']
        follower_positions[index]['y'] = data['stateEstimate.y']
        follower_positions[index]['z'] = data['stateEstimate.z']
    return follower_position_callback

# Create log configurations for followers to track their positions
follower_logconfs = []
for i, _ in enumerate(follower_uris):
    logconf = LogConfig(name=f'Follower Position {i+1}', period_in_ms=100)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    follower_logconfs.append(logconf)

def update_follower_target_positions(leader_x, leader_y, leader_z):
    """
    Update follower drones' target positions to maintain a minimum distance from the leader drone.
    """
    MIN_DISTANCE = 1.0  # meters
    for index, phlc in enumerate(follower_phlcs):
        dx = leader_x - follower_positions[index]['x']
        dy = leader_y - follower_positions[index]['y']
        # Calculate distance in XY plane only
        distance = math.sqrt(dx**2 + dy**2)
        if distance < MIN_DISTANCE:
            # Calculate adjustment ratio
            ratio = MIN_DISTANCE / distance
            adjust_x = leader_x - dx * ratio
            adjust_y = leader_y - dy * ratio
            # Update target position for follower
            phlc.go_to(adjust_x, adjust_y, leader_z, 0.5)

def position_callback(timestamp, data, logconf):
    # Leader drone's position is now being used to update followers' positions
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    print('Leader Position: ({}, {}, {})'.format(x, y, z))
    update_follower_target_positions(x, y, z)

def mission_phlc(leader_cf, follower_cfs, leader_init_pos, follower_init_positions):
    global follower_phlcs
    takeoff_height = 1.0

    # Initialize and take off with the leader drone
    leader_phlc = PositionHlCommander(leader_cf, x=leader_init_pos[0], y=leader_init_pos[1], z=leader_init_pos[2], default_velocity=0.5)
    leader_phlc.take_off(takeoff_height, velocity=0.5)
    time.sleep(2)

    # Initialize and take off with each follower drone
    follower_phlcs = [PositionHlCommander(follower_cf.cf, x=pos[0], y=pos[1], z=pos[2], default_velocity=0.5) for follower_cf, pos in zip(follower_cfs, follower_init_positions)]
    for phlc in follower_phlcs:
        phlc.take_off(takeoff_height, velocity=0.5)
    time.sleep(2)

    # Perform mission (e.g., leader moves forward)
    leader_phlc.forward(2)
    time.sleep(2)

    # Land all drones
    leader_phlc.land()
    for phlc in follower_phlcs:
        phlc.land()

def main():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(leader_uri, cf=Crazyflie(rw_cache='./cache')) as leader_sc:
        leader_cf = leader_sc.cf
        # Setup log configuration for tracking leader's position
        logconf = LogConfig(name='Position', period_in_ms=100)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        leader_cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(position_callback)
        logconf.start()

        # Initialize and configure each follower
        follower_cfs = []
        for i, uri in enumerate(follower_uris):
            scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
            scf.open_link()
            initialize(scf.cf)
            follower_cfs.append(scf)
            # Setup log configuration for each follower
            follower_logconf = follower_logconfs[i]
            follower_logconf.data_received_cb.add_callback(make_follower_position_callback(i))
            scf.cf.log.add_config(follower_logconf)
            follower_logconf.start()

        # Get initial positions for leader and followers
        leader_init_pos = [0, 0, 0.5]  # Example initial position
        follower_init_positions = [[1, 0, 0.5], [1, 1, 0.5], [-1, 0, 0.5]]  # Example initial positions

        # Start the mission
        mission_phlc(leader_cf, follower_cfs, leader_init_pos, follower_init_positions)

if __name__ == '__main__':
    main()
