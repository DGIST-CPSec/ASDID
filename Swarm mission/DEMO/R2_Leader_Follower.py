
import time
import datetime
import os
import math
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig

# URI to the Crazyflie to connect to
leader_uri = 'radio://0/80/2M/E7E7E7E701'
follower_uri = [
    'radio://0/80/2M/E7E7E7E70A',
    'radio://0/80/2M/E7E7E7E70B',
    'radio://0/80/2M/E7E7E7E70C',
    'radio://0/80/2M/E7E7E7E70D',
    'radio://0/80/2M/E7E7E7E70E',
    'radio://0/80/2M/E7E7E7E70E',
    'radio://0/80/2M/E7E7E7E70E',
    'radio://0/80/2M/E7E7E7E70E',
]

follower_positions = {uri: (0, 0, 0) for uri in follower_uris}

def initialize(scf):
    scf.cf.param.request_update_of_all_params()
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    print('[INIT]: kalman prediction reset')
    scf.cf.param.set_value('health.startPropTest', '1')  # propeller test before flight
    time.sleep(5)
    print('[INIT]: initialization complete')

def position_callback(timestamp, data, logconf):
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    print('Position: ({}, {}, {})'.format(x, y, z))

def follower_position_callback(timestamp, data, logconf, uri):
    global follower_positions
    follower_positions[uri] = (data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])

def update_follower_positions(scfs, leader_phlc):
    safe_distance = 0.1  # 10 cm
    leader_position = (leader_phlc._x, leader_phlc._y, leader_phlc._z)

    for scf in scfs:
        with PositionHlCommander(scf.cf) as follower_phlc:
            current_position = follower_positions[scf.cf.link_uri]
            distance_to_leader = math.sqrt(sum((c - l) ** 2 for c, l in zip(current_position, leader_position)))

            if distance_to_leader < safe_distance:
                # 리더 드론으로부터 멀어지기
                follower_phlc.back(0.1)
            else:
                # 리더 드론을 따라가기
                follower_phlc.go_to(leader_position[0], leader_position[1], leader_position[2])

            # 다른 팔로워 드론들과의 거리 확인 및 조정
            for other_uri, other_position in follower_positions.items():
                if scf.cf.link_uri != other_uri:
                    distance_to_other = math.sqrt(sum((c - o) ** 2 for c, o in zip(current_position, other_position)))
                    if distance_to_other < safe_distance:
                        # 다른 드론으로부터 멀어지기
                        follower_phlc.back(0.1)

def mission_phlc(leader_cf, scfs, leader_init_pos, follower_init_poses):
    global follower_phlc
    takeoff_height = 1.0

    # 리더 드론 설정 및 이륙
    with PositionHlCommander(leader_cf, x=leader_init_pos[0], y=leader_init_pos[1], z=leader_init_pos[2]) as leader_phlc:
        leader_phlc.take_off(takeoff_height, 1.0)
        time.sleep(5)
        print('[MISSION]: Leader takeoff complete')

        leader_phlc.forward(1)
        time.sleep(3)
        update_follower_positions(scfs, leader_phlc)

        leader_phlc.land()
        print('[MISSION]: Leader landing')

    for i, scf in enumerate(scfs):
        follower_pos = follower_init_poses[i]
        with PositionHlCommander(scf.cf, x=follower_pos[0], y=follower_pos[1], z=follower_pos[2]) as follower_phlc:
            follower_phlc.take_off(takeoff_height, 1.0)
            time.sleep(5)
            print(f'[MISSION]: Follower {i+1} takeoff complete')

            follower_phlc.land()
            print(f'[MISSION]: Follower {i+1} landing')

def main():
    now = datetime.datetime.now()
    
    # 리더 드론의 초기 위치 입력
    leader_init_pos = list(map(int, input('Start position of leader drone (x, y, z in meters):\n>> ').split()))

    # 팔로워 드론들의 초기 위치 입력
    follower_init_poses = []
    for i in range(len(follower_uris)):
        pos = list(map(int, input(f'Start position of follower drone {i+1} (x, y, z in meters):\n>> ').split()))
        follower_init_poses.append(pos)

    cflib.crtp.init_drivers()

    with SyncCrazyflie(leader_uri, cf=Crazyflie(rw_cache='./cache')) as leader_cf:
        scfs = [SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) for uri in follower_uris]
        try:
            initialize(leader_cf)
            for scf in scfs:
                initialize(scf.cf)

            leader_cf.cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(position_callback)
            logconf.start()

            for scf in scfs:
                follower_cf = scf.cf
                follower_logconf = LogConfig(name='Follower Position', period_in_ms=100)
                follower_logconf.add_variable('stateEstimate.x', 'float')
                follower_logconf.add_variable('stateEstimate.y', 'float')
                follower_logconf.add_variable('stateEstimate.z', 'float')
                follower_cf.log.add_config(follower_logconf)
                uri = follower_cf.link_uri
                follower_logconf.data_received_cb.add_callback(lambda ts, data, lc: follower_position_callback(ts, data, lc, uri))
                follower_logconf.start()

            mission_phlc(leader_cf, scfs, leader_init_pos, follower_init_poses)

            print('[MAIN]: mission complete. Rebooting...')
        except KeyboardInterrupt:
            print('\n\n[MAIN]: EMERGENCY STOP TRIGGERED\n')

if __name__ == '__main__':
    main()
