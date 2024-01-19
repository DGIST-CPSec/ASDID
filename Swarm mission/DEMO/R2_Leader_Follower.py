import time, datetime, os

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.power_switch import PowerSwitch
from cflib.positioning.position_hl_commander import PositionHlCommander

from cflib.crazyflie.swarm import Swarm, CachedCfFactory

drones = [
    'radio://0/80/2M/E7E7E7E70A',
    'radio://0/80/2M/E7E7E7E70B',
    'radio://0/80/2M/E7E7E7E70C',
    'radio://0/80/2M/E7E7E7E70D',
    'radio://0/80/2M/E7E7E7E70E',
    'radio://0/80/2M/E7E7E7E701',
    'radio://0/80/2M/E7E7E7E702',
    'radio://0/80/2M/E7E7E7E703',
]

psws = [PowerSwitch(drone) for drone in drones]

initialPos = [
    [-0.7, 0.7, 0], [0.7, 0.7, 0],
    [-1.0, 0, 0], [0, 0, 0], [1.0, 0, 0],
    [-0.7, -0.7, 0], [0.7, -0.7, 0],
]

moveDelta = [
    [1.0, 0.0, 0.0], # 각 드론을 앞으로 1미터 이동
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
]

missNo    = int(input('Mission type: \n - [0]Blink \n - [1]In-position \n - [2]Line \n - [6]Up-down\n >>'))

arguments = {
    drones[0] : [0, missNo],
    drones[1] : [1, missNo],
    drones[2] : [2, missNo],
    drones[3] : [3, missNo],
    drones[4] : [4, missNo],
    drones[5] : [5, missNo],
    drones[6] : [6, missNo],
    drones[7] : [7, missNo],
}

miss_type = ['NO', 'TR']
drone_ID = ['0A', '0B', '0C', '0D', '0E', '0F', '0G', '0H']

leader_x, leader_y, leader_z = 0.0, 0.0, 0.0
prev_leader_x, prev_leader_y, prev_leader_z = 0.0, 0.0, 0.0

leader_logconf = LogConfig(name='Leader Position', period_in_ms=100)
leader_logconf.add_variable('stateEstimate.x', 'float')
leader_logconf.add_variable('stateEstimate.y', 'float')
leader_logconf.add_variable('stateEstimate.z', 'float')

def leader_position_callback(timestamp, data, logconf):
    global leader_x, leader_y, leader_z
    global prev_leader_x, prev_leader_y, prev_leader_z

    # 리더 드론의 현재 위치 업데이트
    leader_x = data['stateEstimate.x']
    leader_y = data['stateEstimate.y']
    leader_z = data['stateEstimate.z']

    # 리더 드론의 이동 거리 및 방향 계산
    delta_x = leader_x - prev_leader_x
    delta_y = leader_y - prev_leader_y
    delta_z = leader_z - prev_leader_z

    # 팔로워 드론들에게 리더 드론의 이동 정보 전달
    for follower_uri in follower_drones:
        move_follower_drone(follower_uri, delta_x, delta_y, delta_z)

    # 이전 위치 업데이트
    prev_leader_x, prev_leader_y, prev_leader_z = leader_x, leader_y, leader_z
    
# 팔로워 드론을 이동시키는 함수
def move_follower_drone(uri, delta_x, delta_y, delta_z):
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        phlc = PositionHlCommander(scf)
        phlc.move_distance(delta_x, delta_y, delta_z)
    
def mission(scf: SyncCrazyflie, posNo, code):
    takeoff_height = 1.0

    if scf.cf.link_uri == leader_drone:  # 리더 드론일 경우에만 동작
        phlc = PositionHlCommander(scf, 
                                   x=initialPos[posNo][0], 
                                   y=initialPos[posNo][1], 
                                   z=initialPos[posNo][2])
        phlc.take_off(1.0, 1.0)
        time.sleep(2)

        # 리더 드론을 정의된 대로 앞으로 1미터 이동
        phlc.move_distance(moveDelta[posNo][0], 
                           moveDelta[posNo][1], 
                           moveDelta[posNo][2])
        time.sleep(2)

        phlc.land()

leader_drone = None

def main():
    global leader_drone, follower_drones
    cflib.crtp.init_drivers()

    # 드론 URI와 위치 매핑
    drone_positions = dict(zip(drones, initialPos))

    # 리더 및 팔로워 드론 리스트 초기화
    leader_drone = None
    follower_drones = []

    # 각 드론을 리더 또는 팔로워로 분류
    for drone_uri, pos in drone_positions.items():
        if pos == [0.0, 0.0, 0.0]:
            leader_drone = drone_uri
        else:
            follower_drones.append(drone_uri)

    factory = CachedCfFactory(rw_cache='./cache')

    with SyncCrazyflie(leader_drone, cf=Crazyflie(rw_cache='./cache')) as leader_cf:
        leader_cf.cf.log.add_config(leader_logconf)
        leader_logconf.data_received_cb.add_callback(leader_position_callback)
        leader_logconf.start()

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
