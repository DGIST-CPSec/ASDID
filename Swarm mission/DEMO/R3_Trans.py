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
    [-0.5, 0.0, 0.0],  # 왼쪽 1번 드론
    [-0.5, 1.0, 0.0],  # 왼쪽 2번 드론
    [-0.5, 2.0, 0.0],  # 왼쪽 3번 드론
    [-0.5, 3.0, 0.0],  # 왼쪽 4번 드론
    [0.5, 0.0, 0.0],   # 오른쪽 1번 드론
    [0.5, 1.0, 0.0],   # 오른쪽 2번 드론
    [0.5, 2.0, 0.0],   # 오른쪽 3번 드론
    [0.5, 3.0, 0.0],   # 오른쪽 4번 드론
]

moveDelta = [
    [1.0, 0.5, 0.0],  # 왼쪽 1번 드론 오른쪽으로 이동
    [1.0, 0.5, 0.0],  # 왼쪽 2번 드론 오른쪽으로 이동
    [1.0, 0.5, 0.0],  # 왼쪽 3번 드론 오른쪽으로 이동
    [1.0, 0.5, 0.0],  # 왼쪽 4번 드론 오른쪽으로 이동
    [-1.0, -0.5, 0.0], # 오른쪽 1번 드론 왼쪽으로 이동
    [-1.0, -0.5, 0.0], # 오른쪽 2번 드론 왼쪽으로 이동
    [-1.0, -0.5, 0.0], # 오른쪽 3번 드론 왼쪽으로 이동
    [-1.0, -0.5, 0.0], # 오른쪽 4번 드론 왼쪽으로 이동
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


def mission(scf: SyncCrazyflie, posNo, code):
    takeoff_height = 1.0
    # logconf.start()
    if code == 0:
        time.sleep(3)
        print('[MISSION]: begin to blink')
        for i in range(20):
            scf.cf.param.set_value('led.bitmask', 255)
            time.sleep(0.5)
            scf.cf.param.set_value('led.bitmask', 0)
            time.sleep(0.5)
            

    elif code == 2:  # 새로운 미션 유형
        phlc = PositionHlCommander(scf, 
                                   x=initialPos[posNo][0], 
                                   y=initialPos[posNo][1], 
                                   z=initialPos[posNo][2])
        phlc.take_off(1.0, 1.0)
        time.sleep(2)

        # 위치 교차 이동
        phlc.move_distance(moveDelta[posNo][0], 
                           moveDelta[posNo][1], 
                           moveDelta[posNo][2])
        time.sleep(2)

        phlc.land()
          
    else:
        phlc = PositionHlCommander(scf, 
                                x = initialPos[posNo][0], 
                                y = initialPos[posNo][1], 
                                z = initialPos[posNo][2]
                                )
        phlc.take_off(takeoff_height, 1.0)
        time.sleep(5)
        print(f'[{scf.cf.link_uri}]: takeoff complete')
        phlc.set_default_height(takeoff_height)
        phlc.set_default_velocity(0.5)
        phlc.set_landing_height(0.0)

        if code == 1:
            phlc.up(0.5)
            time.sleep(3)
            phlc.down(0.5)
            time.sleep(3)

        else:
            time.sleep(6)
        print(f'[{scf.cf.link_uri}]: mission complete')
        phlc.land()
        print(f'[{scf.cf.link_uri}]: landing')
    # logconf.stop()
    # logFile.close()

    # logFile.close()

if __name__ == '__main__':
    now = datetime.datetime.now()
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(drones, factory=factory) as swarm:
        print("Connected to Swarm CFs")
        """ swarm.reset_estimators()
        print('Estimators reset') """
        # logpath ='./swarm/'+str(now)[5:10]+'/' 
        # if not os.path.exists(logpath):
            # os.mkdir(logpath)
        try:
            swarm.parallel_safe(mission, args_dict=arguments)
        except KeyboardInterrupt:
            print('EMERGENCY STOP TRIGGERED')
            for i in psws:
                i.platform_power_down()
                print('['+str(hex(i.address[4]))+']: SHUTDOWN')

