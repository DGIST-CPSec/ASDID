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
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.utils.power_switch import PowerSwitch
from cflib.crazyflie.syncLogger import SyncLogger


leader = 'radio://0/80/2M/E7E7E7E701'

drones = [
    'radio://0/80/2M/E7E7E7E701'
    'radio://0/80/2M/E7E7E7E702',
    'radio://0/80/2M/E7E7E7E703',
    'radio://0/80/2M/E7E7E7E704',
    'radio://0/80/2M/E7E7E7E70A',
    'radio://0/80/2M/E7E7E7E70B',
    'radio://0/80/2M/E7E7E7E70C',
]


psws = [PowerSwitch(drone) for drone in drones]


initial_pos = [0, 0, 0] #수정 필요

distance = [] # 수정필요

def order_follow(timestamp, data, logconf):
    print(timestamp,":", data)
    for i in range(len(drones)):
        retpos = []
        retpos.append(leader[0] + distance[i][0])
        retpos.append(leader[1] + distance[i][1])
        retpos.append(max(0, leader[2] + distance[i][2]))
        hlcs[i].go_to(retpos[0], retpos[1], retpos[2], 0, 0.1, relative=False)
    time.sleep(0.5)

def leader_mission(scf, code, pos):
    uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')
    takeoff_height = 1.0
    phlc = PositionHlCommander(scf, x = pos[0], y= pos[1], z = pos[2])
    phlc.take_off(takeoff_height, 1.0)
    time.sleep(5)
    print('[MISSION]: takeoff complete')
    # DEFAULT = 1.0
    phlc.set_default_height(1.0)
    phlc.set_default_velocity(0.5)
    phlc.set_landing_height(0.0)
    if code == 1:
        phlc.up(0.5)
        time.sleep(4)
        phlc.down(0.5)
        time.sleep(4)
    elif code == 2: # linear round trip
        phlc.forward(1)
        time.sleep(4)
        phlc.back(1)
        time.sleep(4)
    elif code == 3: # triangular round trip
        phlc.go_to( 0.0, 1.2)
        time.sleep(4)
        phlc.go_to(-1.2, 1.2)
        time.sleep(4)
        phlc.go_to( 0.0, 0.0)
        time.sleep(4)
    elif code == 4: # square round trip
        phlc.go_to( 0.6,  0.6)
        time.sleep(3)
        phlc.go_to( 0.6, -0.6)
        time.sleep(3)
        phlc.go_to(-0.6, -0.6)
        time.sleep(3)
        phlc.go_to(-0.6,  0.6)
        time.sleep(3)
        phlc.go_to( 0.0,  0.0)
        time.sleep(3)
    elif code == 5: # circular round trip
        with MotionCommander(scf, default_height=takeoff_height) as mc:
            mc.circle_left(0.5, 0.4)
            time.sleep(10)
    else:
        time.sleep(5) # up-down
        
def synchronous_mission(scf, logconf):
    with SyncLogger(scf, logconf) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            order_follow(timestamp, data, logconf)
            break

def asynchronous_mission(scf, logconf, mission_code):
    try:
        cf = scf.cf
        cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(order_follow)
        logconf.start()
        print('logconf start')
        leader_mission(scf, mission_code)
        logconf.stop()
        print('logconf stop')
    except KeyboardInterrupt:
        triggerRestart()

def triggerRestart():
    for i in range(len(drones)):
        psws[i].reboot_to_fw()
        print('[MAIN]: EMERGENCY STOP TRIGGERED\n['+str(hex(psws[i].address[4]))+']: Restarting...')
        scfs[i].close_link()
    
    
if __name__ == '__main__':
    try:
        cflib.crtp.init_drivers()
        print("initialized")

        scfs = [SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) for uri in drones]
        hlcs = [scf.cf.high_level_commander for scf in scfs]
        
        for i in range(len(scfs)):
            scfs[i].open_link()
            print("[SWARM] opened link: ", scfs[i].cf.link_uri)

        mission = int(input('Enter mission code: [1] up-down, [2] forward-backward, [3] left-right, [4] square, [0] hold: \n>>> '))
        logconf = LogConfig(name='Position', period_in_ms=100)
        logconf.add_variable('kalman.stateX', 'float')
        logconf.add_variable('kalman.stateY', 'float')
        logconf.add_variable('kalman.stateZ', 'float')
        leader_scf = scfs[0]

        print('[MAIN]: starting mission')
        for drone in hlcs:
            drone.takeoff(0.5, 3.0)
        time.sleep(5)
        print('[MISSION]: takeoff complete')

        asynchronous_mission(leader_scf, logconf, mission)
        for drone in hlcs:
            drone.land(0.0, 2.0)
        time.sleep(10)
        for i in range(len(psws)):
            psws[i].reboot_to_fw()
            scfs[i].close_link()
        print("[MAIN]: MISSION COMPLETE\n")

    except KeyboardInterrupt:
        triggerRestart()
