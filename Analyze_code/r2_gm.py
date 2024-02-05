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
lader = 'radio://0/80/2M/E7E7E7E701'

drones = [
    'radio://0/80/2M/E7E7E7E702',
    'radio://0/80/2M/E7E7E7E703',
    'radio://0/80/2M/E7E7E7E704',
    'radio://0/80/2M/E7E7E7E70A',
    'radio://0/80/2M/E7E7E7E70B',
    'radio://0/80/2M/E7E7E7E70C',
]

psws = [PowerSwitch(drone) for drone in drones]


initial_pos = [0, 0, 0] #수정 필요

def leader_move(scf, code, pos):
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
        
def followers_mission(scf):
    
