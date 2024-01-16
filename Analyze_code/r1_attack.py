import time
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.utils.power_switch import PowerSwitch
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

drones = [
    # 'radio://0/80/2M/E7E7E7E70E',
    'radio://0/80/2M/E7E7E7E701',
    'radio://0/80/2M/E7E7E7E702',
    'radio://0/80/2M/E7E7E7E703',
    'radio://0/80/2M/E7E7E7E704',
    'radio://0/80/2M/E7E7E7E70A',
    'radio://0/80/2M/E7E7E7E70B',
    'radio://0/80/2M/E7E7E7E70C',
    # 'radio://0/80/2M/E7E7E7E70C',
]

psws = [PowerSwitch(uri) for uri in drones]

position = [
                         [0, 2.0, 0],
            [-1.4, 1.4, 0],        [1.4, 1.4, 0], 
    [-2.0, 0, 0],        [0, 0, 0],      [2.0, 0, 0],
            [-1.4, -1.4, 0],       [1.4, -1.4, 0],
                         [0, -2.0, 0],
]

param0 = {'d': 0  , 'z': 1.1 }
param1 = {'d': 1.0, 'z': 0.9 }
param2 = {'d': 1.0, 'z': 0.9 }
param3 = {'d': 1.0, 'z': 0.7 }
param4 = {'d': 0  , 'z': 0.7 }
param5 = {'d': 1.0, 'z': 0.7 }
param6 = {'d': 1.0, 'z': 0.5 }
param7 = {'d': 1.0, 'z': 0.5 }
param8 = {'d': 0  , 'z': 0.3 }

params = {
    # drones[0] : [param0],
    drones[0] : [param1],
    drones[1] : [param2],
    drones[2] : [param3],
    drones[3] : [param4],
    drones[4] : [param5],
    drones[5] : [param6],
    drones[6] : [param7],
    # drones[8] : [param8],
}

def poshold(cf, t, z):
    steps = t * 10

    for r in range(steps):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)
        
def run_sequence(scf, params):
    cf = scf.cf

    # Number of setpoints sent per second
    fs = 4
    fsi = 1.0 / fs

    # Compensation for unknown error :-(
    comp = 1.7

    # Base altitude in meters
    base = 0.15

    d = params['d']
    z = params['z']

    poshold(cf, 2, base)

    ramp = fs * 2
    for r in range(ramp):
        cf.commander.send_hover_setpoint(0, 0, 0, base + r * (z - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 2, z)

    for _ in range(2):
        # The time for one revolution
        circle_time = 8

        steps = circle_time * fs
        for _ in range(steps):
            cf.commander.send_hover_setpoint(d * comp * math.pi / circle_time,
                                             0, 360.0 / circle_time, z)
            if _ > (steps // 2):
                param_name = 'imu_sensors.imuPhi'
                value = cf.param.get_value(param_name, _)
                if _ // 2 == 0:
                    cf.param.set_value(param_name, value + 5)
                else :
                    cf.param.set_value(param_name, value - 5)
            time.sleep(fsi)
            

    poshold(cf, 2, z)

    for r in range(ramp):
        cf.commander.send_hover_setpoint(0, 0, 0,
                                         base + (ramp - r) * (z - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 1, base)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(drones, factory=factory) as swarm:
        swarm.reset_estimators()
        swarm.parallel(run_sequence, args_dict=params)
