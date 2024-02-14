import time
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

# 드론 URI 설정
URIS = [
    'radio://0/80/2M/E7E7E7E701', # Leader
    'radio://0/80/2M/E7E7E7E702',
    'radio://0/80/2M/E7E7E7E703',
    'radio://0/80/2M/E7E7E7E704',
]

# 각 드론의 시작 위치 설정
# 예시 값, 실제 환경에 맞게 조정 필요
initial_positions = {
    URIS[0]: (0, 0, 0.5),
    URIS[1]: (-0.5, 0, 0.5),
    URIS[2]: (0.5, 0, 0.5),
    URIS[3]: (0, -0.5, 0.5),
}

# Follower 드론이 유지해야 하는 거리
FOLLOW_DISTANCE = 0.5

def position_callback(timestamp, data, logconf):
    # Leader의 현재 위치를 업데이트하는 콜백 함수
    # 여기서는 단순화를 위해 구현하지 않음
    pass

def simple_swarm_sequence(scf):
    cf = scf.cf

    # Log 설정으로 드론의 위치 데이터 추적
    logconf = LogConfig(name='Position', period_in_ms=100)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')

    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(position_callback)

    logconf.start()

    # Leader 드론의 경우, 임의의 경로로 이동
    # Follower 드론의 경우, Leader를 따라가도록 구현
    # 여기서는 단순화를 위해 구현하지 않음

    logconf.stop()

if __name__ == '__main__':
    swarm = Swarm(URIS)
    swarm.parallel(simple_swarm_sequence)
