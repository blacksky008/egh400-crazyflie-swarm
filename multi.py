import time
import os
import sys
import signal

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

CMD_DELAY = 0.1
SECOND_STEPS = int(1 / CMD_DELAY)

def find_initial_position(scf):
  print('Finding intial position and state')
  log_config = LogConfig(name='Variance', period_in_ms=100)
  log_config.add_variable('stateEstimate.x', 'float')
  log_config.add_variable('stateEstimate.y', 'float')
  log_config.add_variable('stateEstimate.z', 'float')
  log_config.add_variable('stabilizer.yaw', 'float')

  with SyncLogger(scf, log_config) as logger:
    for log_entry in logger:
      data = log_entry[1]
      return data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'], data['stabilizer.yaw']


def reset_estimator(scf):
  cf = scf.cf
  cf.param.set_value('kalman.resetEstimation', '1')
  time.sleep(0.1)
  cf.param.set_value('kalman.resetEstimation', '0')
  time.sleep(2)

def run_sequence(scf):
  cf = scf.cf

  #home_x, home_y, home_z, home_yaw = 0, 0, 0, 0
  home_x, home_y, home_z, home_yaw = find_initial_position(scf)

  for i in range(SECOND_STEPS * 5):
    cf.commander.send_position_setpoint(home_x, home_y, home_z + 0.8, home_yaw)
    time.sleep(CMD_DELAY)

  for i in range(SECOND_STEPS * 5):
    cf.commander.send_position_setpoint(home_x + 0.5, home_y + 0.5, home_z + 0.8, home_yaw)
    time.sleep(CMD_DELAY)

  for i in range(SECOND_STEPS * 4):
    cf.commander.send_position_setpoint(home_x + 0.5, home_y + 0.5, home_z + 0.8-0.8 * (i / float(SECOND_STEPS * 4)), home_yaw)
    time.sleep(CMD_DELAY)

  for i in range(int(SECOND_STEPS * 0.5)):
    cf.commander.send_stop_setpoint()
    time.sleep(CMD_DELAY)

if __name__ == '__main__':
  cflib.crtp.init_drivers(enable_debug_driver=False)
  #print('Scanning interfaces for Crazyflies...')
  #available = cflib.crtp.scan_interfaces()
  #print('Crazyflies found:')
  uris = [
    # 'radio://0/80/2M',
    # 'radio://0/60/2M',
    'radio://0/100/2M'
  ]

  #for i in available:
  #  print(i[0])
  #  uris.append(i[0])

  factory = CachedCfFactory(rw_cache='./cache')
  with Swarm(uris, factory=factory) as swarm:
    #swarm.parallel(reset_estimator)
    swarm.parallel(run_sequence)
