import time
import os
import sys
import signal
import csv

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

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

def wait_for_position_estimator(scf):
  print('Waiting for estimator to find position...')

  log_config = LogConfig(name='Variance', period_in_ms=500)
  log_config.add_variable('kalman.varPX', 'float')
  log_config.add_variable('kalman.varPY', 'float')
  log_config.add_variable('kalman.varPZ', 'float')

  var_y_history = [1000] * 10
  var_x_history = [1000] * 10
  var_z_history = [1000] * 10

  threshold = 0.001

  with SyncLogger(scf, log_config) as logger:
    for log_entry in logger:
      data = log_entry[1]

      var_x_history.append(data['kalman.varPX'])
      var_x_history.pop(0)
      var_y_history.append(data['kalman.varPY'])
      var_y_history.pop(0)
      var_z_history.append(data['kalman.varPZ'])
      var_z_history.pop(0)

      min_x = min(var_x_history)
      max_x = max(var_x_history)
      min_y = min(var_y_history)
      max_y = max(var_y_history)
      min_z = min(var_z_history)
      max_z = max(var_z_history)

      if (max_x - min_x) < threshold and (
          max_y - min_y) < threshold and (
          max_z - min_z) < threshold:
        break

def end_test(sig, frame):
  print('SIGINT/SIGTERM received')

  if cf:
    print('Stopping crazyflie')

    for i in range(int(0.2 * SECOND_STEPS)):
      cf.commander.send_stop_setpoint()
      time.sleep(CMD_DELAY)

  print('Exiting')
  sys.exit(0) 

if __name__ == '__main__':

  signal.signal(signal.SIGTERM, end_test)
  signal.signal(signal.SIGINT, end_test)

  cflib.crtp.init_drivers(enable_debug_driver=False)
  print('Scanning interfaces for Crazyflies...')
  available = cflib.crtp.scan_interfaces()
  print('Crazyflies found:')

  for i in available:
    print(i[0])

  if len(available) == 0:
    print('No Crazyflies found, cannot run test')
  else:
    init_cf = Crazyflie(rw_cache='./cache')

    with SyncCrazyflie('radio://0/100/2M', cf=init_cf) as scf:
      cf = scf.cf
      wait_for_position_estimator(scf)
      home_x, home_y, home_z, home_yaw = find_initial_position(scf)

      for i in range(SECOND_STEPS * 5):
        cf.commander.send_position_setpoint(home_x, home_y, home_z + 0.8, home_yaw)
        time.sleep(CMD_DELAY)

      for i in range(SECOND_STEPS * 5):
        cf.commander.send_position_setpoint(home_x + 0.5, home_y + 0.5, home_z + 0.8, home_yaw)
        time.sleep(CMD_DELAY)

      for i in range(SECOND_STEPS * 4):
        cf.commander.send_position_setpoint(home_x + 0.5, home_y + 0.5, home_z + 0.8-0.9 * (i / float(SECOND_STEPS * 4)), home_yaw)
        time.sleep(CMD_DELAY)

      for i in range(int(SECOND_STEPS * 0.5)):
        cf.commander.send_stop_setpoint()
        time.sleep(CMD_DELAY)


