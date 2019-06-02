import time
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

METHOD = 1
MOVE_TIME = 1
CMD_DELAY = 0.1
LOG_DELAY = 0.01
HOVER_HEIGHT = 0.5
HOVER_THRUST = 26000
MAX_HOVER_THRUST = 30000

def wait_for_position_estimator(scf):
  print('Waiting for estimator to find position...')

  log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
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

      # print("{} {} {}".
      #       format(max_x - min_x, max_y - min_y, max_z - min_z))

      if (max_x - min_x) < threshold and (
          max_y - min_y) < threshold and (
          max_z - min_z) < threshold:
        break

if METHOD == 1:
  def set_roll(cf, value):
    value = float(value)
    cf.commander.send_setpoint(value, 0, 0, HOVER_THRUST)
    print("Sent: %d, %d, %d (y), %d (t)", value, 0, 0, HOVER_THRUST)

  def set_pitch(cf, value):
    value = float(value)
    cf.commander.send_setpoint(0, value, 0, HOVER_THRUST)
    print("Sent: %d, %d, %d (y), %d (t)", 0, value, 0, HOVER_THRUST)
else:
  def set_roll(cf, value):
    value = float(value)
    cf.commander.send_zdistance_setpoint(value, 0, 0, HOVER_HEIGHT)
    print("Sent: %d, %d, %d (yr), %d (h)", value, 0, 0, HOVER_HEIGHT)

  def set_pitch(cf, value):
    value = float(value)
    cf.commander.send_zdistance_setpoint(0, value, 0, HOVER_HEIGHT)
    print("Sent: %d, %d, %d (yr), %d (h)", 0, value, 0, HOVER_HEIGHT)

def set_yawrate(cf, value):
  value = float(value)
  cf.commander.send_zdistance_setpoint(0, 0, value, HOVER_HEIGHT)
  print("Sent: %d, %d, %d (yr), %d (h)", 0, 0, value, HOVER_HEIGHT)


if __name__ == '__main__':
  if len(sys.argv) < 2:
    print('Test required')
    sys.exit(1)

  test = sys.argv[1]
  value = sys.argv[2] if len(sys.argv) > 2 else 0.1

  if test == "pitch":
    test = set_pitch
  elif test == "roll":
    test = set_roll
  elif test == "yawrate":
    test = set_yawrate

  cflib.crtp.init_drivers(enable_debug_driver=False)
  print('Scanning interfaces for Crazyflies...')
  available = cflib.crtp.scan_interfaces()
  print('Crazyflies found:')
  for i in available:
      print(i[0])

  if len(available) == 0:
      print('No Crazyflies found, cannot run test')
  else:
    log_conf = LogConfig(name='System ID', period_in_ms=LOG_DELAY*1000)
    log_conf.add_variable('stabilizer.roll', 'float')
    log_conf.add_variable('stabilizer.pitch', 'float')
    log_conf.add_variable('stabilizer.yaw', 'float')
    log_conf.add_variable('gyro.z', 'float')

    cf = Crazyflie(rw_cache='./cache')

    with SyncCrazyflie(available[0][0], cf=cf) as scf:
      wait_for_position_estimator(scf)
      cf = scf.cf

      print('Moving to hover...')

      if METHOD == 1:
        steps = int(1 / CMD_DELAY)
        thrust = 0

        for i in range(steps):
          thrust = min(thrust + (MAX_HOVER_THRUST / steps), MAX_HOVER_THRUST)
          cf.commander.send_setpoint(0, 0, 0, thrust)
          time.sleep(CMD_DELAY)

        print('Reached launch thrust')
        time.sleep(0.5)
        print('Launch complete, delaying to stabilise hover...')

        for i in range(5):
          cf.commander.send_setpoint(0, 0, 0, HOVER_THRUST)
          time.sleep(CMD_DELAY)
      else:
        steps = int(1 / CMD_DELAY)
        height = 0

        for i in range(steps):
          height = min(height + (HOVER_HEIGHT / steps), HOVER_HEIGHT)
          cf.commander.send_zdistance_setpoint(0, 0, 0, height)
          time.sleep(CMD_DELAY)

        print('Delaying to stabilise hover...')

        for i in range(5):
          cf.commander.send_zdistance_setpoint(0, 0, 0, HOVER_HEIGHT)

      with SyncLogger(scf, log_conf) as logger:
        counter = 0
        print('Beginning test')

        for log_entry in logger:
          timestamp = log_entry[0]
          data = log_entry[1]
          logconf_name = log_entry[2]

          print('Received [%d][%s]: %s' % (timestamp, logconf_name, data))

          counter -= 1

          if counter <= 0:
            test(cf, value)
            counter = int(CMD_DELAY / LOG_DELAY) 


