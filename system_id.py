# roll, pitch, yaw units = degrees
# yawrate units = degrees/second
# thrust units = uint16 scale 0-100%

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

CMD_DELAY = 0.01 # delay between crazyflie commands
LOG_DELAY = 0.01 # delay between crazyflie log outputs
LOG_POSITION = False

# home hover parameters 
HOME_HOVER_ADDITION = 0.4 # added to initial detected z
MIN_HOME_HOVER = 0.5
MAX_HOME_HOVER = 1.1
MAX_LOW_HOVER_HEIGHT = 0.3
LOW_HOVER_ADDITION = 0.1

# test timing 
INITIAL_HOVER_TIME = 5
PRELOG_TIME = 1
DEFAULT_TEST_TIME = 2
REVERSE_TIME = 1
POSTLOG_TIME = 0
FINAL_HIGH_HOVER_TIME = 6
FINAL_LOW_HOVER_TIME = 3

# order in which logged values are saved
if LOG_POSITION:
  LOG_ORDER = [
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z'
  ]  
else:
  LOG_ORDER = [
    'stabilizer.roll',
    'stabilizer.pitch',
    'stabilizer.yaw',
    'gyro.z',
    'stabilizer.thrust',
  ]

SECOND_STEPS = int(1 / CMD_DELAY)

# delays for position estimate to become reasonably certain
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

# returns current x, y, z, yaw estimate
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



# roll test
def set_roll(cf, value, hover_height):
  value = float(value)
  cf.commander.send_zdistance_setpoint(value, 0, 0, hover_height)
  print("Sent: {} (r), {} (p), {} (yr), {} (h)".format(value, 0, 0, hover_height))
  return [value, 0, 0, hover_height]

# pitch test
def set_pitch(cf, value, hover_height):
  value = float(value)
  cf.commander.send_zdistance_setpoint(0, value, 0, hover_height)
  print("Sent: {} (r), {} (p), {} (yr), {} (h)".format(0, value, 0, hover_height))
  return [0, value, 0, hover_height]

# yawrate test
def set_yawrate(cf, value, hover_height):
  value = float(value)
  cf.commander.send_zdistance_setpoint(0, 0, value, hover_height)
  print("Sent: {} (r), {} (p), {} (yr), {} (h)".format(0, 0, value, hover_height))
  return [0, 0, value, hover_height]


# shares cf, filename, test_complete, csv_file with main below

def end_test(sig, frame):
  print('SIGINT/SIGTERM received')

  if cf:
    print('Stopping crazyflie')

    for i in range(int(0.2 * SECOND_STEPS)):
      cf.commander.send_stop_setpoint()
      time.sleep(CMD_DELAY)

  if csv_file:
    csv_file.close()

    if not test_complete:
      print('Removing log file as test incomplete')
      os.remove(filename)

  print('Exiting')
  sys.exit(0)

if __name__ == '__main__':
  if len(sys.argv) < 3:
    print('Test type and value required')
    sys.exit(1)

  signal.signal(signal.SIGTERM, end_test)
  signal.signal(signal.SIGINT, end_test)
  test_complete = False

  test_name = sys.argv[1]
  value = float(sys.argv[2])
  test_time = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_TEST_TIME

  print("Testing {}, value {}, move time {}".format(test_name, value, test_time))

  if test_name == "pitch":
    test = set_pitch
  elif test_name == "roll":
    test = set_roll
  elif test_name == "yawrate":
    REVERSE_TIME = 0 # disable reversing for yawrate
    POSTLOG_TIME = 0 # disable postlog for yawrate
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

    if LOG_POSITION:
      log_conf.add_variable('stateEstimate.x', 'float')
      log_conf.add_variable('stateEstimate.y', 'float')
      log_conf.add_variable('stateEstimate.z', 'float')
    else:
      log_conf.add_variable('stabilizer.roll', 'float')
      log_conf.add_variable('stabilizer.pitch', 'float')
      log_conf.add_variable('stabilizer.yaw', 'float')
      log_conf.add_variable('gyro.z', 'float')
      log_conf.add_variable('stabilizer.thrust', 'uint16_t')

    init_cf = Crazyflie(rw_cache='./cache')

    filename = time.strftime('%Y-%m-%d-%H-%M-%S-') + '{}-{}-dur-{}.csv'.format(test_name, value, test_time)
    print('Logging to {}'.format(filename))
    csv_file = open(filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)

    with SyncCrazyflie(available[0][0], cf=init_cf) as scf:
      cf = scf.cf

      wait_for_position_estimator(scf)
      home_x, home_y, home_z, home_yaw = find_initial_position(scf)

      print('Home found: ({}, {}, {})'.format(home_x, home_y, home_z))
      hover_height = max(min(home_z + HOME_HOVER_ADDITION, MIN_HOME_HOVER), MAX_HOME_HOVER)
      low_hover_height = max(home_z + LOW_HOVER_ADDITION, MAX_LOW_HOVER_HEIGHT)
      print('Using hover height {}, low hover height {}'.format(hover_height, low_hover_height))
      values_sent = [0, 0, 0, hover_height]

      cf.param.set_value('flightmode.posSet', '1')

      print('Moving to hover...')

      for i in range(int((INITIAL_HOVER_TIME - PRELOG_TIME) * SECOND_STEPS)):
        cf.commander.send_position_setpoint(home_x, home_y, hover_height, home_yaw)
        time.sleep(CMD_DELAY)

      start_time = time.time()
      test_msg = False
      rev_msg = False
      delay_ratio = int(CMD_DELAY / LOG_DELAY)

      with SyncLogger(scf, log_conf) as logger:
        counter = 0
        print('Beginning logging')

        for log_entry in logger:
          timestamp = log_entry[0]
          data = [log_entry[1][key] for key in LOG_ORDER]
          csv_writer.writerow([timestamp] + values_sent + data)
          print('Received [%d]: %s' % (timestamp, data))

          counter -= 1
          time_diff = time.time() - start_time

          if counter <= 0:
            if time_diff < PRELOG_TIME:
              cf.commander.send_position_setpoint(home_x, home_y, hover_height, home_yaw)
            elif time_diff < PRELOG_TIME + test_time:
              if not test_msg:
                test_msg = True
                print('Beginning test')

              values_sent = test(cf, value, hover_height)
            elif time_diff < PRELOG_TIME + test_time + REVERSE_TIME:
              if not rev_msg:
                rev_msg = True
                print('Reversing...')

              values_sent = test(cf, -value, hover_height)
            elif time_diff < PRELOG_TIME + test_time + REVERSE_TIME + POSTLOG_TIME:
              cf.commander.send_position_setpoint(home_x, home_y, hover_height, home_yaw)
            else:
              print('Stopping logging')
              break

          counter = delay_ratio

      test_complete = True
      print('Lowering drone...')

      for i in range(int((FINAL_HIGH_HOVER_TIME - POSTLOG_TIME) * SECOND_STEPS)):
        cf.commander.send_position_setpoint(home_x, home_y, hover_height, home_yaw)
        time.sleep(CMD_DELAY)

      for i in range(FINAL_LOW_HOVER_TIME * SECOND_STEPS):
        cf.commander.send_position_setpoint(home_x, home_y, low_hover_height, home_yaw)
        time.sleep(CMD_DELAY)

      print('Stopping')
      cf.commander.send_stop_setpoint()
      csv_file.close()
