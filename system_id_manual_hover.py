
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
MAX_HOME_HOVER = 0.6
MAX_LOW_HOVER_HEIGHT = 0.1 
LOW_HOVER_ADDITION = 0.1
# int(min(max((target - HOVER_COEFF_1*z - HOVER_COEFF_2*vz) * HOVER_COEFF_3 + HOVER_COEFF_4, 100), 65535))
HOVER_COEFF_1 = 0.8
HOVER_COEFF_2 = 1.5
HOVER_COEFF_3 = 1000
HOVER_COEFF_4 = 500

# test timing 
INITIAL_HOVER_TIME = 5 # Time for Stage 1: Hovering using commander: cf.commander.send_position_setpoint(home_x, home_y, hover_height, 0)
PRELOG_TIME = 3
DEFAULT_TEST_TIME = 2
REVERSE_TIME = 1
POSTLOG_TIME = 0
FINAL_HIGH_HOVER_TIME = 2
FINAL_LOW_HOVER_TIME = 1

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
    'stabilizer.thrust',
    'stateEstimate.z',
    'stateEstimate.vz'
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
      return data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']


# roll test
def set_roll(cf, value, thrust):
  value = float(value)
  cf.commander.send_setpoint(value, 0, 0, thrust)
  print("Sent: {} (r), {} (p), {} (y), {} (t)".format(value, 0, 0, thrust))
  return [value, 0, 0, thrust]

# pitch test
def set_pitch(cf, value, thrust):
  value = float(value)
  cf.commander.send_setpoint(0, value, 0, thrust)
  print("Sent: {} (r), {} (p), {} (y), {} (t)".format(0, value, 0, thrust))
  return [0, value, 0, thrust]

# yawrate test
def set_yaw(cf, value, thrust):
  value = float(value)
  cf.commander.send_setpoint(0, 0, value, thrust)
  print("Sent: {} (r), {} (p), {} (y), {} (t)".format(0, 0, value, thrust))
  return [0, 0, value, thrust]

def calc_thrust(target, z, vz):
  return int(min(max((target - HOVER_COEFF_1*z - HOVER_COEFF_2*vz) * HOVER_COEFF_3 + HOVER_COEFF_4, 500), 800))

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

    #if not test_complete:
      #print('Removing log file as test incomplete')
      #os.remove(filename)

  print('Exiting')
  sys.exit(0)

if __name__ == '__main__':
  commandflag = True 
  commandflag2 = False 
  # the test needs 3 inputs: test type of {pitch, roll, yaw}, step input value and duration of test
  if len(sys.argv) < 3:
    print('Test type and value required')
    sys.exit(1)
  # to shutdown the test if it failed at the initial stage
  signal.signal(signal.SIGTERM, end_test)
  signal.signal(signal.SIGINT, end_test)
  test_complete = False
  # get the user inputs and display it
  test_name = sys.argv[1]
  value = float(sys.argv[2])
  test_time = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_TEST_TIME

  print("Testing {}, value {}, move time {}".format(test_name, value, test_time))
  # get the test type
  if test_name == "pitch":
    test = set_pitch
  elif test_name == "roll":
    test = set_roll
  elif test_name == "yaw":
    REVERSE_TIME = 0 # disable reversing for yawrate
    POSTLOG_TIME = 0 # disable postlog for yawrate
    test = set_yaw
  # start to connect with the drone
  cflib.crtp.init_drivers(enable_debug_driver=False)
  print('Scanning interfaces for Crazyflies...')
  available = cflib.crtp.scan_interfaces()
  print('Crazyflies found:')

  for i in available:
      print(i[0])

  if len(available) == 0:
      print('No Crazyflies found, cannot run test')
  else:
  # start to define the logger in drone
    log_conf = LogConfig(name='System ID', period_in_ms=LOG_DELAY*1000)

    if LOG_POSITION:
      log_conf.add_variable('stateEstimate.x', 'float')
      log_conf.add_variable('stateEstimate.y', 'float')
      log_conf.add_variable('stateEstimate.z', 'float')
    else:
      log_conf.add_variable('stabilizer.roll', 'float')
      log_conf.add_variable('stabilizer.pitch', 'float')
      log_conf.add_variable('stabilizer.yaw', 'float')
      log_conf.add_variable('stabilizer.thrust', 'uint16_t')
      log_conf.add_variable('stateEstimate.z', 'float')
      log_conf.add_variable('stateEstimate.vz', 'float')
  # initialization of drone
    init_cf = Crazyflie(rw_cache='./cache')
  # make a file to store all of the data
    filename = time.strftime('Ini_test'+'%Y-%m-%d-%H-%M-%S-') + '{}-{}-dur-{}.csv'.format(test_name, value, test_time)
    print('Logging to {}'.format(filename))
    csv_file = open(filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
  # start to fly the drone with Syn Commander
    with SyncCrazyflie(available[0][0], cf=init_cf) as scf:
      cf = scf.cf
  # find the initial point of drone
      wait_for_position_estimator(scf)
      home_x, home_y, home_z = find_initial_position(scf)
  # the hovering height for the first stage should be home_z + addition
      print('Home found: ({}, {}, {})'.format(home_x, home_y, home_z))
      hover_height = max(min(home_z + HOME_HOVER_ADDITION, MIN_HOME_HOVER), MAX_HOME_HOVER)
      low_hover_height = max(home_z + LOW_HOVER_ADDITION, MAX_LOW_HOVER_HEIGHT)
      print('Using hover height {}, low hover height {}'.format(hover_height, low_hover_height))
  # store the hovering setpoints
      if commandflag:
        values_sent = [home_x, home_y, hover_height, 0]
      else:
        values_sent = [0, 0, 0, hover_height]

      cf.param.set_value('flightmode.posSet', '1')
      cf.param.set_value('kalman.resetEstimation', '1')
      time.sleep(2)

      print('Moving to hover...')
      for i in range(SECOND_STEPS * 1):
        cf.commander.send_velocity_world_setpoint(0, 0, 0.6, 0)
        time.sleep(CMD_DELAY)
  # Stage 2: Hovering via Controlling of thrust
      start_time = time.time()
      test_msg = False
      rev_msg = False
      delay_ratio = int(CMD_DELAY / LOG_DELAY)
      hover_thrust = None
  # get the data from the drone logger in real time with some delay
      with SyncLogger(scf, log_conf) as logger:
        counter = 0
        print('Beginning logging')

        for log_entry in logger:
          timestamp = log_entry[0]
          data = [log_entry[1][key] for key in LOG_ORDER]
  # store the sented commander values and the data in real timw
          csv_writer.writerow([timestamp] + values_sent + data)
          print('Received [%d]: %s' % (timestamp, data))

          counter -= 1
          time_diff = time.time() - start_time
          hover_thrust = int(600)
          if counter <= 0:
            if time_diff < PRELOG_TIME:
              cf.commander.send_setpoint(0, 0, 0, hover_thrust)
              hover_thrust = calc_thrust(hover_height, data[4], data[5])
              values_sent = [0, 0, 0, hover_thrust]
  # start to hover via thrust controll und test with step input
            elif time_diff < PRELOG_TIME + test_time:
              if not test_msg:
                test_msg = True
                print('Beginning test')
              hover_thrust = calc_thrust(hover_height, data[4], data[5])
              values_sent = test(cf, value, hover_thrust)
  # Reversing
            elif time_diff < PRELOG_TIME + test_time + REVERSE_TIME:
              if not rev_msg:
                rev_msg = True
                print('Reversing...')
              values_sent = test(cf, -value, hover_thrust)
              hover_thrust = calc_thrust(hover_height, data[4], data[5])
  # ready to stop, auto-hovering take over the controll
            elif time_diff < PRELOG_TIME + test_time + REVERSE_TIME + POSTLOG_TIME:
              if commandflag2:
                cf.commander.send_position_setpoint(0, 0, 0, 0)
              else:
                cf.commander.send_zdistance_setpoint(0, 0, 0, hover_height)
            else:
              print('Stopping logging')
              break

          counter = delay_ratio

      test_complete = True
      print('Lowering drone...')
  # back to initial point
      for i in range(int((FINAL_HIGH_HOVER_TIME - POSTLOG_TIME) * SECOND_STEPS)):
        cf.commander.send_position_setpoint(0, 0, hover_height, 0)
        time.sleep(CMD_DELAY)

      for i in range(FINAL_LOW_HOVER_TIME * SECOND_STEPS):
        cf.commander.send_position_setpoint(0, 0, low_hover_height, 0)
        time.sleep(CMD_DELAY)

      print('Stopping')
      cf.commander.send_stop_setpoint()
      csv_file.close()
