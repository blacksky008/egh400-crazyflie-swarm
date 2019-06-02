# yawrate units = degrees/second

import time
import sys
import csv

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

MOVE_TIME = 2
CMD_DELAY = 0.01
LOG_DELAY = 0.01
HOVER_HEIGHT = 0.8

HOME_X = 1.5
HOME_Y = 0

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

def set_roll(cf, value):
  value = float(value)
  cf.commander.send_zdistance_setpoint(value, 0, 0, HOVER_HEIGHT)
  print("Sent: {} (r), {} (p), {} (yr), {} (h)".format(value, 0, 0, HOVER_HEIGHT))

def set_pitch(cf, value):
  value = float(value)
  cf.commander.send_zdistance_setpoint(0, value, 0, HOVER_HEIGHT)
  print("Sent: {} (r), {} (p), {} (yr), {} (h)".format(0, value, 0, HOVER_HEIGHT))

def set_yawrate(cf, value):
  value = float(value)
  cf.commander.send_zdistance_setpoint(0, 0, value, HOVER_HEIGHT)
  print("Sent: {} (r), {} (p), {} (yr), {} (h)".format(0, 0, value, HOVER_HEIGHT))


if __name__ == '__main__':
  if len(sys.argv) < 2:
    print('Test required')
    sys.exit(1)

  test = sys.argv[1]
  value = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05

  if test == "pitch":
    test = set_pitch
    values_sent = [0, value, 0, HOVER_HEIGHT]
  elif test == "roll":
    test = set_roll
    values_sent = [value, 0, 0, HOVER_HEIGHT]
  elif test == "yawrate":
    test = set_yawrate
    values_sent = [0, 0, value, HOVER_HEIGHT]

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
    log_conf.add_variable('stabilizer.thrust', 'uint16_t')
    cf = Crazyflie(rw_cache='./cache')

    with open(time.strftime('%Y-%m-%d-%H-%M-%S.csv'), 'w', newline='') as file:
      csv_writer = csv.writer(file)
      csv_writer.writerow([
        'Roll: {}'.format(values_sent[0]),
        'Pitch: {}'.format(values_sent[1]),
        'Yawrate: {}'.format(values_sent[2]),
        'Hover height: {}'.format(values_sent[3])
      ])

      with SyncCrazyflie(available[0][0], cf=cf) as scf:
        wait_for_position_estimator(scf)
        cf = scf.cf
        cf.param.set_value('flightmode.posSet', '1')
        print('Moving to hover...')

        steps = int(1 / CMD_DELAY)
        height = 0

        for i in range(3* steps):
          height = min(height + (HOVER_HEIGHT / steps), HOVER_HEIGHT)
          #cf.commander.send_zdistance_setpoint(0, 0, 0, height)
          cf.commander.send_position_setpoint(HOME_X, HOME_Y, height, 0)
          time.sleep(CMD_DELAY)

        print('Delaying to stabilise hover...')

        for i in range(int(.5 / CMD_DELAY)):
          cf.commander.send_position_setpoint(HOME_X, HOME_Y, HOVER_HEIGHT, 0)
          #cf.commander.send_zdistance_setpoint(0, 0, 0, HOVER_HEIGHT)
          time.sleep(CMD_DELAY)

        start_time = time.time()

        with SyncLogger(scf, log_conf) as logger:
          counter = 0
          print('Beginning test')

          for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            csv_writer.writerow([timestamp] + list(data.values()))
            print('Received [%d]: %s' % (timestamp, data))

            counter -= 1

            if counter <= 0:
              #test(cf, value)
              cf.commander.send_position_setpoint(HOME_X, HOME_Y, HOVER_HEIGHT, 0)
              #cf.commander.send_zdistance_setpoint(0, 0, 0, HOVER_HEIGHT)
              counter = int(CMD_DELAY / LOG_DELAY)

            if (time.time() - start_time) >= MOVE_TIME:
                break

        print('Reversing...')

        #for i in range(steps):
        #    test(cf, -value)
        #    time.sleep(CMD_DELAY)

        print('Lowering drone...')

        for i in range(4 * steps):
          cf.commander.send_position_setpoint(HOME_X, HOME_Y, HOVER_HEIGHT, 0)
          #cf.commander.send_zdistance_setpoint(0, 0, 0, HOVER_HEIGHT)
          time.sleep(CMD_DELAY)

        for i in range(1 * steps):
          cf.commander.send_position_setpoint(HOME_X, HOME_Y, 0.2, 0)
          #cf.commander.send_zdistance_setpoint(0, 0, 0, height)
          time.sleep(CMD_DELAY)

        print('Stopping')
        cf.commander.send_stop_setpoint()
