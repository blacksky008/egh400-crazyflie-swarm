import sys
import time
from threading import Thread, Lock, Condition

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

scfs = []
num_init = 0
init_lock = Lock()
init_condition = Condition()

CMD_DELAY = 0.01
INIT_TIME = 4
MANUAL_TIME = 3
POST_TIME = 3

HEIGHT = 0.6
LOW_HEIGHT = 0.2
INIT_THRUST = 600

COEFF_1 = 0.8
COEFF_2 = 1.5
COEFF_3 = 1000
COEFF_4 = 500

def cancel_flight():
  print('Cancelling flight')

  for scf in scfs:
    try:
      scf.cf.commander.send_stop_setpoint()
    except Exception as e:
      print(e)

  sys.exit(0)

def find_initial_position(scf):
  log_config = LogConfig(name='Variance', period_in_ms=500)
  log_config.add_variable('kalman.varPX', 'float')
  log_config.add_variable('kalman.varPY', 'float')
  log_config.add_variable('kalman.varPZ', 'float')
  log_config.add_variable('stateEstimate.x', 'float')
  log_config.add_variable('stateEstimate.y', 'float')
  log_config.add_variable('stateEstimate.z', 'float')

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

      if (max_x - min_x) < threshold and
         (max_y - min_y) < threshold and
         (max_z - min_z) < threshold:
        return data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']

def calc_thrust(target, z, vz):
  return int(min(max((target - COEFF_1*z - COEFF_2*vz) * COEFF_3 + COEFF_4, 500), 800))

def run_drone(scf, uri):
  print(uri + ': waiting for estimator to find position...')
  init_x, init_y, init_z = find_initial_position(scf)
  cf = scf.cf
  print('{}: located at ({:.3f}, {:.3f}, {:.3f})'.format(uri, init_x, init_y, init_z))

  log_conf = LogConfig(name='Hover', period_in_ms=CMD_DELAY*1000)
  log_conf.add_variable('stabilizer.thrust', 'uint16_t')
  log_conf.add_variable('stateEstimate.z', 'float')
  log_conf.add_variable('stateEstimate.vz', 'float')

  init_lock.acquire()
  num_init += 1

  # wait for all threads to initialise to their position
  if num_init == len(scfs):
    init_lock.release()
    with cv: cv.notify_all()
  else:
    init_lock.release()
    with cv: cv.wait()

  with SyncLogger(scf, log_conf) as logger:
    start_time = time.time()
    thrust = INIT_THRUST

    for entry in logger:
      data = entry[1]
      z, vz = data['stateEstimate.z'], data['stateEstimate.vz']
      time_diff = time.time() - start_time

      if time_diff < INIT_TIME:
        cf.commander.send_setpoint(0, 0, 0, thrust)
        thrust = calc_thrust(HEIGHT, z, vz)
      elif time_diff < INIT_TIME + MANUAL_TIME:
        thrust = calc_thrust(HEIGHT, z, vz)
        cf.commander.send_setpoint(0, 0, 0, thrust)
      elif time_diff < INIT_TIME + MANUAL_TIME + POST_TIME:
        thrust = calc_thrust(LOW_HEIGHT, z, vz)
        cf.commander.send_setpoint(0, 0, 0, thrust)
      else:
        break

  cf.send_stop_setpoint()

def run_flight():
  cflib.crtp.init_drivers(enable_debug_driver=False)

  available = cflib.crtp.scan_interfaces()
  print('Crazyflies found:')

  for i in available:
    print(i[0])

  if len(available) == 0:
    print('No Crazyflies found, cannot run test')
    return

  threads = []

  for uri in available:
    scf = SyncCrazyflie(uri[0], cf=Crazyflie(rw_cache='./cache'))
    scfs.append(scf)

    t = Thread(target=run_drone, args=(scf, uri[0]), daemon=True)
    threads.append(t)
    t.start()

  for t in threads:
    t.join()

if __name__ == '__main__':
  signal.signal(signal.SIGTERM, cancel_flight)
  signal.signal(signal.SIGINT, cancel_flight)
  run_flight()
