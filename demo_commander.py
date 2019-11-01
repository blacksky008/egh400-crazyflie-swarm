import sys
from threading import Thread, Lock, Condition

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander

DIST = 0.25
VELOCITY = 0.3

SPEC = [
  ('radio://0/20/2M', DIST, DIST, 0.5),
  ('radio://0/40/2M', DIST, -DIST, 0.7),
  ('radio://0/60/2M', -DIST, -DIST, 0.9),
  ('radio://0/80/2M', -DIST, DIST, 1.1)
]

ROOM_X = 3.3
ROOM_Y = 4.4

FLIGHT_POS = [
  (ROOM_X / 2, ROOM_Y / 2),
  (ROOM_X / 3, ROOM_Y / 2),
  (ROOM_X / 3, ROOM_Y / 3),
  (ROOM_X * 2 / 3, ROOM_Y * 2 / 3),
  (ROOM_X * 2 / 3, ROOM_Y / 3),
  (ROOM_X / 2, ROOM_Y / 3),
  (ROOM_X / 2, ROOM_Y / 2)
]

scfs = []
num_init = 0
init_lock = Lock()
init_condition = Condition()

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

def run_drone(scf, params):
  uri, dx, dy, height = params

  print(uri + ': waiting for estimator to find position...')
  init_x, init_y, init_z = find_initial_position(scf)

  init_lock.acquire()
  num_init += 1

  # wait for all threads to initialise to their position
  if num_init == len(SPEC):
    init_lock.release()
    with cv: cv.notify_all()
  else:
    init_lock.release()
    with cv: cv.wait()

  # enter/exit hlcommander triggers takeoff/land
  with PositionHlCommander(
    scf,
    x=init_x, y=init_y, z=init_z,
    default_velocity=VELOCITY,
    default_height=height,
    controller=PositionHlCommander.CONTROLLER_MELLINGER
  ) as pc:
    for pos in FLIGHT_POS:
      pos.go_to(pos[0] + dx, pos[1] + dy)

def run_flight():
  cflib.crtp.init_drivers(enable_debug_driver=False)
  threads = []

  for spec in SPEC:
    scf = SyncCrazyflie(spec[0], cf=Crazyflie(rw_cache='./cache'))
    scfs.append(scf)

    t = Thread(target=run_drone, args=(scf, spec), daemon=True)
    threads.append(t)
    t.start()

  for t in threads:
    t.join()

if __name__ == '__main__':
  signal.signal(signal.SIGTERM, cancel_flight)
  signal.signal(signal.SIGINT, cancel_flight)
  run_flight()
