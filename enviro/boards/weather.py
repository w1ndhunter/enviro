import time, math, os
from breakout_bme280 import BreakoutBME280
from breakout_ltr559 import BreakoutLTR559
from machine import Pin, PWM
from pimoroni import Analog
from enviro import i2c, activity_led
import enviro.helpers as helpers
from phew import logging
from enviro.constants import WAKE_REASON_RTC_ALARM, WAKE_REASON_BUTTON_PRESS

# amount of rain required for the bucket to tip in mm
RAIN_MM_PER_TICK = 0.2794

# distance from the centre of the anemometer to the centre 
# of one of the cups in cm
WIND_CM_RADIUS = 7.0
# scaling factor for wind speed in m/s
WIND_FACTOR = 0.0218

# debounce threshold for spurious, too-fast tick intervals (ms)
# values below this are treated as contact bounce/noise rather than real rotations
WIND_BOUNCE_MS = 5

# hard cap for physically plausible wind speeds (m/s) to reject spikes
WIND_MAX_M_S_CAP = 75.0

# default averaging window for wind direction (ms)
WIND_DIRECTION_AVG_WINDOW_MS = 5000

# average fastest N intervals for gust to reduce single-sample spikes
WIND_GUST_FASTEST_SAMPLES = 3

# minimum normalized resultant vector length for direction average confidence
WIND_DIRECTION_MIN_RESULTANT = 0.2


def _wind_direction_resultant_from_bins(bin_indices):
  """Mean resultant length (0..1) for discrete 45° sector indices 0..7."""
  if not bin_indices:
    return 0.0
  n = len(bin_indices)
  sum_x = 0.0
  sum_y = 0.0
  for i in bin_indices:
    rad = math.radians(i * 45)
    sum_x += math.cos(rad)
    sum_y += math.sin(rad)
  return math.sqrt((sum_x * sum_x) + (sum_y * sum_y)) / n


def _circular_median_bin(bin_indices):
  """Circular median on the 8-sector ring: minimizes sum of shortest arc distances in steps."""
  if not bin_indices:
    return 0
  best_cost = None
  best_ks = None
  for k in range(8):
    cost = 0
    for i in bin_indices:
      d = abs(i - k)
      cost += min(d, 8 - d)
    if best_cost is None or cost < best_cost:
      best_cost = cost
      best_ks = [k]
    elif cost == best_cost:
      best_ks.append(k)
  return min(best_ks, key=lambda kb: (-bin_indices.count(kb), kb))


bme280 = BreakoutBME280(i2c, 0x77)
ltr559 = BreakoutLTR559(i2c)

wind_direction_pin = Analog(26)
wind_speed_pin = Pin(9, Pin.IN, Pin.PULL_UP)
rain_pin = Pin(10, Pin.IN, Pin.PULL_DOWN)
last_rain_trigger = False

def startup(reason):
  global last_rain_trigger
  import wakeup

  # check if rain sensor triggered wake
  rain_sensor_trigger = wakeup.get_gpio_state() & (1 << 10)

  if rain_sensor_trigger:
    # read the current rain entries
    rain_entries = []
    if helpers.file_exists("rain.txt"):
      with open("rain.txt", "r") as rainfile:
        rain_entries = rainfile.read().split("\n")

    # add new entry
    logging.info(f"> add new rain trigger at {helpers.datetime_string()}")
    rain_entries.append(helpers.datetime_string())

    # limit number of entries to 190 - each entry is 21 bytes including
    # newline so this keeps the total rain.txt filesize just under one
    # filesystem block (4096 bytes)
    rain_entries = rain_entries[-190:]

    # write out adjusted rain log
    with open("rain.txt", "w") as rainfile:
      rainfile.write("\n".join(rain_entries))

    last_rain_trigger = True

    # if we were woken by the RTC or a Poke continue with the startup
    return (reason is WAKE_REASON_RTC_ALARM 
      or reason is WAKE_REASON_BUTTON_PRESS)

  # there was no rain trigger so continue with the startup
  return True

def check_trigger():
  global last_rain_trigger
  rain_sensor_trigger = rain_pin.value()

  if rain_sensor_trigger and not last_rain_trigger:
    activity_led(100)
    time.sleep(0.05)
    activity_led(0)

    # read the current rain entries
    rain_entries = []
    if helpers.file_exists("rain.txt"):
      with open("rain.txt", "r") as rainfile:
        rain_entries = rainfile.read().split("\n")

    # add new entry
    logging.info(f"> add new rain trigger at {helpers.datetime_string()}")
    rain_entries.append(helpers.datetime_string())

    # limit number of entries to 190 - each entry is 21 bytes including
    # newline so this keeps the total rain.txt filesize just under one 
    # filesystem block (4096 bytes)
    rain_entries = rain_entries[-190:]

    # write out adjusted rain log
    with open("rain.txt", "w") as rainfile:
      rainfile.write("\n".join(rain_entries))

  last_rain_trigger = rain_sensor_trigger

def wind_speed(sample_time_ms=40000):
  # get initial sensor state
  state = wind_speed_pin.value()

  # create an array for each sensor to log the times when the sensor state changed
  # then we can use those values to calculate an average tick time for each sensor
  ticks = []

  start = time.ticks_ms()
  while time.ticks_diff(time.ticks_ms(), start) <= sample_time_ms:
    now = wind_speed_pin.value()
    if now != state: # sensor output changed
      # record the time of the change and update the state
      ticks.append(time.ticks_ms())
      state = now

  logging.info(f"ticks: {ticks}")

  # Filter out duplicate ticks that may occur due to noise/bouncing
  filtered_ticks = []
  for tick in ticks:
    if not filtered_ticks or tick != filtered_ticks[-1]:
      filtered_ticks.append(tick)
  ticks = filtered_ticks

  logging.info(f"filtered_ticks: {filtered_ticks}")

  # if no sensor connected then we have no readings, skip
  if len(ticks) < 2:
    return 0, 0, 0

  # build list of intervals between ticks (ms)
  tick_diffs = []
  for i in range(1, len(ticks)):
    diff = time.ticks_diff(ticks[i], ticks[i-1])
    if diff > 0:
      tick_diffs.append(diff)

  # debounce: drop unphysically small intervals likely caused by contact bounce
  tick_diffs = [d for d in tick_diffs if d >= WIND_BOUNCE_MS]

  if not tick_diffs:
    return 0, 0, 0

  # calculate average, min, and max intervals
  average_tick_ms = sum(tick_diffs) / len(tick_diffs)
  min_tick_ms = min(tick_diffs)
  max_tick_ms = max(tick_diffs)

  logging.info(f"average_tick_ms: {average_tick_ms}")
  logging.info(f"min_tick_ms: {min_tick_ms}")
  logging.info(f"max_tick_ms: {max_tick_ms}")

  # compute wind speeds from intervals
  avg_wind_m_s = 0
  max_wind_m_s = 0
  min_wind_m_s = 0

  circumference = WIND_CM_RADIUS * 2.0 * math.pi

  if average_tick_ms > 0:
    avg_rotation_hz = (1000 / average_tick_ms) / 2
    avg_wind_m_s = avg_rotation_hz * circumference * WIND_FACTOR

  if min_tick_ms > 0:
    max_rotation_hz = (1000 / min_tick_ms) / 2
    max_wind_m_s = max_rotation_hz * circumference * WIND_FACTOR

  if max_tick_ms > 0:
    min_rotation_hz = (1000 / max_tick_ms) / 2
    min_wind_m_s = min_rotation_hz * circumference * WIND_FACTOR

  # clamp unrealistic spikes
  if max_wind_m_s > WIND_MAX_M_S_CAP:
    max_wind_m_s = WIND_MAX_M_S_CAP

  return avg_wind_m_s, max_wind_m_s, min_wind_m_s

def wind_speed_and_direction_avg(sample_time_ms=40000, dir_sample_interval_ms=25):
  # measure wind speed ticks and accumulate direction vector average in the same window
  state = wind_speed_pin.value()

  ticks = []

  dir_bin_indices = []

  def averaged_direction_degrees():
    if not dir_bin_indices:
      return wind_direction()

    resultant = _wind_direction_resultant_from_bins(dir_bin_indices)
    if resultant < WIND_DIRECTION_MIN_RESULTANT:
      logging.warn(
        f"! wind direction average unstable (resultant={resultant}), "
        "falling back to stabilized instant direction"
      )
      return wind_direction()

    return _circular_median_bin(dir_bin_indices) * 45

  # helper for voltage -> discrete 45° step
  ADC_TO_DEGREES = (0.9, 2.0, 3.0, 2.8, 2.5, 1.5, 0.3, 0.6)
  def voltage_to_degrees(value):
    closest_index = -1
    closest_value = float('inf')
    for i in range(8):
      distance = abs(ADC_TO_DEGREES[i] - value)
      if distance < closest_value:
        closest_value = distance
        closest_index = i
    return closest_index * 45

  start = time.ticks_ms()
  next_dir_sample_at = start
  while time.ticks_diff(time.ticks_ms(), start) <= sample_time_ms:
    now_state = wind_speed_pin.value()
    if now_state != state:
      ticks.append(time.ticks_ms())
      state = now_state

    now_ms = time.ticks_ms()
    if time.ticks_diff(now_ms, next_dir_sample_at) >= 0:
      angle_deg = voltage_to_degrees(wind_direction_pin.read_voltage())
      dir_bin_indices.append(int(angle_deg // 45) % 8)
      next_dir_sample_at = time.ticks_add(next_dir_sample_at, dir_sample_interval_ms)

  logging.info(f"ticks: {ticks}")

  # Filter out duplicate ticks that may occur due to noise/bouncing
  filtered_ticks = []
  for tick in ticks:
    if not filtered_ticks or tick != filtered_ticks[-1]:
      filtered_ticks.append(tick)
  ticks = filtered_ticks

  logging.info(f"filtered_ticks: {filtered_ticks}")

  if len(ticks) < 2:
    # no speed, still return averaged direction if we have it
    return 0, 0, 0, averaged_direction_degrees()

  # intervals between ticks with debounce
  tick_diffs = []
  for i in range(1, len(ticks)):
    diff = time.ticks_diff(ticks[i], ticks[i-1])
    if diff > 0:
      tick_diffs.append(diff)
  tick_diffs = [d for d in tick_diffs if d >= WIND_BOUNCE_MS]
  if not tick_diffs:
    return 0, 0, 0, averaged_direction_degrees()

  average_tick_ms = sum(tick_diffs) / len(tick_diffs)
  min_tick_ms = min(tick_diffs)
  max_tick_ms = max(tick_diffs)

  logging.info(f"average_tick_ms: {average_tick_ms}")
  logging.info(f"min_tick_ms: {min_tick_ms}")
  logging.info(f"max_tick_ms: {max_tick_ms}")

  circumference = WIND_CM_RADIUS * 2.0 * math.pi

  avg_wind_m_s = 0
  max_wind_m_s = 0
  min_wind_m_s = 0

  if average_tick_ms > 0:
    avg_rotation_hz = (1000 / average_tick_ms) / 2
    avg_wind_m_s = avg_rotation_hz * circumference * WIND_FACTOR

  gust_tick_ms = min_tick_ms
  if len(tick_diffs) > 1:
    sorted_tick_diffs = sorted(tick_diffs)
    fastest_sample_count = min(WIND_GUST_FASTEST_SAMPLES, len(sorted_tick_diffs))
    gust_tick_ms = sum(sorted_tick_diffs[:fastest_sample_count]) / fastest_sample_count

  if gust_tick_ms > 0:
    max_rotation_hz = (1000 / gust_tick_ms) / 2
    max_wind_m_s = max_rotation_hz * circumference * WIND_FACTOR

  if max_tick_ms > 0:
    min_rotation_hz = (1000 / max_tick_ms) / 2
    min_wind_m_s = min_rotation_hz * circumference * WIND_FACTOR

  if max_wind_m_s > WIND_MAX_M_S_CAP:
    max_wind_m_s = WIND_MAX_M_S_CAP

  return avg_wind_m_s, max_wind_m_s, min_wind_m_s, averaged_direction_degrees()

def wind_direction():
  # adc reading voltage to cardinal direction taken from our python
  # library - each array index represents a 45 degree step around
  # the compass (index 0 == 0, 1 == 45, 2 == 90, etc.)
  # we find the closest matching value in the array and use the index
  # to determine the heading
  ADC_TO_DEGREES = (0.9, 2.0, 3.0, 2.8, 2.5, 1.5, 0.3, 0.6)

  # require two matching samples in a row to avoid glitching mid-transition
  # (fixes https://github.com/pimoroni/enviro/issues/20). Cap attempts so a
  # noisy or disconnected vane cannot hang the board forever.
  MAX_ATTEMPTS = 10
  SAMPLE_DELAY_MS = 5

  last_index = None
  closest_index = -1

  for _ in range(MAX_ATTEMPTS):
    value = wind_direction_pin.read_voltage()

    closest_index = -1
    closest_value = float('inf')
    for i in range(8):
      distance = abs(ADC_TO_DEGREES[i] - value)
      if distance < closest_value:
        closest_value = distance
        closest_index = i

    if last_index == closest_index:
      return closest_index * 45

    last_index = closest_index
    time.sleep_ms(SAMPLE_DELAY_MS)

  logging.warn(f"! wind_direction did not stabilise after {MAX_ATTEMPTS} samples, returning last value {closest_index * 45}")
  return closest_index * 45

def rainfall(seconds_since_last):
  amount = 0
  now = helpers.timestamp(helpers.datetime_string())
  if helpers.file_exists("rain.txt"):
    with open("rain.txt", "r") as rainfile:
      rain_entries = rainfile.read().split("\n")

    # count how many rain ticks since the last reading
    for entry in rain_entries:
      if entry:
        ts = helpers.timestamp(entry)
        if now - ts < seconds_since_last:
          amount += RAIN_MM_PER_TICK

    os.remove("rain.txt")
  
  per_second = 0
  if seconds_since_last > 0:
    per_second = amount / seconds_since_last

  return amount, per_second

def get_sensor_readings(seconds_since_last, is_usb_power):
  # bme280 returns the register contents immediately and then starts a new reading
  # we want the current reading so do a dummy read to discard register contents first
  bme280.read()
  time.sleep(0.1)
  bme280_data = bme280.read()

  ltr_data = ltr559.get_reading()
  rain, rain_per_second = rainfall(seconds_since_last)
  avg_wind_speed, max_wind_speed, min_wind_speed, avg_wind_direction = wind_speed_and_direction_avg(sample_time_ms=WIND_DIRECTION_AVG_WINDOW_MS)

  from ucollections import OrderedDict
  return OrderedDict({
    "temperature": round(bme280_data[0], 2),
    "humidity": round(bme280_data[2], 2),
    "pressure": round(bme280_data[1] / 100.0, 2),
    "luminance": round(ltr_data[BreakoutLTR559.LUX], 2),
    "avg_wind_speed": avg_wind_speed,
    "min_wind_speed": min_wind_speed,
    "max_wind_speed": max_wind_speed,
    "rain": rain,
    "rain_per_second": rain_per_second,
    "wind_direction": avg_wind_direction,
  })
