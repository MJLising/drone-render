# drone_motors.py
"""
Motor driver helper (H-bridge / IN1-IN2-ENA) for your Pi drone.
Uses pigpio. Configure DETAILED_PINS below to match your wiring (BCM).

This version uses your mapping (DETAILED_PINS) by default.
"""
import logging
try:
    import pigpio
except Exception:
    pigpio = None

log = logging.getLogger("drone_motors")
if not log.handlers:
    h = logging.StreamHandler()
    h.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    log.addHandler(h)
log.setLevel(logging.INFO)

# ========== DETAILED (H-bridge) PIN MAP - edit if needed ==========
DETAILED_PINS = {
    'left_in1': 4,  'left_in2': 17, 'left_ena': 18,
    'right_in1': 27,'right_in2': 22,'right_ena': 23,
    'top_in1': 24,  'top_in2': 25, 'top_ena': 12,
    'bottom_in1':16,'bottom_in2':20,'bottom_ena':13,
}
# =================================================================

PWM_RANGE = 255
PWM_FREQ = 800

pi = None
_mode = None

def _setup_pigpio():
    global pi, _mode
    if pigpio is None:
        log.warning("pigpio module not available -- motors are no-op")
        _mode = "noop"
        return
    try:
        pi = pigpio.pi()
        if not pi.connected:
            log.warning("pigpiod not connected; motors are no-op")
            pi = None
            _mode = "noop"
            return
    except Exception as e:
        log.exception("pigpio.pi() error: %s", e)
        pi = None
        _mode = "noop"
        return

    if DETAILED_PINS:
        _mode = "detailed"
        log.info("Motor mode: DETAILED (H-bridge). Pins: %s", DETAILED_PINS)
        for k,v in DETAILED_PINS.items():
            try:
                pi.set_mode(v, pigpio.OUTPUT)
                if 'in' in k:
                    pi.write(v, 0)
                else:
                    # ena/pwm pin
                    pi.set_PWM_range(v, PWM_RANGE)
                    pi.set_PWM_frequency(v, PWM_FREQ)
                    pi.set_PWM_dutycycle(v, 0)
            except Exception as e:
                log.warning("init pin %s (%s) error: %s", k, v, e)
    else:
        _mode = "noop"
        log.warning("No pin map found; motors noop")

_setup_pigpio()

def _set_pwm(pin, frac):
    if pi is None:
        log.info("[motor-noop] set_pwm pin=%s frac=%.3f", pin, frac)
        return
    duty = int(max(0.0, min(1.0, frac)) * PWM_RANGE)
    try:
        pi.set_PWM_dutycycle(pin, duty)
    except Exception as e:
        log.warning("set_PWM_dutycycle error pin %s: %s", pin, e)

def set_motor(name, value):
    """
    name: left|right|top|bottom
    value: -1.0 .. 1.0 (sign = direction)
    """
    v = float(value)
    if _mode == "noop":
        log.info("[motor-noop] %s = %.3f", name, v)
        return

    if _mode == "detailed":
        in1 = DETAILED_PINS.get(f"{name}_in1")
        in2 = DETAILED_PINS.get(f"{name}_in2")
        ena  = DETAILED_PINS.get(f"{name}_ena") or DETAILED_PINS.get(f"{name}_pwm")
        if in1 is None or in2 is None or ena is None:
            log.warning("detailed pins missing for %s (in1=%s in2=%s ena=%s)", name, in1, in2, ena)
            return

        if v == 0:
            try:
                pi.write(in1, 0); pi.write(in2, 0)
                pi.set_PWM_dutycycle(ena, 0)
            except Exception as e:
                log.warning("failed to stop %s: %s", name, e)
            log.info("DETAILED stop %s", name)
            return

        duty = int(max(0.0, min(1.0, abs(v))) * PWM_RANGE)
        try:
            if v > 0:
                pi.write(in1, 1); pi.write(in2, 0)
            else:
                pi.write(in1, 0); pi.write(in2, 1)
            pi.set_PWM_dutycycle(ena, duty)
            log.info("DETAILED set %s -> %.3f (in1=%s in2=%s ena=%s pwm=%d)", name, v, in1, in2, ena, duty)
        except Exception as e:
            log.warning("DETAILED set error %s: %s", name, e)

def stop_all():
    log.info("stop_all called")
    if _mode == "detailed":
        for nm in ('left','right','top','bottom'):
            in1 = DETAILED_PINS.get(f"{nm}_in1")
            in2 = DETAILED_PINS.get(f"{nm}_in2")
            ena  = DETAILED_PINS.get(f"{nm}_ena") or DETAILED_PINS.get(f"{nm}_pwm")
            try:
                if in1 is not None: pi.write(in1, 0)
                if in2 is not None: pi.write(in2, 0)
                if ena is not None: pi.set_PWM_dutycycle(ena, 0)
            except Exception:
                pass
    log.info("All motors set to 0")

def shutdown():
    stop_all()
    if pi:
        try: pi.stop()
        except: pass
    log.info("shutdown done")
