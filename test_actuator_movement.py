import time
from machine import UART, Pin, PWM
from rotary import RotaryIRQ
from Motor import Motor
from Actuator import Actuator

# ==============================================================================

# ------------------------------------------------------------------------------
# initialize heartbeat LED
heartbeat = Pin(25, Pin.OUT)

# Rotary encoder ---------------------------------------------------------------
encoder = RotaryIRQ(
    pin_num_clk=22,
    pin_num_dt=21,
    block_radius=0.062, # block radius in meters,
)
# ------------------------------------------------------------------------------

# Winch motor ------------------------------------------------------------------
winch = Motor(
    FWD0_REV1_pin = 15,
    ON_OFF_pin = 14,
    mot_pot_pin = 19,
)
# ------------------------------------------------------------------------------

# Level wind actuator ------------------------------------------------------------------
level_wind = Actuator(
    scl_pin = 11,
    sda_pin = 10,
    ON_OFF_pin = 17,
    direction_pin = 16,
    feedback_pin = 7,
    rotation_pin = 26,
    cable_diameter = 0.375
)
level_wind.writeSpeed(0)
# ------------------------------------------------------------------------------

# initialize serial comm -------------------------------------------------------
# board LED will stay constant on while attempting to establish connection
while True:
    try:
        uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
        heartbeat.off()
        break
    except:
        time.sleep(1)
        heartbeat.on()

print("initialized")
# ------------------------------------------------------------------------------

# ==============================================================================


level_wind.MAspeed = 100
level_wind.move(0, 1, 12, True)

level_wind.MAspeed = 20
distance = 0.375
steps = 8

SPD =  32  # motor speed

it = 0
while it < 8:

    winch.ON.value(1)
    winch.FWD0_REV1.value(0)
    time.sleep(0.1)
    winch.move_servo(SPD)

    if level_wind.NeedToMoveActuator:
        level_wind.move(0, 0, abs(distance), True)
        level_wind.NeedToMoveActuator = False
        it = it + 1
        print(it)
    else:
        time.sleep(0.5)
        level_wind.NeedToMoveActuator = True



winch.move_servo(0)
time.sleep(0.1)
winch.ON.value(0)
winch.FWD0_REV1.value(0)
while True:
    time.sleep(1)
