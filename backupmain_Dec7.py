import time, utime
from machine import UART, Pin, PWM
from rotary_irq_rp2 import RotaryIRQ

# GPIO Pin assignments:
#   0  : serial comm. T1 input
#   1  : Serial comm. R1 output
#   2  : -
#   3  : -
#   4  : -
#   5  : -
#   6  : -
#   7  : -
#   8  : actuator feedback
#   9  : -
#   10 : 0-5V DAC, SDA
#   11 : 0-5V DAC, SCL
#   12 : -
#   13 : -
#   14 : FWD/REV relay #1
#   15 : FWD/REV relay #2
#   16 : Actuator DPDT relay (direction reverse)
#   17 : -
#   18 : -
#   19 : motorized pot. servo control (winch speed)
#   20 : encoder Z+
#   21 : encoder A+
#   22 : encoder B+
#   26 : Reed switch
#   27 : - (inaccessible)
#   28 : -

# ==============================================================================

# ------------------------------------------------------------------------------
# initialize heartbeat LED
heartbeat = Pin(25, Pin.OUT)

# ------------------------------------------------------------------------------
# Definitions for rotary encoder
rot_irq = RotaryIRQ(pin_num_clk=22,
    pin_num_dt=21,
    min_val=0,
    reverse=False,
    range_mode=RotaryIRQ.RANGE_UNBOUNDED)
time0 = time.ticks_us()
prior_encoder_position = rot_irq.value()
block_radius = 0.062 # block radius in meters
block_circ = 2.0 * block_radius * 3.14159
def SerialWriteEncoder(encoder_position):
    global time0
    global prior_encoder_position
    # serial write encoder position
    time1 = time.ticks_us()
    dt = time.ticks_diff(time1,time0)*1e-6
    enc_vel = (encoder_position-prior_encoder_position)/dt
    out_string='SPD '+str(enc_vel)+' DIS '+str(encoder_position)+'\r\n'
    uart0.write(bytes(out_string,'UTF-8'))
    prior_encoder_position = encoder_position
    time0 = time1

# ------------------------------------------------------------------------------
# define FWD/REV/ON pinout (winch direction)
FWD0_REV1 = Pin(15, Pin.OUT) # wire yellow --> REV; red --> FWD
ON = Pin(14, Pin.OUT)
ON.value(0)

# ------------------------------------------------------------------------------
# Definitions for motorized potentiometer (winch speed)
pwm = PWM(Pin(19))
pwm.freq(50)
uMIN = 500000
uMAX = 1320000# full 270 deg of motion = 2550000
uSTEP = int((uMAX-uMIN)/100)
#   set speed to zero on startup
pwm.duty_ns(uMIN)
servo_position = uMIN
SPD = 0  # intialize winch speed as 0
#   function to move motorized potentiometer
def move_servo(servo_position, percent):
    servo_move_to = int(uMIN + (uMAX-uMIN)*(percent/100))
    if servo_move_to>=servo_position:
        dir = 1
    else:
        dir = -1
    for i in range(servo_position,servo_move_to,dir*uSTEP):
        pwm.duty_ns(i)
        time.sleep(0.01)
    return servo_move_to

# ------------------------------------------------------------------------------

# Definitions for actuator (level wind) control
#   Create I2C object
i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10))
#   define function to write an actuator speed 0->100
def writeActuatorSpeed(value):
    value = round(value/100*4095)
    buf=bytearray(2)
    buf[0]=(value >> 8) & 0xFF
    buf[1]=value & 0xFF
    i2c.writeto(0x60,buf)
#   actuator direction control
actuator_direction = Pin(16, Pin.OUT)
retract = 1
extend = 0
current_direction = extend
writeActuatorSpeed(0)

#   actuator feedback
actuatorFeedback = Pin(7, Pin.IN, machine.Pin.PULL_UP)
#   set up irq for position tracking
actuator_position = 0
falsepulseDelay = 0 # milliseconds
last_pulse_time = -falsepulseDelay
# this function updates the position tracker for the level wind
def updatePosition(p):
    global actuator_position
    global last_pulse_time
    print("updating actuator position")
    current_pulse_time = time.ticks_ms()
    if (current_pulse_time - last_pulse_time ) > falsepulseDelay: # debouncing
        actuator_position = actuator_position + 1
        last_pulse_time = current_pulse_time
        print("optical feedback postition: ", actuator_position)

actuatorFeedback.irq(trigger=machine.Pin.IRQ_RISING, handler=updatePosition)

# define level wind speed as a function of the winch speed
def actuatorSpeed(winch_speed, distance = 0):
    if winch_speed == 0: # if adjusting, base speed off distance to move
        speed = distance/12*100
    else: # else if winching, base speed off winch speed
        speed = winch_speed  # some function of winch speed (to be made later)

    speed = max(40, min(100, speed)) # bound speed from 20 <-> 100
    print("actuator speed: " + str(speed))
    return speed

# move actuator some distance in inches

def moveActuator(distance = 0.42): # default to cable diameter

    global actuator_position
    global current_direction
    global line_stack_state

    target_pulses = round(50*distance)

    actuator_position = 0   # zero position tracker
    writeActuatorSpeed(actuatorSpeed(SPD, distance))  # write a speed

    # check counted pulses every 250 ms.
    while True:
        prior_actuator_position = actuator_position
        print("actuator position 1 : ", str(prior_actuator_position))
        time.sleep(0.25)  # wait for actuator to move
        print("actuator position 2 : ", str(actuator_position))
        if actuator_position >= target_pulses:  # if we've reached our target,
            print("reached target position")
            writeActuatorSpeed(0)  # turn off actuator
            break
        elif actuator_position == prior_actuator_position:  # if actuator is not moving
            print("edge detected")
            if False:#SPD != 0: # reverse direction only if currently winching
                changeActuatorDirection()
            else:  # otherwise (if adjusting position) just stop here
                print("stopping actuation")
                writeActuatorSpeed(0)  # turn off actuator
                actuator_position = 0   # zero position tracker
                break

def changeActuatorDirection():
    global current_direction
    global line_stack_state

    print("reversing direction")
    current_direction = opposite(current_direction)
    actuator_direction.value(current_direction) # reverse direction
    line_stack_state = opposite(line_stack_state)
    with open('stacking_state.txt') as infp:
        infp.write(str(line_stack_state))

# ------------------------------------------------------------------------------
#   load line stack state from prior run
falling = 1
rising = 0
with open('stacking_state.txt') as infp:
    line_stack_state = int(infp.read())

# ------------------------------------------------------------------------------
# reed switch irq for rotation tracking
reed_sw = machine.Pin(26, machine.Pin.IN, machine.Pin.PULL_UP)
falsepulseDelay_reed = 500 # (ms)
last_reed_time = -falsepulseDelay_reed
def rotationTrackingReedSwitchTrigger(p):
    global current_direction
    global line_stack_state
    global last_reed_time
    global actuator_position
    time.sleep_ms(10)
    if reed_sw.value() == 1:
        current_reed_time = time.ticks_ms()
        if (current_reed_time - last_reed_time ) > falsepulseDelay_reed and ON.value()==1:
            print("reed switch triggered")
            last_reed_time = current_reed_time
            # move actuator one cable width
            moveActuator(0.42)

reed_sw.irq(trigger=machine.Pin.IRQ_RISING, handler=rotationTrackingReedSwitchTrigger)

# ------------------------------------------------------------------------------
# Automated saftey to stop if all line is out
max_line_out = 200
def StopIfMaxLineOut():
    global ROF
    global SPD
    if encoder_position >= max_line_out:
        ROF = 0
        ON.value(0)
        SPD = 0
        servo_position = move_servo(servo_position, SPD)

# ------------------------------------------------------------------------------
# useful function to flip booleans
# ie 0 --> 1 and 1 --> 0
def opposite(input):
    output = 1 - input
    return output

# ------------------------------------------------------------------------------
# initialize serial comm
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

# ==============================================================================

while True:

    # read any serial in
    try:
        serial_in = uart0.readline()
        in_decoded = serial_in.decode('UTF-8')

        in_strings = in_decoded.split()

        # move motor FWD / REV / OFF
        if in_strings[0] is "ROF":

            # switch to FWD / REV / OFF
            ROF = int(in_strings[1])
            if (ROF == 0): # off
                ON.value(0)
            elif ROF == 1 and encoder_position < max_line_out: # FWD (feed out line)
                ON.value(1)
                FWD0_REV1.value(0)
                current_direction = line_stack_state
                actuator_direction.value(current_direction)
            elif ROF == -1: # REV (take in line)
                ON.value(1)
                FWD0_REV1.value(1)
                current_direction = opposite(line_stack_state)
                actuator_direction.value(current_direction)

            # set the speed
            SPD = float(in_strings[3])
            if (SPD >=0 and SPD <= 100):
                time.sleep(0.1)
                servo_position = move_servo(servo_position, SPD)
                print("done")
            else:
                ON.value(0)

        # adjust max line out
        elif in_strings[0] is "MLO":
            print("max line out updated from "+str(max_line_out))
            max_line_out = int(in_strings[1])
            print("to "+str(max_line_out))

        # manually adjust level wind postition
        elif in_strings[0] is "LWA":
            if ON.value() == 0: # make sure motor is OFF!
                try:
                    distance = float(in_strings[1])
                    print('level wind adjusting '+in_strings[1]+' inches')
                    if distance > 0: # extend actuator
                        actuator_direction.value(extend)
                    else: # retract actuator
                        actuator_direction.value(retract)

                    moveActuator(abs(distance))
                    actuator_direction.value(current_direction)  # reset the direction to earlier state
                except:
                    if in_strings[1] == 'cd':
                        print('level wind changing direction')
                        changeActuatorDirection()


    except:
        pass

    try:
        encoder_position = rot_irq.value()/100.0 * block_circ
        SerialWriteEncoder(encoder_position)
    except:
        print("error in sending serial output")

    # check for max line out
    StopIfMaxLineOut()

    time.sleep(0.25)
    heartbeat.toggle()
