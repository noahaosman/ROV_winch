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
    current_sensor_pin = 28
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

while True:
    
    try:
        
        # read any serial in
        try:
            serial_in = uart0.readline()
            in_decoded = serial_in.decode('UTF-8')
            in_strings = in_decoded.split()
            print(in_strings)
        except:
            in_strings = ['N/A']
            pass

        # move motor FWD / REV / OFF
        if in_strings[0] is "ROF":

            # switch to FWD / REV / OFF
            ROF = int(in_strings[1])
            if (ROF == 0): # off
                winch.ON.value(0)
            elif ROF == 1: #and encoder_position < max_line_out: # FWD (feed out line)
                winch.ON.value(1)
                winch.FWD0_REV1.value(0)
                level_wind.current_direction = level_wind.line_stack_state
            elif ROF == -1: # REV (take in line)
                winch.ON.value(1)
                winch.FWD0_REV1.value(1)
                level_wind.current_direction = level_wind.opposite(level_wind.line_stack_state)

            # set the speed
            SPD = float(in_strings[3])
            if (SPD >=0 and SPD <= 100):
                time.sleep(0.1)
                winch.move_servo(SPD)
                print("winch speed set")
            else:
                print("ERROR: invalid winch speed")
                winch.ON.value(0)

        # manually adjust level wind postition
        elif in_strings[0] is "LWA":
            if winch.ON.value() == 0: # make sure motor is OFF!
                SPD = 0
                if in_strings[1] == 'cd':
                        print('changing level wind direction')
                        level_wind.changeDirection()
                else:
                    distance = float(in_strings[1])
                    print('level wind adjusting '+in_strings[1]+' inches')
                    if distance > 0:
                        direction = 0
                    else:
                        direction = 1
                    level_wind.move(0, direction, abs(distance), True)


        # serial write encoder position & velocity 
        try:
            uart0.write(bytes(encoder.getState(),'UTF-8'))
        except:
            print("error in sending serial output")
            

        # Check if actuator needs to be moved
        if level_wind.NeedToMoveActuator:
            if ROF == 1: # FWD (feed out line)
                level_wind.current_direction = level_wind.line_stack_state
                level_wind.move(SPD)
            elif ROF == -1: # REV (take in line)
                level_wind.current_direction = level_wind.opposite(level_wind.line_stack_state)
                level_wind.move(SPD)
            level_wind.NeedToMoveActuator = False
        else:
            time.sleep(0.1)
            
        # Measure current draw of motor. Shut off if above threshold 
        if winch.ON.value()==1:
            volty = winch.current_sensor.read_u16()* 3.3 / 65535
            curry = -10 * volty + 25
            vs = '%.2f' % volty
            cs = '%.2f' % curry
            print(vs,"V ; ",cs, "A")
            
            if curry > 5:
                winch.ON.value(0)
                winch.move_servo(0)
                level_wind.writeSpeed(0)
                print("HIGH CURRENT DETECTED! shutting off motor...")

        heartbeat.toggle()
        
    except Exception as err:
        print("Exception raised. Turning off winch ... ")
        winch.ON.value(0)
        winch.move_servo(0)
        level_wind.writeSpeed(0) 
        print(err)
    
    
    
    
