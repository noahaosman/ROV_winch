# pyright: reportMissingImports=false
import time
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
from threading import Thread
import serial
import traceback

from Motor import Motor
from Actuator import Actuator
import Overboarding


# =============================================================================

def control_winch(mode):

    # -----------------------------------------------------------------------------
    # initialize heartbeat LED
    heartbeat = DigitalInOut(board.D13)
    heartbeat.direction = Direction.OUTPUT
 
    # # Rotary encoder --------------------------------------------------------------
    # encoder = RotaryIRQ(
    #     pin_num_clk=22,
    #     pin_num_dt=21,
    #     block_radius=0.062,  # block radius in meters,
    # )
    # -----------------------------------------------------------------------------

    # Winch motor -----------------------------------------------------------------
    winch = Motor(
        FWD0_REV1_pin=26,
        ON_OFF_pin=19,
        mot_pot_pin=16,
        current_limit=6
    )
    # -----------------------------------------------------------------------------

    # Level wind actuator ---------------------------------------------------------
    level_wind = Actuator(
        ON_OFF_pin=20,
        direction_pin=21,
        feedback_pin=10,
        rotation_pin=24,
        cable_diameter=0.4
    )
    level_wind.writeSpeed(0)
    # -----------------------------------------------------------------------------

    # initialize input medium -----------------------------------------------------
    #  - debug mode      : terminal input
    #  - deployment mode : serial comm via ubox
    #  note: board LED will stay constant on while attempting to establish connection
    if mode == 'debug':
        print('COMMAND OPTIONS:')
        print('    - ROF <ARG1> SPD <arg2> :: move motor (-1 = in | 0 = off | 1 = out) and set speed (<float> 0<-->100)')
        print('    - LWA :: Level wind adjust ( CD = change dir. | <float> = move <float> inches)')
        print('    - TDA :: Tether diameter adjust (<float> set tether diameter to <float> inches)' )
        print('    - CLA :: Current limit adjust (<float> sets motor current limit to <float> amps)')

        # start thread for debug mode user input
        in_strings = ['N/A']
        def get_usr_input():
            while True:
                in_strings = input('Input (<COMMAND> <VALUE>) : ' )
                in_strings = in_strings.split()
        Thread(daemon=True, target=get_usr_input).start()
    else:
        while True:
            try:
                uart0 = serial.Serial('dev/ttyAMA0', baudrate=115200)
                heartbeat.value = 0
                break
            except Exception:
                time.sleep(1)
                heartbeat.value = 1

    print("initialized")
    # -----------------------------------------------------------------------------

    # =============================================================================

    while True:

        try:

            # read any serial in
            try:
                if mode != 'debug':
                    serial_in = uart0.readline()
                    in_decoded = serial_in.decode('UTF-8')
                    in_strings = in_decoded.split()
                    print(in_strings)
            except Exception:
                in_strings = ['N/A']
                out_string = level_wind.RotationCounter
                pass

            # move motor FWD / REV / OFF
            if in_strings[0] == "ROF":

                # switch to FWD / REV / OFF
                ROF = int(in_strings[1])
                if (ROF == 0):  # off
                    winch.ON.value = 0
                elif ROF == 1:  # and encoder_position < max_line_out: # FWD (feed out line)
                    winch.ON.value = 1
                    winch.FWD0_REV1.value = 0
                    level_wind.current_direction = level_wind.line_stack_state
                elif ROF == -1:  # REV (take in line)
                    winch.ON.value = 1
                    winch.FWD0_REV1.value = 1
                    level_wind.current_direction = level_wind.opposite(level_wind.line_stack_state)

                # set the speed
                SPD = float(in_strings[3])
                if (SPD >= 0 and SPD <= 100):
                    time.sleep(0.1)
                    winch.move_servo(SPD)
                    print("winch speed set")
                else:
                    print("ERROR: invalid winch speed")
                    winch.ON.value = 0

                out_string = "INFO Winch speed set.\r\n"

            # manually adjust level wind postition
            elif in_strings[0] == "LWA":
                if winch.ON.value == 0:  # make sure motor is OFF!
                    if in_strings[1] == "CD":
                        level_wind.changeDirection()
                        out_string = "INFO Level wind direction changed.\r\n"
                    else:
                        level_wind.ManualAdjust(in_strings[1])
                        out_string = "INFO Level wind adjusted.\r\n"
                else:
                    out_string = "INFO Motor must be stationary before adjusting level wind.\r\n"

            # Adjust cable diameter parameter
            elif in_strings[0] == "TDA":
                if winch.ON.value == 0:  # make sure motor is OFF!
                    level_wind.cable_diameter = float(in_strings[1])
                    out_string = "INFO Cable diameter updated.\r\n"
                else:
                    out_string = "INFO Motor must be stationary before changing parameter.\r\n"

            # Adjust current limit
            elif in_strings[0] == "CLA":
                winch.current_limit = float(in_strings[1])
                out_string = "INFO Current limit updated.\r\n"

             # undefined user input
            else:
                out_string = "INFO invalid input.\r\n"


            # serial write encoder position & velocity
            if mode == 'debug':
                print(out_string)
                pass
            else:
                try:
                    uart0.write(bytes(out_string, 'UTF-8'))
                    if out_string.split()[0] == "INFO":
                        print(out_string)
                except Exception:
                    print("error sending serial output")


            # Check if actuator needs to be moved
            if level_wind.NeedToMoveActuator:
                if ROF == 1:  # FWD (feed out line)
                    level_wind.current_direction = level_wind.line_stack_state
                    level_wind.move(SPD)
                elif ROF == -1:  # REV (take in line)
                    level_wind.current_direction = level_wind.opposite(level_wind.line_stack_state)
                    level_wind.move(SPD)
                level_wind.NeedToMoveActuator = False
            else:
                time.sleep(0.1)

            winch.measure_current()

            heartbeat.value = not heartbeat.value # toggle LED

        except Exception:
            print("Exception raised. Turning off winch ... ")
            winch.ON.value = 0
            winch.move_servo(0)
            level_wind.writeSpeed(0)
            print(traceback.format_exc())

