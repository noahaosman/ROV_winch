# pyright: reportMissingImports=false
import time
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
from threading import Thread

false_pulse_delay_actuator = 2  # (zero for no debounce delay)
false_pulse_delay_reed_sw = 0.250  # s

class Switch:

    def __init__(
        self,
        overboardPin,
        RetractedPin
    ):
        
        logic_high = DigitalInOut(board.D12)
        logic_high.direction = Direction.OUTPUT
        logic_high.value = 1
        
        self.deployed = DigitalInOut(eval('board.D'+str(overboardPin)))
        self.deployed.direction = Direction.INPUT
        self.deployed.pull = Pull.DOWN

        self.retracted = DigitalInOut(eval('board.D'+str(RetractedPin)))
        self.retracted.direction = Direction.INPUT
        self.retracted.pull = Pull.DOWN

    # three possible states:
    #   1) retracted HIGH, deployed LOW  ::  arm is retracted --> Motor OFF!
    #   2) retracted LOW , deployed HIGH ::  ROV is in water  --> Motor speed unrestricted
    #   3) retracted LOW , deployed LOW  ::  arm is moving into retracted position --> restrict motor to low speed
    def read_state(self, winch):
        last_state = 0
        while True:
            if self.retracted.value == 1 and self.deployed.value == 0 and last_state != 1:
                print('Arm retracted, shutting motor off.')
                winch.ON.value = 0
                winch.move_servo(0)
                last_state = 1

            elif self.retracted.value == 0 and self.deployed.value == 1 and last_state != 2:
                print('Arm overboard, no speed restrictions.')
                last_state = 2

            elif self.retracted.value == 0 and self.deployed.value == 0:
                print('Arm retracting, slowing motor down.')
                if winch.ON.value == 1 and winch.servo.angle > 20 * 270/100:
                    winch.move_servo(20)
                last_state = 3

            else: # this state is undefined. Stop everything.
                winch.move_servo(0)
                last_state = 0
            
            time.sleep(0.25)