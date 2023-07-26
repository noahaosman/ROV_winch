# pyright: reportMissingImports=false
import time
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module

false_pulse_delay_actuator = 2  # (zero for no debounce delay)
false_pulse_delay_reed_sw = 0.250  # s

class Switch:

    def __init__(
        self,
        overboardPin,
        retractPin
    ):
        
        self.deployed = DigitalInOut(eval('board.D'+str(overboardPin)))
        self.deployed.direction = Direction.INPUT
        self.deployed.pull = Pull.DOWN

        self.retracted = DigitalInOut(eval('board.D'+str(retractPin)))
        self.retracted.direction = Direction.INPUT
        self.retracted.pull = Pull.DOWN

    # three possible states:
    #   1) retracted HIGH, deployed LOW  ::  arm is retracted --> Motor OFF!
    #   2) retracted LOW , deployed HIGH ::  ROV is in water  --> Motor speed unrestricted
    #   3) retracted LOW , deployed LOW  ::  arm is moving into retracted position --> restrict motor to low speed
    def read_state(self, winch):
        while True:
            if   self.retracted.value == 1 and self.deployed.value == 0:
                winch.ON.value = 0
                winch.move_servo(0)

            elif self.retracted.value == 0 and self.deployed.value == 1:
                winch.move_servo(0)

            elif self.retracted.value == 0 and self.deployed.value == 0:
                winch.move_servo(0)

            else:
                winch.move_servo(0)
            
            time.sleep(0.25)