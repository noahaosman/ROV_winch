# pyright: reportMissingImports=false
import time
import board
import busio
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()


class Motor:

    def __init__(
        self,
        FWD0_REV1_pin,
        ON_OFF_pin,
        mot_pot_pin,
        current_limit=5
    ):

        self.FWD0_REV1 = DigitalInOut(eval('board.D'+str(FWD0_REV1_pin)))
        self.FWD0_REV1.direction = Direction.OUTPUT

        self.ON = DigitalInOut(eval('board.D'+str(ON_OFF_pin)))
        self.ON.direction = Direction.OUTPUT
        self.ON.value = 0

        self.servo = AngularServo(mot_pot_pin, min_angle=0, max_angle=270, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.servo.angle = 0

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.current_sensor = ADS.ADS1115(self.i2c, address=0x48)
        self.current_limit = current_limit

    def move_servo(self, percent):
        self.servo.angle = percent * 270/100

    def seconds_per_rotation(x):
        rpm = -11.8876 + 0.372714*x + 0.00207748*x**2.0
        out = 60/rpm
        return out

    def measure_current(self):
        # Measure current draw of motor. Shut off if above threshold
        voltage_divider = (100+47)/100
        if self.ON.value == 1:
            ADCread = AnalogIn(self.current_sensor, ADS.P0)
            volty = ADCread.voltage
            curry = (-10 * volty * voltage_divider + 25)
            vs = '%.2f' % volty
            cs = '%.2f' % curry
            # print(vs, "V ; ", cs, "A")

            if abs(curry) > self.current_limit:
                for i in range(4):  # double check before shutoff -- take an average over 50 ms
                    time.sleep(0.01)
                    ADCread = AnalogIn(self.current_sensor, ADS.P0)
                    volty = ADCread.voltage
                    curry = curry - 10 * volty * voltage_divider + 25
                curry = curry/(i+2)
                if abs(curry) > self.current_limit:
                    self.ON.value = 0
                    self.move_servo(0)
#                     level_wind.writeSpeed(0)
                    print("HIGH CURRENT (", str(curry), "A ) DETECTED! shutting off motor...")
