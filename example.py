import time, utime
from machine import UART, Pin, PWM

# define things for actuator control
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
actuatorDirection = Pin(16, Pin.OUT)
#   set initial speed to zero
extend = 0
retract = 1
actuatorSpeed = 0


#  home actuator
actuatorDirection.value(retract)
writeActuatorSpeed(80)
print("homing...")
time.sleep(8)
print("done")
actuatorDirection.value(extend)
writeActuatorSpeed(actuatorSpeed)


# define things for actuator feedback
actuatorFeedback = Pin(7, Pin.IN, machine.Pin.PULL_UP)
#   set up irq for position tracking
pos = 0
falsepulseDelay = 8 # milliseconds
last_time = -falsepulseDelay
def UpdatePosition(p):
    global pos
    global last_time
    global falsepulseDelay
    if (time.ticks_ms() - last_time ) > falsepulseDelay:
        if actuatorDirection.value() == retract:
            pos = pos - 1
        else:
            pos = pos + 1
        last_time = time.ticks_ms()
        print("optical feedback postition: ", pos)#   start irq
    else:
        print("debounced")
actuatorFeedback.irq(trigger=machine.Pin.IRQ_RISING, handler=UpdatePosition)


while True:

    # extend for 5 seconds
    actuatorSpeed = 20
    writeActuatorSpeed(actuatorSpeed)
    time.sleep(10)

    # sleep for 2 seconds
    actuatorSpeed = 0
    writeActuatorSpeed(actuatorSpeed)
    actuatorDirection.value(retract)
    time.sleep(2)

    # retract for 5 seconds
    actuatorSpeed = 20
    writeActuatorSpeed(actuatorSpeed)
    time.sleep(20)

    # sleep for 2 seconds
    actuatorSpeed = 0
    writeActuatorSpeed(actuatorSpeed)
    actuatorDirection.value(extend)
    time.sleep(2)
