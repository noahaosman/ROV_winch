import time, utime
from machine import UART, Pin, PWM
from simple_pid import PID

# -------------------------------------------
# define things for actuator controller
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
actuatorSpeed = 0
retract = 1
extend = 0

actuatorFeedback = Pin(7, Pin.IN, machine.Pin.PULL_UP)
#   set up irq for position tracking
position = 0
falsepulseDelay = 8 # milliseconds
last_time = -falsepulseDelay
def UpdatePosition(p):
    global position
    global last_time
    if (time.ticks_ms() - last_time ) > falsepulseDelay:
        if actuatorDirection.value() == retract:
            position = position - 1 # retracting
        else:
            position = position + 1 # extending

        last_time = time.ticks_ms()
        print("optical feedback postition: ", position/475*100)#   start irq
    else:
        print("debounced")
actuatorFeedback.irq(trigger=machine.Pin.IRQ_RISING, handler=UpdatePosition)


# home actuator
# actuatorDirection.value(retract)
# writeActuatorSpeed(80)
# while True:
#     pos_last = pos
#     time.sleep(0.25)
#     if pos_last == pos:
#         pos = 0
#         writeActuatorSpeed(0)
#         break


while True:

    position = 0
    pos_last = 0
    destination = input("Enter % distance to move actuator from (-100 -> 100) : ")
    if abs(float(destination)) < 0.1:
        break
    destination = float(destination)/100*475
    pid = PID(1, 0.1, 0.05, setpoint=destination)
    pid.sample_time = 0.93 # pico cant process faster than this...
    pid.output_limits = (-70, 70)
    count = 0

    while True:
        speed = pid(position)
        print(speed)
        if speed > 0:
            actuatorDirection.value(extend)
        else:
            actuatorDirection.value(retract)
        writeActuatorSpeed(abs(speed))


        pos_last = position
        time.sleep(0.01)
        if pos_last == position:
            count = count + 1
            if count > 10:
                if abs(destination - position) < 2.5 or count > 50:
                    writeActuatorSpeed(0)
                    print("break")
                    break
        else:
            count = 0

    time.sleep(1)



state = input("Is the line stack state falling |::..| or rising |..::| when viewed from behind?  (Enter 1 for falling or 0 for rising) : ")

f = open('stacking_state.txt', 'w')
f.write(state)
f.close()
