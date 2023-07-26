import ROVwinch
import time

while True:
    try:
        ROVwinch.control_winch()
    except Exception as e:
        print(e)
        time.sleep(15)
        
    