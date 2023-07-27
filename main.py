import ROVwinch
import time
import argparse

# parse arguements
argParser = argparse.ArgumentParser()
argParser.add_argument("-m", "--mode", help=" mode of operation. Options: debug, deploy", default='deploy')
args = argParser.parse_args()

while True:
    try:
        ROVwinch.control_winch(args.mode)
    except Exception as e:
        print(e)
        time.sleep(15)
        
    