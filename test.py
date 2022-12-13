import _thread
import time

baton = _thread.allocate_lock()

print("hello world")

def th_func(string_input, id):
    global j
    baton.acquire()
    print(string_input)
    time.sleep(2)
    j = j + 1
    print("secondary loops:", j)
    baton.release()




i = 0
j = 0
while True:
    i = i + 1
    time.sleep(1)
    _thread.start_new_thread(th_func, ("hi :)", 0) )
    print("main loops:", i)
