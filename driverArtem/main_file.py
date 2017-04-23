import  cmd_list
import  driver
import  packets

def stm_test():
    d = driver.Driver(1, 2, 3)
    d.connect()
    command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
    print d.process_cmd(command)


stm_test()

