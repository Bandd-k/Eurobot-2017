import stmDriver
import multiprocessing
import time
import sys
import vk_api
sys.path.insert(0,'../localisation')
import ParticleFilter as pf
reply_to_fsm_queue = multiprocessing.Queue()
input_command_queue = multiprocessing.Queue()
from hokuyolx import HokuyoLX
import matplotlib.pyplot as plt




vk = None


def init_vk():
    global vk

    vk_session = vk_api.VkApi(login, password)
    try:
        vk_session.authorization()
    except vk_api.AuthorizationError as error_msg:
        print(error_msg)
        return

    vk = vk_session.get_api()

    #response = vk.messages.send(user_id =22100673 ,message="hi")


def send_message(msg):
    #vk.messages.send(user_id=2117805, message=msg)
    response = vk.messages.send(user_id=22100673, message=msg)


prev_id = 0
def get_command():
    global prev_id
    response = vk.messages.get(count=1)['items'][0]
    answer = []
    if response['user_id'] == 22100673 and response['id'] != prev_id:
        prev_id = response['id']
        for i in response['body'].split():
            answer.append(float(i))
        answer.append(4)
        return answer
    return False

# Function add stm_task to queue
def stm_driver(command, parameters = ''):
    command = {'request_source': 'fsm', 'command': command, 'parameters': parameters}
    input_command_queue.put(command)
    return reply_to_fsm_queue.get()




# stm started


# use stm_driver to send command
# go to point
#
def ManipulatorsTest():
    print (stm_driver('echo'))
    stm_driver('switch_on_pneumo')
    stm_driver('move_out')
    time.sleep(1)

    stm_driver('move_in')
    stm_driver('move_180')
    time.sleep(1)
    stm_driver('switch_off_pneumo')
    stm_driver('move_90')

#set start points
#parameters = [0,0,0]
#stm_driver('set_coordinates_without_movement',parameters)

# go to 100,100
#parameters = [0, 0, 3.14, 4]
#stm_driver('go_to_global_point',parameters)
#time.sleep(2)
#print stm_driver('is_point_was_reached')

#is_point_was_reached

#go_to_global_point(parameters)

#particles = None
#main_robot = None
#lidar = HokuyoLX(tsync=False)
#lidar.convert_time = False

def lidar_sense():
    """Function returns landmark data"""
    timestamp, scan = lidar.get_intens()
    # distances to landmarks(up to 3)!
    ans = []
    for i in pf.get_beacons(scan):
        ans.append(i[1]+40)
    return ans


def movements():
    stm = multiprocessing.Process(target=stmDriver.stm_loop, args=(input_command_queue, reply_to_fsm_queue))
    stm.start()
    init_vk()
    parameters = [0, 0, 0]
    particles = [pf.Robot() for i in range(pf.particle_number)]
    main_robot = pf.calculate_main(particles)
    send_message(str(main_robot))
    stm_driver('set_coordinates_without_movement', parameters)
    ax = plt.subplot(111)
    plot = ax.plot([], [], 'ro')[0]
    plt.axis([0, 2000, 0, 3000])
    plt.show()
    cord = [0,0]
    plot.set_data(cord[0], cord[1])
    # make movement
    parameters = [1000, 0, 0, 4]
    make_mov(parameters,parameters)
    cord = [main_robot.x,main_robot.y]
    plot.set_data(cord[0], cord[1])
    print main_robot


def make_mov(parameters,particles):
    pm = [parameters[0]/1000.,parameters[1]/1000.,parameters[2],parameters[3]]

    stm_driver('go_to_global_point', pm)
    stamp = time.time()
    while not stm_driver('is_point_was_reached'):
        time.sleep(0.3)
        if (time.time() - stamp) > 30:
            return False
    particles = pf.particles_move(particles, parameters[:-1])
    lidar_data = lidar_sense()
    particles = pf.particles_sense(particles, lidar_data)
    main_robot = pf.calculate_main(particles)
    send_message(str(main_robot))
    return True


def test():
    #init_vk()
    #send_message(str(pf.Robot()))
    # stm started
    stm = multiprocessing.Process(target=stmDriver.stm_loop, args=(input_command_queue,reply_to_fsm_queue))
    stm.start()


    # use stm_driver to send command
    print (stm_driver('echo'))
    #go to point

    # set start points
    parameters = [0,0,0]
    stm_driver('set_coordinates_without_movement',parameters)

    # go to 100,100

    parameters = [0, 0, 3.14, 4]
    stm_driver('go_to_global_point', parameters)




#test()
#print lidar_sense()
#init_vk()
#print get_command()
#response = vk.messages.get(count=1)['items'][0]
#print  response

# t = pf.Robot()
# send_message(str(t))
# parameters = [10,0,0]
# t=t.move(parameters)
# send_message(str(t))
# parameters = [0,10,0]
# t= t.move(parameters)
# send_message(str(t))
# parameters = [0,0,3.14]
# t = t.move(parameters)
# send_message(str(t))
# stm = multiprocessing.Process(target=stmDriver.stm_loop, args=(input_command_queue, reply_to_fsm_queue))
# stm.start()
# print stm_driver('echo')
# parameters = [0,0,0]
# stm_driver('set_coordinates_without_movement',parameters)
#
# parameters = [0.5,0,0,4]
# print stm_driver('go_to_global_point', parameters)
# while not stm_driver('is_point_was_reached'):
#     time.sleep(0.1)
# parameters = [0.5,0,3.15,4]
# print stm_driver('go_to_global_point', parameters)
# while not stm_driver('is_point_was_reached'):
#     time.sleep(0.1)
# parameters = [0,0,-0.1,4]
# print stm_driver('go_to_global_point', parameters)
#print stm_driver('is_point_was_reached')
#while not stm_driver('is_point_was_reached'):
#     time.sleep(0.1)
#print "Povernul"

#print stm_driver('move_90')
#time.sleep(2)

#print stm_driver('move_180')
#stm_driver('move_out')
#stm_driver('move_out')
#ManipulatorsTest()
#parameters = [0,0,0]
#ans = get_command()
#stm_driver('set_coordinates_without_movement',parameters)
#while True:
#    ans = get_command()
#    if ans:
#        stm_driver('go_to_global_point', ans)

def final():
    stm = multiprocessing.Process(target=stmDriver.stm_loop, args=(input_command_queue, reply_to_fsm_queue))
    stm.start()
    print stm_driver('echo')
    parameters = [0, 0, 0]
    stm_driver('set_coordinates_without_movement', parameters)
    stm_driver('move_180')
    stm_driver('switch_on_pneumo')

    parameters = [0.5, 0, 0, 4]
    print stm_driver('go_to_global_point', parameters)
    while not stm_driver('is_point_was_reached'):
        time.sleep(0.1)
    parameters = [0.5, 0, 3.15, 4]
    print stm_driver('go_to_global_point', parameters)
    while not stm_driver('is_point_was_reached'):
        time.sleep(0.1)
    parameters = [0, 0, -0.3, 4]
    print stm_driver('go_to_global_point', parameters)
    stm_driver('move_90')
    time.sleep(10)
    stm_driver('switch_off_pneumo')


stm = multiprocessing.Process(target=stmDriver.stm_loop, args=(input_command_queue, reply_to_fsm_queue))
stm.start()
#print stm_driver('echo')
parameters = [0, 0, 0]
stm_driver('set_coordinates_without_movement', parameters)
parameters = [0.05, 0, 0, 4]
print stm_driver('go_to_global_point', parameters)
