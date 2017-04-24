#! /usr/bin/env python

import driver
import time
import sys
import signal
from multiprocessing import Process, Queue, Value,Array
from multiprocessing.queues import Queue as QueueType

from hokuyolx import HokuyoLX
import logging
import npParticle as pf
import numpy as np

lvl = logging.INFO
if sys.argv[-1][0] != "l":
    filename = 'Eurobot.log'
else:
    filename = sys.argv[-1]
    
logging.basicConfig(filename=filename, filemode='w', format='%(levelname)s:%(asctime)s %(message)s',
                    datefmt='%m/%d/%Y %I:%M:%S %p', level=lvl)

console = logging.StreamHandler()

def rev_field(val, color):
    if color == "blue":
        return [3000-val[0], val[1], (np.pi - val[2] + 2*np.pi)%(2*np.pi)] + val[3:]
    return val
    

## TODO: Defined according to mechanical situation, make from scrath to 90, 90, 180, 180 (left_up, right_up, left_down, right_down)
angle_left_up, angle_right_up, angle_left_down, angle_right_down = 80., 45., 175., 136.

console.setLevel(lvl)
# set a format which is simpler for console use
formatter = logging.Formatter('%(levelname)s: %(message)s')
console.setFormatter(formatter)
# add the handler to the root logger
logging.getLogger('').addHandler(console)
logger = logging.getLogger(__name__)

funny_done = 0
obstacles=[]
class Robot:
    def __init__(self, lidar_on=True, color = 'yellow', sen_noise = 20, 
        angle_noise=0.2, dist_noise = 45, init_coord = [170, 170, 0]):
        self.color = color
        self.sensor_range = 60
        self.collision_avoidance = False
        self.localisation = Value('b', True)
        #change for collision
        #self.sensors_map = {0:(0, np.pi/3),7:(7*np.pi/4, 2*np.pi),3: (np.pi*0.7, np.pi*1.3),1: (5/3.*np.pi,2*np.pi),2:(0,np.pi*1/4.),6:(7/4.*np.pi,2*np.pi),8:(0,np.pi/4),4:(np.pi/4,3*np.pi/4),5:(np.pi*5/4,7*np.pi/4)} #[(7/4.*np.pi,2*np.pi),(0,np.pi*1/4.)]
        self.sensors_map= {0: (np.pi/6, np.pi/2+np.pi/3), 1: (np.pi/2-np.pi/3, np.pi*5/6), 2: (0, np.pi/4), 3: (np.pi - np.pi/3, np.pi + np.pi/3), 4: (3*np.pi/2-np.pi/3,11*np.pi/6), 5: (np.pi-np.pi/6,3*np.pi/2+np.pi/3), 6:(3*np.pi/2 + np.pi/4, 2*np.pi)}  # can be problem with 2pi and 0
        self.lidar_on = lidar_on
        self.map = np.load('npmap.npy')
        if lidar_on:
            logging.debug('lidar is connected')
            # add check for lidar connection
            try:
                self.lidar = HokuyoLX(tsync=False)
                self.lidar.convert_time = False
            except:
                self.lidar_on = False
                self.localisation = Value('b', False)
                logging.warning('lidar is not connected')
        #self.x = 170  # mm
        #self.y = 150  # mm
        #self.angle = 0.0  # pi
        driver.PORT_SNR = '325936843235' # need change
        # for test
        x1, x2, y1, y2 = 1000, 2500, 600, 1100
        dx, dy = x2-x1, y2-y1
        angle = np.pi/2
        start = [x1, y1, angle]
        #
        self.coords = Array('d', rev_field(init_coord, self.color))# 170, 170
        #self.coords = Array('d',rev_field([1000, 1100, np.pi],self.color))
        self.input_queue = Queue()
        self.loc_queue = Queue()
        self.fsm_queue = Queue()
        self.PF = pf.ParticleFilter(particles=6000, sense_noise=sen_noise, 
            distance_noise=dist_noise, angle_noise=angle_noise, 
            in_x=self.coords[0], in_y=self.coords[1], 
            in_angle=self.coords[2],input_queue=self.input_queue, 
            out_queue=self.loc_queue, color = self.color)
        # driver process
        self.dr = driver.Driver(self.input_queue,self.fsm_queue,self.loc_queue)
        self.p = Process(target=self.dr.run)
        self.p.start()
        self.p2 = Process(target=self.PF.localisation,args=(self.localisation,self.coords,self.get_raw_lidar))
        logging.info(self.send_command('echo','ECHO'))
        logging.info(self.send_command('setCoordinates',[self.coords[0] / 1000., self.coords[1] / 1000., self.coords[2]]))
        self.p2.start()
        self.init_manipulators()
        time.sleep(0.1)
        logging.info("Robot __init__ done!")

    def send_command(self,name,params=None):
        self.input_queue.put({'source': 'fsm','cmd': name,'params': params})
        return self.fsm_queue.get()
        
    def is_start(self):
        return self.send_command('start_flag')['data']

    def get_raw_lidar(self):
        # return np.load('scan.npy')[::-1]
        timestamp, scan = self.lidar.get_intens()
        return scan
        # return scan[::-1]  our robot(old)

    def check_lidar(self):
        try:
            state = self.lidar.laser_state()
        except:
            self.lidar_on = False
            logging.warning('Lidar off')

    def go_to_coord_rotation(self, parameters):  # parameters [x,y,angle,speed]
        parameters = rev_field(parameters,self.color)
        if self.PF.warning:
            time.sleep(1)
        pm = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2]),
            parameters[0] / 1000., parameters[1] / 1000., float(parameters[2]), parameters[3]]
        x = parameters[0] - self.coords[0]
        y = parameters[1] - self.coords[1]
        sm = x+y
        logging.info("Go to coordinates: " + str(parameters[:2] +[np.rad2deg(parameters[2])]))
        logging.info(self.send_command('go_to_with_corrections',pm))
        #self.PF.debug_info = [time.time() - self.PF.start_time, parameters[:2]
        #                        +[np.rad2deg(parameters[2])] , [0]]
        # After movement
        stamp = time.time()
        pids = True
        time.sleep(0.100001)  # sleep because of STM interruptions (Maybe add force interrupt in STM)
        while not self.send_command('is_point_was_reached')['data']:
            time.sleep(0.01)
            if self.collision_avoidance:
                direction = (float(x), float(y))
                while self.check_collisions(direction):
                    logging.info("Collision made!")
                    if pids:
                        self.send_command('stopAllMotors')
                        pids = False
                    time.sleep(1)
                if not pids:
                    pids = True
                    logging.info(self.send_command('switchOnPid'))
                #return False
                # check untill ok and then move!
            # add Collision Avoidance there
            if (time.time() - stamp) > 30:
                return False  # Error, need to handle somehow (Localize and add new point maybe)
        if self.localisation.value == 0:
            self.PF.move_particles([parameters[0]-self.coords[0],parameters[1]-self.coords[1],parameters[2]-self.coords[2]])
            self.coords[0] = parameters[0]
            self.coords[1] = parameters[1]
            self.coords[2] = parameters[2]
        logging.info('point reached')
        return True

    def check_collisions(self, direction):
        angle = np.arctan2(direction[1],direction[0])%(np.pi*2)
        sensor_angle = (angle-self.coords[2]) %(np.pi*2)
        #### switch on sensor_angle
        collisions = self.sensor_data()
        for index,i in enumerate(collisions):
            if (i==True and sensor_angle<=self.sensors_map[index][1] and sensor_angle>=self.sensors_map[index][0]):
                logging.info("Collision at index "+str(index))
                if self.check_map(direction):
                    continue
                return True
        return False

    def receive_sensors_data(self):
        data = self.send_command('sensors_data')['data']
        answer = []
        for i in range(6):
            answer.append((data & (1 << i)) != 0)
        return answer

    def sensor_data(self):
        data = self.send_command('ir_sensors')['data']
        data.append(data[2])
        return data

    def check_map(self,direction): # probably can be optimized However O(1)
        direction = (direction[0]/np.sum(np.abs(direction)),direction[1]/np.sum(np.abs(direction)))
        for i in range(0, self.sensor_range, 2):
            for dx in range(-8,8):
                x = int(self.coords[0]/10+direction[0]*i+dx)
                y = int(self.coords[1]/10+direction[1]*i)
                if x > pf.WORLD_X/10 or x < 0 or y > pf.WORLD_Y/10 or y < 0:
                    return True
                    # Or maybe Continue
                if self.map[x][y]:
                    return True
        return False


    def go_last(self,parameters, dur = None):
        tmstmp = None
        if dur is not None:
            tmstmp = time.time()
        while abs(parameters[0]-self.coords[0]) > 10 or abs(parameters[1]-self.coords[1]) > 10:
            logging.info('Calibrate')
            self.go_to_coord_rotation(parameters)
            if tmstmp is not None and time.time() - tmstmp > dur:
                break
        
    
    ##########################################################
    ################# BIG Robot ############################
    ##########################################################

    def left_ball_down(self, dur = 1, angle = angle_left_down, speed = 100.0):
        self.collision_avoidance = False
        self.send_command('left_ball_down', [angle, speed])
        logging.info("left_ball_down")
        time.sleep(dur)
        #self.collision_avoidance = True

    def left_ball_up(self, dur = 1, angle = angle_left_up, speed = 100.0):
        self.collision_avoidance = False
        self.send_command('left_ball_up', [angle, speed])
        logging.info("left_ball_up")
        time.sleep(dur)
        #self.collision_avoidance = True

    def left_ball_drop(self, dur = 1, angle = angle_left_up + 33., speed = 100.0):
        self.collision_avoidance = False
        self.send_command('left_ball_drop', [angle, speed])
        logging.info("left_ball_drop")
        time.sleep(dur)
        #self.collision_avoidance = True 

    def right_ball_down(self, dur = 1, angle = angle_right_down, speed = 100.):
        self.collision_avoidance = False
        self.send_command('right_ball_down', [angle, speed])
        logging.info("right_ball_down")
        time.sleep(dur)
        #self.collision_avoidance = True

    def right_ball_up(self, dur = 1, angle = angle_right_up, speed = 100.):
        self.collision_avoidance = False
        self.send_command('right_ball_up', [angle, speed])
        logging.info("right_ball_up")
        time.sleep(dur)
        #self.collision_avoidance = True

    def right_ball_drop(self, dur = 1, angle = angle_right_up + 35., speed = 100.0):
        rselfcollision_avoidance = False
        self.send_command('right_ball_drop', [angle, speed])
        logging.info("right_ball_drop")
        time.sleep(dur)
        #self.collision_avoidance = True

    ### servos for cylinder take
    def front_down_cylinder_no(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('front_down_cylinder_no')
        time.sleep(dur)
        #self.collision_avoidance = True

    def front_up_cylinder_yes(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('front_up_cylinder_yes')
        time.sleep(dur)
        #self.collision_avoidance = True

    def front_drop_cylinder_yes(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('front_drop_cylinder_yes')
        time.sleep(dur)
        #self.collision_avoidance = True

    def back_down_cylinder_no(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('back_down_cylinder_no')
        time.sleep(dur)
        #self.collision_avoidance = True

    def back_up_cylinder_yes(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('back_up_cylinder_yes')
        time.sleep(dur)
        #self.collision_avoidance = True

    def back_drop_cylinder_yes(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('back_drop_cylinder_yes')
        time.sleep(dur)
        #self.collision_avoidance = True

    # sticks to use
    def both_sticks_open(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('both_sticks_open')
        #self.collision_avoidance = True
        time.sleep(dur)
        #self.collision_avoidance = True

    def both_sticks_close(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('both_sticks_close')
        time.sleep(dur)
        #self.collision_avoidance = True
    
    ### seesaw manipulator
    def seesaw_hand_down(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('seesaw_hand_down')
        logging.info("seesaw_hand_down")
        time.sleep(dur)
        #self.collision_avoidance = True
        
    def seesaw_hand_up(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('seesaw_hand_up')
        logging.info("seesaw_hand_up")
        time.sleep(dur)    
        #self.collision_avoidance = True
    
    # funny action
    def funny_action(self, signum, frame):
        #self.send_command('stopAllMotors')
        #self.send_command('funny_action_open')
        funny_done = 1
        logging.info('FUNNY ACTION')
        raise Exception

    def lin_interpol_traj(x1, y1, x2, y2, ratio):
        return [x1+(x2-x1)*ratio, y1+(y2-y1)*ratio]
    

    ############################################################################
    ######## HIGH LEVEL FUNCTIONS ##############################################
    ############################################################################

    def init_manipulators(self, dur = 1):
        self.left_ball_up(dur=dur)
        self.right_ball_up(dur=dur)
        self.seesaw_hand_up(dur=0.1)


    def first_trajectory(self, speed = 4, mode=1):
        self.collision_avoidance = False #True
        if mode == 1:
            speed = 1
            angle = np.pi/2
            parameters = [950, 200, angle, speed]
            self.go_to_coord_rotation(parameters)
        elif mode == 2:
            speed = 1
            angle = np.pi/6
            if self.color == "yellow":
                angle = np.pi/6
            else:
                angle = 2*np.pi - np.pi/6
            self.collision_avoidance = False
            #self.localisation.value = False
            parameters = [900, 170, angle, speed]
            self.go_to_coord_rotation(parameters)
            #self.localisation.value = True
            #parameters = [900, 170, angle, speed]
            #self.go_to_coord_rotation(parameters)
        else:
            peed = 1
            angle = np.pi/2
            parameters = [950, 200, angle, speed]
            self.go_to_coord_rotation(parameters)
        
        angle = np.pi/2
        if mode == 1:
            speed = 0
        else:
            speed = 1
        parameters = [900, 700, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = False #True
        parameters = [740, 1000, angle, 0]
        self.go_to_coord_rotation(parameters)
        #self.go_last(parameters, dur = 4)
        #self.localisation.value = 
        speed = 1
        parameters = [640, 1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        speed = 4
        parameters = [500, 1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.left_ball_down()
        if self.color == "yellow":
            self.left_ball_down()
        else:
            self.right_ball_down()
        parameters = [640, 1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.left_ball_up()
        dur = 0.3
        if self.color == "yellow":
            self.left_ball_up(dur=dur)
        else:
            self.right_ball_up(dur=dur)
        speed = 1
        angle = np.pi/2
        parameters = [640, 2000-450, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        ## TODO: add new point to take cylinder front! Then go as usually
        time.sleep(5)
        ## End TODO
        angle = np.pi + np.pi/4
        #parameters = [640, 2000-450, angle, speed]
        #self.go_to_coord_rotation(parameters)
        speed = 4
        parameters = [320, 2000-400, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.right_ball_down()
        dur = 1
        if self.color == "yellow":
            self.right_ball_down(dur=dur)
        else:
            self.left_ball_down(dur=dur)
        self.collision_avoidance = False
        parameters = [440, 2000-570, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = False#True
        #self.right_ball_up(dur = 0.5)
        dur = 0.3
        if self.color == "yellow":
            self.right_ball_up(dur = dur)
        else:
            self.left_ball_up(dur = dur)
        
        ## TODO: place another point to drop clinder in lateral slot, or place it near DESAs robot somwhere
        time.sleep(7)
        ##
        speed = 1
        #angle = 6*np.pi/7
        #parameters = [965, 1100, angle,0] # 865 1100
        #self.go_to_coord_rotation(parameters)
        angle = np.pi/2
        parameters = [965, 1100, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        self.collision_avoidance = False#True
        speed = 1
        parameters = [900, 230, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/10
        if self.color == "yellow":
            angle = np.pi/10#np.pi/6
        else:
            angle = 2*np.pi - np.pi/10
        parameters = [880, 240, angle, speed]
        self.go_last(parameters, dur=3)
        parameters = [870, 240, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.seesaw_hand_down()
        self.collision_avoidance = False
        #self.localisation.value = False
        parameters = [235, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.seesaw_hand_up()
        #self.localisation.value = True
        self.collision_avoidance = False#True
        angle = 0.0
        parameters = [205, 280, angle, speed]
        self.go_last(parameters, dur = 3)
        ## BOARDER localisation can be pasted
        ## end
        #self.right_ball_drop()
        #self.right_ball_up()
        if self.color == "yellow":
            self.right_ball_drop()
            self.right_ball_up()
        else:
            self.left_ball_drop()
            self.left_ball_up()
            
        speed = 1
        angle = 0.0
        parameters = [200, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi
        parameters = [200, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 1
        parameters = [205, 280, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.left_ball_drop()
        #self.left_ball_up()
        if self.color == "yellow":
            self.left_ball_drop()
            self.left_ball_up()
        else:
            self.right_ball_drop()
            self.right_ball_up()
        
        # TODO: go once more
        speed = 1
        angle = np.pi + np.pi/6
        if self.color == "yellow":
            angle = np.pi + np.pi/6#np.pi/6
        else:
            angle = 2*np.pi - np.pi - np.pi/6
        parameters = [170, 170, angle, speed]
        rb.go_last(parameters, dur = 2)
        speed = 1
        self.collision_avoidance = False
        #self.localisation.value = False
        parameters = [800, 170, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.localisation.value = True
        self.collision_avoidance = False#True
        angle = 3*np.pi/2
        parameters = [875, 550, angle, speed]
        self.go_last(parameters, dur = 4)
        self.collision_avoidance = False#True
        #self.right_ball_down(dur = 1)
        #self.right_ball_up(dur = 0.2)
        if self.color == "yellow":
            self.right_ball_down(dur = 1.5, angle=angle_right_down+10)
            self.right_ball_up(dur = 0.2)
        else:
            self.left_ball_down(dur = 1)
            self.left_ball_up(dur = 0.2)
        
        #####################
        if True:
            speed = 1
            angle = 3*np.pi/2
            parameters = [860, 200, angle, speed]
            self.go_to_coord_rotation(parameters)
            #self.right_ball_drop()
            #self.right_ball_up()
            if self.color == "yellow":
                self.right_ball_drop()
                self.right_ball_up()
            else:
                self.left_ball_drop()
                self.left_ball_up()
            return
        
        #####################################
        if False:
            parameters = [1000, 280, angle, speed]
            self.go_to_coord_rotation(parameters)
            angle = np.pi/18
            if self.color == "yellow":
                angle = np.pi/18
            else:
                angle = 2*np.pi - np.pi/18
            parameters = [880, 230, angle, speed]
            self.go_last(parameters, dur = 4)
            self.seesaw_hand_down()
            self.collision_avoidance = False
            #self.localisation.value = False
            parameters = [235, 230, angle, speed]
            self.go_last(parameters)
            self.seesaw_hand_up()
            #self.localisation.value = True
            #self.collision_avoidance = True
            angle = 0.0
            parameters = [235, 250, angle, speed]
            self.go_to_coord_rotation(parameters)
            self.go_last(parameters, dur = 1)
            return
            ## BOARDER localisation can be pasted
            ## end
            if self.color == "yellow":
                self.right_ball_drop()
                self.right_ball_up()
            else:
                self.left_ball_drop()
                self.left_ball_up()
            return
            
    def first_traverse_trajectory(self,speed = 4):
        speed = 1
        angle = np.pi/6
        if self.color == "yellow":
            angle = np.pi/6
        else:
            angle = 2*np.pi - np.pi/10
        self.collision_avoidance = False
        #self.localisation.value = False
        parameters = [950, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.localisation.value = True
        #parameters = [1000, 170, angle, speed]
        #self.go_to_coord_rotation(parameters)
        angle = np.pi/2
        parameters = [950, 600, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.collision_avoidance = True

        parameters = [640, 1000, angle, speed]
        self.go_last(parameters, dur = 4)
        parameters = [640, 1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/2
        speed = 4
        parameters = [500, 1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.left_ball_down()
        if self.color == "yellow":
            self.left_ball_down()
        else:
            self.right_ball_down()
        parameters = [640, 1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.left_ball_up()
        if self.color == "yellow":
            self.left_ball_up()
        else:
            self.right_ball_up()
        angle = np.pi/2
        speed = 1
        parameters = [640, 2000-450, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 1
        angle = np.pi + np.pi/4
        parameters = [640, 2000-450, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 4
        parameters = [320, 2000-400, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.right_ball_down()
        if self.color == "yellow":
            self.right_ball_down()
        else:
            self.left_ball_down()
        self.collision_avoidance = False
        parameters = [440, 2000-570, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.collision_avoidance = True
        #self.right_ball_up(dur = 0.5)
        if self.color == "yellow":
            self.right_ball_up(dur = 0.5)
        else:
            self.left_ball_up(dur = 0.5)
        
        speed = 1
        angle = 6*np.pi/7
        parameters = [865, 1100, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/2
        parameters = [865, 1100, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        #self.collision_avoidance = True
        speed = 1
        angle = 3*np.pi/2
        parameters = [865, 210, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/18
        if self.color == "yellow":
            angle = np.pi/18#np.pi/6
        else:
            angle = 2*np.pi - np.pi/18
        parameters = [860, 250, angle, speed]
        self.go_last(parameters, dur = 4)
        self.seesaw_hand_down()
        self.collision_avoidance = False
        #self.localisation.value = False
        parameters = [235, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.seesaw_hand_up()
        #self.localisation.value = True
        #self.collision_avoidance = True
        angle = 0.0
        parameters = [225, 280, angle, speed]
        self.go_last(parameters, dur = 3)
        ## BOARDER localisation can be pasted
        ## end
        #self.right_ball_drop()
        #self.right_ball_up()
        if self.color == "yellow":
            self.right_ball_drop()
            self.right_ball_up()
        else:
            self.left_ball_drop()
            self.left_ball_up()

        speed = 1
        angle = 0.0
        parameters = [200, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi
        parameters = [200, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 1
        parameters = [225, 280, angle, speed]
        self.go_last(parameters, dur = 2)
        #self.left_ball_drop()
        #self.left_ball_up()
        if self.color == "yellow":
            self.left_ball_drop()
            self.left_ball_up()
        else:
            self.right_ball_drop()
            self.right_ball_up()
        
        # TODO: go once more
        speed = 4
        angle = np.pi + np.pi/6
        if self.color == "yellow":
            angle = np.pi + np.pi/6#np.pi/6
        else:
            angle = 2*np.pi - np.pi - np.pi/6
        parameters = [170, 170, angle, speed]
        rb.go_last(parameters, dur = 1)
        speed = 1
        self.collision_avoidance = False
        #self.localisation.value = False
        parameters = [1000, 170, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.localisation.value = True
        #self.collision_avoidance = True
        angle = 3*np.pi/2
        parameters = [1000, 550, angle, speed]
        self.go_last(parameters, dur = 3)
        #self.collision_avoidance = True
        #self.right_ball_down(dur = 1)
        #self.right_ball_up(dur = 0.2)
        if self.color == "yellow":
            self.right_ball_down(dur = 1)
            self.right_ball_up(dur = 0.2)
        else:
            self.left_ball_down(dur = 1)
            self.left_ball_up(dur = 0.2)
        
        
        speed = 1
        angle = 3*np.pi/2
        parameters = [800, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.localisation.value = True
        #self.right_ball_drop()
        #self.right_ball_up()
        if self.color == "yellow":
            self.right_ball_drop()
            self.right_ball_up()
        else:
            self.left_ball_drop()
            self.left_ball_up()
        return
        
        
    
        
    def loc_test2(self,  x1x2y1y2 = [500, 1500, 1000, 1000], speed=4, 
                    angle=np.pi, n_times=3, deltas = 1):
        x1, x2, y1, y2 = x1x2y1y2#1000, 2500, 600, 1100
        dx, dy = x2-x1, y2-y1
        if angle is None:
            angle = np.arctan2(dy, dx)
        start = [x1, y1, angle]
        for i in xrange(n_times):
            for j in xrange(deltas+1):
                parameters = [start[0] + j*dx/deltas, start[1] + j*dy/deltas, start[2], speed]
                self.go_to_coord_rotation(parameters)
        parameters = [start[0], start[1], start[2], speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)


    def loc_test(self, speed=4, angle=np.pi/2, n_times=3):
        x1, x2, y1, y2 = 1000, 2500, 600, 1100
        dx, dy = x2-x1, y2-y1
        #angle = np.pi/2
        start = [x1, y1, angle]
        for i in xrange(n_times):
            parameters = [start[0], start[1], start[2] + 0*np.pi/2, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [start[0], start[1], start[2] + 1*np.pi/2, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [start[0], start[1], start[2] + 2*np.pi/2, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [start[0], start[1], start[2] + 3*np.pi/2, speed]
            self.go_to_coord_rotation(parameters)
            #parameters = [start[0], start[1], start[2] + 4*np.pi/2, speed]
            #self.go_to_coord_rotation(parameters)
        parameters = [start[0], start[1], start[2], speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        
        

    def localisation_test(self, point_lst = [[500, 1100]],
                          angle_lst = [0], speed_lst = [4], n_times = 1):
        odometry_coords = []
        #self.coords = point_lst[0] + [angle_lst[0]]
        #self.send_command('setCoordinates',
        #                  [self.coords[0] / 1000.,
        #                   self.coords[1] / 1000.,
        #                   self.coords[2]])
        for i in xrange(n_times):
            a = []
            for angle in angle_lst:
                b = []
                for speed in speed_lst:
                    c = []
                    for point in point_lst+point_lst[-2:0:-1]:
                        parameters = point + [angle, speed]
                        self.go_to_coord_rotation(parameters)
                        coords = self.send_command('getCurrentCoordinates')['data']
                        # check for invalid
                        coords[0] = coords[0]*1000
                        coords[1] = coords[1]*1000
                        c += [coords]
                    b += [c]
                a += [b]
            odometry_coords += [a]
        return odometry_coords

    
    def collision_test(self,speed=1):
        angle = np.pi/2
        while True:
            parameters = [1145, 400, angle, speed]
            t = self.go_to_coord_rotation(parameters)

            parameters = [945, 600, angle, speed]
            t = self.go_to_coord_rotation(parameters)


            parameters = [1145, 800, angle, speed]
            t = self.go_to_coord_rotation(parameters)

            parameters = [1345, 1000, angle, speed]
            t = self.go_to_coord_rotation(parameters)


            parameters = [1545, 800, angle, speed]
            t = self.go_to_coord_rotation(parameters)

            parameters = [1745, 600, angle, speed]
            t = self.go_to_coord_rotation(parameters)

            parameters = [1545, 400, angle, speed]
            t = self.go_to_coord_rotation(parameters)

rb = None
tmstmp = None

def test():
    global rb
    global tmstmp

    # Color 
    if len(sys.argv) > 1:
        color = sys.argv[1]
    else:
        color == "yellow"
    # Strategy mode
    if len(sys.argv) > 2:
        mode = int(sys.argv[2])
    else:
        mode = 1
    # initialize coordinates from this two parameters
    init_coordinates = {(1, 'y'): [950, 200, np.pi/2],
                        (1, 'b'): [950, 200, np.pi/2],
                        (2, 'y'): [170., 170., np.pi + np.pi/6],
                        (2, 'b'): [170., 200., 2*np.pi - np.pi/10]}
    init_coord = init_coordinates[(mode, color[0])]
    # Init robot
    rb = Robot(lidar_on=True, color=color, sen_noise=25., 
        dist_noise=35., angle_noise=0.2, init_coord=init_coord)
    if False:
        while not rb.is_start():
            continue
        while 1:
            rb.left_ball_drop()
            rb.left_ball_up()
            rb.right_ball_drop()
            rb.right_ball_up()
    ########### Various tests ###########
    if False: # collision avoidance and localisation test
        rb.collision_avoidance = False
        rb.collision_test(1)
        return
    if False: # inplace test for localisation
        parameters = init_coord + [4]
        rb.go_last(parameters)
        return
    if False: # 90 seconds funny action  (on STM) test
        rb.send_command('funny_action_open')
        time.sleep(2)
        rb.send_command('funny_action_close')
        while 1:
            rb.right_ball_down()
            rb.right_ball_up()
            rb.right_ball_drop()
            rb.right_ball_up()
        #rb.funny_action(None, None)
        return
    ####################################
    # Funny action prepare
    if len(sys.argv) == 1 or len(sys.argv) > 3 and sys.argv[3][0] == "f":
        rb.send_command('funny_action_open') 
        time.sleep(2)
        rb.send_command('funny_action_close')
    # All system tests (all in all)
    if len(sys.argv) == 1 or len(sys.argv) > 4 and sys.argv[4][0] == "m":
        while not rb.is_start():
            continue
        rb.right_ball_up()#(angle = angle_right_up)
        rb.right_ball_down()#(angle = angle_right_down)
        rb.right_ball_up()#(angle = angle_right_up)
        rb.right_ball_drop()#(angle = angle_right_up + 35.)
        rb.right_ball_up()#(angle = angle_right_up)
        rb.left_ball_up()#(angle = angle_left_up)
        rb.left_ball_down()#(angle = angle_left_dwon)
        rb.left_ball_up()#(angle = angle_left_up)
        rb.left_ball_drop()#(angle = angle_right_up + 33.)
        rb.left_ball_up()#(angle =() angle_left_down)
        rb.seesaw_hand_down()
        rb.seesaw_hand_up()
        #rb.front_down_cylinder_no()
        #rb.front_up_cylinder_yes()
        #rb.back_down_cylinder_no()
        rb.back_up_cylinder_yes()
        signal.signal(signal.SIGALRM, rb.funny_action)
        signal.alarm(3)
        parameters = [500, 500, 0., 4]
        rb.go_to_coord_rotation(parameters)
        return
    if True:
        while not rb.is_start():
            continue
    logging.info("Start Game!")
    tmstmp = time.time()
    signal.signal(signal.SIGALRM, rb.funny_action)
    signal.alarm(90)
    if len(sys.argv) == 1 or len(sys.argv) > 1:
        if len(sys.argv) == 1 or sys.argv[1][0] != "b":
            rb.color = "yellow"
            rb.first_trajectory(mode=mode)
            return
        else:
            rb.color = "blue"
            #rb.first_traverse_trajectory(mode=mode)
            rb.first_trajectory(mode=mode)
            return

    
    
try:
    test()
except KeyboardInterrupt:
    logging.exception('KeyboardInterrupt!')
except Exception:
    logging.exception('High level Error!')
    #rb.PF.debug_info += [time.time() - tmstmp, [], []]
    #np.savetxt("localisation_debug.txt", np.array(rb.PF.debug_info))
finally:
    #while not funny_done:
    #    continue
    logging.info("Time for strategy passes:  " + str(time.time() - tmstmp))
    logging.exception("Finally done")
    rb.p.terminate()
    rb.p2.terminate()
    



