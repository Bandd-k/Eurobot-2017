
#! /usr/bin/env python

import driver
import time
from hokuyolx import HokuyoLX
import logging
import signal
import npParticle as pf
import numpy as np
import sys
from multiprocessing import Process, Queue, Value,Array
from flask import Flask,jsonify
import requests
import random
lvl = logging.INFO
fl_log = 'Eurobot.log'
if sys.argv[-1][:2] == "lo":
    fl_log = sys.argv[-1]
logging.basicConfig(filename=fl_log, filemode='w', format='%(levelname)s:%(asctime)s %(message)s',
                    datefmt='%m/%d/%Y %I:%M:%S %p', level=lvl)

console = logging.StreamHandler()

funny_done = 0
funny_delay = 5

def rev_field(val, color):
    if color == "blue":
        return [3000-val[0], val[1], (2*np.pi - val[2])%(2*np.pi)] + val[3:] ## was np.pi - ...
    return val

console.setLevel(lvl)
# set a format which is simpler for console use
formatter = logging.Formatter('%(levelname)s: %(message)s')
console.setFormatter(formatter)
# add the handler to the root logger
logging.getLogger('').addHandler(console)
logger = logging.getLogger(__name__)

# sharing coords
app = Flask(__name__)
@app.route('/coords')
def get_coords():
    return str(rb.coords[0])+" "+ str(rb.coords[1])+" "+ str(rb.coords[2])

@app.route('/stop')
def stop():
    print "stop from phone"
    rb.funny_action(1,2)


class Robot:
    def __init__(self, lidar_on=True, small=True, init_coord =[900, 200, np.pi/2], color = 'yellow'):
        # Cylinder Staff
        self.coll_go = False
        ##################
        self.color = color
        self.cur_state = 0 # 0-neutral,1-suck,2-throw
        self.sensor_range = 35
        self.collision_d = 16
        self.coll_ind = -1
        self.collision_avoidance = True
        self.localisation = Value('b', True)
        if small:
            self.sensors_places = [np.pi/2,0,3*np.pi/2,np.pi,3*np.pi/2,0,np.pi,np.pi/2,0,0]
            self.sensors_map = {1:(0,np.pi/4),5:(0,np.pi/4),7:(np.pi/4,3*np.pi/4),0:(np.pi/4,3*np.pi/4),6:(3*np.pi/4,5*np.pi/4),3:(3*np.pi/4,5*np.pi/4),2:(5*np.pi/4,7*np.pi/4),4:(5*np.pi/4,7*np.pi/4),8:(7*np.pi/4,2*np.pi),9:(7*np.pi/4,2*np.pi)}
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
        # self.x = 170  # mm
        # self.y = 150  # mm
        # self.angle = 0.0  # pi
        if small:
            #850 170 3p/2
            # 900 200 np.pi/2
            # 400, 850, 0.
            self.coords = Array('d',rev_field(init_coord, self.color))
        else:
            driver.PORT_SNR = '325936843235' # need change
            self.coords = Array('d', rev_field([170, 170, 0], self.color))
        self.input_queue = Queue()
        self.loc_queue = Queue()
        self.fsm_queue = Queue() # 2000,25,25,0.1
        self.PF = pf.ParticleFilter(particles=2000, sense_noise=25, distance_noise=30, angle_noise=0.15, in_x=self.coords[0], in_y=self.coords[1], in_angle=self.coords[2],input_queue=self.input_queue, out_queue=self.loc_queue,color = self.color)
        
        # coords sharing procces
        self.p3 = Process(target=app.run,args = ("0.0.0.0",))
        # driver process
        print "Paricle filter On"
        self.dr = driver.Driver(self.input_queue,self.fsm_queue,self.loc_queue)
        print "Driver On"
        self.p = Process(target=self.dr.run)
        print "Process driver Init"
        self.p.start()
        print "Process driver Start"
        self.p2 = Process(target=self.PF.localisation,args=(self.localisation,self.coords,self.get_raw_lidar))
        print "Process npParticle Init"
        logging.info(self.send_command('echo','ECHO'))
        logging.info(self.send_command('setCoordinates',[self.coords[0] / 1000., self.coords[1] / 1000., self.coords[2]]))
        if self.lidar_on:
            self.p2.start()
            print "Process npParticle Start"
        else:
            self.localisation = Value('b', False)
        print "Robot __init__ done!"

    def send_command(self,name,params=None):
        self.input_queue.put({'source': 'fsm','cmd': name,'params': params})
        return self.fsm_queue.get()

    def get_raw_lidar(self):
        # return np.load('scan.npy')[::-1]
        timestamp, scan = self.lidar.get_intens()
        return scan
        # return scan[::-1]  our robot(old)


    def second_robot_cords(self):
        r = requests.get("http://192.168.1.213:5000/coords")
        return ([float(i) for i in r.content.split()])

    def check_lidar(self):
        try:
            state = self.lidar.laser_state()
        except:
            self.lidar_on = False
            logging.warning('Lidar off')

    def go_to_coord_rotation(self, parameters):
        # beta version of clever go_to
        direct_random = [np.pi,np.pi/2,-np.pi/2]
        distance = 200

        # gomologization version and change timer in go_to
        self.coll_go = False
        ok = self.go_to(parameters)
        #return
        #
        #ok = self.go_to(parameters)
        while not ok:
            logging.critical("Collision, go back")
            angle = (self.coords[2] + self.sensors_places[self.coll_ind] +random.choice(direct_random)) %(np.pi*2)
            direction = (np.cos(angle),np.sin(angle))
            pm = [self.coords[0]+direction[0]*distance, self.coords[1]+direction[1]*distance, self.coords[2], parameters[3]]
            logging.critical("New point " + str(pm))
            self.go_to(pm)
            logging.critical("go to correct")
            self.coll_go = True
            ok = self.go_to(parameters)
            self.coll_go = False
        

    def go_to(self, parameters):  # parameters [x,y,angle,speed]
        parameters = rev_field(parameters,self.color)
        if self.PF.warning:
            time.sleep(1)
        pm = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2]),parameters[0] / 1000., parameters[1] / 1000., float(parameters[2]), parameters[3]]
        x = parameters[0] - self.coords[0]
        y = parameters[1] - self.coords[1]
        tm = 7
        if self.coll_go == True:
            tm = 1
        logging.info(self.send_command('go_to_with_corrections',pm))
        # After movement
        stamp = time.time()
        pids = True
        time.sleep(0.100001)  # sleep because of STM interruptions (Maybe add force interrupt in STM)
        filter_queue = [self.send_command('is_point_was_reached')['data'], 
                        self.send_command('is_point_was_reached')['data'],
                        self.send_command('is_point_was_reached')['data']]
        while not np.prod(filter_queue):
            time.sleep(0.05)
            if self.collision_avoidance:
                direction = (float(x), float(y))
                while self.check_collisions(direction):
                    if pids:
                        self.send_command('stopAllMotors')
                        pids = False
                    time.sleep(0.5)
                    if (time.time() - stamp) > tm:
                        self.send_command('cleanPointsStack')
                        cur = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2])]
                        self.send_command('setCoordinates',cur)
                        logging.info(self.send_command('switchOnPid'))
                        return False
            
                if not pids:
                    pids = True
                    self.send_command('cleanPointsStack')
                    cur = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2])]
                    self.send_command('setCoordinates',cur)
                    logging.info(self.send_command('switchOnPid'))
                    pm[0] = cur[0]
                    pm[1] = cur[1]
                    pm[2] = cur[2]
                    logging.info(self.send_command('go_to_with_corrections',pm))
                    time.sleep(0.10000001)
            filter_queue = filter_queue[1:] + [self.send_command('is_point_was_reached')['data']]
                # return False
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
                angle_map = (self.coords[2] + self.sensors_places[index]) %(np.pi*2) 
                if self.check_map2(angle_map):
                    continue
                self.coll_ind = index
                return True
        return False

    def receive_sensors_data(self):
        data = self.send_command('sensors_data')['data']
        answer = []
        for i in range(6):
            answer.append((data & (1 << i)) != 0)
        return answer

    def pre_sensor_data(self):
        data = self.send_command('sensors_data')['data']
        data.append(data[1])
        data.append(data[5])
        return np.array(data)

    def sensor_data(self):
        return self.pre_sensor_data()*self.pre_sensor_data()*self.pre_sensor_data()
        
    def check_map2(self,angle):
        direction = (np.cos(angle),np.sin(angle))
        for i in range(0, self.sensor_range, 2):
            for dx in range(-self.collision_d,self.collision_d):
                x = int(self.coords[0]/10+direction[0]*i+dx)
                y = int(self.coords[1]/10+direction[1]*i)
                #logging.info("x = "+str(x)+" y = " + str(y))
                if x >= pf.WORLD_X/10 or x <= 0 or y >= pf.WORLD_Y/10 or y <= 0:
                    return True
                    # Or maybe Continue
                if self.map[x][y]:
                    return True
        return False
        
        
        

    def check_map(self,direction): # probably can be optimized However O(1)
        direction = (direction[0]/np.sum(np.abs(direction)),direction[1]/np.sum(np.abs(direction)))
        for i in range(0, self.sensor_range, 2):
            for dx in range(-self.collision_d,self.collision_d):
                x = int(self.coords[0]/10+direction[0]*i+dx)
                y = int(self.coords[1]/10+direction[1]*i)
                #logging.info("x = "+str(x)+" y = " + str(y))
                if x > pf.WORLD_X/10 or x < 0 or y > pf.WORLD_Y/10 or y < 0:
                    return True
                    # Or maybe Continue
                if self.map[x][y]:
                    return True
        return False


    def go_last(self,parameters):
        pm = rev_field(parameters,self.color)
        while abs(pm[0]-self.coords[0]) > 10 or abs(pm[1]-self.coords[1]) > 10:
            print 'calibrate'
            self.go_to_coord_rotation(parameters)
            time.sleep(0.1)


    ##########################################################
    ################# Sucker Robot ############################
    ##########################################################


    def on_coolers_suck(self):
        logging.info(self.send_command('on_coolers_suck'))

    def on_coolers_throw(self):
        logging.info(self.send_command('on_coolers_throw'))

    def off_coolers(self):
        logging.info(self.send_command('off_coolers'))

    def on_mixer(self,direction = 1):
        logging.info(self.send_command('on_mixer',[direction]))

    def off_mixer(self):
        logging.info(self.send_command('off_mixer'))

    def up_front_seasaw(self):
        logging.info(self.send_command('up_front_seasaw'))

    def up_back_seasaw(self):
        logging.info(self.send_command('up_back_seasaw'))

    def down_front_seasaw(self):
        logging.info(self.send_command('down_front_seasaw'))

    def down_back_seasaw(self):
        logging.info(self.send_command('down_back_seasaw'))

    def open_door(self):
        logging.info(self.send_command('open_door'))

    def close_door(self):
        logging.info(self.send_command('close_door'))




    def is_start(self):
        return self.send_command('start_flag')['data']

    ############################################################################
    ######## HIGH LEVEL FUNCTIONS ##############################################
    ############################################################################

    def first(self,speed = 4):
        angle = np.pi/2
        self.on_mixer()
        self.suck()
        parameters = [950,550 ,angle, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [900,900 ,angle, 4]
        self.go_to_coord_rotation(parameters)
        self.stop()
        angle = np.pi/4
        parameters = [500,1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.suck()
        time.sleep(2)
        parameters = [300,1700, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 0.0
        parameters = [300,1700, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/2
        parameters = [300,1700, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/2
        parameters = [300,1700, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [920,1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [920,1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        time.sleep(1)
        #parameters = [600,1600, angle, speed]
        #self.go_to_coord_rotation(parameters)
        angle = 0.0
        parameters = [600,1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [600,1300, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.stop()
        return
    
    def first_back(self,speed = 4):
        # 500 1500 np.pi/2
        angle = 0.0
        parameters = [920, 900, angle, speed]
        self.go_to_coord_rotation(parameters)
        if False:
            angle = 3*np.pi/2
            parameters = [1000, 600, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [1500, 550, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [2120, 550, angle, speed]
            self.go_to_coord_rotation(parameters)
            self.suck()
            time.sleep(2)
            self.stop()
            parameters = [1500, 550, angle, speed]
            self.go_to_coord_rotation(parameters)
            angle = 0.0
            parameters = [1050, 550, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [950, 280, angle, speed]
            self.go_to_coord_rotation(parameters)
        else:
            parameters = [920, 250, angle, speed]
            self.go_to_coord_rotation(parameters)
        
        parameters = [850, 210, angle, speed]
        self.go_to_coord_rotation(parameters)
        if self.color == "yellow":
            self.down_back_seasaw()
        else:
            self.down_front_seasaw()
        time.sleep(1)
        parameters = [400, 210, angle, speed]
        self.go_to_coord_rotation(parameters)
        if self.color == "yellow":
            self.up_back_seasaw()
        else:
            self.up_front_seasaw()
        parameters = [200,210, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [215,170 , angle, speed]
        self.go_to_coord_rotation(parameters)
        self.throw()
        time.sleep(3)
        self.stop()
        time.sleep(0.2)
        self.on_mixer(0)
        self.suck()
        time.sleep(2)
        self.stop()
        time.sleep(0.2)
        self.throw()
        time.sleep(3)
        self.stop()
        self.off_mixer()
        time.sleep(40) # Check
        return

    def steal_strategy(self,speed = 4):
        angle = np.pi/2
        self.on_mixer()
        self.suck()
        parameters = [880,500 ,angle, 4]
        self.go_to_coord_rotation(parameters)
        
        if False:
            angle = 3*np.pi/2
            parameters = [880,500 ,angle, 4]
            self.go_to_coord_rotation(parameters)
            self.stop()
            parameters = [1500,500 ,angle, 4]
            self.go_to_coord_rotation(parameters)
            parameters = [2120,550 ,angle, 4]
            self.go_to_coord_rotation(parameters)
            self.suck()
            time.sleep(1)
            self.stop()
            parameters = [1000,550 ,angle, 4]
            self.go_to_coord_rotation(parameters)

        angle = np.pi/2
        parameters = [900,900 ,angle, 4]
        self.go_to_coord_rotation(parameters)
        self.stop()
        angle = np.pi/4
        parameters = [500,1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.suck()
        time.sleep(2)
        parameters = [300,1700, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 0.0
        parameters = [300,1700, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/2
        parameters = [300,1700, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/2
        parameters = [300,1700, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [920,1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [920,1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        time.sleep(1)
        #parameters = [600,1600, angle, speed]
        #self.go_to_coord_rotation(parameters)
        angle = 0.0
        parameters = [600,1300, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [600,1300, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.stop()
        return

    def steal_strategy2(self,speed = 4):
        angle = np.pi/2
        self.on_mixer()
        self.suck()
        parameters = [900,550 ,angle, 4]
        self.go_to_coord_rotation(parameters)
        
        if False:
            parameters = [900,550 ,angle, 4]
            self.go_to_coord_rotation(parameters)
            angle = 3*np.pi/2
            parameters = [1000,550 ,angle, 4]
            self.go_to_coord_rotation(parameters)
            self.stop()
            parameters = [1500,550 ,angle, 4]
            self.go_to_coord_rotation(parameters)
            parameters = [2120,550 ,angle, 4]
            self.go_to_coord_rotation(parameters)
            self.suck()
            time.sleep(2)
        
        self.stop()
        parameters = [1000,550 ,angle, speed]
        self.go_to_coord_rotation(parameters)
        #~ angle = 0.0
        #~ parameters = [900,900 ,angle, 4]
        #~ self.go_to_coord_rotation(parameters)
        #~ self.stop()
        #~ parameters = [250,1350, angle, speed]
        #~ self.go_to_coord_rotation(parameters)
        #~ self.suck()
        #~ time.sleep(1)
        #~ parameters = [250,1700, angle, speed]
        #~ self.go_to_coord_rotation(parameters)
        #~ angle = 3*np.pi/2
        #~ parameters = [250,1700, angle, speed]
        #~ self.go_to_coord_rotation(parameters)
        #~ parameters = [920,1800, angle, speed]
        #~ self.go_to_coord_rotation(parameters)
        #~ parameters = [920,1800, angle, speed]
        #~ self.go_to_coord_rotation(parameters)
        #~ time.sleep(1)
        angle = np.pi/6
        parameters = [550, 2000-480, angle, 1]
        self.go_to_coord_rotation(parameters)
        self.stop()
        self.suck()
        time.sleep(0.2)
        parameters = [300,2000-630 ,angle, 4]
        self.go_to_coord_rotation(parameters)
        #self.stop()
        #self.suck()
        #time.sleep(1)
        speed = 4
        parameters = [225,2000-440, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 0.0
        parameters = [225,2000-300, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 2*np.pi - np.pi/6
        parameters = [190, 2000-310, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 2*np.pi - np.pi/2 - np.pi/6
        parameters = [265, 2000-230, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [650,2000-330, angle, speed]
        self.go_to_coord_rotation(parameters)
        # suck front crater
        angle = 2*np.pi - np.pi/2
        parameters = [900,1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [900,1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        time.sleep(1)
        #parameters = [600,1600, angle, speed]
        #self.go_to_coord_rotation(parameters)
        angle = 0.0
        parameters = [600,1300, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [600,1300, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.stop()
        return

    def cube(self,speed = 4):
        angle = 0.0
        #parameters = [920, 200, angle, speed]
        #self.go_to_coord_rotation(parameters)
        parameters = [920,600 , angle, speed]
        self.go_to_coord_rotation(parameters)
        return
        while True:
            parameters = [1200, 500, angle, speed]
            self.go_to_coord_rotation(parameters)
            angle = np.pi
            parameters = [1200, 1000, angle, speed]
            self.go_to_coord_rotation(parameters)
            continue
            parameters = [1700, 1000, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [1700, 500, angle, speed]
            self.go_to_coord_rotation(parameters)

    def throw(self):
        self.cur_state = 2
        self.close_door()
        self.on_coolers_throw()
        time.sleep(0.2)
        self.off_coolers()
        time.sleep(0.2)
        self.on_coolers_throw()

    def suck(self):
        self.cur_state = 1
        self.off_coolers()
        time.sleep(0.1)
        self.on_coolers_suck()
        self.open_door()

    def stop(self):
        self.cur_state = 0
        self.close_door()
        time.sleep(0.4)
        self.off_coolers()

    def funny_action(self, signum, frame):
        logging.critical('FUNNNY ACTION start')
        logging.critical('CHECK if not throwing')
        if self.cur_state == 1:
            self.stop()
            time.sleep(0.2)
        tms = time.time()
        logging.critical('Adjust position')
        try:
            self.collision_avoidance = False ## Not to be stucked in while
            self.localisation.value = False ## Not to correct
            x, y, ang = self.coords[:3]
            throw_x, throw_y = 260, 110
            ang_dir = np.arctan2(y-throw_y, x - throw_x) # wil be from 0 to pi/2
            new_ang =  (2*np.pi - (np.pi/2 + np.pi/2 - ang_dir) + np.pi/2) % (2*np.pi) #(... + np.pi/2 because of have hole at right)
            new_x = x; new_y = y
            speed = 4 # isn't too slow??
            ## TODO if collision or bad place to rotate -> go away first: x->new_x, y->new_y
            pm = [new_x, new_y, new_ang, speed]
            self.go_to_coord_rotation(pm)
            logging.critical('Start Throw')
            if self.cur_state == 0:
                self.throw()
                logging.info("Throw")
            tms_n = time.time()
            while tms_n - tms < funny_delay:
                time.sleep(0.1)
                logging.info("Here in While!")
                tms_n = time.time()
            logging.info("Stopping")
            self.stop()
            self.off_mixer()
            logging.critical('Stop Propeller. Stop mixer.')
            logging.info(self.send_command('stopAllMotors'))
            logging.critical('Stop All Motors')
            logging.info(self.send_command('funny_action_open'))
            funny_done = 1
            logging.critical('FUNNNY ACTION end')
        except Exception:
            logging.critical("We failed to throw. Error in High level code or in Low level!")
        finally:
            exit()
   
def third_strategy():
    rb.steal_strategy2()
    rb.first_back()
    return

def second_strategy():
    rb.steal_strategy()
    rb.first_back()
    return
    
def first_strategy():
    rb.first()
    rb.first_back()
    return
    
rb = None
tmstmp = None
def competition(color = "yellow",strategy = 1):
    global rb
    global tmstmp
    init_test = None
    init_coords = {'main': [900, 200, np.pi/2], 'test1': [400, 850, 0.0]}
    init_coord = init_coords['main']
    if init_test is not None:
        init_coord = init_coords['test'+str(init_test)]
    rb = Robot(lidar_on=True, small=True, init_coord = init_coord, color=color)
    rb.p3.start()
    if init_test == 1:
        rb.localisation.value = False
        logging.info("Start speed test: " + "speed 1")
        tms = time.time()
        speed = 1
        parameters = [1150, 850, 0.0, speed]
        rb.go_to_coord_rotation(parameters)
        parameters = [1900, 850, 0.0, speed]
        rb.go_to_coord_rotation(parameters)
        parameters = [1150, 850, 0.0, speed]
        rb.go_to_coord_rotation(parameters)
        parameters = [400, 850, 0.0, speed]
        rb.go_to_coord_rotation(parameters)
        logging.info("4 points forward-f-back-b time: " + str(time.time() - tms))
        #time.sleep(5)
        #parameters = [1900, 850, 0.0, speed]
        #rb.go_to_coord_rotation(parameters)
        #parameters = [400, 850, 0.0, speed]
        #rb.go_to_coord_rotation(parameters)
        #logging.info("2 points forward-back time: " + str(time.time() - tms))
        return
    if False:
        rb.first_back()
        return
    if False:
        third_strategy()
    if False:
        rb.test44()
    if False: # funny action in movement and sucking
        signal.signal(signal.SIGALRM, rb.funny_action)
        signal.alarm(10)
        rb.open_door()
        rb.on_mixer()
        rb.on_coolers_suck()
        rb.go_to_coord_rotation([900, 1300, np.pi/2, 4])
        return
    if True:
        strategies = {1:first_strategy,
                      2:second_strategy,
                      3:third_strategy}
        if strategy not in strategies.keys():
            raise NameError('\n\t\t\Strategy NUMBER invalid!\n \t\t\t1 or 2  or 3\n')
        if True:
            signal.signal(signal.SIGALRM, rb.funny_action)
            signal.alarm(89-funny_delay)
        tmstmp = time.time()
        strategies[strategy]();
        logging.critical("Time passed for strategy:  " + str(time.time() - tmstmp));
    return
    
clr = "yellow"
if len(sys.argv) > 1:
    if sys.argv[1].lower() != "yellow" and sys.argv[1].lower() != "blue":
        raise NameError('\n\t\t\tCOLOR side invalid!\n\t\t\tYELLOW or BLUE (or lowercase)\n')
    clr = sys.argv[1].lower()

str_num = 1
if len(sys.argv) > 2:
    str_num = int(sys.argv[2])

try:
    competition(clr,str_num)
except KeyboardInterrupt:
    logging.exception('KeyboardInterrupt!')
except NameError:
    logging.exception('Custom NAME ERROR')
except Exception:
    logging.exception('High level Error!')
finally:
    #while not funny_done:
    #    continue
    if tmstmp is not None:
        logging.info("Time for strategy passes:  " + str(time.time() - tmstmp))
    logging.info("Finally DONE")
    rb.p.terminate()
    rb.p2.terminate()
    rb.p3.terminate()
