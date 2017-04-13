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
logging.basicConfig(filename='Eurobot.log', filemode='w', format='%(levelname)s:%(asctime)s %(message)s',
                    datefmt='%m/%d/%Y %I:%M:%S %p', level=lvl)

console = logging.StreamHandler()

def rev_field(val, color):
    if color == "blue":
        return [3000-val[0], val[1], (np.pi - val[2] + 2*np.pi)%(2*np.pi)] + val[3:]
    return val
    

## TODO: Defined according to mechanical situation, make from scrath to 90, 90, 180, 180 (left_up, right_up, left_down, right_down)
angle_left_up, angle_right_up, angle_left_down, angle_right_down = 80., 55., 170., 143.

console.setLevel(lvl)
# set a format which is simpler for console use
formatter = logging.Formatter('%(levelname)s: %(message)s')
console.setFormatter(formatter)
# add the handler to the root logger
logging.getLogger('').addHandler(console)
logger = logging.getLogger(__name__)


obstacles=[]
class Robot:
    def __init__(self, lidar_on=True, color = 'yellow', sen_noise = 20, 
        angle_noise=0.2, dist_noise = 45, init_coord = [170, 170, 0]):
        self.color = color
        self.sensor_range = 50
        self.collision_avoidance = True
        self.localisation = Value('b', True)
        #change for collision
        #self.sensors_map = {0:(0, np.pi/3),7:(7*np.pi/4, 2*np.pi),3: (np.pi*0.7, np.pi*1.3),1: (5/3.*np.pi,2*np.pi),2:(0,np.pi*1/4.),6:(7/4.*np.pi,2*np.pi),8:(0,np.pi/4),4:(np.pi/4,3*np.pi/4),5:(np.pi*5/4,7*np.pi/4)} #[(7/4.*np.pi,2*np.pi),(0,np.pi*1/4.)]
        self.sensors_map= {0: (np.pi/6, np.pi/2+np.pi/6), 1: (np.pi/2-np.pi/6, np.pi*5/6), 2: (0, np.pi/4), 3: (np.pi - np.pi/4, np.pi + np.pi/4), 4: (3*np.pi/2-np.pi/6,11*np.pi/6), 5: (np.pi+np.pi/6,3*np.pi/2+np.pi/6), 6:(7*np.pi/4,2*np.pi)}  # can be problem with 2pi and 0
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
        self.PF = pf.ParticleFilter(particles=1500, sense_noise=sen_noise, 
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
            time.sleep(0.05)
            if self.collision_avoidance:
                direction = (float(x), float(y))
                while self.check_collisions(direction):
                    if pids:
                        self.send_command('stopAllMotors')
                        pids = False
                    time.sleep(0.5)
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
            print 'calibrate'
            self.go_to_coord_rotation(parameters)
            if tmstmp is not None and time.time() - tmstmp > dur:
                break
        
    
    ##########################################################
    ################# BIG Robot ############################
    ##########################################################

    def left_ball_down(self, dur = 1, angle = angle_left_down, speed = 100.0):
        self.collision_avoidance = False
        self.send_command('left_ball_down', [angle, speed])
        time.sleep(dur)
        self.collision_avoidance = False

    def left_ball_up(self, dur = 1, angle = angle_left_up, speed = 100.0):
        self.collision_avoidance = False
        self.send_command('left_ball_up', [angle, speed])
        time.sleep(dur)
        self.collision_avoidance = False

    def left_ball_drop(self, dur = 1, angle = angle_left_up + 33., speed = 100.0):
        self.collision_avoidance = False
        self.send_command('left_ball_drop', [angle, speed])
        time.sleep(dur)
        self.collision_avoidance = True 

    def right_ball_down(self, dur = 1, angle = angle_right_down, speed = 100.):
        self.collision_avoidance = False
        self.send_command('right_ball_down', [angle, speed])
        time.sleep(dur)
        self.collision_avoidance = False

    def right_ball_up(self, dur = 1, angle = angle_right_up, speed = 100.):
        self.collision_avoidance = False
        self.send_command('right_ball_up', [angle, speed])
        time.sleep(dur)
        self.collision_avoidance = True

    def right_ball_drop(self, dur = 1, angle = angle_right_up + 35., speed = 100.0):
        rb.collision_avoidance = False
        self.send_command('right_ball_drop', [angle, speed])
        time.sleep(dur)
        rb.collision_avoidance = True

    # servos for cylinder take
    def front_down_cylinder_no(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('front_down_cylinder_no')
        time.sleep(dur)
        rb.collision_avoidance = True

    def front_up_cylinder_yes(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('front_up_cylinder_yes')
        time.sleep(dur)
        rb.collision_avoidance = True

    def front_drop_cylinder_yes(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('front_drop_cylinder_yes')
        time.sleep(dur)
        rb.collision_avoidance = True

    def back_down_cylinder_no(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('back_down_cylinder_no')
        time.sleep(dur)
        rb.collision_avoidance = True

    def back_up_cylinder_yes(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('back_up_cylinder_yes')
        time.sleep(dur)
        rb.collision_avoidance = True

    def back_drop_cylinder_yes(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('back_drop_cylinder_yes')
        time.sleep(dur)
        rb.collision_avoidance = True

    # sticks to use
    def both_sticks_open(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('both_sticks_open')
        time.sleep(dur)
        rb.collision_avoidance = True

    def both_sticks_close(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('both_sticks_close')
        time.sleep(dur)
        rb.collision_avoidance = True
    
    #seesaw manipulator
    def seesaw_hand_down(self, dur = 1):
        rb.collision_avoidance = False
        self.send_command('seesaw_hand_down')
        time.sleep(dur)
        rb.collision_avoidance = True
        
    def seesaw_hand_up(self, dur = 1):
        self.collision_avoidance = False
        self.send_command('seesaw_hand_up')
        time.sleep(dur)    
        self.collision_avoidance = True
    
    # funny action
    def funny_action(self, signum, frame):
        self.send_command('stopAllMotors')
        self.send_command('funny_action_open')
        print 'FUNNNY ACTION'

    def lin_interpol_traj(x1, y1, x2, y2, ratio):
        return [x1+(x2-x1)*ratio, y1+(y2-y1)*ratio]
    

    ############################################################################
    ######## HIGH LEVEL FUNCTIONS ##############################################
    ############################################################################

    def init_manipulators(self):
        self.left_ball_up(dur = 0)
        self.right_ball_up(dur = 0)
        self.seesaw_hand_up(dur = 0)


    def another_trajectory(self,speed = 4):
        signal.signal(signal.SIGALRM, self.funny_action)
        signal.alarm(90)
        speed = 1
        angle = np.pi/6
        self.collision_avoidance = False
        
        self.localisation.value = False
        parameters = [850, 170, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.localisation.value = True
        angle = 6*np.pi/7
        parameters = [900, 600, angle, speed]
        self.go_last(parameters)
        speed = 4
        parameters = [250, 1850, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/2
        parameters = [350, 1800, angle, speed]
        self.go_last(parameters)
        self.left_ball_down()
        parameters = [400, 1850, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.left_ball_up()
        angle = 3*np.pi/2*0.95
        parameters = [330, 1800, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.go_last(parameters)
        parameters = [400, 1600, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [350, 1600, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_last(parameters, dur = 3)
        self.right_ball_down()
        parameters = [400, 1600, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.right_ball_up()
        
        speed = 1
        angle = 6*np.pi/7
        parameters = [900, 600, angle, speed]
        self.go_last(parameters)
        parameters = [950, 170, angle, speed]
        self.go_to_coord_rotation(parameters)
        
    def big_robot_trajectory(self,speed=4):
        
        signal.signal(signal.SIGALRM, self.funny_action)
        signal.alarm(90)
        speed = 4
        angle = np.pi/4
        self.collision_avoidance = False
        parameters = [170, 170, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 4
        self.localisation.value = False
        parameters = [950, 170, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.localisation.value = True
        angle = np.pi/2 + np.pi/3 + np.pi/18
        parameters = [1000, 600, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_last(parameters)
        #angle = np.pi/2
        #parameters = [835, 180, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #self.go_to_coord_rotation(parameters)
        #speed = 4
        #parameters = [930, 390, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #angle = np.pi/2*0.88
        #parameters = [935, 440, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #self.go_to_coord_rotation(parameters)
        #self.front_down_cylinder_no()
        #self.front_up_cylinder_yes()
        #angle = 3*np.pi/4
        speed = 1
        #parameters = [800, 850, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #parameters = [650, 1250, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #angle = np.pi/2
        #parameters = [950, 200, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #angle = np.pi
        angle = np.pi/2 + np.pi/3 + np.pi/18
        parameters = [1000, 600, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_last(parameters)
        self.localisation.value = False
        #self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True
        ##parameters = [600, 1250, angle, speed]
        ##self.go_to_coord_rotation(parameters)
        #parameters = [553, 1516, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #self.go_to_coord_rotation(parameters)
        #parameters = [553-130, 1516, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #angle = 3*np.pi/2 + 2*np.pi/18
        #self.go_to_coord_rotation(parameters)
        #parameters = [374, 1794, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #angle = 3*np.pi/2
        #speed=4
        #parameters = [374, 1794, angle, speed]
        #self.go_to_coord_rotation(parameters)
        speed = 1
        #angle = np.pi/2 + np.pi/3 
        parameters = [50, 1950, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.localisation.value = True
        self.go_last(parameters)
        #self.go_last(parameters)
        #time.sleep(5)
        #self.collision_avoidance = False
        angle = np.pi/2
        parameters = [300, 1794, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [374, 1794, angle, speed]
        self.go_to_coord_rotation(parameters)   
        #self.go_to_coord_rotation(parameters)     
        speed = 1
        self.left_ball_down()
        time.sleep(1.5) # to take balls
        parameters = [400, 1794, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.left_ball_up(dur = 0.5)
        angle = 3*np.pi/2
        parameters = [254, 1794  - 10, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 1
        parameters = [384, 1794 - 120, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 4
        angle = 3*np.pi/2
        parameters = [400, 1794 - 120, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.go_to_coord_rotation(parameters)
        parameters = [384, 1794 - 120, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.go_to_coord_rotation(parameters)
        self.right_ball_down()
        time.sleep(1) # to take balls
        #self.left_ball_up()

    def big_robot_trajectory_r(self,speed=4):
        self.localisation.value = True
        self.collision_avoidance = False
        speed = 4
        angle = 3*np.pi/2 + 3*np.pi/18
        parameters = [450, 1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.right_ball_up(dur = 0.5)
        self.collision_avoidance = True
        angle = 3*np.pi/2 + np.pi/4
        speed = 1
        parameters = [650, 1250, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/2
        parameters = [1000, 850, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/2*0.8
        parameters = [940, 410, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi*0.1
        speed = 1
        parameters = [920, 260, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 0.0 + np.pi/18
        parameters = [870, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [860, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.seesaw_hand_down()
        self.localisation.value = False
        parameters = [265, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.seesaw_hand_up()
        self.localisation.value = True
        angle = 0.0
        parameters = [235, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        parameters = [235, 320, angle, speed]
        self.go_to_coord_rotation(parameters)
        ## check boarder; set exact coordinates
        while 1:
            logging.info("Boarder localisation")
            coords = self.send_command('getCurrentCoordinates')['data']
            #print type(coords)
            if type(coords[0]) is not type(1000.):
                logging.critical("Incorrect coordinates format")
                continue
            else:
                break
        coords[2] = 0.0 # angle 
        coords[1] = 280.0# y - precise
        coords[0] = coords[0]*1000.0 # x
        self.send_command('setCoordinates', 
                          [coords[0] / 1000.,
                           coords[1] / 1000.,
                           coords[2]])
        self.coords = Array('d', coords)
        ##
        self.collision_avoidance = False
        self.front_drop_cylinder_yes()
        self.right_ball_drop()
        self.right_ball_up()
        speed = 4
        angle = 0.0
        parameters = [180, 180, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi
        parameters = [180, 180, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [235, 280, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.left_ball_drop()
        self.left_ball_up()
        
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
            #~ parameters = [start[0] + 1*dx/3, start[1] + 1*dy/3, start[2], speed]
            #~ self.go_to_coord_rotation(parameters)
            #~ parameters = [start[0] + 2*dx/3, start[1] + 2*dy/3, start[2], speed]
            #~ self.go_to_coord_rotation(parameters)
            #~ parameters = [start[0] + 3*dx/3, start[1] + 3*dy/3, start[2], speed]
            #~ self.go_to_coord_rotation(parameters)
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

    
    def collisionTest(self,speed=1):
        #signal.signal(signal.SIGALRM, self.funny_action)
        #signal.alarm(40)
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
init_coord = [170., 170., np.pi/6]
def test():
    global rb
    rb = Robot(lidar_on=True, color = 'yellow', sen_noise=25, 
        dist_noise=45., angle_noise=0.2, init_coord = init_coord)
    #rb.left_ball_down()
    #rb.collisionTest(4)
    #return
    if True: # den staff
        rb.another_trajectory()
        return
    if False:
        parameters = [1000., 600., np.pi, 4]
        #rb.go_to_coord_rotation(parameters)
        rb.go_last(parameters)
        return
    if False:
        rb.right_ball_up()#(angle = angle_right_up)
        rb.right_ball_down()#(angle = angle_right_down)
        rb.right_ball_up()#(angle = angle_right_up)
        rb.right_ball_drop(angle = angle_right_up + 35.)#(angle = angle_right_up + 35.)
        rb.right_ball_up()#(angle = angle_right_up)
        rb.left_ball_up()#(angle = angle_left_up)
        rb.left_ball_down()#(angle = angle_left_dwon)
        rb.left_ball_up()#(angle = angle_left_up)
        rb.left_ball_drop(angle = angle_left_up + 33.)#()
        rb.left_ball_up()#(angle = angle_left_down)
        return
    ### LOCALISATION TEST
    #points = [[500, 1000], [1500, 1000]]#[750-40, 1400]]
    #odometry_coords = rb.localisation_test(point_lst = points,
    #                      angle_lst = [np.pi, 0], speed_lst = [1], n_times = 3)
    #return
    #### END
    if False:
        rb.send_command('funny_action_open')
        time.sleep(2)
        rb.send_command('funny_action_close')
    #speed = 4
    if False:
        while not rb.is_start():
            continue
    #rb.localisation.value = False
    #parameters = [-750, 1250, -np.pi/6, 1]
    #rb.go_to_coord_rotation(parameters)
    #return
    #while 1:
    #    x1x2y1y2 = [500, 1000, 600, 1100]
    #    rb.loc_test2(x1x2y1y2 = x1x2y1y2, n_times=2, speed=4, angle = 0.0, deltas = 1)
    #    rb.loc_test2(x1x2y1y2 = x1x2y1y2, n_times=2, speed=1, angle = 0.0, deltas = 1)
    #return
    i = 0
    while i<10:
    ########## Big robot test START
        tmstmp = time.time()
        rb.big_robot_trajectory(4)
        rb.big_robot_trajectory_r(4)
        logging.info("Time for strategy passes:  ", time.time() - tmstmp)
        rb.PF.debug_info += [time.time() - tmstmp, [], []]
        np.savetxt("localisation_debug.txt", np.array(rb.PF.debug_info))
        return
        while 1:
            angle = np.pi
            speed = 4
            parameterss = [160, 180, angle, speed]
            rb.go_to_coord_rotation(parameters)
            parameters = [180, 180, angle, speed]
            rb.go_to_coord_rotation(parameters)
    rb = Robot(True)
    rb.collisionTest(4)
    return

try:
    test()
except KeyboardInterrupt:
    logging.info("Time for strategy passes:  ", time.time() - tmstmp)
    rb.PF.debug_info += [time.time() - tmstmp, [], []]
    np.savetxt("localisation_debug.txt", np.array(rb.PF.debug_info))
    rb.p.terminate()
    rb.p2.terminate()



