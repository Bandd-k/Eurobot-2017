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

console.setLevel(lvl)
# set a format which is simpler for console use
formatter = logging.Formatter('%(levelname)s: %(message)s')
console.setFormatter(formatter)
# add the handler to the root logger
logging.getLogger('').addHandler(console)
logger = logging.getLogger(__name__)


obstacles=[]
class Robot:
    def __init__(self, lidar_on=True, color = 'yellow', sen_noise = 25, 
        angle_noise=0.2, dist_noise = 45):
        self.color = color
        self.sensor_range = 30
        self.collision_avoidance = False
        self.localisation = Value('b', True)
        #change for collision
        #self.sensors_map = {0:(0, np.pi/3),7:(7*np.pi/4, 2*np.pi),3: (np.pi*0.7, np.pi*1.3),1: (5/3.*np.pi,2*np.pi),2:(0,np.pi*1/4.),6:(7/4.*np.pi,2*np.pi),8:(0,np.pi/4),4:(np.pi/4,3*np.pi/4),5:(np.pi*5/4,7*np.pi/4)} #[(7/4.*np.pi,2*np.pi),(0,np.pi*1/4.)]
        #self.sensors_map= {0: (0, np.pi/3), 1: (np.pi/4, np.pi*7/12), 2: (np.pi*0.5, np.pi*1.5), 3: (17/12.*np.pi, 7/4.*np.pi), 4: (5/3.*np.pi,2*np.pi), 5: [(7/4.*np.pi,2*np.pi),(0,np.pi*1/4.)]}  # can be problem with 2pi and 0
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
        self.coords = Array('d', rev_field([170, 170, 0], self.color))# 170, 170
        self.input_queue = Queue()
        self.loc_queue = Queue()
        self.fsm_queue = Queue()
        self.PF = pf.ParticleFilter(particles=1500, sense_noise=sen_noise, 
            distance_noise=dist_noise, angle_noise=angle_noise, 
            in_x=self.coords[0], in_y=self.coords[1], 
            in_angle=self.coords[2],input_queue=self.input_queue, 
            out_queue=self.loc_queue,color = self.color)
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
        self.p2.start()
        print "Process npParticle Start"
        time.sleep(0.1)
        print "Robot __init__ done!"

    def send_command(self,name,params=None):
        self.input_queue.put({'source': 'fsm','cmd': name,'params': params})
        return self.fsm_queue.get()

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
        pm = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2]),parameters[0] / 1000., parameters[1] / 1000., float(parameters[2]), parameters[3]]
        x = parameters[0] - self.coords[0]
        y = parameters[1] - self.coords[1]
        sm = x+y
        logging.info(self.send_command('go_to_with_corrections',pm))
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
        data = self.send_command('sensors_data')['data']
        data.append(data[2])
        data.append(data[0])
        data.append(data[1])
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


    def go_last(self,parameters):
        while abs(parameters[0]-self.coords[0]) > 10 or abs(parameters[1]-self.coords[1]) > 10:
            print 'calibrate'
            self.go_to_coord_rotation(parameters)
        
    
    ##########################################################
    ################# BIG Robot ############################
    ##########################################################

    def left_ball_down(self):
        self.send_command('left_ball_down')
        time.sleep(1)

    def left_ball_up(self):
        self.send_command('left_ball_up')
        time.sleep(1)

    def left_ball_drop(self):
        self.send_command('left_ball_drop')
        time.sleep(1)

    def right_ball_down(self):
        self.send_command('right_ball_down')
        time.sleep(1)

    def right_ball_up(self):
        self.send_command('right_ball_up')
        time.sleep(1)

    def right_ball_drop(self):
        self.send_command('right_ball_drop')
        time.sleep(1)

    # servos for cylinder take
    def front_down_cylinder_no(self):
        self.send_command('front_down_cylinder_no')
        time.sleep(1)

    def front_up_cylinder_yes(self):
        self.send_command('front_up_cylinder_yes')
        time.sleep(1)

    def front_drop_cylinder_yes(self):
        self.send_command('front_drop_cylinder_yes')
        time.sleep(1)

    def back_down_cylinder_no(self):
        self.send_command('back_down_cylinder_no')
        time.sleep(1)

    def back_up_cylinder_yes(self):
        self.send_command('back_up_cylinder_yes')
        time.sleep(1)

    def back_drop_cylinder_yes(self):
        self.send_command('back_drop_cylinder_yes')
        time.sleep(1)
    # sticks to use
    def both_sticks_open(self, dur = 1):
        self.send_command('both_sticks_open')
        time.sleep(dur)

    def both_sticks_close(self, dur = 1):
        self.send_command('both_sticks_close')
        time.sleep(dur)
    
    #seesaw manipulator
    def seesaw_hand_down(self, dur = 1):
        self.send_command('seesaw_hand_down')
        time.sleep(dur)
        
    def seesaw_hand_up(self, dur = 1):
        self.send_command('seesaw_hand_up')
        time.sleep(dur)    
    
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

    def big_robot_trajectory(self,speed=4):
        parameters = [170, 170, 0, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi*0.1
        self.localisation.value = False
        parameters = [900, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.localisation.value = True
        angle = np.pi/2*0.8
        parameters = [835, 180, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        parameters = [930, 390, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.front_down_cylinder_no()
        self.front_up_cylinder_yes()
        angle = 3*np.pi/4
        speed = 1
        parameters = [800, 850, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [650, 1250, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/2
        parameters = [553, 1516, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        parameters = [553-130, 1516, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/2 + 2*np.pi/18
        self.go_to_coord_rotation(parameters)
        parameters = [374, 1794, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/2
        speed=4
        parameters = [374, 1794, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [254, 1794, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [374, 1794, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)    
        speed = 4
        self.right_ball_down()
        time.sleep(1.5)
        self.right_ball_up()
        angle = np.pi/2
        parameters = [254, 1794  - 10, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 1
        parameters = [384, 1794 - 100, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)    
        speed = 4
        #self.left_ball_down()
        #time.sleep(1.5)
        #self.left_ball_up()
    ##    angle = np.pi/2
    ##    parameters = [280, 2000 - 160, angle, speed]
    ##        self.go_to_coord_rotation(parameters)
    ##    parameters = [310, 2000 - 160, angle, speed]
    ##        self.go_to_coord_rotation(parameters)
    ##    self.left_ball_down()
    ##    self.left_ball_up()
    ##    parameters = [310, 2000 - 160, angle, speed]
    ##        self.go_to_coord_rotation(parameters)

    def big_robot_trajectory_r(self,speed=4):
        angle = np.pi/2 + 3*np.pi/18
        parameters = [600, 1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/2 + np.pi/4
        speed = 1
        parameters = [650, 1250, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/2
        parameters = [1000, 850, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi/2*0.8
        parameters = [940, 410, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi*0.1
        speed = 4
        parameters = [920, 260, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 0.0 + np.pi/18
        parameters = [870, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.seesaw_hand_down()
        self.localisation.value = False
        parameters = [245, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.seesaw_hand_up()
        self.localisation.value = True
        angle = 0.0
        parameters = [235, 340, angle, speed]
        self.go_to_coord_rotation(parameters)
        ## check boarder; set exact coordinates
        while 1:
            coords = self.send_command('getCurrentCoordinates')['data']
            if type(coords) is not type(list()):
                logging.critical("Incorrect coordinates format")
                continue
            if type(coords[0]) is type(1000.):
                break
        coords[2] = 0.0 # angle 
        coords[1] = 280 # y - precise
        coords[0] = coords[0] # x
        self.send_command('setCoordinates', 
                          [coords[0] / 1000.,
                           coords[1] / 1000.,
                           coords[2]])
        self.coords = coords
        ##
        self.front_drop_cylinder_yes()
        self.right_ball_drop()
        self.right_ball_up()
        parameters = [180, 180, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi
        parameters = [180, 180, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [235, 280, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        #self.left_ball_drop()
        #self.left_ball_up()
        #self.funny()
        
    def loc_test2(self, speed=4, angle=np.pi, n_times=3):
        x1, x2, y1, y2 = 1000, 2500, 600, 1100
        dx, dy = x2-x1, y2-y1
        angle = np.arctan2(dy, dx) + np.pi
        start = [x1, y1, angle]
        for i in xrange(n_times):
            parameters = [start[0] + 0*dx/3, start[1] + 0*dy/3, start[2], speed]
            self.go_to_coord_rotation(parameters)
            parameters = [start[0] + 1*dx/3, start[1] + 1*dy/3, start[2], speed]
            self.go_to_coord_rotation(parameters)
            parameters = [start[0] + 2*dx/3, start[1] + 2*dy/3, start[2], speed]
            self.go_to_coord_rotation(parameters)
            parameters = [start[0] + 3*dx/3, start[1] + 3*dy/3, start[2], speed]
            self.go_to_coord_rotation(parameters)
        parameters = [start[0], start[1], start[2], speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)


    def loc_test(self, speed=4, angle=np.pi, n_times=3):
        x1, x2, y1, y2 = 1000, 2500, 600, 1100
        dx, dy = x2-x1, y2-y1
        angle = np.pi/2
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
        angle = 3*np.pi/2
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
def test():
    global rb
    rb = Robot(lidar_on=True, color = 'yellow', sen_noise=25, 
        dist_noise=30, angle_noise=0.3)
    ### LOCALISATION TEST
    #points = [[500, 1100], [1000, 1100]]#[750-40, 1400]]
    #odometry_coords = rb.localisation_test(point_lst = points,
                          #angle_lst = [0, np.pi], speed_lst = [4, 1], n_times = 1)
    #return
    ### END
    #rb.send_command('funny_action_open')
    #time.sleep(2)
    #rb.send_command('funny_action_close')
    #signal.signal(signal.SIGALRM, rb.funny_action)
    #signal.alarm(90)
    #while 1:
    #    rb.loc_test(n_times=2, speed=4)
    #    rb.loc_test(n_times=2, speed=1)
    #    return
        #rb.go_to_coord_rotation([500, 1100, np.pi, speed])
        #rb.go_to_coord_rotation([1000, 1100, np.pi, speed])
        #rb.go_to_coord_rotation([1500, 1100, np.pi, speed])
        #rb.go_to_coord_rotation([2000, 1100, np.pi, speed])
        #rb.go_to_coord_rotation([2500, 1100, np.pi, speed])
    #rb.loc_test(speed=4, angle=np.pi, n_times=2)
    #rb.loc_test(speed=4, angle=0, n_times=2)
    #rb.loc_test(speed=1, angle=np.pi, n_times=2)
    #rb.loc_test(speed=1, angle=0, n_times=2)
    i = 0
    while i<10:
    ########## Big robot test START
        rb.big_robot_trajectory(4)
        rb.big_robot_trajectory_r(4)
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
    rb.p.terminate()
    rb.p2.terminate()



