import driver
import time
from hokuyolx import HokuyoLX
import logging
import signal
import npParticle as pf
import numpy as np
from multiprocessing import Process, Queue, Value,Array
from multiprocessing.queues import Queue as QueueType
lvl = logging.INFO
logging.basicConfig(filename='Eurobot.log', filemode='w', format='%(levelname)s:%(asctime)s %(message)s',
                    datefmt='%m/%d/%Y %I:%M:%S %p', level=lvl)

console = logging.StreamHandler()
console.setLevel(lvl)
# set a format which is simpler for console use
formatter = logging.Formatter('%(levelname)s: %(message)s')
console.setFormatter(formatter)
# add the handler to the root logger
logging.getLogger('').addHandler(console)
logger = logging.getLogger(__name__)

obstacles=[]
class Robot:
    def __init__(self, lidar_on=True,small=True):
        sensors_number=6
        self.sensor_range = 20
        self.collision_avoidance = False
        if small:
            self.sensors_map= {0: (0, np.pi/3), 1: (np.pi/4, np.pi*7/12), 2: (np.pi*0.5, np.pi*1.5), 3: (17/12.*np.pi, 7/4.*np.pi), 4: (5/3.*np.pi,2*np.pi), 5: [(7/4.*np.pi,2*np.pi),(0,np.pi*1/4.)]}  # can be problem with 2pi and 0
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
                logging.warning('lidar is not connected')
        #self.x = 170  # mm
        #self.y = 150  # mm
        #self.angle = 0.0  # pi
        if small:
            self.coords = Array('d',[850, 170, 0])
        else:
            self.coords = Array('d', [170, 170, 0])
        self.localisation = Value('b', True)
        self.input_queue = Queue()
        self.loc_queue = Queue()
        self.fsm_queue = Queue()
        self.PF = pf.ParticleFilter(particles=1500, sense_noise=25, distance_noise=20, angle_noise=0.3, in_x=self.coords[0],
                                    in_y=self.coords[1], in_angle=self.coords[2],input_queue=self.input_queue, out_queue=self.loc_queue)

        # driver process
        self.dr = driver.Driver(self.input_queue,self.fsm_queue,self.loc_queue)
        p = Process(target=self.dr.run)
        p.start()
        p2 = Process(target=self.PF.localisation,args=(self.localisation,self.coords,self.get_raw_lidar))
        logging.info(self.send_command('echo','ECHO'))
        logging.info(self.send_command('setCoordinates',[self.coords[0] / 1000., self.coords[1] / 1000., self.coords[2]]))
        p2.start()
        time.sleep(0.1)

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
        if self.PF.warning:
            time.sleep(1)
        pm = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2]),parameters[0] / 1000., parameters[1] / 1000., float(parameters[2]), parameters[3]]
        x = parameters[0] - self.coords[0]
        y = parameters[1] - self.coords[1]
        sm = x+y
        logging.info(self.send_command('go_to_with_corrections',pm))
        # After movement
        stamp = time.time()
        time.sleep(0.100001)  # sleep because of STM interruptions (Maybe add force interrupt in STM)
        while not self.send_command('is_point_was_reached')['data']:
            time.sleep(0.05)
            if self.collision_avoidance:
                direction = (float(x) / sm, float(y) / sm)
                if self.check_collisions(direction):
                    self.send_command('stopAllMotors')
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
        angle = np.arctan2(direction[1],direction[0]) % (np.pi*2)
        sensor_angle = (angle-self.coords[2]) %(np.pi*2)
        #### switch on sensor_angle
        collisions = [0,0,0,0,1]
        for index,i in enumerate(collisions):
            if i and sensor_angle<=self.sensors_map[index][1] and sensor_angle>=self.sensors_map[index][0]:
                logging.info("Collision at index "+str(index))
                if self.check_map(direction):
                    continue
                return True
        return False

    def receive_sensors_data(self):
        data = self.send_command('sensors_data')
        answer = []
        for i in range(6):
            answer.append((data & (1 << i)) != 0)
        return answer


    def check_map(self,direction): # probably can be optimized However O(1)
        for i in range(0,self.sensor_range,2):
            for dx in range(-2,2):
                for dy in range(-2,2):
                    x = int(self.coords[0]/10+direction[0]*i+dx)
                    y = int(self.coords[1]/10+direction[1]*i+dy)
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

    def take_cylinder(self): # approx time = 2
        self.send_command('take_cylinder')
        time.sleep(4)
    def store_cylinder(self): # approx time = 0.5
        self.send_command('store_cylinder')
        time.sleep(0.5)
    def drop_cylinder(self): # approx time = 1
        self.send_command('drop_cylinder')
        time.sleep(1)

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
    def funny(self):
        self.send_command('funny')
        time.sleep(1)




    ############################################################################
    ######## HIGH LEVEL FUNCTIONS ##############################################
    ############################################################################
    def demo(self, speed=1):
        """robot Demo, go to coord and take cylinder"""
        signal.signal(signal.SIGALRM, self.funny_action)
        signal.alarm(90)
        # TODO take cylinder
        angle = np.pi
        parameters = [850, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 700, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [650, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [300, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)

    def demo_r(self, speed=1):
        """robot Demo, go to coord and take cylinder"""
        # TODO take cylinder
        angle = np.pi
        parameters = [650, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 700, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [850, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 0.0
        parameters = [170, 150, angle, speed]
        self.go_to_coord_rotation(parameters)

    def test_trajectory(self,speed=1):
        angle =3*np.pi / 2.
        parameters = [700, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1150, 190, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1150, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1350, 1570, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1350, 1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1350, 1400, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1350, 1300, angle, speed]
        self.go_to_coord_rotation(parameters)
        ######
        parameters = [900, 1200, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [900, 1200, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        ########
        parameters = [1150, 1200, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1300, 1550, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1270, 1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1250, 1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1200, 1450, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1200, 1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1250, 1550, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1300, 1600, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [900, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [170, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        return

    def simpliest_trajectory(self,speed=1):
        angle =3*np.pi / 2.
        #1 area 2: after seesaw
        parameters = [700, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        #2 back rocket stand to take
        parameters = [1150, 190, angle, speed]
        self.go_to_coord_rotation(parameters)
        #3 back rocket pass big robot
        parameters = [1250, 190, angle, speed]
        self.go_to_coord_rotation(parameters)
        time.sleep(1)
        #4 back rocket stand to take
        parameters = [1150, 190, angle, speed]
        self.go_to_coord_rotation(parameters)
        #########
        #5 between adjust
        parameters = [1250, 1200, angle, speed]
        self.go_to_coord_rotation(parameters)
        #6 1 lunar module
        parameters = [1350, 1600, angle, speed]
        self.go_to_coord_rotation(parameters)
        #7 2nd lunar module
        parameters = [1300, 1550, angle, speed]
        self.go_to_coord_rotation(parameters)
        #8  3rd lunar module
        parameters = [1280, 1530, angle, speed]
        self.go_to_coord_rotation(parameters)
        #9 last module
        parameters = [1250, 1500, angle, speed]
        self.go_to_coord_rotation(parameters)
        #10 between adjust
        parameters = [1080, 1300, angle, speed]
        self.go_to_coord_rotation(parameters)
        #11 lateral rocket stand to take
        parameters = [250, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_last(parameters)
        ####
        #12 between adjustment: back 20
        parameters = [270, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        #13 1st module
        parameters = [270, 1150, angle, speed]
        self.go_to_coord_rotation(parameters)
        #14 2nd module
        parameters = [270, 1050, angle, speed]
        self.go_to_coord_rotation(parameters)
        #15 3rd module take + rbg detect
        parameters = [270, 950, angle, speed]
        self.go_to_coord_rotation(parameters)
        #16 2th module take + rbg detect
        parameters = [270, 1050, angle, speed]
        self.go_to_coord_rotation(parameters)
        #17 1t module take + rbg detect
        parameters = [270, 1150, angle, speed]
        self.go_to_coord_rotation(parameters)

        ######
        ## funny action
        parameters = [270, 1150, 0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [270, 1150, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [270, 1150, 0, speed]
        self.go_to_coord_rotation(parameters)



    def big_robot_trajectory(self,speed=1):
        angle = np.pi*0.1
        self.left_ball_up()
        self.localisation.value = False
        parameters = [900, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.localisation.value = True
        angle = np.pi/2
        parameters = [950, 400, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [950, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 0.0
        parameters = [250, 1750, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.left_ball_down()
        self.left_ball_up()

    def big_robot_trajectory_r(self,speed=1):
        angle = np.pi/2
        parameters = [900, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [950, 400, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [950, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi * 0.1
        self.localisation.value = False
        parameters = [170, 180, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.localisation.value = True
        self.left_ball_drop()
        self.funny()

    def first_cylinder(self,speed=1):
        angle = np.pi
        ############### take cylinder
        parameters = [700, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1135, 400, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi*3/2.
        parameters = [1135, 400, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1135, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1135, 220, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder()
        #self.store_cylinder()
        ##############
        parameters = [1135, 400, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.drop_cylinder()
        return
        self.go_to_coord_rotation(parameters)
        parameters = [1150, 200, angle, speed]
        angle = np.pi
        self.go_to_coord_rotation(parameters)
        parameters = [1150, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [400, 800, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [200, 800, angle, speed]
        self.go_last(parameters)
        self.go_to_coord_rotation(parameters)
        parameters = [120, 800, angle, speed]
        self.drop_cylinder()


    def funny_action(self, signum, frame):
        print 'Main functionaly is off'
        print 'FUNNNY ACTION'


def test():
    rb = Robot(True)
    #rb.take_cylinder()
    #rb.first_cylinder()
    i = 0
    while i<10:
        #rb.big_robot_trajectory(4)
        #rb.big_robot_trajectory_r(4)
        rb.demo(4)
        rb.demo_r(4)
        i+=1

def tst_time():
    rb = Robot(True)
    stamp = time.time()
    for i in range(100):
        command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
        rb.dr.process_cmd(command)
    print time.time()-stamp

test()

