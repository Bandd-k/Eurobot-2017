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
            self.sensors_map = {0:(0, np.pi/3),1: (np.pi*0.5, np.pi*1.5),2: (5/3.*np.pi,2*np.pi),3: [(7/4.*np.pi,2*np.pi),(0,np.pi*1/4.)]}
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
                logging.warning('lidar is not connected')
        #self.x = 170  # mm
        #self.y = 150  # mm
        #self.angle = 0.0  # pi
        if small:
            self.coords = Array('d',[850, 170, 3*np.pi / 2])
        else:
            self.coords = Array('d', [170, 170, 0])
        self.localisation = Value('b', True)
        self.input_queue = Queue()
        self.loc_queue = Queue()
        self.fsm_queue = Queue()
        self.PF = pf.ParticleFilter(particles=1500, sense_noise=25, distance_noise=45, angle_noise=0.2, in_x=self.coords[0],
                                    in_y=self.coords[1], in_angle=self.coords[2],input_queue=self.input_queue, out_queue=self.loc_queue)

        # driver process
        self.dr = driver.Driver(self.input_queue,self.fsm_queue,self.loc_queue)
        self.p = Process(target=self.dr.run)
        self.p.start()
        self.p2 = Process(target=self.PF.localisation,args=(self.localisation,self.coords,self.get_raw_lidar))
        logging.info(self.send_command('echo','ECHO'))
        logging.info(self.send_command('setCoordinates',[self.coords[0] / 1000., self.coords[1] / 1000., self.coords[2]]))
        self.p2.start()
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
        sensor_angle = (angle-self.coords[2]) % (np.pi*2)
        #### switch on sensor_angle
        collisions = self.sensor_data()
        for index,i in enumerate(collisions):
            if i and sensor_angle<=self.sensors_map[index][1] and sensor_angle>=self.sensors_map[index][0]:
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
        return data


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
        self.send_command('funny_action')
        time.sleep(1)

    def on_sucker(self):
        self.send_command('on_sucker')

    def off_sucker(self):
        self.send_command('off_sucker')

    def rotate_cylinder_horizonal(self):
        logging.info(self.send_command('rotate_cylinder_horizonal'))
        time.sleep(0.3)

    def rotate_cylinder_vertical(self):
        logging.info(self.send_command('rotate_cylinder_vertical'))
        time.sleep(0.3)

    def take_cylinder_outside(self):
        logging.info(self.send_command('take_cylinder_outside'))
        time.sleep(0.5)

    def take_cylinder_inside(self):
        logging.info(self.send_command('take_cylinder_inside'))
        time.sleep(0.5)

    def lift_up(self):
        logging.info(self.send_command('lift_up'))
        time.sleep(0.5)

    def store(self):
        logging.info(self.send_command('store'))
        # time.sleep(0.5)

    def out_cylinders(self):
        logging.info(self.send_command('out_cylinders'))
        time.sleep(0.5)

    def pick_up(self):
        self.rotate_cylinder_vertical()
        self.take_cylinder_inside()
        self.lift_up()
        self.off_sucker()
        self.store()
        self.rotate_cylinder_horizonal()


    def pick_up2(self):
        self.rotate_cylinder_vertical()
        self.take_cylinder_inside()
        self.lift_up()
        self.off_sucker()
        self.rotate_cylinder_horizonal()

    def cyl_test(self):
        self.pick_up()
        self.pick_up()
        self.pick_up()
        self.out_cylinders()




    ############################################################################
    ######## HIGH LEVEL FUNCTIONS ##############################################
    ############################################################################
    def demo(self, speed=1):
        """robot Demo, go to coord and take cylinder"""
        signal.signal(signal.SIGALRM, self.funny_action)
        signal.alarm(90)
        # TODO take cylinder
        self.rotate_cylinder_horizonal()
        angle = np.pi
        parameters = [850, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 700, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [650, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [400, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)
        # return
        self.on_sucker()
        self.take_cylinder_outside()
        parameters = [160, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)

        parameters = [320, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up()
        self.on_sucker()
        self.take_cylinder_outside()
        parameters = [160, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)

        parameters = [320, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up()
        self.on_sucker()
        self.take_cylinder_outside()
        parameters = [160, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)

    def demo_r2(self, speed=1):
        angle = np.pi
        parameters = [400, 1200, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up2()
        parameters = [150, 1140, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        parameters = [150, 950, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        parameters = [150, 800, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        parameters = [600, 800, angle, speed]
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


    def loc_test(self):
        while True:
            angle = np.pi
            parameters = [900, 200, angle, 6]
            self.go_to_coord_rotation(parameters)
            parameters = [900, 400, angle, 6]
            self.go_to_coord_rotation(parameters)
            parameters = [900, 600, angle, 6]
            self.go_to_coord_rotation(parameters)
            parameters = [900, 400, angle, 6]
            self.go_to_coord_rotation(parameters)


    def small_robot_trajectory(self,speed=1):
        angle = 3*np.pi / 2
        self.rotate_cylinder_horizonal()
        parameters = [1100, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1145, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.on_sucker()
        self.take_cylinder_outside()
        parameters = [1145, 160, angle, speed]
        self.go_to_coord_rotation(parameters)

        parameters = [1145, 320, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up()

        self.on_sucker()
        self.take_cylinder_outside()
        parameters = [1145, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1145, 320, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up()

        self.on_sucker()
        self.take_cylinder_outside()
        parameters = [1145, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1145, 320, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up2()

    def small_robot_trajectory_r(self, speed=1):
        angle = np.pi
        parameters = [1150, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi-np.pi/4
        parameters = [1320, 1520, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 6
        parameters = [1320, 1690, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        parameters = [1217, 1590, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        parameters = [1117, 1490, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        speed = 4
        parameters = [1150, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)


    def funny_action(self, signum, frame):
        print 'Main functionaly is off'
        print 'FUNNNY ACTION'
    def sens_test(self):
        

rb = None
def test():
    global rb
    rb = Robot(True)
    #rb.take_cylinder()
    #rb.first_cylinder()
    #rb.pick_up()
    #time.sleep(10)
    #rb.pick_up()
    #rb.out_cylinders()
    #rb.out_cylinders()
    #rb.out_cylinders()
    #return
    i = 0
    while i<10:
        #rb.big_robot_trajectory(4)
        #rb.big_robot_trajectory_r(4)
        rb.small_robot_trajectory(4)
        rb.small_robot_trajectory_r(4)
        return
        i+=1

try:
    test()
except:
    print "Ending"
    rb.p.terminate()
    rb.p2,terminate()
    

