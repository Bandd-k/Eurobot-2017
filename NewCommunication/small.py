import driver
import time
from hokuyolx import HokuyoLX
import logging
import signal
import npParticle as pf
import numpy as np
from multiprocessing import Process, Queue, Value,Array
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


class Robot:
    def __init__(self, lidar_on=True, small=True, color = 'yellow'):
        # Cylinder Staff
        self.cylinders = 0
        self.cyl_prepare = [3000,2000,2000]
        self.cyl_up = [7000,7000,9000]
        self.cyl_down = [-11000,-9000,-10000]
        ##################
        self.color = color
        self.sensor_range = 35
        self.collision_d = 9
        self.coll_ind = -1
        self.collision_avoidance = False
        self.localisation = Value('b', True)
        if small:
            self.sensors_places = [0.0,0.0,np.pi,np.pi/2,3*np.pi/2,0,0,0]
            self.sensors_map = {0:(0, np.pi/3),7:(7*np.pi/4, 2*np.pi),3: (np.pi*0.7, np.pi*1.3),1: (5/3.*np.pi,2*np.pi),2:(0,np.pi*1/4.),6:(7/4.*np.pi,2*np.pi),8:(0,np.pi/4),4:(np.pi/4,3*np.pi/4),5:(np.pi*5/4,7*np.pi/4)}
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
            self.coords = Array('d',rev_field([170, 170, 3*np.pi/2],self.color))
        else:
            driver.PORT_SNR = '325936843235' # need change
            self.coords = Array('d', rev_field([170, 170, 0], self.color))
        self.input_queue = Queue()
        self.loc_queue = Queue()
        self.fsm_queue = Queue()
        self.PF = pf.ParticleFilter(particles=1500, sense_noise=25, distance_noise=25, angle_noise=0.15, in_x=self.coords[0], in_y=self.coords[1], in_angle=self.coords[2],input_queue=self.input_queue, out_queue=self.loc_queue,color = self.color)

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
        #time.sleep(0.1)
        self.rotate_cylinder_vertical()
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

    def go_to(self, parameters):
        # beta version of clever go_to
        distance = 100
        ok = self.go_to_coord_rotation(parameters)
        if not ok:
            angle = (self.coords[2] + self.sensors_places[self.coll_ind] - np.pi) %(np.pi*2)
            direction = (np.cos(angle),np.sin(angle))
            pm = [self.coords[0]+direction[0]*distance,self.coords[1]+direction[1]*distance,self.coords[2],parameters[3]]
            self.go_to_coord_rotation(pm)
            self.go_to_coord_rotation(parameters)
        

    def go_to_coord_rotation(self, parameters):  # parameters [x,y,angle,speed]
        parameters = rev_field(parameters,self.color)
        if self.PF.warning:
            time.sleep(1)
        pm = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2]),parameters[0] / 1000., parameters[1] / 1000., float(parameters[2]), parameters[3]]
        x = parameters[0] - self.coords[0]
        y = parameters[1] - self.coords[1]
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
                    if (time.time() - stamp) > 7:
                        # return self.coll_ind
                        # ha
                        #self.send_command('setCoordinates',)
                        
                if not pids:
                    pids = True
                    logging.info(self.send_command('switchOnPid'))
                # return False
                # check untill ok and then move!
            # add Collision Avoidance there
            if (time.time() - stamp) > 7:
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

    def sensor_data(self):
        data = self.send_command('sensors_data')['data']
        data.append(data[2])
        data.append(data[0])
        data.append(data[1])
        return data


    def check_map2(self,angle):
        direction = (np.cos(angle),np.sin(angle))
        for i in range(0, self.sensor_range, 2):
            for dx in range(-self.collision_d,self.collision_d):
                x = int(self.coords[0]/10+direction[0]*i+dx)
                y = int(self.coords[1]/10+direction[1]*i)
                logging.info("x = "+str(x)+" y = " + str(y))
                if x > pf.WORLD_X/10 or x < 0 or y > pf.WORLD_Y/10 or y < 0:
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
                logging.info("x = "+str(x)+" y = " + str(y))
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
    ################# SMALL Robot ############################
    ##########################################################

    def on_sucker(self):
        self.send_command('on_sucker')

    def off_sucker(self):
        self.send_command('off_sucker')

    def rotate_cylinder_horizonal(self):
        logging.info(self.send_command('rotate_cylinder_horizonal'))
        time.sleep(0.3)

    def rotate_cylinder_vertical(self):
        logging.info(self.send_command('rotate_cylinder_vertical',[150.0]))
        time.sleep(0.3)

    def take_cylinder_outside(self):
        logging.info(self.send_command('take_cylinder_outside'))
        time.sleep(1)

    def take_cylinder_inside(self):
        logging.info(self.send_command('take_cylinder_inside'))
        time.sleep(1)

    def lift_up(self):
        logging.info(self.send_command('lift_up',[self.cyl_up[self.cylinders]]))
        time.sleep(0.4)

    def store(self):
        logging.info(self.send_command('lift_up',[self.cyl_prepare[self.cylinders]]))
        # time.sleep(0.5)

    def out_cylinders(self):
        logging.info(self.send_command('lift_up',[self.cyl_down[self.cylinders-1]]))
        time.sleep(0.5)
        self.cylinders -= 1

    def is_start(self):
        return self.send_command('start_flag')['data']

    def pick_up(self):
        #self.rotate_cylinder_vertical()
        self.take_cylinder_inside()
        self.lift_up()
        self.off_sucker()
        self.store()
        self.rotate_cylinder_vertical()
        self.cylinders += 1

    def pick_up2(self):
        self.rotate_cylinder_vertical()
        self.take_cylinder_inside()
        self.lift_up()
        self.off_sucker()
        self.rotate_cylinder_horizonal()



    ############################################################################
    ######## HIGH LEVEL FUNCTIONS ##############################################
    ############################################################################

    def loc_test(self):
        while True:
            angle = 0.0
            parameters = [900, 200, angle, 4]
            self.go_to_coord_rotation(parameters)
            parameters = [900, 400, angle, 4]
            self.go_to_coord_rotation(parameters)
            parameters = [900, 600, angle, 4]
            self.go_to_coord_rotation(parameters)
            parameters = [900, 400, angle, 4]
            self.go_to_coord_rotation(parameters)

    def small_robot_trajectory(self,speed=1):
        signal.signal(signal.SIGALRM, rb.funny_action)
        signal.alarm(90)
        angle = 3*np.pi / 2

        parameters = [850, 170, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        parameters = [1145, 300, angle, speed]
        self.go_last(parameters)
        #self.go_to_coord_rotation(parameters)
        parameters = [1145, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        return
        self.collision_avoidance = False
        self.sensor_range = 60
        speed =4
        parameters = [1145, 250, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True
        self.on_sucker()
        self.take_cylinder_outside()
        self.collision_avoidance = False
        parameters = [1145, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True

        parameters = [1145, 320, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up()
        #time.sleep(3)
        parameters = [1145, 320, angle, speed]
        self.go_to_coord_rotation(parameters)

        self.on_sucker()
        self.take_cylinder_outside()
        self.collision_avoidance = False
        parameters = [1145, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True
        parameters = [1145, 320, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up()

        self.on_sucker()
        self.take_cylinder_outside()
        self.collision_avoidance = False
        parameters = [1145, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True
        speed = 6
        parameters = [1145, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.pick_up()
        time.sleep(5)

        speed = 4

        # cylinder corrections
        self.on_sucker()
        self.take_cylinder_outside()

        parameters = [1245, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1245, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1185, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1185, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        parameters = [1045, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1045, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1100, 200, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1100, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        

        parameters = [1145, 300, angle, speed]
        self.go_to_coord_rotation(parameters)
        ###########
        
        parameters = [1145, 180, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1145, 320, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.rotate_cylinder_horizonal()
        self.sensor_range = 35

    def small_robot_trajectory_r(self, speed=1):
        angle = np.pi
        speed = 4
        parameters = [1150, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/4
        parameters = [1320, 1520, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 6
        return
        parameters = [1320, 1690, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.off_sucker()
        self.take_cylinder_inside()
        parameters = [1210, 1580, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        parameters = [1115, 1490, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        parameters = [1023, 1405, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        speed = 4
        logging.info(self.send_command('lift_up',[30000]))
        parameters = [1150, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)



        ####
        speed = 4
        parameters = [750, 1300, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [830, 1380, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi

        parameters = [350, 1380, angle, speed]
        self.go_to_coord_rotation(parameters)

        parameters = [350, 600, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        parameters = [350, 600, angle, speed]
        self.go_to_coord_rotation(parameters)

        parameters = [350, 600, angle, speed]
        self.go_to_coord_rotation(parameters)

        self.on_sucker()
        self.take_cylinder_outside()
        if self.color == "blue":
            self.rotate_cylinder_horizonal()
        speed = 4
        parameters = [120, 600, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [170, 600, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [170, 900, angle, speed]
        self.go_to_coord_rotation(parameters)

        #parameters = [300, 800, angle, speed]
        #self.go_to_coord_rotation(parameters)
        
        #parameters = [200, 800, angle, speed]
        #self.go_to_coord_rotation(parameters)
        if self.color == "yellow":
            self.rotate_cylinder_horizonal()
        else:
            self.rotate_cylinder_vertical()
        self.off_sucker()

        
        

        

        ### One cylinder more
        

        

        #parameters = [1150, 1000, angle, speed]
        #self.go_to_coord_rotation(parameters)

        ##
        #parameters = [950, 270, 3*np.pi / 2,speed]
        #self.go_last(parameters)

    def funny_action(self, signum, frame):
        self.on_sucker()
        logging.info(self.send_command('stopAllMotors'))
        logging.critical('FUNNNY ACTION')
        exit()

    def collisionTest(self,speed=1):
        angle = 3*np.pi/2
        while True:
            parameters = [1145, 400, angle, speed]
            self.go_to_coord_rotation(parameters)

            parameters = [945, 600, angle, speed]
            self.go_to_coord_rotation(parameters)

            parameters = [1145, 800, angle, speed]
            self.go_to_coord_rotation(parameters)

            parameters = [1345, 1000, angle, speed]
            self.go_to_coord_rotation(parameters)

            parameters = [1545, 800, angle, speed]
            self.go_to_coord_rotation(parameters)

            parameters = [1745, 600, angle, speed]
            self.go_to_coord_rotation(parameters)

            parameters = [1545, 400, angle, speed]
            self.go_to_coord_rotation(parameters)


    def cylinder_test(self):
        #### Cylinder test
        self.on_sucker()
        self.take_cylinder_outside()
        time.sleep(2)
        self.pick_up()

        self.on_sucker()
        self.take_cylinder_outside()
        time.sleep(2)
        self.pick_up()

        self.on_sucker()
        self.take_cylinder_outside()
        time.sleep(2)
        self.pick_up()
        time.sleep(4)

        self.out_cylinders()
        self.out_cylinders()
        self.out_cylinders()
        return


rb = None


def test():
    global rb
    rb = Robot(lidar_on=True, small=True)
    parameters = [900, 200, 0.0, 4]
    rb.go_last(parameters)
    #return
    #rb.go_last(parameters)
    #return
    #while not rb.is_start():
    #    continue
    #while True:
    #    rb.take_cylinder_outside()
    #    time.sleep(3)
    #    rb.take_cylinder_inside()
    #    time.sleep(3)
    #return

    #rb.take_cylinder_inside()
    #return
    #return
    #parameters = [850, 270, 3*np.pi/2, 1]
    #rb.go_last(parameters)
    #return
    #parameters = [1000, 600, np.pi, 4]
    #rb.go_last(parameters)
    #return
    #rb.rotate_cylinder_horizonal()
    #while True:
    #    time.sleep(3)
    #    rb.take_cylinder_outside()
    #    time.sleep(3)
    #
    #return
 #   logging.info(rb.send_command('lift_up',[30000]))
 #   return
    # start system
    #print "go"
    rb.small_robot_trajectory(4)
    return
    rb.small_robot_trajectory_r(4)
    return


try:
    test()
except KeyboardInterrupt:
    rb.p.terminate()
    rb.p2.terminate()
