import driver
import time
from hokuyolx import HokuyoLX
import logging
import signal
import npParticle as pf
import numpy as np
import sys
from multiprocessing import Process, Queue, Value,Array
import random
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
        self.cyl_prepare = [0.215,0.143,0.143]
        self.cyl_up = [0.523,0.64,0.66]
        self.cyl_down = [-0.79,-0.67,-0.72]
        self.coll_go = False
        self.collision_belts = False
        ##################
        self.color = color
        self.sensor_range = 35
        self.collision_d = 9
        self.coll_ind = -1
        self.collision_avoidance = True
        self.localisation = Value('b', True)
        if small:
            self.sensors_places = [0,0,0,np.pi,np.pi/2,3*np.pi/2,0,0,0]
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
            # 
            self.coords = Array('d',rev_field([170, 170,3*np.pi/2],self.color))
        else:
            driver.PORT_SNR = '325936843235' # need change
            self.coords = Array('d', rev_field([170, 170, 0], self.color))
        self.input_queue = Queue()
        self.loc_queue = Queue()
        self.fsm_queue = Queue() # 2000,25,25,0.1
        self.PF = pf.ParticleFilter(particles=2000, sense_noise=25, distance_noise=25, angle_noise=0.1, in_x=self.coords[0], in_y=self.coords[1], in_angle=self.coords[2],input_queue=self.input_queue, out_queue=self.loc_queue,color = self.color)

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
        #time.sleep(0.1)
        logging.info(self.send_command('rotate_cylinder_vertical',[146.0]))
        #if self.color == "yellow":
        #    self.rotate_cylinder_vertical()
        #else:
        #    self.rotate_cylinder_horizonal()
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
            pm = [self.coords[0]+direction[0]*distance,self.coords[1]+direction[1]*distance,self.coords[2],parameters[3]]
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
        self.PF.prev = [self.coords[0],self.coords[1],self.coords[2]]
        # After movement
        stamp = time.time()
        pids = True
        time.sleep(0.100001)  # sleep because of STM interruptions (Maybe add force interrupt in STM)
        while not (self.send_command('is_point_was_reached')['data']):
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
        data.append(data[2])
        data.append(data[0])
        data.append(data[1])
        if self.collision_belts:
            data[0] =  False
            data[7]= False
            data[1] =  False
            data[8]= False
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
    ################# SMALL Robot ############################
    ##########################################################

    def is_cylinder_taken(self):
        return self.send_command('cylinder_taken')['data']

    def on_sucker(self):
        self.send_command('on_sucker')

    def off_sucker(self):
        self.send_command('off_sucker')
        time.sleep(0.3)

    def rotate_cylinder_horizonal(self):
        if self.color == 'yellow':
            logging.info(self.send_command('rotate_cylinder_horizonal',[249.0]))
        else:
            logging.info(self.send_command('rotate_cylinder_vertical',[146.0]))

        time.sleep(0.1)

    def rotate_cylinder_vertical(self):
        if self.color == 'yellow':
            logging.info(self.send_command('rotate_cylinder_vertical',[146.0]))
        else:
            logging.info(self.send_command('rotate_cylinder_horizonal',[249.0]))
        time.sleep(0.1)

    def take_cylinder_outside(self):
        logging.info(self.send_command('take_cylinder_outside'))
        time.sleep(0.3)

    def take_cylinder_inside(self,rotation = 'l'):
        if self.color == 'yellow':
            if rotation == 'l':
                #rb.rotate_cylinder_vertical()
                logging.info(self.send_command('take_cylinder_inside_l',[249.0]))
            else:
                #rb.rotate_cylinder_horizonal()
                logging.info(self.send_command('take_cylinder_inside_r',[146.0]))
        else:
            if rotation == 'l':
                #rb.rotate_cylinder_vertical()
                logging.info(self.send_command('take_cylinder_inside_r',[146.0]))
            else:
                #rb.rotate_cylinder_horizonal()
                logging.info(self.send_command('take_cylinder_inside_l',[249.0]))
        time.sleep(0.5)

    def lift_up(self):
        logging.info(self.send_command('lift_up',[self.cyl_up[self.cylinders]]))
        time.sleep(self.cyl_up[self.cylinders])

    def store(self):
        logging.info(self.send_command('lift_up',[self.cyl_prepare[self.cylinders]]))
        time.sleep(self.cyl_prepare[self.cylinders])
        # time.sleep(0.5)

    def out_cylinders(self):
        logging.info(self.send_command('lift_up',[self.cyl_down[self.cylinders-1]]))
        time.sleep(0.5)
        self.cylinders -= 1

    def is_start(self):
        return self.send_command('start_flag')['data']

    def off_wheels(self):
        logging.info(self.send_command('off_wheels'))

    def on_wheels(self):
        logging.info(self.send_command('on_wheels'))
        

    def pick_up(self,version =False):
        #self.rotate_cylinder_vertical()
        if(version==False):
            self.take_cylinder_inside()
        else:
            self.take_cylinder_inside('r')
        self.lift_up()
        self.off_sucker()
        self.store()
        if (version==False):
            self.rotate_cylinder_vertical()
        else:
            self.rotate_cylinder_horizonal()
        self.cylinders += 1

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
        #self.rotate_cylinder_vertical()

        parameters = [870, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        parameters = [1150, 300, angle, 4]
        self.go_last(parameters)
        parameters = [1150, 290, angle, 4]
        self.go_to_coord_rotation(parameters)
        #self.go_to_coord_rotation(parameters)
        #parameters = [1150, 290, angle, speed]
        #self.go_to_coord_rotation(parameters)
        self.collision_avoidance = False
        self.sensor_range = 60
        #parameters = [1150, 250, angle, speed]
        #self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True
        self.on_sucker()
        self.take_cylinder_outside()
        self.collision_avoidance = False
        parameters = [1150, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True
        #time.sleep(0.2)

        parameters = [1150, 340, angle, 1]
        self.go_to_coord_rotation(parameters)
        self.pick_up()
        #time.sleep(3)
        #parameters = [1150, 330, angle, speed]
        #self.go_to_coord_rotation(parameters)

        self.on_sucker()
        self.take_cylinder_outside()
        self.collision_avoidance = False
        parameters = [1150, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True
        parameters = [1150, 340, angle, 1]
        self.go_to_coord_rotation(parameters)
        self.pick_up()

        self.on_sucker()
        self.take_cylinder_outside()
        self.collision_avoidance = False
        parameters = [1150, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.collision_avoidance = True
        speed = 4
        parameters = [1150, 340, angle, 1]
        self.go_to_coord_rotation(parameters)
        self.pick_up()

        speed = 4

        # cylinder corrections
        self.on_sucker()
        self.take_cylinder_outside()


        parameters = [1250, 300, angle, 1]
        self.go_to_coord_rotation(parameters)
        parameters = [1250, 210, angle, 1]
        self.go_to_coord_rotation(parameters)
        parameters = [1190, 210, angle, 1]
        self.go_to_coord_rotation(parameters)
        parameters = [1190, 300, angle, 1]
        self.go_to_coord_rotation(parameters)
        
        parameters = [1050, 300, angle, 1]
        self.go_to_coord_rotation(parameters)
        parameters = [1050, 210, angle, 1]
        self.go_to_coord_rotation(parameters)
        parameters = [1105, 210, angle, 1]
        self.go_to_coord_rotation(parameters)
        parameters = [1105, 300, angle, 1]
        self.go_to_coord_rotation(parameters)
        

        #parameters = [1150, 300, angle, speed]
        #self.go_to_coord_rotation(parameters)
        ###########
        
        parameters = [1150, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1150, 340, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.rotate_cylinder_horizonal()
        self.sensor_range = 35

    def small_robot_trajectory_r(self, speed=1):
        angle = 3*np.pi/2
        speed = 4
        parameters = [1150, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/4
        parameters = [1320, 1520, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 6
        #return
        self.collision_belts = True
        parameters = [1350, 1650, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.off_sucker()
        self.take_cylinder_inside()
        parameters = [1230, 1540, angle, speed] # [1250, 1560, angle, speed] no push
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        self.take_cylinder_outside()
        self.take_cylinder_inside()

        ### PUSHING

        parameters = [1180, 1485, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_outside()

        parameters = [1300, 1600, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_inside()

        #########

        
        parameters = [1210, 1510, angle, speed] # [1180, 1485, angle, speed] no push
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        self.take_cylinder_outside()
        self.take_cylinder_inside()
        parameters = [1120, 1440, angle, speed] # [1095, 1410, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        self.take_cylinder_outside()
        self.take_cylinder_inside()
        speed = 4
        self.collision_belts = False
        #logging.info(self.send_command('lift_up',[30000]))
        parameters = [875, 1150, angle, speed]
        self.go_to_coord_rotation(parameters)


        ### multicolor
        angle = 0.2

        parameters = [760, 1390, angle, speed]
        self.go_to_coord_rotation(parameters)
        time.sleep(0.5)
        parameters = [780, 1380, angle, speed]
        self.go_to_coord_rotation(parameters)
        #if self.color == 'yellow': # maybe change
        #    self.rotate_cylinder_horizonal()
        #else:
        #    self.rotate_cylinder_vertical()
            
        self.on_sucker()
        self.take_cylinder_outside()

        parameters = [850, 1410, angle, speed]
        self.go_to_coord_rotation(parameters)
        #rb.rotate_cylinder_vertical()
        
        parameters = [700, 1360, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi
        self.rotate_cylinder_vertical()
        #if self.color == 'yellow': # maybe change
        #    self.rotate_cylinder_vertical()
        #else:
        #    self.rotate_cylinder_horizonal()
        #self.take_cylinder_inside()
        parameters = [600, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [500, 1000, angle, speed] 
        self.go_to_coord_rotation(parameters)
        speed = 4
        parameters = [120, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        #self.take_cylinder_inside()
        #parameters = [130, 1050, angle, speed]
        #self.go_to_coord_rotation(parameters)
        self.off_sucker()

        parameters = [500, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        angle = 3*np.pi/4+np.pi/2
        parameters = [410, 780, angle, speed]
        self.go_to_coord_rotation(parameters)
        rb.rotate_cylinder_horizonal()
        parameters = [410, 780, angle, speed]
        self.go_to_coord_rotation(parameters)


        parameters = [210, 580, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.rotate_cylinder_vertical()
        #if self.color == "yellow":
        #    self.rotate_cylinder_vertical()
        ####self.on_sucker()
        ####self.take_cylinder_outside()
            
        parameters = [130, 500, angle, speed]
        self.go_to_coord_rotation(parameters)

        #parameters = [210, 580, angle, speed]
        #self.go_to_coord_rotation(parameters)

        self.off_wheels()
        self.on_sucker()
        self.take_cylinder_outside()
        self.on_wheels()

        #parameters = [100, 470, angle, speed]
        #self.go_to_coord_rotation(parameters)
        
        parameters = [210, 580, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi
        self.rotate_cylinder_horizonal()
        #if self.color == "yellow":
        #    self.rotate_cylinder_horizonal()
        #else:
        #    self.rotate_cylinder_vertical()
            
        parameters = [300, 800, angle, speed]
        self.go_to_coord_rotation(parameters)

        parameters = [120, 800, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.off_sucker()

        return
        ####
        speed = 4
        #parameters = [750, 1300, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #parameters = [830, 1380, angle, speed]
        #self.go_to_coord_rotation(parameters)
        angle = np.pi

        parameters = [200, 600, angle, speed]
        self.go_to_coord_rotation(parameters)

                    
        parameters = [200, 800, angle, speed]
        self.go_to_coord_rotation(parameters)

        if self.color == "yellow":
            self.rotate_cylinder_horizonal()
        else:
            self.rotate_cylinder_vertical()
        self.off_sucker()
        return
        #parameters = [350, 600, angle, speed]
        #self.go_to_coord_rotation(parameters)
        
        #parameters = [340, 600, angle, speed]
        #self.go_to_coord_rotation(parameters)

        #parameters = [350, 600, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #return
        #self.on_sucker()
        #self.take_cylinder_outside()
        speed = 4
        parameters = [120, 600, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [170, 600, angle, speed] # 250,600 cylinder
        self.go_to_coord_rotation(parameters)
        parameters = [170, 900, angle, speed]
        self.go_to_coord_rotation(parameters)

        #parameters = [300, 800, angle, speed]
        #self.go_to_coord_rotation(parameters)
        
        #parameters = [200, 800, angle, speed]
        #self.go_to_coord_rotation(parameters)
        return

        
        ### One cylinder more
        

        #parameters = [1150, 1000, angle, speed]
        #self.go_to_coord_rotation(parameters)

        ##
        #parameters = [950, 270, 3*np.pi / 2,speed]
        #self.go_last(parameters)

    def second_trajectory_r(self,speed =1):
        angle = 3*np.pi/2
        speed = 4
        parameters = [1250, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/4
        parameters = [1320, 1520, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 6
        #return
        self.collision_belts = True
        parameters = [1350, 1650, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.off_sucker()
        self.take_cylinder_inside()
        parameters = [1230, 1540, angle, speed] # [1250, 1560, angle, speed] no push
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        self.take_cylinder_outside()
        #self.take_cylinder_inside()

        ### PUSHING

        #parameters = [1180, 1485, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #self.take_cylinder_outside()

        parameters = [1300, 1600, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_inside()
        ######### 
        parameters = [1210, 1510, angle, speed] # [1180, 1485, angle, speed] no push
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        self.take_cylinder_outside()
        self.take_cylinder_inside()
        parameters = [1120, 1440, angle, speed] # [1095, 1410, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        self.take_cylinder_outside()
        self.take_cylinder_inside()
        speed = 4
        self.collision_belts = False
        logging.info(self.send_command('lift_up',[2.55]))
        parameters = [875, 1150, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = np.pi
        
        parameters = [300, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.on_sucker()
        self.take_cylinder_outside()
        self.rotate_cylinder_horizonal()
        
        parameters = [160, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [350, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_inside('r')
        
        parameters = [350, 1080, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [180, 1080, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_outside()
        self.off_sucker()


        parameters = [300, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.on_sucker()
        self.take_cylinder_outside()
        self.rotate_cylinder_vertical()
        parameters = [160, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [350, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_inside('l')
        
        parameters = [350, 960, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [180, 960, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_outside()
        self.off_sucker()

        parameters = [300, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.on_sucker()
        self.take_cylinder_outside()
        self.rotate_cylinder_horizonal()
        
        parameters = [160, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [350, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_inside('r')
        
        parameters = [350, 840, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [180, 840, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_outside()
        self.off_sucker()

        

    
    def without_last(self,speed = 1):
        signal.signal(signal.SIGALRM, rb.funny_action)
        signal.alarm(90)
        angle = 3*np.pi/2
        #self.rotate_cylinder_vertical()

        parameters = [870, 160, angle, 6]
        self.go_to_coord_rotation(parameters)
        
        parameters = [1150, 300, angle, 4]
        self.go_last(parameters)
        time.sleep(0.4)
        parameters = [1150, 290, angle, 4]
        self.go_to_coord_rotation(parameters)
        self.sensor_range = 60
        #parameters = [1150, 250, angle, speed]
        #self.go_to_coord_rotation(parameters)
        self.on_sucker()
        self.take_cylinder_outside()
        parameters = [1150, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1150, 340, angle, 1]
        self.go_to_coord_rotation(parameters)
        ### New concevik
        if(self.is_cylinder_taken()==0):
            parameters = [1150, 160, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [1150, 340, angle, 1]
            self.go_to_coord_rotation(parameters)
            
        self.pick_up(self.color=='blue')

        self.on_sucker()
        self.take_cylinder_outside()
        parameters = [1150, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1150, 340, angle, 1]
        self.go_to_coord_rotation(parameters)

        ### New concevik
        if(self.is_cylinder_taken()==0):
            parameters = [1150, 160, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [1150, 340, angle, 1]
            self.go_to_coord_rotation(parameters)
        self.pick_up(self.color=='blue')

        self.on_sucker()
        self.take_cylinder_outside()
        self.rotate_cylinder_vertical()
        parameters = [1150, 160, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 4
        parameters = [1150, 340, angle, 1]
        self.go_to_coord_rotation(parameters)

        ### New concevik
        if(self.is_cylinder_taken()==0):
            parameters = [1150, 160, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [1150, 340, angle, 1]
            self.go_to_coord_rotation(parameters)
        #self.rotate_cylinder_horizonal()

        ###### FIELD ADDITION
        self.take_cylinder_inside()
        #self.pick_up()

        speed = 4

    def without_last_r(self,speed = 1):
        angle = 3*np.pi/2
        speed = 4
        parameters = [1250, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        angle = 3*np.pi/4
        parameters = [1320, 1520, angle, speed]
        self.go_to_coord_rotation(parameters)
        speed = 6
        self.collision_belts = True
        parameters = [1350, 1650, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_outside()
        self.off_sucker()
        self.take_cylinder_inside()
        parameters = [1230, 1540, angle, speed] # [1250, 1560, angle, speed] no push
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        #KL#self.take_cylinder_outside()
        #self.take_cylinder_inside()

        ### PUSHING

        #parameters = [1180, 1485, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #self.take_cylinder_outside()

        #parameters = [1300, 1600, angle, speed]
        #self.go_to_coord_rotation(parameters)
        #KL#self.take_cylinder_inside()
        ######### 
        parameters = [1120, 1440, angle, speed] # [1095, 1410, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.out_cylinders()
        #KL#self.take_cylinder_outside()
        #KL#self.take_cylinder_inside()
        speed = 4
        self.collision_belts = False
        logging.info(self.send_command('lift_up',[2.55]))
        parameters = [875, 1150, angle, speed]
        self.go_to_coord_rotation(parameters)

        angle = np.pi
        
        parameters = [300, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.on_sucker()
        self.take_cylinder_outside()
        self.rotate_cylinder_horizonal()

    ### New concevik
        while(self.is_cylinder_taken()==0):
            #self.take_cylinder_outside()
            parameters = [145, 1350, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [350, 1350, angle, speed]
            self.go_to_coord_rotation(parameters)
            
        self.take_cylinder_inside('r')
        
        parameters = [350, 1080, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [175, 1080, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_outside()
        self.off_sucker()

        parameters = [300, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.on_sucker()
        #self.take_cylinder_outside()
        self.rotate_cylinder_vertical()

    ### New concevik
        while (self.is_cylinder_taken()==0):
            #self.take_cylinder_outside()
            parameters = [145, 1350, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [350, 1350, angle, speed]
            self.go_to_coord_rotation(parameters)
            
        self.take_cylinder_inside('l')
        
        parameters = [350, 960, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [175, 960, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_outside()
        self.off_sucker()

        parameters = [300, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_to_coord_rotation(parameters)
        self.on_sucker()
        #self.take_cylinder_outside()
        self.rotate_cylinder_horizonal()

    ### New concevik
        while (self.is_cylinder_taken()==0):
            #self.take_cylinder_outside()
            parameters = [145, 1350, angle, speed]
            self.go_to_coord_rotation(parameters)
            parameters = [350, 1350, angle, speed]
            self.go_to_coord_rotation(parameters)
            
        self.take_cylinder_inside('r')
        
        parameters = [350, 840, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [175, 840, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.take_cylinder_outside()
        self.off_sucker()

    def bad_move(self,speed =1):
        angle = 0.0
        parameters = [1800, 1000, angle, speed]
        self.go_to_coord_rotation(parameters)
        
        
        
        
    def funny_action(self, signum, frame):
        self.off_sucker()
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

            parameters = [1545, 300, angle, speed]
            self.go_to_coord_rotation(parameters)


    def cylinder_test(self):
        #### Cylinder test
        self.on_sucker()
        self.take_cylinder_outside()
        time.sleep(2)
        print "Before " + str(self.is_cylinder_taken())
        if(True):
            self.pick_up()
        print self.is_cylinder_taken()

        self.on_sucker()
        self.take_cylinder_outside()
        time.sleep(2)
        print "Before " + str(self.is_cylinder_taken())
        if(True):
            self.pick_up()
        print self.is_cylinder_taken()

        self.on_sucker()
        self.take_cylinder_outside()
        time.sleep(2)
        print "Before " + str(self.is_cylinder_taken())
        if(True):
            self.pick_up()
        print self.is_cylinder_taken()
        time.sleep(4)

        self.out_cylinders()
        self.out_cylinders()
        self.out_cylinders()
        return


rb = None

def last_strategy():
    rb.without_last(4)
    rb.without_last_r(4)
    rb.bad_move(6)
    return

def first_strategy():
    rb.small_robot_trajectory(4)
    rb.small_robot_trajectory_r(4)
    return

def second_strategy():
    rb.small_robot_trajectory(4)
    rb.second_trajectory_r(4)
    
    
    


def test(color = "yellow"):
    global rb
    rb = Robot(lidar_on=True, small=True,color=color)
    rb.take_cylinder_outside()
    #rb.rotate_cylinder_horizonal()
    rb.take_cylinder_inside('r')
    #return
    #rb.collisionTest(4)
    #return
    #while True:
    #    continue
    #rb.collisionTest(4)
    #return
    #rb.go_last(parameters)
    #rb.take_cylinder_inside()
    #rb.take_cylinder_outside()
    #rb.take_cylinder_inside()
    #return
    #rb.take_cylinder_outside()
    #rb.take_cylinder_inside()
    #return
    #rb.take_cylinder_inside()
    #return
    #while not rb.is_start():
    #    continue
    #rb.take_cylinder_inside()
    #return
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



    
    #rb.small_robot_trajectory(4)
    #rb.second_trajectory_r(4)
    rb.without_last(4)
    rb.without_last_r(4)
    
    return

def competition(color = "yellow",strategy = 2):
    global rb
    rb = Robot(lidar_on=True, small=True,color=color)
    while not rb.is_start():
        print color
        continue
    strategies = {0:first_strategy,
                  1:second_strategy,
                  2:last_strategy,
                  }
    strategies[strategy]()
    return
    
default_color = "yellow"
try:
    try:
        clr = sys.argv[1]
    except IndexError:
        print "no argument"
        clr = default_color
    competition(clr,2)
except KeyboardInterrupt:
    rb.p.terminate()
    rb.p2.terminate()
