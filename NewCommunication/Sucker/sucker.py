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
        self.coll_go = False
        ##################
        self.color = color
        self.cur_state = 0 # 0-neutral,1-suck,2-throw
        self.sensor_range = 35
        self.collision_d = 9
        self.coll_ind = -1
        self.collision_avoidance = False
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
            self.coords = Array('d',rev_field([920, 200,3*np.pi/2],self.color))
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

    def on_mixer(self):
        logging.info(self.send_command('on_mixer'))

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
        angle = 3*np.pi/2
        parameters = [920,1000 , angle, speed]
        self.go_to_coord_rotation(parameters)
        return
    def first_back(self,speed = 4):
        angle = 3*np.pi/2
        parameters = [920, 200, angle, speed]
        self.go_last(parameters)
        return

    def cube(self,speed = 4):
        angle = 3*np.pi/2
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

    def coolers_test(self):
        self.on_mixer()
        self.open_door()
        self.off_coolers()
        #time.sleep(10)
        time.sleep(0.5)
        self.on_coolers_suck()     
        time.sleep(2)
        self.close_door()
        self.on_coolers_throw()
        self.off_coolers()
        time.sleep(0.1)
        self.on_coolers_throw()
        self.off_coolers()
        time.sleep(1)
        self.on_coolers_throw()
        time.sleep(7)
        self.on_coolers_throw()
        time.sleep(10)
        self.off_coolers()
        return

    def test44(self):
        self.suck()
        time.sleep(2)
        self.stop()
        time.sleep(1)
        self.suck()
        time.sleep(1)
        #self.stop()
        self.throw()
        return
        time.sleep(2)
        self.stop()
        time.sleep(2)
        self.throw()
        time.sleep(2)
        self.stop()

    def throw(self):
        self.cur_state = 2
        self.close_door()
        self.off_coolers()
        time.sleep(0.2)
        self.on_coolers_throw()
        self.off_coolers()
        self.on_coolers_throw()
        self.off_coolers()

    def suck(self):
        self.open_door()
        time.sleep(0.1)
        self.off_coolers()
        self.on_coolers_suck()

    def stop(self):
        self.close_door()
        if self.cur_state == 1:
            self.on_coolers_throw()
            self.off_coolers()
        else:
            self.off_coolers()
            self.on_coolers_throw()
        self.cur_state = 0
        


        
        
        
            

def first_strategy():
    rb.first()
    print "GOGOGOGOGOGOGOGO"
    rb.first_back()
def competition(color = "yellow",strategy = 0):
    global rb
    rb = Robot(lidar_on=True, small=True,color=color)
    rb.up_front_seasaw()
    time.sleep(1)
    rb.down_front_seasaw()
    time.sleep(1)
    rb.up_back_seasaw()
    time.sleep(1)
    rb.down_back_seasaw()
    return
    rb.test44()
    return
    #while not rb.is_start():
    #    continue

    strategies = {0:first_strategy,
                  #1:second_strategy,
                  #2:last_strategy,
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
    competition(clr,0)
except KeyboardInterrupt:
    rb.p.terminate()
    rb.p2.terminate()
