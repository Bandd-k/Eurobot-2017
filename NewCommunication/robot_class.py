obstacles=[]
class Robot:
    def __init__(self, lidar_on=True,small=True):
        # sensors_number=6
        # self.sensor_range = 20
        # self.collision_avoidance = False
        # if small:
        #     self.sensors_map= {0: (0, np.pi/3), 1: (np.pi/4, np.pi*7/12), 2: (np.pi*0.5, np.pi*1.5), 3: (17/12.*np.pi, 7/4.*np.pi), 4: (5/3.*np.pi,2*np.pi), 5: [(7/4.*np.pi,2*np.pi),(0,np.pi*1/4.)]}  # can be problem with 2pi and 0
        # self.lidar_on = lidar_on
        # self.map = np.load('npmap.npy')
        # if lidar_on:
        #     logging.debug('lidar is connected')
        #     # add check for lidar connection
        #     try:
        #         self.lidar = HokuyoLX(tsync=False)
        #         self.lidar.convert_time = False
        #     except:
        #         self.lidar_on = False
        #         logging.warning('lidar is not connected')
        # #self.x = 170  # mm
        # #self.y = 150  # mm
        # #self.angle = 0.0  # pi
        # if small:
        #     self.coords = Array('d',[850, 170, 0])
        # else:
        #     self.coords = Array('d', [170, 170, 0])
        # self.localisation = Value('b', True)
        # self.input_queue = Queue()
        # self.loc_queue = Queue()
        # self.fsm_queue = Queue()
        # self.PF = pf.ParticleFilter(particles=1500, sense_noise=25, distance_noise=20, angle_noise=0.3, in_x=self.coords[0],
        #                             in_y=self.coords[1], in_angle=self.coords[2],input_queue=self.input_queue, out_queue=self.loc_queue)

        # # driver process
        # self.dr = driver.Driver(self.input_queue,self.fsm_queue,self.loc_queue)
        # p = Process(target=self.dr.run)
        # p.start()
        # p2 = Process(target=self.PF.localisation,args=(self.localisation,self.coords,self.get_raw_lidar))
        # logging.info(self.send_command('echo','ECHO'))
        # logging.info(self.send_command('setCoordinates',[self.coords[0] / 1000., self.coords[1] / 1000., self.coords[2]]))
        # p2.start()
        # time.sleep(0.1)
        pass

    def send_command(self,name,params=None):
        # self.input_queue.put({'source': 'fsm','cmd': name,'params': params})
        # return self.fsm_queue.get()
        pass

    def get_raw_lidar(self):
        # # return np.load('scan.npy')[::-1]
        # timestamp, scan = self.lidar.get_intens()
        # return scan
        # # return scan[::-1]  our robot(old)
        pass

    def check_lidar(self):
        # try:
        #     state = self.lidar.laser_state()
        # except:
        #     self.lidar_on = False
        #     logging.warning('Lidar off')
        pass

    def go_to_coord_rotation(self, parameters):  # parameters [x,y,angle,speed]
        # if self.PF.warning:
        #     time.sleep(1)
        # pm = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2]),parameters[0] / 1000., parameters[1] / 1000., float(parameters[2]), parameters[3]]
        # x = parameters[0] - self.coords[0]
        # y = parameters[1] - self.coords[1]
        # sm = x+y
        # logging.info(self.send_command('go_to_with_corrections',pm))
        # # After movement
        # stamp = time.time()
        # time.sleep(0.100001)  # sleep because of STM interruptions (Maybe add force interrupt in STM)
        # while not self.send_command('is_point_was_reached')['data']:
        #     time.sleep(0.05)
        #     if self.collision_avoidance:
        #         direction = (float(x) / sm, float(y) / sm)
        #         if self.check_collisions(direction):
        #             self.send_command('stopAllMotors')
        #         # check untill ok and then move!
        #     # add Collision Avoidance there
        #     if (time.time() - stamp) > 30:
        #         return False  # Error, need to handle somehow (Localize and add new point maybe)
        # if self.localisation.value == 0:
        #     self.PF.move_particles([parameters[0]-self.coords[0],parameters[1]-self.coords[1],parameters[2]-self.coords[2]])
        #     self.coords[0] = parameters[0]
        #     self.coords[1] = parameters[1]
        #     self.coords[2] = parameters[2]

        # logging.info('point reached')
        # return True
        return

    def check_collisions(self, direction):
        # angle = np.arctan2(direction[1],direction[0]) % (np.pi*2)
        # sensor_angle = (angle-self.coords[2]) %(np.pi*2)
        # #### switch on sensor_angle
        # collisions = [0,0,0,0,1]
        # for index,i in enumerate(collisions):
        #     if i and sensor_angle<=self.sensors_map[index][1] and sensor_angle>=self.sensors_map[index][0]:
        #         logging.info("Collision at index "+str(index))
        #         if self.check_map(direction):
        #             continue
        #         return True
        # return False
        pass

    def receive_sensors_data(self):
        # data = self.send_command('sensors_data')
        # answer = []
        # for i in range(6):
        #     answer.append((data & (1 << i)) != 0)
        # return answer
        pass


    def check_map(self,direction): # probably can be optimized However O(1)
        # for i in range(0,self.sensor_range,2):
        #     for dx in range(-2,2):
        #         for dy in range(-2,2):
        #             x = int(self.coords[0]/10+direction[0]*i+dx)
        #             y = int(self.coords[1]/10+direction[1]*i+dy)
        #             if x > pf.WORLD_X/10 or x < 0 or y > pf.WORLD_Y/10 or y < 0:
        #                 return True
        #                 # Or maybe Continue
        #             if self.map[x][y]:
        #                 return True
        # return False
        pass


    def go_last(self,parameters):
        # while abs(parameters[0]-self.coords[0]) > 10 or abs(parameters[1]-self.coords[1]) > 10:
        #     print 'calibrate'
        #     self.go_to_coord_rotation(parameters)
        pass
    def left_ball_down(self):
    #     self.send_command('left_ball_down')
    #     time.sleep(1)
        pass

    def left_ball_up(self):
    #     self.send_command('left_ball_up')
    #     time.sleep(1)
        pass

    def left_ball_drop(self):
    #     self.send_command('left_ball_drop')
    #     time.sleep(1)
        pass

    def right_ball_down(self):
    #     self.send_command('right_ball_down')
    #     time.sleep(1)
        pass

    def right_ball_up(self):
    #     self.send_command('right_ball_up')
    #     time.sleep(1)
        pass

    def right_ball_drop(self):
    #     self.send_command('right_ball_drop')
    #     time.sleep(1)
        pass

    def funny(self):
    #     self.send_command('funny_action')
    #     time.sleep(1)
        pass

    def on_sucker(self):
    #     self.send_command('on_sucker')
        pass

    def off_sucker(self):
    #     self.send_command('off_sucker')
        pass

    def rotate_cylinder_horizonal(self):
    #     logging.info(self.send_command('rotate_cylinder_horizonal'))
    #     time.sleep(0.2)
        pass

    def rotate_cylinder_vertical(self):
    #     logging.info(self.send_command('rotate_cylinder_vertical'))
    #     time.sleep(0.2)
        pass

    def take_cylinder_outside(self):
    #     logging.info(self.send_command('take_cylinder_outside'))
    #     time.sleep(0.5)
        pass

    def take_cylinder_inside(self):
    #     logging.info(self.send_command('take_cylinder_inside'))
    #     time.sleep(0.5)
        pass

    def lift_up(self):
    #     logging.info(self.send_command('lift_up'))
    #     time.sleep(0.5)
        pass

    def store(self):
    #     logging.info(self.send_command('store'))
    #     # time.sleep(0.5)
        pass

    def out_cylinders(self):
    #     logging.info(self.send_command('out_cylinders'))
    #     time.sleep(0.5)
        pass

    def pick_up(self):
    #     self.rotate_cylinder_vertical()
    #     self.take_cylinder_inside()
    #     self.lift_up()
    #     self.off_sucker()
    #     self.store()
    #     self.rotate_cylinder_horizonal()
        pass

    def cyl_test(self):
    #     self.pick_up()
    #     self.pick_up()
    #     self.pick_up()
    #     self.out_cylinders()
        pass