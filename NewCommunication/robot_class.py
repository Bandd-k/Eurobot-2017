
class Robot:
    def __init__(self, lidar_on=True, config_data=None):
        self.lidar_on = lidar_on
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
        # self.coords = Array('d',[170, 150, 0])
        # self.input_queue = Queue()
        # self.loc_queue = Queue()
        # self.fsm_queue = Queue()
        # self.PF = pf.ParticleFilter(particles=800, sense_noise=30, distance_noise=30, angle_noise=0.1, in_x=self.coords[0],
        #                             in_y=self.coords[1],input_queue=self.input_queue,out_queue=self.loc_queue)
        #
        #
        # # driver process
        # self.dr = driver.Driver(self.input_queue,self.fsm_queue,self.loc_queue)
        # p = Process(target=self.dr.run)
        # p.start()
        # p2 = Process(target=self.PF.localisation,args=(self.coords,self.get_raw_lidar))
        # logging.info(self.send_command('echo','ECHO'))
        # logging.info(self.send_command('setCoordinates',[self.coords[0] / 1000., self.coords[1] / 1000., self.coords[2]]))
        # p2.start()

    def send_command(self,name,params=None):
        pass

    def get_raw_lidar(self):
        pass

    def check_lidar(self):
        pass

    def go_to_coord_rotation(self, parameters):  # parameters [x,y,angle,speed]
        pass

    def go_last(self,parameters):
        pass