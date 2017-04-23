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
        ## CAN BE PASTED start
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
        ## CAN BE PASTED end
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


def test():
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
