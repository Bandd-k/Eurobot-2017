import math
import random

landmarks = [[50.0, 0.0], [100.0, 100.0], [0.0, 100.0]]  # position of 4 landmarks in (x, y) format.
# Dimensions of the playing field  
WORLD_X = 3100
WORLD_Y = 2100
INT_MAX = 99999990

# Noises
distance_noise = 5.0  # Noise parameter: should be included in move function.
angle_noise = 5.0  # Noise parameter: should be included in move function.
sense_noise = 5.0

particle_number = 100  # Number of Particles

# ------------------------------------------------
# 
# this is the robot class
#
start_position = [10, 10, 0]  # should be two types and not [0,0,0]

# Support Functions




class Robot:
    # --------
    # init:
    #    creates robot and initializes location/orientation
    #

    def __init__(self):
        # add gausssian
        self.x = start_position[0] + random.gauss(0, distance_noise)   # initial x position
        self.y = start_position[1] + random.gauss(0, distance_noise) # initial y position
        self.orientation = start_position[2] + random.gauss(0,0.05)  # initial orientation


    # --------
    # set:
    #    sets a robot coordinate
    #


    def set(self, x_new, y_new, orientation_new):
        """Set particle position on the field"""
        if -100 <= x_new <= WORLD_X:
            self.x = x_new
        else:
            raise Exception("Invalid x cord={}".format(x_new))
        if -100 <= y_new <= WORLD_Y:
            self.y = y_new
        else:
            raise Exception("Invalid y cord={}".format(y_new))
        self.orientation = orientation_new % (2 * math.pi)  # maybe need to add warning!

    def move(self, delta):
        """Move particle by creating new one and setting position"""
        x_new = self.x + delta[0]  # + random.gauss(0, 10)
        y_new = self.y + delta[1]  # + random.gauss(0, 10)
        orientation_new = self.orientation + delta[2]  # + random.gauss(0, 0.03)
        new_robot = Robot()
        new_robot.set(x_new, y_new, orientation_new)
        return new_robot

    def pose(self):
        """Return particle pose"""
        return self.x, self.y, self.orientation

    def gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    def weight(self, lidar_data):
        """Calculate particle weight based on its pose and lidar data"""

        if len(lidar_data) == 1:
            minimum = INT_MAX
            for mark in landmarks:
                theory_dist = math.sqrt((self.x - mark[0]) ** 2 + (self.y - mark[1]) ** 2)
                theory_dist = abs(theory_dist - lidar_data[0])
                if minimum > theory_dist:
                    minimum = theory_dist
            minimum += 1
            return 1 / minimum

        elif len(lidar_data) == 2:
            comb = [[0, 1], [0, 2], [1, 2], [1, 0], [2, 0], [2, 1]]  # Precalculated sets
            minimum = INT_MAX
            for st in comb:
                theory_dist = abs(lidar_data[0] - math.sqrt((self.x - landmarks[st[0]][0]) ** 2 + (
                    self.y - landmarks[st[0]][1]) ** 2)) ** 2  # distance to first mark
                theory_dist += abs(lidar_data[1] - math.sqrt((self.x - landmarks[st[1]][0]) ** 2 + (
                    self.y - landmarks[st[1]][1]) ** 2)) ** 2  # distance to second mark
                # theory_dist = dif1^2 + dif2^2
                if minimum > theory_dist:
                    minimum = theory_dist
            minimum += 1
            return 1 / minimum

        elif len(lidar_data) == 3:
            comb = [[2, 0, 1], [1, 0, 2], [0, 1, 2], [2, 1, 0], [1, 2, 0], [0, 2, 1]]  # Precalculated sets
            best = -10
            for st in comb:
                #error = 1
                #error_bearing = abs(lidar_data[0] - math.sqrt((self.x - landmarks[st[0]][0]) ** 2 + (
                #    self.y - landmarks[st[0]][1]) ** 2))
                #error_bearing = (error_bearing + pi) % (2.0 * pi) - pi  # truncate
                prob = 1.0
                dist = math.sqrt((self.x - landmarks[st[0]][0]) ** 2 + (self.y - landmarks[st[0]][1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[0])

                dist = math.sqrt((self.x - landmarks[st[1]][0]) ** 2 + (self.y - landmarks[st[1]][1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[1])

                dist = math.sqrt((self.x - landmarks[st[2]][0]) ** 2 + (self.y - landmarks[st[2]][1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[2])

                if best < prob:
                    best = prob
            return best
                #######
                # theory_dist = abs(lidar_data[0] - math.sqrt((self.x - landmarks[st[0]][0]) ** 2 + (
                #     self.y - landmarks[st[0]][1]) ** 2)) ** 2  # distance to first mark
                #
                # theory_dist += abs(lidar_data[1] - math.sqrt((self.x - landmarks[st[1]][0]) ** 2 + (
                #     self.y - landmarks[st[1]][1]) ** 2)) ** 2  # distance to second mark
                #
                # theory_dist += abs(lidar_data[2] - math.sqrt((self.x - landmarks[st[2]][0]) ** 2 + (
                #     self.y - landmarks[st[2]][1]) ** 2)) ** 2  # distance to third mark

                # theory_dist = dif1^2 + dif2^2 +dif3^2


        else:
            raise Exception("Invalid lidar data len".format(len(lidar_data)))

    def __str__(self):
        """Print statement"""
        return 'Particle pose: x = %.2f mm, y = %.2f mm, theta = %.2f deg' \
               % (self.x, self.y, math.degrees(self.orientation))


def lidar_sense():
    """Function returns landmark data"""
    # distances to landmarks(up to 3)!
    # add to all distances radius of beacon(4 cm)


def resample(p, w, N):
    """Random pick algorithm for resempling"""
    sample = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in xrange(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        sample.append(p[index])
    return sample


def calculate_main(particles):
    """Function returns robot with mean data"""
    p_x = 0
    p_y = 0
    p_orient = 0
    for i in range(len(particles)):
        p_x += particles[i].x
        p_y += particles[i].y
        p_orient += (((particles[i].orientation - particles[0].orientation + math.pi) % (2.0 * math.pi))
                        + particles[0].orientation - math.pi)

    answer = Robot()
    answer.set(p_x / len(particles), p_y / len(particles), p_orient / len(particles))
    return answer


def particles_move(particles, delta):
    """Function returns new particle set after movement"""
    for i in range(len(particles)):
        particles[i] = particles[i].move(delta)
    return particles


def particles_sense(particles, lidar_data):
    """Function returns new particle set after sensing"""
    weights = []
    for i in range(len(particles)):
        weights.append(particles[i].weight(lidar_data))
    sm = sum(weights)
    for i in range(len(weights)):
        weights[i] = weights[i] / sm
    answer = resample(particles, weights, particle_number)
    return answer


def localisation():
    # initialize particles
    particles = [Robot() for i in range(particle_number/2)]
    for i in range(particle_number/2):
        t = Robot()
        t.set(50 ,50 , 0)
        particles.append(t)
    # initialize main robot
    #main_robot = calculate_main(particles)
    #print(main_robot)
    delta = [15, 15, 0]
    # move ->sense
    particles = particles_move(particles, delta)
    main_robot = calculate_main(particles)
    print(main_robot)
    lidar_data = [35.355339, 106.066017, 79.05694]

    for i in range(10):
        particles = particles_sense(particles, lidar_data)
        main_robot = calculate_main(particles)
        print main_robot
    #particles = particles_sense(particles, lidar_data)
    #main_robot = calculate_main(particles)
    #print main_robot
    #particles = particles_sense(particles, lidar_data)
    #main_robot = calculate_main(particles)
    #print main_robot


#######
# test data
#######

#test_lidar = []

#print("finish")
#rob = Robot()
#rob.set(0, 0, 0)
#lid = [35.355339, 106.066017, 79.05694]
#lid = [50, 141.4213562373, 100]
#print(rob.weight(lid))
#rob.set(24, 24, 0)
#print(rob.weight(lid))
#rob.set(23, 25, 0)
#print(rob.weight(lid))
localisation()