import math
import random
import logging
import numpy as np
from hokuyolx import HokuyoLX
import time
import matplotlib.pyplot as plt

# Dimensions of the playing field
WORLD_X = 3000
WORLD_Y = 2000
INT_MAX = 99999990
mark_r = 40

landmarks = [[-mark_r, WORLD_Y / 2.], [WORLD_X + mark_r, WORLD_Y + mark_r], [WORLD_X + mark_r, - mark_r]]
# Noises
distance_noise = 30.0  # Noise parameter: should be included in move function.
angle_noise = 0.02  # 0.01  # Noise parameter: should be included in move function.
sense_noise = 15
particle_number = 5000  # Number of Particles

# ------------------------------------------------
# 
# this is the Particle class
#
start_position = [170, 170, 0]  # should be two types and not [0,0,0]


# Support Functions


def ideal(x, y):
    """returns almost ideal lidar data"""
    answer = []
    for i in landmarks:
        answer.append(((x - i[0]) ** 2 + (y - i[1]) ** 2) ** 0.5 + 0 * random.gauss(0, distance_noise))
    return answer


class Particle:
    # --------
    # init:
    #    creates Particle and initializes location/orientation
    #

    def __init__(self, a=start_position[0], b=start_position[1], ang=start_position[2]):
        # add gauss errors
        self.x = a + np.random.normal(0, distance_noise)  # initial x position
        self.y = b + np.random.normal(0, distance_noise)  # initial y position
        self.orientation = (ang + np.random.normal(0, angle_noise)) % (2 * math.pi)  # initial orientation

    def set(self, x_new, y_new, orientation_new):
        """Set particle position on the field"""
        self.x = x_new
        self.y = y_new
        self.orientation = orientation_new % (2 * math.pi)  # maybe need to add warning!

    def move(self, delta):
        """Move particle by creating new one and setting position"""
        return Particle(self.x + delta[0], self.y + delta[1], self.orientation + delta[2])

    def pose(self):
        """Return particle pose"""
        return self.x, self.y, self.orientation

    def Gaus(self, x, sigma=sense_noise):
        """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
        return math.exp(- (x ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    def weight(self, x_rob, y_rob, BEACONS):
        """Calculate particle weight based on its pose and lidar data"""
        # TODO check ICP implementation
        temp_beac = [(beacon[0] - self.x, beacon[1] - self.y) for beacon in BEACONS]
        beacons = [(math.cos(self.orientation) * beac[0] + math.sin(self.orientation) * beac[1],
                    -math.sin(self.orientation) * beac[0] + math.cos(self.orientation) * beac[1])
                   for beac in temp_beac]

        beacon = [0, 0, 0]
        num_point = [0, 0, 0]
        for j in xrange(len(x_rob)):
            l1 = abs(math.sqrt((beacons[0][0] - x_rob[j]) ** 2 +
                               (beacons[0][1] - y_rob[j]) ** 2) - 40)
            l2 = abs(math.sqrt((beacons[1][0] - x_rob[j]) ** 2 +
                               (beacons[1][1] - y_rob[j]) ** 2) - 40)
            l3 = abs(math.sqrt((beacons[2][0] - x_rob[j]) ** 2 +
                               (beacons[2][1] - y_rob[j]) ** 2) - 40)
            lmin = l1
            num = 0
            if l2 < lmin:
                lmin = l2
                num = 1
            if l3 < lmin:
                lmin = l3
                num = 2
            if lmin > 200:
                continue
            beacon[num] += lmin
            num_point[num] += 1
        mean = [(beacon[i] / num_point[i]) if num_point[i] != 0 else (1000) for i in xrange(3)]
        # TODO try use median instead mean
        # TODO if odometry works very bad and weights are small use only lidar
        try:
            return self.Gaus(float(sum(mean)) / len(mean))
        except ZeroDivisionError:
            print 'Zero division error in weights'
            return 0

    def __str__(self):
        """Print statement"""
        return 'Particle pose: x = %.2f mm, y = %.2f mm, theta = %.2f deg' \
               % (self.x, self.y, math.degrees(self.orientation))


def get_landmarks(scan):
    """Returns filtrated lidar data"""
    angles = []
    distances = []
    max_intens = 2600  # 2800
    for i in range(len(scan)):
        if scan[i][1] > max_intens and scan[i][0] < 3700:
            angles.append(i * math.pi / 180 / 4)
            distances.append(scan[i][0])
    return angles, distances


def angle5(angle):
    """Transform lidar points from lidar coord sys to Particle cord sys"""
    # if angle >= math.pi/4:
    #	return angle - math.pi/4
    # else:
    #	return angle + 7*math.pi/4
    return (angle + math.pi / 4) % (2 * math.pi)


def p_trans(agl, pit):
    """Transform lidar measurment to xy in Particle coord sys"""
    x_rob = [-1 * pit[i] * math.cos(angle5(agl[i])) for i in xrange(len(agl))]
    y_rob = [-1 * pit[i] * math.sin(angle5(agl[i])) for i in xrange(len(agl))]
    return x_rob, y_rob


def resample(p, w, N):
    """Random pick algorithm for resempling"""
    # TODO check Stanford implementation (speed?)
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
    """Function returns Particle with mean data"""
    p_x = 0
    p_y = 0
    p_orient = 0
    for i in range(len(particles)):
        p_x += particles[i].x
        p_y += particles[i].y
        p_orient += (((particles[i].orientation - particles[0].orientation + math.pi) % (2.0 * math.pi))
                     + particles[0].orientation - math.pi)

    answer = Particle()
    answer.set(p_x / len(particles), p_y / len(particles), p_orient / len(particles))
    logging.info(answer)
    return answer


def particles_move(particles, delta):
    """Function returns new particle set after movement"""
    for i in range(len(particles)):
        particles[i] = particles[i].move(delta)
    return particles


def particles_sense(particles, scan):
    """Calculates new particles according to Lidar data"""
    angle, distance = get_landmarks(scan)
    x_coords, y_coords = p_trans(angle, distance)
    print x_coords
    print y_coords
    weights = []
    for i in range(len(particles)):
        weights.append(particles[i].weight(x_coords, y_coords, landmarks))
    sm = sum(weights)
    if sm == 0:
        weights = [0.0 for i in xrange(len(particles))]
        answer = resample(particles, weights, particle_number)
        return answer
    for i in range(len(weights)):
        weights[i] = weights[i] / sm
    answer = resample(particles, weights, particle_number)

    # print particles weights
    # for i in range(len(particles)):
    # print i
    # print particles[i]
    # print weights[i]
    # print '##########'

    return answer
