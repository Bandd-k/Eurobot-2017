from hokuyolx import HokuyoLX
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, sqrt, atan2
import time
d2r = pi/180
def circle_intersection(circle1, circle2):
        '''
        @summary: calculates intersection points of two circles
        @param circle1: tuple(x,y,radius)
        @param circle2: tuple(x,y,radius)
        @result: tuple of intersection points (which are (x,y) tuple)
        '''
        # return self.circle_intersection_sympy(circle1,circle2)
        x1,y1,r1 = circle1
        x2,y2,r2 = circle2
        # http://stackoverflow.com/a/3349134/798588
        dx,dy = x2-x1,y2-y1
        d = sqrt(dx*dx+dy*dy)
        if d > r1+r2:
            print ("#1")
            return None # no solutions, the circles are separate
        if d < abs(r1-r2):
            print ("#2")
            return None # no solutions because one circle is contained within the other
        if d == 0 and r1 == r2:
            print ("#3")
            return None # circles are coincident and there are an infinite number of solutions

        a = (r1*r1-r2*r2+d*d)/(2*d)
        h = sqrt(r1*r1-a*a)
        xm = x1 + a*dx/d
        ym = y1 + a*dy/d
        xs1 = xm + h*dy/d
        xs2 = xm - h*dy/d
        ys1 = ym - h*dx/d
        ys2 = ym + h*dx/d

        return (xs1,ys1),(xs2,ys2)


def get_beacons(scan):
        max_intens = scan[:,1].max()*0.8
        beacons = []
        print(max_intens)# debug
        prev =-1
        dist = 99999
        for i in range(1080):
                if(scan[i][1]>max_intens):
                        if(prev==-1):
                                dist = scan[i][0]
                                prev = i
                                continue
                        if(scan[i][0]<dist and (abs(dist-scan[i][0])<100) and abs(i-prev)<7):
                                dist = scan[i][0]
                                prev = i
                        elif(abs(scan[i][0]-dist)>100 or abs(i-prev)>7):
                                beacons.append((prev*0.25-135,dist))
                                prev = i
                                dist = scan[i][0]
        beacons.append((prev*0.25-135,dist))
        return beacons


field_x = 2000
field_y = 3000
BEACONS = [(0,field_y),(field_x,field_y),(field_x/2,0)]
class robot():
        #lidar,x,y
        def __init__(self):
                self.lidar = HokuyoLX(tsync=False)
                self.lidar.convert_time = False
                self.x = 0
                self.y = 0
        def update_pos(self):
                timestamp, scan = self.lidar.get_intens()
                beacons = get_beacons(scan)
                circle1 = (field_x,field_y,beacons[0][1])
                circle2= (0,field_y,beacons[1][1])
                print(circle1)
                print(circle2)
                answer = circle_intersection(circle1,circle2)
                if(answer[0][0]>0 and answer[0][0]<2000 and answer[0][1]>0 and answer[0][1]<3000):
                        return answer[0]
                else:
                        return answer[1]
                
                
def run():
    plt.ion()
    rbt = robot()
    #plot = plt.plot([1,1], 'ro')
    ax = plt.subplot(111)
    plot = ax.plot([], [], 'ro')[0]
    plt.axis([0, 2000, 0, 3000])
    plt.show()
    cord = rbt.update_pos()
    while plt.get_fignums():
        plot.set_data(cord[0],cord[1])
        #get coordinates
        cord = rbt.update_pos()
        plt.pause(0.3)
        time.sleep(0.2)

        



if __name__ == '__main__':
        run()

