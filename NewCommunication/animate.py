'''Animates distances using single measurment mode'''
from hokuyolx import HokuyoLX
import matplotlib.pyplot as plt

DMAX = 3000
IMIN = 300.
IMAX = 2000.

images = []
fig = None

def get_colors(intens):
    max_val = intens.max()
    return np.repeat(intens, 3).reshape((4,3))/max_val 

def update(laser, plot, text):
    timestamp, scan = laser.get_filtered_intens(dmax=DMAX,imin= 2300)
    kscan = []
    #for i in range(len(scan)):
    #    if scan[i][1]>2000 and scan[i][0]<4000:
    #        kscan.append(scan[i][1])
    #scan = kscan
        
    plot.set_offsets(scan[:, :2])
    plot.set_array(scan[:, 2])
    text.set_text('t: %d' % timestamp)
    plt.show()
    plt.pause(0.001)

def run():
    plt.ion()
    laser = HokuyoLX(tsync=False)
    laser.convert_time=False
    ax = plt.subplot(111, projection='polar')
    plot = ax.scatter([0, 1], [0, 1], s=10, c='b', lw=0)
    text = plt.text(0, 1, '', transform=ax.transAxes)
    ax.set_rmax(DMAX)
    ax.grid(True)
    images.append((plot,))
    plt.show()
    while plt.get_fignums():
        update(laser, plot, text)
    laser.close()

if __name__ == '__main__':
    run()
    #from matplotlib.animation import ArtistAnimation
    #line_anim = ArtistAnimation(fig, images, interval=50, blit=True)
    #
    line_anim.save('my_animation.mp4')
