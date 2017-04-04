# High-Level Documentation

[Robot simple launch tutorial](https://gist.github.com/Bandd-k/fc296274c5f6b33cb29929696d98cb1b)
## Eurobot Localisation
We use simple particle filter with NumPy operations. To understand how it works look at [Udacity course by Sebastian Thrun](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373).  
We combine data from odometry and lidar (HokuyoLX).  
**The trickest thing** is [the weight function](https://github.com/SkRobo/Eurobot-2017/blob/master/NewCommunication/ParticleFilter.py#L70) of our particle filter. It is based on https://github.com/simama realization.
How it works?  
* We filter all data according to intensity rate from Lidar
* For each particle we calculate theoretical data
* We fit theoretical data to real and get error
* We calculate Gaussian function from error (It is our weight, then we normalize it)
* That's all

[good Particle Filter Example](https://github.com/mjl/particle_filter_demo)

## Finite State Machine
