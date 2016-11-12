import numpy as np
import random
import math
import copy

# want a list of particle objects, want a function for 
# want to be able to resample the list of particles using a transition model
# eventually want to include observations for resampling -> WITHOUT MEASUREMENTS IT IS UNIFORM/NO RESAMPLING!!!


def sample_normal_dist(b):
    """ Normal distribution specified by motion model slides """
    sum = 0
    for i in range(12):
        sum+=.5 * random.uniform(-b, b)
    return sum

class OdometryParticle():
    """ STATE FORMAT *AND* TRANSITION MODEL
    of odometry particle.  See:
    http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf"""
    def __init__(self, x=0, y=0, theta=0, alpha_params=[1, 1, 1, 1]):
        """ @x x position of robot
            @y y position of robot
            @theta angle of robot
            @alpha list of parameters for gaussian normal noise model
            """
        self.alpha = alpha_params
        self.x = x
        self.y = y
        self.theta = theta
        
    def transition(self, action):
        """ move particle according to motion model with appropriate noise
        @action = [drot1, drot2, dtrans]"""
        drot1_h = drot1 + sample_normal_dist(alpha[0]* abs(drot1) + alpha[1]*dtrans)
        dtrans_h = dtrans + sample_normal_dist(alpha[2] * dtrans + alpha[3]*(abs(drot1) + abs(drot2)))
        drot2_h = drot2 + sample_normal_dist(alpha[0]*abs(drot2) + alpha[1]*dtrans)    
        
        self.x = self.x + dtrans_h*math.cos(self.theta + drot1_h)
        self.y = self.y + dtrans_h*math.sin(self.theta + drot1_h)
        self.theta = self.theta + drot1_h + drot2_h
        return  

class ParticleFilter():
    """ Particle filter implementation.  Contains list of particles and a transition model"""
    def __init__(self, starting_particle, num_particles, starting_state_dist = 'None'):
        self.particle_list = []
        if starting_state_dist == 'None':  
            for i in range(num_particles):
                self.particle_list.append(copy.deepcopy(starting_particle))
             
    def update_filter(self, action, measurement = []):
        """ Update the particle filter with an action and a measurement"""
        self._update_particles(action)
        self._measurement_score(measurement)
        self._resample()
        return
             
    def _update_particles(self, action):
        """ Update the particles according to actions particles get new position """
        for particle in self.particle_list:
            particle.transition()
            
    def _measurement_score(self, measurement = []):
        """  Use external measurements to assign scores to particles  """
        return
        
    def _resample():
        return
        
my_particle = OdometryParticle()
my_particle_filter = ParticleFilter(my_particle, 10)