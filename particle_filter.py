import numpy as np
import numpy as np
import math
import scipy.stats
from numpy.random import choice
import copy

np.random.seed(0)

class Particle:
    def __init__(self, x, y, theta, ln_p):
        self.x = x
        self.y = y
        self.theta = theta
        self.ln_p = ln_p


class ParticleFilter:
    def __init__(self, map, num_particles, translation_variance, rotation_variance, measurement_variance):
        self._particles = []
        self._map = map
        self._translation_variance = translation_variance
        self._rotation_variance = rotation_variance
        self._measurement_variance = measurement_variance
        # sample possible states for particles uniformly
        p = 1.0 / num_particles
        for i in range(0, num_particles):
            self._particles.append(Particle(
                np.random.uniform(map.bottom_left[0], map.top_right[0]),
                np.random.uniform(map.bottom_left[1], map.top_right[1]),
                # choice([-math.pi, -math.pi/2, 0, math.pi/2]),
                np.random.uniform(0, 2*math.pi),
                # 0,
                math.log(p)))

    def move_by(self, x, y, theta):
        for particle in self._particles:
            particle.x += x + np.random.normal(0.0, self._translation_variance)
            particle.y += y + np.random.normal(0.0, self._translation_variance)
            particle.theta += theta + np.random.normal(0.0, self._rotation_variance)
            # bound check
            particle.x = min(self._map.top_right[0], max(particle.x, self._map.bottom_left[0]))
            particle.y = min(self._map.top_right[1], max(particle.y, self._map.bottom_left[1]))
            particle.theta = math.fmod(particle.theta, 2 * math.pi)
        #self.particles_for_map = self._particles

    def measure(self, z, servo_angle_in_rad):
        for particle in self._particles:
            # compute what the distance should be, if particle position is accurate
            distance = self._map.closest_distance([particle.x, particle.y], particle.theta + servo_angle_in_rad)
            print(particle.theta + servo_angle_in_rad)
            print([particle.x, particle.y])
            print(distance)
            # compute the probability P[measured z | robot @ x]
            p = scipy.stats.norm(distance, self._measurement_variance).pdf(z)
            if p == 0:
                p_measured_z_if_robot_at_x = float("-inf")
            else:
                p_measured_z_if_robot_at_x = math.log(p)
            # compute the probability P[robot@x | measured]
            #    NOTE: This is not a probability, since we don't know P[measured z]
            #          Hence we normalize afterwards
            particle.ln_p = p_measured_z_if_robot_at_x + particle.ln_p
        # normalize probabilities (take P[measured z into account])
        probabilities = np.array([particle.ln_p for particle in self._particles])
        a = scipy.special.logsumexp(probabilities)
        probabilities -= a
        for j in range(0, len(probabilities)):
            self._particles[j].ln_p = probabilities[j]

        # resample
        self._particles = choice(self._particles, len(self._particles), p=[math.exp(particle.ln_p) for particle in self._particles])
        self._particles = [copy.copy(particle) for particle in self._particles]

    def get_estimate(self):
        weights = [math.exp(particle.ln_p) for particle in self._particles]
        x = np.average([particle.x for particle in self._particles], weights=weights)
        y = np.average([particle.y for particle in self._particles], weights=weights)
        theta = np.average([particle.theta for particle in self._particles], weights=weights)
        return x, y, theta
