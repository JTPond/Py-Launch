import numpy as np

class satellite(object):
    """docstring for satellite."""
    def __init__(self, mass, radius, position, orbital_velocity):
        self.mass = mass
        self.radius = radius
        self.position = position
        self.orbital_velocity = orbital_velocity
