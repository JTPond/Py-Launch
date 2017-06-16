import numpy as np

class stabalizer(object):
    """docstring for stabalizer."""
    def __init__(self, position, orientation, thrust):
        self.position = position
        self.orientation = orientation
        self.thrust = thrust
        self.on = 0.0

    def ignition(self):
        self.on = abs(self.on - 1.0)

    def set_position(self, value):
        self.position = value

    def set_orientation(self, value):
        self.orientation = value

class booster(object):
    """docstring for booster."""
    def __init__(self, position, length, radius, cStar, frho, thrust):
        self.position = position
        self.length = length
        self.radius = radius
        self.cStar = cStar
        self.frho = frho
        self.thrust = thrust
        self.dfdt = self.thrust/self.cStar
        self.fuel = self.get_fuel_mass()
        self.on = 0.0

    def ignition(self):
        self.on = abs(self.on - 1.0)

    def set_position(self, value):
        self.position = value

    def update_fuel(self, dt):
        if self.fuel > 0.0:
            self.fuel = self.fuel - self.dfdt*dt
            return True
        else:
            self.thrust = 0.0
            return False

    def get_fuel_mass(self):
        return np.pi*(self.radius**2.0)*self.length*self.frho

class rocket(object):
    """docstring for rocket."""
    def __init__(self, position, orientation, velocity, omega, mass, length, radius):
        self.position = position
        self.orientation = orientation
        self.velocity = velocity
        self.omega  = omega
        self.mass = mass
        self.length = length
        self.radius = radius
        self.stabalizers = {}
        self.boosters = {}
        self.MOI = np.zeros(shape=(3,3))

    def update_MOI(self):
        self.MOI[0,0] = (1.0/12.0)*self.m()*(3.0*self.radius**2.0 + self.length**2.0)
        self.MOI[1,1] = (1.0/12.0)*self.m()*(3.0*self.radius**2.0 + self.length**2.0)
        self.MOI[2,2] = (1.0/2.0)*self.m()*(self.radius**2.0)

    def add_booster(self, name, position, length, radius, cStar, frho, thrust):
        self.boosters[name] = (booster(position, length, radius, cStar, frho, thrust))

    def add_stabalizer(self, name, position, orientation, thrust):
        self.stabalizers[name] = (stabalizer(position, orientation, thrust))

    def m(self):
        return self.mass+ sum([self.boosters[boo].fuel for boo in self.boosters])

    def alpha(self):
        tor = np.zeros(3)
        for stab in self.stabalizers:
            tor[0] += (self.stabalizers[stab].position[2])*self.stabalizers[stab].thrust*self.stabalizers[stab].on*self.stabalizers[stab].orientation[1]
            tor[1] += -(self.stabalizers[stab].position[2])*self.stabalizers[stab].thrust*self.stabalizers[stab].on*self.stabalizers[stab].orientation[0]
        return tor/np.array([self.MOI[0,0],self.MOI[1,1],self.MOI[2,2]])

    def a(self):
        return sum([self.boosters[boo].on*self.boosters[boo].thrust*self.orientation for boo in self.boosters])/self.m()

    def rotate(self,dt):
        for j in range(3):
            if j == 0:
                if (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt) > 0.0:
                    if self.orientation[1] > 0.0 and self.orientation[2] >= 0.0:
                        self.orientation[1] -= (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                        self.orientation[2] += (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                    elif self.orientation[1] <= 0.0 and self.orientation[2] > 0.0:
                        self.orientation[1] -= (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                        self.orientation[2] -= (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                    elif self.orientation[1] < 0.0 and self.orientation[2] <= 0.0:
                        self.orientation[1] += (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                        self.orientation[2] -= (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                    elif self.orientation[1] >= 0.0 and self.orientation[2] < 0.0:
                        self.orientation[1] += (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                        self.orientation[2] += (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                elif (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt) < 0.0:
                    if self.orientation[1] >= 0.0 and self.orientation[2] > 0.0:
                        self.orientation[1] += (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                        self.orientation[2] -= (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                    elif self.orientation[1] < 0.0 and self.orientation[2] >= 0.0:
                        self.orientation[1] += (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                        self.orientation[2] += (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                    elif self.orientation[1] <= 0.0 and self.orientation[2] < 0.0:
                        self.orientation[1] -= (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                        self.orientation[2] += (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                    elif self.orientation[1] > 0.0 and self.orientation[2] <= 0.0:
                        self.orientation[1] -= (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
                        self.orientation[2] -= (self.omega[0]*dt + 0.5*self.alpha()[0]*dt*dt)
            elif j == 1:
                if (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt) > 0.0:
                    if self.orientation[2] > 0.0 and self.orientation[0] >= 0.0:
                        self.orientation[2] -= (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                        self.orientation[0] += (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                    elif self.orientation[2] <= 0.0 and self.orientation[0] > 0.0:
                        self.orientation[2] -= (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                        self.orientation[0] -= (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                    elif self.orientation[2] < 0.0 and self.orientation[0] <= 0.0:
                        self.orientation[2] += (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                        self.orientation[0] -= (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                    elif self.orientation[2] >= 0.0 and self.orientation[0] < 0.0:
                        self.orientation[2] += (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                        self.orientation[0] += (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                elif (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt) < 0.0:
                    if self.orientation[2] >= 0.0 and self.orientation[0] > 0.0:
                        self.orientation[2] += (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                        self.orientation[0] -= (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                    elif self.orientation[2] < 0.0 and self.orientation[0] >= 0.0:
                        self.orientation[2] += (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                        self.orientation[0] += (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                    elif self.orientation[2] <= 0.0 and self.orientation[0] < 0.0:
                        self.orientation[2] -= (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                        self.orientation[0] += (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                    elif self.orientation[2] > 0.0 and self.orientation[0] <= 0.0:
                        self.orientation[2] -= (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
                        self.orientation[0] -= (self.omega[1]*dt + 0.5*self.alpha()[1]*dt*dt)
            elif j == 2:
                if (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt) > 0.0:
                    if self.orientation[0] > 0.0 and self.orientation[1] >= 0.0:
                        self.orientation[0] -= (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                        self.orientation[1] += (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                    elif self.orientation[0] <= 0.0 and self.orientation[1] > 0.0:
                        self.orientation[0] -= (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                        self.orientation[1] -= (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                    elif self.orientation[0] < 0.0 and self.orientation[1] <= 0.0:
                        self.orientation[0] += (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                        self.orientation[1] -= (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                    elif self.orientation[0] >= 0.0 and self.orientation[1] < 0.0:
                        self.orientation[0] += (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                        self.orientation[1] += (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                elif (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt) < 0.0:
                    if self.orientation[0] >= 0.0 and self.orientation[1] > 0.0:
                        self.orientation[0] += (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                        self.orientation[1] -= (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                    elif self.orientation[0] < 0.0 and self.orientation[1] >= 0.0:
                        self.orientation[0] += (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                        self.orientation[1] += (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                    elif self.orientation[0] <= 0.0 and self.orientation[1] < 0.0:
                        self.orientation[0] -= (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                        self.orientation[1] += (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                    elif self.orientation[0] > 0.0 and self.orientation[1] <= 0.0:
                        self.orientation[0] -= (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
                        self.orientation[1] -= (self.omega[2]*dt + 0.5*self.alpha()[2]*dt*dt)
