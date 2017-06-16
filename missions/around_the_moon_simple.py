import numpy as np
from machines import get_simple_machine
from satellite import satellite


sm = get_simple_machine()
moon = satellite(7.342e22, 1737.1e3, np.array([1500e3,1500e3,4000e3]), 1.022e3)

def gravity():
    G = 6.67408e-11
    Me = 6e24
    Mm = moon.mass
    rmrp = sm.position[2] + 6371e3
    rRrM = sm.position - moon.position
    R2 = (rRrM*rRrM).sum()
    aG = ((G*Me)/rmrp**2.0)*np.array([0.0,0.0,-1.0])
    mG = -((G*moon.mass)/R2) *(rRrM/np.sqrt(R2))
    return aG + mG

def translate(dt):
    if (sm.position + sm.velocity*dt + 0.5*(sm.a()+gravity())*dt*dt)[2] >= 0.0:
        sm.position += (sm.velocity*dt + 0.5*(sm.a()+gravity())*dt*dt)
        return True
    else:
        return False

def timeStep(dt):
    a1 = sm.a() + gravity()
    alpha1 = sm.alpha()
    if (translate(dt)):
        sm.rotate(dt)
        sm.velocity += 0.5*(a1 + sm.a() + gravity())*dt
        sm.omega += 0.5*(alpha1 + sm.alpha())*dt
        for booster in sm.boosters:
            if not (sm.boosters[booster].update_fuel(dt)):
                return 'empty'
        sm.update_MOI()
        return 'success'
    else:
        return 'grounded'
