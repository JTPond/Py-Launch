import numpy as np
from rocket import rocket

def get_simple_machine():
    #            position                orientation               velocity                  omega                  mass w/0 boosters   lenght  radius
    sm = rocket(np.array([0.0,0.0,5.0]), np.array([0.0,0.0,1.0]), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), 100.0, 10.0, 0.5)
    #                name         position               length  radius  cstar frho thrust
    sm.add_booster('Booster1',np.array([0.0,0.0,-5.0]), 60.0, 0.45, 20000.0, 1000.0, 5000000.0)
    #                    name              position              orientation          thrust
    sm.add_stabalizer('StabalizerXP',np.array([0.5,0.0,4.5]), np.array([1.0,0.0,0.0]), 1100.0)
    sm.add_stabalizer('StabalizerXN',np.array([-0.5,0.0,4.5]), np.array([-1.0,0.0,0.0]), 1100.0)
    sm.add_stabalizer('StabalizerYP',np.array([0.0,0.5,4.5]), np.array([0.0,1.0,0.0]), 1100.0)
    sm.add_stabalizer('StabalizerYN',np.array([0.0,-0.5,4.5]), np.array([0.0,-1.0,0.0]), 1100.0)
    sm.update_MOI()
    return sm
