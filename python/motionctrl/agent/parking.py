from agent.agent import Agent
import numpy as np
from utility.general_utils import sabs

# import sys
# import os.path
# sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
# from utility.general_utils import sabs

class Parking(Agent):
    def __init__(self, params):
        # agent parameters
        self.nx = 4
        self.nu = 2
        self.x0 = np.array([1.0, 1.0, 4.7124, 0.0])
        self.ctrl_lims = np.array([ [-0.5, 0.5], [-2.0, 2.0] ])
        self.thres = 1e8
        # geometric parameters
        self.d = 2.0 # distance btw back and front axles
        self.h = 0.03 # timestep
        Agent.__init__(self, params)

    def getInitial(self):
        return self.x0

    def step(self, x, u):
        next_state = self.dyn(x, u)
        cost = self.cost(x, u)
        done = False
        return next_state, cost, done

    def dyn(self, x, u):
        w = u[0] # front wheel angle
        a = u[1] # front wheel acc
        o = x[2] # car angle
        z = np.array([np.cos(o), np.sin(o)]) # unit_vector(o)
        v = x[3] # front wheel velcoity
        f = self.h*v # f : front wheel rolling distance
        b = self.d + f * np.cos(w) - np.sqrt(self.d*self.d - f*np.sin(w)*f*np.sin(w))
        do = np.arcsin(np.sin(w)*f/self.d)
        dy = np.hstack([b*z, np.array([do, self.h*a])])
        y = x+dy
        return y

    def cost(self, x, u):
        '''
        lu: quadratic cost on controls
        lf: final cost on distance from target parking configuration
        lx: running cost on distance from origin to encourage tight turns
        '''
        isfinal = np.isnan(u[0])
        if isfinal:
            u = np.zeros(self.nu)
        cu = 0.01*np.array([1, 0.01]) # control cost coefficients
        cf = np.array([0.1, 0.1, 1, 0.3]) # final cost coefficients
        pf = np.array([0.01, 0.01, 0.01, 1]) # smoothness scales for final cost
        cx = 0.001*np.array([1, 1])
        px = np.array([0.1, 0.1])
        lu = np.dot(cu, u*u)
        if isfinal:
            lf = np.dot(cf, sabs(x, pf))
        else:
            lf = 0.0
        lx = np.dot(cx, sabs(x[0:2], px))
        c = lu + lx + lf
        return c

if __name__ == "__main__":
    x = np.array([1,1,4.7124,0])
    u = np.array([-0.1050, 0.0469])
    agent = Parking()
    next_state, cost, _ = agent.step(x, u)
    print(next_state)
    print(cost)

    print("test done")
