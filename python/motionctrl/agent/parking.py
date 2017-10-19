from agent.agent import Agent
import numpy as np
from utility.general_utils import sabs

class Parking(Agent):
    def __init__(self, params):
        # agent parameters
        self.nx = 4
        self.nu = 2
        self.x0 = np.array([1.0, 1.0, np.pi*3./2., 0.0])
        self.ctrl_lims = np.array([ [-0.5, 0.5], [-2.0, 2.0] ])
        self.thres = 1e8
        # geometric parameters
        self.d = 2.0 # distance btw back and front axles
        self.h = 0.03 # timestep
        Agent.__init__(self, params)

    def reset(self):
        return self.x0

    def step(self, x, u):
        """
        Communicate with environment
        Args:
            x: Current state
            u: Input
        Returns:
            next_state: Next state
            cost: Cost
            done: Boolean for terminate
        """
        next_state = self.dyn(x, u)
        cost = self.cost(x, u)
        done = False
        return next_state, cost, done

    def dynAug(self, xu):
        w = xu[4] # front wheel angle
        a = xu[5] # front wheel acc
        o = xu[2] # car angle
        z = np.array([np.cos(o), np.sin(o)]) # unit_vector(o)
        v = xu[3] # front wheel velcoity
        f = self.h*v # f : front wheel rolling distance
        b = self.d + f * np.cos(w) - np.sqrt(self.d*self.d - f*np.sin(w)*f*np.sin(w))
        do = np.arcsin(np.sin(w)*f/self.d)
        dy = np.hstack([b*z, np.array([do, self.h*a])])
        y = xu[0:4]+dy
        return y

    def dyn(self, x, u):
        """
        Dynamics computing next state given current state and input
        Args:
            x: Current state
            u: Input
        Returns:
            y: next state
        """
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

    def costAug(self, xu):
        """
        Compute cost
        Args:
            lu: Quadratic cost on controls
            lf: Final cost on distance from target parking configuration
            lx: Running cost on distance from origin to encourage tight turns
        Returns:
            c: cost
        """
        isfinal = np.isnan(xu[4])
        if isfinal:
            xu[-2:] = np.zeros(self.nu)
        cu = 0.01*np.array([1, 0.01]) # control cost coefficients
        cf = np.array([0.1, 0.1, 1, 0.3]) # final cost coefficients
        pf = np.array([0.01, 0.01, 0.01, 1]) # smoothness scales for final cost
        cx = 0.001*np.array([1, 1])
        px = np.array([0.1, 0.1])
        lu = np.dot(cu, xu[-2:]*xu[-2:])
        if isfinal:
            lf = np.dot(cf, sabs(xu[0:4], pf))
        else:
            lf = 0.0
        lx = np.dot(cx, sabs(xu[0:2], px))
        c = lu + lx + lf
        return c

    def cost(self, x, u):
        """
        Compute cost
        Args:
            lu: Quadratic cost on controls
            lf: Final cost on distance from target parking configuration
            lx: Running cost on distance from origin to encourage tight turns
        Returns:
            c: cost
        """
        isfinal = np.isnan(u[0])
        if isfinal:
            u = np.zeros(self.nu)
        cu = 0.01*np.array([1.0, 0.01]) # control cost coefficients
        cf = np.array([0.1, 0.1, 1., 0.3]) # final cost coefficients
        pf = np.array([0.01, 0.01, 0.01, 1.]) # smoothness scales for final cost
        cx = 0.001*np.array([1.0, 1.0])
        px = np.array([0.1, 0.1])
        lu = np.dot(cu, u*u)
        if isfinal:
            lf = np.dot(cf, sabs(x, pf))
        else:
            lf = 0.0
        lx = np.dot(cx, sabs(x[0:2], px))
        c = lu + lx + lf
        return c

