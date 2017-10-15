from agent import Agent

class Parking(Agent):
    def __init__(self, params):
        Agent.__init__(self, params)
        # model parameters
        self.d = 2.0 # distance btw back and front axles
        self.h = 0.03 # timestep

    def step(self, x, u):
        next_state = self.dyn(x, u)
        cost = self.cost(x, u)
        return next_state, reward, done

    def dyn(self, x, u):
        w = u[0, :, :] # front wheel angle
        a = u[1, :, :] # front wheel acc
        o = x[2, :, :] # car angle
        z = np.array([np.cos(o), np.sin(o)]) # unit_vector(o)
        v = x[3, :, :] # front wheel velcoity
        f = self.h*v # f : front wheel rolling distance
        b = self.d + f * np.cos(w) - sqrt(self.d*self.d - f*np.sin(w)*f*np.sin(w))
        do = np.arcsin(np.sin(w)*f/d)
        dy = np.array([b*z, do, h*a])
        return y = x+dy

    def cost(self, x, u):
        '''
        lu: quadratic cost on controls
        lf: final cost on distance from target parking configuration
        lx: running cost on distance from origin to encourage tight turns
        '''
        isfinal = np.isnan(u[0, :])
        cu = 0.01*np.array([1, 0.01]) # control cost coefficients
        cf = np.array([0.1, 0.1, 1, 0.3]) # final cost coefficients
        pf = np.array([0.01, 0.01, 0.01, 1]) # smoothness scales for final cost
        cx = 0.001*np.array([1, 1])
        px = np.array([0.1, 0.1])
        lu = cu*u*cu*u
        
% final cost
if any(final)
   llf      = cf*sabs(x(:,final),pf);
   lf       = double(final);
   lf(final)= llf;
else
   lf    = 0;
end

% running cost
lx = cx*sabs(x(1:2,:),px);

% total cost
c     = lu + lx + lf;
