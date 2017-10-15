import abc
import numpy as np
from utility.general_utils import isDiverge, generate_noise

class Agent(object):
    __metaclass__ = abc.ABCMeta
    def __init__(self, params):
        self.params = params
        self.alpha = 10 ** np.arange(0, -3.5, -0.5)

    @abc.abstractmethod
    def step(self, s, a):
        raise NotImplementedError("Must be implemented in subclass.")

    def getInitial(self):
        raise NotImplementedError("Must be implemented in subclass.")

    def forward_pass(self, x0, policy, noisy=False):
        """
        :x0: initial state
        :policy: policy fn or input array
        :noisy: add Gaussian noise
        :traj: return trajectory list composed of [state, unew, cnew]
        """
        traj = []
        state = np.zeros([self.nx, self.params.iterations+1])
        state[:, 0] = x0
        cost = np.zeros(self.params.iterations+1)
        if isinstance(policy, np.ndarray):
            for i in range(self.params.num_samples):
                if noisy:
                    policy = generate_noise(policy)
                u = np.zeros([self.nu, self.params.iterations+1])
                u[:, -1] = np.nan
                u[:, :-1] = policy
                for alpha in self.alpha:
                    for t in range(self.params.iterations):
                        state[:, t+1], cost[t], _ = self.step(state[:, t], u[:, t])
                    _, cost[-1], _ = self.step(state[:,-1], u[:,-1])
                    if not isDiverge(state, self.thres):
                        break
                    if alpha == alpha[-1]:
                        raise ValueError("Trajectory is Diverged")
                traj.append({'state_list':state, 'cost_list':cost})
        else:
            pass
        return traj


    def reset(self, condition):
        pass
