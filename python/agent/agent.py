import abc
import numpy

class Agent(object):
    __metaclass__ = abc.ABCMeta
    def __init__(self, params):
        self.params = params

    @abc.abstractmethod
    def step(self, s, a):
        raise NotImplementedError("Must be implemented in subclass.")

    def sample(self, x0, policy, noisy=False):
        if isinstance(policy, np.ndarray):
            x = np.zeros([ns, ])
            for i in self.params.num_samples:
                for t in iterations:

            return
        else:
            for i in self.params.num_samples:
                obs = self.env.reset()
                for t in self.params.iterations:
                    pass


    def reset(self, condition):


