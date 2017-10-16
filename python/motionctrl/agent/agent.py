import abc

class Agent(object):
    __metaclass__ = abc.ABCMeta
    def __init__(self, params):
        self.params = params

    @abc.abstractmethod
    def step(self, s, a):
        raise NotImplementedError("Must be implemented in subclass.")

    @abc.abstractmethod
    def reset(self):
        raise NotImplementedError("Must be implemented in subclass.")

    @abc.abstractmethod
    def dynCstDiff(self, xu):
        raise NotImplementedError("Must be implemented in subclass.")
