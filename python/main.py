import numpy as np
from agent import Agent

class Main(object):
    def __init__(self, params):
        self.params = params
        self.agent = Agent(params.env)

        self.env = gym.make(params)
        observation = self.env.reset()
        for i in range(100):
            observation, reward, done, envinfo = self.env.step(self.env.action_space.sample())
            self.env.render()

def main():
    import argparse
    import os
    parser = argparse.ArgumentParser(description='Run the Guided Policy Search algorithm.')
    parser.add_argument('--env', type=str, default='CartPole-v0')
    parser.add_argument('--test', type=bool, default=True)
    parser.add_argument('--iterations', type=int, default=500)
    parser.add_argument('--num_samples', type=bool, default=1)
    parser.add_argument('--traj_optimizer', type=str, default='iLQR')
    parser.add_argument('--done', type=str, default='done!')
    args = parser.parse_args()

    main = Main(args)
    if not args.test:
        Main.run()
    else:
        print("Not implemented yet")

    print(args.done)
if __name__ == "__main__":
    main()
