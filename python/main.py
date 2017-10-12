import numpy as np
import gym

class Main(object):
    def __init__(self, params):
        self.params = params

        self.env = gym.make('DartHopper-v1')
        observation = env.reset()
        for i in range(100):
            observation, reward, done, envinfo = env.step(env.action_space.sample())
                    env.render()

def main():
    import argparse
    import os
    parser = argparse.ArgumentParser(description='Run the Guided Policy Search algorithm.')
    parser.add_argument('--env', type=str, default='Cartpole-v1')
    parser.add_argument('--test', type=bool, default=True)
    parser.add_argument('--iterations', type=int, default=10)
    parser.add_argument('--num_samples', type=bool, default=5)
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
