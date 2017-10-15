import numpy as np
from agent.agent import Agent
from agent.parking import Parking

class Main(object):
    def __init__(self, params):
        self.params = params
        if params.env == 'parking':
            self.agent = Parking(params)
        else:
            pass
    def run(self):
        u0 = 0.1 * np.ones([self.agent.nu, self.params.iterations])
        # forword step
        traj = self.agent.forward_pass(self.agent.getInitial(), u0)
        # 
        # 

def main():
    import argparse
    import os
    parser = argparse.ArgumentParser(description='Run the iLQR algorithm.')
    parser.add_argument('--env', type=str, default='parking')
    parser.add_argument('--test', type=bool, default=False)
    parser.add_argument('--iterations', type=int, default=500)
    parser.add_argument('--num_samples', type=bool, default=1)
    parser.add_argument('--traj_optimizer', type=str, default='iLQR')
    parser.add_argument('--done', type=str, default='done!')
    args = parser.parse_args()

    main = Main(args)
    if not args.test:
        main.run()
    else:
        print("Not implemented yet")

    print(args.done)
if __name__ == "__main__":
    main()
    print("Test Done")
