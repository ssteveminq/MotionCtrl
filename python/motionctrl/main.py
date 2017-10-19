import numpy as np
from agent.parking import Parking
from algorithm.traj_opt.ilqr import iLQR
import matplotlib.pyplot as plt

class Main(object):
    def __init__(self, params):
        self.params = params
        if params.env == 'parking':
            self.agent = Parking(params)
        else:
            pass
        if params.traj_opt== 'iLQR':
            self.algorithm = iLQR(self.agent, self.params)
        else:
            pass

    def run(self):
        # Run algorithm with arbitrary input
        u0 = 0.1 * np.ones([self.agent.nu, self.params.iterations])
        traj_list, L, Vx, Vxx = self.algorithm.run(u0)
        plt.plot(traj_list[0]['state_list'][0,:], traj_list[0]['state_list'][1,:], '*')
        plt.show()

def main():
    import argparse
    import os
    parser = argparse.ArgumentParser(description='Run the iLQR algorithm.')
    parser.add_argument('--env', type=str, default='parking')
    parser.add_argument('--test', type=bool, default=False)
    parser.add_argument('--iterations', type=int, default=500)
    parser.add_argument('--num_samples', type=bool, default=1)
    parser.add_argument('--traj_opt', type=str, default='iLQR')
    parser.add_argument('--done', type=str, default='done!')
    args = parser.parse_args()

    main = Main(args)
    if not args.test:
        main.run()
    else:
        print("Not implemented yet")

    print(args.done)

def sum(x):
    return x[1:4]

if __name__ == "__main__":
    main()
    print("Test Done")
