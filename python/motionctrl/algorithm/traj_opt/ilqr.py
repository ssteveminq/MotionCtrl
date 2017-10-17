from agent.agent import Agent
from utility.general_utils import finite_differences, isDiverge, generate_noise
from algorithm.boxqp import boxQP
import numpy as np

class iLQR(object):
    def __init__(self, agent, params):
        self.agent = agent
        self.params = params
        self.alpha = 10 ** np.arange(0, -3.5, -0.5)

    def run(self, u0):
        """
        Run main algorithm of iLQR
        Args:
            u0: Initial input sequence
        Returns:

        """
        u = u0
        ### Step 1 : Forword step, differentiate dynamics and cost along new trajectory
        traj_list = self.forward_pass(self.agent.reset(), u)
        for traj in traj_list:
            fx, fu, cx, cu, cxx, cxu, cuu = self.dynCstDiff(traj)

        ### Step 2 : Backward pass, compute optimal control law and cost to go
        diverge, Vx, Vxx, k, K, dV = self.backward_pass(cx, cu, cxx, \
                                                        cxu, cuu, fx, fu, \
                                                        1, \
                                                        self.agent.ctrl_lims, u)
        import ipdb
        ipdb.set_trace()
        exit()
        ### Step 3 : Line-search to find new control sequence, trajectory, cost
        ### Step 4 : Accept Step (or not) and print status
        pass
    # return

    def backward_pass(self, cx, cu, cxx, cxu, cuu, fx, fu, lamb, \
                      ctrl_lims, u):
        """
        Execute backward pass along trajectory
        Args:
            cx, cu, cxx, cxu, cuu, fx, fu
            lamb: Final value of the regularization parameter
            reg_type: Regularization type (q_uu+lamb*np.eye)
        Returns:


        """
        k = np.zeros([self.agent.nu, self.params.iterations])
        K = np.zeros([self.agent.nu, self.agent.nx, self.params.iterations])
        Vx = np.zeros([self.agent.nx, self.params.iterations+1])
        Vxx = np.zeros([self.agent.nx, self.agent.nx, self.params.iterations+1])
        dV = np.array([0, 0])
        Vx[:,-1] = cx[:, -1]
        Vxx[:,:,-1] = cxx[:,:,-1]
        for i in np.arange(self.params.iterations-1, -1, -1):
            Qu = cu[:,i] + np.dot(fu[:,:,i].T, Vx[:,i+1])
            Qx = cx[:,i] + np.dot(fx[:,:,i].T, Vx[:,i+1])
            Qux = cxu[:,:,i].T + np.dot(np.dot(fu[:,:,i].T, Vxx[:,:,i+1]), fx[:,:,i])
            Quu = cuu[:,:,i] + np.dot(np.dot(fu[:,:,i].T, Vxx[:,:,i+1]), fu[:,:,i])
            Qxx = cxx[:,:,i] + np.dot(np.dot(fx[:,:,i].T, Vxx[:,:,i+1]), fx[:,:,i])
            Vxx_reg = Vxx[:,:,i+1]
            Qux_reg = cxu[:,:,i].T + np.dot(np.dot(fu[:,:,i].T, Vxx_reg), fx[:,:,i])
            QuuF = cuu[:,:,i] + np.dot(np.dot(fu[:,:,i].T, Vxx_reg), fu[:,:,i]) + \
                    lamb * np.eye(self.agent.nu)
            if np.isnan(ctrl_lims).any():
                L = np.linalg.cholesky(QuuF)
                kK = -np.dot(np.linalg.inv(L), np.dot(np.linalg.inv(L.T), np.vstack([Qu, Qux_reg.T]).T))
                k_i = kK[:,0]
                K_i = kK[:,1:]
            else:
                lower = ctrl_lims[:,0] - u[:,i]
                upper = ctrl_lims[:,1] - u[:,i]
                k_i, result, R, free = boxQP(QuuF, Qu, lower, upper, k[:,min(i+1, self.params.iterations-1)])
                K_i = np.zeros([self.agent.nu, self.agent.nx])
                if free.any():
                    Lfree = np.dot(np.linalg.inv(-R), (np.dot(np.linalg.inv(R.T), Qux_reg[free])))
                    K_i[free,:] = Lfree
            dV = dV + np.array([np.dot(k_i, Qu), 0.5*np.dot(np.dot(k_i, Quu), k_i)])
            Vx[:,i] = Qx + np.dot(np.dot(K_i.T, Quu), k_i) + np.dot(K_i.T, Qu) + \
                    np.dot(Qux.T, k_i)
            Vxx[:,:,i] = Qxx + np.dot(np.dot(K_i.T, Quu), K_i) + np.dot(K_i.T, Qux) + \
                    np.dot(Qux.T, K_i)
            Vxx[:,:,i] = 0.5*(Vxx[:,:,i] + Vxx[:,:,i].T)
            k[:,i] = k_i
            K[:,:,i] = K_i
            # if i==300:
                # print(k_i)
                # print(K_i)
                # import ipdb
                # ipdb.set_trace()
                # exit()

        return diverge, Vx, Vxx, k, K, dV

    def forward_pass(self, x0, policy, noisy=False):
        """
        Roll out through dynamics given x0 and poliy
        Args:
            x0: Initial state
            policy: Policy fn or input array
            noisy: Add Gaussian noise
        Returns:
            traj: Return trajectory list composed of [state, input, cost]
        """
        traj = []
        state = np.zeros([self.agent.nx, self.params.iterations+1])
        state[:, 0] = x0
        cost = np.zeros(self.params.iterations+1)
        if isinstance(policy, np.ndarray):
            for i in range(self.params.num_samples):
                if noisy:
                    policy = generate_noise(policy)
                u = np.zeros([self.agent.nu, self.params.iterations+1])
                u[:, -1] = np.nan
                u[:, :-1] = policy
                for alpha in self.alpha:
                    for t in range(self.params.iterations):
                        state[:, t+1], cost[t], _ = self.agent.step(state[:, t], u[:, t])
                    _, cost[-1], _ = self.agent.step(state[:,-1], u[:,-1])
                    if not isDiverge(state, self.agent.thres):
                        break
                    if alpha == alpha[-1]:
                        raise ValueError("Trajectory is Diverged")
                traj.append({'state_list':state, 'input_list': u, 'cost_list':cost})
        else:
            pass
        return traj

    def fn_J_dyn(self, xu):
        """
        x(n+1) = f(x), where x = [state, input]'
        Compute f_x
        Args:
            xu: State and input augmented vector
        Returns:
            j_dyn: (nx, nx+nu) jacobian

        """
        j_dyn = (finite_differences(self.agent.dynAug, \
                xu, (self.agent.nx, ))).T
        return j_dyn

    def fn_J_cst(self, xu):
        """
        c(n) = cst(x(n)), where x = [state, input]'
        Compute cst_x
        Args:
            xu: State and input augmented vector
        Returns:
            j_cst: (nx+nu) gradient

        """

        j_cst = (finite_differences(self.agent.costAug, \
                xu )).T
        return j_cst

    def dynCstDiff(self, traj):
        """
        Differentiate dynamics and cost along trajectory
        Args:
            traj: dict composed of 'state_list', 'input_list', 'cost_list'
        Returns:
            fx, fu, fxx, fxu, fuu, cx, cu, cxx, cxu, cuu

        """
        J_dyn = np.zeros([self.agent.nx,
                          self.agent.nx+self.agent.nu,
                          self.params.iterations+1])
        J_dyn[:,-2:,-1] = np.nan
        J_cst = np.zeros([self.agent.nx+self.agent.nu,
                          self.params.iterations+1])
        J_cstcst = np.zeros([self.agent.nx+self.agent.nu,
                             self.agent.nx+self.agent.nu,
                             self.params.iterations+1])

        for t in range(self.params.iterations+1):
              J_dyn[:,:,t] = self.fn_J_dyn(
                      np.hstack([traj['state_list'][:,t],
                          traj['input_list'][:,t]]))
              J_cst[:,t] = self.fn_J_cst(
                      np.hstack([traj['state_list'][:,t],
                          traj['input_list'][:,t]]))
              J_cstcst[:,:,t] = (finite_differences(self.fn_J_cst, \
                      np.hstack([traj['state_list'][:,t],
                          traj['input_list'][:,t]]),
                      (self.agent.nx+self.agent.nu, ))).T

        fx = J_dyn[:, 0:4, :]
        fu = J_dyn[:, -2:, :]
        cx = J_cst[0:4, :]
        cu = J_cst[-2:, :]
        cxx = J_cstcst[0:4, 0:4, :]
        cxu = J_cstcst[0:4, 4:6, :]
        cuu = J_cstcst[4:6, 4:6, :]

        return fx, fu, cx, cu, cxx, cxu, cuu

