import torch
import numpy as np

from scipy.linalg   import solve_continuous_are
from gbp.core       import FactorGraph, GBPSettings, MeasModel, SquaredLoss
from trajectory     import generate_line_trajectory


class LQRController:
    """
    A class to implement an LQR controller for trajectory tracking.
    """

    def __init__(self, 
                 A          : np.ndarray, 
                 B          : np.ndarray, 
                 Q          : np.ndarray,
                 R          : np.ndarray,
                 ref_traj   : np.ndarray,
                 x0         : np.ndarray,
                 dt         : float,
                 horizon    : int):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.ref_traj = ref_traj
        self.x0 = x0
        self.dt = dt
        self.horizon = horizon

    def control(self):
        """
        Computes the control inputs and state trajectory using LQR.
        """
        # ----------------------------------------
        # LQR gain computation
        # ----------------------------------------
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P

        # ----------------------------------------
        # Closed-loop simulation
        # ----------------------------------------
        x = np.zeros((self.horizon + 1, self.A.shape[0]))
        x[0] = self.x0
        u = np.zeros((self.horizon, self.B.shape[1]))

        for k in range(self.horizon):
            e = x[k] - self.ref_traj[k]
            u[k] = -K @ e
            x[k + 1] = x[k] + self.dt * (self.A @ x[k] + self.B @ u[k])

        return x, u

class GBPController:
    def __init__(self, 
                 A: np.ndarray,
                 B: np.ndarray,
                 horizon: int,
                 dt: float,
                 sigma_collision: float,
                 robot_radius: float,
                 safety_eps: float,
                 settings: GBPSettings):
        self.A = A
        self.B = B
        self.A_torch = torch.from_numpy(self.A).float()
        self.B_torch = torch.from_numpy(self.B).float()
        self.dt = dt
        self.horizon = horizon
        self.sigma_collision = sigma_collision
        self.robot_radius = robot_radius
        self.safety_eps = safety_eps
        self.settings = settings

        self.fg = FactorGraph(settings)
        self.x_ids_list = []
        self.u_ids_list = []
        self.y_ids_list = []
        self.r_ids_list = []
        self.r_star = 2 * self.robot_radius + self.safety_eps
        self.x0_list = []

    def add_agent(self,
                  Q                 : np.ndarray,
                  R                 : np.ndarray,
                  S                 : np.ndarray,
                  ref_traj          : np.ndarray,
                  x0                : np.ndarray,
                  sigma_0           : float,
                  sigma_pos         : float,
                  sigma_vel         : float,
                  sigma_u           : float,
                  sigma_dynamics    : float):
        """
        Constructs a GBP controller for a single agent.
        """
        Q_inv_diag = torch.tensor(np.diag(np.linalg.inv(Q)), dtype=torch.float)
        R_inv = np.linalg.inv(R)
        R_inv_diag = torch.tensor(np.diag(R_inv), dtype=torch.float)
        S_inv_diag = torch.tensor(np.diag(np.linalg.inv(S)), dtype=torch.float)

        state_cov = torch.tensor([sigma_pos, sigma_pos, sigma_vel, sigma_vel], dtype=torch.float)

        # ----------------------------------------
        # GBP variables
        # ----------------------------------------
        x_ids, u_ids = [], []
        for t in range(self.horizon + 1):
            # State node with different sigmas for position and velocity
            if t == 0:
                self.fg.add_var_node(dofs=self.A.shape[0],
                                     prior_mean=torch.from_numpy(x0).float(),
                                     prior_diag_cov=torch.full((self.A.shape[0],), sigma_0))
            else:
                # Create diagonal covariance with different values for pos and vel
                self.fg.add_var_node(dofs=self.A.shape[0],
                                     prior_mean=torch.from_numpy(ref_traj[t]).float(),
                                     prior_diag_cov=state_cov)
            x_ids.append(len(self.fg.var_nodes) - 1)
        for t in range(self.horizon):
            # Control input node
            self.fg.add_var_node(dofs=self.B.shape[1],
                                 prior_mean=torch.zeros(self.B.shape[1]),
                                 prior_diag_cov=torch.full((self.B.shape[1],), sigma_u))
            u_ids.append(len(self.fg.var_nodes) - 1)

        # ----------------------------------------
        # Precision Matrices
        # ----------------------------------------
        dynamics_prec = torch.full((self.A.shape[0],), sigma_dynamics)

        # ----------------------------------------
        # Factors
        # ----------------------------------------
        for t in range(self.horizon):
            # Dynamics factor
            self.fg.add_factor(adj_var_ids=[x_ids[t], u_ids[t], x_ids[t + 1]],
                               measurement=torch.zeros(self.A.shape[0]),
                               meas_model=MeasModel(self._dynamics_meas, self._dynamics_jac, SquaredLoss(self.A.shape[0], dynamics_prec)))
            # Tracking reference factor
            self.fg.add_factor(adj_var_ids=[x_ids[t]],
                               measurement=torch.from_numpy(ref_traj[t]).float(),
                               meas_model=MeasModel(self._id_meas, self._id_jac, SquaredLoss(self.A.shape[0], Q_inv_diag)))
            # Control effort factor
            self.fg.add_factor(adj_var_ids=[u_ids[t]],
                               measurement=torch.zeros(self.B.shape[1]),
                               meas_model=MeasModel(self._id_meas, self._id_jac, SquaredLoss(self.B.shape[1], R_inv_diag)))
        # Terminal tracking factor on final state
        self.fg.add_factor(adj_var_ids=[x_ids[self.horizon]],
                           measurement=torch.from_numpy(ref_traj[self.horizon]).float(),
                           meas_model=MeasModel(self._id_meas, self._id_jac, SquaredLoss(self.A.shape[0], S_inv_diag)))
        
        # -----------------------------------------
        # Add variable IDs to lists
        # -----------------------------------------
        self.x_ids_list.append(x_ids)
        self.u_ids_list.append(u_ids)

    def add_agent_with_sensing(self,
                  Q                 : np.ndarray,
                  R                 : np.ndarray,
                  S                 : np.ndarray,
                  ref_traj          : np.ndarray,
                  x0                : np.ndarray,
                  ts                : np.ndarray,
                  sigma_0           : float,
                  sigma_pos         : float,
                  sigma_vel         : float,
                  sigma_u           : float,
                  sigma_sensor      : float,
                  sigma_dynamics    : float,
                  sigma_meas        : float):
        """
        Constructs a GBP controller for a single agent.
        """
        Q_inv_diag = torch.tensor(np.diag(np.linalg.inv(Q)), dtype=torch.float)
        R_inv = np.linalg.inv(R)
        R_inv_diag = torch.tensor(np.diag(R_inv), dtype=torch.float)
        S_inv_diag = torch.tensor(np.diag(np.linalg.inv(S)), dtype=torch.float)

        state_cov = torch.tensor([sigma_pos, sigma_pos, sigma_vel, sigma_vel], dtype=torch.float)

        r_init = generate_line_trajectory(start_pos=x0[:2],
                                          end_pos=ref_traj[0][:2],
                                          ts=ts)
        # ----------------------------------------
        # GBP variables
        # ----------------------------------------
        x_ids, u_ids, y_ids, r_ids = [], [], [], []
        for t in range(self.horizon + 1):
             # ─── y-t measurement node (unchanged) ───────────────────────────────────────
            self.fg.add_var_node(dofs=self.A.shape[0],
                                 prior_mean=torch.from_numpy(r_init[t]).float(),
                                 prior_diag_cov=torch.full((self.A.shape[0],), sigma_sensor))
            y_ids.append(len(self.fg.var_nodes) - 1)
             # ─── r-t reference node  (⇐ **NEW special-case handling**) ──────────────────
            if t == 0:  # start of the path  → lock to x0
                self.fg.add_var_node(
                    dofs=self.A.shape[0],
                    prior_mean=torch.from_numpy(x0).float(),
                    prior_diag_cov=torch.full((self.A.shape[0],), sigma_0)
                )
            elif t == self.horizon:  # end of the path    → lock to goal (= last ref_traj row)
                self.fg.add_var_node(
                    dofs=self.A.shape[0],
                    prior_mean=torch.from_numpy(ref_traj[-1]).float(),
                    prior_diag_cov=torch.full((self.A.shape[0],), sigma_0)
                )
            else:  # intermediate r-t  → loose prior so GBP can move it
                self.fg.add_var_node(
                    dofs=self.A.shape[0],
                    prior_mean=torch.from_numpy(r_init[t]).float(),
                    prior_diag_cov=state_cov
                )
            r_ids.append(len(self.fg.var_nodes) - 1)
            # ─── x-t executed-state node  ───────────────────────────────────────────────
            if t == 0:
                self.fg.add_var_node(dofs=self.A.shape[0],
                                     prior_mean=torch.from_numpy(x0).float(),
                                     prior_diag_cov=torch.full((self.A.shape[0],), sigma_0))
            else:
                # Create diagonal covariance with different values for pos and vel
                self.fg.add_var_node(dofs=self.A.shape[0],
                                     prior_mean=torch.from_numpy(ref_traj[t]).float(),
                                     prior_diag_cov=state_cov)
            x_ids.append(len(self.fg.var_nodes) - 1)
        # ─── u-t control input node  ───────────────────────────────────────────────── 
        for t in range(self.horizon):
            # Control input node
            self.fg.add_var_node(dofs=self.B.shape[1],
                                 prior_mean=torch.zeros(self.B.shape[1]),
                                 prior_diag_cov=torch.full((self.B.shape[1],), sigma_u))
            u_ids.append(len(self.fg.var_nodes) - 1)

        # ----------------------------------------
        # Precision Matrices
        # ----------------------------------------
        dynamics_prec = torch.full((self.A.shape[0],), sigma_dynamics)
        sensor_prec = torch.full((self.A.shape[0],), sigma_sensor)

        # ----------------------------------------
        # Factors
        # ----------------------------------------
        for t in range(self.horizon):
            # Dynamics factor
            self.fg.add_factor(adj_var_ids=[x_ids[t], u_ids[t], x_ids[t + 1]],
                               measurement=torch.zeros(self.A.shape[0]),
                               meas_model=MeasModel(self._dynamics_meas, self._dynamics_jac, SquaredLoss(self.A.shape[0], dynamics_prec)))
            # Tracking factor between executed state and reference
            self.fg.add_factor(adj_var_ids=[r_ids[t], x_ids[t]],
                               measurement=torch.zeros(self.A.shape[0]),
                               meas_model=MeasModel(self._measurement_meas, self._measurement_jac, SquaredLoss(self.A.shape[0], Q_inv_diag)))
            # Control effort factor
            self.fg.add_factor(adj_var_ids=[u_ids[t]],
                               measurement=torch.zeros(self.B.shape[1]),
                               meas_model=MeasModel(self._id_meas, self._id_jac, SquaredLoss(self.B.shape[1], R_inv_diag)))
            # Sensor measurement factor
            self.fg.add_factor(adj_var_ids=[y_ids[t], x_ids[t]],
                        measurement=torch.zeros(self.A.shape[0]),
                        meas_model=MeasModel(self._measurement_meas, self._measurement_jac, SquaredLoss(self.A.shape[0], sensor_prec)))
        # Terminal tracking factor on final state
        self.fg.add_factor(adj_var_ids=[x_ids[self.horizon]],
                           measurement=torch.from_numpy(ref_traj[self.horizon]).float(),
                           meas_model=MeasModel(self._id_meas, self._id_jac, SquaredLoss(self.A.shape[0], S_inv_diag)))

        self.fg.add_factor(adj_var_ids=[r_ids[self.horizon], x_ids[self.horizon]],
                    measurement=torch.zeros(self.A.shape[0]),
                    meas_model=MeasModel(self._measurement_meas, self._measurement_jac,SquaredLoss(self.A.shape[0], dynamics_prec)))

        # -----------------------------------------
        # Add variable IDs to lists
        # -----------------------------------------
        self.x0_list.append(x0)
        self.x_ids_list.append(x_ids)
        self.u_ids_list.append(u_ids)
        self.y_ids_list.append(y_ids)
        self.r_ids_list.append(r_ids)
    
    def add_inter_agent_collision(self):
        """
        Adds inter-agent collision factors to the factor graph.
        """
        # -----------------------------------------
        # Precision Matrices
        # -----------------------------------------
        inter_prec = torch.full((1,), self.sigma_collision)

        # -----------------------------------------
        # Add collision factors between all pairs of agents
        # -----------------------------------------
        for t in range(self.horizon + 1):
            for i in range(len(self.x_ids_list)):
                for j in range(i + 1, len(self.x_ids_list)):
                    self.fg.add_factor(adj_var_ids=[self.x_ids_list[i][t], self.x_ids_list[j][t]],
                                       measurement=torch.zeros(1),
                                       meas_model=MeasModel(self._inter_meas, self._inter_jac, SquaredLoss(1, inter_prec)))
                    
    def solve(self, n_iters=100, converged_threshold=1e-3):
        """
        Solves the factor graph using GBP.
        """
        self.fg.gbp_solve(n_iters=n_iters, converged_threshold=converged_threshold)
        x_list = [
            torch.stack([self.fg.var_nodes[i].belief.mean() for i in ids]).numpy()
            for ids in self.x_ids_list
        ]
        u_list = [
            torch.stack([self.fg.var_nodes[i].belief.mean() for i in ids]).numpy()
            for ids in self.u_ids_list
        ]
        return x_list, u_list
    
    def solve_with_sensing(self, sigma_meas=0.0):
        """
        Solves the factor graph using GBP.
        """
        true_x = [torch.from_numpy(x0.copy()).float() for x0 in self.x0_list]
        vec_meas = torch.full((self.A.shape[0],), sigma_meas, dtype=torch.float)
        measurement_prec = vec_meas ** 2

        self.fg.gbp_solve(n_iters=10)  # cold start

        for k in range(self.horizon):  # main loop
            for r in range(len(self.x0_list)):
                # 1) read current control
                u_cmd = self.fg.var_nodes[self.u_ids_list[r][k]].belief.mean()
                # 2) propagate real plant
                true_x[r] = true_x[r] + self.dt * (self.A_torch @ true_x[r] + self.B_torch @ u_cmd)
                # 3) noisy sensor reading
                z = true_x[r] + torch.normal(torch.zeros_like(true_x[r]), measurement_prec)
                self.fg.var_nodes[self.y_ids_list[r][k + 1]].set_prior(mean=z, diag_cov=measurement_prec)
            # 4) a few GBP sweeps to absorb evidence
            self.fg.gbp_solve(n_iters=2)

        # solve joint graph
        self.fg.gbp_solve(n_iters=10)
        # extract trajectories
        x_list = [
            torch.stack([self.fg.var_nodes[i].belief.mean() for i in ids]).numpy()
            for ids in self.x_ids_list
        ]
        u_list = [
            torch.stack([self.fg.var_nodes[i].belief.mean() for i in ids]).numpy()
            for ids in self.u_ids_list
        ]
        return x_list, u_list

    def _id_meas(self, z):
        return z
    
    def _id_jac(self, z):
        return torch.eye(len(z))
    
    def _dynamics_meas(self, z):
         # dimensions
        n, m = self.A_torch.shape[0], self.B_torch.shape[1]
        # slice out x_t, u_t, x_{t+1}
        x_t   = z[:n]               # shape (n,)
        u_t   = z[n:n + m]          # shape (m,)
        x_tp1 = z[n + m:2*n + m]    # shape (n,)
        # residual r = x_{t+1} - (A x_t + B u_t)
        return x_tp1 - (x_t + self.dt*(self.A_torch @ x_t + self.B_torch @ u_t))
    
    def _dynamics_jac(self, z):
        n, m = self.A_torch.shape[0], self.B_torch.shape[1]
        I_n = torch.eye(n)
        A_d = self.A_torch
        B_d = self.B_torch
        J = torch.zeros(n, 2*n + m)
        # ∂r/∂x_t = -I - dt·A
        J[:, 0:n] = -I_n - self.dt*A_d
        # ∂r/∂u_t = -dt·B
        J[:, n:n+m] = -self.dt*B_d
        # ∂r/∂x_{t+1} =  I
        J[:, n+m:]  =  I_n
        return J
    
    def _inter_meas(self,z):
        n = self.A_torch.shape[0]
        xA = z[:n]
        xB = z[n:]
        eps = 1e-6 # small epsilon for numerical stability
        diff = xA[:2] - xB[:2] + eps
        d = torch.norm(diff)
        return torch.clamp(1 - d / self.r_star, min=0.0).unsqueeze(0)
    
    def _inter_jac(self, z):
        # Analytical Jacobian of the truncated-linear collision cost
        n_ = z.shape[0] // 2
        eps = 1e-6
        # only consider positional dimensions (first 2 of each state)
        diff2 = z[:2] - z[n_:n_+2] + eps
        r = torch.norm(diff2)
        J = torch.zeros(1, 2*n_)
        if r <= self.r_star:
            grad2 = -diff2 / (self.r_star * r)
            J[0, 0:2] = grad2
            J[0, n_:n_+2] = -grad2
        return J

    def _measurement_meas(self, z):
        """z = [y_t | x_t]  ⇒  residual  r = y_t - x_t  (dim = n)"""
        n = self.A.shape[0]
        y = z[:n]
        x = z[n:]
        return y - x
    
    def _measurement_jac(self, z):
        n = self.A.shape[0]
        J = torch.zeros(n, 2 * n)
        J[:, :n] = torch.eye(n)  # ∂r/∂y = +I
        J[:, n:2 * n] = -torch.eye(n)  # ∂r/∂x = -I
        return J