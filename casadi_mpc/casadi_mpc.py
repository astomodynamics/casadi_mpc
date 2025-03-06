#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import numpy as np
import casadi as ca
import math

class CasadiMPCNode(Node):
    def __init__(self):
        super().__init__('casadi_mpc_node') 

        # Declare and get the robot_id parameter
        self.declare_parameter("robot_id", "j100_0857")
        self.robot_id = self.get_parameter("robot_id").value

        # MPC parameters
        self.horizon = 20       # MPC prediction horizon (number of control intervals)
        self.dt = 0.1           # Discrete time step [s]

        # States: [x, y, theta] and Controls: [v, omega]
        # Cost weights
        self.Q = ca.diag([0.1, 0.1, 0.0])
        self.R = ca.diag([0.01, 0.01])
        self.Qf = ca.diag([0.0, 0.0, 0.0])  # Terminal cost weight

        # Box constraints for state and inputs.
        self.x_min = -ca.inf
        self.x_max = ca.inf
        self.y_min = -ca.inf
        self.y_max = ca.inf
        self.theta_min = -ca.pi
        self.theta_max = ca.pi
        self.v_min = -1.0
        self.v_max = 1.0
        self.omega_min = -ca.pi
        self.omega_max = ca.pi

        # Current state (initialized to zeros)
        self.current_state = ca.DM.zeros(3)

        # Default goal state: [x, y, theta]
        self.goal_state = ca.DM([2.0, 5.0, math.pi/2])

        self.is_current_pose_received = False
        self.is_goal_pose_received = False # FIXME: if you want to use fixed goal, set this to True

        # Obstacle parameters:
        self.obstacle_centers = []
        # FIXME: if you want to add obstacles, add them here, i.e.
        # self.obstacle_centers = [(0.762, 2.54), (2.794, 3.429), (0.762, 4.318)]
        self.obstacle_radius = 0.41 * np.sqrt(2) / 2

        # Set up the MPC problem formulation using CasADi.
        self.setup_mpc()

        self.state_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_id}/pose',
            self.current_state_callback,
            10)
        
        self.path_sub = self.create_subscription(
            Path,
            f'/{self.robot_id}/global_path',
            self.path_callback,
            10)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_id}/goal_pose',
            self.goal_pose_callback,
            10)

        self.control_pub = self.create_publisher(
            Twist,
            f'/{self.robot_id}/cmd_vel',
            10)
        
        self.local_path_pub = self.create_publisher(
            Path,
            f'/{self.robot_id}/local_path',
            10)

        # Timer for continuous control (every 0.1 s)
        self.control_timer = self.create_timer(0.1, self.control_callback)

        self.get_logger().info(
            f'CasadiMPCNode for robot_id "{self.robot_id}" has been initialized with goal [2.0, 5.0, pi/2]'
        )

    def setup_mpc(self):
        # ---------------------------------------------------------------------
        # 1. Define Symbolic Variables for State and Control
        # ---------------------------------------------------------------------
        x = ca.SX.sym('x', 3)  # state: [x, y, theta]
        u = ca.SX.sym('u', 2)  # control: [v, omega]

        # Continuous dynamics:
        x_dot = ca.vertcat(u[0] * ca.cos(x[2]),
                           u[0] * ca.sin(x[2]),
                           u[1])
        # Discrete-time dynamics via Euler integration:
        x_next = x + self.dt * x_dot
        f = ca.Function('f', [x, u], [x_next])
        self.f = f

        # ---------------------------------------------------------------------
        # 2. Define Decision Variables and Parameters
        # ---------------------------------------------------------------------
        X = ca.SX.sym('X', 3, self.horizon + 1)  # states at time steps 0,...,horizon
        U = ca.SX.sym('U', 2, self.horizon)       # controls at time steps 0,...,horizon-1
        self.X = X
        self.U = U

        # Parameters: initial state P and reference (goal) state ref.
        P = ca.SX.sym('P', 3)
        ref = ca.SX.sym('ref', 3)
        self.P = P
        self.ref = ref

        # ---------------------------------------------------------------------
        # 3. Build the Cost Function
        # ---------------------------------------------------------------------
        cost = 0
        for k in range(self.horizon):
            cost += ca.mtimes((X[:, k] - ref).T, ca.mtimes(self.Q, (X[:, k] - ref))) \
                    + ca.mtimes(U[:, k].T, ca.mtimes(self.R, U[:, k]))
        cost += ca.mtimes((X[:, self.horizon] - ref).T, ca.mtimes(self.Qf, (X[:, self.horizon] - ref)))

        # ---------------------------------------------------------------------
        # 4. Build the Constraints
        # ---------------------------------------------------------------------
        g_eq = []
        # (a) Initial condition: X[:, 0] = P.
        g_eq.append(X[:, 0] - P)
        # (b) Dynamics constraints: for k = 0,...,horizon-1.
        for k in range(self.horizon):
            g_eq.append(X[:, k+1] - f(X[:, k], U[:, k]))
        g_eq = ca.vertcat(*g_eq)

        # (B) Obstacle avoidance constraints.
        g_obs_list = []
        for k in range(self.horizon + 1):
            for (obs_x, obs_y) in self.obstacle_centers:
                obs_constraint = (X[0, k] - obs_x)**2 + (X[1, k] - obs_y)**2 - self.obstacle_radius**2
                g_obs_list.append(obs_constraint)
        g_obs = ca.vertcat(*g_obs_list)

        # Combine constraints.
        g_total = ca.vertcat(g_eq, g_obs)
        self.g_total = g_total

        # ---------------------------------------------------------------------
        # 5. Formulate the NLP
        # ---------------------------------------------------------------------
        Z = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        self.Z = Z

        # Parameter vector: [P; ref]
        params = ca.vertcat(P, ref)
        nlp = {'x': Z, 'f': cost, 'g': g_total, 'p': params}

        opts = {
            'ipopt.print_level': 0,
            'ipopt.max_iter': 500,
            'ipopt.tol': 1e-6,
            'print_time': False
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # Save constraint dimensions for later use.
        self.n_eq = 3 + 3 * self.horizon
        self.n_obs = len(self.obstacle_centers) * (self.horizon + 1)

    def current_state_callback(self, msg):
        current_x = msg.pose.position.x 
        current_y = msg.pose.position.y
        _, _, current_yaw = self.euler_from_quaternion(msg.pose.orientation)
        self.current_state = ca.DM([current_x, current_y, current_yaw]) 
        self.is_current_pose_received = True
        self.get_logger().info(
            f'Received state: x={current_x:.2f}, y={current_y:.2f}, theta={current_yaw:.2f}'
        )

    def path_callback(self, msg):
        self.path = msg
        self.ref_path = ca.DM([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])

    def goal_pose_callback(self, msg):
        # Extract goal position and orientation.
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        _, _, goal_yaw = self.euler_from_quaternion(msg.pose.orientation)
        self.goal_state = ca.DM([goal_x, goal_y, goal_yaw])
        self.is_goal_pose_received = True
        self.get_logger().info(
            f'Updated goal state to: x={goal_x:.2f}, y={goal_y:.2f}, theta={goal_yaw:.2f}'
        )

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
    
    def control_callback(self):
        # Check if pose and goal are received:
        if not self.is_current_pose_received or not self.is_goal_pose_received:
            self.get_logger().info(f'Current or Goal pose not received')
            return

        # If the robot reaches the goal, reset is_goal_pose_received
        if np.linalg.norm(self.current_state[:2] - self.goal_state[:2]) < 0.05:
            # Return zero control
            control_msg = Twist()
            control_msg.linear.x = 0.0
            control_msg.angular.z = 0.0
            self.control_pub.publish(control_msg)

            self.is_goal_pose_received = False
            self.get_logger().info(f'Reached goal. Goal pose reset')
            return
        
        # 1. Create an Initial Guess for the Decision Variables.
        x0_val = np.array(self.current_state.full().flatten())
        goal_val = np.array(self.goal_state.full().flatten())
        X0 = np.zeros((3, self.horizon + 1))
        for i in range(3):
            X0[i, :] = np.linspace(x0_val[i], goal_val[i], self.horizon + 1)
        U0 = np.zeros((2, self.horizon))
        Z0 = np.concatenate((X0.reshape(-1, order='F'), U0.reshape(-1, order='F')))

        # 2. Build Variable Bounds.
        lbx_states = []
        ubx_states = []
        for _ in range(self.horizon + 1):
            lbx_states.extend([self.x_min, self.y_min, self.theta_min])
            ubx_states.extend([self.x_max, self.y_max, self.theta_max])
        lbx_controls = []
        ubx_controls = []
        for _ in range(self.horizon):
            lbx_controls.extend([self.v_min, self.omega_min])
            ubx_controls.extend([self.v_max, self.omega_max])
        lbx = lbx_states + lbx_controls
        ubx = ubx_states + ubx_controls

        # 3. Build Constraint Bounds.
        lbg_eq = [0.0] * self.n_eq
        ubg_eq = [0.0] * self.n_eq
        lbg_obs = [0.0] * self.n_obs
        ubg_obs = [1e20] * self.n_obs
        lbg_total = lbg_eq + lbg_obs
        ubg_total = ubg_eq + ubg_obs

        # 4. Solve the NLP.
        p_val = np.concatenate((x0_val, goal_val))
        sol = self.solver(
            x0=Z0,
            lbx=lbx,
            ubx=ubx,
            lbg=lbg_total,
            ubg=ubg_total,
            p=p_val
        )
        Z_opt = sol['x'].full().flatten()

        # Extract the state and control trajectories.
        n_states = 3 * (self.horizon + 1)
        X_opt = Z_opt[:n_states].reshape((3, self.horizon+1), order='F')
        U_opt = Z_opt[n_states:].reshape((2, self.horizon), order='F')

        # 5. Publish the First Control Input.
        control_msg = Twist()
        control_msg.linear.x = float(U_opt[0, 0])
        control_msg.angular.z = float(U_opt[1, 0])
        self.control_pub.publish(control_msg)
        self.get_logger().info(
            f'Published velocities: linear={U_opt[0, 0]:.2f}, angular={U_opt[1, 0]:.2f}'
        )

        # 6. Publish the Predicted Local Path.
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"  # Adjust frame as needed
        for i in range(self.horizon + 1):
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = X_opt[0, i]
            pose_stamped.pose.position.y = X_opt[1, i]
            pose_stamped.pose.position.z = 0.0
            theta = X_opt[2, i]
            # Convert yaw to quaternion (assuming roll=pitch=0)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = math.sin(theta/2.0)
            pose_stamped.pose.orientation.w = math.cos(theta/2.0)
            path_msg.poses.append(pose_stamped)
        self.local_path_pub.publish(path_msg)
        self.get_logger().info('Published local predicted path.')

def main(args=None):
    rclpy.init(args=args)
    node = CasadiMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
