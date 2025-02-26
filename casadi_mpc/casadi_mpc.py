import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import numpy as np
import casadi as ca

class CasadiMPCNode(Node):
    def __init__(self):
        super().__init__('casadi_mpc_node') 

        # Declare and get the robot_id parameter
        self.declare_parameter("robot_id", "j100_0857")
        self.robot_id = self.get_parameter("robot_id").value

        # MPC parameters
        self.horizon = 20
        self.dt = 0.1

        # State: [x, y, theta]
        # Input: [v, omega]

        # Weights for the cost function 
        self.Q = ca.diag([0.01, 0.01, 0.0])
        self.R = ca.diag([0.1, 0.1])
        self.Qf = ca.diag([1, 1, 0.0])

        # Box constraints for state and inputs
        self.x_min = 0
        self.x_max = 10
        self.y_min = 0
        self.y_max = 10
        self.theta_min = -ca.pi
        self.theta_max = ca.pi
        self.v_min = -1
        self.v_max = 1
        self.omega_min = -1
        self.omega_max = 1

        # current state and goal state (3x1 DM vectors)
        self.current_state = ca.DM.zeros(3)
        self.goal_state = ca.DM.zeros(3)

        self.is_current_pose_received = False
        self.is_goal_pose_received = False

        # Define obstacle parameters:
        # Each obstacle is originally a square of side 0.41.
        # We use the circumscribed circle with radius = (0.41*sqrt(2))/2.
        self.obstacle_centers = [(0.762, 2.54), (2.794, 3.429), (0.762, 4.318)]
        self.obstacle_radius = 0.41 * np.sqrt(2) / 2

        # Setup optimization problem
        self.setup_mpc()

        # ROS2 publishers and subscribers using robot_id in topic names
        self.state_sub = self.create_subscription(
            PoseStamped,
            # f'/{self.robot_id}/pose',
            f'/zed/zed_node/pose',
            self.current_state_callback,
            10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # goal can remain on a global topic if desired
            self.goal_state_callback,
            10)
        
        self.path_sub = self.create_subscription(
            Path,
            f'/{self.robot_id}/path',
            self.path_callback,
            10)

        self.control_pub = self.create_publisher(
            Twist,
            f'/{self.robot_id}/cmd_vel',
            10)
        
        # Timer for continuous control in seconds
        self.control_timer = self.create_timer(0.1, self.control_callback)

        self.get_logger().info(f'CasadiMPCNode for robot_id "{self.robot_id}" has been initialized')

    def setup_mpc(self):
        # CasADi symbols for state and input
        self.x = ca.SX.sym('x', 3)
        self.u = ca.SX.sym('u', 2)

        # Differential drive kinematics
        x_dot = self.u[0] * ca.cos(self.x[2])
        y_dot = self.u[0] * ca.sin(self.x[2])
        theta_dot = self.u[1]
        x_next = self.x + self.dt * ca.vertcat(x_dot, y_dot, theta_dot)
        self.f = ca.Function('f', [self.x, self.u], [x_next])

        # Decision variables over the horizon:
        # States: a 3 x (N+1) matrix, Inputs: a 2 x N matrix.
        self.opt_x = ca.SX.sym('opt_x', 3, self.horizon + 1)
        self.opt_u = ca.SX.sym('opt_u', 2, self.horizon)

        # Parameters: initial state and reference (goal) state
        self.p = ca.SX.sym('p', 3)   # initial state
        self.ref = ca.SX.sym('ref', 3)  # goal state

        # Cost function
        obj = 0
        for k in range(self.horizon):
            state_error = self.opt_x[:, k] - self.ref
            obj += ca.mtimes([state_error.T, self.Q, state_error]) + \
                   ca.mtimes([self.opt_u[:, k].T, self.R, self.opt_u[:, k]])
        # Terminal cost
        state_error = self.opt_x[:, self.horizon] - self.ref
        obj += ca.mtimes([state_error.T, self.Qf, state_error])
        
        # Constraints list
        g = []

        # Dynamics constraints: for each k, enforce x_{k+1} = f(x_k, u_k)
        for k in range(self.horizon):
            g.append(self.opt_x[:, k+1] - self.f(self.opt_x[:, k], self.opt_u[:, k]))
        
        # Initial condition constraint
        g.append(self.opt_x[:, 0] - self.p)

        # Obstacle avoidance constraints:
        # For every state along the horizon, ensure that the robot stays outside each obstacle.
        # For each obstacle i and each time step k:
        #     (x_k - x_obs_i)^2 + (y_k - y_obs_i)^2 - (obstacle_radius)^2 >= 0.
        for k in range(self.horizon + 1):
            xk = self.opt_x[0, k]
            yk = self.opt_x[1, k]
            for (obs_x, obs_y) in self.obstacle_centers:
                g.append((xk - obs_x)**2 + (yk - obs_y)**2 - self.obstacle_radius**2)

        # Create the NLP problem dictionary.
        # The decision variable vector is the vertical concatenation of opt_x and opt_u.
        nlp = {'x': ca.vertcat(ca.reshape(self.opt_x, -1, 1), 
                               ca.reshape(self.opt_u, -1, 1)),
               'f': obj,
               'g': ca.vertcat(*g),
               'p': ca.vertcat(self.p, self.ref)}

        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

        # Save the number of equality and inequality constraints for later use.
        # Dynamics constraints: 3 per time step for horizon steps.
        # Initial condition: 3 constraints.
        self.eq_constr = 3 * self.horizon + 3
        # Obstacle constraints: one per obstacle per time step.
        self.ineq_constr = len(self.obstacle_centers) * (self.horizon + 1)
    
    def current_state_callback(self, msg):
        # Extract current state (assuming PoseStamped with one pose)
        current_x = msg.pose.position.x 
        current_y = msg.pose.position.y
        _, _, current_yaw = self.euler_from_quaternion(msg.pose.orientation)

        self.current_state = ca.DM([current_x, current_y, current_yaw]) 
        self.is_current_pose_received = True
        self.get_logger().info(f'Received state update: x={current_x:.2f}, y={current_y:.2f}, theta={current_yaw:.2f}')

    def goal_state_callback(self, msg):
        # Extract goal state (assuming PoseStamped with one pose)
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        _, _, goal_yaw = self.euler_from_quaternion(msg.pose.orientation)

        self.goal_state = ca.DM([goal_x, goal_y, goal_yaw])
        self.is_goal_pose_received = True
        self.get_logger().info(f'Received goal update: x={goal_x:.2f}, y={goal_y:.2f}, theta={goal_yaw:.2f}')

    def path_callback(self, msg):
        self.path = msg
        # Extract the 2D path as a DM (if needed)
        self.ref_path = ca.DM([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])

    def euler_from_quaternion(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
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
        if not self.is_pose_received or not self.is_goal_pose_received:
            self.get_logger().info(f'Current or Goal pose not received')
            return

        # If the robot reaches the goal, reset is_goal_received
        if np.linalg.norm(self.current_state[:2] - self.goal_state[:2]) < 0.1:
            self.is_goal_pose_received = False
            self.get_logger().info(f'Reached goal. Goal pose reset')
            return

        # Initialize guess trajectories for state and input over the horizon.
        x0 = np.zeros((3, self.horizon + 1))
        x0[:, 0] = np.array(self.current_state.full().flatten())
        u0 = np.zeros((2, self.horizon))

        # Construct the decision variable initial guess.
        init_guess = ca.vertcat(ca.reshape(x0, -1, 1), ca.reshape(u0, -1, 1))

        # Set bounds on decision variables.
        # States: [x, y, theta] over horizon+1 steps.
        lbx_states = [self.x_min, self.y_min, self.theta_min] * (self.horizon + 1)
        ubx_states = [self.x_max, self.y_max, self.theta_max] * (self.horizon + 1)
        # Inputs: [v, omega] over horizon steps.
        lbx_inputs = [self.v_min, self.omega_min] * self.horizon
        ubx_inputs = [self.v_max, self.omega_max] * self.horizon
        lbx = ca.vertcat(*(lbx_states + lbx_inputs))
        ubx = ca.vertcat(*(ubx_states + ubx_inputs))

        # Build the lower and upper bounds for constraints.
        # The first self.eq_constr constraints are equality (dynamics + initial condition): set to 0.
        lbg_eq = [0] * self.eq_constr
        ubg_eq = [0] * self.eq_constr
        # The remaining self.ineq_constr constraints are the obstacle avoidance inequalities:
        # They must be greater than or equal to 0.
        lbg_ineq = [0] * self.ineq_constr
        ubg_ineq = [ca.inf] * self.ineq_constr

        lbg_all = ca.vertcat(*(lbg_eq + lbg_ineq))
        ubg_all = ca.vertcat(*(ubg_eq + ubg_ineq))

        # Solve the MPC problem.
        res = self.solver(
            x0=init_guess,
            lbx=lbx,
            ubx=ubx,
            lbg=lbg_all,
            ubg=ubg_all,
            p=ca.vertcat(self.current_state, self.goal_state)
        )
        
        # Extract optimal control input (first control move).
        sol = res['x']
        # The control inputs are in the last 2*self.horizon entries.
        u_opt = np.array(sol[-2*self.horizon:]).reshape(2, self.horizon)
        
        # Publish control input (linear and angular velocities).
        control_msg = Twist()
        control_msg.linear.x = float(u_opt[0, 0])
        control_msg.angular.z = float(u_opt[1, 0])
        self.control_pub.publish(control_msg)
        
        self.get_logger().info(f'Published velocities: linear={u_opt[0, 0]:.2f}, angular={u_opt[1, 0]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CasadiMPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
