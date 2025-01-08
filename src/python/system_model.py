import sympy as sp
import numpy as np

class DronePayloadModel:
    def __init__(self, m_d=1, m_p=10, I_p=np.diag([1, 1, 1]), I_d=np.diag([0.05, 0.05, 0.05]), 
                 l=0.5, a=1, g=9.81):
        self.m_d = m_d  # Mass of drones
        self.m_p = m_p  # Mass of payload
        self.I_p = sp.Matrix(I_p)  # Inertia matrix of payload
        self.I_d = sp.Matrix(I_d)  # Inertia matrix of drones
        self.l = l  # Cable length
        self.a = a  # Rod length
        self.g = g  # Gravity

        self._define_symbols()
        self._define_lagrangian()
        self._lambdify_matrices()

    def _define_symbols(self):
        # === 1. Symbol Definitions ===

        # Time
        t = sp.symbols('t')

        # Payload position and orientation (quaternion)
        self.x_p, self.y_p, self.z_p = sp.symbols('x_p y_p z_p', cls=sp.Function)
        self.q_w, self.q_x, self.q_y, self.q_z = sp.symbols('q_w q_x q_y q_z', cls=sp.Function)

        # Drone orientations (quaternions)
        self.q_d1_w, self.q_d1_x, self.q_d1_y, self.q_d1_z = sp.symbols('q_d1_w q_d1_x q_d1_y q_d1_z', cls=sp.Function)
        self.q_d2_w, self.q_d2_x, self.q_d2_y, self.q_d2_z = sp.symbols('q_d2_w q_d2_x q_d2_y q_d2_z', cls=sp.Function)

        # Cable angles
        self.theta_c1, self.theta_c2 = sp.symbols('t_c1 t_c2', cls=sp.Function)
        self.phi_c1, self.phi_c2 = sp.symbols('phi_c1 phi_c2', cls=sp.Function)

        # Control inputs (thrust and torques)
        self.T_d1, self.T_d2 = sp.symbols('T_d1 T_d2', cls=sp.Function)
        self.tau_d1_x, self.tau_d1_y, self.tau_d1_z = sp.symbols('tau_d1_x tau_d1_y tau_d1_z', cls=sp.Function)
        self.tau_d2_x, self.tau_d2_y, self.tau_d2_z = sp.symbols('tau_d2_x tau_d2_y tau_d2_z', cls=sp.Function)

        # === 2. Dependencies on Time ===

        # Define time-dependent variables for payload
        self.x_p, self.y_p, self.z_p = self.x_p(t), self.y_p(t), self.z_p(t)
        self.q_w, self.q_x, self.q_y, self.q_z = self.q_w(t), self.q_x(t), self.q_y(t), self.q_z(t)
        self.q_p = sp.Matrix([self.q_w, self.q_x, self.q_y, self.q_z])
        

        # Time-dependent variables for drones
        self.q_d1_w, self.q_d1_x, self.q_d1_y, self.q_d1_z = self.q_d1_w(t), self.q_d1_x(t), self.q_d1_y(t), self.q_d1_z(t)
        self.q_d2_w, self.q_d2_x, self.q_d2_y, self.q_d2_z = self.q_d2_w(t), self.q_d2_x(t), self.q_d2_y(t), self.q_d2_z(t)

        # Time-dependent variables for cables
        self.theta_c1, self.theta_c2 = self.theta_c1(t), self.theta_c2(t)
        self.phi_c1, self.phi_c2 = self.phi_c1(t), self.phi_c2(t)

        # Control inputs are also time-dependent
        self.T_d1, self.T_d2 = self.T_d1(t), self.T_d2(t)
        self.tau_d1_x, self.tau_d1_y, self.tau_d1_z = self.tau_d1_x(t), self.tau_d1_y(t), self.tau_d1_z(t)
        self.tau_d2_x, self.tau_d2_y, self.tau_d2_z = self.tau_d2_x(t), self.tau_d2_y(t), self.tau_d2_z(t)

        # Vector of all state variables
        self.state_variables = sp.Matrix([
            self.x_p, self.y_p, self.z_p,
            self.q_w, self.q_x, self.q_y, self.q_z,
            self.q_d1_w, self.q_d1_x, self.q_d1_y, self.q_d1_z,
            self.theta_c1, self.phi_c1,
            self.q_d2_w, self.q_d2_x, self.q_d2_y, self.q_d2_z, 
            self.theta_c2, self.phi_c2
        ])

        # === 3. Derivatives ===

        # Derivatives for payload position
        self.x_p_dot, self.y_p_dot, self.z_p_dot = sp.diff(self.x_p, t), sp.diff(self.y_p, t), sp.diff(self.z_p, t)

        # Derivatives for payload quaternion
        self.q_w_dot, self.q_x_dot, self.q_y_dot, self.q_z_dot = sp.diff(self.q_w, t), sp.diff(self.q_x, t), sp.diff(self.q_y, t), sp.diff(self.q_z, t)
        self.q_p_dot = sp.Matrix([self.q_w_dot, self.q_x_dot, self.q_y_dot, self.q_z_dot])

        # Derivatives for drone quaternions
        self.q_d1_dot = sp.Matrix([sp.diff(self.q_d1_w, t), sp.diff(self.q_d1_x, t), sp.diff(self.q_d1_y, t), sp.diff(self.q_d1_z, t)])
        self.q_d2_dot = sp.Matrix([sp.diff(self.q_d2_w, t), sp.diff(self.q_d2_x, t), sp.diff(self.q_d2_y, t), sp.diff(self.q_d2_z, t)])

        # Derivatives for cable angles
        self.theta_c1_dot, self.phi_c1_dot = sp.diff(self.theta_c1, t), sp.diff(self.phi_c1, t)
        self.theta_c2_dot, self.phi_c2_dot = sp.diff(self.theta_c2, t), sp.diff(self.phi_c2, t)

        # Vector of all state derivatives
        self.state_derivatives = sp.Matrix([
            self.x_p_dot, self.y_p_dot, self.z_p_dot,
            self.q_w_dot, self.q_x_dot, self.q_y_dot, self.q_z_dot,
            self.q_d1_dot[0], self.q_d1_dot[1], self.q_d1_dot[2], self.q_d1_dot[3],
            self.theta_c1_dot, self.phi_c1_dot,
            self.q_d2_dot[0], self.q_d2_dot[1], self.q_d2_dot[2], self.q_d2_dot[3],
            self.theta_c2_dot, self.phi_c2_dot
        ])

        # === 4. Calculations ===

        # Payload position and velocity
        self.payload_pos = sp.Matrix([self.x_p, self.y_p, self.z_p])
        self.payload_vel = sp.Matrix([self.x_p_dot, self.y_p_dot, self.z_p_dot])

        # Payload orientation (quaternion) and angular velocity
        self.payload_orientation = sp.Matrix([self.q_w, self.q_x, self.q_y, self.q_z])
        self.payload_angular_vel = self.quaternion_angular_velocity(self.q_p, self.q_p_dot)

        # Define drone endpoints of the rod
        self.r1 = self.payload_pos + (self.a / 2) * self._rotate_vector([1, 0, 0], self.payload_orientation)
        self.r2 = self.payload_pos - (self.a / 2) * self._rotate_vector([1, 0, 0], self.payload_orientation)

        # Drone positions with respect to cable orientations
        self.drone1_pos = self.r1 + self.l * (sp.Matrix([sp.sin(self.theta_c1) * sp.cos(self.phi_c1), sp.sin(self.theta_c1) * sp.sin(self.phi_c1), sp.cos(self.theta_c1)]))
        self.drone2_pos = self.r2 + self.l * (sp.Matrix([sp.sin(self.theta_c2) * sp.cos(self.phi_c2), sp.sin(self.theta_c2) * sp.sin(self.phi_c2), sp.cos(self.theta_c2)]))

        # Drone orientations
        self.drone1_orientation = sp.Matrix([self.q_d1_w, self.q_d1_x, self.q_d1_y, self.q_d1_z])
        self.drone2_orientation = sp.Matrix([self.q_d2_w, self.q_d2_x, self.q_d2_y, self.q_d2_z])

        # Drone velocities
        self.drone1_vel = sp.diff(self.drone1_pos, t)
        self.drone2_vel = sp.diff(self.drone2_pos, t)

        # Drone angular velocities
        self.drone1_angular_vel = self.quaternion_angular_velocity(self.drone1_orientation, self.q_d1_dot)
        self.drone2_angular_vel = self.quaternion_angular_velocity(self.drone2_orientation, self.q_d2_dot)


    @staticmethod
    def _rotate_vector(vec, quaternion):
        """Rotate a vector using a quaternion."""
        vec = sp.Matrix(vec)
        q_w, q_x, q_y, q_z = quaternion
        R = sp.Matrix([
            [1 - 2 * (q_y**2 + q_z**2), 2 * (q_x * q_y - q_z * q_w), 2 * (q_x * q_z + q_y * q_w)],
            [2 * (q_x * q_y + q_z * q_w), 1 - 2 * (q_x**2 + q_z**2), 2 * (q_y * q_z - q_x * q_w)],
            [2 * (q_x * q_z - q_y * q_w), 2 * (q_y * q_z + q_x * q_w), 1 - 2 * (q_x**2 + q_y**2)]
        ])
        return R * vec

    @staticmethod
    def quaternion_angular_velocity(quaternion, quaternion_derivative):
        """
        Compute the angular velocity from the time derivative of a quaternion.
        
        Args:
            quaternion: sympy.Matrix([q_w, q_x, q_y, q_z])
            quaternion_derivative: sympy.Matrix([q_w_dot, q_x_dot, q_y_dot, q_z_dot])
        
        Returns:
            Angular velocity (sympy.Matrix([omega_x, omega_y, omega_z]))
        """
        
        # Calculate angular velocity components
        q_w, q_x, q_y, q_z = quaternion
        q_w_dot, q_x_dot, q_y_dot, q_z_dot = quaternion_derivative

        E = sp.Matrix([
            [-q_x, q_w, -q_z, q_y],
            [-q_y, q_z, q_w, -q_x],
            [-q_z, -q_y, q_x, q_w]
        ])
        omega = 2 * E * sp.Matrix([q_w_dot, q_x_dot, q_y_dot, q_z_dot])
        
        # Return angular velocity as a matrix
        return sp.Matrix([omega])
    
    def calculate_generalized_force(self, coordinate):
        """
        Calculate the generalized force for a given coordinate.

        Args:
            coordinate: The symbolic coordinate (e.g., x_p, q_d1_w).

        Returns:
            The generalized force corresponding to the coordinate.
        """
        # Rotate thrust to world frame
        thrust_d1 = self._rotate_vector([0, 0, self.T_d1], [self.q_d1_w, self.q_d1_x, self.q_d1_y, self.q_d1_z])
        thrust_d2 = self._rotate_vector([0, 0, self.T_d2], [self.q_d2_w, self.q_d2_x, self.q_d2_y, self.q_d2_z])

        # Forces acting on drone positions (thrust only)
        position_forces = [
            (thrust_d1, self.drone1_pos),  # Thrust affects position of Drone 1
            (thrust_d2, self.drone2_pos)   # Thrust affects position of Drone 2
        ]

        # Forces acting on drone quaternions (torques only)
        def compute_torque_force(quaternion, torque):
            q_w, q_x, q_y, q_z = quaternion
            G = sp.Matrix([
                [-q_x,  q_w,  q_z, -q_y],
                [-q_y, -q_z,  q_w,  q_x],
                [-q_z,  q_y, -q_x,  q_w]
            ])
            return 2 * G.T * sp.Matrix(torque)

        quaternion_forces = [
            (compute_torque_force([self.q_d1_w, self.q_d1_x, self.q_d1_y, self.q_d1_z], [self.tau_d1_x, self.tau_d1_y, self.tau_d1_z]),
            [self.q_d1_w, self.q_d1_x, self.q_d1_y, self.q_d1_z]),
            (compute_torque_force([self.q_d2_w, self.q_d2_x, self.q_d2_y, self.q_d2_z], [self.tau_d2_x, self.tau_d2_y, self.tau_d2_z]),
            [self.q_d2_w, self.q_d2_x, self.q_d2_y, self.q_d2_z])
        ]

        # Calculate contributions to generalized force from thrust (position)
        generalized_force_position = sum(
            F.dot(sp.Matrix([sp.diff(pos[i], coordinate) for i in range(3)]))
            for F, pos in position_forces
        )

        # Calculate contributions to generalized force from torques (quaternion)
        generalized_force_quaternion = sum(
            F.dot(sp.Matrix([sp.diff(quat, coordinate) for quat in quaternions]))
            for F, quaternions in quaternion_forces
        )

        # Combine contributions
        return generalized_force_position + generalized_force_quaternion



    def _define_lagrangian(self):
        # Kinetic Energy
        T_p = 0.5 * self.m_p * self.payload_vel.dot(self.payload_vel) + \
            0.5 * self.payload_angular_vel.dot(self.I_p * self.payload_angular_vel)

        T_d = 0.5 * self.m_d * (self.drone1_vel.dot(self.drone1_vel) + self.drone2_vel.dot(self.drone2_vel)) + \
            0.5 * (self.drone1_angular_vel.dot(self.I_d * self.drone1_angular_vel) + self.drone2_angular_vel.dot(self.I_d * self.drone2_angular_vel))

        # Potential Energy
        V_p = self.m_p * self.g * self.payload_pos[2]
        V_d = self.m_d * self.g * (self.drone1_pos[2] + self.drone2_pos[2])

        # Lagrangian
        self.L = T_p + T_d - (V_p + V_d)

        # Generalized forces for each degree of freedom
        self.generalized_forces = sp.Matrix([self.calculate_generalized_force(var) for var in self.state_variables])

        # Lagrange Equations
        self.equations = [
            sp.Eq(self.L.diff(derivative).diff('t') - self.L.diff(variable), force)
            for derivative, variable, force in zip(self.state_derivatives, self.state_variables, self.generalized_forces)
        ]


        # Debug output
        # print("Equations for Lagrange system:", self.equations)
        # print("Types of equations:", [type(eq) for eq in self.equations])


        # Second derivatives (accelerations)
        self.q_ddot = [var.diff('t', 2) for var in self.state_variables]

        # Extract coefficients and known terms (A, B)
        self.A, self.B = sp.linear_eq_to_matrix(self.equations, self.q_ddot)


    def _lambdify_matrices(self):
        """
        Lambdify the A and B matrices for numerical evaluation.
        Includes all necessary symbolic arguments: state variables, derivatives, and control inputs.
        """
        # Combine all state variables, state derivatives, and control inputs
        all_symbols = list(self.state_variables) + list(self.state_derivatives) + [
            self.T_d1, self.T_d2,
            self.tau_d1_x, self.tau_d1_y, self.tau_d1_z,
            self.tau_d2_x, self.tau_d2_y, self.tau_d2_z
        ]

        # Lambdify A and B matrices
        self.A_lambdified = sp.lambdify(all_symbols, self.A, "numpy")
        self.B_lambdified = sp.lambdify(all_symbols, self.B, "numpy")


    def get_A(self, state_and_control):
        """Numerically evaluate matrix A."""
        return self.A_lambdified(*state_and_control)

    def get_B(self, state_and_control):
        """Numerically evaluate matrix B."""
        return self.B_lambdified(*state_and_control)
        

