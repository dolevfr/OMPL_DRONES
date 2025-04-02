import sympy as sp
import numpy as np
import pandas as pd
import webbrowser

class DronePayloadModel:
    def __init__(self, m_d=1, m_p=10, I_p=np.diag([1, 1, 1]), I_d=np.diag([0.05, 0.05, 0.05]), 
                 l=0.5, a=2, g=9.81):
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
        self.q_x, self.q_y, self.q_z, self.q_w = sp.symbols('q_x q_y q_z q_w', cls=sp.Function)

        # Drone orientations (quaternions)
        self.q_d1_x, self.q_d1_y, self.q_d1_z, self.q_d1_w = sp.symbols('q_d1_x q_d1_y q_d1_z q_d1_w', cls=sp.Function)
        self.q_d2_x, self.q_d2_y, self.q_d2_z, self.q_d2_w = sp.symbols('q_d2_x q_d2_y q_d2_z q_d2_w', cls=sp.Function)

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
        self.q_x, self.q_y, self.q_z, self.q_w = self.q_x(t), self.q_y(t), self.q_z(t), self.q_w(t)
        self.q_p = sp.Matrix([self.q_x, self.q_y, self.q_z, self.q_w])
        
        # Time-dependent variables for drones
        self.q_d1_x, self.q_d1_y, self.q_d1_z, self.q_d1_w = self.q_d1_x(t), self.q_d1_y(t), self.q_d1_z(t), self.q_d1_w(t)
        self.q_d2_x, self.q_d2_y, self.q_d2_z, self.q_d2_w = self.q_d2_x(t), self.q_d2_y(t), self.q_d2_z(t), self.q_d2_w(t)

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
            self.q_x, self.q_y, self.q_z, self.q_w,
            self.q_d1_x, self.q_d1_y, self.q_d1_z, self.q_d1_w,
            self.theta_c1, self.phi_c1,
            self.q_d2_x, self.q_d2_y, self.q_d2_z, self.q_d2_w, 
            self.theta_c2, self.phi_c2
        ])

        # === 3. Derivatives ===

        # Derivatives for payload position
        self.x_p_dot, self.y_p_dot, self.z_p_dot = sp.diff(self.x_p, t), sp.diff(self.y_p, t), sp.diff(self.z_p, t)

        # Derivatives for payload quaternion
        self.q_x_dot, self.q_y_dot, self.q_z_dot, self.q_w_dot = sp.diff(self.q_x, t), sp.diff(self.q_y, t), sp.diff(self.q_z, t), sp.diff(self.q_w, t)
        self.q_p_dot = sp.Matrix([self.q_x_dot, self.q_y_dot, self.q_z_dot, self.q_w_dot])

        # Derivatives for drone quaternions
        self.q_d1_dot = sp.Matrix([sp.diff(self.q_d1_x, t), sp.diff(self.q_d1_y, t), sp.diff(self.q_d1_z, t), sp.diff(self.q_d1_w, t)])
        self.q_d2_dot = sp.Matrix([sp.diff(self.q_d2_x, t), sp.diff(self.q_d2_y, t), sp.diff(self.q_d2_z, t), sp.diff(self.q_d2_w, t)])

        # Derivatives for cable angles
        self.theta_c1_dot, self.phi_c1_dot = sp.diff(self.theta_c1, t), sp.diff(self.phi_c1, t)
        self.theta_c2_dot, self.phi_c2_dot = sp.diff(self.theta_c2, t), sp.diff(self.phi_c2, t)

        # Vector of all state derivatives
        self.state_derivatives = sp.Matrix([
            self.x_p_dot, self.y_p_dot, self.z_p_dot,
            self.q_x_dot, self.q_y_dot, self.q_z_dot, self.q_w_dot,
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
        self.payload_orientation = sp.Matrix([self.q_x, self.q_y, self.q_z, self.q_w])
        self.payload_angular_vel = self.quaternion_angular_velocity(self.q_p, self.q_p_dot)

        # Define drone endpoints of the rod
        self.r1 = self.payload_pos + (self.a / 2) * self._rotate_vector([1, 0, 0], self.payload_orientation)
        self.r2 = self.payload_pos - (self.a / 2) * self._rotate_vector([1, 0, 0], self.payload_orientation)

        # Drone positions with respect to cable orientations
        self.drone1_pos = self.r1 + self.l * (sp.Matrix([sp.sin(self.theta_c1) * sp.cos(self.phi_c1), sp.sin(self.theta_c1) * sp.sin(self.phi_c1), sp.cos(self.theta_c1)]))
        self.drone2_pos = self.r2 + self.l * (sp.Matrix([sp.sin(self.theta_c2) * sp.cos(self.phi_c2), sp.sin(self.theta_c2) * sp.sin(self.phi_c2), sp.cos(self.theta_c2)]))

        # Drone orientations
        self.drone1_orientation = sp.Matrix([self.q_d1_x, self.q_d1_y, self.q_d1_z, self.q_d1_w])
        self.drone2_orientation = sp.Matrix([self.q_d2_x, self.q_d2_y, self.q_d2_z, self.q_d2_w])

        # Drone velocities
        self.drone1_vel = sp.diff(self.drone1_pos, t)
        self.drone2_vel = sp.diff(self.drone2_pos, t)

        # Drone angular velocities
        self.drone1_angular_vel = self.quaternion_angular_velocity(self.drone1_orientation, self.q_d1_dot)
        self.drone2_angular_vel = self.quaternion_angular_velocity(self.drone2_orientation, self.q_d2_dot)

        # === 5. Angular Velocities and Accelerations ===

        # Angular velocities
        self.omega_p_x, self.omega_p_y, self.omega_p_z = sp.symbols('omega_p_x omega_p_y omega_p_z', cls=sp.Function)
        self.omega_d1_x, self.omega_d1_y, self.omega_d1_z = sp.symbols('omega_d1_x omega_d1_y omega_d1_z', cls=sp.Function)
        self.omega_d2_x, self.omega_d2_y, self.omega_d2_z = sp.symbols('omega_d2_x omega_d2_y omega_d2_z', cls=sp.Function)
        self.omega_p_x, self.omega_p_y, self.omega_p_z = self.omega_p_x(t), self.omega_p_y(t), self.omega_p_z(t)
        self.omega_d1_x, self.omega_d1_y, self.omega_d1_z = self.omega_d1_x(t), self.omega_d1_y(t), self.omega_d1_z(t)
        self.omega_d2_x, self.omega_d2_y, self.omega_d2_z = self.omega_d2_x(t), self.omega_d2_y(t), self.omega_d2_z(t)
        self.omega_p = sp.Matrix([self.omega_p_x, self.omega_p_y, self.omega_p_z])
        self.omega_d1 = sp.Matrix([self.omega_d1_x, self.omega_d1_y, self.omega_d1_z])
        self.omega_d2 = sp.Matrix([self.omega_d2_x, self.omega_d2_y, self.omega_d2_z])

        # Angular accelerations
        self.omega_dot_p_x, self.omega_dot_p_y, self.omega_dot_p_z = sp.symbols('omega_dot_p_x omega_dot_p_y omega_dot_p_z', cls=sp.Function)
        self.omega_dot_d1_x, self.omega_dot_d1_y, self.omega_dot_d1_z = sp.symbols('omega_dot_d1_x omega_dot_d1_y omega_dot_d1_z', cls=sp.Function)
        self.omega_dot_d2_x, self.omega_dot_d2_y, self.omega_dot_d2_z = sp.symbols('omega_dot_d2_x omega_dot_d2_y omega_dot_d2_z', cls=sp.Function)
        self.omega_dot_p_x, self.omega_dot_p_y, self.omega_dot_p_z = self.omega_dot_p_x(t), self.omega_dot_p_y(t), self.omega_dot_p_z(t)
        self.omega_dot_d1_x, self.omega_dot_d1_y, self.omega_dot_d1_z = self.omega_dot_d1_x(t), self.omega_dot_d1_y(t), self.omega_dot_d1_z(t)
        self.omega_dot_d2_x, self.omega_dot_d2_y, self.omega_dot_d2_z = self.omega_dot_d2_x(t), self.omega_dot_d2_y(t), self.omega_dot_d2_z(t)
        self.omega_dot_p = sp.Matrix([self.omega_dot_p_x, self.omega_dot_p_y, self.omega_dot_p_z])
        self.omega_dot_d1 = sp.Matrix([self.omega_dot_d1_x, self.omega_dot_d1_y, self.omega_dot_d1_z])
        self.omega_dot_d2 = sp.Matrix([self.omega_dot_d2_x, self.omega_dot_d2_y, self.omega_dot_d2_z])

        self.new_derivatives = [self.x_p_dot, self.y_p_dot, self.z_p_dot,
                                self.omega_p_x, self.omega_p_y, self.omega_p_z,
                                self.omega_d1_x, self.omega_d1_y, self.omega_d1_z,
                                self.theta_c1_dot, self.phi_c1_dot,
                                self.omega_d2_x, self.omega_d2_y, self.omega_d2_z,
                                self.theta_c2_dot, self.phi_c2_dot]




    @staticmethod
    def _rotate_vector(vec, quaternion):
        """Rotate a vector using a quaternion."""
        vec = sp.Matrix(vec)
        q_x, q_y, q_z, q_w = quaternion
        R = sp.Matrix([
            [1 - 2 * (q_y**2 + q_z**2), 2 * (q_x * q_y - q_z * q_w), 2 * (q_x * q_z + q_y * q_w)],
            [2 * (q_x * q_y + q_z * q_w), 1 - 2 * (q_x**2 + q_z**2), 2 * (q_y * q_z - q_x * q_w)],
            [2 * (q_x * q_z - q_y * q_w), 2 * (q_y * q_z + q_x * q_w), 1 - 2 * (q_x**2 + q_y**2)]
        ])
        return R * vec
    
    def G(self, q):
        G = sp.Matrix([
            [q[0],  q[3], -q[2], -q[1]],
            [-q[3],  q[0],  q[1], -q[2]],
            [q[2], -q[1],  q[0], -q[3]]
        ])
        return G
    
    def E(self, q):
        E = sp.Matrix([
            [q[0], -q[3], q[2], -q[1]],
            [q[3], q[0], -q[1], -q[2]],
            [-q[2], q[1], q[0], -q[3]]
        ])
        return E

    def quaternion_angular_velocity(self, q, q_dot):
        # Quaternion derivative to angular velocity

        return 2 * self.E(q) * q_dot

    
    def q_dot_to_omega(self, q, omega):
        # Calculate quaternion derivative

        q_dot = 0.5 * self.E(q).T * omega

        return q_dot
        
    def q_ddot_to_omega_dot(self, q, q_dot, omega_dot):

        # Calculate quaternion double derivative
        q_ddot = self.E(q_dot).T * self.E(q) * q + 0.5 * self.E(q).T * omega_dot

        return q_ddot
    
    
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

        def compute_torque_force(q, torque):

            forces = 2 * self.G(q).T * sp.Matrix(torque)
            return forces

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
        L = T_p + T_d - (V_p + V_d)

        # Generalized forces for each degree of freedom
        generalized_forces = sp.Matrix([self.calculate_generalized_force(var) for var in self.state_variables])


        # # Pretty print the generalized forces
        # for coordinate, force in zip(self.state_variables, generalized_forces):
        #     sp.pprint(f"{coordinate}: {force}")

        # Lagrange terms without constraints
        terms = sp.Matrix([
            L.diff(derivative).diff('t') - L.diff(variable) - force
            for derivative, variable, force in zip(self.state_derivatives, self.state_variables, generalized_forces)
        ])

        terms_payload = sp.Matrix(terms[3:7])
        terms_drone1 = sp.Matrix(terms[7:11])
        terms_drone2 = sp.Matrix(terms[13:17])

        result_payload = self.G(self.q_p) * terms_payload
        result_drone1 = self.G(self.drone1_orientation) * terms_drone1
        result_drone2 = self.G(self.drone2_orientation) * terms_drone2

        # Rewrite terms
        terms = sp.Matrix([
            terms[0],  # Old 1st term
            terms[1],  # Old 2nd term
            terms[2],  # Old 3rd term
            *result_payload,  # Result of payload G multiplication (4th-6th terms)
            *result_drone1,   # Result of drone1 G multiplication (7th-9th terms)
            terms[11],        # Old 12th term
            terms[12],        # Old 13th term
            *result_drone2,   # Result of drone2 G multiplication (14th-16th terms)
            terms[17],        # Old 18th term
            terms[18]         # Old 19th term
        ])

        # Print terms to a table and show it in Chrome
        terms_df = pd.DataFrame([[str(term)] for term in terms], columns=["Terms"])

        # Save DataFrame to HTML
        html_terms = f"""
        <html>
        <head>
            <title>Lagrange Terms</title>
        </head>
        <body>
            <h1>Lagrange Terms</h1>
            {terms_df.to_html()}
        </body>
        </html>
        """
        with open("/tmp/terms.html", "w") as f:
            f.write(html_terms)

        # Open the HTML file in Chrome
        webbrowser.get(using='google-chrome').open("file:///tmp/terms.html")

        q_p_dot = self.q_dot_to_omega(self.q_p, self.omega_p)
        q_d1_dot = self.q_dot_to_omega(self.drone1_orientation, self.omega_d1)
        q_d2_dot = self.q_dot_to_omega(self.drone2_orientation, self.omega_d2)

        q_p_ddot = self.q_ddot_to_omega_dot(self.q_p, self.q_p_dot, self.omega_dot_p)
        q_d1_ddot = self.q_ddot_to_omega_dot(self.drone1_orientation, self.q_d1_dot, self.omega_dot_d1)
        q_d2_ddot = self.q_ddot_to_omega_dot(self.drone2_orientation, self.q_d2_dot, self.omega_dot_d2)

        substitutions = {

            self.q_x.diff('t'): q_p_dot[0],
            self.q_y.diff('t'): q_p_dot[1],
            self.q_z.diff('t'): q_p_dot[2],
            self.q_w.diff('t'): q_p_dot[3],

            self.q_d1_x.diff('t'): q_d1_dot[0],
            self.q_d1_y.diff('t'): q_d1_dot[1],
            self.q_d1_z.diff('t'): q_d1_dot[2],
            self.q_d1_w.diff('t'): q_d1_dot[3],

            self.q_d2_x.diff('t'): q_d2_dot[0],
            self.q_d2_y.diff('t'): q_d2_dot[1],
            self.q_d2_z.diff('t'): q_d2_dot[2],
            self.q_d2_w.diff('t'): q_d2_dot[3],


            self.q_x.diff('t', 2): q_p_ddot[0],
            self.q_y.diff('t', 2): q_p_ddot[1],
            self.q_z.diff('t', 2): q_p_ddot[2],
            self.q_w.diff('t', 2): q_p_ddot[3],

            self.q_d1_x.diff('t', 2): q_d1_ddot[0],
            self.q_d1_y.diff('t', 2): q_d1_ddot[1],
            self.q_d1_z.diff('t', 2): q_d1_ddot[2],
            self.q_d1_w.diff('t', 2): q_d1_ddot[3],

            self.q_d2_x.diff('t', 2): q_d2_ddot[0],
            self.q_d2_y.diff('t', 2): q_d2_ddot[1],
            self.q_d2_z.diff('t', 2): q_d2_ddot[2],
            self.q_d2_w.diff('t', 2): q_d2_ddot[3]
        }

        terms = terms.subs(substitutions)

        equations = [sp.Eq(term, 0) for term in terms]


        # Print equations to a table and show it in Chrome
        equations_df = pd.DataFrame([[str(eq.lhs), str(eq.rhs)] for eq in equations], columns=["LHS", "RHS"])

        # Save DataFrame to HTML
        html_equations = f"""
        <html>
        <head>
            <title>Lagrange Equations</title>
        </head>
        <body>
            <h1>Lagrange Equations</h1>
            {equations_df.to_html()}
        </body>
        </html>
        """
        with open("/tmp/equations.html", "w") as f:
            f.write(html_equations)

        # Open the HTML file in Chrome
        webbrowser.get(using='google-chrome').open("file:///tmp/equations.html")

        q_ddot = [self.x_p_dot.diff('t'), self.y_p_dot.diff('t'), self.z_p_dot.diff('t'),
                  self.omega_dot_p_x, self.omega_dot_p_y, self.omega_dot_p_z,
                  self.omega_dot_d1_x, self.omega_dot_d1_y, self.omega_dot_d1_z,
                  self.theta_c1_dot.diff('t'), self.phi_c1_dot.diff('t'),
                  self.omega_dot_d2_x, self.omega_dot_d2_y, self.omega_dot_d2_z,
                  self.theta_c2_dot.diff('t'), self.phi_c2_dot.diff('t')]

        # Extract A and B matrices
        self.A, self.B = sp.linear_eq_to_matrix(equations, q_ddot)

        # Print A and B matrices to a table and show it in Chrome

        # Convert A and B matrices to DataFrames
        A_df = pd.DataFrame(self.A.tolist())
        B_df = pd.DataFrame(self.B.tolist())

        # Save DataFrames to HTML
        html = f"""
        <html>
        <head>
            <title>A and B Matrices</title>
        </head>
        <body>
            <h1>Matrix A</h1>
            {A_df.to_html()}
            <h1>Matrix B</h1>
            {B_df.to_html()}
        </body>
        </html>
        """
        with open("/tmp/matrices.html", "w") as f:
            f.write(html)

        # Open the HTML file in Chrome
        webbrowser.get(using='google-chrome').open("file:///tmp/matrices.html")



    def _lambdify_matrices(self):
        """
        Lambdify the A and B matrices for numerical evaluation.
        Includes all necessary symbolic arguments: state variables, derivatives, and control inputs.
        """
        # Combine all state variables, state derivatives, and control inputs
        all_symbols = list(self.state_variables) + list(self.new_derivatives) + [
            self.T_d1, self.tau_d1_x, self.tau_d1_y, self.tau_d1_z,
            self.T_d2, self.tau_d2_x, self.tau_d2_y, self.tau_d2_z
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
        

