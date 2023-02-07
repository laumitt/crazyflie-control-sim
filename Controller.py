
class Controller1D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains, dt):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains dataclass):           pid gain class
        """
        self.params = cfparams

        # control gains
        self.kp_z = pid_gains.kp
        self.ki_z = pid_gains.ki
        self.kd_z = pid_gains.kd

        self.dt = dt

        # intialize pid components
        self.proportional = 0
        self.integral = 0
        self.derivative = 0

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """
        U = 0
        z_acc_des = 0

        # compute errors
        error = setpoint.z_pos - state.z_pos
        error_dot = setpoint.z_vel - state.z_vel

        self.proportional = self.kp_z*error
        self.integral += self.ki_z*error*self.dt
        self.derivative = self.kd_z*error_dot

        # zcc = z_acc_des + self.kd_z*(setpoint.z_vel - state.z_vel) + self.kp_z*(setpoint.z_pos - state.z_pos)
        zcc = z_acc_des + self.proportional + self.integral + self.derivative
        U = self.params.mass*(self.params.g + zcc)

        return U
