import threading

import rospy
from dynamic_reconfigure.server import Server
from hippocampus_common.node import Node
from std_msgs.msg import Float64
from hippocampus_msgs.msg import PidDebug

from bluerov_control.cfg import PidControlConfig


class PidNode(Node):
    """A template class for PID controller nodes.
    """
    def __init__(self, name):
        """

        Args:
            name (str): The node's name.
        """
        super(PidNode, self).__init__(name=name)

        #: lock to provide thread safety for non local variables
        self.data_lock = threading.RLock()

        self.controller = Controller()
        self._t_last = rospy.get_time()

        self._dyn_reconf_pid = Server(PidControlConfig,
                                      self._on_pid_reconfigure)
        self.pid_debug_pub = rospy.Publisher("~pid_debug",
                                             PidDebug,
                                             queue_size=30)

    def _on_pid_reconfigure(self, config, level):
        """Callback for the dynamic reconfigure service to set PID control
        specific parameters.

        Args:
            config (dict): Holds parameters and values of the dynamic
                reconfigure config file.
            level (int): Level of the changed parameters

        Returns:
            dict: The actual parameters that are currently applied.
        """
        with self.data_lock:
            self.controller.K_p = config["K_p"]
            self.controller.T_i = config["T_i"]
            self.controller.T_d = config["T_d"]
            
            self.controller.saturation = [
                config["saturation_lower"], config["saturation_upper"]
            ]
            self.controller.integral_limits = [
                config["integral_limit_lower"], config["integral_limit_upper"]
            ]
            config["K_p"] = self.controller.K_p
            config["T_i"] = self.controller.T_i
            config["T_d"] = self.controller.T_d
            lower, upper = self.controller.saturation
            config["saturation_lower"] = lower
            config["saturation_upper"] = upper
            lower, upper = self.controller.integral_limits
            config["integral_limit_lower"] = lower
            config["integral_limit_upper"] = upper
        return config

    def _update_dt(self, now):
        """Updates the time difference between controller updates.

        Args:
            now (float): Current UTC stamp in seconds.

        Returns:
            float: Time difference between current and last controller update.
        """
        dt = now - self._t_last
        dt_max = 0.1
        dt_min = 0.01
        if dt > dt_max:
            rospy.logwarn(
                "[%s] Timespan since last update too large (%fs)."
                "Limited to %fs", rospy.get_name(), dt, dt_max)
            dt = dt_max
        elif dt < dt_min:
            rospy.logwarn(
                "[%s] Timespan since last update too small (%fs)."
                "Limited to %fs", rospy.get_name(), dt, dt_min)
            dt = dt_min

        self._t_last = now
        return dt

    def publish_pid_debug(self):
        """Publish debug information.
        """
        with self.data_lock:
            msg = PidDebug(error=self.controller._error,
                           error_derivative=self.controller._derivative,
                           error_integral=self.controller._integral,
                           setpoint=self.setpoint,
                           K_p=self.controller.K_p,
                           T_i=self.controller.T_i,
                           T_d=self.controller.T_d,
                           u_p=self.controller._u_p,
                           u_i=self.controller._u_i,
                           u_d=self.controller._u_d,
                           u=self.controller._u)
        msg.header.stamp = rospy.Time.now()
        self.pid_debug_pub.publish(msg)

    def update_controller(self, error, now, derror=None):
        """Computes the updated PID controller's output.

        Args:
            error (float): Control error.
            now (float): Current UTC timestamp in seconds.
            derror (float, optional): If set the value is used as derivate of
                the control error. Otherwise a difference quotient is computed.
                Defaults to None.

        Returns:
            float: Control output.
        """
        dt = self._update_dt(now)
        u = self.controller.update(error=error, dt=dt, derror=derror)
        self.publish_pid_debug()
        return u


class Controller():
    """A quite normal PID controller.

    PID gains can either be set on the creation of a controller instance or
    via the respective properties any time later on. Compute the control output
    by invoking ``update``.

    Examples:
        ::
            controller = Controller(p_gain=3.4, i_gain=1.1, d_gain=0.1)
            controller.p_gain = 1.0
            print("Controller's p_gain: {}".format(controller.p_gain))

            control_error = 1.0
            control_output = controller.update(error=control_error, dt=0.02)
            print("Control output: {}".format(control_output))
    """
    def __init__(self, K_p=1.0, T_i=0.0, T_d=0.0):
        self.K_p = K_p
        self.T_i = T_i
        self.T_d = T_d
        self.saturation = [-100, 100]
        self.integral_limits = [-1, 1]
        self._error = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._last_error = 0.0
        self._u_p = 0.0
        self._u_i = 0.0
        self._u_d = 0.0
        self._u = 0.0

    def update(self, error, dt, derror=None):
        """Compute the control output.

        Args:
            error (float): Control error.
            dt (float): Timespan used to integrate the control error and
                optionally compute the derivative of the control error.
            derror (float, None, optional): If not None use this argument as
                derivative of the control error instead of computing it.
                Defaults to None.

        Returns:
            float: Control output.
        """
        self._error = error
        self._update_integral(error, dt)
        self._update_derivative(error, dt, derror)
        self._u_p = error * self.K_p
        
        if self.T_i == 0.0:
            self._u_i = 0.0
        else:
            self._u_i = self._integral * (self.K_p / self.T_i)
        
        self._u_d = self._derivative * (self.K_p * self.T_d)
        
        self._u = self.constrain(self._u_p + self._u_i + self._u_d)
        return self._u

    def constrain(self, u):
        return max(self.saturation[0], min(self.saturation[1], u))

    def _update_integral(self, error, dt):
        delta_integral = dt * error
        self._integral = max(
            self.integral_limits[0],
            min(self.integral_limits[1], self._integral + delta_integral))

    def _update_derivative(self, error, dt, derror=None):
        """Computes the derivative of the control error.

        Args:
            error (float): The control error. If 'derror' is passed this value
                has no effect.
            dt (float): Timespan to compute the difference quotient.
            derror (float): If not None this value is used as derivative of the
                control error. Otherwise the derivate of computed via difference
                quotient.
        """
        if derror is None:
            self._derivative = (error - self._last_error) / dt
        else:
            self._derivative = derror

    @property
    def saturation(self):
        """Get or set the saturation.

        The saturation limits the control output. Useful in cases where the
        control output is not allowed to exceed certain limits.
        """
        return self._saturation

    @saturation.setter
    def saturation(self, boundaries):
        lower = float(boundaries[0])
        upper = float(boundaries[1])
        if lower > upper:
            self._saturation = [upper, lower]
        else:
            self._saturation = [lower, upper]

    @property
    def integral_limits(self):
        """Get or set the limits of the controller's integral part.

        """
        return self._integral_limits

    @integral_limits.setter
    def integral_limits(self, boundaries):
        lower = float(boundaries[0])
        upper = float(boundaries[1])
        if lower > upper:
            self._integral_limits = [upper, lower]
        else:
            self._integral_limits = [lower, upper]

    @property
    def K_p(self):
        """Get or set the proportional gain of the controller.

        """
        return self._K_p

    @K_p.setter
    def K_p(self, value):
        self._K_p = float(value)

    @property
    def T_i(self):
        """Get or set the integral gain of the controller.

        """
        return self._T_i

    @T_i.setter
    def T_i(self, value):
        self._T_i = float(value)

    @property
    def T_d(self):
        """Get or set the derivative gain of the controller.

        """
        return self._T_d

    @T_d.setter
    def T_d(self, value):
        self._T_d = float(value)
