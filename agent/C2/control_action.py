class ControlAction:
    # PID Constants
    _max_u: float = 0.15  # Saturation value for control signal
    _h: float = .05  # h - sampling interval

    def __init__(self, ki: float, td: float, kp: float):
        # Controller main parameters
        self._Kp: float = kp  # Kp - proportional control
        self._Ki: float = ki  # Ti - integration time
        self._Td: float = td  # Td - differential time
        # memory for error
        self.e_m1: float = 0
        self.e_m2: float = 0
        # memory for the control signal
        self.u_m1: float = 0
        # Aux constants for the PID Controller
        self.K0: float = self._Kp * (1 + self._h * self._Ki + self._Td / self._h)
        self.K1: float = -self._Kp * (1 + 2 * self._Td / self._h)
        self.K2: float = self._Kp * self._Td / self._h

    def c_action(self, set_point: float, feedback: float, c_type: str = "PID"):
        """
        Python implementation of the "controlAction" function at:
            - https://github.com/nunolau/ciberRatoTools/blob/speedc/robsample/mainRob.c
        :param set_point: Goal value
        :param feedback: Real value
        :param c_type: Type of controller. Default is Proportional Integral Derivative
        :return: Controller outputs in such a way that, if it is called in a loop like in the do_micro_action
         in mainRob.py, then over time, the tendency is feedback == set_point
        """
        u: float = 0.0  # Error signal
        e: float  # Control signal
        # Compute error
        e = set_point - feedback

        # Implement control action depending on the type of control
        # PID only for now...
        if c_type == "PID":

            # Compute the control signal
            u = self.u_m1 + self.K0*e + self.K1*self.e_m1 + self.K2*self.e_m2

            # Store values for next iterations
            self.e_m2 = self.e_m1
            self.e_m1 = e
            self.u_m1 = u

            # Clip the control signal to avoid saturation
            if self.u_m1 > self._max_u:
                self.u_m1 = self._max_u
            if self.u_m1 < -self._max_u:
                self.u_m1 = -self._max_u

        else:
            print("Error: Type is invalid!")

        return u
