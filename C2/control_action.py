import sys


class ControlAction:
    # PID Constants TODO constants or arguments?
    _max_u: float = 1.4  # Saturation value for control signal
    _Kp: float = 0.70  # Kp - proportional control
    _Ti: float = 10.0  # Ti - integration time
    _Td: float = .0  # Td - differential time
    _h: float = .05  # h - sampling interval

    def __init__(self):
        # memory for error
        self.e_m1: float = 0
        self.e_m2: float = 0
        # memory for the control signal
        self.u_m1: float = 0

        # Aux constants for the PID Controller
        self.K0: float = self._Kp * (1 + self._h / self._Ti + self._Td / self._h)
        self.K1: float = -self._Kp * (1 + 2 * self._Td / self._h)
        self.K2: float = self._Kp * self._Td / self._h

    def c_action(self, set_point: float, feedback: float, c_type: str = "PID"):
        """
        Python implementation of the "controlAction" function at:
            - https://github.com/nunolau/ciberRatoTools/blob/speedc/robsample/mainRob.c
        :param set_point:
        :param feedback:
        :param c_type:
        :return:
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
