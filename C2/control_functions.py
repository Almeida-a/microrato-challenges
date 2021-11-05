import sys

# TODO create a class for storing these variables (instead of being global)
K0: float
K1: float
K2: float
e_m1: float = 0
e_m2: float = 0
u_m1: float = 0


def control_action(set_point: float, feedback: float, c_type: str = "PID"):
    """
    Python implementation of the "controlAction" function at:
        - https://github.com/nunolau/ciberRatoTools/blob/speedc/robsample/mainRob.c
    :param set_point:
    :param feedback:
    :param c_type:
    :return:
    """
    u: float = 0.0  # Error signal
    e: float = 0.0  # Control signal

    max_u: float = 0.5  # Saturation value for control signal

    # PID Constants
    max_u: float = 0.5
    _Kp: float = 1
    _Ti: float = sys.float_info.max
    _Td: float = .0
    h: float = .05
    delta: float = .05

    # Aux constants for the PID Controller
    global K0, K1, K2
    K0 = _Kp * (1 + h/_Ti + _Td/h)
    K1 = -_Kp * (1 + 2*_Td/h)
    K2 = _Kp * _Td/h

    # memory for error
    global e_m1, e_m2

    # memory for the control signal
    global u_m1

    # Compute error
    e = set_point - feedback

    # Implement control action depending on the type of control
    # PID only for now...
    if c_type == "PID":

        # Compute the control signal
        u = u_m1 + K0*e + K1*e_m1 + K2*e_m2

        # Store values for next iterations
        e_m2 = e_m1
        e_m1 = e
        u_m1 = u

        # Clip the control signal to avoid saturation
        if u_m1 > max_u:
            u_m1 = max_u
        if u_m1 < -max_u:
            u_m1 = -max_u

    else:
        print("Error: Type is invalid!")

    return u
