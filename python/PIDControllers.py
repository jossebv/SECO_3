from scipy import signal
import math


class Controller:
    def __init__(self, transfer_function: signal.lti):
        self.transfer_function = transfer_function

    def get_transfer_function(self) -> signal.lti:
        return self.transfer_function

    def get_label(self) -> str:
        return ""

    def get_controller_type(self) -> str:
        return ""


class PController(Controller):
    def __init__(self, p: float, K: float, Kp: float):
        transfer_function = signal.lti([Kp * K], [p**2, p**2, K * Kp])
        super().__init__(transfer_function)
        self.Kp = Kp
        self.p = p
        self.K = K

    def get_label(self):
        return (
            f"Kp: {self.Kp}, $\\zeta$ {math.sqrt((self.p**2)/(4*self.Kp*self.K)):.2f}"
        )

    def get_controller_type(self):
        return "P"


class P_DController(Controller):
    def __init__(self, p: float, K: float, Kp: float, tauD: float):
        transfer_function = signal.lti(
            [Kp * K], [p**2, (p + Kp * K * tauD) * p, K * Kp]
        )
        super().__init__(transfer_function)
        self.Kp = Kp
        self.tauD = tauD
        self.p = p
        self.K = K

    def get_label(self):
        return f"Kp: {self.Kp}, $\\tau_D$: {self.tauD}"

    def get_controller_type(self):
        return "P-D"


class PDController(Controller):
    def __init__(self, p: float, K: float, Kp: float, tauD: float):
        transfer_function = signal.lti(
            [Kp * K * tauD * p, Kp * K],
            [p**2, (p + Kp * K * tauD) * p, K * Kp],
        )
        super().__init__(transfer_function)
        self.Kp = Kp
        self.tauD = tauD
        self.p = p
        self.K = K

    def get_label(self):
        return f"Kp: {self.Kp}, $\\tau_D$: {self.tauD}"

    def get_controller_type(self):
        return "PD"


class PIController(Controller):
    def __init__(self, p: float, K: float, Kp: float, tauI: float):
        # transfer_function = signal.lti(
        #     [Kp * K, Kp * K / tauI],
        #     [1, p, Kp * K, K * Kp / tauI],
        # )
        transfer_function = signal.lti(
            [Kp * K * p, Kp * K / tauI],
            [p**3, p**3, Kp * K * p, K * Kp / tauI],
        )
        super().__init__(transfer_function)
        self.Kp = Kp
        self.tauI = tauI
        self.p = p
        self.K = K

    def get_label(self):
        return f"Kp: {self.Kp}, $\\tau_I$: {self.tauI}"

    def get_controller_type(self):
        return "PI"


class PIDController(Controller):
    def __init__(self, p: float, K: float, Kp: float, tauD: float, tauI: float):
        # transfer_function = signal.lti(
        #     [Kp * K, Kp * K / tauI],
        #     [1, p, Kp * K, K * Kp / tauI],
        # )
        transfer_function = signal.lti(
            [K * Kp * tauD * p**2, p / tauD, 1 / (tauD * tauI)],
            [p**3, (p**3 + K * Kp * tauD * p**2), p / tauD, 1 / (tauD * tauI)],
        )
        super().__init__(transfer_function)
        self.Kp = Kp
        self.tauI = tauI
        self.tauD = tauD
        self.p = p
        self.K = K

    def get_label(self):
        return f"Kp: {self.Kp}, $\\tau_I$: {self.tauI}, $\\tau_D$: {self.tauD}"

    def get_controller_type(self):
        return "PID"


class PI_DController(Controller):
    def __init__(self, p: float, K: float, Kp: float, tauI: float, tauD: float):
        transfer_function = signal.lti(
            [K * Kp * p, K * Kp / tauI],
            [p**3, (p**3 + K * Kp * tauD * p**2), K * Kp * p, K * Kp / tauI],
        )
        super().__init__(transfer_function)
        self.Kp = Kp
        self.tauI = tauI
        self.tauD = tauD
        self.p = p
        self.K = K

    def get_label(self):
        return f"Kp: {self.Kp}, $\\tau_I$: {self.tauI}, $\\tau_D$: {self.tauD}"

    def get_controller_type(self):
        return "PI-D"


class PID_DController(Controller):
    def __init__(
        self,
        p: float,
        K: float,
        Kp: float,
        tauI: float,
        tauD: float,
    ):
        transfer_function = signal.lti(
            [K * Kp * tauD * (p**2), K * Kp * p, K * Kp / tauI],
            [
                p**3,
                K * Kp * tauD * (p**2),
                K * Kp * p,
                K * Kp / tauI,
            ],
        )
        super().__init__(transfer_function)
        self.Kp = Kp
        self.tauI = tauI
        self.tauD = tauD
        self.p = p
        self.K = K

    def get_label(self):
        return f"Kp: {self.Kp}, $\\tau_I$: {self.tauI}, $\\tau_D1$: {self.tauD}"

    def get_controller_type(self):
        return "PID-D"


class D__PIDController(Controller):
    def __init__(
        self,
        p: float,
        K: float,
        Kp: float,
        tauI: float,
        tauD: float,
    ):
        transfer_function = signal.lti(
            [(p**3) + K * Kp * tauD * (p**2), K * Kp * p, K * Kp / tauI],
            [
                p**3,
                (p**3) + K * Kp * tauD * (p**2),
                K * Kp * p,
                K * Kp / tauI,
            ],
        )
        super().__init__(transfer_function)
        self.Kp = Kp
        self.tauI = tauI
        self.tauD = tauD
        self.p = p
        self.K = K

    def get_label(self):
        return f"Kp: {self.Kp}, $\\tau_I$: {self.tauI}, $\\tau_D1$: {self.tauD}"

    def get_controller_type(self):
        return "D|PID"
