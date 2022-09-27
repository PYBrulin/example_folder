import os

MATPLOTLIB_AVAILABLE = False
try:
    import matplotlib.pyplot as plt

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    pass


class simplestPID:
    def __init__(
        self,
        Kp: int = 5,
        Kd: int = 5,
        Ki: int = 5,
        max_clamp: float = 30,
    ) -> None:
        self.i = 0  # initialize integral
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.max_clamp = max_clamp

    def update(
        self,
        error: float,
        error_prev: float,
        dt: float,
    ) -> float:
        p = self.Kp * error
        d = self.Kd * ((error - error_prev) / dt)
        i = self.i + self.Ki * error * dt

        out = p

        # Save intergal for later
        self.i = i

        return out


def simulation():

    # Solid object properties
    m = 1
    g = 9.81

    # Set point
    target = 400
    dt = 0.01

    # Variable initialization
    pos = 0
    vel = 0
    time = 0
    error_prev = 0
    pid = simplestPID(Kp=5, Kd=5, Ki=5)

    d = {}
    d["time"] = []
    d["thrust"] = []
    d["pos"] = []
    d["target"] = []

    # Save results to txt file relative to this script
    with open(
        os.path.join(os.path.dirname(__file__), "feedback_control.txt"), "w"
    ) as f:
        for _ in range(1000):
            # PID computation
            error = target - pos
            thrust = pid.update(error, error_prev, dt)

            new_vel = vel + ((thrust / m) - g) * dt
            new_pos = pos + vel * dt + (0.5 * ((thrust / m) - g)) * (dt**2)

            # Update
            error_prev = target - pos
            pos = new_pos
            vel = new_vel
            time += dt

            time, thrust, pos, target
            _str = "Time: {:.4f}\tThrust: {:.4f} N\tPosition: {:.4f} m\tDesired: {:.4f} m".format(
                time, thrust, pos, target
            )
            f.write(_str)
            d["time"].append(time)
            d["thrust"].append(thrust)
            d["pos"].append(pos)
            d["target"].append(target)
            print(_str)

    if MATPLOTLIB_AVAILABLE:
        plt.figure()
        plt.plot(d["time"], d["pos"])
        plt.plot(d["time"], d["target"])
        plt.show()


if __name__ == "__main__":
    simulation()
