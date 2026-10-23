"""量測模擬中懸停與巡航的平均電功率，用來校正 energy.py 的估算參數。

python calibrate.py
"""
from drone_agent.geometry import Rect
from drone_agent.physics import Body
from drone_agent.servo import ServoLayer
from drone_agent.trajectory import TrajectoryController
from drone_agent.world import Environment

SERVO_HZ, TRAJ_HZ = 1000, 100


def average_power(airspeed_kmh, wind_kmh, seconds=120):
    env = Environment(wind_kmh=wind_kmh, wind_toward_deg=180.0, obstacles=[])  # 逆風
    body, servo, traj = Body(), ServoLayer(SERVO_HZ), TrajectoryController(TRAJ_HZ)
    body.p = [0.0, 0.0, 60.0]
    traj.z_sp = 60.0
    traj.set_path([(0.0, 0.0), (20000.0, 0.0)], 60.0, airspeed_kmh / 3.6)
    thrust_ref, direction = 0.0, (0.0, 0.0, 1.0)
    energy, n = 0.0, int(seconds * SERVO_HZ)
    for k in range(n):
        thrust, power = servo.step(thrust_ref)
        body.step(1.0 / SERVO_HZ, thrust, direction, env.wind_vector(0))
        if k > n // 4:
            energy += power / SERVO_HZ
        if k % (SERVO_HZ // TRAJ_HZ) == 0:
            thrust_ref, direction = traj.update(body)
    return energy / (seconds * 0.75)


if __name__ == "__main__":
    for airspeed, wind in [(0.001, 0), (36, 0), (36, 18.5), (36, 25)]:
        print(f"airspeed {airspeed:5.1f} km/h, headwind {wind:4.1f} km/h → {average_power(airspeed, wind):6.1f} W")
