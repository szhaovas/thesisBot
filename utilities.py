import numpy as np
import constants as c
from simulator import SIMULATOR
# from board import SCL, SDA
# import busio
# from adafruit_pca9685 import PCA9685
# from adafruit_motor import servo


# cosine similarity mapped to the range [0, 1]
def mapped_cosine_similarity(vec1, vec2):
    dot_product = np.dot(vec1, vec2)
    norm1 = np.linalg.norm(vec1)
    norm2 = np.linalg.norm(vec2)
    return (dot_product / (norm1 * norm2) + 1) / 2


def args_mutate(args):
    assert args.size == 18
    result = np.zeros(18)
    # thigh_lens > 0.01
    for i in range(6):
        len = np.random.normal(args[i], c.sim_len_sigma)
        while len <= 0.01:
            len = np.random.normal(args[i], c.sim_len_sigma)
        result[i] = len
    # FIXME: looser restrictions?
    # 0.01 < thigh_rads <= c.body_height/2
    for i in range(6, 12):
        rad = np.random.normal(args[i], c.sim_rad_sigma)
        while rad <= 0.01 or rad > c.body_height/2:
            rad = np.random.normal(args[i], c.sim_rad_sigma)
        result[i] = rad
    # thigh_lens**2 - calf_lens**2 > thigh_rads**2
    for i in range(12, 18):
        len = np.random.normal(args[i], c.sim_len_sigma)
        thigh_len_sqrt = result[i-12]**2
        thigh_rad_sqrt = result[i-6]**2
        while (thigh_len_sqrt - len**2) <= thigh_rad_sqrt:
            len = np.random.normal(args[i], c.sim_len_sigma)
        result[i] = len
    return result


# Gives the move that causes the largest disagreement among args_population
#   also returns the estimates of all args_population for the move in sim_vec
def controversial_move(args_population):
    # FIXME: lo & hi boundaries; num_sim_tests
    random_moves = np.random.uniform(c.lo_h, c.hi_h, (c.num_sim_tests, 12))

    highest_difference_rating = 0
    controversial_move = np.zeros(12)
    controversial_estimates = np.zeros((c.sim_popSize, 3))
    for move in random_moves:
        estimates = np.zeros(controversial_estimates.shape)
        for r in range(c.sim_popSize):
            estimates[r] = sim_move(args_population[r], move)
        difference_rating = np.sum(np.var(estimates, axis=0))
        if difference_rating > highest_difference_rating:
            highest_difference_rating = difference_rating
            controversial_move = move
            controversial_estimates = estimates

    return controversial_move, controversial_estimates


def sim_move(args, move, pb=True, pp=False):
    simulator = SIMULATOR(args=args, exploration_mode=True, angles=move, pb=pb, pp=pp)
    simulator.sim.start()
    simulator.sim.wait_to_finish()
    new_x = simulator.sim.get_sensor_data(
        sensor_id=simulator.position_sensor, svi=0)[-1]
    new_y = simulator.sim.get_sensor_data(
        sensor_id=simulator.position_sensor, svi=1)[-1]
    new_z = simulator.sim.get_sensor_data(
        sensor_id=simulator.position_sensor, svi=2)[-1]
    return np.array([new_x, new_y, new_z-c.body_height/2])


def eval_nn(args, wts, pb=True, pp=False):
    simulator = SIMULATOR(args=args, exploration_mode=False, wts=wts, pb=pb, pp=pp)
    simulator.sim.start()
    simulator.sim.wait_to_finish()
    new_y = simulator.sim.get_sensor_data(
        sensor_id=simulator.position_sensor, svi=1)[-1]
    return new_y

# nparray
def real_move(move):
    pass
    # kit = ServoKit(channels=16)
    # servo = adafruit_motor.servo.Servo(servo_channel)
    # kit.servo[0].angle = 0
    #
    # i2c = busio.I2C(SCL, SDA)
    # pca = PCA9685(i2c)
    # pca.frequency = 50
