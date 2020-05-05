import constants as c
import utilities as util
import numpy as np
from eea import EEA
from simulator import SIMULATOR


starting_args = [c.init_thigh_len]*6
starting_args.extend([c.init_thigh_rad]*6)
starting_args.extend([c.init_calf_len]*6)
starting_args = np.array(starting_args)
starting_wts = np.random.rand(c.num_wts)*2-1

eea = EEA(starting_args, starting_wts)
# # move = np.random.uniform(c.lo_h, c.hi_h, 12)
# # y = util.sim_move(starting_args, move, pb=False, pp=True)
# util.eval_nn(starting_args, starting_wts, pb=False, pp=True)
eea.optimize_nn()
simulator = SIMULATOR(args=starting_args, exploration_mode=False, wts=eea.nn_wts, pb=False, pp=True)
simulator.sim.start()
simulator.sim.wait_to_finish()
