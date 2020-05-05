from math import pi
import numpy as np


def block_diag(*arrs):
    if arrs == ():
        arrs = ([],)
    arrs = [np.atleast_2d(a) for a in arrs]

    bad_args = [k for k in range(len(arrs)) if arrs[k].ndim > 2]
    if bad_args:
        raise ValueError("arguments in the following positions have dimension "
                            "greater than 2: %s" % bad_args)

    shapes = np.array([a.shape for a in arrs])
    out = np.zeros(np.sum(shapes, axis=0), dtype=arrs[0].dtype)

    r, c = 0, 0
    for i, (rr, cc) in enumerate(shapes):
        out[r:r + rr, c:c + cc] = arrs[i]
        r += rr
        c += cc
    return out


# robot body params
density = 40
body_length = 0.6
body_width = 0.25
body_height = 0.15
pseudo_mass = 0.01

# init sim params
init_thigh_len = 0.15
init_thigh_rad = 0.02
init_calf_len = 0.15

# nn params
input_output_ctrnn = np.ones((28, 28))
slow_context_ctrnn = np.ones((12, 12))
ctrnn_connections = block_diag(input_output_ctrnn, slow_context_ctrnn)
ctrnn_connections[24:28, 28:32] = np.ones((4, 4))
ctrnn_connections[28:32, 24:28] = np.ones((4, 4))
num_wts = 28**2+12**2+32
taus = [5]*28
taus.extend([70]*12)

# leg params
hindleg_y = -0.25
frontleg_y = 0.25
lo_h = -pi/4
hi_h = pi/4
lo_v = -pi/4
hi_v = pi/4

# evolution params
num_sim_tests = 4
sim_fitness_threshold = 0.95
sim_evalTime = 100
sim_popSize = 10
sim_len_sigma = 0.01
sim_rad_sigma = 0.001
nn_evalTime = 300
nn_cma_sigma = 0.1
