import cma
import constants as c
import utilities as util
import numpy as np


class EEA:
    def __init__(self, starting_args, starting_wts):
        self.sim_args = starting_args
        self.nn_wts = starting_wts

    # Optimizes sim first, then nn
    def optimize_both(self):
        self.optimize_sim()
        self.optimize_nn()

    # Optimizes the current sim_args using vanilla EA
    #   FIXME: stopping condition
    def optimize_sim(self):
        num_good_transfers = 0
        args_population = np.array([util.args_mutate(self.sim_args) for i in range(c.sim_popSize)])
        while True:
            transfer_move, estimates = util.controversial_move(args_population)
            real_vec = util.real_move(transfer_move)
            fitnesses = np.array([util.mapped_cosine_similarity(real_vec, e) for e in estimates])

            # if the average fitness is already good without optimization, this is considered a good transfer
            if np.mean(fitnesses) >= c.sim_fitness_threshold:
                num_good_transfers += 1
                # if 3 consecutive transfers have been good, return the best
                if num_good_transfers >= 3:
                    self.sim_args = args_population[np.argmax(fitnesses)]
                    return
                else:
                    continue
            else:
                num_good_transfers = 0

            while np.mean(fitnesses) < c.sim_fitness_threshold:
                for i in range(c.sim_popSize):
                    child_args = util.args_mutate(args_population[i])
                    child_fitness = util.mapped_cosine_similarity(real_vec, util.sim_move(child_args, transfer_move))
                    if child_fitness > fitnesses[i]:
                        args_population[i] = child_args
                        fitnesses[i] = child_fitness

    # Optimizes the current nn_wts using CMA-ES
    #   FIXME: stop(), popsize, logger, plot
    def optimize_nn(self):
        es = cma.CMAEvolutionStrategy(self.nn_wts, c.nn_cma_sigma)
        # while not es.stop():
        for i in range(10):
            wts_population = es.ask()
            es.tell(wts_population, [util.eval_nn(self.sim_args, wts) for wts in wts_population])
            # es.logger.add()
            es.disp()
        self.nn_wts = es.best.x
        # es.logger.disp([-1])
        # es.logger.plot()


# # Tests the fitness of the current sim_args
# #   fitness is averaged across c.num_sim_tests random moves
# def test_sim(self):
#     random_moves = np.random.uniform(c.joint_lo, c.joint_hi, (c.num_sim_tests, 12))
#     fitness_sum = 0
#     for move in random_moves:
#         sim_vec = util.sim_move(self.sim_args, move)
#         real_vec = util.real_move(move)
#         fitness_sum += util.mapped_cosine_similarity(sim_vec, real_vec)
#     return fitness_sum / c.num_sim_tests
