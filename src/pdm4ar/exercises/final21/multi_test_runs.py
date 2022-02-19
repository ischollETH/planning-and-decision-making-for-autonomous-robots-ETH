import time

from dg_commons.sim.simulator import Simulator

from pdm4ar.exercises.final21.debug_tools import context_generator
from pdm4ar.exercises_def import get_sim_context_static

NUMBER_OF_RUNS = 10

# No obstacle test run
if False and __name__ == '__main__':
    total_time = 0
    for i in range(NUMBER_OF_RUNS):
        print(f'Run {i} of {NUMBER_OF_RUNS}')
        seed = i
        sim_context = context_generator(seed, noObstacles=True)
        sim = Simulator()
        time_start = time.time()
        sim.run(sim_context)
        time_end = time.time()
        total_time += time_end - time_start

    print(f'NO OBSTACLES : TOTAL TIME : {total_time}, AVG. : {total_time / NUMBER_OF_RUNS} ')

# Static obstacle test run
if True and __name__ == '__main__':
    total_time = 0
    for i in range(NUMBER_OF_RUNS):
        print(f'Run {i} of {NUMBER_OF_RUNS}')
        seed = i
        sim_context = get_sim_context_static(seed)
        sim = Simulator()
        time_start = time.time()
        sim.run(sim_context)
        time_end = time.time()
        total_time += time_end - time_start

    print(f'STATIC OBSTACLES :  TOTAL TIME : {total_time}, AVG. : {total_time / NUMBER_OF_RUNS} ')
