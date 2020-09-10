# SOCS-Project

Run 

    bash run_simulation.sh ${paramset}

to get a test flight running. (Try for example paramset=11)

This will show show both the graphs for visualization and a 3D model of a drone.

This will start a simulation where a drone collects coins successively in
predefined positions. Each coin will earn the drone a reward, the reward score will decrease with each passed time step.

As of now, there is no real use of the rewards system.
Next steps would be to implement a training routine to make use of the reward.

## Neuroevolution
### Training
Executing

    python evolve/evolution.py

starts a simple neuroevolution algorithm to train and evolve the controller unit within the drones of a population.

Current objective is to collect a coin and/or stay in the air for as long as possible.

_Status Quo:_ Training is enabled, but the training is not very successful as of yet.
Some things can be improved, such as:
* Parallelization of fly() function
* Training strategy:
    - Tune hyperparameters such as mutation rate, crossover rate, ...
    - Tune rewards for time in air, distance to coin, coin collected, crashed, and so on

### Result

To see the best drone of a certain training run in action, execute

    bash show_best_drone.sh ${execution_time}
    

## Test Framework
You can write unit tests for your code by placing files of test classes in the ./test directory.

You can take `test/test_sensor.py` as an example for the correct syntax.

Then simply execute the script `run_tests.sh` (or manually execute `python -m pytest . --cov ./py --cov-report html`)
