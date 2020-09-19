# SOCS-Project

Run 

    bash run_simulation.sh ${paramset}

to get a test flight running. (Try for example paramset=11)

This will show show both the graphs for visualization and a 3D model of a drone (Shoutout to Jan Schiffeler, [@Platygator]( https://github.com/Platygator ), for the visualization).

Alternatively, to have a slightly quicker simulation but without the 3D model, run

    bash run_simulation_without_viz.sh ${paramset}

This will start a simulation where a drone collects coins successively in
predefined positions. Each coin will earn the drone a reward.
Different reward strategies have been tried, for now - to encourage the drones to stay in the air - each passed time step yields a little bit of a positive reward towards the fitness score.

## Learning
Possible future approaches might include reinforcement learning or weight agnostic networks.

So far, a neuroevolutionary algorithm has been implemented.

### Neuroevolution
#### Training
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

Another thing to explore could be pretraining the drone's brain.
For this, one could use random coin + drone positions at one single time step and let the old PID approach compute a ground truth for each prediction. This could also calculate training points on the fly (no pun intended) which in turn emulates an infinite training set.

This pretrained network could then be used as an initial network for the drones to run the neuroevolution algorithm more efficiently.

### Visualize Results

To see the best drone of a certain training run in action, execute

    bash show_best_drone.sh ${execution_time}

The execution time will also be printed at the end of one successful training run as reference point for this.
    

## Test Framework
You can write unit tests for your code by placing files of test classes in the ./test directory.

You can take `test/test_sensor.py` as an example for the correct syntax.

Then simply execute the script `run_tests.sh` (or manually execute `python -m pytest . --cov ./py --cov-report html`)
