# SOCS-Project

Run 

    bash run_simulation.sh ${paramset}

to get a test flight running.
Atm it is set up to only show the graphs for visualization and no 3D model.

This will start a simulation where a drone collects coins successively in
predefined positions. Each coin will earn the drone a reward, the reward score will decrease with each passed time step.

As of now, there is no real use of the rewards system.
Next steps would be to implement a training routine to make use of the reward.


## Test Framework
You can write unit tests for your code by placing files of test classes in the ./test directory.

You can take `test/test_sensor.py` as an example for the correct syntax.

Then simply execute the script `run_tests.sh` (or manually execute `python -m pytest . --cov ./py --cov-report html`)
