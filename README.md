# SOCS-Project

Run 

    bash run_simulation.sh

to get a test flight running.
Atm it is set up to only show the graphs for visualization and no 3D model.

## Test Framework
You can write unit tests for your code by placing files of test classes in the ./test directory.

You can take `test/test_sensor.py` as an example for the correct syntax.

Then simply execute the script `run_tests.sh` (or manually execute `python -m pytest . --cov ./py --cov-report html`)
