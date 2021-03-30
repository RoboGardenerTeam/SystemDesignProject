## How to run the demo

1. Make sure your Webots is using python 3.8
    - 3.9 does **not** work (at least on Mac)
    - I think 3.7 might also work, not sure
    - For Mac users:
        - `brew install python@3.8`
        - This means setting the `Python command` setting in Webots preferences to `/usr/local/Cellar/python@3.8/3.8.8_1/Frameworks/Python.framework/Versions/3.8/bin/python3.8` (make sure this is actually where your python binary lives! this is just the one I use, might/probably will be different for you)
2. Make sure flask and numpy are installed
    - Lots of ways for this to go wrong (spent lots of time banging my head against my desk figuring this out) because Webots may use a differnt python than the one that you use on the command line
    - Navigate to wherever the python binary you specified in the Webots settings above lives (in this case `/usr/local/Cellar/python@3.8/3.8.8_1/Frameworks/Python.framework/Versions/3.8/bin`), and run `./python3.8 -m pip install numpy` and `./python3.8 -m pip install flask` 
3. Open and run the desired demo world
4. Follow the README at https://github.com/RoboGardenerTeam/RoboGardenerWebsite to open and run the UI to control the robot
