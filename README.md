# RoboGardener

- How to build the robot
https://www.youtube.com/watch?v=LEAtWFIZF50&t=1s

- How to set up sensor and change direction
https://www.youtube.com/watch?v=u-sSG-5wGTs

- How to set up camera
https://www.youtube.com/watch?v=BuQazGSChaw&t=380s

- Extend: Avoidence Algorithm
https://www.youtube.com/watch?v=l0JuUM58nOs

***

## Matt D's New Controller
To use the controller:
  - make sure the robot controller in Webots is set to \<extern\>
  - save and reload the world, make sure it is not paused
  - in a terminal, go to the SystemDesignProject/Demo\ Robot/controllers/rest_controller directory, and call `python3 rest_interface.py`
  - using [Postman](https://www.postman.com/downloads/) or web browser, call http://127.0.0.1:5000/control/run then to stop, call http://127.0.0.1:5000/control/stop
