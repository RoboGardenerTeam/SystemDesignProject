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
  - set your environment variables as described [here](https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python#environment-variables)
  - open a world in Webots with a 6-wheel robot
  - make sure the robot controller in Webots is set to \<extern\>
  - save and reload the world, make sure it is not paused
  - in a terminal, go to the SystemDesignProject/rest_controller directory, and call `python3 rest_interface.py`
  - using [Postman](https://www.postman.com/downloads/) or web browser, call http://127.0.0.1:5000/start then to stop, call http://127.0.0.1:5000/pause
    -  We also now have dummy functions /stop (return to base), /battery, /baseFill, /status
  - You can use Ngrok to tunnel the local ip address to an external address, allowing you to call the functions remotely over the internet. To do this, `cd /path/to/SystemDesignProject/rest_controller`, then run `./ngrok http 5000`. The terminal will tell you which online address it has assigned. For example, if it assigns 1235afg2354523.ngrok.io, you can call the battery function using 1235afg2354523.ngrok.io/battery
