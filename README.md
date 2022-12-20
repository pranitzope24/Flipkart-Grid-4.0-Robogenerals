# Flipkart Grid 4.0

## Working on this project
* Have ardupilot and its plugins installed
```bash
mkdir -p drone_ws/src
cd drone_ws
catkin init
cd src

echo ************* PLS NOTE THAT ****************
echo ********** RUN MANUALLY IF GITHUB AUTHENTICATION ASKED ***********

git clone https://github.com/pranitzope24/Flipkart-Grid-4.0-Robogenerals.git
mv Flipkart-Grid-4.0-Robogenerals simulation
catkin build
```

## Plan

* Step 0 : Find the 1st box (since we cannot see the entire) make path for this

* Step 1 : Takeoff, find 1st marker, go to that marker and lower your position to a certain altitude.

* Step 2 : Take that marker to the land drop zone (aruco or normal setpoint)

* Step 3 : Go back and detect the 2nd box and lower

* Step 4 : Srop in drop off zone

* Step 5 : RTL