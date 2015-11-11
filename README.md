# njllrd_proj2
MAE 5750 Project 2 Richard Dunphey, Naveen Jasty, Lisa Li

How to run:
  - Clone the project with https://github.com/navjas73/njllrd_proj2.git into the ros_ws/src folder
  - Make sure Baxter is on and powered up if using the real robot. Next, in a new terminal, connect to Baxter with the command : ". baxter.sh" or ". baxter.sh sim" if using the simulator from within the ros_ws directory
  - From the ros_ws directory, enable the robot with the command: "rosrun baxter_tools enable_robot.py -e"
  - Navigate to the ros_ws/src/njllrd_proj2/launch directory
  - Run the launch file with the command: "roslaunch proj2.launch"

How to select a mode:

- In the launch file: proj2.launch, change the mode parameter to 
  - connect_points: connects two points in a straight line
  - typewriter: allows user to input letters which Baxter will type
  - draw: Baxter will draw pyramids or draw and solve a maze
  - RRT: Baxter will use an RRT to navigate around obstacles
  - BIRRT: Baxter will use a bi-directional RRT to navigate around obstacles
  
Connect_points:
  - run the launch file with mode set to connect_points
  - move Baxter's arm to the first desired point
  - In a new terminal, connect to Baxter. From the ros_ws directory, publish a message to the "user_input" topic containing any string. Enter: "rostopic pub command std_msgs/String ANY_STRING_HERE".
  - Move Baxter's arm to the second desired point
  - Publish another message containing any string to the user_input topic
  - Baxter should move in a straight line
  
Typewriter:
  - run the launch file with mode set to typewriter
  - Define the drawing plane
    - place Baxter's arm such that the marker tip is touching the board
    - From the ros_ws directory, publish a message to the "user_input" topic containing any string. Enter: "rostopic pub command std_msgs/String 'ANY_STRING_HERE'".
    - Repeat at two other points
  - type any lowercase letter and hit enter
  - type "return" and hit enter to move to a new line
  - type "space" and hit enter to "type" an empty space
  - To exit, type "end" hit enter and hit Ctrl+c
  
Draw
  - run the launch file with mode set to draw
  - Define the drawing plane as in Typewriter mode
  - type "pyramids" and hit enter to draw a picture of pyramids
  - type "maze" and hit enter to draw and solve a maze
  - To exit, type "end" hit enter and hit Ctrl+c
  
RRT
  - run the launch file with mode set to RRT
  - move Baxter to the first desired point
  - From the ros_ws directory, publish a message to the "user_input" topic containing any string. Enter: "rostopic pub command std_msgs/String ANY_STRING_HERE".
  - move Baxter to the second desired point
  - publish another string as above
  
BIRRT
  - run the launch file with mode set to BIRRT
  - repeat steps in RRT
  
Notes:
  - Orienting the pen at 45 degrees for writing works best. 
  - The control gain can be adjusted my changing kp in robot_interface.py

Objective Function:
  - An objective function is used to avoid joint limits and singularities. This function calculates the manipulability of a given configuration, and then uses a small angle step for each joint to calculate the change in manipulability for a small movement in each joint. If the manipulability increases a lot, a high magnitude, positive change in the objective function is observed. If the manipulability decreases a little, a low magnitude, negative change in the objective function is used. These changes in manipulability (definition of a derivative of manipulability) directly correspond to the b-value for the given joint. The b-vector is scaled to an appropriate magnitude in order that it keeps baxter manipulable within the workspace, but does not interfere with the pen velocity (although no choice in b should affect the pen velocity, commanding very high joint velocities does impact the smoothness of our lines).

Smoothing:
  - A smoothing function is used to evaluate the path created by our RRT. We start with the path we have chosen (array of configuration waypoints), and iterate through each index of that path. For each waypoint (let's say p1), we select a random waypoint (let's say p3) between the one we are testing and the goal (let's say p3). We attempt to connect those two points, and collision check. If no collision is found, we remove un-necessary waypoints (in this case, p2) and continue to iterate through this new path array. 
