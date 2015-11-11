# njllrd_proj2
MAE 5750 Project 2 Richard Dunphey, Naveen Jasty, Lisa Li

SYMBOLIC SIMULATOR

REAL ROBOT

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
  - run the launch file
  - move Baxter's arm to the first desired point
  - In a new terminal, connect to Baxter. From the ros_ws directory, publish a message to the "user_input" topic containing any string. Enter: "rostopic pub command std_msgs/String ANY_STRING_HERE".
  - Move Baxter's arm to the second desired point
  - Publish another message containing any string to the user_input topic
  - Baxter should move in a straight line
  
Typewriter:
  - run the launch file
  - Define the drawing plane
    - place Baxter's arm such that the marker tip is touching the board
    - From the ros_ws directory, publish a message to the "user_input" topic containing any string. Enter: "rostopic pub command std_msgs/String ANY_STRING_HERE".
    - Repeat at two other points
  - type any lowercase letter and hit enter
  - type "return" and hit enter to move to a new line
  - type "space" and hit enter to "type" an empty space
  - To exit, type "end" hit enter and hit Ctrl+c
  
Draw
  - run the launch file
  - Define the drawing plane as in Typewriter mode
  - type "pyramids" and hit enter to draw a picture of pyramids
  - type "maze" and hit enter to draw and solve a maze
  - To exit, type "end" hit enter and hit Ctrl+c
  
RRT
  - run the launch file
  ....
  
BIRRT
  - run the launch file 
  .....
  
Notes:
  - Orienting the pen at 45 degrees for writing works best. 
  - The control gain can be adjusted my changing kp in robot_interface.py
