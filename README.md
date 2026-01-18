### Group 31
  - Alban HOUEL, albanhervejoel.houel@studenti.unipd.it
  - Kavita SINGLA, kavita.singla@studenti.unipd.it
  - Franck GUILLOTEAU, franckchristiangabriel.guilloteau@studenti.unipd.it

### Assignment
The assignment requires us to make a turtle robot move through a simulated map. The map consists of a narrow corridor which leads to an open room with a large wall in the middle of it. The final destination
of the robot has multiple obstacles near it, those being a table's legs and two apriltags.
The goal of the robot is to move to a position at the midpoint of the two apriltags, then detect the table legs around it using its Lidar. It must then report the absolute position of the obstacles. It is
important to note that the robot must ignore other obstacles and must only print the position of the table legs, not of the walls of the map.

The assignment requires us to make a robotic arm anchored to the ground swap the position of two cubes disposed on two different tables in front of it. The cubes have tags in front of them to help the arm
locate them and pick them up. An external camera is also present at the scene to help read thoe apriltags.
The goal is to first use the camera to get the coordinates of the cubes, convert them to the world space before finally sending them to the robot that needs to process those coordinates and move the two
cubes accordingly.

### How to run
In your ROS2 workspace containing the package of the similation, move to the "src" file and clone the github repository using this command : "git clone https://github.com/guilloteaufranck21/Assignment-2".
This will create a package "Assignment-2" in your "src" folder.

#### Warning : you need to take the two nodes "apriltag_detector" and "tag_transformer" and put them in a separate package called "assignment" for them to work properly, otherwise they won't.

After building your workspace correctly, in a first terminal :

  - Move to the root folder of your workspace
  - Source your environment using "source install/setup.bash"
  - Run the simulation and the first two nodes by using "ros2 launch group31_assignment_2 assignment_2_launch.launch.py"

In a second terminal, after the first terminal starts to show the two coordinates of the cubes (roughly 30 seconds after launch) :
  - Move to the root folder of your workspace
  - Source your environment using "source install/setup.bash"
  - Run the arm movement node by using "ros2 run group31_assignment_2 move_arm".

The arm should first pick up the blue cube, move to the red one's table, put it down, pick up the red cube before finally moving back to the blue cube's table.
