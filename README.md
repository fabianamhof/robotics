## Robotics project
# Configure Gripper with RBT and estimate the table velocity

### Running the RBT
In order to run RBT do the following steps:
1. Open coppeliaSim and load the file "RotatingTable_Simulation_new.ttt"
2. Start the simulation in coppeliaSim
3. Open the file rbt-framework/python/gui/guiMain.py
4. Load the tree basicTreeProject.json from the folder rbt-framework/python/json
5. Start the execution of the tree

If you want to see a visual representation of the tree open the rbt-framework/python/treeviewer.py file afterwards.

### Reset the RBT

If a failure occurs, or you just want to restart the RBT please close guiMain, then follow the steps from before

### Known issues with estimating the gripper velocity
Since we are estimating the end-effector velocity by moving it and measuring the time it takes to move it there are some problems.
1. There are many overheads that can not be considered (communication, inverse kinematics,...). Because of that the calculated arm velocity is not as exact as we wish, it could be that the gripper does not grab the cube exactly in the middle.
2. Changing the simulation velocity after starting the RBT obviously leads to problems since the calculated arm velocity isn't right anymore.

### Comments for the RBT framework
We have changed the RBT framework a little for our needs. Now you can specify the type of the ROS Topic in the topics.json file.
In order to be able to load our Trees (pickUpTree.json, initTree.json,...) you first have to load the basicTreeProject.json, otherwise the rbt-framework throws an error.