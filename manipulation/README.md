# Manipulation
This assignment will show you how to do forward and inverse kinematics for higher dimensional arms, using code.

## Assignment 2
Ensure all your dependencies are installed by going to your `catkin_ws` folder and running `rosdep install --from-paths src --ignore-src -r`.

Ensure you can run the simulation by running `roslaunch manipulation tabletop.launch`. You should see a robot arm on a table that has some items on it.

In the Gazebo window, press the play button to allow rviz to load.

Some useful tips:
- Try adding a `TF` component in rviz to see all the frames. You can hide/show particular frames using the checkboxes under `Frames`.
- You can move the arm in moveit by dragging the sphere and pressing `Plan and Execute`.
- You can hide parts of the moveit display, like the sphere or the planned path, by expanding `Planning Request` or `Planned Path` and toggling the checkboxes.

Your task is to complete all the incomplete functions in `scripts/tabeltop.py`.

Any function with `COMPLETE` in the docstring does not need to be and should not be edited.

There is test code for the forward kinematics in the main function. If you run the script with `rosrun manipulation tabletop.py` it will run this test code. Think about any test code you'd like to add for the inverse kinematics to help verify your solution.
