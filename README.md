# Vizzy-mask-patrol

In this project, a random patrol sequence is created for Vizzy, a humanoid robot. I used python and behaviour trees to modulate the sequence and it consists of the selection of random coordinates from a list of rooms (from a pre-existing model of the floor), the movement of the robot to that room, and the detection of people in the room. If Vizzy identified a face, it would run a mask detection software (not developed by me) and, in case the person was not wearing a mask, it would tell them to put one on. This software was tested only in simulation.
