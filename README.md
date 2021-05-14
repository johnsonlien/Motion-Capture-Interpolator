# CSCI 520 Homework 2 Motion-Capture-Interpolator


This program takes in a skeleton file (.asf) and a motion file (.amc) and interpolates a model's movement using two techniques:
1. Linear Interpolation
2. Bezier Splines

# How to use
## Interpolator
When running the program, the command line arguments for "interpolater.cpp" go in this format:
 - skeleton file (.asf)
 - motion file (.amc)
 - interpolation technique (either l or b for linear / bezier, respectively)
 - angle (either e or q for euler / quaternion, respectively)
 - # for keyframe intervals
 - output motion file
This outputs the interpolated motion that you can view using "mocapPlayer.cpp"

## Player
This program uses OpenGL to provide the user a interface to play the interpolated motion.
Run "mocapPlayer" and load in the skeleton file and motion you want to view. Then press the 'play' button. 
