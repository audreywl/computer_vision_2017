# Path Follower
### Katie Butler and Audrey Lewis, computer vision project 2017


## Project Goal
Our goal was to have a neato robot analyze a curved path in a human-drawn image and replicate it by moving along the path in real life.

## What Works


## Design Decision/Challenges
Early in the project, we decided we wanted to execute the path by driving along a parametric equation that descibed the path, instead of discreetly going from point to point. This led to several other decisions - for example, because we didn't want to be constrained to drawing functions, we didn't use a polynomial fitting algorithm, and instead split up our path into multiple curves, each of which we fit an ellipse to. This...didn't work. Our error minimization function for the ellipse fitting was so finicky that we just ended up having to draw ellipse-like paths.


## Code Structure
We structured our code in two python scripts. We organized fit_curve.py into two classes, Ellipses and Fit_Curve, that were imported into curve_follower.py, used a finite state controller class Follower that subscribed and published messages to the robot. Follower was made up of the state objects wait, analyze, and drive that run based on which state of the robot, waiting, analyzing, or driving, is currently True. If the bump sensor is triggered, the robot goes out of bounds, or the program is exited, an emergency stop function fucking_stop ran to stop and shutdown the robot

The Fit_Curve objects were called in the analyze state function in curve_follower's main class Follower to process camera images into contours that were cleaned up to leave one continuous array of points along a curve to pass into the Ellipse class. The objects of Ellipse fit the points to an ellipse. That Ellipse object get_velocities was called in the drive state function in curve_follower to calculate the linear and angular velocities needed to drive the path of the curve.


## What would you do to improve your project if you had more time?
If we had more time, we would debug more around what makes the ellipse fitting so bad, and maybe choose a different equation fitting algorithm

## Lessons Learned


