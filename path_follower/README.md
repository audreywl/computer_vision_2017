    What was the goal of your project?
    Our goal was to have the robot analyze a curved path from an image and replicate it by moving along it in real life.
    
    
    Describe how your system works.  Make sure to include the basic components and algorithms that comprise your project.
    
    
    Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
    
    
    How did you structure your code?
    We structured our code in two python scripts. We organized fit_curve.py into two classes, Ellipses and Fit_Curve, that were imported into curve_follower.py, used a finite state controller class Follower that subscribed and published messages to the robot. Follower was made up of the state functions wait, analyze, and drive that ran based on which state, waiting, analyzing, or driving, was currently True. If the bump sensor was triggered, the robot went out of bounds, or the program was exited, an emergency stop function fucking_stop ran to stop and shutdown the robot.

    The Fit_Curve objects were called in the analyze state function in curve_follower's main class Follower to process camera images into contours that were cleaned up to leave one continuous array of points along a curve to pass into the Ellipse class. The objects of Ellipse attempted to fit the points to an ellipse and tested whether or not the ellipse was valid. That Ellipse object get_velocities was called in the drive state function in curve_follower to calculate the linear and angular velocities needed to drive the path of the curve.
    
    What if any challenges did you face along the way?
    
    
    What would you do to improve your project if you had more time?
    
    
    Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
