Overall state of project:
* Robot with kicking mechanism built.
* Starter code has been run successfully w/ camera and remote control of the bot working.
* Began to implement some parts of the penalty FSM in roboAI.c.

For Tuesday Nov 17 deadline:

Done:
* logo.

To do:
* Finite state diagrams for "chase the ball" mode and "play soccer" mode.
* Working penalty kick behavior.
* Working "chase the ball" behavior.

Friday Nov 13: Some functions are started in roboAI.c for penalty kick. There is various data we can access about the ball and robot. What we couldn't figure out today was how to achieve precise control of the robot's movements like we had with the first project when the code was running on the brick directly. The big thing though is figuring out a path finding strategy. We will know where we want to go and what direction we want to be pointing when we get there. If we knew how to do something like OnFwdSync from the C code on the computer, the path for a certain steering value is an arc of a circle. If we choose an arc where our desired final direction is a tangent, maybe a PID controller could work for choosing the steering value. Alternately, maybe there will be some other solution that proves easier to implement in time for the check-in.



**********************************************************************
Original readme contents:

UTSC - RoboSoccer

Will compile and run on Linux provided the OpenGL and Bluetooth libraries are
installed.

From the project's folder type:

>./configure

If any dependencies are missing, install them and re-run configure.

Once the configuration ends successfully, type

>make

The executable file will be in ./src so you can run it with

./src/roboSoccer /dev/video1 0 0

See the handout and the executable's usage doc. for the meaning of each
command line parmeter.


