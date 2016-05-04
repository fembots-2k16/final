# to do
- [x] fix transform for fake amcl
- [x] write transform for apriltag for homework5 explanation
- [x] solve restarting exploration (by dummying an interrupt on simulation)
- [x] get simulation to work with two robots... (kind of)
- [ ] dummy out a finder that works with another robot
- [ ] related work (read and summarize academically)
- [ ] results from first iteration (get data? time to find?? distance/efficiency of hiding??)
- [ ] finesse hiding/finding algorithm in simulation and get new results?
- [ ] finish writing paper

# simulation setup

<!-- 1. Install pioneer gazebo and stuff

    http://web.engr.oregonstate.edu/~chungje/Code/Pioneer3dx%20simulation/ros-indigo-gazebo2-pioneer.pdf -->

2. Install fembots-final

    $ cd ~/catkin_ws/src

    $ git clone https://github.com/fembots-2k16/final

    $ sudo mv final fembots

    $ cd ~/catkin_ws

    $ catkin_make

    $ source ~/catkin_ws/devel/setup.bash

--------------------------------------------------------------------------------

# simulation run

1. run the stage simulation

    $ cd ~/catkin_ws/src/fembots

    $ ./run_simulation

2. run the hider or finder nodes??

    $ rosrun fembots hider_sim.py

    $ rosrun fembots finder_sim.py

--------------------------------------------------------------------------------

# real-life run

--------------------------------------------------------------------------------

# paper
https://docs.google.com/a/crimson.ua.edu/document/d/1Fq3d9fZN-58wJUKkR7wnKpp-wr10OOZoWVD2SqoR1-E/edit?usp=sharing_eid&ts=571fdd31

# presentation
https://docs.google.com/a/crimson.ua.edu/presentation/d/1GbshE_gfKpy-EX5ZntbKaycoqi5rux2k4jiswZ6wmkU/edit?usp=sharing_eid&ts=571fd696
