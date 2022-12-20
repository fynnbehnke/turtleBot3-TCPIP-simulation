### Advanced Programming for Robots - TurtleBot Simulation

## Installation

- Install ROS on your Workstation (preferably ROS-Noetic / Ubuntu 20.04).

     Follow this [Tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu) (Install the Desktop-Full-Version for Gazebo).


-  Create, Build and Source a new ROS Workspace

    ```BASH
    mkdir -p ~/apr_ws/src
    cd ~apr_ws
    catkin_make
    source ~/apr_ws/devel/setup.bash
    ```

- Navigate to the src folder of your ROS-Workspace and clone the [git repository](https://github.com/fynnbehnke/mre1-APR-Simulation.git)
    
    ```BASH
    cd ~/apr_ws/src
    git clone https://github.com/fynnbehnke/mre1-APR-Simulation.git
    ```

- After that Build and Source once again
    
    ```BASH
    catkin_make
    source ~/apr_ws/devel/setup.bash
    ```


## Usage

- To run the Simulation simply launch the start_sim launch-file of the apr_sim package
    ```BASH
    roslaunch apr_sim start_sim.launch
    ```