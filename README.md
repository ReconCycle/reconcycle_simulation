# reconcycle_simulation

Package to simulate the Reconcycle cell. Currently only simulates the Panda robots.

# Install

* Cone the repository to your ROS workspace:
`$ git clone https://github.com/ReconCycle/reconcycle_simulation`

* Use `wstool` to get all of the other dependencies:
```
$ wstool init
$ wstool merge reconcycle_simulation/dependencies.rosinstall
$ wstool up
```
The cloned dependencies should show up in your `src` directory.

* Query and install all libraries and packages:
```
$ rosdep install --from-paths . --ignore-src --rosdistro kinetic
```

* Catkin build:
```
$ cd ..
$ catkin build
```

# Usage

```
$ roslaunch reconcycle_simulation simulate_r obot.launch rviz:=true
```

# Tricks and tips