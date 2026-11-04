# Dynamic Motion Planning for Transporting Bulky Objects in Constrained Construction Environments

This repository contains the software, simulation assets, configuration files, and supporting tools developed for a master's thesis on dynamic motion planning for mobile robots transporting bulky objects in constrained construction environments.

The project focuses on robot-based object transport tasks such as moving wall panels, beams, or other prefabricated construction components through narrow, obstacle-rich environments while considering static and dynamic obstacles.

## Thesis Context

Mobile robots used in construction must often transport objects that are large relative to the available free space. This creates motion-planning challenges when the robot must move through constrained passages such as corridors, scaffolding, temporary site layouts, or partially built structures.

The thesis investigates dynamic motion-planning methods that support safe and feasible transport of bulky objects using onboard sensing and robot motion planning.

## Objectives

The repository supports the following thesis objectives:

1. Review existing motion-planning approaches for robots transporting bulky objects in construction environments.
2. Design a methodology for dynamic motion planning in constrained construction scenarios.
3. Implement a motion-planning framework based on the developed methodology.
4. Validate the framework through simulated transport experiments.
5. Analyze the results and evaluate the performance, strengths, and limitations of the implemented framework.

## Scope

The project is intended to support research and experimentation related to:

- mobile robot navigation in constrained construction-like environments
- transport of bulky or extended objects
- static and dynamic obstacle avoidance
- onboard sensing for environment perception
- simulation-based validation
- integration of navigation, manipulation, and motion-planning components

## Repository Structure

## Development Environment

The project targets the following software environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- MoveIt 2
- Nav2
- RViz2

## Getting Started

### Open in Dev Container

```text
Dev Containers: Reopen in Container
```

### Build the Workspace

Inside the development environment:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Clean Build

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Running the System

## Documentation

Documentation is developed alongside the implementation. Relevant documentation may include:

- system architecture
- package descriptions
- simulation setup
- experiment setup
- parameter documentation
- evaluation procedure
- known limitations

Documentation files should be placed in the `docs/` directory where appropriate.

## Development Notes

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

## Acknowledgement

This repository is developed as part of a master's thesis in the field of robotics, motion planning, and construction automation.
