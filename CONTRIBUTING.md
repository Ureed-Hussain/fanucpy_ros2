# Contributing

Contributions should preserve the project's central rule: exactly one driver
process owns the TCP connection to a robot controller.

Before submitting a change:

1. Work outside Conda and source ROS 2 Humble.
2. Add or update tests without requiring physical hardware.
3. Run `rosdep install --from-paths src --ignore-src -r -y`.
4. Run `colcon build --symlink-install`.
5. Run `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test`.
6. Run `colcon test-result --verbose`.
7. Document every ROS interface, parameter, unit, frame, and controller-side
   resource it uses.

Real-robot testing is useful evidence, but it never replaces offline tests and
is not expected from outside contributors. Describe real-robot tests, active
safety gates, controller version, and robot model in the pull request without
publishing the controller address or site network details.

Never commit robot credentials, private network configuration, generated
`build/`, `install/`, or `log/` directories.
