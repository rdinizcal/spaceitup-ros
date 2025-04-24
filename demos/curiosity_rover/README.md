# Curiosity Rover - Demo

This is a simple demo of controlling the curiosity rover using spaceROS.

1. To move around, you can use the `move` service. The service takes two arguments, `linear` and `angular` velocities. You can call the service using the following command.
    ```bash
    # Move forward
    ros2 service call /move_forward std_srvs/srv/Empty

    # Turn left
    ros2 service call /turn_left std_srvs/srv/Empty

    # Turn right
    ros2 service call /turn_right std_srvs/srv/Empty

    # Stop
    ros2 service call /move_stop std_srvs/srv/Empty
    ```
2. To control the arm of the rover, you can run the following services,
    ```bash
    # Open the arm
    ros2 service call /open_arm std_srvs/srv/Empty

    # Close the arm
    ros2 service call /close_arm std_srvs/srv/Empty
    ```
3. To control the mast arm of the rover, you can run the following services,
    ```bash
    # Open the mast arm
    ros2 service call /mast_open std_srvs/srv/Empty

    # Close the mast arm
    ros2 service call /mast_close std_srvs/srv/Empty

    # To rotate the mast arm
    ros2 service call /mast_rotate std_srvs/srv/Empty
    ```
