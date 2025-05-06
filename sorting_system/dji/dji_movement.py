import time
def robomaster_move(ep_robot):

    # Get chassis and arm modules
    chassis = ep_robot.chassis

    #Move forward
    print("Moving forward...")
    chassis.move(x=3, y=0, z=0, xy_speed=1).wait_for_completed()
    time.sleep(1)

    # Rotate right (clockwise) 90°
    print("Turning RIGHT 90 degrees")
    chassis.move(x=0, y=0, z=-90, xy_speed=1).wait_for_completed()
    time.sleep(1)

    #Move forward
    print("Moving forward...")
    chassis.move(x=3, y=0, z=0, xy_speed=1).wait_for_completed()
    time.sleep(1)

    #Move backward
    print("Moving backward...")
    chassis.move(x=-3, y=0, z=0, xy_speed=1).wait_for_completed()
    time.sleep(1)

    # Rotate right (clockwise) 90°
    print("Turning RIGHT 90 degrees")
    chassis.move(x=0, y=0, z=-90, xy_speed=1).wait_for_completed()
    time.sleep(1)

    #Move forward
    print("Moving forward...")
    chassis.move(x=3, y=0, z=0, xy_speed=1).wait_for_completed()
    time.sleep(1)

    #Close connection
    ep_robot.close()
    print("Finished.")