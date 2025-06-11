from robomaster import robot

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_robot.set_robot_mode(mode="chassis_lead") 