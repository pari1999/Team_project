from robomaster import robot

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_robot.set_robot_mode(mode="chassis_lead") 
ep_chassis = ep_robot.chassis
ep_camera = ep_robot.camera
ep_gripper = ep_robot.gripper
ep_camera.start_video_stream(display=False)
