import cv2
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_img = cv2.aruco.drawMarker(aruco_dict, id=1, sidePixels=200)
cv2.imwrite("aruco_id1.png", marker_img)