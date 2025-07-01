import cv2
import numpy as np
import serial
import time
import depthai as dai

class ColorObjectDetector:
    def __init__(self):
        self.command_delay = 2
        self.last_command_time = 0
        self.red_counter = 0
        self.yellow_counter = 0
        self.green_counter = 0
        self.detection_threshold = 5

        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            print("Arduino connected.")
        except Exception as e:
            print(f"Arduino not connected - detection-only mode. ({e})")
            self.arduino = None

        self.current_command = None
        self.awaiting_done = False


    def send_command(self, color):
        if self.awaiting_done:
            return  # wait for DONE before sending new command

        current_time = time.time()
        if current_time - self.last_command_time > self.command_delay:
            if self.arduino:
                try:
                    self.arduino.write(f"{color.upper()}\n".encode())
                    print(f"Sent to Arduino: {color}")
                    self.current_command = color.upper()
                    self.awaiting_done = True
                except serial.SerialException as e:
                    print(f"Arduino communication error: {e}")
                    self.arduino = None
            self.last_command_time = current_time

    def check_arduino_done(self):
        if not self.arduino or not self.awaiting_done:
            return
        while self.arduino.in_waiting:
            response = self.arduino.readline().decode().strip()
            if response:
                print(f"[Arduino] {response}")
            if response == "DONE":
                print("Received DONE from Arduino")
                self.awaiting_done = False
                self.current_command = None

    def detect_colors(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 100])
        upper_red2 = np.array([180, 255, 255])

        lower_yellow = np.array([22, 150, 150])
        upper_yellow = np.array([32, 255, 255])
        
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)


        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)


        return mask_red, mask_yellow ,mask_green

    def find_and_draw_contours(self, frame, mask, color_name, box_color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                cv2.putText(frame, color_name.upper(), (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
                detected = True
        return detected

    def run(self):
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(640, 480)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("video")
        cam.preview.link(xout.input)

        with dai.Device(pipeline) as device:
            video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)

            print("Starting color-based detection. Press 'q' to quit.")
            while True:
                in_frame = video_queue.get().getCvFrame()
                mask_red, mask_yellow ,mask_green = self.detect_colors(in_frame)

                red_detected = self.find_and_draw_contours(in_frame, mask_red, "red", (0, 0, 255))
                yellow_detected = self.find_and_draw_contours(in_frame, mask_yellow, "yellow", (0, 255, 255))
                green_detected = self.find_and_draw_contours(in_frame, mask_green, "green", (0, 255, 0))

                self.check_arduino_done()

                if red_detected:
                    self.red_counter += 1
                else:
                    self.red_counter = 0

                if yellow_detected:
                    self.yellow_counter += 1
                else:
                    self.yellow_counter = 0
                    
                if green_detected:
                    self.green_counter += 1
                else:
                    self.green_counter = 0

                if self.red_counter >= self.detection_threshold and not self.awaiting_done:
                    if self.current_command != "RED":
                        self.send_command("RED")

                if self.yellow_counter >= self.detection_threshold and not self.awaiting_done:
                    if self.current_command != "YELLOW":
                        pass
                        #self.send_command("YELLOW")
                        
                if self.green_counter >= self.detection_threshold and not self.awaiting_done:
                    if self.current_command != "GREEN":
                        self.send_command("GREEN")

                cv2.imshow("Color Detection", in_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        if self.arduino:
            self.arduino.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    detector = ColorObjectDetector()
    detector.run()
