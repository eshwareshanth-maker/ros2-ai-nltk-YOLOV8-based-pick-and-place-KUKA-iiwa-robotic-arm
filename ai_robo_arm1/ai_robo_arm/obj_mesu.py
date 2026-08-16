#=================================================================
#Author : Eshanth Eshwar M
#email : eshwareshanth@gmail.com
#=================================================================


import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from ultralytics import YOLO


from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

latest_msg = None


fx = 920      # focal length x (pixels)
fy = 920      # focal length y

cx = 320      # principal point
cy = 240



OBJECT_WIDTHS = {
    "person": 450,
    "bottle": 70,
    "cup": 80,
    "cell phone": 75,
    "mouse": 65,
    "book": 150
}



rclpy.init()
node = rclpy.create_node('obj_mesu')
pub = node.create_publisher(Float32MultiArray, '/mesure', 10)


model = YOLO("yolov8n.pt")

cap = cv2.VideoCapture(0)

def listener_callback(msg1):
    global latest_msg
    latest_msg = msg1
    node.get_logger().info(f'Processing: "{latest_msg.data}"')

node.create_subscription(String, 'chatter', listener_callback, 10)

while True:

    rclpy.spin_once(node, timeout_sec=0)   # <-- added: without this, listener_callback never fires

    ret, frame = cap.read()
    

    if not ret:
        break

    results = model(frame)

    annotated = frame.copy()

    for r in results:

        boxes = r.boxes

        for box in boxes:

            cls = int(box.cls[0])

            name = model.names[cls]

            if name not in OBJECT_WIDTHS:
                continue

            if latest_msg is not None:
             print(latest_msg.data)
             
             if name == latest_msg.data:
              x1, y1, x2, y2 = map(int, box.xyxy[0])

              width_pixels = x2 - x1

              if width_pixels <= 0:
                 continue

             # Center pixel

              u = (x1 + x2) / 2
              v = (y1 + y2) / 2

              real_width = OBJECT_WIDTHS[name]

              # Depth (mm)

              Z = (fx * real_width) / width_pixels

              # Real world coordinates

              X = ((u - cx) * Z) / fx
              Y = ((v - cy) * Z) / fy

              # Convert to meters

              X /= 1000
              Y /= 1000
              Z /= 1000

              print("--------------------------------")
              print(name)
              print("Pixel :", int(u), int(v))
              print("X =", round(X,3),"m")
              print("Y =", round(Y,3),"m")
              print("Z =", round(Z,3),"m")




              target = [X, Y, Z]
              tar_msg = Float32MultiArray()

              tar_msg.data = [float(t) for t in target]
              pub.publish(tar_msg)
              node.get_logger().info(f'publishing "{tar_msg.data}"')


              cv2.rectangle(
                 annotated,
                 (x1,y1),
                 (x2,y2),
                 (0,255,0),
                 2
                  )

              cv2.circle(
                annotated,
                (int(u),int(v)),
                5,
                (0,0,255),
                -1
                )

              text = f"{name}"

              cv2.putText(
                annotated,
                text,
                (x1,y1-40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255,255,0),
                2
                )

              coord = f"X:{X:.2f}m"

              cv2.putText(
                annotated,
                coord,
                (x1,y1-20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,255),
                2
                )

              coord = f"Y:{Y:.2f}m"

              cv2.putText(
                annotated,
                coord,
                (x1,y1),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,255),
                2
              )

              coord = f"Z:{Z:.2f}m"

              cv2.putText(
                annotated,
                coord,
                (x1,y2+20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0,255,255),
                2
               )

    cv2.imshow("3D Object Coordinates", annotated)

    if cv2.waitKey(1) == 27:
        break


node.destroy_node()
rclpy.shutdown()

cap.release()
cv2.destroyAllWindows()
