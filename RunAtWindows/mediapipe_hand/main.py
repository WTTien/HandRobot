import cv2
import numpy as np

import socket

import mediapipe.python.solutions.hands as mp_hands
import mediapipe.python.solutions.drawing_utils as mp_drawing
import mediapipe.python.solutions.drawing_styles as mp_drawing_styles

cap = cv2.VideoCapture(0)

# DISPLAY COORDINATES
fingertip_indices = [4, 8, 12, 16, 20]

# EXAMPLE TO DISPLAY BENT DEGREE
# index_mcp = 5
# index_pip = 6
# index_dip = 7


# DISPLAY BENT DEGREE FOR EACH JOINT
thumb_indices = [2,3,4]
index_indices = [5,6,7]
middle_indices = [9,10,11]
ring_indices = [13,14,15]
pinky_indices = [17,18,19]

finger_indices = {
    "thumb": thumb_indices,
    "index": index_indices,
    "middle": middle_indices,
    "ring": ring_indices,
    "pinky": pinky_indices
}

UDP_IP = "172.21.249.248"
UDP_PORT = 5005


def calculate_angle(a, b, c):
    
    # EXAMPLE TO DISPLAY BENT DEGREE
    # AB = np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]])
    # BC = np.array([c[0] - b[0], c[1] - b[1], c[2] - b[2]])
    
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    
    AB = b - a
    BC = c - b

    # dot_product = np.dot(AB, BC)
    dot_product = np.einsum('ij,ij->i', AB, BC)

    magnitude_AB = np.linalg.norm(AB, axis=1)
    magnitude_BC = np.linalg.norm(BC, axis=1)

    cos_theta = dot_product / (magnitude_AB * magnitude_BC)

    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    angle_radians = np.arccos(cos_theta)

    angle_degrees = np.degrees(angle_radians)

    return angle_degrees


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

with mp_hands.Hands(
    model_complexity=0,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
) as hands:

    while cap.isOpened():
        success, frame = cap.read()
        
        if not success:
            print("Ignoring empty camera frame...")
            continue

        frame = cv2.flip(frame, 1)

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)

        if results.multi_hand_landmarks is None:
            print("No hands detected")
        else:
            print(f"Number of hands detected: {len(results.multi_hand_landmarks)}")
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:

                # DISPLAY LANDMARKS
                mp_drawing.draw_landmarks(
                    image=frame,
                    landmark_list=hand_landmarks,
                    connections=mp_hands.HAND_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing_styles.get_default_hand_landmarks_style(),
                    connection_drawing_spec=mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # DISPLAY COORDINATES
                # for idx in fingertip_indices:
                
                #     landmark = hand_landmarks.landmark[idx]

                #     #print(f"Landmark {idx}: x={landmark.x}, y={landmark.y}, z={landmark.z}")

                #     h, w, _ = frame.shape
                #     x = int(landmark.x * w)
                #     y = int(landmark.y * h)
                #     z = landmark.z

                #     text = f"{idx}: ({x} , {y}, {z:.2f})"
                #     cv2.putText(frame, text, (x + 5, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                
                # EXAMPLE TO DISPLAY BENT DEGREE
                # mcp_landmark = hand_landmarks.landmark[index_mcp]
                # pip_landmark = hand_landmarks.landmark[index_pip]
                # dip_landmark = hand_landmarks.landmark[index_dip]

                # a = [mcp_landmark.x, mcp_landmark.y, mcp_landmark.z]
                # b = [pip_landmark.x, pip_landmark.y, pip_landmark.z]
                # c = [dip_landmark.x, dip_landmark.y, dip_landmark.z]

                # bent_degree = calculate_angle(a,b,c)
                # bent_degree_text = f"Bent Degree: {bent_degree:.2f}°"
                # cv2.putText(frame, bent_degree_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

                
                
                # DISPLAY BENT DEGREE FOR EACH JOINT
                a=[]
                b=[]
                c=[]

                for finger,indices in finger_indices.items():
                    landmark1 = hand_landmarks.landmark[indices[0]]
                    landmark2 = hand_landmarks.landmark[indices[1]]
                    landmark3 = hand_landmarks.landmark[indices[2]]

                    a.append([landmark1.x,landmark1.y,landmark1.z])
                    b.append([landmark2.x,landmark2.y,landmark2.z])
                    c.append([landmark3.x,landmark3.y,landmark3.z])

                bent_degree = calculate_angle(a,b,c)
                udp_message = []
                # for i, (finger,indices) in enumerate(finger_indices.items()):
                for i in range(len(finger_indices.items())):
                
                    displayDegree = hand_landmarks.landmark[fingertip_indices[i]]

                    h, w, _ = frame.shape
                    x = int(displayDegree.x * w)
                    y = int(displayDegree.y * h)
                    z = displayDegree.z

                    bent_degree_text = f"{bent_degree[i]:.2f}"
                    cv2.putText(frame, bent_degree_text, (x-10 , y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                    
                    udp_message.append(bent_degree_text)
                    # sock.sendto(bent_degree_text.encode(), (UDP_IP, UDP_PORT))
                    # print(bent_degree_text)
                
                udp_message = ",".join(udp_message)
                sock.sendto(udp_message.encode(), (UDP_IP, UDP_PORT))



        cv2.imshow("Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cap.release()
sock.close()  