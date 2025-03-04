import cv2
import numpy as np

import socket

import mediapipe.python.solutions.hands as mp_hands
import mediapipe.python.solutions.drawing_utils as mp_drawing
import mediapipe.python.solutions.drawing_styles as mp_drawing_styles

from dotenv import load_dotenv
import os

cap = cv2.VideoCapture(0)

# DISPLAY COORDINATES
fingertip_indices = [4, 8, 12, 16, 20]

# TARGET JOINTS
joint_indices = {
    "thumb": [2,3],
    "index": [5,6,7],
    "middle": [9,10,11],
    "ring": [13,14,15],
    "pinky": [17,18,19]
}


def calculate_angle(a, b, c):

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


load_dotenv()
UDP_IP = os.getenv("UDP_IP")
UDP_PORT = int(os.getenv("UDP_PORT"))

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
        
        # Flip the image horizontally
        frame = cv2.flip(frame, 1)
        # Convert default BGR image to RGB image for mediapipe processing
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Mediapipe processing
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
                
                # DISPLAY BENT DEGREE FOR EACH JOINT
                a=[]
                b=[]
                c=[]

                # DETERMINE WHICH JOINTS INVOLVED TO CALCULATE EACH JOINT BENT DEGREE
                # Example:
                # TARGET JOINTS: 2. JOINTS INVOLVED: 1(a), 2(b), 3(c)
                # TARGET JOINTS: 6. JOINTS INVOLVED: 5(a), 6(b), 7(c)
                # (Joint number based on mediapipe numbering)
                for finger,indices in joint_indices.items():
                    
                    for i in range (0,len(indices)):
                        if(len(indices) == 3) and (i == 0):
                            landmark1 = hand_landmarks.landmark[0]
                        else:
                            landmark1 = hand_landmarks.landmark[indices[i] - 1]
                
                        landmark2 = hand_landmarks.landmark[indices[i]]
                        landmark3 = hand_landmarks.landmark[indices[i] + 1]

                        a.append([landmark1.x,landmark1.y,landmark1.z])
                        b.append([landmark2.x,landmark2.y,landmark2.z])
                        c.append([landmark3.x,landmark3.y,landmark3.z])

                # CARRY OUT CALCULATIONS
                bent_degree = calculate_angle(a,b,c)
                
                
                i = 0

                udp_message = []
                
                display_positions = {
                    "thumb": (10, 30),
                    "index": (110, 30),
                    "middle": (210, 30),
                    "ring": (310, 30),
                    "pinky": (410, 30)
                }
                line_height = 20

                # LOOP THROUGH EACH JOINT BENT DEGREE, 
                # APPEND TO udp_message[]
                # and DISPLAY ON THE SCREEN

                for finger,indices in joint_indices.items():
                    for j in range(len(indices)):
                        
                        bent_degree_text = f"{bent_degree[i]:.2f}"

                        # APPEND TO udp_message[]
                        udp_message.append(bent_degree_text)

                        # DISPLAY ON THE SCREEN
                        text_x, text_y = display_positions[finger]
                        cv2.putText(frame, bent_degree_text, (text_x, text_y + j*line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                        
                        i += 1

                # SEND UDP MESSAGE
                udp_message = ",".join(udp_message)
                sock.sendto(udp_message.encode(), (UDP_IP, UDP_PORT))


        cv2.imshow("Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cap.release()
sock.close()  