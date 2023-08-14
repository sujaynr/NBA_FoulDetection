import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import cv2
import numpy as np
import time

connections = [
    (0, 7),
    (7, 8),
    (8, 9),
    (9, 10),
    (7, 11),
    (11, 12),
    (12, 13),
    (7, 14),
    (14, 15),
    (15, 16),
    (0, 4),
    (4, 5),
    (5, 6),
    (0, 1),
    (1, 2),
    (2, 3)
] #connections between keypoints to make body

left_arm_indices = [11, 12, 13] #relevant points for shooting foul detection
right_arm_indices = [14, 15, 16]


def check_line_overlap(arm_lines1, arm_lines2, tolerance=.1): #checks arm lines overlap - used to call foul
    overlapping_frames = []

    for frame_num, (arm_dict1, arm_dict2) in enumerate(zip(arm_lines1, arm_lines2)):
        for arm_name in arm_dict1:
            for line1, line2 in zip(arm_dict1[arm_name], arm_dict2[arm_name]):
                overlap_x = False
                overlap_y = False
                overlap_z = False
                
                if line1['Line'] == line2['Line']:
                    dx1, x1 = line1['x']
                    dx2, x2 = line2['x']
                    
                    dy1, y1 = line1['y']
                    dy2, y2 = line2['y']
                    
                    dz1, z1 = line1['z']
                    dz2, z2 = line2['z']
                    
                    # Check for overlap in x-coordinate
                    if dx1 != 0 and dx2 != 0:
                        t_overlap_x = (x2 - x1) / (dx1 - dx2)
                        overlap_x = (0 <= t_overlap_x <= 1) or abs(t_overlap_x) < tolerance
                    
                    # Check for overlap in y-coordinate
                    if dy1 != 0 and dy2 != 0:
                        t_overlap_y = (y2 - y1) / (dy1 - dy2)
                        overlap_y = (0 <= t_overlap_y <= 1) or abs(t_overlap_y) < tolerance
                    
                    # Check for overlap in z-coordinate
                    if dz1 != 0 and dz2 != 0:
                        t_overlap_z = (z2 - z1) / (dz1 - dz2)
                        overlap_z = (0 <= t_overlap_z <= 1) or abs(t_overlap_z) < tolerance
                    
                    if overlap_x and overlap_y and overlap_z:
                        overlapping_frames.append(frame_num)
                        print(f"Overlap found at frame {frame_num}, {arm_name}, {line1['Line']} between both people.")
                        print(f"Body 1: {line1}")
                        print(f"Body 2: {line2}\n")
    
    return overlapping_frames


def compute_arm_lines(frames, frame_numbers): #finds equations for relevant arm lines - used for checking overlap
    arm_lines = [{}] * len(frames)
    
    for num, frame in enumerate(frames):
        if num not in frame_numbers:
            continue
        
        arm_lines[num] = {}
        for arm_indices, arm_name in [(left_arm_indices, "Left Arm"), (right_arm_indices, "Right Arm")]:
            arm_lines[num][arm_name] = []
            for i in range(len(arm_indices) - 1):
                p1, p2 = arm_indices[i], arm_indices[i + 1]
                x1, y1, z1 = frame[p1]
                x2, y2, z2 = frame[p2]
                dx = x2 - x1
                dy = y2 - y1
                dz = z2 - z1
                if i == 0:
                    line_name = "Shoulder to Elbow"
                else:
                    line_name = "Elbow to Wrist"
                arm_lines[num][arm_name].append({
                    "Line": line_name,
                    "x": (dx, x1),
                    "y": (dy, y1),
                    "z": (dz, z1)
                })
    
    return arm_lines

def plot_frames_with_lines_as_gif(frames1, frames2, filename): #visualization
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    x_coords1 = []
    y_coords1 = []
    z_coords1 = []
    for frame in frames1:
        x_coords1.append([point[0] for point in frame])
        y_coords1.append([point[1] for point in frame])
        z_coords1.append([point[2] for point in frame])
    
    x_coords2 = []
    y_coords2 = []
    z_coords2 = []
    for frame in frames2:
        x_coords2.append([point[0] for point in frame])
        y_coords2.append([point[1] for point in frame])
        z_coords2.append([point[2] for point in frame])
    
    max_length = max(len(x_coords1), len(x_coords2))
    
    # Create empty lists for missing frames
    while len(x_coords1) < max_length:
        x_coords1.append([])
    while len(y_coords1) < max_length:
        y_coords1.append([])
    while len(z_coords1) < max_length:
        z_coords1.append([])
    
    while len(x_coords2) < max_length:
        x_coords2.append([])
    while len(y_coords2) < max_length:
        y_coords2.append([])
    while len(z_coords2) < max_length:
        z_coords2.append([])
    
    sc = ax.scatter([], [], [], marker='o')
    
    # Create lines between specific points to form the body
    lines = [ax.plot([], [], [], color='blue')[0] for _ in connections]
    
    arm_lines1 = compute_arm_lines(frames1)
    arm_lines2 = compute_arm_lines(frames2)
    
    # Create text annotation for frame number
    text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes)
    
    def update_frame_with_lines(num, frames1, frames2, sc, lines, arm_lines1, arm_lines2, text):
        if num < len(frames1):
            frame1 = frames1[num]
            x_coords1 = [point[0] for point in frame1]
            y_coords1 = [point[1] for point in frame1]
            z_coords1 = [point[2] for point in frame1]
        else:
            x_coords1 = []
            y_coords1 = []
            z_coords1 = []

        if num < len(frames2):
            frame2 = frames2[num]
            x_coords2 = [point[0] for point in frame2]
            y_coords2 = [point[1] for point in frame2]
            z_coords2 = [point[2] for point in frame2]
        else:
            x_coords2 = []
            y_coords2 = []
            z_coords2 = []

        x_coords = x_coords1 + x_coords2
        y_coords = y_coords1 + y_coords2
        z_coords = z_coords1 + z_coords2
        
        sc._offsets3d = (x_coords, y_coords, z_coords)
        
        # Update lines between points
        for line, (i, j) in zip(lines, connections):
            line.set_data([x_coords[i], x_coords[j]], [y_coords[i], y_coords[j]])
            line.set_3d_properties([z_coords[i], z_coords[j]])
        
        # Set text annotation with frame number
        text.set_text(f'Frame: {num}')
        
        return [sc, text]

    
    ani = FuncAnimation(fig, update_frame_with_lines, frames=max_length,
                        fargs=(frames1, frames2, sc, lines, arm_lines1, arm_lines2, text), interval=200, blit=False)
    
    ani.save(filename, writer='pillow')  # Save animation as GIF


def calculate_overlap(box1, box2): #use 2D data to determine frames worth checking with 3D
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    
    intersection_area = max(0, min(x1_max, x2_max) - max(x1_min, x2_min)) * \
                        max(0, min(y1_max, y2_max) - max(y1_min, y2_min))
    
    area_box1 = (x1_max - x1_min) * (y1_max - y1_min)
    area_box2 = (x2_max - x2_min) * (y2_max - y2_min)
    
    total_area = area_box1 + area_box2 - intersection_area
    
    overlap_percentage = intersection_area / total_area
    
    return overlap_percentage >= 0.15


def apply_red_flash(input_path, output_path, flash_frames, pause_duration=2): #foul visualizer
    cap = cv2.VideoCapture(input_path)
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    frame_number = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        if frame_number in flash_frames:
            # Apply red flash effect
            red_overlay = np.full_like(frame, (255, 100, 100), dtype=np.uint8)  # Lighter red
            alpha = 0.3  # Transparency level
            cv2.addWeighted(red_overlay, alpha, frame, 1 - alpha, 0, frame)
            
            # Add text overlay
            font_scale = 2
            font_color = (0, 0, 255)  # Blue color
            font = cv2.FONT_HERSHEY_SIMPLEX
            text = "ILLEGAL CONTACT"
            text_size = cv2.getTextSize(text, font, font_scale, 2)[0]
            text_x = width - text_size[0] - 10
            text_y = text_size[1] + 10
            cv2.putText(frame, text, (text_x, text_y), font, font_scale, font_color, 2)

            out.write(frame)  # Write the frame with red flash and text
            
            # Pause the video for pause_duration seconds
            for _ in range(int(pause_duration * fps)):
                out.write(frame)  # Write the same frame multiple times
                time.sleep(1 / fps)  # Pause for the frame duration

        out.write(frame)

        frame_number += 1

    cap.release()
    out.release()
    cv2.destroyAllWindows()


def main():
    file_path = "/Users/sujaynair/Documents/NBA_FoulDetection/demo.json"  # Replace with the path to your 3D JSON
    file2d = "/Users/sujaynair/Documents/NBA_FoulDetection/demo2D.json" # Replace with the path to your 2D JSON
    leftArms = []
    rightArms = []
    body1Ani = []
    body2Ani = []
    frameOverlapsBool = []
    frameOverlaps = []

    try:
        with open(file_path, "r") as json_file, open(file2d, "r") as json_file2d:
            json_data = json.load(json_file)
            json_data2D = json.load(json_file2d)
            frames = json_data["instance_info"]
            frames2D = json_data2D


            for i in range(len(frames)): #iterate through frames
                overlap = False
                if len(frames2D[i]["instances"]) == 2:
                    bottomLeftX1 = frames2D[i]["instances"][0]["bbox"][0][0]
                    bottomLeftY1 = frames2D[i]["instances"][0]["bbox"][0][1]
                    topRightX1 = frames2D[i]["instances"][0]["bbox"][0][2]
                    topRightY1 = frames2D[i]["instances"][0]["bbox"][0][3]

                    bottomLeftX2 = frames2D[i]["instances"][1]["bbox"][0][0]
                    bottomLeftY2 = frames2D[i]["instances"][1]["bbox"][0][1]
                    topRightX2 = frames2D[i]["instances"][1]["bbox"][0][2]
                    topRightY2 = frames2D[i]["instances"][1]["bbox"][0][3]

                    box1 = (bottomLeftX1, bottomLeftY1, topRightX1, topRightY1)
                    box2 = (bottomLeftX2, bottomLeftY2, topRightX2, topRightY2)

                    if calculate_overlap(box1, box2) == True:
                        overlap = True
                

                frameOverlapsBool.append(overlap)

            for i in range(len(frameOverlapsBool)):
                if frameOverlapsBool[i] == True:
             	   frameOverlaps.append(i)

                currentFrame = frames[i]
                frameInstances = currentFrame["instances"]
                instance1 = frameInstances[0]
                keypoints1 = instance1["keypoints"]
                if len(frameInstances) == 2:
                    instance2 = frameInstances[1]
                    keypoints2 = instance2["keypoints"]
                    body2Ani.append(keypoints2)
                body1Ani.append(keypoints1)

            # plot_frames_with_lines_as_gif(body1Ani, body2Ani, 'bodys.gif') # if plotting is wanted

            body1Lines = compute_arm_lines(body1Ani, frameOverlaps)
            body2Lines = compute_arm_lines(body2Ani, frameOverlaps)

            finalFrames = check_line_overlap(body1Lines, body2Lines)

            apply_red_flash("/Users/sujaynair/Downloads/flightAdin.mp4", "output.mp4", finalFrames) #adjust path


    except FileNotFoundError:
        print("File not found.")
    except json.JSONDecodeError:
        print("Invalid JSON format.")

if __name__ == "__main__":
    main()
