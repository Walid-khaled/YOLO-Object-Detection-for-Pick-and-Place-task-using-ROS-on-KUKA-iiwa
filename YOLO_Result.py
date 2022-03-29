import random
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import darknet
        

def showInRow(list_of_images, titles = None, disable_ticks = False):
    plt.figure(figsize=(16, 10))
    count = len(list_of_images)
    for idx in range(count):
        subplot = plt.subplot(1, count, idx+1)
        if titles is not None:
            subplot.set_title(titles[idx])
        img = list_of_images[idx]
        subplot.imshow(img)
        # plt.axis("off")
        if disable_ticks:
            plt.xticks([]), plt.yticks([])
    # plt.show(block=False)
    # plt.pause(5)
    # plt.close()
    plt.show()

def save_annotations(image, detections, class_names):
    """
    Files saved with image_name.txt and relative coordinates
    """
    with open("BBOX.txt", "w") as f:
        for label, confidence, bbox in detections:
            x, y, w, h = bbox
            label = class_names.index(label)
            f.write("{} {} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}\n".format(label, class_names[label], x, y, w, h, float(confidence)))


def main():
    config_file = "./cfg/custom-yolov4-detector.cfg"
    data_file = "./cfg/coco.data"
    weights = "./best.weights"
    batch_size = 1
    thresh = 0.7

    random.seed(3)  # deterministic bbox colors
    network, class_names, class_colors = darknet.load_network(
        config_file,
        data_file,
        weights,
        batch_size
    )

    cap = cv2.VideoCapture(0)                                       # To use Webcam
    # cap = cv2.VideoCapture("test1.avi")                           # Local Stored video detection - Set input video

    frame_width = darknet.network_width(network)
    frame_height = darknet.network_height(network)
    # Set out for video writer
    # Set the Output path for video writer
    # out = cv2.VideoWriter("./Demo/output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 10.0, (frame_width, frame_height))

    print("Starting the YOLO loop...")

    # Create an image we reuse for each detect
    darknet_image = darknet.make_image(frame_width, frame_height, 3) # Create image according darknet for compatibility of network
   
    counter = 0
    while True:                                                      # Load the input frame and write output frame.

        prev_time = time.time()
        ret, frame_read = cap.read()                                 # Capture frame and return true if frame present
        # For Assertion Failed Error in OpenCV
        if not ret:                                                  # Check if frame present otherwise he break the while loop
            break

        frame_resized = cv2.resize(frame_read, (frame_width, frame_height), interpolation=cv2.INTER_LINEAR)
        # Copy that frame bytes to darknet_image
        darknet.copy_image_from_bytes(darknet_image,frame_resized.tobytes())                
        detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
        image = darknet.draw_boxes(detections, frame_resized.copy(), class_colors)

        save_annotations(image, detections, class_names)
        darknet.print_detections(detections, coordinates = True)
        fps = int(1/(time.time() - prev_time))
        print("FPS: {}".format(fps))

        cv2.imshow('Demo', image)                                    # Display Image window
        cv2.waitKey(3) 
        # out.write(image)                                           # Write that frame into output video
        counter += 1
       
        if counter >= 40:
            cv2.imwrite('./Object Detection.jpg', image)
            break

    cap.release()                                                    # For releasing cap and out. 
    # out.release()
    cv2.destroyAllWindows()
    print(":::Object Detection Completed\n")

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    showInRow([image])

    # Objects center and orientation
    # lower = [10] 
    # upper = [35] 
    # lower = np.array(lower, dtype="uint8")
    # upper = np.array(upper, dtype="uint8")
    print(":::Objects centers")

    img = np.zeros((frame_resized.shape[:2]),np.uint8)
    list_shapes = []
    for label, confidence, bbox in detections:
        xmin, ymin, xmax, ymax = darknet.bbox2points(bbox)
        cx,cy = int(round(xmax-xmin)/2), int(round(ymax-ymin)/2)
        label = class_names.index(label)
        shape = frame_resized[ymin:ymax,xmin:xmax]
        # shape_grey = cv2.cvtColor(shape, cv2.COLOR_BGR2GRAY)
        # mask = cv2.inRange(shape_grey, lower, upper)
        # img[ymin:ymax,xmin:xmax] = mask
        # cv2.circle(shape,(cx,cy), 3, (255,0,255), -1)
        list_shapes.append(shape)

        CX, CY = xmin+cx, ymin+cy
        cv2.circle(frame_resized,(CX, CY), 3, (255,0,255), -1)
        print("{} CX:{} CY:{}".format(class_names[label], CX, CY))

    showInRow(list_shapes)
    
    # frame_resized = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
    cv2.imwrite('./Objects centers.jpg', frame_resized)
    showInRow([frame_resized])

    # cv2.imshow('Demo', img)                                  
    # cv2.waitKey() 
    # _, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # contours_list = [i for i in contours if cv2.contourArea(i) < 4000 and cv2.contourArea(i) > 1000]
    # cv2.drawContours(image, contours_list, -1, (255,0,0), 2)
    # cv2.imshow('Demo', image)                                  
    # cv2.waitKey() 

if __name__ == "__main__":
    main()

