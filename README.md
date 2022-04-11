## Pick-and-Place-Object-detection-using-YOLO-on-KUKA
In this repository, we are going to address one of these operations, pick and place, which is the most widely used in production lines and assembly processes.
The aim of this project is to select 3 objects out of 6 objects in robot work environment, and place them in specific-colored regions, so that each object will be placed in
colored region initially specified by the operator.

<p align="left"><img src="https://user-images.githubusercontent.com/90580636/162748319-1fc91285-5d85-4501-b2fd-9bae62f6d7af.png" width="600" height="280" /></p>


The challenges of this project go as follow:
- Object Detection: YOLOv4 
- Box Detection: Color Segmentation in HSI and Morphology operations
- Pick and Place: Object center and orientation and Box center and orientation
- Localization w.r.t Robot Base: Eye-In-Hand Camera Calibration and Transformations
- Hardware Implementation: KUKA iiwa and ROS
                                                    
