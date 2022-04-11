## Pick-and-Place-YOLO-Object-detection-using-ROS-on-KUKA
In this repository, we are going to address one of these operations, pick and place, which is the most widely used in production lines and assembly processes.
The aim of this project is to select 3 objects out of 6 objects in robot work environment, and place them in specific-colored regions, so that each object will be placed in
colored region initially specified by the operator.

<p align="left"><img src="https://user-images.githubusercontent.com/90580636/162748319-1fc91285-5d85-4501-b2fd-9bae62f6d7af.png" width="600" height="280" /></p>


### The challenges of this project go as follows:
- **Object Detection**: YOLOv4 
- **Box Detection**: Color Segmentation in HSI and Morphology operations
- **Pick and Place**: Object center and orientation and Box center and orientation
- **Localization w.r.t Robot Base**: Eye-In-Hand Camera Calibration and Transformations
- **Hardware Implementation**: KUKA iiwa and ROS

### Workplace Setup (Objects)
<p align="left"><img src="https://user-images.githubusercontent.com/90580636/162749872-b8e22f37-6889-4952-b363-8e2a41be1263.png" width="400" height="280" /></p>

Dataset can be found [here](https://drive.google.com/drive/folders/1f0c6RHHA6wQWN_TKWIUymxcq18y4UrNl?usp=sharing). 

Labelling is done on [roboflow](https://roboflow.com/)

### Dataset preprocessing and augmentations 
<p align="left"><img src="https://user-images.githubusercontent.com/90580636/162843034-79c7fc70-a615-4cde-a105-39089ea2c5f3.png" width="500" height="280" /></p>

### YOLO model testing for object detection
- Test1. Full video is attached [test1](https://drive.google.com/file/d/1i6OHv5xaYZdAKVNJCA2oywVlVbPEVjBz/view?usp=sharing).
<p align="left"><img src="https://user-images.githubusercontent.com/90580636/162844006-b72dc54b-8f00-44b7-b4e8-284ade53c485.gif" width="250" height="140" /></p>
                                    
Problems arise due to illumination change, camera orientation, false positive results (detect the gripper as an object, detect background as an object).  

Solution: Applying thresholding on detection confidence score.

- Test2. Full video is attached [test2](https://drive.google.com/file/d/1ancea1p9NzBAG7z_7wcQ2mSaT_crPq88/view?usp=sharing).
<p align="left"><img src="https://user-images.githubusercontent.com/90580636/162844601-5b23ccc4-eaec-4402-b841-1047a62ac1ee.gif" width="250" height="140" /></p>

### Object center and orientation 
<p align="left"><img src="https://user-images.githubusercontent.com/90580636/162844828-a24be0e2-37f9-4fe2-b5c4-9c12115d80aa.png" width="500" height="200" /></p>
