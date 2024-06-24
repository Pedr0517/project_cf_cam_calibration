# Project Cam Calibration

## Mission

The folder contains a number of Python3 scripts which will be used to calibrate a camera inside a given drone.

 
### Images

With the given Python3 scripts, a few of them allow one to obtain a number of images in PNG format. 

* mission_Test.py
* take_image.py
* save_image.py

#### mission_Test.py
The use of **mission_Test.py** allows for image capturing whilst being fully autonomous. This is done by giving the script a number of paramaters before running the code. Once the parameters are given, the script will produce a path through some backend Python3 scritps; which can be found inside the folder. Once script is running, drone will follow path and at each point the drone pauses, an image will be uploaded to the wanted directory. Once path is finished, the option to repeat path will be shown, if one chooses not to repeat path, drone will land safely and shutoff.

#### take_image.py
The use of **take_image.py** allows one to capture images while having full control of the drone. The whole point of this script is to allow one to manouver the drone manually and take images when doing so. When running the script, a small box will appear. This box will allow for the user to have interface with the drone. When one interfaces with the system, an image will be uploaded to its wanted directory. Once image capturing is completed, close box to kill script.

#### save_image.py
The use of **save_image.py** allows for image capturing while having full controll of the drone. Though with this script, one will not be able to elect when an image is uploaded to its wanted directory. Once the script is ran, images will automatically upload to the directory of choice and will only shutoff once scirpt is killed. 


### Calibration
Inside the folder, two Python3 scripts help with the claibration process.

* charucoCalib.py
* camera_info.py

#### charucoCalib.py
The script **charucoCalib.py** allows for calibration of the images to occur. Takes in images from a specified directoy and follows a calibration process. Once completed, information regarding the calibration will be obtained.

#### camera_info.py
The script **camera_info** helps the user have info regarding the calibration to be formatted in a seperate txt file.
