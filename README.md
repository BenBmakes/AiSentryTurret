# AiSentryTurret
This is a project that is a sentry turret which uses yolov3 to track people
videos of this project built: 
https://www.tiktok.com/@.ryanair_uk/video/7365949124581149984
https://www.tiktok.com/@.ryanair_uk/video/7366289537993035041

What you need:
* 3 servos i used 9g ones
* arduino uno or another compastible micro controller
* bread board and jumper cables
* usb cable
* webcam
* hot glue
* building suplies

How to build:
connect all positive and all negetive of servos to the positive and negetive of the arduino
connect the bottom servo (servo1) (which is the left and right) to pin 9
connect the two servos which are the up and down to pin 10 and 11 respectively
then using wood or mechano build a u shape big enough to mount the two servos and your webcam inside it should look like this:

![should look like this](https://github.com/BenBmakes/AiSentryTurret/assets/169482343/28d838b1-51e8-44f9-b652-964d1805a4df)

Before running the code make sure you have: 
* edited the python code to have the same COM port as the arduino
* downloaded the files 'coco.names' , 'yolov3-tiny.cfg' and 'yolov3-tiny.weights' from the internet and put them in the same folder as the python script schould look like this

![what you need](https://github.com/BenBmakes/AiSentryTurret/assets/169482343/3196b28b-d040-4280-bb76-044fe33b7cfb)

if the up and down movment is reversed edit the arduino code to remove:

servo1.write(servo1Value);

servo2.write(180 - servo2Value);

servo3.write(servo3Value);


and replace with: 

servo1.write(servo1Value);

servo2.write(servo2Value);

servo3.write(180 - servo3Value);


if the left and right is reversed, replace:

servo1.write(servo1Value);


with:

servo1.write(180 - servo1Value);


