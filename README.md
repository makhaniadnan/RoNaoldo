# NAO Robot Soccer 


## Vision Marker

The file vision-marker.cpp is subsribing to two topics:

* ballposition
* image 

It then tries to find two aruco markers in the image. If it finds exactly two, it will calculate the horizontal center of them. Then, it will calculate if the ball center is left or right of the horizontal center of the two markers. It will publish the distance to the center under a new topic called `ballOffMiddle`.


### How to run and test it?

Run the commands below from your catin_ws folder, where under src/RoNaldo this project is located.


```
#start ros core
roscore &

#start publishing and example camera image to the /image topic
python src/RoNaoldo/publishImage.py src/RoNaoldo/images-testmarker/frame-test1.png &

#send an example ball position
rostopic pub -r 1 ballposition RoNAOldo/controlMsg  '{ballPositionX: 310.0, ballPositionY: 268.1}' &

#start our vision-marker to track markers
#remap image to image
#remap ball/imagePosition where vision-marker listens to our publisher, which is ballposition
rosrun RoNAOldo RoNAOldo_vision_marker image:=/image ball/imagePosition:=ballposition

```

Here, you can see an example image of the viewer debug output.

![Image](screenshots/vision-marker-1.png?raw=true)

The *blue line* shows the horizontal center between the two aruco marker's centers.

The *green circle* shows the ball position.

The *red boxes* show the recgonized aruco markers.



In order to see the next image, you have to press a key on the image viewer.
Then the next image will be processed.
