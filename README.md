# hades_comm_handler

Includes two nodes - `hades_streamer` and `base_receiver`. 

`hades_streamer` subscribes to a theora video stream, packetises it, then publishes onto a serialOut topic subsribed to by the rosserial node running on the teensy (see included teensy code). 

the teensy code handles SPI connection to the Si1060, and publishes received radio packets on a serial topic too.

`base_receiver` subscribes to the rosserial link from teensy, reconstitutes the packets into theora frames, then publishes them as a video stream.

## launch order

on the receiving end:

1.`roscore`

2. `rosrun rosserial_python _port:=/dev/ttyACM0 _baud:=2000000` **check port is correct**.

3. `rosrun hades_comm_handler base_receiver`

4. `rosrun image_view image_view _image:=videoFeed/ transport:=theora`

on the streaming end:

1. `roscore`

2. `rosrun rosserial_python _port:=/dev/ttyACM0 _baud:=2000000` **check port is correct**.

3. `rosrun hades_comm_handler hades_streamer`

**check everything set up and ready - this step must be done last or theora stream will break**

4. `roslaunch realsense_camera r200_nodelet_rgbd.launch color_fps:=15`
