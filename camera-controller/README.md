# How to Use

## View the Camera image
```
roslaunch rpl camera.launch
```

## View the AR image 
```
roslaunch rpl ar.launch
```

## View the Calibrated image
```
roslaunch rpl calibration.launch
```

---

## When you use Rosbot2, 

### Save robot positions to create the Transform matrix from camera coordinates to robot coordinates
```
roslaunch rpl pre_processing.launch
```

### Estimate robot position from camera image
```
roslaunch rpl post_processing.launch
```

### Detect the error 

---

## When you use minimini2, 

### Save robot positions to create the Transform matrix from camera coordinates to robot coordinates
```
roslaunch rpl pre_processing_minimini2.launch
```

### Estimate robot position from camera image
```
roslaunch rpl post_processing_minimini2.launch
```


## external-camera
```
cd docker/
docker build -t external-camera .
docker run --device=/dev/video0:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it --rm --net rosnet --name external-camera --env ROS_HOSTNAME=external-camera --env ROS_MASTER_URI=http://rosmaster:11311 external-camera rosrun external_camera external_camera
```
