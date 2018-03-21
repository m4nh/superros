# Super ROS!

<img src="https://m4nh.github.io/superros/superros.svg" alt="SuperROS" width="300"/>


## Table of Contents
- [Why SuperROS](#whysuperros)
- [Basic Usage: Publishers/Subscribers](#basicusage)
- [TF Transformations](#transforms)
- [3D Scene Visualization](#visualization)
- [RGB Camera](#rgbcamera)
 
 

  

<a name="whysuperros" />

## Why SuperROS?

SuperROS is something "super" (above) the Robot Operating System (<a href='http://www.ros.org/' target="_blank">ROS</a>). First of all, SuperROS allows you to separate your business logic from the ROS framework making more transparent the transition to another *middleware* (e.g. ROS2 in the years to come). But, more importantly, allows you to *Write Less and Do More*.

You can use SuperROS as a classical ROS package or just include the *scripts* folder to your PYTHONPATH. If you use it as a ROS package be sure to `catkin_make` it. More generally, to make things work you need ROS installed with the default procedure <a href="http://wiki.ros.org/kinetic/Installation/Ubuntu" target="_blank" >here</a>.

<a name="basicusage" />

## Basic usage: Publishers/Subscribers

In this simple example we want to design a simple *Amplifier* reading a *Float32* data which is then published (*doubled*) on another topic at a certain frequency. Although in this snippet the code reduction is not astonishing, it is a basic example to introduce the Publisher/Subscriber paradigm in SuperROS.

<img style='left:0;right:0;margin:0' width=200 src='https://media.giphy.com/media/229OC7hqUL2gzLwDtO/giphy.gif'/> 

<table >
  <tr>
  <th>Super ROS</th>
  <th>Plain ROS</th>
  </tr>
<tr>
<td>
    
```python
from superros.comm import RosNode
from std_msgs.msg import Float32
node = RosNode("amplifier")
node.setHz(node.setupParameter("hz", 30))
node.createBufferedSubscriber("in", Float32)
node.createPublisher("out", Float32)

while node.isActive():
    data = Float32(node.getBufferedData("in").data * 2.0)
    node.getPublisher("out").publish(data)
    node.tick()
``` 
    
</td>
   
<td>
    
```python
import rospy
from std_msgs.msg import Float32

current_value = 0.0
def newData(msg):
    global current_value
    current_value = msg.data

rospy.init_node('amplifier', anonymous=True)
rate = rospy.Rate(rospy.get_param("~hz", 30))
sub = rospy.Subscriber("in", Float32, newData)
pub = rospy.Publisher("out", Float32)

while not rospy.is_shutdown():
    data = Float32(current_value*2.0)
    pub.publish(data)
    rate.sleep()
``` 
    
</td>
</tr>
</table>

You can run directly the Amplifier Example with:
```bash
roslaunch superros example_subpub.launch
```
<a name="transforms" />

## TF Transforms made easy


In this example (a simple TF Listener/Broadcaster with some 3D transformations) it is clear the benefit of an unified proxy (ie. the *RosNode*) in order to reduce UserApp<->ROS interactions. Moreover, SuperROS uses the robust *PyKDL* library as shared model for 3D Transformations, simplifying code readability and portability.

<img style='left:0;right:0;margin:0' width=200 src='https://media.giphy.com/media/w78i2Mzqge1AToxP72/giphy.gif'/> 

<table >
  <tr>
  <th>Super ROS</th>
  <th>Plain ROS</th>
  </tr>
<tr>
<td>
    
```python
from superros.comm import RosNode
import PyKDL

node = RosNode("tf_manipulator")
node.setHz(node.setupParameter("hz", 30))

while node.isActive():
    # Fetches Frame from TF Tree
    frame = node.retrieveTransform("base_link", "odom")
   
    if frame is not None:
        # Creates a relative transformation frame
        t = PyKDL.Frame(
            PyKDL.Rotation.RotZ(1.57),
            PyKDL.Vector(0, 0, 0.5)
        )
        
        # Applies transformation
        frame = frame * t
        
        # Send Frame to TF Tree
        node.broadcastTransform(
            frame, "base_link_2", "odom", 
            node.getCurrentTime()
        )
        
    node.tick()
``` 
    
</td>
   
<td>
    
```python
import rospy
import tf
import numpy as np

rospy.init_node('tf_manipulator')
rate = rospy.Rate(rospy.get_param("hz", 30))

listener = tf.TransformListener()
br = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    # Fetches Frame (T,R) from TF Tree. May fail
    try:
        (trans1, rot1) = listener.lookupTransform(
            '/odom', '/base_link',
            rospy.Time(0)
        )
    except (tf.LookupException):
        continue
    
    # Packs T,R in a single 4x4 matrix 
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)
  
    # Creates a relative transformation frame
    t = tf.transformations.rotation_matrix(1.57, [0, 0, 1])
    t[:3, 3] = np.array([0, 0, 0.5]).T
    
    # Creates a relative transformation frame
    mat2 = np.dot(mat1, t)
    
    # Unpacks T,R from the 4x4 matrix
    trans2 = tf.transformations.translation_from_matrix(mat2)
    rot2 = tf.transformations.quaternion_from_matrix(mat2)
  
    # Send Frame to TF Tree
    br.sendTransform(
        trans2,  rot2, rospy.get_rostime(),
        "base_link2", "odom"
    )

    rate.sleep()
``` 
    
</td>
</tr>
</table>

<a name="visualization" />

## 3D Scene Visualization


In this example an animated 3D Scene was presented using the SuperROS simplified library. The communication between Rviz and your node is -- surely -- made with topics but this behaviour is masked by the *VisualizationScene* object.

<img style='left:0;right:0;margin:0' width=300 src='https://media.giphy.com/media/229B59VHfH4skJ6vEc/giphy.gif'/> 

<table >
  <tr>
  <th>Super ROS</th>
  <th>Plain ROS</th>
  </tr>
<tr>
<td>
    
```python
from superros.comm import RosNode
from visualization_msgs.msg import MarkerArray
from superros.draw import VisualizationScene, Color
import PyKDL
import math

node = RosNode("example_3dscene")
node.setHz(node.setupParameter("hz", 30))

pub = node.createPublisher("test", MarkerArray)

# Visualization Scene
scene = VisualizationScene(node, "test")
scene.createCone(name="cone1", color=Color.pickFromPalette(0))
scene.createCone(name="cone2", color=Color.pickFromPalette(1))
scene.createCube(
    name="cube1",
    color=Color.pickFromPalette(2),
    transform=PyKDL.Frame(PyKDL.Vector(-2.0, 0, 0))
)

# Cone2 Frame
cone_2_frame = PyKDL.Frame(PyKDL.Vector(2.0, 0, 0))

while node.isActive():

    # Update Cone2 Pose getting stored Object
    cone_2_frame.M.DoRotZ(0.02)
    scene.getObjectByName("cone2").setTransform(cone_2_frame)

    # Update Cone1 Pose overwriting old Object
    scene.createCone(name="cone1", transform=PyKDL.Frame(
        PyKDL.Vector(
                     0.0,
                     0.0,
                     math.sin(1.0+2*math.pi*1.0*node.getCurrentTimeInSecs())
                     )
    ))

    # Publish World Frame
    node.broadcastTransform(PyKDL.Frame(), "world",
                            "base", node.getCurrentTime())

    # Update Scene
    scene.show()
    # Node Forward
    node.tick()
``` 
    
</td>
   
<td>
    
```python
Ehm...?
``` 
    
</td>
</tr>
</table>

You can run directly the 3DScene Example with:
```bash
roslaunch superros example_3dscene.launch
```

<a name="rgbcamera" />

## RGB Camera

In this example the simple CameraRGB wrapper is shown. All the original ImageTransport and CvBridge image conversions stuff is hidden behind the simple *CameraRGB* object. With *registerUserCallabck* the user can bind its custom callback function called whenever a new *FrameRGB* is available. The frame contains in *rgb_image* the off-the-shelf numpy matrix representing the image, compliant with the *OpenCV/cv2* library.


```python
from superros.comm import RosNode
from superros.cameras import CameraRGB
import cv2

node = RosNode("example_rgbcamera")
node.setHz(node.setupParameter("hz", 30))
rgb_topic = node.setupParameter("camera_topic","")

# New Frame callback
def newFrame(frame):
    cv2.imshow("image",frame.rgb_image)
    cv2.waitKey(1)

# Camera Object
camera = CameraRGB(
    node,
    rgb_topic=rgb_topic,
    compressed_image='compressed'in rgb_topic
)
camera.registerUserCallabck(newFrame)

while node.isActive():
    node.tick()
``` 

You can run directly the 3DScene Example with:
```bash
roslaunch superros example_3dscene.launch
```
