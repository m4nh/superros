# Super ROS

<img src="https://m4nh.github.io/superros/superros.svg" alt="SuperROS" width="300"/>


## Why SuperROS?

SuperROS is something "super" (above) the Robot Operating System (<a href='http://www.ros.org/' target="_blank">ROS</a>). First of all, SuperROS allows you to separate your business logic from the ROS framework making more transparent the transition to another *middleware* (e.g. ROS2 in the years to come). But, more importantly, allows you to *Write Less and Do More*.

You can use SuperROS as a classical ROS package or just include the *scripts* folder to your PYTHONPATH. 

## Basic usage: Publisher/Subscribers

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

## TF Transforms made easy


In this example (a simple TF Listener/Broadcaster with some 3D transformations) it is clear the benefit of an unified proxy (ie. the *RosNode*) in order to reduce UserApp<->ROS interactions. Moreover, SuperROS uses the robust *PyKDL* library as shared model for 3D Transformations, simplifying code readability and portability.

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


