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


