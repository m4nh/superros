#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math
from superros.comm import RosNode
from superros.logger import Logger
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolResponse


def serviceCallback(msg):
    """Enable Service Callback

    Arguments:
        msg {BoolMsg} -- TRUE/FALSE
    """
    global enabled
    enabled = msg.data
    response = SetBoolResponse()
    response.success = True
    return response


node = RosNode("ping")
node.setupParameter("hz", 30)
node.setHz(node.getParameter("hz"))
enabled = node.setupParameter("enabled", True)

# Creates a Float Publisher
output_publisher = node.createPublisher("~output", Float32)

# Creates an Enable Service
enable_service = node.createService("~enable", SetBool, serviceCallback)

# Remote Service
remote_name = "ping" if "pong" in node.getName() else "pong"
remote_enable_service = node.getServiceProxy(remote_name + "/enable", SetBool)

# start
start_time = 0.0
start_offset = 0.0 if "ping" in node.getName() else math.pi

# Main Loop
while node.isActive():

    # Generates new Float message
    value = Float32()
    value.data = math.sin(node.getElapsedTimeInSecs() *
                          2 * math.pi * 0.2 - start_offset)

    # Publish if enabled
    if enabled:
        Logger.warning("{} -> {}".format(node.getName(), value.data))
        output_publisher.publish(value)
        if value.data < 0:
            remote_enable_service(True)
            enabled = False
    else:
        output_publisher.publish(Float32(0.0))

    node.tick()
