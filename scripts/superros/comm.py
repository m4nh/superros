#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil
import rospy
import tf
import rospkg
import os
from logger import Logger
import transformations


class RosParamType(object):
    PRIVATE_PARAM = 1
    GLOBAL_PARAM = 10


class RosNode(object):

    def __init__(self, node_name="new_node", hz=30, disable_signals=False):
        """Creates a Ros Node Wrapper implementing whole communication stack.

        Keyword Arguments:
            node_name {str} -- Node Name (default: {"new_node"})
            hz {int} -- Node frequency (default: {30})
            disable_signals {bool} -- If TRUE disable system signals (default: {False})
        """

        self.node_name = node_name
        self.hz = hz
        rospy.init_node(self.node_name, disable_signals=disable_signals)

        if self.hz > 0:
            self.rate = rospy.Rate(self.hz)
        else:
            self.rate = None

        # Parameters map
        self._parameters = {}

        # Topics
        self.publishers = {}
        self.subsribers = {}
        self.subsribers_buffered_data = {}
        self.subsribers_buffered_data_types = {}

        # Services
        self.services = {}
        self.clients = {}

        # helpers
        self.tf_listener = None
        self.tf_broadcaster = None
        self.rospack = rospkg.RosPack()

        # Timings
        self.starting_time = -1

    def getName(self, replace_slash=True):
        """Returns Node name

        Keyword Arguments:
            replace_slash {bool} -- If TRUE replaces slashes (default: {True})

        Returns:
            {str} -- Node name
        """

        name = rospy.get_name()
        if replace_slash:
            name = name.replace("/", "")
        return name

    def setHz(self, hz):
        """Sets Node Frequency (Hz)

        Arguments:
            hz {int} -- Frequency (Hz)
        """

        self.hz = hz
        self.rate = rospy.Rate(self.hz)

    def createSubscriber(self, topic_name, msg_type, callback, queue_size=1):
        """Creates a Topic Subscriber

        Arguments:
            topic_name {str} -- Topic Name
            msg_type {object} -- Message Type
            callback {function} -- New Message callback

        Keyword Arguments:
            queue_size {int} -- Topic queue size (default: {1})

        Returns:
            {Subscriber} -- Subscriber handle
        """

        self.subsribers[topic_name] = rospy.Subscriber(
            topic_name,
            msg_type,
            callback,
            queue_size=queue_size
        )
        return self.subsribers[topic_name]

    def _subscribersBufferedCallbacks(self, msg, topic_name):
        self.subsribers_buffered_data[topic_name] = msg

    def getBufferedData(self, topic_name):
        if topic_name not in self.subsribers_buffered_data_types:
            return None

        if topic_name not in self.subsribers_buffered_data:
            return self.subsribers_buffered_data_types[topic_name]()
        return self.subsribers_buffered_data[topic_name]

    def createBufferedSubscriber(self, topic_name, msg_type):
        self.subsribers_buffered_data_types[topic_name] = msg_type
        self.subsribers[topic_name] = rospy.Subscriber(
            topic_name,
            msg_type,
            self._subscribersBufferedCallbacks,
            topic_name
        )
        return self.subsribers[topic_name]

    def createPublisher(self, topic_name, msg_type, queue_size=1):
        """Creates a Topic Publisher

        Arguments:
            topic_name {str} -- Topic Name
            msg_type {object} -- Message Type

        Keyword Arguments:
            queue_size {int} -- Topic queue size (default: {1})

        Returns:
            {Publihser} -- Publisher handle
        """

        self.publishers[topic_name] = rospy.Publisher(
            topic_name,
            msg_type,
            queue_size=queue_size
        )
        return self.publishers[topic_name]

    def getPublisher(self, name):
        """Retrieves a Publisher's handle

        Arguments:
            name {str} -- Topic Name

        Returns:
            {Publisher} -- Publisher handle
        """

        if name in self.publishers:
            return self.publishers[name]
        return None

    def getSubscriber(self, name):
        """Retrices a Subscriber's handle

        Arguments:
            name {str} -- Topic Name

        Returns:
            {Subscriber} -- Subscriber handle
        """

        if name in self.subsribers:
            return self.subsribers[name]
        return None

    def createService(self, service_name, srv_type, callback):
        """Creates a Service 

        Arguments:
            service_name {str} -- Service Name
            srv_type {object} -- Service Message Type
            callback {function} -- Service consume callback

        Returns:
            {Service} -- Service Handle
        """

        self.services[service_name] = rospy.Service(
            service_name, srv_type, callback)
        return self.services[service_name]

    def getServiceProxy(self, service_name, srv_type):
        """Creates a Proxy to a remote Service

        Arguments:
            service_name {str} -- Service Name
            srv_type {object} -- Service Message Type

        Returns:
            {ServiceProxy} -- ServiceProxy Handle
        """

        rospy.wait_for_service(service_name)
        try:
            proxy = rospy.ServiceProxy(service_name, srv_type)
            return proxy
        except rospy.ServiceException, e:
            print ("Service call failed:", e)
            return None

    def _sleep(self):
        self.rate.sleep()

    def tick(self):
        """Update function, call it every Loop iteration to let the time passing by """

        if self.starting_time < 0:
            if self.getCurrentTime().to_sec() > 0:
                self.starting_time = self.getCurrentTime().to_sec()
        self._sleep()

    def isActive(self):
        """Is Node active?

        Returns:
            {bool} -- TRUE if is Active, FALSE otherwise
        """

        if self.starting_time < 0:
            self.tick()
        return not rospy.is_shutdown()

    def getTFListener(self):
        """Returns the Tf Listener handle

        Returns:
            {TfListener} -- TfListener Handle
        """

        if self.tf_listener == None:
            self.tf_listener = tf.TransformListener()
        return self.tf_listener

    def getTFBroadcaster(self):
        """Returns the Tf Broadcaster handle

        Returns:
            {TfBroadcaster} -- TfBroadcaster
        """

        if self.tf_broadcaster == None:
            self.tf_broadcaster = tf.TransformBroadcaster()
        return self.tf_broadcaster

    def getCurrentTime(self):
        """Returns Current Time 

        Returns:
            {Time} -- Ros Time
        """

        return rospy.Time.now()

    def getCurrentTimeInSecs(self):
        """Returns Current Time in seconds

        Returns:
            {float} -- Time in seconds
        """

        return self.getCurrentTime().to_sec()

    def getElapsedTimeInSecs(self):
        """Returns Elapsed Time from node start

        Returns:
            {float} -- Elapsed Time in seconds
        """

        if self.starting_time < 0:
            return 0
        return self.getCurrentTimeInSecs() - self.starting_time

    def broadcastTransform(self, frame, frame_id, parent_frame_id, time):
        """Broadcasts a Tf Transform (Frame) to the Tf Tree

        Arguments:
            frame {PyKDL.Frame} -- Frame to broadcast
            frame_id {str} -- Frame name
            parent_frame_id {str} -- Frame parent name
            time {Time} -- Ros Time
        """

        self.getTFBroadcaster().sendTransform(
            (frame.p.x(), frame.p.y(), frame.p.z()),
            frame.M.GetQuaternion(),
            time,
            frame_id,
            parent_frame_id
        )

    def retrieveTransform(self, frame_id, parent_frame_id, time=-1, print_error=False):
        """Retrieves a Tf Transform (Frame) from the Tf Tree        

        Arguments:
            frame_id {str} -- Frame 
            parent_frame_id {str} -- Frame parent name
            time {Time} -- Ros Time, set as "-1" to ignore Time
            print_erro {bool} -- TRUE for debug

        Returns:
            {PyKDL.Frame} -- Frame
        """

        if time == -1:
            time = rospy.Time(0)

        try:
            tf_transform = self.getTFListener().lookupTransform(
                parent_frame_id, frame_id, time)
            frame = transformations.tfToKDL(tf_transform)
            return frame
        except (tf.ExtrapolationException, tf.LookupException, tf.ConnectivityException) as e:
            if print_error == True:
                Logger.error("{}".format(str(e)))
            return None

    def getFileInPackage(self, pkg_name, file_path):
        """Returns an absolute Filepath given the relative path and the package name

        Arguments:
            pkg_name {str} -- Ros Package name
            file_path {str} -- Relative (to the package) file path

        Returns:
            {str} -- Absolute File Path
        """

        pack_path = self.rospack.get_path(pkg_name)
        return os.path.join(pack_path, file_path)

    def getRosParameter(self, parameter_name, default_value, type=RosParamType.PRIVATE_PARAM):
        """Gets a Ros Parameter from the parameters service or command line 

        Arguments:
            parameter_name {str} -- Parameter name
            default_value {object} -- Parameter default value if not found

        Keyword Arguments:
            type {RosParamType} -- Type of the parameter (default: {RosParamType.PRIVATE_PARAM})

        Returns:
            {object} -- value
        """

        final_name = parameter_name
        if type == RosParamType.PRIVATE_PARAM:
            final_name = '~' + final_name
        return rospy.get_param(final_name, default_value)

    def setupParameter(self, parameter_name, default_value, type=RosParamType.PRIVATE_PARAM, array_type=None):
        """Setups a parameter in one line

        Arguments:
            parameter_name {str} -- Parameter name
            default_value {object} -- Default value if not found

        Keyword Arguments:
            type {RosParamType} -- Type of the parameter (default: {RosParamType.PRIVATE_PARAM})
            array_type {object} -- if not NONE the input parameter is parsed as an array with ";" as separator (default: {None})

        Returns:
            {object} -- value
        """

        par = self.getRosParameter(parameter_name, default_value, type=type)
        if array_type != None:
            par = map(array_type, par.split(";"))
        self._parameters[parameter_name] = par
        return par

    def getParameter(self, parameter_name):
        """Gets a stored parameters

        Arguments:
            parameter_name {str} -- Parameter name

        Returns:
            {object} -- value
        """

        if parameter_name in self._parameters:
            return self._parameters[parameter_name]
        return None

    def await(self):
        """await for messages until end"""
        rospy.spin()
