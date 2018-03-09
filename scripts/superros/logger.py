#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import inspect


class Logger(object):

    LOGGER_LEVEL_BASE = 1
    LOGGER_LEVEL_FULL = 2
    LOG_LEVEL = LOGGER_LEVEL_FULL

    LOGGER_TYPE_LOG = 1000
    LOGGER_TYPE_ERROR = 1001
    LOGGER_TYPE_WARNING = 1002
    LOGGER_TYPE_DEBUG = 1003

    @staticmethod
    def _log_private(message, logtype=LOGGER_TYPE_LOG, loglevel=LOGGER_LEVEL_BASE):
        full_message = ""
        if loglevel == Logger.LOGGER_LEVEL_FULL:
            file_name = inspect.stack()[2][1].split("/")[-1]
            file_row = inspect.stack()[2][2]
            method_name = inspect.stack()[2][3]
            full_message = "{}({})#{}: {}".format(
                file_name, file_row, method_name, message)
        elif loglevel == Logger.LOGGER_LEVEL_BASE:
            full_message = message

        if logtype == Logger.LOGGER_TYPE_LOG:
            rospy.loginfo("%s", full_message)
        elif logtype == Logger.LOGGER_TYPE_ERROR:
            rospy.logerr("%s", full_message)
        elif logtype == Logger.LOGGER_TYPE_WARNING:
            rospy.logwarn("%s", full_message)
        elif logtype == Logger.LOGGER_TYPE_DEBUG:
            rospy.logdebug("%s", full_message)

    @staticmethod
    def error(message, loglevel=LOGGER_LEVEL_FULL):
        Logger._log_private(
            message, logtype=Logger.LOGGER_TYPE_ERROR, loglevel=loglevel)

    @staticmethod
    def log(message):
        Logger._log_private(message, logtype=Logger.LOGGER_TYPE_LOG)

    @staticmethod
    def debug(message):
        Logger._log_private(message, logtype=Logger.LOGGER_TYPE_DEBUG)

    @staticmethod
    def warning(message):
        Logger._log_private(message, logtype=Logger.LOGGER_TYPE_WARNING)
