#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import unittest

import rospy
import roslib.message

from sensor_msgs.msg import LaserScan

from nlj_laser.msg import LaserDescriptor

from lama_interfaces.interface_factory import interface_factory

from nlj_laser.srv import GetLaserDescriptorRequest


class RosTestCase(unittest.TestCase):
    def assertMsgEqual(self, msg0, msg1):
        """Fail if two ROS messages are not equal"""
        self.assertIsInstance(msg0, roslib.message.Message,
                              msg='Argument 1 is not a Message')
        self.assertIsInstance(msg1, roslib.message.Message,
                              msg='Argument 2 is not a Message')
        slots0 = msg0.__slots__
        slots1 = msg1.__slots__
        self.assertEquals(slots0, slots1,
                          msg=('Messages do not have the same arguments\n' +
                               'Msg1 args: {}\n'.format(slots0) +
                               'Msg2 args: {}\n'.format(slots1)))
        for slot in slots0:
            value0 = getattr(msg0, slot)
            value1 = getattr(msg1, slot)
            if isinstance(value0, roslib.message.Message):
                self.assertMsgEqual(value0, value1)
            elif isinstance(value0, (list, tuple)):
                self.assertEquals(len(value0), len(value1),
                                        msg="list size is not same")
#                self.assertAlmostEquals(list(value0), list(value1), places=6,
#                                        msg='Argument {} differ: {} != {}'.
#                                        format(slot, value0, value1))
            else:
                self.assertAlmostEqual(value0, value1, places=6,
                                       msg='Argument {} differ: {} != {}'.
                                       format(slot, value0, value1))


class TestDbMessagePassing(RosTestCase):
    """test setting and getting laser descriptors"""
    def __init__(self, *args, **kwargs):
        rospy.init_node('test_lama_interfaces_nlj_laser', anonymous=True)
        super(TestDbMessagePassing, self).__init__(*args, **kwargs)

    def test_laser(self):
        """Test passing and getting a Laser message"""
        interface_name = 'laser_descriptor'
        getter_service = 'nlj_laser/GetLaserDescriptor'
        setter_service = 'nlj_laser/SetLaserDescriptor'

        # Set up node as well as getter and setter services.
        iface = interface_factory(interface_name,
                                  getter_service,
                                  setter_service)
        get_srv = rospy.ServiceProxy(iface.getter_service_name,
                                     iface.getter_service_class)
        set_srv = rospy.ServiceProxy(iface.setter_service_name,
                                     iface.setter_service_class)

        laser0 = LaserDescriptor()
        laser0.scans.append(LaserScan())
        for  r in xrange(0, 100):
            laser0.scans[0].ranges.append(r*0.01)
        laser1 = LaserDescriptor()
        laser1.scans.append(LaserScan())
        for  r in xrange(0, 100):
            laser1.scans[0].ranges.append(r*0.01)

        id_from_setter0 = set_srv(laser0)
        # id_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a Get...Request()
        id_to_getter0 = GetLaserDescriptorRequest()
        id_to_getter0.id = id_from_setter0.id
        response0 = get_srv(id_to_getter0)

        id_from_setter1 = set_srv(laser1)
        # id_from_setter cannot be passed to get_srv because of
        # type incompatibility, "transform" it to a Get...Request()
        id_to_getter1 = GetLaserDescriptorRequest()
        id_to_getter1.id = id_from_setter1.id
        response1 = get_srv(id_to_getter1)

        self.assertIsNot(laser0, response0.descriptor)
        self.assertIsNot(laser1, response1.descriptor)
        self.assertMsgEqual(laser0, response0.descriptor)
        self.assertMsgEqual(laser1, response1.descriptor)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('nlj_laser',
                   'test_db_message_passing',
                   TestDbMessagePassing)
