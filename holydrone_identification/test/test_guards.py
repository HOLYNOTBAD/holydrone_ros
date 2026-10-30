import threading
import unittest
from unittest.mock import patch

try:
    import rospy
    from mavros_msgs.msg import State
except ImportError:
    rospy=None


@unittest.skipIf(rospy is None,'ROS Noetic required')
class GuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from holydrone_identification import fsm
        cls.module=fsm

    def construct(self,parameters):
        with patch.object(rospy,'get_param',side_effect=lambda name,default=None:parameters.get(name,default)):
            return self.module.IdentificationFSM()

    def test_real_cannot_auto_arm(self):
        with self.assertRaises(ValueError):
            self.construct({'~environment':'real','/use_sim_time':False,'~auto_takeoff':True,'~enable_real_excitation':True})

    def test_real_requires_explicit_opt_in(self):
        with self.assertRaises(ValueError):self.construct({'~environment':'real','/use_sim_time':False})

    def test_clock_mismatch_rejected(self):
        with self.assertRaises(ValueError):self.construct({'~environment':'sim','/use_sim_time':False})

    def test_prepare_refuses_other_controller(self):
        node=self.module.IdentificationFSM.__new__(self.module.IdentificationFSM)
        node.lock=threading.RLock();node.exclusive=lambda:False
        self.assertFalse(node.prepare(None).success)

    def test_start_requires_armed_offboard(self):
        node=self.module.IdentificationFSM.__new__(self.module.IdentificationFSM)
        node.lock=threading.RLock();node.ready=lambda:True;node.stable=lambda:True;node.phase='hold'
        for armed,mode in [(False,'OFFBOARD'),(True,'POSCTL')]:
            node.data={'state':(State(armed=armed,mode=mode),0)}
            self.assertFalse(node.start(None).success)

    def test_stop_idle_does_not_publish_hold(self):
        node=self.module.IdentificationFSM.__new__(self.module.IdentificationFSM)
        node.lock=threading.RLock();node.phase='idle'
        self.assertTrue(node.stop(None).success)
        self.assertEqual(node.phase,'idle')
