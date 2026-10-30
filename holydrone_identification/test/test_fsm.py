"""State transitions, control ownership and completion semantics without hardware."""
import collections
import time
import runpy
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from holydrone_identification.fsm import IdentificationFSM


class FSMTest(unittest.TestCase):
    def setUp(self):
        self.clock=patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.));self.clock.start()
        self.log=patch.object(rospy,'loginfo');self.log.start()
        n=self.node=IdentificationFSM.__new__(IdentificationFSM)
        n.phase='pulse';n.reason='test';n.index=0;n.segment_base=0;n.plan=[(0,1,1)]
        n.sequence_complete=False;n.auto=False;n.auto_used=True;n.begin=0.;n.phase_time=0.;n.settled=None
        n.reference=PoseStamped();n.reference.pose.position.z=2.;n.reference.pose.orientation.w=1.
        n.p=dict(max_session_seconds=1000.,max_radius=1.,max_tilt=.35,min_z=.5,max_z=3.,
                 state_timeout=2.,settle_seconds=1.,recovery_seconds=2.,baseline_seconds=.3,pulse_hold=.35,thrust_amplitude=.015)
        n.data={'state':(State(connected=True,armed=True,mode='OFFBOARD'),time.monotonic()),
                'pose':(n.reference,time.monotonic()),'velocity':(TwistStamped(),time.monotonic()),'imu':(Imu(),time.monotonic())}
        n.ready=lambda:True;n.stable=lambda:True;n.exclusive=lambda:True;n.tilt=lambda:0.
        n.last_authority_check=time.monotonic();n.hover_samples=collections.deque();n.hover_source='feedback'
        for name in ('position','command','status','state_pub','events'):setattr(n,name,Mock())
        n.set_mode=Mock()
    def tearDown(self):self.clock.stop();self.log.stop()

    def test_last_pulse_recovers_before_complete(self):
        n=self.node;n.tick(10.)
        self.assertEqual(n.phase,'recover');self.assertFalse(n.sequence_complete)
        n.settled=10.;n.tick(13.)
        self.assertEqual(n.phase,'complete');self.assertTrue(n.sequence_complete)
        n.set_mode.assert_not_called()

    def test_sim_landing_waits_for_actual_disarm(self):
        n=self.node;n.phase='complete';n.sequence_complete=True;n.auto=True
        n.tick(10.);self.assertEqual(n.phase,'landing');n.set_mode.assert_called_with('AUTO.LAND')
        n.data['state'][0].mode='AUTO.LAND';n.tick(11.);self.assertEqual(n.phase,'landing')
        n.data['state'][0].armed=False;n.tick(12.);self.assertEqual(n.phase,'done')

    def test_mode_exit_aborts_without_commands_or_reentry(self):
        n=self.node;n.data['state'][0].mode='POSCTL';n.tick(10.)
        self.assertEqual(n.phase,'fault');self.assertFalse(n.sequence_complete)
        n.tick(11.);n.command.publish.assert_not_called();n.position.publish.assert_not_called();n.set_mode.assert_not_called()

    def test_waiting_for_hover_feedback_keeps_position_stream(self):
        n=self.node;n.phase='recover';n.settled=1.;n.tick(10.)
        self.assertEqual(n.phase,'recover');n.position.publish.assert_called_once();n.status.publish.assert_called_once()

    def test_completion_then_manual_takeover_finishes(self):
        n=self.node;n.phase='complete';n.sequence_complete=True;n.data['state'][0].mode='POSCTL'
        n.tick(10.);self.assertEqual(n.phase,'done');n.command.publish.assert_not_called()

    def test_illegal_transition_rejected(self):
        with self.assertRaises(ValueError):self.node.transition('done','skip recovery')

    def test_conflicting_controller_withdraws_hold_even_after_completion(self):
        n=self.node;n.phase='complete';n.sequence_complete=True;n.exclusive=lambda:False;n.last_authority_check=0.
        n.tick(10.);self.assertEqual(n.phase,'fault');self.assertIsNone(n.reference)
        n.position.publish.assert_not_called();n.command.publish.assert_not_called()


class EntryTest(unittest.TestCase):
    def test_both_script_names_share_module_entry(self):
        from holydrone_identification.fsm import main
        scripts=Path(__file__).resolve().parents[1]/'scripts'
        for name in ('excite.py','identification_fsm.py'):
            self.assertIs(runpy.run_path(str(scripts/name))['main'],main)
