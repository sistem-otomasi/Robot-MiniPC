"""Unit tests untuk robot_example package"""

import unittest


class TestRobotExample(unittest.TestCase):

    def test_import(self):
        """Test bahwa module dapat di-import"""
        from robot_example import talker
        from robot_example import listener
        self.assertIsNotNone(talker)
        self.assertIsNotNone(listener)

    def test_talker_exists(self):
        """Test bahwa RobotTalker class ada"""
        from robot_example.talker import RobotTalker
        self.assertIsNotNone(RobotTalker)

    def test_listener_exists(self):
        """Test bahwa RobotListener class ada"""
        from robot_example.listener import RobotListener
        self.assertIsNotNone(RobotListener)


if __name__ == '__main__':
    unittest.main()
