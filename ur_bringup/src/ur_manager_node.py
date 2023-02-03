#!/usr/bin/python3

import subprocess

import rospy

from std_srvs.srv import Trigger

from ur_dashboard_msgs.srv import GetRobotMode, IsInRemoteControl, Load, LoadRequest


class URManagerNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        # -------------------------------
        #   Service clients
        # -------------------------------

        self._quit_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        self._connect_service = rospy.ServiceProxy(
            '/ur_hardware_interface/dashboard/connect', Trigger
        )
        self._power_on_service = rospy.ServiceProxy(
            '/ur_hardware_interface/dashboard/power_on', Trigger
        )
        self._e_stop_reset_service = rospy.ServiceProxy('/panther/hardware/e_stop_reset', Trigger)
        self._brake_release_service = rospy.ServiceProxy(
            '/ur_hardware_interface/dashboard/brake_release', Trigger
        )
        self._unlock_protective_stop_service = rospy.ServiceProxy(
            '/ur_hardware_interface/dashboard/unlock_protective_stop', Trigger
        )
        self._play_service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)

        self._get_robot_mode_service = rospy.ServiceProxy(
            '/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode
        )
        self._load_program_service = rospy.ServiceProxy(
            '/ur_hardware_interface/dashboard/load_program', Load
        )
        self._is_in_remote_control_service = rospy.ServiceProxy(
            '/ur_hardware_interface/dashboard/is_in_remote_control', IsInRemoteControl
        )

        self._ur_start_sequence()

    def _ur_start_sequence(self):

        rospy.loginfo(f'[{rospy.get_name()}] Calling quit service')
        while not self._call_trigger_service(self._quit_service) and not rospy.is_shutdown():
            rospy.logwarn(f'[{rospy.get_name()}] Retrying')
            rospy.sleep(1.0)
        rospy.loginfo(f'[{rospy.get_name()}] Calling connect service')
        while not self._call_trigger_service(self._connect_service) and not rospy.is_shutdown():
            rospy.logwarn(f'[{rospy.get_name()}] Retrying')
            rospy.sleep(1.0)

        rospy.loginfo(f'[{rospy.get_name()}] Calling is_in_remote_control service')
        try:
            self._is_in_remote_control_service.wait_for_service(5.0)
            res = self._is_in_remote_control_service.call()
            rospy.loginfo(
                f'[{rospy.get_name()}] called {self._is_in_remote_control_service.resolved_name} service. Response: {res.answer}'
            )
            if not res.in_remote_control:
                rospy.logerr(
                    f'[{rospy.get_name()}] UR robot not in remote control mode. Shutting down manager'
                )
                subprocess.call(['rosnode', 'kill', '/ur_hardware_interface'])
                rospy.signal_shutdown('UR robot not in remote control mode')
                return
        except (rospy.exceptions.ROSException, rospy.service.ServiceException) as err:
            rospy.logerr(f'[{rospy.get_name()}] {err}')

        rospy.loginfo(f'[{rospy.get_name()}] Calling power_on service')
        while not self._call_trigger_service(self._power_on_service) and not rospy.is_shutdown():
            rospy.logwarn(f'[{rospy.get_name()}] Retrying')
            rospy.sleep(1.0)

        rospy.loginfo(f'[{rospy.get_name()}] Calling get_robot_mode service')
        mode = 0
        while not mode == 5 and not rospy.is_shutdown():
            if mode == 7:
                break
            try:
                self._get_robot_mode_service.wait_for_service(5.0)
                res = self._get_robot_mode_service.call()
                mode = res.robot_mode.mode
                if res.success:
                    rospy.loginfo(
                        f'[{rospy.get_name()}] Successfuly called {self._get_robot_mode_service.resolved_name} service. Response: {res.answer}'
                    )
                else:
                    rospy.logerr(
                        f'[{rospy.get_name()}] Failed to called {self._get_robot_mode_service.resolved_name} service. Response: {res.answer}'
                    )
            except (rospy.exceptions.ROSException, rospy.service.ServiceException) as err:
                rospy.logerr(f'[{rospy.get_name()}] {err}')
            rospy.sleep(1.0)

        rospy.loginfo(f'[{rospy.get_name()}] Calling e_stop_reset service')
        while (
            not self._call_trigger_service(self._e_stop_reset_service) and not rospy.is_shutdown()
        ):
            rospy.logwarn(f'[{rospy.get_name()}] Retrying')
            rospy.sleep(1.0)
        rospy.loginfo(f'[{rospy.get_name()}] Calling brake_release service')
        while (
            not self._call_trigger_service(self._brake_release_service) and not rospy.is_shutdown()
        ):
            rospy.logwarn(f'[{rospy.get_name()}] Retrying')
            rospy.sleep(1.0)
        rospy.loginfo(f'[{rospy.get_name()}] Calling unlock_protective_stop service')
        while (
            not self._call_trigger_service(self._unlock_protective_stop_service)
            and not rospy.is_shutdown()
        ):
            rospy.logwarn(f'[{rospy.get_name()}] Retrying')
            rospy.sleep(1.0)

        rospy.logerr(f'[{rospy.get_name()}] Calling load_program service')
        while not rospy.is_shutdown():
            try:
                self._load_program_service.wait_for_service(5.0)
                req = LoadRequest()
                req.filename = 'husarion_ext_control.urp'
                res = self._load_program_service.call(req)
                rospy.loginfo(
                    f'[{rospy.get_name()}] called {self._get_robot_mode_service.resolved_name} service. Response: {res.answer}'
                )
                break
            except (rospy.exceptions.ROSException, rospy.service.ServiceException) as err:
                rospy.logerr(f'[{rospy.get_name()}] {err}')
            rospy.sleep(1.0)

        rospy.loginfo(f'[{rospy.get_name()}] Calling play service')
        self._call_trigger_service(self._play_service)
        # Retrying was comented out due to service always returning failure even when succeded
        # while not self._call_trigger_service(self._play_service) and not rospy.is_shutdown():
        #     rospy.logwarn(f'[{rospy.get_name()}] Retrying')

        rospy.loginfo(f'[{rospy.get_name()}] UR robot start sequence completed. Killing node')
        rospy.signal_shutdown('Finished')

    def _call_trigger_service(self, service: rospy.ServiceProxy) -> None:
        try:
            service.wait_for_service(5.0)
            res = service.call()
            if res.success:
                rospy.loginfo(
                    f'[{rospy.get_name()}] Successfuly triggered {service.resolved_name} service. Response: {res.message}'
                )
                return True
            else:
                rospy.logerr(
                    f'[{rospy.get_name()}] Failed to trigger {service.resolved_name} service. Response: {res.message}'
                )
        except (rospy.exceptions.ROSException, rospy.service.ServiceException) as err:
            rospy.logerr(f'[{rospy.get_name()}] {err}')

        return False


def main():
    ur_manager_node = URManagerNode('ur_manager_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
