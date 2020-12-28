#!/usr/bin/env python
class BaseNode:
    def __init__(self):
        pass

    def wait_for_simulator(self):
        # Wait for the simulator to be ready. If simulator is not ready, then time will be stuck at zero
        while rospy.Time.now().to_sec() == 0:
            rate.sleep()

  def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

    def shutdown_hook(self):
        self.shutdown_requested = True
        print("\n**** Shutdown Requested ****")
        self.stop()

    def main_loop(self):
        pass

    def pre_loop(self):
        pass

    def run(self):
        rate = rospy.Rate(self.hz)
        self.pre_loop()
        while not rospy.is_shutdown() and not self.shutdown_requested:
            self.main_loop()
            rate.sleep()
        self.stop()
        
if __name__ == '__main__':
    rn = BaseNode()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()

