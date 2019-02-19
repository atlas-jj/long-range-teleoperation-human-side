#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/carv/script', String, queue_size=1000)
    rospy.init_node('UDP_Server_teleop', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    raw_input('Press enter to continue: ')
    f = open("chris_CARV_Files_large", "r")
    script_block = ""
    enter_block = False
    cmdCount = 0
    while not rospy.is_shutdown():
        script = f.readline().rstrip("\n")
        script_block = script_block + script +"@"

        if script.endswith("{"):
            enter_block = True
        elif script.endswith("}"):
            enter_block = False

        if enter_block is False:
            script_block = script_block[:-1]
            pub.publish(script_block)
            cmdCount += 1
            print(str(cmdCount) + "  "+ script_block)
            script_block = ""
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
