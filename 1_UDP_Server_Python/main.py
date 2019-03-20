#!/usr/bin/env python
import rospy
from datetime import datetime
import pytz
from std_msgs.msg import String
import os.path
import socket


class fileLogger:
    def __init__(self):
        self.logFileName = "log_" + str(datetime.now()) + ".txt"
        self.f = open(self.logFileName, "w")
        self.f.write("UDP Human Server Side Started ......\n")
        self.f.close()

    def to_timestamp(self):
        a_date = datetime.now()
        if a_date.tzinfo:
            epoch = datetime(1970, 1, 1, tzinfo=pytz.UTC)
            diff = a_date.astimezone(pytz.UTC) - epoch
        else:
            epoch = datetime(1970, 1, 1)
            diff = a_date - epoch
        return float(diff.total_seconds()*1000)

    def close(self):
        self.f.close()

    def log(self, msg):
        self.f = open(self.logFileName, "a")
        currentTimeStamp = self.to_timestamp()
        time_str = "{:.3f}".format(currentTimeStamp * 0.001 + 25200)
        self.f.write(msg + "," + time_str + "\n")
        self.f.close()


## "103.125.217.74"
## "127.0.0.1
class UDP_Server:
    def __init__(self, logger, localIP = "103.125.217.74", localPort = 30002, bufferSize = 8000, endTag = "clc"):
        self.logger = logger
        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPServerSocket.bind((localIP, localPort))
        self.bufferSize = bufferSize
        self.clientAddress = None
        self.endTag = endTag
        self.recvBuffer = ""
        print("UDP server up and listening, local ip: " + localIP + ", port: " + str(localPort))

    def calcBytes(self, str):
        return len(str) * 8

    def recv(self):
        # loop until find end tag
        endFlag = False
        rtStr = ""
        while endFlag is False:
            (msg, self.clientAddress) = self.UDPServerSocket.recvfrom(self.bufferSize)
            self.recvBuffer = self.recvBuffer + msg
            if self.recvBuffer.find(self.endTag) is not -1: #find end tag
                rtStr = self.recvBuffer.replace(self.endTag, "")
                endFlag = True
                self.recvBuffer = ""
        clientIP = "Client IP Address:{}".format(self.clientAddress)
        print("recvr from :" + clientIP)
        return rtStr, self.calcBytes(rtStr+self.endTag)

    def send(self, toSendStr):
        if self.clientAddress is None:
            return
        toSendStr = toSendStr + self.endTag
        bytesToSend = str.encode(toSendStr)
        self.UDPServerSocket.sendto(bytesToSend, self.clientAddress)
        print("send str")

    def send_log(self, toSendStr, logStr):
        self.send(toSendStr)
        datasize = self.calcBytes(toSendStr + self.endTag)
        self.logger.log("SEND," + logStr.upper() + "," + str(datasize))


class ROS_talker:
    def __init__(self, logger, server):
        self.logger = logger
        self.server = server
        self.pub_carv = rospy.Publisher('/carv/script', String, queue_size=1000)
        self.pub_response = rospy.Publisher('/udp/response', String, queue_size=1000)
        raw_input('Press enter to continue: ')
        rospy.init_node('UDP_Server_teleop', anonymous=True)

        rospy.Subscriber("/udp/request", String, self.callback)

    def callback(self, msg):
        tag = msg.data.split(":")[0]
        print("udp request received! tag: " + tag)
        self.server.send_log(msg.data, tag)

    def processReceived(self, strReceived, datasize):  # strReceived endTag already removed
        """
        cmd_index#CARV#data   COARSE_DONE#data  FINE_DONE#data  RQ_IMAGE#data
        :param strReceived: endTag already removed
        :return:
        """
        print(strReceived)
        if strReceived.find("test_conn") is not -1: # reply connection heartbeat
            self.server.send("echo_test")
            return

        strArray = strReceived.split('#')
        if len(strArray) is not 3:
            print("not our command, ignore...")
            print(strReceived + " .bye.")
            return

        cmd_index = strArray[0]
        tag = strArray[1]
        data = strArray[2]
        print("data  " + data)
        print("tag  " + tag)
        if tag.find("CARV") is not -1:
            print("pub carv")
            # response with
            self.server.send(cmd_index+":RESP:CARV:" + str(datasize))
            self.pub_carv.publish(data)
        elif tag.find("COARSE_DONE") is not -1:
            self.server.send(cmd_index+":RESP:COARSE_DONE:" + str(datasize))
            self.pub_response.publish("coarse_target:0:done")
        elif tag.find("FINE_DONE") is not -1:
            self.server.send(cmd_index+":RESP:FINE_DONE:" + str(datasize))
            self.pub_response.publish("fine_target:0:done")
        elif tag.find("RQ_IMAGE") is not -1:
            self.server.send(cmd_index+":RESP:RQ_IMAGE:" + str(datasize))
            self.pub_response.publish("target_image:0:" + data)

        print("RECRV," + tag.upper() + "," + str(datasize))
        self.logger.log("RECRV," + tag.upper() + "," + str(datasize))

    def run(self):
        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():  # recv and rospin callback
            recvdStr, datasize= self.server.recv()
            self.processReceived(recvdStr, datasize)
            rate.sleep()
        #rospy.spin()


if __name__ == '__main__':

    try:
        logger = fileLogger()
        server = UDP_Server(logger)
        talker = ROS_talker(logger, server)
        talker.run()

    except rospy.ROSInterruptException:
        pass
