import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib import colors as mcolors

Robot_Send_CMDs = {}
Robot_RSP_CMDs = {}
RobotTimeDelays = []
RobotDataSize = []
RobotTimeDelayPerKB = []

Human_Send_CMDs = {}
Human_RSP_CMDs = {}
HumanTimeDelays = []
HumanDataSize = []
HumanTimeDelayPerKB = []

allTimeDelayPerKB = []
start_time_stamp = 1550718810
## for plotting
AllSendPointCollection = [] # draw send points
AllRSPPointCollection = [] # draw RSP points
RobotLineCollection = [] # draw delay line
RobotLineColor = (0, 0, 1, 1)  # R, G, B, A, Blue
RobotLineColors = []
HumanLineCollection = []
HumanLineColor = (1, 0, 0, 0.5)  # gray
HumanLineColors = []
scale = 1
def add2Dictionary(lineStr, h_r_mark):
    print(lineStr)
    lineArray = lineStr.split(",")
    if len(lineArray) is 5:
        #print("5")
        cmd_index = int(lineArray[1])
        cmd_size = float(lineArray[3]) / (1000 *8) # convert to KB
        time_stamp = (float(lineArray[4]) - start_time_stamp)
        if lineArray[0].find("SEND") is not -1:
            #print("send")
            if h_r_mark is 1:# human
                Human_Send_CMDs[cmd_index]=(time_stamp, cmd_size)
            else:
                Robot_Send_CMDs[cmd_index]=(time_stamp, cmd_size)
        elif lineArray[0].find("RESP") is not -1:
            #print("RESP")
            if h_r_mark is 1:# human
                Human_RSP_CMDs[cmd_index]=(time_stamp, cmd_size)
            else:
                Robot_RSP_CMDs[cmd_index]=(time_stamp, cmd_size)

def processData():  # process all data
    robot_cmd_length = len(Robot_Send_CMDs)
    human_cmd_length = len(Human_Send_CMDs)
    for i in range(robot_cmd_length):
        SendPoint = Robot_Send_CMDs[i+1]
        RSPPoint = Robot_RSP_CMDs[i+1]
        timeDelay = RSPPoint[0] - SendPoint[0]
        datasize = SendPoint[1]
        delayPerKB = timeDelay / datasize
        if timeDelay < 0.4 * scale:
            RSPPoint = (SendPoint[0] + 0.4 * scale, RSPPoint[1])  # make it visible in graph
        RobotTimeDelays.append(timeDelay)
        RobotDataSize.append(datasize)
        if delayPerKB < 1: # tick out wrong measurements
            RobotTimeDelayPerKB.append(delayPerKB)
        AllSendPointCollection.append(SendPoint)
        AllRSPPointCollection.append(RSPPoint)
        lineSeg = [SendPoint, RSPPoint]
        RobotLineCollection.append(lineSeg)
        RobotLineColors.append(RobotLineColor)

    for i in range(human_cmd_length):
        SendPoint = Human_Send_CMDs[i+1]
        RSPPoint = Human_RSP_CMDs[i+1]
        timeDelay = RSPPoint[0] - SendPoint[0]
        datasize = SendPoint[1]
        delayPerKB = timeDelay / datasize
        if timeDelay < 1 * scale:
            RSPPoint = (SendPoint[0] + 0.4 * scale, RSPPoint[1])  # make it visible in graph
        if datasize < 1000:
            datasize = datasize * 100
            SendPoint = (SendPoint[0], datasize)
            RSPPoint = (RSPPoint[0], datasize)

        HumanTimeDelays.append(timeDelay)
        HumanDataSize.append(datasize)
        if delayPerKB < 1:
            HumanTimeDelayPerKB.append(delayPerKB)
        AllSendPointCollection.append(SendPoint)
        AllRSPPointCollection.append(RSPPoint)
        lineSeg = [SendPoint, RSPPoint]
        HumanLineCollection.append(lineSeg)
        HumanLineColors.append(HumanLineColor)

    allTimeDelayPerKB = RobotTimeDelayPerKB + HumanTimeDelayPerKB
    return np.mean(allTimeDelayPerKB), np.mean(RobotTimeDelayPerKB), np.mean(HumanTimeDelayPerKB)


#  read robot file
with open("3G_robot.txt", "r") as fileHandler:
    # Read next line
    line = fileHandler.readline()
    # check line is not empty
    while line:
        add2Dictionary(line.strip(), 0)
        line = fileHandler.readline()

#  read robot file
with open("3G_human.txt", "r") as fileHandler:
    # Read next line
    line = fileHandler.readline()
    # check line is not empty
    while line:
        add2Dictionary(line.strip(), 1)
        line = fileHandler.readline()

all_d, r_d, h_d = processData()
print("all time delay per KB:" + str(all_d))
print("Robot time delay per KB:" + str(r_d))
print("Human time delay per KB:" + str(h_d))

plt.style.use('ggplot')
# lcRobot = LineCollection(RobotLineCollection, colors=RobotLineColors, linewidths=10)
lcHuman = LineCollection(HumanLineCollection, colors=HumanLineColors, linewidths=20)
fig, ax = plt.subplots()
ax.set_xlim(0, 600 * scale)
ax.set_ylim(0, 350)
# ax.add_collection(lcRobot)
# ax.add_collection(lcHuman)

for i in range(len(RobotLineCollection)):
    x = np.array(RobotLineCollection[i])
    if i == len(RobotLineCollection) - 1:
        ax.fill_between(x[:,0], x[:,1], facecolor="green", alpha = 0.5, interpolate=True, label="Robot Side Process")
    else:
        ax.fill_between(x[:,0], x[:,1], facecolor="green", alpha = 0.5, interpolate=True)

for i in range(len(HumanLineCollection)):
    x = np.array(HumanLineCollection[i])
    if i == len(HumanLineCollection) - 1:
        ax.fill_between(x[:,0], x[:,1], facecolor="red", alpha = 0.5, interpolate=True, label = "Human Side Process")
    else:
        ax.fill_between(x[:,0], x[:,1], facecolor="red", alpha = 0.5, interpolate=True)
# ax.autoscale()
# ax.margins(0.1)
allSends = np.array(AllSendPointCollection)
allRSPs = np.array(AllRSPPointCollection)
plt.scatter(allSends[:,0], allSends[:,1], s=10, c="b", label="SEND") ## marker = "o"
plt.scatter(allRSPs[:,0], allRSPs[:,1], s=10, c=(0.84,0.07,0.49,1), label="RECV RESPONSE")
plt.xlabel("time (ms)")
plt.ylabel("datagram size (KB)")
plt.legend(loc='upper left')
plt.show()
