import serial
import time
import numpy as np
import csv   
import random

# ROS imports
import rospy
from ti_mmwave_rospkg.msg import RadarScan
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from math import pi
from math import atan

# Global for reusing old value if new is not good enough
# Initialized to 1 [meter] as the pile is positioned one meter from the radar
bestX = 1

# Initialize as global for logging
# fields = [rospy.get_time, x1, x2, x3, closestValue, bestX, msg.x, msg.y, msg.z, msg.range, msg.velocity, msg.bearing, msg.intensity]
fields = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


# ROS joint state class
class CommandToJointState:
    def __init__(self):
        self.joint_state = JointState()
        self.joint_state.name.append("base_to_pipe_x_radar")
        self.joint_state.name.append("base_to_pipe_y_radar")
        self.joint_state.name.append("base_to_pipe_z_radar")
        rospy.loginfo("Publishing joint_states for " + str(self.joint_state.name))
        self.joint_state.position.append([0.0, 0.0, 0.0])
        self.joint_state.velocity.append(0.0)
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.command_sub = rospy.Subscriber("/ti_mmwave/radar_scan", RadarScan, self.command_callback, queue_size=1)



    def command_callback(self, msg):
        # Filter based on  x value and velocity 
        global fields
        global bestX
        if msg:

            # Calculate position with atan(x/ pipe length)
            # msg.x-1 : 1 meter is subtracted as the pile is placed 1 meter from the radar
            self.joint_state.position = (atan(msg.y/0.5), bestX, 0)
            rospy.loginfo("thetaX:" + str(-atan((msg.x-1)/0.5)*(180/pi)) + " \tthetaY: " + str(atan(msg.y/0.5)*(180/pi)) + " \t\tIntensity: " + str(msg.intensity) + " point_id: " + str(msg.point_id) + " velocity: " + str(msg.velocity))
            #rospy.loginfo_throttle(1, "X: " + str(msg.x) + " Y: " + str(msg.y))

            # Update logging fields
            # Fields are written to file on 6843 update, not 1843 update
            fields = [fields[0], fields[1], fields[2], fields[3], fields[4], fields[5], msg.x, msg.y, msg.z, msg.range, msg.velocity, msg.bearing, msg.intensity]

            self.joint_state.header.stamp = rospy.Time.now()
            # Publish to topic
            self.joint_pub.publish(self.joint_state)





# Change the configuration file name
configFileName = 'high_accuracy_demo_68xx.cfg'

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;





# ------------------------------------------------------------------

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport

    # Linux
    CLIport = serial.Serial('/dev/ttyUSB0', 115200)
    Dataport = serial.Serial('/dev/ttyUSB1', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            #freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                
            digOutSampleRate = int(splitWords[11]);
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = int(splitWords[5]);

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    #configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeResolutionMeters"] = 1
    #configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["rangeIdxToMeters"] = 1
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    #configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxRange"] = 1
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters
   
# ------------------------------------------------------------------

# Funtion to read and parse the incoming data
def readAndParseData14xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength
    
    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12;
    BYTE_VEC_ACC_MAX_SIZE = 2**15;
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1;
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2;
    maxBufferSize = 2**15;
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}

    x1 = 0
    x2 = 0
    x3 = 0

    global bestX
    global fields
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
               
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    #print(f"magicOK = {magicOK}")

    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]
        
        # Initialize the pointer index
        idX = 0
        
        # Read the header
        magicNumber = byteBuffer[idX:idX+8]
        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        #idX += 4
        #print(f"magicNumber = {magicNumber} \t version = {version} \t totalPacketLen = {totalPacketLen} \t platform = {platform} \t frameNumber = {frameNumber} ")
        #print(f"timeCpuCycles = {timeCpuCycles} \t\t numDetectedObj = {numDetectedObj} \t numTLVs = {numTLVs} \t\t idX = {idX}")
        #np.savetxt("bytes.txt", byteBuffer)        
        
        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            #print(f"tlv_type = {tlv_type} \t MMWDEMO_UART_MSG_DETECTED_POINTS = {MMWDEMO_UART_MSG_DETECTED_POINTS}")
            # Read the data depending on the TLV message
            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                            
                # word array to convert 4 bytes to a 16 bit number
                word = [1, 2**8]
                tlv_numObj = np.matmul(byteBuffer[idX:idX+2],word)
                idX += 2
                tlv_xyzQFormat = 2**np.matmul(byteBuffer[idX:idX+2],word)
                idX += 2
                
                # Initialize the arrays
                rangeIdx = np.zeros(numDetectedObj,dtype = 'int16')
                dopplerIdx = np.zeros(numDetectedObj,dtype = 'int16')
                peakVal = np.zeros(numDetectedObj,dtype = 'int16')
                x = np.zeros(numDetectedObj,dtype = 'int16')
                y = np.zeros(numDetectedObj,dtype = 'int16')
                z = np.zeros(numDetectedObj,dtype = 'int16')
                #print(f"tlv_numObj = {tlv_numObj}")
                for objectNum in range(numDetectedObj):
                    
                    # Read the data for each object
                    rangeIdx[objectNum] =  np.matmul(byteBuffer[idX:idX+2],word)                    
                    idX += 2
                    dopplerIdx[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2                    
                    peakVal[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2                    
                    x[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2
                    y[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2
                    z[objectNum] = np.matmul(byteBuffer[idX:idX+2],word)
                    idX += 2
                    #print(f"rangeIdx[{objectNum}] = {rangeIdx[objectNum]} \t dopplerIdx[{objectNum}] = {dopplerIdx[objectNum]} \t peakVal[{objectNum}] = {peakVal[objectNum]} \t x[{objectNum}] = {x[objectNum]} \t y[{objectNum}] = {y[objectNum]} \t z[{objectNum}] = {z[objectNum]} \t")

                    # x1: 6 bytes: rangeIDX1, rangeIDX2*256, x*65536
                    # x2: 6 bytes: peakval1, peakval2*256, y*65536
                    # x3: 6 bytes: dopplerIdx1, dopplerIdx2*256, z*65536

                    x1 = rangeIdx[objectNum] + x[objectNum]*65536
                    x2 = peakVal[objectNum] + y[objectNum]*65536
                    x3 = dopplerIdx[objectNum] + z[objectNum]*65536

                    # Multiply by 1.36 and divide by 1048576 to get number to meters

                    x1 = round((x1*1.36)/1048576, 10)
                    x2 = round((x2*1.36)/1048576, 10)
                    x3 = round((x3*1.36)/1048576, 10)


                    # Check which range value is closest to 1 meter
                    strongestPeaks = [x1, x2, x3]
                    myNumber = 1 # Select the number closest to bestX, initally 1 [meter]
                    closestValue = min(strongestPeaks, key=lambda x:abs(x-myNumber))


                    # Only use new value if is is within reasonable constraints, if not reuse old value
                    if closestValue > 1.5 or closestValue < 0.7:
                        bestX = bestX
                    else:
                        bestX =  closestValue

                    usedXrad = -atan((bestX-1)/1)

                    # CSV logging
                    fields = [rospy.get_time(), x1, x2, x3, closestValue, bestX, fields[6], fields[7], fields[8], fields[9], fields[10], fields[11], fields[12]]

                    with open(csvname, 'a') as f:
                        writer = csv.writer(f)
                        writer.writerow(fields)

                    print(f"x1: {x1} \t x2: {x2} \t x3: {x3} \t bestX:{bestX} \t Deg: {round(usedXrad*(180/pi),4)}")

                dataOK = 1
        
  
        # Remove already processed data
        if idX > 0 and byteBufferLength > idX:
            shiftSize = totalPacketLen
               
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                

    return dataOK, frameNumber, x1

# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():  
      
    # Read and parse the received data
    dataOk, frameNumber, x1 = readAndParseData14xx(Dataport, configParameters)
    rospy.sleep(0.01)


# -------------------------    MAIN   -----------------------------------------  

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)
   
   
# Main loop 
detObj = {}  
frameData = {}    
currentIndex = 0

rospy.init_node('command_to_joint_state_radar')
command_to_joint_stateX = CommandToJointState()
#rospy.spin()
rate = rospy.Rate(30)

# Initialize log

csvheader = ['rospy.get_time', 'x1', 'x2', 'x3', 'closestValue', 'bestX', 'msg.x', 'msg.y', 'msg.z', 'msg.range', 'msg.velocity', 'msg.bearing', 'msg.intensity']
csvname = str("6843_1843_datalog_" + str(random.randint(1, 999)) + ".csv")

with open(csvname, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(csvheader)


while True:
    try:
        # Update the data and check if the data is okay
        dataOk = update()

        if dataOk:
            # Store the current frame into frameData
            frameData[currentIndex] = detObj
            currentIndex += 1
        
        #time.sleep(0.033) # Sampling frequency of 30 Hz
        rate.sleep()
        
    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        CLIport.close()
        Dataport.close()
        #win.close()
        break
        
    





