import serial
import time
import packetParser


class SerialWrapper(object):
    def __init__(self, portName):
        self.SerialPort = serial.Serial(portName, baudrate=9600, timeout=1)
        self.name = self.SerialPort.name

    # send data without checking answer
    def sendData(self, dataToSend):
        self.SerialPort.write(dataToSend)

    def recieveData(self):
        return self.SerialPort.readline()

    # send data and recieve answer
    def sendRequest(self, dataToSend, timeout=0.01):
        # read all previous answers (clean buffer)
        self.SerialPort.flushInput()
        self.SerialPort.flushOutput()
        print [c for c in dataToSend]
        self.SerialPort.write(dataToSend)
        requestRecieved = False
        recievedPacket = ""

        startT1 = time.time()
        while not requestRecieved:  # and (time.time() - sendRequestTime) < timeout):
            bytesToRead = self.SerialPort.inWaiting()
            # print 'First bytes: ', bytesToRead
            if bytesToRead > 0:
                time.sleep(0.01)
                bytesToRead2 = self.SerialPort.inWaiting()
                # print 'Second bytes: ', bytesToRead2
                # if bytesToRead2 != bytesToRead:
                # print 'Difference: ', (bytesToRead2 - bytesToRead)
                # raise Exception('Too Slow')
                recievedData = self.SerialPort.read(bytesToRead2)
                # print 'Data: ', recievedData
                if len(recievedData) is not 0:
                    recievedPacket = packetParser.ParsePacket(recievedData)
                    requestRecieved = True

        if recievedPacket == "" or requestRecieved == False:
            raise Exception("Reply was not recieved")
        return recievedPacket

    def __del__(self):
        self.SerialPort.close()
