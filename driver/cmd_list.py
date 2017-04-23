CMD_LIST = {
    'echo': 0x01, #expected parameters: char[4] = 'ECHO'
    'setCoordinates': 0x02,   #expected parameters: float32[3]
    'dutyCycle': 0x03,    #expected parameters: int[1], float32[1]
    'setDirectionBit': 0x04, #expected parameters: int[1]
    'removeDirectionBit': 0x05,   #expected parameters: int[1]
    'setMotorVoltage': 0x06,  #expected parameters: int[1], float32[1]
    'setPidParameters': 0x08, #expected parameters: float32[3]
    'setRotatoinSpeed': 0x09, #expected parameters: float32[4]
    'switchOnKinematicCalculation': 0xB,
    'switchOffKinematicCalculation': 0xC, 
    'setMovementSpeed': 0xD, #expected parameters: float32[3]
    'switchOnTrajectoryRegulator': 0xE,
    'switchOffTrajectoryRegulator': 0xF,
    'cleanPointsStack': 0x10,
    'addPointToStack': 0x11,  #expected parameters: float32[3], int[1]
    'getStackState': 0x12,
    'getCurentCoordinates': 0x13, # todo Current
    'getCurrentSpeed': 0x14,
    'setMovementParameters': 0x15,    #expected parameters: float32[5]
    'setADCPinMode': 0x16,    #expected parameters: int[1], int[1]
    'getADCPinState': 0x17,   #expected parameters: int[1]
    'getAllADCPinsStet': 0x18,
    'getDigitalPinState': 0x19,   #expected parameters: int[1]
    'getAllDigitalPinState': 0x1a,
    'setOutputState': 0x1b,   #expected parameters: int[1]
    'getPinMode': 0x1c,   #expected parameters: int[1]
    'setPinModeExit': 0x1d,   #expected parameters: int[2]
    'getDiscretePinState': 0x1e,  #expected parameters: int[1]
    'getAllDiscretePinStates': 0x1f,
    'setDiscreteOutputState': 0x20,   #expected parameters: int[1]
    'determineCurrentPinMode': 0x21,  #expected parameters: int[1]
    'set12VState': 0x22, #expected parameters: int[1]
    'switchOffPid': 0x23,
    'switchOnPid': 0x24,
    'stopAllMotors': 0x29,
    'setCorectCoordinates': 0x25,
    # TODO': laying field side
    # TODO: beginning of the competition sign
    # TODO: implement commands listed below 
    #getManipulatorState = 0x26
    #changeSuckerState = 0x27   #expected parameters: int[1]
    #uploadPuck = 0x28
    #unloadAllPucks = 0x29  #expected parameters: int[1]
    'changeFishingRodState': 0x30,   #expected parameters: int[1]
    'changeFishingLatchState': 0x2A, #expected parameters: int[1]
    'openCubeCollector': 0x2B,
    'closeCubeCollector': 0x2C,
    'setManipulatorAngle': 0x31, # expected parameter: float[1]
    'releaseCubeMoovers': 0x2D,
    'raiseCubeMoovers': 0x2E,
    'switchOnVibrationTable': 0x2F,
    'switchOffVibrationTable': 0x30,
    'switchOnBelts': 0x32,
    'switchOffBelts': 0x33,
    'startGame': 0x34,
    'openCubeBorder': 0x2D,
}

REVERSED_CMD_LIST = dict((v,k) for k, v in  CMD_LIST.items())
