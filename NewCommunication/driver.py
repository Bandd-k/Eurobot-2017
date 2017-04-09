from multiprocessing import Process, Queue
from multiprocessing.queues import Queue as QueueType
import serial
from serial.tools import list_ports
from cmd_list import CMD_LIST
import time
from packets import encode_packet, decode_packet
import logging


#<<<<<<< HEAD
#PORT_VID = 1155
#PORT_PID = 22336
#PORT_SNR = '3677346C3034'
#DEVICE_NAME = '/dev/ttyACM6'
#=======
PORT_SNR = '325936843235'
DEVICE_NAME = '/dev/ttyACM0'
#>>>>>>> 4ead14e6dbd7bdcaae48a8ba2a886a9ec203a0d3

class DriverException(Exception):
    pass


# code is sensitive to the types, If input Float use 1.0 not 1!
# message format:
#   {'source': <'fsm' or 'localization'>,
#   'cmd': 'cmd name',
#   'params': [<parameters>]}
# reply format: {'cmd': 'cmd name', 'data': <reply data>}

# You can test it without spawning process:
# >>> from driver import Driver
# >>> d = Driver()
# >>> d.connect()
# >>> d.process_cmd(cmd)
class Driver(Process):
    """Class for communications with STM32. Only one instance can exist at a time.
    !!!warning!!! command parameters are type sensitive, use 0.0 not 0 if
    parameter must be float
    Examples
    -------
    >>> d = Driver()
    >>> # send command in the blocking mode
    >>> d.process_cmd('setCoordinates', [0.0, 0.0, 0.0])
    """


    def __init__(self,inp_queue,fsm_queue,loc_queue, baudrate=9600, timeout=0.5, device=DEVICE_NAME, connect=True, **kwargs):
        super(Driver, self).__init__(**kwargs)
        self.device = device
        self.port = None
        self.baudrate = baudrate
        self.timeout = timeout
        self.input_cmd_queue = inp_queue
        self.output_queues = {'fsm':fsm_queue,'loc':loc_queue}
        if connect:
            self.connect()

    def connect(self):
        """Connect to STM32 using serial port"""
        for port in list_ports.comports():
            if (port.serial_number == PORT_SNR):
                self.device = port.device
                break
        self.port = serial.Serial(self.device,
                                  baudrate=self.baudrate, timeout=self.timeout)

    def close(self):
        '''Close serial port'''
        self.port.close()
        self.port = None

    def process_cmd(self, cmd, params=None):
        '''Process command in the blocking mode
        Parameters
        ----------
        cmd: string
            Command name (see CMD_LIST)
        params: list, optional
            List of command parameters
        '''
        cmd_id = CMD_LIST[cmd]
        logging.critical(cmd_id)
        packet = encode_packet(cmd_id, params)
        logging.debug('data_to_stm:' + ','.join([str(i) for i in packet]))
        self.port.write(packet)
        data = self.port.read(size=3)
        if len(data) != 3:
            self.port.reset_input_buffer()
            self.port.reset_output_buffer()
            logging.critical('Couldn\'t read 3 bytes')
            return {'cmd':'cylinder staff','data':'ok'}
        data = bytearray(data)
        data += self.port.read(size=int(data[2]) - 3)
        return decode_packet(data)

    def register_output(self, name, queue):
        '''Register output queue. Must be called before run()'''
        if not isinstance(queue, QueueType):
            raise TypeError('Wrong type for queue')
        self.output_queues[name] = queue

    def run(self):
        #{'source':'fsm','cmd':'SetCoordinates','params':[0,0,0]}
        try:
            while True:
                cmd = self.input_cmd_queue.get()
                if cmd is None:
                    break
                source = cmd.get('source')
                reply = self.process_cmd(cmd.get('cmd'),cmd.get('params'))
                output_queue = self.output_queues.get(source)
                if output_queue is not None:
                    output_queue.put(reply)
                else:
                    raise DriverException('Incorrect source')
        finally:
            self.close()
