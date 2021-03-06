from multiprocessing import Process, Queue
from multiprocessing.queues import Queue as QueueType
import serial
from serial.tools import list_ports
from .cmd_list import REVERSED_CMD_LIST
from .cmd_list import CMD_LIST
from .exceptions import DriverException
import time
from .packets import encode_packet, decode_packet


PORT_VID = 1155
PORT_PID = 22336
PORT_SNR = '3677346C3034'
DEVICE_NAME = '/dev/ttyACM0'

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
    '''Class for communications with STM32. Only one instance can exist at a time.

    !!!warning!!! command parameters are type sensitive, use 0.0 not 0 if
    parameter must be float

    Examples
    -------
    >>> d = Driver()
    >>> # send command in the blocking mode
    >>> d.process_cmd('setCoordinates', [0.0, 0.0, 0.0])
    >>> # register queue and start process
    >>> d.register_queue('queue_name', queue)
    >>> d.start()
    '''

    output_queues = None
    input_cmd_queue = None

    def __init__(self, baudrate=9600, timeout=0.5, device=DEVICE_NAME, connect=True, **kwargs):
        super(Driver, self).__init__(**kwargs)
        self.device = device
        self.port = None
        self.baudrate = baudrate
        self.timeout = timeout
        self.input_cmd_queue = Queue()
        self.output_queues = {}
        if connect:
            self.connect()

    def connect(self):
        '''Connect to STM32 using serial port'''
        for port in list_ports.comports():
            if (port.serial_number == PORT_SNR) and \
                    (port.pid == PORT_PID) and (port.vid == PORT_VID):
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
        packet = encode_packet(cmd_id, params)
        self.port.write(packet)
        data = self.port.read(size=3)
        if len(data) != 3:
            self.port.reset_input_buffer()
            self.port.reset_output_buffer()
            raise DriverException('Couldn\'t read 3 bytes')
        data = bytearray(data)
        data += self.port.read(size = int(data[2])-3)
        decod_data = decode_packet(data)
        return decod_data

    def register_output(self, name, queue):
        '''Register output queue. Must be called before run()'''
        if not isinstance(queue, QueueType):
            raise TypeError('Wrong type for queue')
        self.output_queues[name] = queue

    def run(self):
        if not self.output_queues:
            raise DriverException('Zero output queues were registered')
        self.connect()
        try:
            while True:
                cmd = self.input_cmd_queue.get()
                if cmd is None:
                    break
                source = cmd.get('source')
                reply = self.process_cmd(cmd)
                output_queue = self.output_queues.get(source)
                if output_queue is not None:
                    output_queue.put(reply)
                else:
                    raise DriverException('Incorrect source')
        finally:
            self.close()
