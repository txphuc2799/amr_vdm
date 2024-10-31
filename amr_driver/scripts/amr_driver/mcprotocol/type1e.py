import re
import socket
import binascii
import amr_driver.mcprotocol.mcprotocolerror as mcprotocolerror
import amr_driver.mcprotocol.mcprotocolconst as const


def get_device_number(device):
    """Extract device number.

    Ex: "D1000" → "1000"
        "X0x1A" → "0x1A
    """
    device_num = re.search(r"\d.*", device)
    if device_num is None:
        raise ValueError("Invalid device number, {}".format(device))
    else:
        device_num_str = device_num.group(0)
    return device_num_str


class CommTypeError(Exception):
    """Communication type error. Communication type must be "binary" or "ascii"

    """
    def __init__(self):
        pass

    def __str__(self):
        return "communication type must be \"binary\" or \"ascii\""

class PLCTypeError(Exception):
    """PLC type error. PLC type must be "F"

    """
    def __init__(self):
        pass

    def __str__(self):
        return "plctype must be 'F'"

class Type1E:
    """mcprotocol 1E communication class.

    Attributes:
        commtype(str):          communication type. "binary" or "ascii". (Default: "binary") 
        Header:                 A header of Ethernet. Normally, it is added automatically
        pc(int):                Specify the network module station No. of an access target. (0<= pc <= 255)
        timer(int):             time to raise Timeout error(/250msec). default=4(1sec)
                                If PLC elapsed this time, PLC returns Timeout answer.
                                Note: python socket timeout is always set timer+1sec. To recieve Timeout answer.
    """
    plctype         = const.F_SERIES
    commtype        = const.COMMTYPE_BINARY
    pc              = 0xFF
    timer           = 4 # MC protocol timeout. 250msec * 4 = 1 sec 
    soc_timeout     = 2 # 2 sec
    _is_connected   = False
    _SOCKBUFSIZE    = 4096
    _wordsize       = 2 #how many byte is required to describe word value 
                        #binary: 2, ascii:4.
    _debug          = False


    def __init__(self, plctype ="F"):
        """Constructor

        """
        self._set_plctype(plctype)
    
    def _set_debug(self, debug=False):
        """Turn on debug mode
        """
        self._debug = debug

    def connect_PLC(self, ip, port):
        """Connect to PLC

        Args:
            ip (str):       ip address(IPV4) to connect PLC
            port (int):     port number of connect PLC   
            timeout (float):  timeout second in communication

        """
        self._ip = ip
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.soc_timeout)
        self._sock.connect((ip, port))
        self._is_connected = True

    def close(self):
        """Close connection

        """
        self._sock.close()
        self._is_connected = False

    def _send(self, send_data):
        """send mc protorocl data 

        Args: 
            send_data(bytes): mc protocol data
        
        """
        if self._is_connected:
            if self._debug:
                print(binascii.hexlify(send_data))
            self._sock.send(send_data)
        else:
            raise Exception("socket is not connected. Please use connect method")

    def _recv(self):
        """recieve mc protocol data

        Returns:
            recv_data
        """
        recv_data = self._sock.recv(self._SOCKBUFSIZE)
        return recv_data

    def _set_plctype(self, plctype):
        """Check PLC type. If plctype is vaild, set self.commtype.

        Args:
            plctype(str):      PLC type. "F" 

        """
        if plctype == "F":
            self.plctype = const.F_SERIES
        else:
            raise PLCTypeError()

    def _set_commtype(self, commtype):
        """Check communication type. If commtype is vaild, set self.commtype.

        Args:
            commtype(str):      communication type. "binary" or "ascii". (Default: "binary") 

        """
        if commtype == "binary":
            self.commtype = const.COMMTYPE_BINARY
            self._wordsize = 2
        else:
            raise CommTypeError()

    def _get_answerdata_index(self):
        """Get answer data index from return data byte.
        """
        if self.commtype == const.COMMTYPE_BINARY:
            return 2
        else:
            raise ValueError("Only supported binary command type, now!")

    def _get_answerstatus_index(self):
        """Get command status index from return data byte.
        """
        if self.commtype == const.COMMTYPE_BINARY:
            return 1
        else:
            raise ValueError("Only supported binary command type, now!")

    def setaccessopt(self, commtype=None, pc=None, timer_sec=None):
        """Set mc protocol access option.

        Args:
            commtype(str):          communication type. "binary" or "ascii". (Default: "binary") 
            pc(int):                network module station No. of an access target. (0<= pc <= 255)
            timer_sec(int):         Time out to return Timeout Error from PLC. 
                                    MC protocol time is per 250msec, but for ease, setaccessopt requires per sec.
                                    Socket time out is set timer_sec + 1 sec.

        """
        if commtype:
            self._set_commtype(commtype)

        if pc:
            try:
                pc.to_bytes(1, "little")
                self.pc = pc
            except:
                raise ValueError("pc must be 0 <= pc <= 255") 

        if timer_sec:
            try:
                timer_250msec = 4 * timer_sec
                timer_250msec.to_bytes(2, "little")
                self.timer = timer_250msec
                self.soc_timeout = timer_sec + 1
                if self._is_connected:
                    self._sock.settimeout(self.soc_timeout)
            except:
                raise ValueError("timer_sec must be 0 <= timer_sec <= 16383, / sec") 
        return None
    
    def _make_senddata(self, subheader, requestdata):
        """Makes send mc protorocl data.

        Args:
            requestdata(bytes): mc protocol request data. 
                                data must be converted according to self.commtype

        Returns:
            mc_data(bytes):     send mc protorocl data

        """
        mc_data = bytes()
        # subheader is big endian
        if self.commtype == const.COMMTYPE_BINARY:
            mc_data += self._encode_value(subheader, "byte")
        else:
            raise ValueError("Only supported binary command type, now!")
        mc_data += self._encode_value(self.pc, "byte")
        #add self.timer size
        mc_data += self._encode_value(self.timer, "short")
        mc_data += requestdata
        return mc_data

    # def _make_commanddata(self, command, subcommand):
    #     """make mc protocol command and subcommand data

    #     Args:
    #         command(int):       command code
    #         subcommand(int):    subcommand code

    #     Returns:
    #         command_data(bytes):command data

    #     """
    #     command_data = bytes()
    #     command_data += self._encode_value(command, "short")
    #     command_data += self._encode_value(subcommand, "short")
    #     return command_data
    
    def _make_devicedata(self, device):
        """make mc protocol device data. (device code and device number)
        
        Args:
            device(str): device. (ex: "D1000", "Y1")

        Returns:
            device_data(bytes): device data
            
        """
        
        device_data = bytes()

        devicetype = re.search(r"\D+", device)
        if devicetype is None:
            raise ValueError("Invalid device ")
        else:
            devicetype = devicetype.group(0)      

        if self.commtype == const.COMMTYPE_BINARY:
            devicecode, devicebase = const.DeviceConstants.get_binary_devicecode(self.plctype, devicetype)
            devicenum = int(get_device_number(device), devicebase)
            device_data += devicenum.to_bytes(4, "little")
            device_data += devicecode.to_bytes(2, "little")
        else:
            raise ValueError("Only supported binary command type, now!")
        return device_data

    def _encode_value(self, value, mode="short", isSigned=False):
        """encode mc protocol value data to byte.

        Args: 
            value(int):   readsize, write value, and so on.
            mode(str):    value type.
            isSigned(bool): convert as sigend value

        Returns:
            value_byte(bytes):  value data
        
        """
        try:
            if self.commtype == const.COMMTYPE_BINARY:
                if mode == "byte":
                    value_byte = value.to_bytes(1, "little", signed=isSigned)
                elif mode == "short":
                    value_byte = value.to_bytes(2, "little", signed=isSigned)
                elif mode == "long":
                    value_byte = value.to_bytes(4, "little", signed=isSigned)
                else: 
                    raise ValueError("Please input value type")
            else:
                raise ValueError("Only supported binary command type, now!")
        except:
            raise ValueError("Exceeeded Device value range")
        return value_byte

    def _decode_value(self, byte, mode="short", isSigned=False):
        """decode byte to value

        Args: 
            byte(bytes):    readsize, write value, and so on.
            mode(str):      value type.
            isSigned(bool): convert as sigend value  

        Returns:
            value_data(int):  value data
        
        """
        try:
            if self.commtype == const.COMMTYPE_BINARY:
                value =int.from_bytes(byte, "little", signed = isSigned)
            else:
                raise ValueError("Only supported binary command type, now!")
        except:
            raise ValueError("Could not decode byte to value")
        return value
        
    def _check_cmdanswer(self, recv_data):
        """check command answer. If answer status is not 0, raise error according to answer  

        """
        answerstatus_index = self._get_answerstatus_index() 
        answerstatus = self._decode_value(recv_data[answerstatus_index:answerstatus_index+1], "short")
        mcprotocolerror.check_mcprotocol_error(answerstatus)
        return None

    def batchread_wordunits(self, headdevice, readsize):
        """batch read in word units.

        Args:
            headdevice(`str`):    Read head device. (ex: `"D1000"`)
            readsize(`int`):      Number of read device points

        Returns:
            wordunits_values(`list[int]`):  word units value list

        """
        subheader = 0x01

        request_data = bytes()
        request_data += self._make_devicedata(headdevice)
        request_data += self._encode_value(readsize, "byte")
        request_data += self._encode_value(0, "byte")
        send_data = self._make_senddata(subheader,request_data)
    
        #send mc data
        self._send(send_data)
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)

        word_values = []
        data_index = self._get_answerdata_index()
        for _ in range(readsize):
            wordvalue = self._decode_value(recv_data[data_index:data_index+self._wordsize], mode="short", isSigned=True)
            word_values.append(wordvalue)
            data_index += self._wordsize
        return word_values

    def batchread_bitunits(self, headdevice, readsize):
        """batch read in bit units.

        Args:
            headdevice(`str`):    Read head device. (ex: `"X1"`)
            size(`int`):          Number of read device points

        Returns:
            bitunits_values(`list[int]`):  bit units value(`0` or `1`) list

        """
        subheader = 0x00

        request_data = bytes()
        request_data += self._make_devicedata(headdevice)
        request_data += self._encode_value(readsize, "byte")
        request_data += self._encode_value(0, "byte")
        send_data = self._make_senddata(subheader, request_data)

        #send mc data
        self._send(send_data)
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)

        bit_values = []
        for i in range(readsize):
            data_index = i//2 + self._get_answerdata_index()
            value = int.from_bytes(recv_data[data_index:data_index+1], "little")
            #if i//2==0, bit value is 4th bit
            if(i%2==0):
                bitvalue = 1 if value & (1<<4) else 0
            else:
                bitvalue = 1 if value & (1<<0) else 0
            bit_values.append(bitvalue)

        return bit_values

    def batchwrite_wordunits(self, headdevice, values):
        """batch write in word units.

        Args:
            headdevice(`str`):    Write head device. (ex: `"D1000"`)
            values(`list[int]`):  Write values.

        """
        write_size = len(values)
        subheader = 0x03
        
        request_data = bytes()
        request_data += self._make_devicedata(headdevice)
        request_data += self._encode_value(write_size, "byte")
        request_data += self._encode_value(0, "byte")
        for value in values:
            request_data += self._encode_value(value, isSigned=True)
        send_data = self._make_senddata(subheader, request_data)

        #send mc data
        self._send(send_data)
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)

        return None

    def batchwrite_bitunits(self, headdevice, values):
        """batch read in bit units.

        Args:
            headdevice(`str`):    Write head device. (ex: `"X10"`)
            values(`list[int]`):  Write values. each value must be `0`(OFF) or `1`(ON).
        """
        write_size = len(values)
        subheader = 0x02

        #check values
        for value in values:
            if not (value == 0 or value == 1): 
                raise ValueError("Each value must be 0 or 1. 0 is OFF, 1 is ON.")

        request_data = bytes()
        request_data += self._make_devicedata(headdevice)
        request_data += self._encode_value(write_size, "byte")
        request_data += self._encode_value(0, "byte")
        #evary value is 0 or 1.
        #Even index's value turns on or off 4th bit, odd index's value turns on or off 0th bit.
        #First, create send data list. Length must be ceil of len(values).
        bit_data = [0 for _ in range((len(values) + 1)//2)]
        for index, value in enumerate(values):
            #calc which index data should be turns on.
            value_index = index//2
            #calc which bit should be turns on.
            bit_index = 4 if index%2 == 0 else 0
            #turns on or off value of 4th or 0th bit, depends on value
            bit_value = value << bit_index
            #Take or of send data
            bit_data[value_index] |= bit_value
        request_data += bytes(bit_data)
        send_data = self._make_senddata(subheader, request_data)
                    
        #send mc data
        self._send(send_data)
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)

        return None

    def randomwrite_wordunits(self, word_devices, word_values):
        """write word units and dword units randomly.

        Args:
            word_devices(`list[str]`):    Write word devices. (ex: `["D1000", "D1020"]`)
            word_values(`list[int]`):     Values for each word devices. (ex: `[100, 200]`)
        """
        if len(word_devices) != len(word_values):
            raise ValueError("word_devices and word_values must be same length")
            
        word_size = len(word_devices)

        subheader = 0x05
        
        request_data = bytes()
        request_data += self._encode_value(word_size, mode="byte")
        request_data += self._encode_value(0, "byte")
        for word_device, word_value in zip(word_devices, word_values):
            request_data += self._make_devicedata(word_device)
            request_data += self._encode_value(word_value, mode="short", isSigned=True)
        send_data = self._make_senddata(subheader, request_data)

        #send mc data
        self._send(send_data)
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)
        return None

    def randomwrite_bitunits(self, bit_devices, values):
        """write bit units randomly.

        Args:
            bit_devices(`list[str]`):    Write bit devices. (ex: `["X10", "X20"]`)
            values(`list[int]`):         Write values. each value must be `0`(OFF) or `1`(ON).
        """
        if len(bit_devices) != len(values):
            raise ValueError("bit_devices and values must be same length")
        write_size = len(values)
        #check values
        for value in values:
            if not (value == 0 or value == 1): 
                raise ValueError("Each value must be 0 or 1. 0 is OFF, 1 is ON.")

        subheader = 0x04
        
        request_data = bytes()
        request_data += self._encode_value(write_size, mode="byte")
        request_data += self._encode_value(0, "byte")
        for bit_device, value in zip(bit_devices, values):
            request_data += self._make_devicedata(bit_device)
            request_data += self._encode_value(value, mode="byte", isSigned=True)
        send_data = self._make_senddata(subheader, request_data)    
                    
        #send mc data
        self._send(send_data)
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)

        return None

    def remote_run(self):
        """Run PLC

        """
        
        subheader = 0x13

        request_data = bytes()
        send_data = self._make_senddata(subheader, request_data)

        #send mc data
        self._send(send_data)
        
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)
        return None

    def remote_stop(self):
        """ Stop remotely.

        """
        subheader = 0x14

        request_data = bytes()
        send_data = self._make_senddata(subheader, request_data)

        #send mc data
        self._send(send_data)
        
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)
        return None

    def read_cputype(self):
        """Read CPU type

        Returns:
            CPU type(str):      CPU type

        """

        subheader = 0x15

        request_data = bytes()
        send_data = self._make_senddata(subheader, request_data)

        #send mc data
        self._send(send_data)
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)
        data_index = self._get_answerdata_index()
        cpu_type = recv_data[data_index]
        cpu_type = hex(cpu_type)
        return cpu_type

    def loopback_test(self, echo_data):
        """Do loopback test.
        Send data and answer data should be same.

        Args:
            echo_data(str):     send data to PLC

        Returns:
            answer_len(int):    answer data length from PLC
            answer_data(str):   answer data from PLC

        """
        if echo_data.isalnum() is False:
            raise ValueError("echo_data must be only alphabet or digit code")
        if not ( 1 <= len(echo_data) <= 254):
            raise ValueError("echo_data length must be from 1 to 254")

        subheader = 0x16

        request_data = bytes()
        request_data += self._encode_value(len(echo_data), mode="short") 
        request_data += echo_data.encode()

        send_data = self._make_senddata(subheader, request_data)

        #send mc data
        self._send(send_data)
        #reciev mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data)

        data_index = self._get_answerdata_index()

        answer_len = self._decode_value(recv_data[data_index:data_index+self._wordsize], mode="short")
        answer = recv_data[data_index+self._wordsize:].decode()
        return answer_len, answer