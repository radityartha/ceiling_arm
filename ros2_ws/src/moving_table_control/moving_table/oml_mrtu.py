"""oml_mrtu.py
Orientalmotor Motion Library for Modbus RTU module ver.1.0 rel.240619

imprementation class:
    commPC
    mathConv
    ModbusAZ, ModbusCVD, ModbusOM

commPC class
    commPC library is based on PyModbus
    PyModbus document: https://pymodbus.readthedocs.io/en/latest/index.html

"""

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

from rclpy.logging import get_logger

logger = get_logger("oml_mrtu")


# ------------------------------------------------------------------------------
# Modbus Read/Write for PC
class commPC:
    """Modbus communication class for Orinetalmotor

    Args:
        Port (str): communication port (Windows e.g. 'COM3'. Linux e.g. '/dev/ttyUSB0')
        Baudrate (int): communication speed [bps]
    """

    def __init__(self, argPort: str, argBaudrate: int = 115200):
        self.client = ModbusSerialClient(
            port=argPort, baudrate=argBaudrate, parity="E", bytesize=8, stopbits=1
        )
        self.client.connect()

    def modbusWriteReg(self, add: int, idx: int, val: list[int]) -> list:
        """Write registers (code 0x10). 16bit

        Args:
            add (int): Modbus slave ID
            idx (int): Start address to write to
            val (list[int]): List of values to write (16bit values)

        Return:
            list: success:[0]/failure:[-1, function code, exception code],[-2, 1, Error Message]
        """
        res = self.client.write_registers(address=idx, values=val, device_id=add)
        if isinstance(res, ModbusIOException):
            logger.info(res)
            return [-2, 1, res]

        if 0x80 < res.function_code:
            retvalue = [-1, hex(res.function_code), hex(res.exception_code)]
        else:
            retvalue = [0]
        return retvalue

    def modbusWriteRegWide(self, add: int, idx: int, val: list[int]) -> list:
        """Write registers (code 0x10). 32bit

        Args:
            add (int): Modbus slave ID
            idx (int): Start address to write to
            val (list[int]): List of values to write (32bit values)

        Return:
            list: success:[0]/failure:[-1, function code, exception code],[-2, 1, Error Message]
        """
        list16bit_val = [0] * len(val) * 2
        cnt = 0
        for v in val:
            list16bit_val[cnt] = v >> 16
            list16bit_val[cnt + 1] = v & 0xFFFF
            cnt += 2
        res = self.client.write_registers(
            address=idx, values=list16bit_val, device_id=add
        )
        if isinstance(res, ModbusIOException):
            logger.info(res)
            return [-2, 1, res]

        if 0x80 < res.function_code:
            retvalue = [-1, hex(res.function_code), hex(res.exception_code)]
        else:
            retvalue = [0]
        return retvalue

    def modbusReadReg(self, add: int, idx: int, length: int) -> list:
        """Read holding registers (code 0x03). 16bit

        Args:
            add (int): slave address (modbus id)
            idx (int): Start address to read from
            length (int): Number of coils to read

        Return:
            list: success:[0, read values]/failure:[-1, function code, exception code], [-2, 1, Error Message]
        """
        res = self.client.read_holding_registers(
            address=idx, device_id=add, count=length
        )
        if isinstance(res, ModbusIOException):
            logger.info(res)
            return [-2, 1, res]

        if 0x80 < res.function_code:
            retvalue = [-1, hex(res.function_code), hex(res.exception_code)]
        else:
            retvalue = [0]
            for i in range(length):
                retvalue.append(res.registers[i])
        return retvalue

    def modbusReadRegWide(self, add: int, idx: int, length: int) -> list:
        """Read holding registers (code 0x03). 32bit

        Args:
            add (int): slave address (modbus id)
            idx (int): Start address to read from
            length (int): Number of coils to read

        Return:
            list: success:[0, read values]/failure:[-1, function code, exception code], [-2, 1, Error Message]
        """
        list16bit_val = [0] * length * 2  # 2倍の要素数のリスト作成
        val = [0] * length
        res = self.client.read_holding_registers(
            address=idx, device_id=add, count=len(list16bit_val)
        )
        if isinstance(res, ModbusIOException):
            logger.info(res)
            return [-2, 1, res]

        if 0x80 < res.function_code:
            retvalue = [-1, hex(res.function_code), hex(res.exception_code)]
        else:
            retvalue = [0]
            cnt = 0
            for i in range(length):
                val[i] = (res.registers[cnt] << 16) + res.registers[cnt + 1]
                cnt += 2
                retvalue.append(val[i])
        return retvalue


class mathConv:
    """Mathematical function class for OMML"""

    def __init__(self):
        pass

    def unsigned_to_signed_32bit_bitwise(self, value: int) -> int:
        """Unsigned 32-bit to signed 32-bit conversion

        Args:
            value (int): unsigned 32bit value

        Returns:
            int: singed 32bit value
        """
        signed_hex = value
        if signed_hex & 0x8000_0000:
            signed_int = (int(signed_hex ^ 0xFFFF_FFFF) * -1) - 1
        else:
            signed_int = int(signed_hex)

        return signed_int

    def to_2s_complement(self, decimal_num: int, bit_length: int) -> int:
        """Converts a signed decimal number to its two's complement representation in hexadecimal

        Args:
            decimal_num (int): signed decimal number
            bit_length (int): Number of bits in 2's complement representation after conversion

        Returns:
            int: converted hexadecimal value
        """
        two_complement = decimal_num
        if 0 >= decimal_num:
            abs_num = abs(decimal_num)
            two_complement = (2**bit_length - abs_num) & (2**bit_length - 1)
        return two_complement


# ---------- OrientalMotor Modbus Motion Library -------------------
class ModbusOM:
    """Modbus Modules for Orientalmotor

    Args:
        comm (instance): RS-485 Comm Instance object.
        serverAddress (int): Address number(Modbus ID).
    """

    def __init__(self, comm, serverAddress: int):
        self.comm = comm
        self.serverAddress = serverAddress
        res = self.comm.modbusReadReg(self.serverAddress, 0x007D, 1)
        self.driveCmd = res[1]
        self.driverOutput = 0
        self.OpeCurrent = 1000  # current 1:0.1% default 100%
        self.rateAcc = 1000000  # accelerate default 1kHz/s
        self.rateDec = 1000000  # decelerate default 1kHz/s

    def free(self, sw: bool = False) -> list:
        """Free/AWO input

        Args:
            sw (bool, Optional): True ON/False OFF. Defaults to False.

        Return:
            list: success:0/failure:[-2, 0],[-2, 1, Error Message]
        """
        if True == sw:
            res = self.comm.modbusWriteReg(
                self.serverAddress, 0x007D, [self.driveCmd | 0x0040]
            )
        elif False == sw:
            res = self.comm.modbusWriteReg(
                self.serverAddress, 0x007D, [self.driveCmd & 0xFFBF]
            )
        else:
            res = [-2, 0]
        return res

    def isReady(self) -> list:
        """Confirmation of READY output

        Returns:
            list: success:[0, True:complete/False:incomplete]/failure:[-2, 1, Error Message]
        """

        self.driverOutput = self.readDriverOutput()
        logger.info(self.driverOutput)
        if -2 == self.driverOutput[0]:
            return self.driverOutput

        ret = [0, False]
        if 0x0020 == (self.driverOutput[1] & 0x0020):
            ret = [0, True]
        return ret

    def ppreset(self):
        """Presets the command position.

        Returns:
            list: success:[0]/failure:[-2, 1, Error Message]
        """
        res = self.comm.modbusWriteReg(self.serverAddress, 0x018A, [0, 1])
        if -2 == res[0]:
            return res
        res = self.comm.modbusWriteReg(self.serverAddress, 0x018A, [0, 0])
        return res

    def readAlarm(self) -> list:
        """Monitors the present alarm code.

        Returns:
            list: success:[0, alarm code(int)]/failure:[-2, 1, Error Message]
        """
        res = self.comm.modbusReadRegWide(self.serverAddress, 0x0080, 1)
        return res

    def readDriverOutput(self) -> list:
        """Reads the output status of the driver.

        Returns:
            list: success:[0, output status(int)]/failure:[-2, 1, Error Message]
        """
        res = self.comm.modbusReadRegWide(self.serverAddress, 0x007E, 1)
        return res

    def requestConfiguration(self) -> list:
        """Reqest Configuration

        Returns:
            list: success:[0]/failure:[-2, 1, Error Message]
        """
        self.comm.modbusWriteReg(self.serverAddress, 0x018C, [0, 1])
        res = self.comm.modbusWriteReg(self.serverAddress, 0x018C, [0, 0])
        return res

    def resetAlarm(self) -> list:
        """Reset alarm

        Returns:
            list: success:[0]/failure:[-2, 1, Error Message]
        """
        res = self.comm.modbusWriteReg(self.serverAddress, 0x0180, [0, 1])
        if 0 != res[0]:
            return res
        res = self.comm.modbusWriteReg(self.serverAddress, 0x0180, [0, 0])
        self.comm.modbusWriteReg(self.serverAddress, 0x007D, [self.driveCmd])
        return res

    def startContinuous(self, speed: int, direction: int = 0) -> list:
        """Start continuous drive

        Args:
            speed (int): speed command
            direction (int, optional): 0:Forward/1:Reverse. Defaults to 0.

        Returns:
            list: success:[0]/failure:[-1, function code, exception code]/[-2, 0]/[-2, 1, Error Message]
        """
        speed = mathConv().to_2s_complement(speed, 32)

        if 0 == direction:
            cmd = 0x4000
        elif 1 == direction:
            cmd = 0x8000
        else:
            return [-2, 0]

        res = self.comm.modbusWriteRegWide(
            self.serverAddress, 0x1806, [self.rateAcc, self.rateDec, self.OpeCurrent]
        )
        if 0 != res[0]:
            return res
        res = self.comm.modbusWriteRegWide(self.serverAddress, 0x1804, [speed])
        if 0 != res[0]:
            return res
        res = self.comm.modbusWriteReg(
            self.serverAddress, 0x007D, [self.driveCmd | cmd]
        )
        return res

    def startPosition(self, position: int, speed: int, OpeType: int = 1) -> list:
        """Start positioning

        Args:
            position (int): Position command.
            speed (int): Speed command.
            modeAbsInc (int, optional): 1:Absolute/2:Incremental. Defaults to 1.

        Returns:
            list: success:[0]/failure:[-1, function code, exception code]/[-2, 0]/[-2, 1, Error Message]
        """
        if 2 < OpeType or 1 > OpeType:
            return [-2, 0]
        res = self.startDirectData(position, speed, OpeType)

        return res

    def startDirectData(
        self,
        position: int,
        speed: int,
        OpeType: int = 2,
        OpeDataNo: int = 0,
        Trigger: int = 1,
        Memory: int = 0,
    ) -> list:
        """Start Direct Data Drive

        Args:
            position (int): Position command.
            speed (int): Speed command.
            OpeDataNo (int, Optional): Operation data No. Defaults to 0.
            OpeType (int, Optional): Operation type. Defaults to 2.
            Trigger (int, Optional): Trigger. Defaults to 1.
            Memory (int, Optional): Memory. Defaults to 0.

        Returns:
            list: success:[0]/failure:[-1, function code, exception code]/[-2, 0]/[-2, 1, Error Message]
        """

        if not -(1 << 31) <= position <= ((1 << 31) - 1):  # -2^31 ~ 2^31-1
            return [-2, 0]
        if not -4_000_000 <= speed <= 4_000_000:
            return [-2, 0]
        if not 0 <= OpeDataNo <= 255:
            return [-2, 0]
        if not 0 <= Memory <= 1:
            return [-2, 0]

        position = mathConv().to_2s_complement(position, 32)
        speed = mathConv().to_2s_complement(speed, 32)
        Trigger = mathConv().to_2s_complement(Trigger, 32)
        OpeDataNo = mathConv().to_2s_complement(OpeDataNo, 32)
        Memory = mathConv().to_2s_complement(Memory, 32)

        directDriveData = [
            OpeDataNo,  # drive no.
            OpeType,  # drive type
            position,  # position reference
            speed,  # speed reference
            self.rateAcc,  # accelerate
            self.rateDec,  # decelerate
            self.OpeCurrent,  # current
            Trigger,  # trigger
            Memory,
        ]  # memory

        res = self.comm.modbusWriteRegWide(self.serverAddress, 0x0058, directDriveData)
        return res

    def stop(self) -> list:
        """Stop motor

        Returns:
            list: success:[0]/[-2, 1, Error Message]
        """
        res = self.comm.modbusWriteReg(
            self.serverAddress, 0x007D, [self.driveCmd | 0x0020]
        )
        if 0 != res[0]:
            return res
        res = self.comm.modbusWriteReg(self.serverAddress, 0x007D, [self.driveCmd])
        return res

    def writeParamAcc(self, time: int, speed: int = 16667) -> list:
        """Sets acceleration parameters

        Args:
            time (int): Time[ms]
            speed (int, optional): Speed[Hz]. Defaults to 16667(1000r/min).

        Return:
            list: success:[0]/failure[-2, 0]
        """
        preAcc = self.rateAcc
        self.rateAcc = int(1000.0 * speed / time)

        if 0 < self.rateAcc <= 1_000_000_000:
            return [0]
        else:
            self.rateAcc = preAcc
            return [-2, 0]

    def writeParamDec(self, time: int, speed: int = 16667) -> list:
        """Sets deceleration parameters

        Args:
            time (int): Time[ms]
            speed (int, optional): Speed[Hz]. Defaults to 16667(1000r/min).

        Return:
            list: success:[0]/failure[-2, 0]
        """
        preDec = self.rateDec
        self.rateDec = int(1000.0 * speed / time)

        if 0 < self.rateDec <= 1_000_000_000:
            return [0]
        else:
            self.rateDec = preDec
            return [-2, 0]

    def writeParamCurrent(self, current: int) -> list:
        """Sets deceleration parameters

        Args:
            current (int): current (1:0.1%)

        Return:
            list: success:[0]/failure[-2, 0]
        """
        preCurrent = self.OpeCurrent
        if 0 <= current <= 1_000:
            self.OpeCurrent = current
            return [0]
        else:
            self.OpeCurrent = preCurrent
            return [-2, 0]


# endModbusOM


class ModbusAZ(ModbusOM):
    """Modbus library for AZ series

    Args:
        comm (object): RS-485 Comm object
        serverAddress (int): Modbus ID
    """

    def __init__(self, comm: object, serverAddress: int):
        ModbusOM.__init__(self, comm, serverAddress)
        self.rateAcc = 1000000  # default 1000 kHz/s
        self.rateDec = 1000000  # default 1000 kHz/s

    def readPosition(self) -> list:
        """Monitors the command and the actual position.

        Returns:
            list: success:[0, actual position(int)]/failure:[-2, 1, Error Message]
        """
        positionSensingVal = self.comm.modbusReadRegWide(self.serverAddress, 0x00CC, 1)
        if 0 != positionSensingVal[0]:
            return positionSensingVal
        else:
            positionSensingVal = [
                positionSensingVal[0],
                mathConv().unsigned_to_signed_32bit_bitwise(positionSensingVal[1]),
            ]

        return positionSensingVal

    def readSpeed(self, unit: int = 0) -> list:
        """Monitors the command and the actual speed.

        Args:
            unit (int, Optional): Unit of speed 0:Hz/1:r/min. Defaults to 0.

        Returns:
            list: success:[0, actual speed(int)]/failure:[-2, 0]/[-2, 1, Error Message]
        """
        if 0 == unit:
            registerAddress = 0x00D0  # Hz
        elif 1 == unit:
            registerAddress = 0x00CE  # r/min
        else:
            return [-2, 0]

        speedSensingVal = self.comm.modbusReadRegWide(
            self.serverAddress, registerAddress, 1
        )
        if 0 != speedSensingVal[0]:
            return speedSensingVal
        speedSensingVal = mathConv().unsigned_to_signed_32bit_bitwise(
            speedSensingVal[1]
        )
        return [0, speedSensingVal]

    def startDirectData(
        self,
        position: int,
        speed: int,
        OpeType: int = 2,
        OpeDataNo: int = 0,
        Trigger: int = 1,
        Memory: int = 0,
    ) -> list:
        """Start Direct Data Drive

        Args:
            position (int): Position command.
            speed (int): Speed command.
            OpeDataNo (int, Optional): Operation data No. Defaults to 0.
            OpeType (int, Optional): Operation type. Defaults to 2.
            Trigger (int, Optional): Trigger. Defaults to 1.
            Memory (int, Optional): Memory. Defaults to 0.

        Returns:
            list: success:[0]/failure:[-1, function code, exception code]/[-2,0]/[-2, 1, Error Message]
        """

        if 0 <= OpeType <= 22:
            if (4 <= OpeType and 6 >= OpeType) or 19 == OpeType:
                return [-2, 0]
        else:
            return [-2, 0]

        res = ModbusOM.startDirectData(
            self, position, speed, OpeType, OpeDataNo, Trigger, Memory
        )

        return res

    def startZhome(self):
        """Start high speed home-seeking

        Return:
            list: success:[0]/failure:[-2, 1, Error Message]

        """
        res = self.comm.modbusWriteReg(
            self.serverAddress, 0x007D, [self.driveCmd | 0x0010]
        )
        if 0 != res[0]:
            return res
        res = self.comm.modbusWriteReg(self.serverAddress, 0x007D, [self.driveCmd])
        return res


# endModbusAZ


class ModbusCVD(ModbusOM):
    """Modbus library for CVD

    Args:
        comm (object): RS-485 Comm object
        serverAddress (int): Modbus ID
    """

    def __init__(self, comm: object, serverAddress: int):
        ModbusOM.__init__(self, comm, serverAddress)
        self.rateAcc = 30000  # default 30kHz/s
        self.rateDec = 30000  # default 30kHz/s

    def readPosition(self) -> list:
        """Monitors the command and the actual position.

        Returns:
            list: success:[0, actual position(int)]/failure:[-2, 1, Error Message]
        """
        positionSensingVal = self.comm.modbusReadRegWide(self.serverAddress, 0x00CC, 1)
        if 0 != positionSensingVal[0]:
            return positionSensingVal
        else:
            positionSensingVal = [
                positionSensingVal[0],
                mathConv().unsigned_to_signed_32bit_bitwise(positionSensingVal[1]),
            ]

        return positionSensingVal

    def readSpeed(self, unit: int = 0) -> list:
        """Monitors the command and the actual speed.

        Args:
            unit (int, Optional): Unit of speed 0:Hz/1:r/min. Defaults to 0.

        Returns:
            list: success:[0, actual speed(int)]/failure:[-2, 0]/[-2, 1, Error Message]
        """
        if 0 == unit:
            registerAddress = 0x00D0  # Hz
        elif 1 == unit:
            registerAddress = 0x00CE  # r/min
        else:
            return [-2, 0]

        speedSensingVal = self.comm.modbusReadRegWide(
            self.serverAddress, registerAddress, 1
        )
        if 0 != speedSensingVal[0]:
            return speedSensingVal
        speedSensingVal = mathConv().unsigned_to_signed_32bit_bitwise(
            speedSensingVal[1]
        )
        return [0, speedSensingVal]

    def startDirectData(
        self,
        position: int,
        speed: int,
        OpeType: int = 2,
        OpeDataNo: int = 0,
        Trigger: int = 1,
        Memory: int = 0,
    ) -> list:
        """Start Direct Data Drive

        Args:
            position (int): Position command.
            speed (int): Speed command.
            OpeDataNo (int, Optional): Operation data No. Defaults to 0.
            OpeType (int, Optional): Operation type. Defaults to 2.
            Trigger (int, Optional): Trigger. Defaults to 1.
            Memory (int, Optional): Memory. Defaults to 0.

        Returns:
            list: success:[0]/failure:[-1, function code, exception code]/[-2,0]/[-2, 1, Error Message]
        """
        if 0 > OpeType or 2 < OpeType:
            return [-2, 0]
        res = ModbusOM.startDirectData(
            self, position, speed, OpeType, OpeDataNo, Trigger, Memory
        )

        return res

    def startHome(self, mode: int = 0, direction: int = 0, offset: int = 0) -> list:
        """Start home-seeking

        Args:
            mode (int, optional): 0:2-sensor, 1:3-sensor, 2:1-direction (3:Push AZ only) Defaults to 0.
            direction (int, optional): direction of home-seeking. 0:Negative/1:Positive. Defaults to 0.
            offset (int, optional): Position offset of home-seeking. Defaults to 0. 32bit.

        Returns:
            list: success:[0]/failure:[-1, function code, exception code]/[-2,0]/[-2, 1, Error Message]
        """

        mode = mathConv().to_2s_complement(mode, 16)
        direction = mathConv().to_2s_complement(direction, 16)
        if not -(1 << 31) + 1 <= offset <= (1 << 31) - 1:
            return [-2, 0]

        offset = mathConv().to_2s_complement(offset, 32)

        res = self.comm.modbusWriteReg(self.serverAddress, 0x02C1, [mode])
        if 0 != res[0]:
            return res
        res = self.comm.modbusWriteReg(self.serverAddress, 0x02C3, [direction])
        if 0 != res[0]:
            return res
        res = self.comm.modbusWriteRegWide(self.serverAddress, 0x02D0, [offset])
        if 0 != res[0]:
            return res
        self.comm.modbusWriteReg(self.serverAddress, 0x007D, [self.driveCmd | 0x0010])
        res = self.comm.modbusWriteReg(self.serverAddress, 0x007D, [self.driveCmd])

        return res


# endModbusCVD


class ModbusBLVR(ModbusOM):
    """Modbus library for BLV-R

    Args:
        comm (object): RS-485 Comm object
        serverAddress (int): Modbus ID
    """

    def __init__(self, comm: object, serverAddress: int):
        ModbusOM.__init__(self, comm, serverAddress)
        self.rateAcc = 30000  # default 30kHz/s
        self.rateDec = 30000  # default 30kHz/s

    def free(self, sw: bool = False) -> list:
        """Free/AWO input

        Args:
            sw (bool, Optional): True ON/False OFF. Defaults to False.

        Return:
            list: success:0/failure:[-2, 0],[-2, 1, Error Message]
        """
        if True == sw:
            res = self.comm.modbusWriteReg(
                self.serverAddress, 0x007D, [self.driveCmd | 0x0040]
            )
        elif False == sw:
            res = self.comm.modbusWriteReg(
                self.serverAddress, 0x007D, [self.driveCmd & 0xFFBF]
            )
        else:
            res = [-2, 0]
        return res


# endModbusBLVR
