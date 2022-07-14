import serial
import atexit

from time import sleep
from labdevices import powersupply

class KA3005PSerial(powersupply.PowerSupply):
	def __init__(
		self,
		port,

		debug = False,
		timeoutRetry = 3,
		readbackRetry = 3,
		serialCommandDelay = 0.1
	):
		super().__init__(
			nChannels = 1,
			vrange = (0, 30, 0.01),
			arange = (0, 5, 0.001),
			prange = (0, 150, 0.001),
			capableVLimit = True,
			capableALimit = True,
			capableMeasureV = True,
			capableMeasureA = True,
			capableOnOff = True
		)

		self._debug = debug
		self._timeoutRetry = timeoutRetry
		self._readbackRetry = readbackRetry
		self._serialCommandDelay = serialCommandDelay

		if isinstance(port, serial.Serial):
			self._port = port
			self._portName = None
			self.__initialRequests()
		else:
			self._portName = port
			self._port = None

		atexit.register(self.__close)

	def __initialRequests(self):
		# Query the identity and do a check this is a KA3005P
		self._idn(initialQuery = True)
		# Disable output, disable overcurrent and overvoltage protection
		self._sendCommand("OUT0")
		self._sendCommand("OCP0")
		self._sendCommand("OVP0")
		self._setVoltage(0, 1)
		self._setCurrent(0, 1)

	# Context management

	def __enter__(self):
		# Open the serial port / pipe in case we should open it ourself
		if (self._port is None) and (not (self._portName is None)):
			self._port = serial.Serial(self._portName, baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
			self.__initialRequests()
		return self

	def __exit__(self, exc_type, exc_val, exc_tb):
		# Close the port if we have opened it ourself and it's currently opened
		self.__close()

	def __close(self):
		atexit.unregister(self.__close)
		if (not (self._port is None)) and (not (self._portName is None)):
			self._off()
			self._port.close()
			self._port = None

	# Utility functions

	def _sendCommand(self, cmd):
		if self._debug:
			print("PSU> {}".format(cmd))
		sleep(self._serialCommandDelay)
		self._port.write(cmd.encode('ascii'))

	def _sendCommandReply(self, cmd, replyLen = None):
		retries = self._timeoutRetry

		if self._debug:
			print("PSU> {}".format(cmd))
		sleep(self._serialCommandDelay)
		self._port.write(cmd.encode('ascii'))

		res = []
		while True:
			c = self._port.read(1).decode('ascii')
			if len(c) <= 0:
				if replyLen < 0:
					break

				if self._debug:
					print("PSU Timeout, received {} until now".format(res))
				if (not (retries is None)):
					if retries > 0:
						retries = retries - 1
						continue
					else:
						raise IOError("Serial port timeout")
			if ord(c) == 0:
				break

			res.append(c)
			if not (replyLen is None):
				if len(res) == replyLen:
					break

		reply = "".join(res)

		if self._debug:
			print("PSU< {}".format(reply))

		return reply

	# Communication functions

	def _idn(self, initialQuery = False):
		repl = self._sendCommandReply("*IDN?", replyLen = -1)

		if len(repl) != 30:
			raise IOError("Unknown IDN response {}, not a KORAD KA3005P?".format(repl))
		if repl[:len("KORAD KA3005P")] != "KORAD KA3005P":
			raise IOError("Unknown IDN response {}, not a KORAD KA3005P?".format(repl))

		sn = repl[22:]
		ver = repl[15:18]

		if self._debug:
			print("PSU: Serial {}, version {}".format(sn, ver))

		if initialQuery:
			self._serialNumber = sn
			self._softwareVersion = ver

		return {
			'idn' : repl,
			'serial' : sn,
			'version' : ver
		}


	def _setChannelEnable(self, enable, channel):
		retries = self._readbackRetry

		while True:
			if enable:
				self._sendCommand("OUT1")
			else:
				self._sendCommand("OUT0")

			# Read back status ...
			repl = self._sendCommandReply("STATUS?", replyLen = 1)
			if ((ord(repl) & 0x40 == 0) and not enable) or ((ord(repl) & 0x40 != 0) and enable):
				return True

			print("PSU Readback failed")

			if retries > 0:
				retries = retries - 1
				if retries == 0:
					raise IOError("Failed to set output status and read back correct state")
				retries = retries - 1


	def _setVoltage(self, voltage, channel):
		retries = self._readbackRetry

		while True:
			self._sendCommand("VSET{0}:{1:05.2f}".format(channel, voltage))

			# Readback status
			repl = self._sendCommandReply("VSET{0}?".format(channel), replyLen = 5)
			try:
				readVoltage = float(repl)
			except ValueError:
				if self._debug:
					print("PSU Failed to parse response on VSET? after setting")
				readVoltage = None

			if abs(readVoltage - voltage) < 1e-2:
				return True

			print("PSU Mismatch of set and read back voltage (set {}, read {})".format(voltage, readVoltage))

			if retries > 0:
				retries = retries - 1
				if retries == 0:
					raise IOError("Failed to read back set voltage")

	def _setCurrent(self, current, channel):
		retries = self._readbackRetry

		while True:
			self._sendCommand("ISET{0}:{1:05.3f}".format(channel, current))

			# Read back status
			repl = self._sendCommandReply("ISET{0}?".format(channel), replyLen = 5)

			try:
				readCurrent = float(repl[:5])
			except ValueError:
				if self._debug:
					print("PSU Failed to parse response on ISET? after setting")
				readCurrent = None

			if abs(readCurrent - current) < 1e-2:
				return True

			print("PSU Mismatch of set and read back current (set {}, read {})".format(current, readCurrent))

			if retries > 0:
				retries = retries - 1
				if retries == 0:
					raise IOError("Failed to read back set current")

	def _getVoltage(self, channel):
		vRead = self._sendCommandReply("VOUT{0}?".format(channel), replyLen = 5)
		try:
			return float(vRead)
		except ValueError:
			raise IOError("Unknown voltage response {}".format(vRead))

	def _getCurrent(self, channel):
		vRead = self._sendCommandReply("IOUT{0}?".format(channel), replyLen = 5)
		try:
			return float(vRead)
		except ValueError:
			raise IOError("Unknown voltage response {}".format(vRead))

	def _off(self):
		self._setChannelEnable(False, 1)
		return True

	def _getLimitMode(self, channel):
		repl = self._sendCommandReply("STATUS?", replyLen = 1)
		if len(repl) <= 0:
			raise IOError("Unknown status response {}".format(repl))
		repl = ord(repl)
		if (repl & 0x40) == 0:
			return powersupply.PowerSupplyLimit.NONE
		if (repl & 0x01) == 0:
			return powersupply.PowerSupplyLimit.CURRENT
		else:
			return powersupply.PowerSupplyLimit.VOLTAGE

	def _isConnected(self):
		if not (self._port is None):
			return True
		else:
			return False
