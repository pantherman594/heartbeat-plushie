import board
import busio
import digitalio
import math
import random
import supervisor
import time

def main():
	rows, cols = [[digitalio.DigitalInOut(getattr(board,f"{x}{i}")) for i in range(1,6)] for x in ["ROW","COL"]]

	for r in rows: r.switch_to_output(False)
	for c in cols: c.switch_to_output(True)

	btn=digitalio.DigitalInOut(board.BTN_A)
	btn.switch_to_input(pull=digitalio.Pull.UP)

	def slp(): time.sleep(.0005)

	if not btn.value:
		return
	else:
		from adafruit_ble import BLERadio
		from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
		from adafruit_ble.services.nordic import UARTService

		addr = 0x57

		# Status Registers
		MAX30105_INTSTAT1 =		0x00
		MAX30105_INTSTAT2 =		0x01
		MAX30105_INTENABLE1 =		0x02
		MAX30105_INTENABLE2 =		0x03

		# FIFO Registers
		MAX30105_FIFOWRITEPTR = 	0x04
		MAX30105_FIFOOVERFLOW = 	0x05
		MAX30105_FIFOREADPTR = 	0x06
		MAX30105_FIFODATA =		0x07

		# Configuration Registers
		MAX30105_FIFOCONFIG = 		0x08
		MAX30105_MODECONFIG = 		0x09
		MAX30105_PARTICLECONFIG = 	0x0A	# Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
		MAX30105_LED1_PULSEAMP = 	0x0C
		MAX30105_LED2_PULSEAMP = 	0x0D
		MAX30105_LED3_PULSEAMP = 	0x0E
		MAX30105_LED_PROX_AMP = 	0x10
		MAX30105_MULTILEDCONFIG1 = 0x11
		MAX30105_MULTILEDCONFIG2 = 0x12

		# Die Temperature Registers
		MAX30105_DIETEMPINT = 		0x1F
		MAX30105_DIETEMPFRAC = 	0x20
		MAX30105_DIETEMPCONFIG = 	0x21

		# Proximity Function Registers
		MAX30105_PROXINTTHRESH = 	0x30

		# Part ID Registers
		MAX30105_REVISIONID = 		0xFE
		MAX30105_PARTID = 			0xFF	# Should always be 0x15. Identical to MAX30102.

		# MAX30105 Commands
		# Interrupt configuration (pg 13, 14)
		MAX30105_INT_A_FULL_MASK =		~0b10000000
		MAX30105_INT_A_FULL_ENABLE = 	0x80
		MAX30105_INT_A_FULL_DISABLE = 	0x00

		MAX30105_INT_DATA_RDY_MASK = ~0b01000000
		MAX30105_INT_DATA_RDY_ENABLE =	0x40
		MAX30105_INT_DATA_RDY_DISABLE = 0x00

		MAX30105_INT_ALC_OVF_MASK = ~0b00100000
		MAX30105_INT_ALC_OVF_ENABLE = 	0x20
		MAX30105_INT_ALC_OVF_DISABLE = 0x00

		MAX30105_INT_PROX_INT_MASK = ~0b00010000
		MAX30105_INT_PROX_INT_ENABLE = 0x10
		MAX30105_INT_PROX_INT_DISABLE = 0x00

		MAX30105_INT_DIE_TEMP_RDY_MASK = ~0b00000010
		MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02
		MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00

		MAX30105_SAMPLEAVG_MASK =	~0b11100000
		MAX30105_SAMPLEAVG_1 = 	0x00
		MAX30105_SAMPLEAVG_2 = 	0x20
		MAX30105_SAMPLEAVG_4 = 	0x40
		MAX30105_SAMPLEAVG_8 = 	0x60
		MAX30105_SAMPLEAVG_16 = 	0x80
		MAX30105_SAMPLEAVG_32 = 	0xA0

		MAX30105_ROLLOVER_MASK = 	0xEF
		MAX30105_ROLLOVER_ENABLE = 0x10
		MAX30105_ROLLOVER_DISABLE = 0x00

		MAX30105_A_FULL_MASK = 	0xF0

		# Mode configuration commands (page 19)
		MAX30105_SHUTDOWN_MASK = 	0x7F
		MAX30105_SHUTDOWN = 		0x80
		MAX30105_WAKEUP = 			0x00

		MAX30105_RESET_MASK = 		0xBF
		MAX30105_RESET = 			0x40

		MAX30105_MODE_MASK = 		0xF8
		MAX30105_MODE_REDONLY = 	0x02
		MAX30105_MODE_REDIRONLY = 	0x03
		MAX30105_MODE_MULTILED = 	0x07

		# Particle sensing configuration commands (pgs 19-20)
		MAX30105_ADCRANGE_MASK = 	0x9F
		MAX30105_ADCRANGE_2048 = 	0x00
		MAX30105_ADCRANGE_4096 = 	0x20
		MAX30105_ADCRANGE_8192 = 	0x40
		MAX30105_ADCRANGE_16384 = 	0x60

		MAX30105_SAMPLERATE_MASK = 0xE3
		MAX30105_SAMPLERATE_50 = 	0x00
		MAX30105_SAMPLERATE_100 = 	0x04
		MAX30105_SAMPLERATE_200 = 	0x08
		MAX30105_SAMPLERATE_400 = 	0x0C
		MAX30105_SAMPLERATE_800 = 	0x10
		MAX30105_SAMPLERATE_1000 = 0x14
		MAX30105_SAMPLERATE_1600 = 0x18
		MAX30105_SAMPLERATE_3200 = 0x1C

		MAX30105_PULSEWIDTH_MASK = 0xFC
		MAX30105_PULSEWIDTH_69 = 	0x00
		MAX30105_PULSEWIDTH_118 = 	0x01
		MAX30105_PULSEWIDTH_215 = 	0x02
		MAX30105_PULSEWIDTH_411 = 	0x03

		#Multi-LED Mode configuration (pg 22)
		MAX30105_SLOT1_MASK = 		0xF8
		MAX30105_SLOT2_MASK = 		0x8F
		MAX30105_SLOT3_MASK = 		0xF8
		MAX30105_SLOT4_MASK = 		0x8F

		SLOT_NONE = 				0x00
		SLOT_RED_LED = 			0x01
		SLOT_IR_LED = 				0x02
		SLOT_GREEN_LED = 			0x03
		SLOT_NONE_PILOT = 			0x04
		SLOT_RED_PILOT =			0x05
		SLOT_IR_PILOT = 			0x06
		SLOT_GREEN_PILOT = 		0x07

		MAX_30105_EXPECTEDPARTID = 0x15

		def readStream(stream):
			while True:
				b = stream.read(1)
				if not b:
					return None
				return bytes(b + stream.readline()).strip()

		def showLed(frame):
			for i, col in enumerate(cols):
				col.value = False
				for j, row in enumerate(rows):
					if frame is None or frame[j][i] != " ":
						r.value=True
						slp()
						r.value = False
					else:
						slp()

				col.value = True

		def getRegister(reg, n=1):
			i = bytearray(n)
			i2c.writeto_then_readfrom(addr, bytearray([reg]), i)
			return i

		def setRegister(reg, val):
			i2c.writeto(addr, bytearray([reg, val]))

		def setBitmask(reg, mask, val):
			setRegister(reg, ((getRegister(reg)[0]) & mask) | val)

		# Check for new data but give up after 250 ms
		def readIR():
			start = supervisor.ticks_ms()
			while True:
				if supervisor.ticks_ms() - start > 250:
					print("not found")
					return 0

				readPointer = getRegister(MAX30105_FIFOREADPTR)[0]
				writePointer = getRegister(MAX30105_FIFOWRITEPTR)[0]

				if readPointer == writePointer:
					time.sleep(0.001)
					continue

				n = writePointer - readPointer

				if n < 0:
					n += 32

				for _ in range(n):
					b = getRegister(MAX30105_FIFODATA, 9)
				return (b[3]<<16 | b[4]<<8 | b[5]) & 0x03FFFF

		irMin = 0
		irMax = 1
		irSigCur = 2
		irSigPrev = 3
		irSigMin = 4
		irSigMax = 5
		irAvg = 6
		buffer = 7
		offset = 8
		firCoeffs = 9
		positiveEdge = 10
		negativeEdge = 11
		lastBeat = 12

		irState = [
			-20, # irMin
			20, # irMax
			0, # irSigCur
			0, # irSigPrev
			0, # irSigMin
			0, # irSigMax
			0, # irAvg
			[0]*32, # buffer
			0, # offset
			[172,321,579,927,1360,1858,2390,2916,3391,3768,4012,4096], # firCoeffs
			False, # positiveEdge
			False, # negativeEdge
			0 # lastBeat
		]

		def isBeat(ir):
			irState[irSigPrev] = irState[irSigCur]

			# Average DC Estimator
			irState[irAvg] += ((ir << 15) - irState[irAvg]) >> 4
			est = irState[irAvg] >> 15

			# Low Pass FIR Filter
			irState[buffer][irState[offset]] = ir - est

			z = irState[firCoeffs][11] * (irState[buffer][(irState[offset] - 11) % 32])

			for i in range(11):
				z += irState[firCoeffs][i] * (irState[buffer][(irState[offset] - i) % 32] + irState[buffer][(irState[offset] - 22 + i) % 32])

			irState[offset] = (irState[offset] + 1) % 32

			irState[irSigCur] = z >> 15

			beat = False

			if irState[irSigPrev] < 0 and irState[irSigCur] >= 0:
				irState[irMax] = irState[irSigMax]
				irState[irMin] = irState[irSigMin]
				irState[positiveEdge] = True
				irState[negativeEdge] = False
				irState[irSigMax] = 0
				if (irState[irMax] - irState[irMin]) > 20 and (irState[irMax] - irState[irMin]) < 1000:
					beat = True

			if irState[irSigPrev] > 0 and irState[irSigCur] <= 0:
				irState[positiveEdge] = False
				irState[negativeEdge] = True
				irState[irSigMin] = 0

			if irState[positiveEdge] and irState[irSigCur] > irState[irSigPrev]:
				irState[irSigMax] = irState[irSigCur]

			if irState[negativeEdge] and irState[irSigCur] < irState[irSigPrev]:
				irState[irSigMin] = irState[irSigCur]

			return beat

		def getHr(ir):
			if not isBeat(ir):
				return None
			delta = supervisor.ticks_ms() - irState[lastBeat]
			irState[lastBeat] = supervisor.ticks_ms()
			if delta == 0:
				return None
			return 60 / (delta / 1000)

		i2c = busio.I2C(board.P0,board.P1)

		while not i2c.try_lock():
			pass

		# softReset
		setBitmask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET)
		start = time.time()
		while (time.time() - start < 0.1):
			resp = getRegister(MAX30105_MODECONFIG)[0]
			if (resp & MAX30105_RESET) == 0:
				break
			time.sleep(0.001)

		# Configure FIFO, averaging 4 values
		setBitmask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, MAX30105_SAMPLEAVG_4)

		# Enable FIFO rollover
		setBitmask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE)

		# Watch all 3 LED channels
		setBitmask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, MAX30105_MODE_MULTILED)

		# Sense 15.63pA per LSB
		setBitmask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, MAX30105_ADCRANGE_4096)
		# 400 samples per second
		setBitmask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, MAX30105_SAMPLERATE_400)
		# 18 bit resolution, with about 6 inches of detection
		setBitmask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, MAX30105_PULSEWIDTH_411)

		# LED pulse amplitude configuration
		setRegister(MAX30105_LED1_PULSEAMP, 0x0A) # red
		setRegister(MAX30105_LED2_PULSEAMP, 0x1F) # ir
		setRegister(MAX30105_LED3_PULSEAMP, 0) # green
		setRegister(MAX30105_LED_PROX_AMP, 0x0A) # proximity

		# Configure slots
		setBitmask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, SLOT_RED_LED)
		setBitmask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, SLOT_IR_LED << 4)
		setBitmask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, SLOT_GREEN_LED)

		# Clear FIFO
		setRegister(MAX30105_FIFOWRITEPTR, 0);
		setRegister(MAX30105_FIFOOVERFLOW, 0);
		setRegister(MAX30105_FIFOREADPTR, 0);

		while True:
			print("Min:110000,Max:130000,val:" + str(readIR()))
			continue
			bpm = getHr(readIR())
			if bpm is not None:
				print((bpm,))

		ble = BLERadio()
		uart = UARTService()
		adv = ProvideServicesAdvertisement(uart)
		adv.complete_name = "HBPlush"

		h = [ " X X ",
			  "XXXXX",
			  "XXXXX",
			  " XXX ",
			  "	X	" ]
		anim = []

		for i in range(-5, 5):
			j = abs(i)
			anim += [(h, 5 - j), (None, j)] * 10

		while True:
			ble.start_advertising(adv)
			while not ble.connected:
				pass

			animFrame = 0
			animTimes = 0
			frame, dur = anim[animFrame]

			while ble.connected:
				while animTimes >= dur:
					animFrame = (animFrame + 1) % len(anim)
					animTimes = 0
					frame, dur = anim[animFrame]

				showLed(frame)
				slp()
				animTimes += 1

				if uart.in_waiting:
					msg = readStream(uart)
					print(msg)

main()
