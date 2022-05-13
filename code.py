import board
import busio
import digitalio
import math
import random
import supervisor
import time
import adafruit_drv2605
import adafruit_ble
import adafruit_ble.advertising.standard as BLEStandard
import adafruit_ble.services.nordic as BLENordic

DigitalInOut = digitalio.DigitalInOut
col = 0
remote_freq = 0
last_played = 0

def main():
	rows, cols = [[DigitalInOut(getattr(board,f"{x}{i}")) for i in range(1, 6)] for x in ["ROW", "COL"]]

	for i, r in enumerate(rows + cols):
		r.switch_to_output(i > 4)

	btn = DigitalInOut(board.BTN_A)
	btn.switch_to_input(pull=digitalio.Pull.UP)

	if not btn.value:
		return

	def sleepms(t):
		time.sleep(t/1000)

	def readStream(s):
		while True:
			b = s.read(1)
			if b != b"b":
				return None
			t = s.readline()
			return t

	def showLedCol(f, i):
		cols[i].value = False
		for j, row in enumerate(rows):
			if f == True or f[(j * 5) + i] != " ":
				row.value = True
		sleepms(1)
		for row in rows:
			row.value = False
		cols[i].value = True

	def showLed(f):
		for i, col in enumerate(cols):
			showLedCol(f, i)

	def nextCol():
		global col
		temp = col
		col = (col + 1) % 5
		return col

	heart = ( " X X "
			  "XXXXX"
			  "XXXXX"
			  " XXX "
			  "  X  " )

	hrUart = busio.UART(board.P8, board.P2, baudrate=115200)

	i2c = busio.I2C(board.P0, board.P1)
	drv = adafruit_drv2605.DRV2605(i2c)
	drv.use_LRM()
	for i, s in enumerate([adafruit_drv2605.Effect(7), adafruit_drv2605.Pause(.03), adafruit_drv2605.Effect(5)]):
		drv.sequence[i] = s

	def loop():
		global remote_freq, last_played

		if btn.value == 0:
			drv.stop()
			return True

		if bleUart.in_waiting and (temp := readStream(bleUart)):
			remote_freq = int(temp)

		if hrUart.in_waiting and (temp := readStream(hrUart)):
			bleUart.write(b"b" + temp)

		if remote_freq == 0:
			return

		delta = supervisor.ticks_ms() - last_played
		if delta < 100:
			showLedCol(heart, nextCol())
		if delta > 60000/remote_freq:
			drv.play()
			last_played = supervisor.ticks_ms()

	ble = adafruit_ble.BLERadio()
	bleUart = BLENordic.UARTService()
	adv = BLEStandard.ProvideServicesAdvertisement(bleUart)
	adv.complete_name = "HBPlush"

	while True:
		ble.start_advertising(adv)

		while ble.connected == 0:
			showLedCol(True, nextCol())
			sleepms(250)

		while ble.connected:
			if loop():
				return

main()
