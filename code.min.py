F=enumerate
import board as B,busio as P,digitalio as Q,supervisor as R,time,adafruit_drv2605 as C,adafruit_ble as Y,adafruit_ble.advertising.standard as Z,adafruit_ble.services.nordic as a
S=Q.DigitalInOut
D=0
E=0
J=0
def A():
	A=True;K,G=[[S(getattr(B,f"{A}{C}"))for C in range(1,6)]for A in['ROW','COL']]
	for (L,b) in F(K+G):b.switch_to_output(L>4)
	M=S(B.BTN_A);M.switch_to_input(pull=Q.Pull.UP)
	if not M.value:return
	def T(t):time.sleep(t/1000)
	def U(s):
		while A:
			B=s.read(1)
			if B!=b'b':return None
			C=s.readline();return C
	def N(f,i):
		C=False;G[i].value=C
		for (D,B) in F(K):
			if f==A or f[D*5+i]!=' ':B.value=A
		T(1)
		for B in K:B.value=C
		G[i].value=A
	def g(f):
		for (A,B) in F(G):N(f,A)
	def V():global D;D=(D+1)%5;return D
	c=' X X XXXXXXXXXX XXX   X  ';W=P.UART(B.P8,B.P2,baudrate=115200);d=P.I2C(B.P0,B.P1);H=C.DRV2605(d);H.use_LRM()
	for (L,e) in F([C.Effect(7),C.Pause(0.03),C.Effect(5)]):H.sequence[L]=e
	def f():
		global E,J
		if M.value==0:H.stop();return A
		if I.in_waiting and(B:=U(I)):E=int(B)
		if W.in_waiting and(B:=U(W)):I.write(b'b'+B)
		if E==0:return
		C=R.ticks_ms()-J
		if C<100:N(c,V())
		if C>60000/E:H.play();J=R.ticks_ms()
	O=Y.BLERadio();I=a.UARTService();X=Z.ProvideServicesAdvertisement(I);X.complete_name='HBPlush'
	while A:
		O.start_advertising(X)
		while O.connected==0:N(A,V());T(250)
		while O.connected:
			if f():return
A()