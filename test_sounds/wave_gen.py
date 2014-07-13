from struct import pack
#from math import sin, pi
import wave
import random

RATE=44100
'''
## GENERATE MONO FILE ##
wv = wave.open('test_mono.wav', 'w')
wv.setparams((1, 2, RATE, 0, 'NONE', 'not compressed'))
maxVol=2**15-1.0 #maximum amplitude
wvData=""
for i in range(0, RATE*30):
        wvData+=pack('h', maxVol*sin((i*3.14*2*440.0)/RATE)) #500Hz
wv.writeframes(wvData)
wv.close()


## GENERATE STERIO FILE ##
wv = wave.open('test_stereo.wav', 'w')
wv.setparams((2, 2, RATE, 0, 'NONE', 'not compressed'))
maxVol=2**15-1.0 #maximum amplitude
wvData=""
for i in range(0, RATE*30):
        wvData+=pack('h', maxVol*sin(i*2*3.14*1000.0/RATE)) #500Hz left
        wvData+=pack('h', maxVol*sin(i*2*3.14*2000.0/RATE)) #200Hz right
wv.writeframes(wvData)
wv.close()
'''
wv = wave.open('test_quad.wav', 'w')
wv.setparams((2, 2, RATE, 0, 'NONE', 'not compressed'))
maxVol=2**15-1.0 #maximum amplitude
wvData=""
frL = 2000
frR = 2000
currL = maxVol
currR = maxVol
dL = RATE/frL/2
dR = RATE/frR/2
for i in range(0, 60*RATE):
	dL -= 1
	if (dL <= 0):
		currL *= -1
		dL = RATE/frL/2
	dR -= 1
	if (dR <= 0):
		currR *= -1
		dR = RATE/frR/2
	wvData+=pack('h', currL) #500Hz left
	wvData+=pack('h', currR) #200Hz right
wv.writeframes(wvData)
wv.close()
