import RPi.GPIO as GPIO
import time

def sendSignal(PIN,Duration):
    GPIO.output(PIN,True)
    time.sleep(Duration)
    GPIO.output(PIN,False)
    return

def calAbsSampleIndex(counter, Index, CHUNK, NumIgnoredFrame = 0, NumReqFrames=0):
    return (counter-NumIgnoredFrame - NumReqFrames)*CHUNK + Index