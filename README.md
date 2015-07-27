# UltrasonicSensor

Using the HC-SR04.   Using a timer and GPIO intterupts the ultrasonic echo pin triggers an interrupt.  

Using system_get_rtc_time() to determine the time it has taken for the echo pin to go low.
Uiing system_rtc_clock_cali_proc() to find out the processor tick time.

Programmed with the ESP8266 SDK  and  SDK API Guide Version 1.2.0 using ECLIPSE.

Values ouput are correct between 7cm and 40cm.  Not tested further than that.

Note:  don't know the difference between lolevel and negedge interrupt.  lolevel does not work. 
Note: (to fix) when the distace of the object in front of the ultrasonic sensor is moved an erranious distance is displayed sometimes.





