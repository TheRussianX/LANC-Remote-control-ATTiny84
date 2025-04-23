This is a port of LANC remote control from http://controlyourcamera.blogspot.com/2011/02/arduino-controlled-video-recording-over.html to ATTiny84
The code is provided as is, with no further updates or bugfixes. Use it anyway you like.
To make it work you first have to calibrate the internal oscillator.
For debugging purposes you can modify the sendByte() function to have a permanent HIGH on the LANC Out when not transmitting and to check what the MCU is sending via serial at 9600 bits/sec with an Arduino as a receiver (RX pin 0). 
