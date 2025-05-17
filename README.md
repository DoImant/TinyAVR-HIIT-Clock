# HIIT Timer for high intensity interval training

This program is used to control a HIIT stopwatch. In high-intensity interval training, there is always an active phase in which an exercise is performed and a relaxation phase until the next exercise.

A specific duration can be set for each of the two phases. After starting the clock (long press), the set time intervals are counted down alternately. When a countdown reaches zero, an acoustic signal sounds and the next countdown begins.

A rotary encoder is used to enter the values and control the clock. The output device is a 128x64 pixel OLED display.  

If no input is made at the clock, the circuit is put into a sleep mode to save power. The power consumption in sleep mode is about < 10µA. To end this, a short press on the encoder button is also sufficient.

This version is customized to an ATtiny 1614. A crystal is connected to the Attiny 1614 so that the time function is more accurate.

## Breadboard View

Breadboard view (Click on the photo to see a video on Youtube.)\
[![breadboard circuit](https://github.com/DoImant/Stuff/blob/main/HIIT-Clock/IMG_2346.jpg?raw=true)](https://youtu.be/UrN1_Pi5jAI)

## Power consumption

Power consumption during operation\
![power consumption operation](https://github.com/DoImant/Stuff/blob/main/HIIT-Clock/Verbrauch-Betrieb.jpg?raw=true)

Power consumption during deep sleep\
![power consumption deepsleep](https://github.com/DoImant/Stuff/blob/main/HIIT-Clock/Verbrauch-Deepsleep.jpg?raw=true)
