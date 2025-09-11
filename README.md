# Esphome DucoRFM69 component
In order to create a non invasive communication/connection solution to my Duco Ventition system, I am in the process of creating a esphome based solution. 

It uses two (later 3) technical parts to get it to work. First, we have the ESP32 platform that allows for interfacing and connection to (a) home assistant implementation, if preferred. I may consider some autonomous functions, but first lets see if we can get som basics going.
For the 868MHz communications I've decided to work with a RFM69w module (low power variant). In my research, I found that the libs would enable me to move away from difficult radio intrinsics and focus on the data and the Duco Box itself.
In the future, I hope to create a serial connection to read values such as voltages and power etcetera but I haven't dived in it yet, as my first aim is to control the device (low-mid-medium-high debit).


## About
This is my autoddidactically challenging project of transforming the interesting work at the project https://github.com/arnemauer/Ducobox-ESPEasy-Plugin into ESPHome.

I have extremely basic programming skills and am using all that is at my disposal to get to a working Duco RF bridge with serial communications. I have an enormous list of todo's before I get to a working sample.

## To do
- [ ] Get hardware
  - [x] ESP32Devkit1
  - [x] RFM69W chip
  - [x] Wiring for chip
  - [ ] wiring for serial
- [x] Get bare minimum SPI component working
  - [ ] Consider test and try interactions with chip 
- [x] Rename and redefine
- [x] Upload to GIT
- [ ] ...
