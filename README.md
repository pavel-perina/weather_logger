# weather_logger

## Goal

Building unattended, battery powered, off-line weather station.
Just for fun. 

Goals:

- [ ] Runs 2 months or more on "reasonable" batteries (4x3000mAH at worst)
- [ ] Monitors temperature, humidity, pressure and voltage battery
- [ ] Logs data to SD card
- [ ] Survives rain, snow and reasonably low temperatures (let's say -15C)

Tests:

- [x] RTC code (set alarm, wake on interrupt, read time, set time)
- [x] Estimate power consumption of Arduino Pro Mini
- [x] HTU21D temperature reading
- [ ] BMP180 pressure reading
- [x] SD card power shut-down and re-ininitalization
- [ ] Battery monitor
- [ ] No data corruption at low power
- [ ] No data corruption at low temperature

Conclusions:

RTC interrupt works, but not when RTC is powered from backup battery.
Purpose of battery is obviously to keep clock running, not to power up
whole circuit. It needs voltage regulator with very low quiescent current
and some electronic parts to power up Arduino.

Power consumption of Arduino Pro Mini 8MHz/3.3V
- Running (i wrote it somewhere)
- Sleeping with power LED on - 1.6mA
- Sleeping with power LED gone - 230uA :-) (5.5mAh/day 170mAh/month)
- Sleeping with ADC off - To be done
- Sleeping with ADC off, BOD off - To be done

It's possible to turn off power for SD card and intitalize it with SDFat 
library. Initialization takes very roughly 100ms. Power consumption of SD
card seems quite low (below 100uA idle tested on two Sandisk SD cards),
but AMS1117-3.3V voltage regulator consumes more than 2.2mA. 
I don't want to use power regulator on Arduino, Arduino UNO had so week one
that most of the cards failed to initialize reliably. I tried to measure
power consumption of SD cards with improvized osciloscope, but I can't detect
short peaks. Conclusion is idle card is negligible, various cards can 
take 15-50mA when SPI communication is ongoing and there are peaks caused by
init/read/write/erase operations. 
TLDR: 
- Power consumption of SD card with power off - 0.000
- Power consumption of SD card with SCLK low - negligible cause of cheap voltage
regulator that takes 2.2mA
- Power consumption of SD card with SCLK changing - 15-50mA seen
- Power consumption of SD card peak - over 100mA seen

TODO: shutdown circuit test
TODO: shutdown circuit scheme

   

