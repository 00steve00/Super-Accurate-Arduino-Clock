# Super-Accurate-Arduino-Clock
An arduino based clock with 1ms accuracy over a year. Time stored in RTC continually updated by GPS module

Project goal: Create a super-accurate (<1ms drift per year) standalone 7-segment LED clock whose time stays accurate and "sets itself" without access to the Internet or a host computer (i.e., without using NTP servers) by updating time via GPS time signals. Since GPS satellites have their own onboard atomic clocks, this is an extremely stable and accurate time source! The code only uses SPI.h and Wire.h comm libraries; everything else (interfacing with GPS module, Max7219, RTC DS3231, time calculations) is coded "manually" with NO OTHER additional libraries (as a challenge and learning experience). So the total code base ends up being quite light (about 7,300 bytes).

Project design: Arduino controller (Mega for prototyping, Nano for deployment); 8-digit 7-segment LED display module with Max 7219 controller chip; Ublox Neo-6M (clone) GPS module; DS3231 RTC module. External power supply via 5V USB phone charger.

Requirements: RTC clock time appears on the 7-segment LED display. RTC clock time is updated *hourly* by syncing with GPS module's 1Hz PPS signal (this achieves the <1ms accuracy)... so that the clock "sets itself" on a permanent basis without user intervention. User-defined UTC offsets (in code) display time in local time zone. User-defined daylight-savings rules, including fractional hour offsets (need to make sure it can work in Nepal!). If power is lost, unit maintains time through RTC battery backup.

How it works:
There are 2 ways to pull time and date from the GPS module. One is by reading various NMEA sentences and parsing them (this appears to be the most common approach if you check out other GPS clock projects). BUT... there is quite a bit of latency in the time it takes for the NMEA sentence to be sent, read, and parsed, especially at 9600 baud. I measured about 450ms (almost half a second!) latency when I compared the time set parsing NMEA sentences to the highly accurate 1Hz time-pulse output of the GPS module. We can do better than 450ms, right? YOU BET! Read on...
The magic happens in that the Neo 6M GPS module is capable of outputting a 1Hz "Pulse Per Second" (PPS) signal. This signal is precisely synced to the UTC time received from the GPS satellites once the GPS module has a fix. So we will set time with this PPS reference (via digital reads), and NOT the NMEA sentences. (Make sure to get a module board that has the PPS pin on it... not all modules do!). Do we really need this absurd level of accuracy? Of course not! But... it's fun.
To further minimize latency, when my code prepares to sync the RTC to GPS, it reads a standard NMEA sentence, then increments the second to be "primed and ready" for the next PPS signal. That way, as soon as the PPS signal is received, we immediately writes the new second to the time registers on the RTC
When syncing with this PPS method, I get a 20 *micro*second difference between the RTC squarewave output and the GPS's PPS signal right after doing the syncing. By doing the sync every hour, we assure a super-accurate RTC time register is maintained. The RTC 1Hz squarewave output in turn triggers an update of the RTC time to the LED display. Accounting for the time it takes to read the RTC time registers and then output those data to the LED module, I've calculated a total latency of about 400 *micro*seconds best case and, as the RTC's accuracy drifts over the hour, around 1.2ms worst case.

IMPLEMENTATION NOTES:

A) GPS Module and super accurate time-setting. This whole project hinges on the ability of the GPS receiver to output a 1 Hz PPS (pulse-per-second) signal based on the UTC time received from the GPS satellites. The documentation for the U-blox Neo-6M GPS receiver is formidable. (My clone seems to have the same functionality.)  The full documentation is 200+ pages:  U-blox 6 Receiver Description and Protocols.

The important part for our application is the timepulse feature. The unit has a timepulse output that defaults to 1Hz pulses (goes high for 100ms every 1000ms) when the GPS receiver has a GPS fix lock. (In the documentation, see Appendix Section A.17 for the timepulse default settings on the 6M.) The timepulse goes high at the "top of the second" -- so the rising edge is what we want to watch for on the module PPS pin in order to set our RTC as precisely as possible. On the modules commonly available for purchase, the timepulse output is on the PPS pin. As mentioned above, make sure the module you purchase actually has the PPS pin. Some Neo-6M modules for sale out there don't have it. Check pictures carefully or ask the seller. You're paying for the whole GPS receiver chip, you should be able to access all its functions!

B) UTC Offsets for local time - without a library! Implementing the offset is fairly straightforward for whole hour offsets, but more complicated for fractional hours. There are many places around the world with fractional hour offsets from UTC. My code allows the user to specify the offset in both hours and minutes relative to UTC.

C) Daylight savings - without a library!  The tricky part here is that in the US and Europe, for example, the begin and end dates of "daylight savings" or "summer time" are defined not as specific dates each year, but in terms of "second Sunday of March" or "first Sunday of November". So the actual start/end dates move around year to year. My code allows the user to specify the formula in a format of "nth" day of the week for a given month, at a given time. The code then determines on the fly whether DST is in effect at any given time based on the settings.  If a location doesn't observe DST, then simply put an arbitrary start/end time and make sure the offset hours are the same.

D) RTC modules - hardware hidden traps! There are loads of very cheap DS3231 RTC modules out there. But there are some things to watch out for! First, not every DS3231 chip is the same on these modules. There is the higher accuracy (+/- 2ppm) DS3231-SN chip, and the lower accuracy (+/- 5ppm) DS3231-M chip. Might as well spend a few extra pennies and get the good one. I had to ask several sellers till I found one that promised it was the SN chip.

E) IR Receiver - this code also implements an IR demodulator receiver feature to decode commands sent by an infrared remote control, in this case a Sony remote. Again, the code does not rely on libraries. This code assumes a 20-bit Sony remote protocol. The code could easily be adapted to other protocols.
