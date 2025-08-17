\## 24 Hour Clock with STM32



Attempting to design a 24 hour clock on a quad seven-segment display using as few pins from an MCU as possible while also having some fun by putting this through a digital circuit of my design.



Since the STM32F446RE has an internal RTC, I was able to get away with only using 9 pins. Direct control of each pin on the quad display would have required 8 segment pins plus the five digit pins equaling 13 total pins from the MCU. My digital circuit is arguably a massive mess compared to just using the extra 4 MCU pins, but who cares. This was a fun project I used to introduce myself to the STM32 environment and get to revisit digital circuits.

