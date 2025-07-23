# Neopixel micro:bit driver

For neopixel, I tried a while to get it working using existing device drivers, but to no avail. So I looked at how they do it in the micro:bit online editor and found that their typescript code uses assembly based bit-banging, implemented in [this repo](https://github.com/microsoft/pxt-ws2812b/tree/master).

Based on this and the help of an AI assistant, the ws2812_loop.S file was created, which is a simple assembly routine that bit-bangs the WS2812 protocol. The main.c file was then updated to use this assembly routine to control the neopixel strip.
But as interrupts are disabled during the bit-banging, this sometimes leads to bluetooth crashes, so I am currently working on a fix for this.

We use the clock cycle of 1 / 64 MHz ≃ 15.625 ns and execute the desired amount of nops to achieve the timing in the WS2812 protocol. The timing is as follows:
- Reset pulse (“LOW” for >50 µs)
  We clear the data line and spin with 3,200 nop calls, for about 3,200 * 15.6 ns ~ 50 µs.

- “0” bit
  - High: ~360 ns -> .rept 20 NOPs (20 * 15.6 ns ~ 312 ns) plus ~3 cycles overhead -> ~360 ns total
  - Low: ~900 ns -> .rept 55 NOPs (55 * 15.6 ns ~ 859 ns) plus ~3 cycles overhead -> ~900 ns total

- “1” bit
  - High: ~800 ns -> .rept 49 NOPs (49 * 15.6 ns ~ 764 ns) plus ~3 cycles overhead -> ~800 ns total
  - Low: ~420 ns -> .rept 25 NOPs (25 * 15.6 ns ~ 390 ns) plus ~3 cycles overhead -> ~420 ns total

- Trailing reset (>50 µs)
  Same as the initial reset pulse (3,200 NOPs) to latch the data.


The following image shows a sent signal with 3 neopixels, each set to green (0x00FF00):

![all green signal](signal_all_green.png "All green signal")

One can nicely see the 5 * 8 * 3 = 120 bits sent, with the first 3 * 8 bits of each pixel being green (in the image, the "1" bit is the one with longer high time) and the last 2 * 8 bits being "0" (less high time for red and blue). Note that, yes it is GRB, not RGB, as the WS2812 protocol specifies it that way.

Here we can see the timing of the high and low signals in abit more detail, with a low signal on the left and a high signal on the right:

![high low timing](low_high_timing.png "High/low timing")