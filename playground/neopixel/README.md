

For neopixel, I tried a while to get it working using existing device drivers, but to no avail. So I looked at how they do it in the micro:bit online editor and found that their typescript code uses assembly based bit-banging, implemented in [this repo](https://github.com/microsoft/pxt-ws2812b/tree/master).

Based on this and the help of an AI assistant, the ws2812_loop.S file was created, which is a simple assembly routine that bit-bangs the WS2812 protocol. The main.c file was then updated to use this assembly routine to control the neopixel strip.
But as interrupts are disabled during the bit-banging, this sometimes leads to bluetooth crashes, so I am currently working on a fix for this.