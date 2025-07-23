    .syntax unified
    .thumb
    .global ws2812_loop
    .type   ws2812_loop, %function

ws2812_loop:
    push    {r4-r7, lr}

    mov     r4, r0                  @ buf ptr
    mov     r5, r1                  @ byte count

    @—— Setup P0.02 ——@
    ldr     r3, =0x50000508         @ OUTSET
    ldr     r2, =0x5000050C         @ OUTCLR
    movs    r1, #1
    lsls    r1, r1, #2              @ mask = 1<<2

    @—— Reset low (> 50 µs) ——@
    str     r1, [r2]
    ldr     r7, =3200               @ ~3200 *15.6 ns ≃ 50 µs
.reset1:
    nop
    subs    r7, #1
    bne     .reset1

.byte_loop:
    movs    r7, #8                  @ 8 bits/byte
    ldrb    r0, [r4], #1            @ fetch next byte

.bit_loop:
    str     r1, [r3]                @ DATA HIGH

    tst     r0, #0x80               @ MSB?
    bne     .bit_one

    @—— “0”-bit high (~0.36 µs) ——@
    .rept   20                      @ 20 *15.6 ns ≃ 312 ns + ~3 cyc overhead ≃360 ns
      nop
    .endr
    str     r1, [r2]                @ DATA LOW

    @—— “0”-bit low (~0.89 µs) ——@
    .rept   55                      @ 55 *15.6 ns ≃ 859 ns + ~3 cyc ≃900 ns
      nop
    .endr
    b       .bit_done

.bit_one:
    @—— “1”-bit high (~0.80 µs) ——@
    .rept   49                      @ 49 *15.6 ns ≃ 764 ns + ~3 cyc ≃800 ns
      nop
    .endr
    str     r1, [r2]                @ DATA LOW

    @—— “1”-bit low (~0.45 µs) ——@
    .rept   25                      @ 25 *15.6 ns ≃ 390 ns + ~3 cyc ≃420 ns
      nop
    .endr

.bit_done:
    lsls    r0, r0, #1              @ shift next bit into MSB
    subs    r7, #1                  @ bits--
    bne     .bit_loop

    subs    r5, #1                  @ bytes--
    bne     .byte_loop

    @—— Trailing reset (> 50 µs) ——@
    str     r1, [r2]
    ldr     r7, =3200
.reset2:
    nop
    subs    r7, #1
    bne     .reset2

    pop     {r4-r7, pc}
