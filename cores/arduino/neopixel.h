#ifndef DEVICE_NEOPIXEL_H
#define DEVICE_NEOPIXEL_H

#define PINOP(pin, OP) (PORT->Group[(pin) / 32].OP.reg = (1 << ((pin) % 32)))

#ifdef PIN_NEOPIXEL

// The timings are taken from Adafruit's NeoPixel library

static void neopixel_send_buffer_core(volatile uint32_t *clraddr, uint32_t pinMask,
                                      const uint8_t *ptr, int numBytes) __attribute__((naked));

static void neopixel_send_buffer_core(volatile uint32_t *clraddr, uint32_t pinMask,
                                      const uint8_t *ptr, int numBytes) {
    asm volatile("        push    {r4, r5, r6, lr};"
                 "        add     r3, r2, r3;"
                 "loopLoad:"
                 "        ldrb r5, [r2, #0];" // r5 := *ptr
                 "        add  r2, #1;"       // ptr++
                 "        movs    r4, #128;"  // r4-mask, 0x80
                 "loopBit:"
                 "        str r1, [r0, #4];"                     // set
                 #ifdef __SAMD51__
                 "        movs r6, #3; d2: subs r6, #1; bne d2;" // delay 3
                 #else // __SAMD21__
                 "        movs r6, #3; d2: sub r6, #1; bne d2;"  // delay 3
                 #endif
                 "        tst r4, r5;"                           // mask&r5
                 "        bne skipclr;"
                 "        str r1, [r0, #0];" // clr
                 "skipclr:"
                 #ifdef __SAMD51__
                 "        movs r6, #6; d0: subs r6, #1; bne d0;" // delay 6
                 #else // __SAMD21__
                 "        movs r6, #6; d0: sub r6, #1; bne d0;"  // delay 6
                 #endif
                 "        str r1, [r0, #0];"    // clr (possibly again, doesn't matter)
                 #ifdef __SAMD51__
                 "        asrs     r4, r4, #1;" // mask >>= 1
                 #else // __SAMD21__
                 "        asr     r4, r4, #1;"  // mask >>= 1
                 #endif
                 "        beq     nextbyte;"
                 "        uxtb    r4, r4;"
                 #ifdef __SAMD51__
                 "        movs r6, #2; d1: subs r6, #1; bne d1;" // delay 2
                 #else // __SAMD21__
                 "        movs r6, #2; d1: sub r6, #1; bne d1;" // delay 2
                 #endif
                 "        b       loopBit;"
                 "nextbyte:"
                 "        cmp r2, r3;"
                 "        bcs stop;"
                 "        b loopLoad;"
                 "stop:"
                 "        pop {r4, r5, r6, pc};"
                 "");
}

// this assumes the pin has been configured correctly
static inline void neopixel_send_buffer(const uint8_t *ptr, int numBytes) {
    uint8_t portNum  = g_APinDescription[PIN_NEOPIXEL].ulPin / 32; // PIN_NEOPIXEL / 32;
    uint32_t pinMask = 1ul << (g_APinDescription[PIN_NEOPIXEL].ulPin % 32); // 1ul << (PIN_NEOPIXEL % 32);

    PINOP(PIN_NEOPIXEL, DIRSET);

#if (USB_VID == 0x239a) && (USB_PID == 0x0013)  // Adafruit Metro M0
    // turn off mux too, needed for metro m0
    PORT->Group[portNum].PINCFG[g_APinDescription[PIN_NEOPIXEL].ulPin % 32].reg = /
        (uint8_t)(PORT_PINCFG_INEN);
#endif

    PINOP(PIN_NEOPIXEL, OUTCLR);
    delay(1);

    volatile uint32_t *clraddr = &PORT->Group[portNum].OUTCLR.reg;

    // equivalent to cpu_irq_is_enabled()
    if (__get_PRIMASK() == 0) {
        __disable_irq();
        neopixel_send_buffer_core(clraddr, pinMask, ptr, numBytes);
        __enable_irq();
    } else {
        neopixel_send_buffer_core(clraddr, pinMask, ptr, numBytes);
    }
}
#endif // PIN_NEOPIXEL


#if defined(INTERNAL_DS_CLK)
static inline void write_apa_byte(uint8_t x) {
    for (uint8_t i = 0x80; i != 0; i >>= 1) {
        if (x & i)
            PINOP(INTERNAL_DS_DATA, OUTSET);
        else
            PINOP(INTERNAL_DS_DATA, OUTCLR);

        PINOP(INTERNAL_DS_CLK, OUTSET);
        // for (uint8_t j=0; j<25; j++) /* 0.1ms */
        //  __asm__ __volatile__("");

        PINOP(INTERNAL_DS_CLK, OUTCLR);
        // for (uint8_t j=0; j<25; j++) /* 0.1ms */
        //  __asm__ __volatile__("");
    }
}
#endif


static inline void RGBLED_set_color(uint32_t color) {
#if defined(INTERNAL_DS_CLK)
    write_apa_byte(0x0);
    write_apa_byte(0x0);
    write_apa_byte(0x0);
    write_apa_byte(0x0);

    write_apa_byte(0xFF);
    write_apa_byte(color >> 16);
    write_apa_byte(color >> 8);
    write_apa_byte(color);

    write_apa_byte(0xFF);
    write_apa_byte(0xFF);
    write_apa_byte(0xFF);
    write_apa_byte(0xFF);

    // set clock port low for ~10ms
    delay(50);

#elif defined(PIN_NEOPIXEL)
    uint8_t buf[NEOPIXEL_COUNT * 3];
#if 0
    memset(buf, 0, sizeof(buf));
    buf[0] = color >> 8;
    buf[1] = color >> 16;
    buf[2] = color;
#else
    for (int i = 0; i < NEOPIXEL_COUNT * 3; i += 3) {
        buf[i + 0] = color >> 8;
        buf[i + 1] = color >> 16;
        buf[i + 2] = color;
    }
#endif
    neopixel_send_buffer(buf, NEOPIXEL_COUNT * 3);
#endif
}

#endif // DEVICE_NEOPIXEL_H
