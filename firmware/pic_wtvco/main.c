// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// WT VCO input PIC: PIC16F18426 as an "external ADC" (wtvco_input_spec.md 7).
// Reads 8 analog inputs with burst averaging and sends one 18-byte frame per millisecond over UART
// (1 Mbps, 8N1, TX only) to the RP2350. No calibration, no reception.
//
// Pins (as wired, spec 2.7):
//   RA5 (2)  UART TX  -> 4.7 kΩ -> RP2350 GP1
//   RA4 (3)  FM_BI    ANA4      RC5 (5)  FM_UNI      ANC5
//   RC4 (6)  WT_BI    ANC4      RC3 (7)  WT_UNI      ANC3
//   RC2 (8)  FM_DEPTH ANC2      RC1 (9)  FREQ_FINE   ANC1
//   RC0 (10) FREQ_COARSE ANC0   RA2 (11) WT_KNOB     ANA2
//   RA3 (4)  MCLR (10 kΩ pull-up)
//   RA0 (13) / RA1 (12): ICSP, otherwise unused. RA0 = frame timing debug output (high while converting).
//
// Build: build.sh. TEST_FIXED=1 sends fixed patterns instead of ADC values (UART link test, 7.3 / 7.4).
#include <xc.h>
#include <stdint.h>

#ifndef TEST_FIXED
#define TEST_FIXED 0
#endif

// Configuration: HFINTOSC 32 MHz (7.1), MCLR enabled (2.5), no WDT, LVP ON (MPLAB Snap needs it).
#pragma config FEXTOSC = OFF, RSTOSC = HFINT32, CLKOUTEN = OFF, CSWEN = ON, FCMEN = OFF
#pragma config MCLRE = ON, PWRTS = PWRT_64, LPBOREN = OFF, BOREN = ON, BORV = LO, ZCD = OFF, PPS1WAY = OFF, STVREN = ON
#pragma config WDTCPS = WDTCPS_31, WDTE = OFF, WDTCWS = WDTCWS_7, WDTCCS = SC
#pragma config BBSIZE = BB512, BBEN = OFF, SAFEN = OFF, WRTAPP = OFF, WRTB = OFF, WRTC = OFF, WRTD = OFF, WRTSAF = OFF, LVP = ON
#pragma config CP = OFF

// ---- ADC channels (ADPCH) ------------------------------------------------------------------------
#define CH_ANA2  0x02   // WT_KNOB
#define CH_ANA4  0x04   // FM_BI
#define CH_ANC0  0x10   // FREQ_COARSE
#define CH_ANC1  0x11   // FREQ_FINE
#define CH_ANC2  0x12   // FM_DEPTH
#define CH_ANC3  0x13   // WT_UNI
#define CH_ANC4  0x14   // WT_BI
#define CH_ANC5  0x15   // FM_UNI

// Frame slots (7.3)
enum { S_WT_UNI, S_WT_BI, S_FM_UNI, S_FM_BI, S_COARSE, S_FINE, S_KNOB, S_DEPTH, S_COUNT };

#if !TEST_FIXED
// Conversion order and averaging (7.2): every value ends up as "average x 4" (0..16380).
typedef struct { uint8_t ch, rpt, crs, slot; } Conv;
static const Conv CONV[8] = {
    {CH_ANC2,  4, 0, S_DEPTH },
    {CH_ANA2,  4, 0, S_KNOB  },
    {CH_ANC1,  8, 1, S_FINE  },
    {CH_ANC0, 16, 2, S_COARSE},   // after the previous frame's UART transmission has ended
    {CH_ANC3,  4, 0, S_WT_UNI},   // the four CVs last, right before sending (10.4)
    {CH_ANC4,  4, 0, S_WT_BI },
    {CH_ANC5,  8, 1, S_FM_UNI},
    {CH_ANA4,  8, 1, S_FM_BI },
};
#endif

// ---- UART TX (interrupt driven, so conversions continue while the frame goes out) ----------------
#define FRAME_LEN 18
static volatile uint8_t tx_buf[FRAME_LEN];
static volatile uint8_t tx_pos = FRAME_LEN;

void __interrupt() isr(void) {
    if (PIE3bits.TX1IE && PIR3bits.TX1IF) {
        TX1REG = tx_buf[tx_pos++];
        if (tx_pos >= FRAME_LEN) PIE3bits.TX1IE = 0;
    }
}

static void send_frame(const uint16_t *v, uint8_t seq) {
    while (tx_pos < FRAME_LEN) { }            // previous frame still going out (should never wait)
    uint8_t sum = 0;
    tx_buf[0] = (uint8_t)(0x80 | (seq & 0x7F));
    for (uint8_t i = 0; i < S_COUNT; i++) {
        uint8_t hi = (uint8_t)((v[i] >> 7) & 0x7F), lo = (uint8_t)(v[i] & 0x7F);
        tx_buf[1 + 2 * i] = hi;
        tx_buf[2 + 2 * i] = lo;
        sum += hi + lo;
    }
    tx_buf[17] = sum & 0x7F;
    tx_pos = 0;
    PIE3bits.TX1IE = 1;                       // TX1IF is set while the buffer is empty -> ISR starts
}

// ---- ADC (ADCC burst average, 7.2) ---------------------------------------------------------------
#if !TEST_FIXED
static uint16_t convert(const Conv *c) {
    ADPCH = c->ch;
    ADRPT = c->rpt;
    ADCON2bits.ADCRS = c->crs;
    ADCON2bits.ADACLR = 1;                    // clear the accumulator
    while (ADCON2bits.ADACLR) { }
    ADCON0bits.ADGO = 1;                      // stays set until ADRPT conversions are accumulated
    while (ADCON0bits.ADGO) { }
    return ADFLTR;                            // ADACC >> ADCRS = average x 4
}
#endif

static void init(void) {
    // Pins: analog inputs as listed, RA5 UART TX, RA0/RA1 outputs low (unused, 2.7).
    LATA = 0; LATC = 0;
    TRISA = 0b00011100;                       // RA2, RA3(MCLR), RA4 inputs; RA0, RA1, RA5 outputs
    TRISC = 0b00111111;
    ANSELA = 0b00010100;                      // RA2, RA4
    ANSELC = 0b00111111;                      // RC0..RC5
    WPUA = 0; WPUC = 0;

    // PPS: EUSART1 TX on RA5 (Table 16-3: TX1/CK1 = 0x0F).
    RA5PPS = 0x0F;

    // EUSART1: 1 Mbps = 32 MHz / (4 x (7 + 1)) exactly (7.4).
    BAUD1CON = 0; BAUD1CONbits.BRG16 = 1;
    TX1STAbits.BRGH = 1; TX1STAbits.SYNC = 0;
    SP1BRGH = 0; SP1BRGL = 7;
    RC1STAbits.SPEN = 1;
    TX1STAbits.TXEN = 1;

    // ADCC: VREF+ = VDD, VREF- = VSS, clock Fosc/32 -> TAD = 1 µs, burst average mode.
    ADREF = 0x00;
    ADCLK = 15;                               // Fosc / (2 x (15 + 1)) = 1 MHz
    ADACQ = 2;                                // 2 TAD acquisition (pins have 100 nF, 7.2)
    ADCON1 = 0; ADCON3 = 0;
    ADCON2 = 0; ADCON2bits.ADMD = 0b011;      // burst average
    ADCON0 = 0; ADCON0bits.ADFM0 = 1; ADCON0bits.ADON = 1;

    // Timer0: 8-bit, Fosc/4 = 8 MHz, 1:32 -> 250 kHz, period 250 -> 1 ms frame tick (7.3).
    T0CON1 = 0; T0CON1bits.T0CS = 0b010; T0CON1bits.T0CKPS = 0b0101;
    TMR0H = 249; TMR0L = 0;
    T0CON0 = 0; T0CON0bits.T0EN = 1;
    PIR0bits.TMR0IF = 0;

    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

void main(void) {
    init();
    uint16_t v[S_COUNT];
    uint8_t seq = 0;
#if TEST_FIXED
    uint16_t n = 0;
#endif
    for (;;) {
        while (!PIR0bits.TMR0IF) { }
        PIR0bits.TMR0IF = 0;
        LATAbits.LATA0 = 1;                   // frame timing debug (7.2: measure the conversion time)
#if TEST_FIXED
        // Link test: fixed, recognisable values + a slowly counting one.
        for (uint8_t i = 0; i < S_COUNT; i++) v[i] = (uint16_t)(1000u * (i + 1) + i);
        v[S_KNOB] = n & 0x3FFF;
        n++;
#else
        for (uint8_t i = 0; i < 8; i++) v[CONV[i].slot] = convert(&CONV[i]);
#endif
        LATAbits.LATA0 = 0;
        send_frame(v, seq++);
    }
}
