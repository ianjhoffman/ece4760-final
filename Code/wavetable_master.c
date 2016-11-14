/*
 * File:        TFT_keypad_BRL4.c
 * Author:      Bruce Land
 * Adapted from:
 *              main.c by
 * Author:      Syed Tahmid Mahbub
 * Target PIC:  PIC32MX250F128B
 */

// graphics libraries
#include "config.h"
#include "tft_master.h"
#include "tft_gfx.h"
#include <inttypes.h>
#include <stdlib.h>
#include <plib.h>
#include "pt_cornell_TFT.h"

// Include basic 8 waveform table
#include "Basic8Tab.h"
// #include "SawSyncEnv.h"

// Include frequency phase accumulator table
#include "freq_to_accum.h"
// Include tempo phase accumulator table
#include "tempo_to_accum.h"

// Sequencer values
char step_notes[16] = {12, 0, 24, 0,
                       36, 0, 24, 0,
                       12, 0, 19, 0,
                       0, 17, 19, 20};
char steps_on[16] = {1, 0, 1, 0,
                     1, 0, 1, 0,
                     1, 0, 1, 0,
                     0, 1, 1, 1};
char curr_step_edit = 0;

#define TEST_TABLE basic_eight

// fix16 definition
typedef signed int fix16 ;

// Set up variables for DAC Chip Select
#define SS      LATAbits.LATA4
#define dirSS   TRISAbits.TRISA4

// DMA blend tri/saw test tables
#define WAVE_TABLE_SIZE 512
//#define exp_table_size 1000
//volatile fix16 exp_table[exp_table_size]; 

// Wave blending macro for wavetable
#define WAVE_BLEND(sample1, sample2, blend) (((255 - blend) * (sample1 >> 8)) + (blend * (sample2 >> 8)))

// the DDS units
//volatile unsigned long phase_acc = 0; // synthesis phase acc
volatile unsigned int phase_acc = 0; // synthesis phase acc
volatile unsigned int blend = 0;
volatile unsigned int tempo_index = 25; // 120 BPM

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// === 16:16 fixed point macros ==========================================
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a) abs(a)
#define onefix16 0x00010000 // int2fix16(1)

//=====Pulldown Macros==========================
// PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;

// SPI SETUP FOR DAC
#define config1 SPI_MODE16_ON | SPI_CKE_ON | MASTER_ENABLE_ON
/*    
 *    FRAME_ENABLE_OFF
 *    ENABLE_SDO_PIN        -> SPI Output pin enabled
 *    SPI_MODE16_ON         -> 16-bit SPI mode
 *    SPI_SMP_OFF           -> Sample at middle of data output time
 *    SPI_CKE_ON            -> Output data changes on transition from active clock
 *                             to idle clock state
 *    SLAVE_ENABLE_OFF      -> Manual SW control of SS
 *    MASTER_ENABLE_ON      -> Master mode enable
 */

#define config2 SPI_ENABLE
/*    SPI_ENABLE    -> Enable SPI module
 */

#define spi_channel    2
// Use channel 2 since channel 1 is used by TFT display

#define spi_divider 2
/* Unlike OpenSPIx(), config for SpiChnOpen describes the non-default
 * settings. eg for OpenSPI2(), use SPI_SMP_OFF (default) to sample
 * at the middle of the data output, use SPI_SMP_ON to sample at end. For
 * SpiChnOpen, using SPICON_SMP as a parameter will use the non-default
 * SPI_SMP_ON setting.
 */
#define config SPI_OPEN_MSTEN | SPI_OPEN_MODE16 | SPI_OPEN_DISSDI | SPI_OPEN_CKE_REV
/*    SPI_OPEN_MSTEN        -> Master mode enable
 *    SPI_OPEN_MODE16        -> 16-bit SPI mode
 *    SPI_OPEN_DISSDI        -> Disable SDI pin since PIC32 to DAC is a
 *                            master-to-slave    only communication
 *    SPI_OPEN_CKE_REV    -> Output data changes on transition from active
 *                            clock to idle clock state
 */
// END SPI SETUP FOR DAC

void initDAC() {
    /* Steps:
     *    1. Setup SS as digital output.
     *    2. Map SDO to physical pin.
     *    3. Configure SPI control and clock with either of a or b:
     *        a. OpenSPIx(config1, config2) and SpiChnSetBrg(spi_channel, spi_brg)
     *        b. SpiChnOpen(spi_channel, config, spi_divider)
     */

    dirSS = 0;                     // make SS an output
    SS = 1;                        // set SS = 1 to deselect slave
    PPSOutput(2, RPB5, SDO2);      // map SDO2 to RB5
    OpenSPI2(config1, config2);    // see pg 193 in plib reference
    SpiChnOpen(spi_channel, config, spi_divider);
}

void initADC() {
    CloseADC10();
    
    #define PARAM1 ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_OFF
    #define PARAM2 ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2
    #define PARAM4 ENABLE_AN0_ANA
    #define PARAM5 SKIP_SCAN_ALL

    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN0 );
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 );
    EnableADC10();
}

// TFT constants
char tempo_label[] = "TEMPO";
char table_label[] = "TABLE";

void initTFT() {
    int i;
    // Draw sequence boxes and on/offs
    for (i = 0; i < 16; i++) {
        tft_drawRect(1 + (i * 20), 115, 18, 100, ILI9340_BLUE);
        tft_drawCircle(9 + (i * 20), 227, 5, ILI9340_CYAN);
    }
    // Draw separator lines
    tft_drawLine(0, 105, 320, 105, ILI9340_WHITE); // horizontal above seq
    tft_drawLine(160, 0, 160, 105, ILI9340_WHITE); // vertical separator
    tft_drawLine(0, 40, 320, 40, ILI9340_WHITE); // horizontal under text
    
    // Draw active sequence
    for (i = 0; i < 16; i++) {
        tft_fillRect(2 + (i * 20), 212 - (step_notes[i] << 1), 16, 2, ILI9340_CYAN);
        if (steps_on[i]) tft_fillCircle(9 + (i * 20), 227, 5, ILI9340_CYAN);
    }
    
    // Draw tempo/table labels
    tft_setTextColor(ILI9340_WHITE);
    tft_setTextSize(3);
    tft_setCursor(36, 10); tft_writeString(tempo_label);
    tft_setCursor(197, 10); tft_writeString(table_label);
}

/* ====== MCP4822 control word ==============
bit 15 A/B: DACA or DACB Selection bit
1 = Write to DACB
0 = Write to DACA
bit 14 ? Don?t Care
bit 13 GA: Output Gain Selection bit
1 = 1x (VOUT = VREF * D/4096)
0 = 2x (VOUT = 2 * VREF * D/4096), where internal VREF = 2.048V.
bit 12 SHDN: Output Shutdown Control bit
1 = Active mode operation. VOUT is available. ?
0 = Shutdown the selected DAC channel. Analog output is not available at the channel that was shut down.
VOUT pin is connected to 500 k???typical)?
bit 11-0 D11:D0: DAC Input Data bits. Bit x is ignored.
*/


//== Timer 2 interrupt handler ===========================================
volatile unsigned int DAC_data ;// output value
//volatile unsigned int exp_index; volatile fix16 multiplier; // Values used for the ramping
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2 ; // 20 MHz max speed for this DAC
//volatile uint8_t index; //indices for sine table
//volatile unsigned int counter = 0; // FOR TESTING
//volatile unsigned int blender = 0;  // FOR TESTING
//
//volatile unsigned int blend_inc = TEST_BLEND_INC;
//volatile signed short blend_mod_inc = 250;
volatile unsigned int tab1_offset;
volatile unsigned int tab2_offset;
volatile unsigned int blender;

volatile unsigned char old_step = 0;
volatile unsigned char curr_step = 0;
volatile unsigned int step_accum = 0;

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void)
{
    mT2ClearIntFlag();
    SS = 0; // CS low during write
    
    // Get blend value
    blender = blend >> 21; // Get top 11 bits
    
    // Get sub-tables to blend between
    tab1_offset = ((blender & 0x700)) << 1;
    tab2_offset = ((blender & 0x700) + 0x100) << 1;
    
    // Get sample to write
    // index of blend is bottom 8 bits of blend
    DAC_data = WAVE_BLEND(((TEST_TABLE + tab1_offset)[phase_acc>>23]), 
                          ((TEST_TABLE + tab2_offset)[phase_acc>>23]),
                          (blender & 0xff)) >> 20;
    
    if (!steps_on[curr_step]) DAC_data = 0; // Rest
    
    WriteSPI2( DAC_config_chan_A | DAC_data );
    
    // Update phase accumulator
    phase_acc += freq_accumulators[step_notes[curr_step]];
    
    // Update step in sequence
    step_accum += tempo_accumulators[tempo_index];
    old_step = curr_step;
    curr_step = step_accum >> 28; // 0 thru 15
    // If we switched to a new step, adjust TFT seq readout to reflect that
    if (old_step != curr_step) {
        tft_drawRect(1 + (old_step * 20), 115, 18, 100, 
            (old_step == curr_step_edit ? ILI9340_GREEN : ILI9340_BLUE));
        tft_drawRect(1 + (curr_step * 20), 115, 18, 100, ILI9340_RED); 
    }

    // test for ready
    while (TxBufFullSPI2());
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // CS high
    SS = 1;
}

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_tft, pt_mux;

static int step_select, note_select, shape_attack, shape_decay, amp_attack, amp_decay;
static int tempo;

char test_buffer[60];

static PT_THREAD (protothread_mux(struct pt *pt)){
    PT_BEGIN(pt);
    while(1){
                
        // Set Channel to zero
        // read Input Pin
        
        tft_fillRect(19, 49, 100, 20, ILI9340_BLACK);
        
        PORTClearBits(IOPORT_B, BIT_7|BIT_8|BIT_9);
//        Nop(); Nop(); Nop(); // for safety
         
        sprintf(test_buffer, "%d BPM", tempo_vals[tempo_index]);
        tft_setTextColor(ILI9340_WHITE);
        tft_setTextSize(2);
        tft_setCursor(20, 50); tft_writeString(test_buffer);

        tempo_index = ReadADC10(0) >> 4;
        AcquireADC10();
        
        // Set Channel to one
//        PORTToggleBits(IOPORT_B, BIT_7);
//        Nop(); Nop(); Nop(); // for safety
//        step_select = ReadADC10(0);
//        AcquireADC10();
//        // Set Channel to two
//        PORTToggleBits(IOPORT_B, BIT_7|BIT_8);
//        Nop(); Nop(); Nop(); // for safety
//        note_select = ReadADC10(0);
//        AcquireADC10();
        // Set Channel to three
//        PORTToggleBits(IOPORT_B, BIT_7);
//        PT_YIELD_TIME_msec(1);
//        AcquireADC10();
//        blend = ReadADC10(0) << 22;
//        // Don't flow into 8 -> 9 fade, that's bad/doesn't exist
//        if ((blend >> 29) > 6) blend = 3758096384;
        // Set Channel to four
//        PORTToggleBits(IOPORT_B, BIT_7|BIT_8|BIT_9);
//        Nop(); Nop(); Nop(); // for safety
//        shape_attack = ReadADC10(0);
//        AcquireADC10();
//        // Set Channel to five
//        PORTToggleBits(IOPORT_B, BIT_7);
//        Nop(); Nop(); Nop(); // for safety
//        shape_decay = ReadADC10(0);
//        AcquireADC10();
//        // Set Channel to six
//        PORTToggleBits(IOPORT_B, BIT_7|BIT_8);
//        Nop(); Nop(); Nop(); // for safety
//        amp_attack = ReadADC10(0);
//        AcquireADC10();
//        // Set Channel to seven
//        PORTToggleBits(IOPORT_B, BIT_7);
//        Nop(); Nop(); Nop(); // for safety
//        amp_decay = ReadADC10(0);
//        AcquireADC10();
        // Clear all the bits, yield

        PT_YIELD_TIME_msec(50); // run approx. 20Hz
         
    }
    PT_END(pt);
}
// system 1 second interval tick
int sys_time_seconds ;

// === Timer Thread =================================================
// update a 1 second tick counter
//static PT_THREAD (protothread_timer(struct pt *pt))
//{
//    PT_BEGIN(pt);
//    tft_setCursor(0, 0);
//    tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
//    tft_writeString("Time in seconds since boot\n");
//    while(1) {
//        PT_YIELD_TIME_msec(1000);
//        sys_time_seconds++;
//
//        // draw sys_time
//        // x,y,w,h,radius,color
//        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);
//        tft_setCursor(0, 10);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(buffer, "%d", sys_time_seconds);
//        tft_writeString(buffer);
//    }
//  PT_END(pt);
//} // timer thread

static PT_THREAD (protothread_tft(struct pt *pt)) {
    PT_BEGIN(pt);
    while(1) {
        PT_YIELD_TIME_msec(1000);
    }
    PT_END(pt);
}

// === Main  ======================================================
void main(void) {
    SYSTEMConfigPerformance(PBCLK);
    ANSELA = 0; ANSELB = 0; CM1CON = 0; CM2CON = 0;

    // === config threads ==========
    // turns OFF UART support and debugger pin
    PT_setup();

    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

    // init the threads
    PT_INIT(&pt_tft);
    PT_INIT(&pt_mux);

    initADC();
    
    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(1); // Use tft_setRotation(1) for 320x240
    
    initTFT(); // Initialize the TFT

      // Set up timer2 on,  interrupts, internal clock, prescalar 8, toggle rate
      // at 30 MHz PB clock 60 counts is two microsec
    // period of 200
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_2, 200);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag

    initDAC();
    // Set the MUX Channel Select Pins as outputs
    mPORTBSetPinsDigitalOut(BIT_7 | BIT_8 | BIT_9);    //Set PINS 16,17,18 as outputs
    
    // round-robin scheduler for threads
    while (1) {
        PT_SCHEDULE(protothread_tft(&pt_tft));
        PT_SCHEDULE(protothread_mux(&pt_mux));
    }
} // main

// === end  ======================================================

