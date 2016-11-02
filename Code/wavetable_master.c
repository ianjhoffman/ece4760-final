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

// fix16 definition
typedef signed int fix16 ;

// Set up variables for DAC Chip Select
#define SS      LATAbits.LATA4
#define dirSS   TRISAbits.TRISA4

// DMA blend tri/saw test tables
#define WAVE_TABLE_SIZE 512
volatile unsigned int tri_table[WAVE_TABLE_SIZE];
volatile unsigned int saw_table[WAVE_TABLE_SIZE];
//#define exp_table_size 1000
//volatile fix16 exp_table[exp_table_size]; 

#define WAVE_BLEND(sample1, sample2, blend) (((255 - blend) * (sample1 >> 8)) + (blend * (sample2 >> 8)))

// TEST VALUES FOR MORPH TEST
#define TEST_PHASE_INC (2 << 32)
#define TEST_BLEND_AMT 128
#define TEST_BLEND_INC 8192

// the DDS units
volatile unsigned long phase_acc = 0; // synthesis phase acc
volatile unsigned char blend = TEST_BLEND_AMT;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// string buffer
char buffer[60];
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

void table_generation()
{
    /*
     * For testing:
     * Generate saw and tri tables
     */
    int i;
    for (i = 0; i < WAVE_TABLE_SIZE; i++) {
        saw_table[i] = ((WAVE_TABLE_SIZE - 1) - i) << 23; // ramp down
        tri_table[i] = (i < (WAVE_TABLE_SIZE >> 1)) ? 
                            (i << 24)
                        :
                            -((i - (WAVE_TABLE_SIZE >> 1)) << 24) - 1
                        ;
    }
}

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

void initDAC(){
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
volatile unsigned int counter = 0; // FOR TESTING
volatile unsigned int blender = 0;  // FOR TESTING

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void)
{
    mT2ClearIntFlag();
    SS = 0; // CS low during write
            
//    DAC_data = WAVE_BLEND(tri_table[phase_acc>>55], 
//                          saw_table[phase_acc>>55],
//                          blend) >> 20;
    
    blend = (blender >> 24);
    
    DAC_data = WAVE_BLEND(tri_table[counter>>3], 
                          saw_table[counter>>3],
                          blend) >> 20;
    
    // DAC_data = counter;
    
    WriteSPI2( DAC_config_chan_A | DAC_data );
    
    // Update phase accumulator
    // phase_acc += TEST_PHASE_INC;
    
    counter = (counter > 4095) ? 0 : (counter + 1);
    blender += TEST_BLEND_INC;

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
static struct pt pt_timer, pt_color, pt_anim, pt_key ;

// system 1 second interval tick
int sys_time_seconds ;

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
    tft_setCursor(0, 0);
    tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
    tft_writeString("Time in seconds since boot\n");
    while(1) {
        PT_YIELD_TIME_msec(1000);
        sys_time_seconds++;

        // draw sys_time
        // x,y,w,h,radius,color
        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", sys_time_seconds);
        tft_writeString(buffer);
    }
  PT_END(pt);
} // timer thread

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
    PT_INIT(&pt_timer);

    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240

      // Set up timer2 on,  interrupts, internal clock, prescalar 8, toggle rate
      // at 30 MHz PB clock 60 counts is two microsec
    // period of 200
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_2, 200);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag

    initDAC();
    table_generation();

    // round-robin scheduler for threads
    while (1) {
        PT_SCHEDULE(protothread_timer(&pt_timer));
    }
} // main

// === end  ======================================================

