/*
 * File:        wavetable_master.c
 * Authors:     
 *              Ian Hoffman          ( ijh6 )
 *              Joval Mathew         ( jjm448 )
 *              Balazs Szegletes     ( bas332 )
 *
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

// Include wave tables
#include "Basic8Tab.h"
#include "SawSyncEnv.h"
#include "SawFilter.h"
#include "VariedDigital.h"

// Include frequency phase accumulator table
#include "freq_to_accum.h"
// Include tempo phase accumulator table
#include "tempo_to_accum.h"
// Include envelope phase accumulator table
#include "env_keyframes.h"

// Sequencer values
volatile char step_notes[16] = {12, 0, 24, 0,
                                36, 0, 24, 0,
                                12, 0, 19, 0,
                                0, 17, 19, 20};

volatile char steps_on[16] = {1, 0, 1, 0,
                              1, 0, 1, 0,
                              1, 0, 1, 0,
                              0, 1, 1, 1};

volatile unsigned char old_step_select = 0;
volatile unsigned char step_select = 0;
volatile unsigned char note_select = 0;
volatile unsigned char old_step = 0;
volatile unsigned char curr_step = 0;

// Synthesis modulation params
volatile unsigned int amp_env = 0;
volatile unsigned int shape_env = 0;
volatile unsigned int shape_amt = 0;

// Synth mod params translated into concrete values
volatile unsigned int amp_rise_acc = 0;
volatile unsigned int amp_fall_acc = 0;
volatile unsigned int shp_rise_acc = 0;
volatile unsigned int shp_fall_acc = 0;
volatile char amp_rising = 0;
volatile char shp_rising = 0;

// buttons
volatile char seq_toggle = 0;
volatile char tab_toggle = 0;
volatile char rest_toggle = 0;
volatile char note_write = 0;

// button overall states
volatile char seq_active = 0;

// table indexing
static unsigned int *tables[4] = {
    basic_eight,
    sawsync_env,
    saw_filter,
    varied_digital
};
volatile unsigned int table_index = 0;

// Set up variables for DAC Chip Select
#define SS      LATAbits.LATA4
#define dirSS   TRISAbits.TRISA4

#define WAVE_TABLE_SIZE 512
#define MAX_INC 4294967295
//#define exp_table_size 1000
//volatile fix16 exp_table[exp_table_size]; 

// Wave blending macro for wavetable
#define WAVE_BLEND(sample1, sample2, blend) (((255 - blend) * (sample1 >> 8)) + (blend * (sample2 >> 8)))
// Envelope blending macro for modal envelope functionality (4-bit blending)
#define ENV_BLEND(acc1, acc2, blend) (((15 - blend) * (acc1 >> 4)) + (blend * (acc2 >> 4)))

// the DDS units
volatile unsigned int phase_acc = 0; // synthesis phase acc
volatile unsigned int blend = 0;
volatile unsigned int tempo_index = 25; // 120 BPM

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// ===== Pulldown Macros ==========================
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

#define spi_channel 2
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
char stepsel_label[] = "Step select:";
char notesel_label[] = "Note select:";

void initTFT() {
    int i;
    // Draw sequence boxes and on/offs
    for (i = 0; i < 16; i++) {
        tft_drawRect(1 + (i * 20), 115, 18, 100, ILI9340_BLUE);
        tft_drawCircle(9 + (i * 20), 227, 5, ILI9340_CYAN);
    }
    // first sequence box is selected by default 
    tft_drawRect(1, 115, 18, 100, ILI9340_GREEN);
    // Draw separator lines
    tft_drawLine(0, 105, 320, 105, ILI9340_WHITE); // horizontal above seq
    tft_drawLine(107, 0, 107, 105, ILI9340_WHITE); // vertical separator 1
    tft_drawLine(214, 0, 214, 105, ILI9340_WHITE); // vertical separator 2
    tft_drawLine(0, 40, 107, 40, ILI9340_WHITE); // horizontal under text 1
    tft_drawLine(214, 40, 320, 40, ILI9340_WHITE); // horizontal under text 2
    tft_drawLine(107, 53, 214, 53, ILI9340_WHITE); // horizontal middle segment
    
    
    // Draw active sequence
    for (i = 0; i < 16; i++) {
        tft_fillRect(2 + (i * 20), 212 - (step_notes[i] << 1), 16, 2, ILI9340_CYAN);
        if (steps_on[i]) tft_fillCircle(9 + (i * 20), 227, 5, ILI9340_CYAN);
    }
    
    // Draw tempo/table labels
    tft_setTextColor(ILI9340_WHITE);
    tft_setTextSize(3);
    tft_setCursor(7, 10); tft_writeString(tempo_label);
    tft_setCursor(220, 10); tft_writeString(table_label);
    
    // WE'RE GONNA WANT TO DRAW STEP SELECT/NOTE SELECT ON TOP OF SCREEN
}

// Timer 3 interrupt handler
volatile char to_reset = 0;
volatile char update_flag = 0;
void __ISR(_TIMER_3_VECTOR, IPL2AUTO) Timer3Handler(void)
{
    mT3ClearIntFlag();
    // If we switched to a new step, adjust TFT seq readout to reflect that
    if (old_step_select != step_select) {
        tft_drawRect(1 + (old_step_select * 20), 115, 18, 100, 
                (old_step_select == curr_step) ? ILI9340_RED : ILI9340_BLUE);
        tft_drawRect(1 + (step_select * 20), 115, 18, 100, ILI9340_GREEN); 
    }
    if (update_flag) {
        tft_drawRect(1 + (to_reset * 20), 115, 18, 100, 
                (to_reset == step_select) ? ILI9340_GREEN : ILI9340_BLUE);
        tft_drawRect(1 + (curr_step * 20), 115, 18, 100, ILI9340_RED);
        update_flag = 0;
    }
}


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
volatile unsigned int step_accum = 0;
volatile unsigned int amp_val = 0;
volatile unsigned int shp_val = 0;
volatile unsigned int shp_env_to_add = 0;

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void)
{
    mT2ClearIntFlag();
    SS = 0; // CS low during write
    
    // Get blend value
    shp_env_to_add = (shp_val >> 8) * shape_amt; // scale by mod amount
    shp_env_to_add = (MAX_INC - blend > shp_env_to_add) ? shp_env_to_add : (MAX_INC - blend);
    blender = (blend + shp_env_to_add) >> 21; // Get top 11 bits
    
    // Get sub-tables to blend between
    tab1_offset = ((blender & 0x700)) << 1;
    tab2_offset = ((blender & 0x700) + 0x100) << 1;
    
    // Get sample to write
    // index of blend is bottom 8 bits of blend
    // (tables[table_index])
    DAC_data = WAVE_BLEND((((tables[table_index]) + tab1_offset)[phase_acc>>23]), 
                          (((tables[table_index]) + tab2_offset)[phase_acc>>23]),
                          (blender & 0xff));
    
    // Scale output by amp envelope
    DAC_data = (DAC_data >> 8) * (amp_val >> 24);
    
    // Write sample to DAC, shifting into DAC's 12-bit range
    WriteSPI2( DAC_config_chan_A | DAC_data >> 20 );
    
    // Update phase accumulator
    phase_acc += freq_accumulators[step_notes[curr_step]];
    
    // Update step in sequence
    step_accum += (seq_active) ? tempo_accumulators[tempo_index] : 0;
    old_step = curr_step;
    curr_step = step_accum >> 28; // 0 thru 15
    
    // Update amp envelope value
    if (amp_rising) {
        amp_val = (MAX_INC - amp_val < amp_rise_acc) ? MAX_INC : (amp_val + amp_rise_acc);
        if (amp_val == MAX_INC) amp_rising = 0;
    } else {
        amp_val = (amp_val < amp_fall_acc) ? 0 : (amp_val - amp_fall_acc);
    }
    // Update shape envelope value
    if (shp_rising) {
        shp_val = (MAX_INC - shp_val < shp_rise_acc) ? MAX_INC : (shp_val + shp_rise_acc);
        if (shp_val == MAX_INC) shp_rising = 0;
    } else {
        shp_val = (shp_val < shp_fall_acc) ? 0 : (shp_val - shp_fall_acc);
    }
    
    // If we switched to a new step, adjust TFT seq readout to reflect that
    // and set envelopes to rising if the note is not a rest
    if (old_step != curr_step) {
        to_reset = old_step;
        update_flag = 1;
        amp_rising = steps_on[curr_step];
        shp_rising = amp_rising;
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
static struct pt pt_tft, pt_mux, pt_button;

char test_buffer[60];
volatile int wait;
static PT_THREAD (protothread_mux(struct pt *pt)){
    PT_BEGIN(pt);
    while(1){

        // Channel 0 (0b000) : tempo
        PORTClearBits(IOPORT_B, BIT_7|BIT_8|BIT_9);
        wait = 0; while(wait < 40) wait++;
        AcquireADC10();
        wait = 0; while(wait < 20) wait++;
        tempo_index = ReadADC10(0) >> 4;
        
        // Channel 1 (0b001) : step select
        PORTSetBits(IOPORT_B, BIT_7);
        wait = 0; while(wait < 40) wait++;
        AcquireADC10();
        wait = 0; while(wait < 20) wait++;
        old_step_select = step_select;
        step_select = ReadADC10(0) >> 6;
        
        // Channel 2 (0b010) : note select
        PORTToggleBits(IOPORT_B, BIT_7|BIT_8);
        wait = 0; while(wait < 40) wait++;
        AcquireADC10();
        wait = 0; while(wait < 20) wait++;
        note_select = ReadADC10(0)/20;
        if (note_select > 48) note_select = 48; // don't flow into indices 49+

        // Channel 3 (0b011) : blend
        PORTSetBits(IOPORT_B, BIT_7);
        wait = 0; while(wait < 40) wait++;
        AcquireADC10();
        wait = 0; while(wait < 20) wait++;
        blend = ReadADC10(0) << 22;
        // Don't flow into 8 -> 9 fade, that's bad/doesn't exist
        if ((blend >> 29) > 6) blend = 3758096384;
        
        // Channel 4 (0b100) : amp envelope
        PORTToggleBits(IOPORT_B, BIT_7|BIT_8|BIT_9);
        wait = 0; while(wait < 40) wait++;
        AcquireADC10();
        wait = 0; while(wait < 20) wait++;
        amp_env = ReadADC10(0);
        // Set amp rise and fall accumulators
        amp_rise_acc = ENV_BLEND((atk_incs[amp_env >> 7]), 
                (atk_incs[(amp_env >> 7) + 1]), (amp_env & 0xf));
        amp_fall_acc = ENV_BLEND((decay_incs[amp_env >> 7]), 
                (decay_incs[(amp_env >> 7) + 1]), (amp_env & 0xf));
        
        // we skip pin six completely
        
        // Channel 6 (0b110) : shape envelope
        PORTSetBits(IOPORT_B, BIT_8);
        wait = 0; while(wait < 40) wait++;
        AcquireADC10();
        wait = 0; while(wait < 20) wait++;
        shape_env = ReadADC10(0);
        // Set shape rise and fall accumulators
        shp_rise_acc = ENV_BLEND((atk_incs[shape_env >> 7]), 
                (atk_incs[(shape_env >> 7) + 1]), (shape_env & 0xf));
        shp_fall_acc = ENV_BLEND((atk_incs[shape_env >> 7]), 
                (decay_incs[(shape_env >> 7) + 1]), (shape_env & 0xf));
        
        // Channel 7 (0b111) : shape amount
        PORTSetBits(IOPORT_B, BIT_7);
        wait = 0; while(wait < 40) wait++;
        AcquireADC10();
        wait = 0; while(wait < 20) wait++;
        shape_amt = ReadADC10(0) >> 2; // 0 to 255
         
        PT_YIELD_TIME_msec(50); // run approx. 20Hz
    }
    PT_END(pt);
}
// system 1 second interval tick
int sys_time_seconds ;

volatile char tab_number[2];
static PT_THREAD (protothread_tft(struct pt *pt)) {
    PT_BEGIN(pt);
    while(1) {
        // Write BPM to screen
        tft_fillRect(9, 59, 85, 20, ILI9340_BLACK);
        sprintf(test_buffer, "%d BPM", tempo_vals[tempo_index]);
        tft_setTextColor(ILI9340_WHITE);
        tft_setTextSize(2);
        tft_setCursor(10, 60); tft_writeString(test_buffer);
        
        // Write current table to screen
        tft_fillRect(255, 59, 30, 20, ILI9340_BLACK);
        sprintf(tab_number, "%d", table_index + 1);
        tft_setTextColor(ILI9340_WHITE);
        tft_setCursor(256, 60); tft_writeString(tab_number);
        
        PT_YIELD_TIME_msec(100);
    }
    PT_END(pt);
}

volatile char prev_seq_toggle = 0;
volatile char prev_tab_toggle = 0;
volatile char prev_rest_toggle = 0;
volatile char prev_note_write = 0;
static PT_THREAD (protothread_button(struct pt *pt)) {
    PT_BEGIN(pt);
    while(1) {
        // Save previous values
        prev_seq_toggle = seq_toggle;
        prev_tab_toggle = tab_toggle;
        prev_rest_toggle = rest_toggle;
        prev_note_write = note_write;
        
        // Read in the four buttons
        seq_toggle = mPORTAReadBits(BIT_1);
        tab_toggle = mPORTBReadBits(BIT_3);
        rest_toggle = mPORTAReadBits(BIT_2); 
        note_write = mPORTAReadBits(BIT_3);
        
        // If they were just pressed, do stuff
        // Turn the music sequence on and off
        if ((seq_toggle ^ prev_seq_toggle) && seq_toggle) seq_active ^= 1;
        
        // Cycle through the wave table
        if ((tab_toggle ^ prev_tab_toggle) && tab_toggle) {
            table_index = (table_index == 3) ? 0 : (table_index + 1);
        }
        // Toggle rests on the selected (green) note
        if ((rest_toggle ^ prev_rest_toggle) && rest_toggle) {
            steps_on[step_select] ^= 1;
            // Draw rest/not_rest
            tft_fillCircle(9 + (step_select * 20), 227, 5, 
                (steps_on[step_select]) ? ILI9340_CYAN : ILI9340_BLACK);
            if (!steps_on[step_select]) {
                tft_drawCircle(9 + (step_select * 20), 227, 5, ILI9340_CYAN);
            }
        }
        // Save the current note to the sequence 
        if ((note_write ^ prev_note_write) && note_write) {
            // Erase old note
            tft_fillRect(2 + (step_select * 20), 
                212 - (step_notes[step_select] << 1), 16, 2, ILI9340_BLACK);
            // Change note
            step_notes[step_select] = note_select;
            // Draw new note
            tft_fillRect(2 + (step_select * 20), 
                212 - (step_notes[step_select] << 1), 16, 2, ILI9340_CYAN);
        }
        
        PT_YIELD_TIME_msec(50);
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
    PT_INIT(&pt_button);

    initADC();
    
    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(1); // Use tft_setRotation(1) for 320x240
    
    initTFT(); // Initialize the TFT

    // Set up timer2 on,  interrupts, internal clock, 100KHz
    // period of 200
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_2, 200);
    // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag

    // Turn on Timer3 for the note_select TFT update
    // period of 10550 and prescalar of 64 has frequency of 14.9998 Hz 
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_32, 41667);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    mT3ClearIntFlag(); // and clear the interrupt flag

    initDAC();
    
    // Set the MUX Channel Select Pins as outputs (pins 16, 17, 18)
    mPORTBSetPinsDigitalOut(BIT_7|BIT_8|BIT_9);
    
    // Set button digital input pins (RA1, RB3, RA2, RA3), for buttons
    mPORTASetPinsDigitalIn(BIT_1|BIT_2|BIT_3);
    mPORTBSetPinsDigitalIn(BIT_3);
    
    // round-robin scheduler for threads
    while (1) {
        PT_SCHEDULE(protothread_tft(&pt_tft));
        PT_SCHEDULE(protothread_mux(&pt_mux));
        PT_SCHEDULE(protothread_button(&pt_button));
    }
} // main

// === end  ======================================================

