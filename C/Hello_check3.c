/**
 * @brief Red Pitaya PID Controller
 *
 * @Author Ales Bardorfer <ales.bardorfer@redpitaya.com>
 *         
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include "../../apps-free/scope/src/main.h"
#include "../../apps-free/scope/src/version.h"
#include "../../apps-free/scope/src/worker.h"
#include "../../apps-free/scope/src/fpga.h"
#include "../../apps-free/scope/src/calib.h"
#include "../../apps-free/scope/src/generate.h"
#include "../../apps-free/scope/src/pid.h" 
#include "../../apps-free/scope/src/fpga_awg.h"
/* I have included the required files now let me check if I can import any command*/



int main()
{
    printf("This works with modification 3");

    /*typedef enum {
    APPLE, 
    ORANGE = 3,
    PEAR 
    } ft;

    printf("%c" ,my_ft = APPLE);
       //int generate_update(rp_app_params_t *params);*/
    

float ampl1 = 0.2;
float freq = 10000;
int calib_dc_offs1 =0;
int calib_fs1 =150000000;
float max_dac_v1 =1;
float user_dc_offs1=0.5;
awg_signal_t type1 = 0;
int32_t data1 =12;
awg_param_t awg1 = awg1; 

  void synthesize_signal(ampl1, freq1, calib_dc_offs1, calib_fs1,
                       max_dac_v1, user_dc_offs1, type1, 
                        data1,  *awg);
}

/** Maximal signal amplitude [Vpp]. */
const double c_max_amplitude = 2.0;

/** Temporary data buffer, used during signal shape calculation. */
int32_t ch1_data[AWG_SIG_LEN];
int32_t ch2_data[AWG_SIG_LEN];

/** Pointer to externally defined calibration parameters. */
rp_calib_params_t *gen_calib_params = NULL;

/** Maximal Signal Voltage on DAC outputs on channel A. It is expressed in [V],
 * and calculated from apparent Back End Full Scale calibration parameter.
 */
float ch1_max_dac_v;

/** Maximal Signal Voltage on DAC outputs on channel B. It is expressed in [V],
 * and calculated from apparent Back End Full Scale calibration parameter.
 */
float ch2_max_dac_v;

/** Predefined File name, used for definition of arbitrary Signal Shape. */
const char *gen_waveform_file1="/tmp/gen_ch1.csv";
const char *gen_waveform_file2="/tmp/gen_ch2.csv";


/*----------------------------------------------------------------------------------*/
/**
 * Synthesize a desired signal.
 *
 * Generates/synthesizes a signal, based on three predefined signal
 * types/shapes, signal amplitude & frequency. The data[] vector of 
 * samples at 125 MHz is generated to be re-played by the FPGA AWG module.
 *
 * @param[in]  ampl           Signal amplitude [Vpp].
 * @param[in]  freq           Signal frequency [Hz].
 * @param[in]  calib_dc_offs  Calibrated Instrument DC offset
 * @param[in]  max_dac_v      Maximum DAC voltage in [V]
 * @param[in]  user_dc_offs   User defined DC offset
 * @param[in]  type           Signal type/shape [Sine, Square, Triangle].
 * @param[in]  data           Returned synthesized AWG data vector.
 * @param[out] awg            Returned AWG parameters.
 */
void synthesize_signal(float ampl, float freq, int calib_dc_offs, int calib_fs,
                       float max_dac_v, float user_dc_offs, awg_signal_t type, 
                       int32_t *data, awg_param_t *awg) 
{
    uint32_t i;

    /* Various locally used constants - HW specific parameters */
    const int trans0 = 30;
    const int trans1 = 300;
    const float tt2 = 0.249;
    const int c_dac_max =  (1 << (c_awg_fpga_dac_bits - 1)) - 1;
    const int c_dac_min = -(1 << (c_awg_fpga_dac_bits - 1));

    int trans = round(freq / 1e6 * ((float) trans1)); /* 300 samples at 1 MHz */
    int user_dc_off_cnt = 
        round((1<<(c_awg_fpga_dac_bits-1)) * user_dc_offs / max_dac_v);
    uint32_t amp; 

    /* Saturate offset - depending on calibration offset, it could overflow */
    int offsgain = calib_dc_offs + user_dc_off_cnt;
    offsgain = (offsgain > c_dac_max) ? c_dac_max : offsgain;
    offsgain = (offsgain < c_dac_min) ? c_dac_min : offsgain;

    awg->offsgain = (offsgain << 16) | 0x2000;
    awg->step = round(65536.0 * freq/c_awg_smpl_freq * ((float) AWG_SIG_LEN));
    awg->wrap = round(65536 * AWG_SIG_LEN - 1);
    
    //= (ampl) * (1<<(c_awg_fpga_dac_bits-2));
    //fpga_awg_calc_dac_max_v(calib_fs)
    
    amp= round(ampl/2/fpga_awg_calc_dac_max_v(calib_fs)* c_dac_max );
    
    /* Truncate to max value */
    amp = (amp > c_dac_max) ? c_dac_max : amp;

    if (trans <= 10) {
        trans = trans0;
    }

    /* Fill data[] with appropriate buffer samples */
    for(i = 0; i < AWG_SIG_LEN; i++) {
        /* Sine */
        if (type == eSignalSine) {
            data[i] = round(amp * cos(2*M_PI*(float)i/(float)AWG_SIG_LEN));
        }
 
        /* Square */
        if (type == eSignalSquare) {
            data[i] = round(amp * cos(2*M_PI*(float)i/(float)AWG_SIG_LEN));
            data[i] = (data[i] > 0) ? amp : -amp;

            /* Soft linear transitions */
            float mm, qq, xx, xm;
            float x1, x2, y1, y2;    

            xx = i;
            xm = AWG_SIG_LEN;
            mm = -2.0*(float)amp/(float)trans; 
            qq = (float)amp * (2 + xm/(2.0*(float)trans));

            x1 = xm * tt2;
            x2 = xm * tt2 + (float)trans;

            if ( (xx > x1) && (xx <= x2) ) {

                y1 = (float)amp;
                y2 = -(float)amp;

                mm = (y2 - y1) / (x2 - x1);
                qq = y1 - mm * x1;

                data[i] = round(mm * xx + qq); 
            }

            x1 = xm * 0.75;
            x2 = xm * 0.75 + trans;

            if ( (xx > x1) && (xx <= x2)) {  

                y1 = -(float)amp;
                y2 = (float)amp;

                mm = (y2 - y1) / (x2 - x1);
                qq = y1 - mm * x1;

                data[i] = round(mm * xx + qq); 
            }
        }

        /* Triangle */
        if (type == eSignalTriangle) {
            data[i] = round(-1.0 * (float)amp *
                     (acos(cos(2*M_PI*(float)i/(float)AWG_SIG_LEN))/M_PI*2-1));
        }
    }
}


/*----------------------------------------------------------------------------------*/
/**
 * @brief Read Time Based Signal Definition from the file system
 *
 * The gen_waveform_file is a simple text file, constituted from lines of triple
 * data: time_base, channel A and channel B values. At most AWG_SIG_LEN lines are
 * parsed and apparent data are put to the specified buffers. It is expected the
 * specified buffers are large enough, no check is made within a function. After the
 * Signal Definition is read from a file, the final Signal Shape is calculated with
 * calculate_data() function.
 *
 * @param[out]  time_vect Time base vector
 * @param[out]  ch1_data  Channel A buffer
 * @param[out]  ch2_data  Channel B buffer
 * @retval      -1        Failure, error message is output on standard error
 * @retval      >0        Number of parsed lines
 */
int read_in_file(int chann,  float *ch_data)
{
    FILE *fi = NULL;
    int i, read_size, samples_read = 0;

    /* open file */
    if (chann == 1) {

        fi = fopen(gen_waveform_file1, "r+");
        if (fi == NULL) {
            fprintf(stderr, "read_in_file(): Can not open input file (%s): %s\n",
                    gen_waveform_file1, strerror(errno));
            return -1;
        }

    } else {

        fi = fopen(gen_waveform_file2, "r+");
        if (fi == NULL) {
            fprintf(stderr, "read_in_file(): Can not open input file (%s): %s\n",
                    gen_waveform_file2, strerror(errno));
            return -1;
        }
    }

    /* parse at most AWG_SIG_LEN lines and save data to the specified buffers */
    for (i = 0; i < AWG_SIG_LEN; i++) {

        read_size = fscanf(fi, "%f \n", &ch_data[i]);
        if((read_size == EOF) || (read_size != 1)) {
            i--;
            break;
        }
    }
    samples_read = i + 1;

    if (samples_read >= AWG_SIG_LEN)
        samples_read = AWG_SIG_LEN - 1;

    /* check for errors */
    if (i == 0) {
        fprintf(stderr, "read_in_file() cannot read in signal, wrong format?\n");
        fclose(fi);
        return -1;
    }

    /* close a file */
    fclose(fi);

    /* and return the number of parsed lines */
    return samples_read;
}


/*----------------------------------------------------------------------------------*/
/**
 * @brief Calculate Signal Shape based on Time Based Signal definition

 * Function is intended to calculate the shape of utput Signal for the individual channel,
 * i.e. a function must be called separatelly for each signal.
 * Time Based Signal definition is taken from input parameters in_data. time_vect and
 * in_data_len arguments. The output signal shape is additionally parameterized with
 * amp, calib_dc_offs, max_dac_v and user_dc_offs arguments, which depict properties of
 * referenced channel. Calculated Signal Shape is returned in the specified out_data buffer.
 * Beside of shape calculation the apparent AWG settings for referenced channel are calculated.
 *
 * @param[in]  in_data        Array of signal values, applied at apparent time from Time Based vector
 * @param[in]  time_vect      Array, specifying Time Based values
 * @param[in]  in_data_len    Number of valid entries in the specified buffers
 * @param[in]  amp            Maximal amplitude of calculated Signal
 * @param[in]  calib_dc_offs  Calibrated DC offset, specified in ADC counts
 * @param[in]  max_dac_v      Maximal Voltage on DAC outputs, expressed in [V]
 * @param[in]  user_dc_offs   Configurable DC offset, expressed in [V]
 * @param[out] out_data       Output Signal Shape data buffer
 * @param[out] awg            Modified AWG settings with updated offsgain, step and wrap parameters

 * @retval -1  Failure, error message is output on standard error
 * @retval  0  Success
 */
int  calculate_data(float *in_data, int in_data_len, 
                    float amp, float freq, int calib_dc_offs, uint32_t calib_fs, float max_dac_v, 
                    float user_dc_offs, int32_t *out_data, awg_param_t *awg)
{
    const int c_dac_max =  (1 << (c_awg_fpga_dac_bits - 1)) - 1;
    const int c_dac_min = -(1 << (c_awg_fpga_dac_bits - 1));

    float max_amp, min_amp;
    float k_norm;
    int i, j;

    /* calculate configurable DC offset, expressed in ADC counts */
    int user_dc_off_cnt = 
        round((1<<(c_awg_fpga_dac_bits-1)) * user_dc_offs / max_dac_v);

    /* partial check for validity of input parameters */
    if((in_data == NULL) || 
       (out_data == NULL) || (awg == NULL) || (in_data_len >= AWG_SIG_LEN)) {
        fprintf(stderr, "Internal error, the Time Based signal definition is not correctly specified.\n");
        return -1;
    }

    /* Saturate offset - depending on calibration offset, it could overflow */
    int offsgain = calib_dc_offs + user_dc_off_cnt;
    offsgain = (offsgain > c_dac_max) ? c_dac_max : offsgain;
    offsgain = (offsgain < c_dac_min) ? c_dac_min : offsgain;

    /* modify AWG settings  */
    awg->offsgain = (offsgain << 16) | 0x2000;
    awg->step = round(65536 * freq/c_awg_smpl_freq * in_data_len); 
    awg->wrap = round(65536 * in_data_len - 1);
    
    /* Retrieve max amplitude of the specified Signal Definition, it is used for the normalization */
    max_amp = -1e30;
    min_amp = +1e30;
    
    for(i = 0; i < in_data_len; i++) {
        max_amp = (in_data[i] > max_amp) ? in_data[i] : max_amp;
        min_amp = (in_data[i] < min_amp) ? in_data[i] : min_amp;
    }
    
    /* Calculate normalization factor */
    if ((max_amp - min_amp) == 0) {
        k_norm = (max_amp == 0) ? 0 : (float)(c_dac_max) * amp /(max_amp*2) / fpga_awg_calc_dac_max_v(calib_fs);
    } 
    else {
        k_norm = (float)(c_dac_max) * amp /(max_amp - min_amp) / fpga_awg_calc_dac_max_v(calib_fs);
    }

    /* Normalize Signal values */
    for(i = 0; i < in_data_len; i++) {

        out_data[i] = round(k_norm * (in_data[i] - (max_amp + min_amp)/2));

        /* Clipping */
        if (out_data[i] > c_dac_max)
            out_data[i] = c_dac_max;
        if (out_data[i] < c_dac_min)
            out_data[i] = c_dac_min;
    }

    /* ...and pad it with zeros */
    // TODO: Really from j = i+1 ??? Should be from j = i IMHO.
    for(j = i+1; j < AWG_SIG_LEN; j++) {
        out_data[j] = 0;
    }

    return 0;
}

/*----------------------------------------------------------------------------------*/
/**
 * @brief Write synthesized data[] to FPGA buffer.
 *
 * @param[in] ch         Channel number [0, 1].
 * @param[in] mode       Trigger mode: 0 - continuous, 1 - single, 2 - external
 * @param[in] trigger    Trigger one pulse (if mode == single).
 * @param[in] data       AWG synthesized data to be written to FPGA.
 * @param[in] awg        AWG parameters to be written to FPGA.
 */
void write_data_fpga(uint32_t ch, int mode, int trigger, const int32_t *data,
                     const awg_param_t *awg, int wrap) 
{
    uint32_t i;
    int mode_mask = 0;
    uint32_t state_machine = g_awg_reg->state_machine_conf;

    switch(mode) {
    case 0: /* continuous */
        if (wrap == 1) {
            mode_mask = 0x01;
        } else {
            mode_mask = 0x11;
        }
        break;

    case 1: /* single */
        if (trigger) {
            mode_mask = 0x21;
        } else {
            mode_mask = 0x20;
        }
        break;

    case 2: /* external */
        mode_mask = 0x22;
        break;
    }

    if(ch == 0) {
        /* Channel A */
        state_machine &= ~0xff;

        g_awg_reg->state_machine_conf = state_machine | 0xC0;
        g_awg_reg->cha_scale_off      = awg->offsgain;
        g_awg_reg->cha_count_wrap     = awg->wrap;
        g_awg_reg->cha_count_step     = awg->step;
        g_awg_reg->cha_start_off      = 0;

        for(i = 0; i < AWG_SIG_LEN; i++) {
            g_awg_cha_mem[i] = data[i];
        }

        g_awg_reg->state_machine_conf = state_machine | mode_mask;

    } else {
        /* Channel B */
        state_machine &= ~0xff0000;

        g_awg_reg->state_machine_conf = state_machine | 0xC00000;
        g_awg_reg->chb_scale_off      = awg->offsgain;
        g_awg_reg->chb_count_wrap     = awg->wrap;
        g_awg_reg->chb_count_step     = awg->step;
        g_awg_reg->chb_start_off      = 0;

        for(i = 0; i < AWG_SIG_LEN; i++) {
            g_awg_chb_mem[i] = data[i];
        }

        g_awg_reg->state_machine_conf = state_machine | (mode_mask<<16);
    }
}


/*----------------------------------------------------------------------------------*/
/** @brief Initialize specified Signal Shape data buffer and apparent AWG settings
 *
 * @param[in]  calib_dc_offs  calibration DC offset value
 * @param[out] data           Signal Shape data buffer
 * @param[out] awg            AWG parameters.
 */
void clear_signal(int calib_dc_offs, int32_t *data, awg_param_t *awg)
{
    int i;

    awg->offsgain = ((calib_dc_offs) << 16) | 0x2000;
    awg->step = 0;
    awg->wrap = 0;

    for(i = 0; i < AWG_SIG_LEN; i++)
        data[i]=0;
}


/*----------------------------------------------------------------------------------*/
/** @brief Initialize Arbitrary Signal Generator module
 *
 * A function is intended to be called within application initialization. It's purpose
 * is to remember a specified pointer to calibration parameters, to initialie
 * Arbitrary Waveform Generator module and to calculate maximal voltage, which can be
 * applied on DAC device on individual channel.
 *
 * @param[in]  calib_params  pointer to calibration parameters
 * @retval     -1 failure, error message is reported on standard error
 * @retval      0 successful initialization
 */

int generate_init(rp_calib_params_t *calib_params)
{
    gen_calib_params = calib_params;

    if(fpga_awg_init() < 0) {
        return -1;
    }

    ch1_max_dac_v = fpga_awg_calc_dac_max_v(gen_calib_params->be_ch1_fs);
    ch2_max_dac_v = fpga_awg_calc_dac_max_v(gen_calib_params->be_ch2_fs);
    return 0;
}


/*----------------------------------------------------------------------------------*/
/** @brief Cleanup Arbitrary Signal Generator module
 *
 * A function is intended to be called on application's termination. The main purpose
 * of this function is to release allocated resources...
 *
 * @retval      0 success, never fails.
 */
int generate_exit(void)
{
    fpga_awg_exit();

    return 0;
}

/*----------------------------------------------------------------------------------*/
/**
 * @brief Update Arbitrary Signal Generator module towards actual settings.
 *
 * A function is intended to be called whenever one of the following settings on each channel
 * is modified
 *    - enable
 *    - signal type
 *    - amplitude
 *    - frequency
 *    - DC offset
 *    - trigger mode
 *
 * @param[in] params  Pointer to overall configuration parameters
 * @retval -1 failure, error message is repoted on standard error device
 * @retval  0 succesful update
 */
int generate_update(rp_app_params_t *params)
{
    awg_param_t ch1_param, ch2_param;
    awg_signal_t ch1_type;
    awg_signal_t ch2_type;
    int ch1_enable = params[GEN_ENABLE_CH1].value;
    int ch2_enable = params[GEN_ENABLE_CH2].value;
    
    //float time_vect[AWG_SIG_LEN], ch1_amp[AWG_SIG_LEN], ch2_amp[AWG_SIG_LEN];
    
    float ch1_arb[AWG_SIG_LEN];
    float ch2_arb[AWG_SIG_LEN];
    
    int wrap;
    
    //int invalid_file=0;
    
    int in_smpl_len1 = 0;
    int in_smpl_len2 = 0;

    ch1_type = (awg_signal_t)params[GEN_SIG_TYPE_CH1].value;
    ch2_type = (awg_signal_t)params[GEN_SIG_TYPE_CH2].value;

    if( (ch1_type == eSignalFile) || (params[GEN_AWG_REFRESH].value == 1) ) {
        if((in_smpl_len1 = read_in_file(1, ch1_arb)) < 0) {
            // Invalid file
            params[GEN_ENABLE_CH1].value = 0;
            params[GEN_SIG_TYPE_CH1].value = eSignalSine;
            ch1_type = params[GEN_SIG_TYPE_CH1].value;
            ch1_enable = params[GEN_ENABLE_CH1].value;
            //invalid_file=1;
        }
    }

    if( (ch2_type == eSignalFile) || (params[GEN_AWG_REFRESH].value == 2) ) {
        if((in_smpl_len2 = read_in_file(2, ch2_arb)) < 0) {
            // Invalid file
            params[GEN_ENABLE_CH2].value = 0;
            params[GEN_SIG_TYPE_CH2].value = eSignalSine;
            ch2_type = params[GEN_SIG_TYPE_CH2].value;
            ch2_enable = params[GEN_ENABLE_CH2].value;
            // invalid_file=1;
        }
    }
    params[GEN_AWG_REFRESH].value = 0;

    /* Waveform from signal gets treated differently then others */
    if(ch1_enable > 0) {
        if(ch1_type < eSignalFile) {
            synthesize_signal(params[GEN_SIG_AMP_CH1].value,
                              params[GEN_SIG_FREQ_CH1].value,
                              gen_calib_params->be_ch1_dc_offs,
                              gen_calib_params->be_ch1_fs,
                              ch1_max_dac_v,
                              params[GEN_SIG_DCOFF_CH1].value,
                              ch1_type, ch1_data, &ch1_param);
            wrap = 0;  // whole buffer used
        } else {
            /* Signal file */
            calculate_data(ch1_arb,  in_smpl_len1,
                           params[GEN_SIG_AMP_CH1].value, params[GEN_SIG_FREQ_CH1].value,
                           gen_calib_params->be_ch1_dc_offs,
                           gen_calib_params->be_ch1_fs,
                           ch1_max_dac_v, params[GEN_SIG_DCOFF_CH1].value,
                           ch1_data, &ch1_param);
            wrap = 0;
            if (in_smpl_len1<AWG_SIG_LEN)
                wrap = 1; // wrapping after (in_smpl_lenx) samples
        }
    } else {
        clear_signal(gen_calib_params->be_ch1_dc_offs, ch1_data, &ch1_param);
    }
    write_data_fpga(0, params[GEN_TRIG_MODE_CH1].value,
                    params[GEN_SINGLE_CH1].value,
                    ch1_data, &ch1_param, wrap);

    /* Waveform from signal gets treated differently then others */
    if(ch2_enable > 0) {
        if(ch2_type < eSignalFile) {
            synthesize_signal(params[GEN_SIG_AMP_CH2].value,
                              params[GEN_SIG_FREQ_CH2].value,
                              gen_calib_params->be_ch2_dc_offs,
                              gen_calib_params->be_ch2_fs,
                              ch2_max_dac_v,
                              params[GEN_SIG_DCOFF_CH2].value,
                              ch2_type, ch2_data, &ch2_param);
            wrap = 0; // whole buffer used
        } else {
            /* Signal file */
            calculate_data(ch2_arb, in_smpl_len2,
                    params[GEN_SIG_AMP_CH2].value, params[GEN_SIG_FREQ_CH2].value,
                    gen_calib_params->be_ch2_dc_offs,
                    gen_calib_params->be_ch2_fs,
                    ch2_max_dac_v, params[GEN_SIG_DCOFF_CH2].value,
                    ch2_data, &ch2_param);
            wrap = 0;
            if (in_smpl_len2<AWG_SIG_LEN)
                wrap = 1; // wrapping after (in_smpl_lenx) samples
        }
    } else {
        clear_signal(gen_calib_params->be_ch2_dc_offs, ch2_data, &ch2_param);
    }
    write_data_fpga(1, params[GEN_TRIG_MODE_CH2].value,
                    params[GEN_SINGLE_CH2].value,
                    ch2_data, &ch2_param, wrap);

    /* Always return singles to 0 */
    params[GEN_SINGLE_CH1].value = 0;
    params[GEN_SINGLE_CH2].value = 0;

    
    //if (invalid_file==1)
    //  return -1;  // Use this return value to notify the GUI user about invalid file. 
    
    return 0;
}


/** 
 * GENERAL DESCRIPTION:
 *
 * This module initializes and provides for other SW modules the access to the 
 * FPGA AWG module. The AWG memory space is divided to three parts:
 *  - registers
 *  - signal buffer 1
 *  - signal buffer 2
 *
 * This module maps physical address of the AWG core to the logical address, 
 * which can be used in the GNU/Linux user-space. To achieve this AWG_BASE_ADDR
 * from CPU memory space is translated automatically to logical address with the
 * function mmap(). After all the initialization is done, other modules can use
 * fpga_awg to control signal outputs (as an example please see generate.c).
 * Before this module is used external SW module must call fpga_awg_init().
 * When this module is no longer needed fpga_awg_exit() should be called.
 *
 * FPGA AWG core state machine operates depending how it is configured. Basic 
 * principle is that after a trigger it reads output signal buffer (for each 
 * channel separately) with configured offset, step (for each clock cycle) and 
 * length (until it stops or wrap readout) - the read-out value is output over 
 * DAC to RF output. For more detailed operation see the FPGA AWG register 
 * description.
 */

/* Internal structures (registers and 2 times signal buffers) */
/** The FPGA register structure (defined in fpga_awg.h) */
awg_reg_t *g_awg_reg     = NULL;
/** The FPGA signal buffer 1.
 * The length of buffer is defined in the FPGA and is AWG_SIG_LEN. Each sample in
 * the buffer is 14-bit signed integer.
  */
uint32_t  *g_awg_cha_mem = NULL;
/** The FPGA signal buffer 2
 * The length of buffer is defined in the FPGA and is AWG_SIG_LEN. Each sample in
 * the buffer is 14-bit signed integer.
  */
uint32_t  *g_awg_chb_mem = NULL;

/** The memory file descriptor used to mmap() the FPGA space */
int g_awg_fd = -1;

/* Constants */
/** DAC frequency (125 Mspmpls (non-decimated)) */
const double c_awg_smpl_freq = 125e6;
const int c_awg_fpga_dac_bits = 14;


/*----------------------------------------------------------------------------*/
/**
 * @brief Internal function used to clean up memory.
 *
 * This function un-maps FPGA register and signal buffers, closes memory file
 * descriptor and cleans all memory allocated by this module.
 *
 * @retval 0 Success
 * @retval -1 Failure, error is printed to standard error output.
 */
int __awg_cleanup_mem(void)
{
    /* If registry structure is NULL we do not need to un-map and clean up */
    if(g_awg_reg) {
        if(munmap(g_awg_reg, AWG_BASE_SIZE) < 0) {
            fprintf(stderr, "munmap() failed: %s\n", strerror(errno));
            return -1;
        }
        g_awg_reg = NULL;
        if(g_awg_cha_mem)
            g_awg_cha_mem = NULL;
        if(g_awg_chb_mem)
            g_awg_chb_mem = NULL;
    }
    if(g_awg_fd >= 0) {
        close(g_awg_fd);
        g_awg_fd = -1;
    }
    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Maps FPGA memory space and prepares register and buffer variables.
 * 
 * This function opens memory device (/dev/mem) and maps physical memory address
 * AWG_BASE_ADDR (of length AWG_BASE_SIZE) to logical addresses. It initializes
 * the pointers g_awg_reg, g_awg_cha_mem, g_awg_chb_mem to point to FPGA AWG.
 * If function fails FPGA variables must not be used.
 *
 * @retval 0  Success
 * @retval -1 Failure, error is printed to standard error output.
 */
int fpga_awg_init(void)
{
    /* Page variables used to calculate correct mapping addresses */
    void *page_ptr;
    long page_addr, page_off, page_size = sysconf(_SC_PAGESIZE);

    /* If module was already initialized, clean all internals */
    if(__awg_cleanup_mem() < 0)
        return -1;

    /* Open /dev/mem to access directly system memory */
    g_awg_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(g_awg_fd < 0) {
        fprintf(stderr, "open(/dev/mem) failed: %s\n", strerror(errno));
        return -1;
    }

    /* Calculate correct page address and offset from AWG_BASE_ADDR and
     * AWG_BASE_SIZE 
     */
    page_addr = AWG_BASE_ADDR & (~(page_size-1));
    page_off  = AWG_BASE_ADDR - page_addr;

    /* Map FPGA memory space to page_ptr. */
    page_ptr = mmap(NULL, AWG_BASE_SIZE, PROT_READ | PROT_WRITE,
                          MAP_SHARED, g_awg_fd, page_addr);
    if((void *)page_ptr == MAP_FAILED) {
        fprintf(stderr, "mmap() failed: %s\n", strerror(errno));
         __awg_cleanup_mem();
        return -1;
    }

    /* Set FPGA AWG module pointers to correct values. */
    g_awg_reg = page_ptr + page_off;
    g_awg_cha_mem = (uint32_t *)g_awg_reg + 
        (AWG_CHA_OFFSET / sizeof(uint32_t));
    g_awg_chb_mem = (uint32_t *)g_awg_reg + 
        (AWG_CHB_OFFSET / sizeof(uint32_t));

    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Cleans up FPGA AWG module internals.
 * 
 * This function closes the memory file descriptor, unmap the FPGA memory space
 * and cleans also all other internal things from FPGA AWG module.
 * @retval 0 Success
 * @retval -1 Failure
 */
int fpga_awg_exit(void)
{
    /* Turn the generator off */
    if (g_awg_reg) {
        g_awg_reg->state_machine_conf = 0xC000C0;
    }

    return __awg_cleanup_mem();
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Calculates maximum DAC voltage in [V].
 * Calculation is done based on the specified Back end full scale voltage and
 * the mapping function defined by FPGA design.
 * @param[in]  be_gain_fs  Back end full scale voltage
 * @retval maximum DAC voltage
*/
float fpga_awg_calc_dac_max_v(uint32_t be_gain_fs)
{
    float max_dac_v;

    max_dac_v = 
        be_gain_fs/(float)((uint64_t)1<<32) * 100;

    return max_dac_v;
}



/* @brief Pointer to FPGA control registers. */
static osc_fpga_reg_mem_t *g_osc_fpga_reg_mem = NULL;

/* @brief Pointer to data buffer where signal on channel A is captured.  */
static uint32_t           *g_osc_fpga_cha_mem = NULL;

/* @brief Pointer to data buffer where signal on channel B is captured.  */
static uint32_t           *g_osc_fpga_chb_mem = NULL;

/* @brief The memory file descriptor used to mmap() the FPGA space. */
static int                 g_osc_fpga_mem_fd = -1;

/* @brief Number of ADC acquisition bits.  */
const int                  c_osc_fpga_adc_bits = 14;

/* @brief Sampling frequency = 125Mspmpls (non-decimated). */
const float                c_osc_fpga_smpl_freq = 125e6;

/* @brief Sampling period (non-decimated) - 8 [ns]. */
const float                c_osc_fpga_smpl_period = (1. / 125e6);


/*----------------------------------------------------------------------------*/
/**
 * @brief Cleanup access to FPGA memory buffers
 *
 * Function optionally cleanups access to FPGA memory buffers, i.e. if access
 * has already been established it unmaps logical memory regions and close apparent
 * file descriptor.
 *
 * @retval  0 Success
 * @retval -1 Failure, error message is printed on standard error device
 */
static int __osc_fpga_cleanup_mem(void)
{
    /* optionally unmap memory regions  */
    if (g_osc_fpga_reg_mem) {
        if (munmap(g_osc_fpga_reg_mem, OSC_FPGA_BASE_SIZE) < 0) {
            fprintf(stderr, "munmap() failed: %s\n", strerror(errno));
            return -1;
        }
        /* ...and update memory pointers */
        g_osc_fpga_reg_mem = NULL;
        g_osc_fpga_cha_mem = NULL;
        g_osc_fpga_chb_mem = NULL;
    }

    /* optionally close file descriptor */
    if(g_osc_fpga_mem_fd >= 0) {
        close(g_osc_fpga_mem_fd);
        g_osc_fpga_mem_fd = -1;
    }

    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Initialize interface to Oscilloscope FPGA module
 *
 * Function first optionally cleanups previously established access to Oscilloscope
 * FPGA module. Afterwards a new connection to the Memory handler is instantiated
 * by opening file descriptor over /dev/mem device. Access to Oscilloscope FPGA module
 * is further provided by mapping memory regions through resulting file descriptor.
 *
 * @retval  0 Success
 * @retval -1 Failure, error message is printed on standard error device
 *
 */
int osc_fpga_init(void)
{
    void *page_ptr;
    long page_addr, page_off, page_size = sysconf(_SC_PAGESIZE);

    /* If maybe needed, cleanup the FD & memory pointer */
    if(__osc_fpga_cleanup_mem() < 0)
        return -1;

    g_osc_fpga_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(g_osc_fpga_mem_fd < 0) {
        fprintf(stderr, "open(/dev/mem) failed: %s\n", strerror(errno));
        return -1;
    }

    page_addr = OSC_FPGA_BASE_ADDR & (~(page_size-1));
    page_off  = OSC_FPGA_BASE_ADDR - page_addr;

    page_ptr = mmap(NULL, OSC_FPGA_BASE_SIZE, PROT_READ | PROT_WRITE,
                          MAP_SHARED, g_osc_fpga_mem_fd, page_addr);
    if((void *)page_ptr == MAP_FAILED) {
        fprintf(stderr, "mmap() failed: %s\n", strerror(errno));
        __osc_fpga_cleanup_mem();
        return -1;
    }
    g_osc_fpga_reg_mem = page_ptr + page_off;
    g_osc_fpga_cha_mem = (uint32_t *)g_osc_fpga_reg_mem + 
        (OSC_FPGA_CHA_OFFSET / sizeof(uint32_t));
    g_osc_fpga_chb_mem = (uint32_t *)g_osc_fpga_reg_mem + 
        (OSC_FPGA_CHB_OFFSET / sizeof(uint32_t));

    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Finalize and release allocated resources while accessing the
 * Oscilloscope FPGA module
 *
 * Function is intended to be  called at the program termination.
 *
 * @retval 0 Success, never fails
 */
int osc_fpga_exit(void)
{
    __osc_fpga_cleanup_mem();

    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Setup Oscilloscope FPGA module based on specified settings
 * FPGA module
 *
 * @param[in] trig_imm           nonzero if acquisition is applied immediately, zero if acquisition is trigger dependent
 * @param[in] trig_source        0 ChannelA, 1 ChannelB, 2 External
 * @param[in] trig_edge          0 Positive, 1 Negative edge
 * @param[in] trig_delay         Number of signal data atoms to be captured after the trigger
 * @param[in] trig_level         Trigger threshold level. expressed in [V]
 * @param[in] time_range         Time range, expressed with value 0..5, defines capture decimation  1,8, 64,1024,8192,65536
 * @param[in] ch1_adc_max_v      Maximal voltage in [V] on ADC inputs for channel A
 * @param[in] ch2_adc_max_v      Maximal voltage in [V] on ADC inputs for channel B
 * @param[in] ch1_calib_dc_off   Calibrated DC offset on channel A, expressed in ADC counts
 * @param[in] ch1_user_dc_off    User defined DC offset on channel A, expressed in [V]
 * @param[in] ch2_calib_dc_off   Calibrated DC offset on channel B, expressed in ADC counts
 * @param[in] ch2_user_dc_off    User defined DC offset on channel B, expressed in [V]
 * @param[in] ch1_probe_att      Channel A Attenuation
 * @param[in] ch2_probe_att      Channel B Attenuation
 * @param[in] enable_avg_at_dec  Apply average calculation during decimation
 *
 * @retval  0 Success
 * @retval -1 Failure, error message is output on standard error device
 */

int osc_fpga_update_params(int trig_imm, int trig_source, int trig_edge, 
                           float trig_delay, float trig_level, int time_range,
                           float ch1_adc_max_v, float ch2_adc_max_v,
                           int ch1_calib_dc_off, float ch1_user_dc_off,
                           int ch2_calib_dc_off, float ch2_user_dc_off,
                           int ch1_probe_att, int ch2_probe_att,
                           int ch1_gain, int ch2_gain,
                           int enable_avg_at_dec)
{
    /* TODO: Locking of memory map */
    int fpga_trig_source = osc_fpga_cnv_trig_source(trig_imm, trig_source, 
                                                    trig_edge);
    int fpga_dec_factor = osc_fpga_cnv_time_range_to_dec(time_range);
    int fpga_delay;
    float after_trigger; /* how much after trigger FPGA should write */
    int fpga_trig_thr;
    
    uint32_t gain_hi_cha_filt_aa=0x7D93;
    uint32_t gain_hi_cha_filt_bb=0x437C7;
    uint32_t gain_hi_cha_filt_pp=0x2666;
    uint32_t gain_hi_cha_filt_kk=0xd9999a;
    
    uint32_t gain_hi_chb_filt_aa=0x7D93;
    uint32_t gain_hi_chb_filt_bb=0x437C7;
    uint32_t gain_hi_chb_filt_pp=0x2666;
    uint32_t gain_hi_chb_filt_kk=0xd9999a;
    
    uint32_t gain_lo_cha_filt_aa=0x4C5F;
    uint32_t gain_lo_cha_filt_bb=0x2F38B;
    uint32_t gain_lo_cha_filt_pp=0x2666;
    uint32_t gain_lo_cha_filt_kk=0xd9999a;
    
    uint32_t gain_lo_chb_filt_aa=0x4C5F;
    uint32_t gain_lo_chb_filt_bb=0x2F38B;
    uint32_t gain_lo_chb_filt_pp=0x2666;
    uint32_t gain_lo_chb_filt_kk=0xd9999a;    
    
    if(trig_source == 0) {
        fpga_trig_thr = osc_fpga_cnv_v_to_cnt(trig_level, ch1_adc_max_v,
                                              ch1_calib_dc_off, ch1_user_dc_off);
        //fprintf(stderr, "Trigger source [V] -> cnts: %f -> %d (max ADC V: %f)\n", trig_level, fpga_trig_thr, ch1_adc_max_v);
    } else {
        fpga_trig_thr = osc_fpga_cnv_v_to_cnt(trig_level, ch2_adc_max_v,
                                              ch2_calib_dc_off, ch2_user_dc_off);
        //fprintf(stderr, "Trigger source [V] -> cnts: %f -> %d (max ADC V: %f)\n", trig_level, fpga_trig_thr, ch2_adc_max_v);
    }

    if((fpga_trig_source < 0) || (fpga_dec_factor < 0)) {
        fprintf(stderr, "osc_fpga_update_params() failed\n");
        return -1;
    }

    /* Pre-trigger - we need to limit after trigger acquisition so we can
     * readout historic (pre-trigger) values */
    /* TODO: Bug in FPGA? We need to put at least 3 less samples to trig_delay */
    after_trigger = 
        ((OSC_FPGA_SIG_LEN-7) * c_osc_fpga_smpl_period * fpga_dec_factor) +
        trig_delay;

    if(after_trigger < 0)
        after_trigger = 0;

    fpga_delay = osc_fpga_cnv_time_to_smpls(after_trigger, fpga_dec_factor);

    /* Trig source is written after ARM */
    /*    g_osc_fpga_reg_mem->trig_source   = fpga_trig_source;*/
    if(trig_source == 0) 
        g_osc_fpga_reg_mem->cha_thr   = fpga_trig_thr;
    else
        g_osc_fpga_reg_mem->chb_thr   = fpga_trig_thr;
    g_osc_fpga_reg_mem->data_dec      = fpga_dec_factor;
    g_osc_fpga_reg_mem->trigger_delay = (uint32_t)fpga_delay;

    g_osc_fpga_reg_mem->other = enable_avg_at_dec;
    
    
    // Updating hysteresys registers
    
    
    g_osc_fpga_reg_mem->cha_hystersis=OSC_HYSTERESIS;
    g_osc_fpga_reg_mem->chb_hystersis=OSC_HYSTERESIS;
    
    
    // Updating equalization filter with default coefficients
    if (ch1_gain==0)
    {
     g_osc_fpga_reg_mem->cha_filt_aa =gain_hi_cha_filt_aa;
     g_osc_fpga_reg_mem->cha_filt_bb =gain_hi_cha_filt_bb;
     g_osc_fpga_reg_mem->cha_filt_pp =gain_hi_cha_filt_pp;
     g_osc_fpga_reg_mem->cha_filt_kk =gain_hi_cha_filt_kk;
    }
    else
    {
     g_osc_fpga_reg_mem->cha_filt_aa =gain_lo_cha_filt_aa;
     g_osc_fpga_reg_mem->cha_filt_bb =gain_lo_cha_filt_bb;
     g_osc_fpga_reg_mem->cha_filt_pp =gain_lo_cha_filt_pp;
     g_osc_fpga_reg_mem->cha_filt_kk =gain_lo_cha_filt_kk;      
    }
    
       if (ch2_gain==0)
    {
     g_osc_fpga_reg_mem->chb_filt_aa =gain_hi_chb_filt_aa;
     g_osc_fpga_reg_mem->chb_filt_bb =gain_hi_chb_filt_bb;
     g_osc_fpga_reg_mem->chb_filt_pp =gain_hi_chb_filt_pp;
     g_osc_fpga_reg_mem->chb_filt_kk =gain_hi_chb_filt_kk;
    }
    else
    {
     g_osc_fpga_reg_mem->chb_filt_aa =gain_lo_chb_filt_aa;
     g_osc_fpga_reg_mem->chb_filt_bb =gain_lo_chb_filt_bb;
     g_osc_fpga_reg_mem->chb_filt_pp =gain_lo_chb_filt_pp;
     g_osc_fpga_reg_mem->chb_filt_kk =gain_lo_chb_filt_kk;      
    }
    
    

    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Reset write state machine
 *
 * @retval 0 Success, never fails
 */
int osc_fpga_reset(void)
{
    g_osc_fpga_reg_mem->conf |= OSC_FPGA_CONF_RST_BIT;
    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Arm the system to capture signal on next trigger
 *
 * @retval 0 Success, never fails
 */
int osc_fpga_arm_trigger(void)
{
    g_osc_fpga_reg_mem->conf |= OSC_FPGA_CONF_ARM_BIT;

    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Define the trigger source
 *
 * @param[in]  trig_source Trigger source
 * @retval 0 Success, never fails
 */
int osc_fpga_set_trigger(uint32_t trig_source)
{
    g_osc_fpga_reg_mem->trig_source = trig_source;
    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Setup the trigger delay
 *
 * @param[in]  trig_delay Trigger delay, expressed in ADC samples
 * @retval 0 Success, never fails
 */
int osc_fpga_set_trigger_delay(uint32_t trig_delay)
{
    g_osc_fpga_reg_mem->trigger_delay = trig_delay;
    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Determine the "Trigger mode" the system is running in
 *
 * @retval 0 The system is not running in the trigger mode
 * @retval 1 The system is running in the trigger mode
 */
int osc_fpga_triggered(void)
{
    return ((g_osc_fpga_reg_mem->trig_source & OSC_FPGA_TRIG_SRC_MASK)==0);
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Retrieve the address of channel A,B memory buffers
 *
 * NOTE: no check is made if argumments are correctly specified.
 *
 * @param[out] cha_signal Pointer to channel A memory buffer
 * @param[out] chb_signal Pointer to channel B memory buffer
 * @retval 0 Success, never fails
 */
int osc_fpga_get_sig_ptr(int **cha_signal, int **chb_signal)
{
    *cha_signal = (int *)g_osc_fpga_cha_mem;
    *chb_signal = (int *)g_osc_fpga_chb_mem;
    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Retrieve Memory buffer Write pointer information
 *
 * @param[out] wr_ptr_curr offset to the currently captured signal atom
 * @param[out] wr_ptr_trig offset to signal atom, captured at detected trigger
 * @retval 0 Success, never fails
 */
int osc_fpga_get_wr_ptr(int *wr_ptr_curr, int *wr_ptr_trig)
{
    if(wr_ptr_curr)
        *wr_ptr_curr = g_osc_fpga_reg_mem->wr_ptr_cur;
    if(wr_ptr_trig)
        *wr_ptr_trig = g_osc_fpga_reg_mem->wr_ptr_trigger;
    return 0;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Convert specified trigger settings into FPGA control value
 *
 * The specified trigger settings are converted into a value, which need to
 * be applied into trigger_source FPGA register in order the specified settings
 * becomes active
 *
 * @param[in] trig_imm     nonzero if acquisition is applied immediately, zero if acquisition is trigger dependent
 * @param[in] trig_source  0 ChannelA, 1 ChannelB, 2 External
 * @param[in] trig_edge    0 Positive, 1 Negative edge
 *
 * @retval -1 failure, indicating invalid arguments are specified
 * @retval >0, <8 Control value to be written into trigger_source FPGA register
 */
int osc_fpga_cnv_trig_source(int trig_imm, int trig_source, int trig_edge)
{
    int fpga_trig_source = 0;

    /* Trigger immediately */    
    if(trig_imm)
        return 1;

    switch(trig_source) {
    case 0: /* ChA*/
        if(trig_edge == 0)
            fpga_trig_source = 2;
        else
            fpga_trig_source = 3;
        break;

    case 1: /* ChB*/
        if(trig_edge == 0)
            fpga_trig_source = 4;
        else
            fpga_trig_source = 5;
        break;

    case 2: /* External */
        if(trig_edge == 0)
            fpga_trig_source = 6;
        else
            fpga_trig_source = 7;
        break;

    default:
        /* Error */
        return -1;
    }

    return fpga_trig_source;
}


/*----------------------------------------------------------------------------*/
/**
 * @brief Converts time_range parameter to decimation factor
 *
 *
 * @param[in] time_range        time range parameter [0..5]
 * @retval    -1                failure, indicating invalid arguments are specified
 * @retval     1,8,64,1K,8K,64K decimation factor
 */
int osc_fpga_cnv_time_range_to_dec(int time_range)
{
    /* Input: 0, 1, 2, 3, 4, 5 translates to:
     * Output: 1x, 8x, 64x, 1kx, 8kx, 65kx */
    switch(time_range) {
    case 0:
        return 1;
        break;
    case 1:
        return 8;
        break;
    case 2:
        return 64;
        break;
    case 3:
        return 1024;
        break;
    case 4:
        return 8*1024;
        break;
    case 5:
        return 64*1024;
        break;
    default:
        return -1;
    }

    return -1;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief Converts time in [s] to ADC samples
 *
 *
 * @param[in] time        time, specified in [s]
 * @param[in] dec_factor  decimation factor
 * @retval    int         number of ADC samples
 */
int osc_fpga_cnv_time_to_smpls(float time, int dec_factor)
{
    /* Calculate sampling period (including decimation) */
    float smpl_p = (c_osc_fpga_smpl_period * dec_factor);
    int fpga_smpls = (int)round(time / smpl_p);

    return fpga_smpls;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief Converts voltage in [V] to ADC counts
 *
 * Function is used for setting up trigger threshold value, which is written into
 * appropriate FPGA register. This value needs to be specified in ADC counts, while
 * user specifies this information in Voltage. The resulting value is based on the
 * specified threshold voltage, maximal ADC voltage, calibrated and user specified
 * DC offsets.
 *
 * @param[in] voltage        Voltage, specified in [V]
 * @param[in] adc_max_v      Maximal ADC voltage, specified in [V]
 * @param[in] calib_dc_off   Calibrated DC offset, specified in ADC counts
 * @param[in] user_dc_off    User specified DC offset, , specified in [V]
 * @retval    int            ADC counts
 */
int osc_fpga_cnv_v_to_cnt(float voltage, float adc_max_v,
                          int calib_dc_off, float user_dc_off)
{
    int adc_cnts = 0;

    /* check and limit the specified voltage arguments towards */
    /* maximal voltages which can be applied on ADC inputs     */
    if(voltage > adc_max_v)
        voltage = adc_max_v;
    else if(voltage < -adc_max_v)
        voltage = -adc_max_v;

    /* adopt the specified voltage with user defined DC offset */
    voltage -= user_dc_off;

    /* map voltage units into FPGA adc counts */
    adc_cnts = (int)round(voltage * (float)((int)(1<<c_osc_fpga_adc_bits)) / 
                          (2*adc_max_v));

    /* clip to the highest value (we are dealing with 14 bits only) */
    if((voltage > 0) && (adc_cnts & (1<<(c_osc_fpga_adc_bits-1))))
        adc_cnts = (1<<(c_osc_fpga_adc_bits-1))-1;
    else
        adc_cnts = adc_cnts & ((1<<(c_osc_fpga_adc_bits))-1);

    /* adopt calculated ADC counts with calibration DC offset */
    adc_cnts -= calib_dc_off;

    return adc_cnts;
}



/*----------------------------------------------------------------------------*/
/**
 * @brief Converts ADC counts to voltage [V]
 *
 * Function is used to publish captured signal data to external world in user units.
 * Calculation is based on maximal voltage, which can be applied on ADC inputs and
 * calibrated and user defined DC offsets.
 *
 * @param[in] cnts           Captured Signal Value, expressed in ADC counts
 * @param[in] adc_max_v      Maximal ADC voltage, specified in [V]
 * @param[in] calib_dc_off   Calibrated DC offset, specified in ADC counts
 * @param[in] user_dc_off    User specified DC offset, specified in [V]
 * @retval    float          Signal Value, expressed in user units [V]
 */
float osc_fpga_cnv_cnt_to_v(int cnts, float adc_max_v,
                            int calib_dc_off, float user_dc_off)
{
    int m;
    float ret_val;

    /* check sign */
    if(cnts & (1<<(c_osc_fpga_adc_bits-1))) {
        /* negative number */
        m = -1 *((cnts ^ ((1<<c_osc_fpga_adc_bits)-1)) + 1);
    } else {
        /* positive number */
        m = cnts;
    }

    /* adopt ADC count with calibrated DC offset */
    m += calib_dc_off;

    /* map ADC counts into user units */
    if(m < (-1 * (1<<(c_osc_fpga_adc_bits-1))))
        m = (-1 * (1<<(c_osc_fpga_adc_bits-1)));
    else if(m > (1<<(c_osc_fpga_adc_bits-1)))
        m =  (1<<(c_osc_fpga_adc_bits-1));

    ret_val =  (m * adc_max_v / 
                (float)(1<<(c_osc_fpga_adc_bits-1)));

    /* and adopt the calculation with user specified DC offset */
    ret_val += user_dc_off;

    return ret_val;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief Calculates maximum [V] in respect to set & calibration parameters
 *
 * Function is used to calculate the maximal voltage which can be applied on ADC inputs.
 * This calculation is based on calibrated Front End Full Scale Gain setting and configured
 * Probe Attenuation.
 *
 * @param[in] fe_gain_fs     Front End Full Scale Gain
 * @param[in] probe_att      Probe attenuation
 * @retval    float          Maximum voltage, expressed in [V]
 */
float osc_fpga_calc_adc_max_v(uint32_t fe_gain_fs, int probe_att)
{
    float max_adc_v;
    int probe_att_fact = (probe_att > 0) ? 10 : 1;

    max_adc_v = 
        fe_gain_fs/(float)((uint64_t)1<<32) * 100 * (probe_att_fact);

    return max_adc_v;
}
