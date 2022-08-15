/**
 * @file app_heartbeat.c
 * @author Otso Jousimaa <otso@ojousima.net>
 * @date 2022-07-11
 * @copyright Ruuvi Innovations Ltd, License BSD-3-Clause.
 */

#include "app_config.h"
#include "app_comms.h"
#include "app_dataformats.h"
#include "app_heartbeat.h"
#include "app_led.h"
#include "app_log.h"
#include "app_sensor.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "ojousima_endpoint_ac.h"
#include "ojousima_endpoint_af.h"
#include "ruuvi_driver_error.h"
#include "ruuvi_driver_sensor.h"
#include "ruuvi_endpoint_5.h"
#include "ruuvi_interface_communication.h"
#include "ruuvi_interface_communication_radio.h"
#include "ruuvi_interface_log.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"
#include "ruuvi_interface_watchdog.h"
#include "ruuvi_interface_yield.h"
#include "ruuvi_library_rms.h"
#include "ruuvi_library_peak2peak.h"
#include "ruuvi_task_adc.h"
#include "ruuvi_task_advertisement.h"
#include "ruuvi_task_gatt.h"
#include "ruuvi_task_nfc.h"

#include <stdio.h>

#define U8_MASK (0xFFU)
#define APP_DF_3_ENABLED RE_3_ENABLED
#define APP_DF_5_ENABLED RE_5_ENABLED
#define APP_DF_8_ENABLED RE_8_ENABLED
#define APP_DF_FA_ENABLED RE_FA_ENABLED
#define APP_DF_AC_ENABLED RE_AC_ENABLED
// FFT output is half of input in size
#define FFT_SIZE ((APP_SENSOR_BUFFER_DEPTH) / 2U)

static inline void LOG (const char * const msg)
{
    ri_log (RI_LOG_LEVEL_INFO, msg);
}

#if RE_AF_ENABLED
static float fft_output[APP_SENSOR_BUFFER_DEPTH];
#endif

static ri_timer_id_t heart_timer; //!< Timer for updating data.

static uint64_t last_heartbeat_timestamp_ms; //!< Timestamp for heartbeat refresh.

#if RE_5_ENABLED
static app_dataformat_t m_dataformat_state; //!< State of heartbeat.
#endif

static rd_status_t send_adv (ri_comm_message_t * const p_msg)
{
    rd_status_t err_code = RD_SUCCESS;
    const uint8_t repeat_count = app_comms_bleadv_send_count_get();

    if (APP_COMM_ADV_DISABLE != repeat_count)
    {
        if (APP_COMM_ADV_REPEAT_FOREVER == repeat_count)
        {
            p_msg->repeat_count = RI_COMM_MSG_REPEAT_FOREVER;
        }
        else
        {
            p_msg->repeat_count = repeat_count;
        }

        err_code |= rt_adv_send_data (p_msg);
    }
    else
    {
        rt_adv_stop();
    }

    return err_code;
}

#if RE_5_ENABLED
static rd_status_t df_5_process()
{
    rd_status_t err_code = RD_SUCCESS;
    rd_status_t op_status = RD_SUCCESS;
    ri_comm_message_t msg = {0};
    rd_sensor_data_t data = { 0 };
    size_t buffer_len = RI_COMM_MESSAGE_MAX_LENGTH;
    data.fields = app_sensor_available_data();
    float data_values[rd_sensor_data_fieldcount (&data)];
    data.data = data_values;
    app_sensor_get (&data);
    // Sensor read takes a long while, indicate activity once data is read.
    app_led_activity_signal (true);
    m_dataformat_state = DF_5;
    app_dataformat_encode (msg.data, &buffer_len, &data, m_dataformat_state);
    msg.data_length = (uint8_t) buffer_len;
    op_status = send_adv (&msg);
    err_code |= op_status;
    // Advertising should always be successful
    RD_ERROR_CHECK (err_code, ~RD_ERROR_FATAL);
    // Cut endpoint data to fit into GATT msg.
    // Actual maximum would be 20, but MAC address bytes aren't meaningful.
    msg.data_length = 18;
    // Gatt Link layer takes care of delivery.
    msg.repeat_count = 1;
    op_status = rt_gatt_send_asynchronous (&msg);
    err_code |= op_status;
    // Restore original message length for NFC
    msg.data_length = (uint8_t) buffer_len;
    op_status = rt_nfc_send (&msg);
    err_code |= op_status;
    // Turn LED off before starting lengthy flash operations
    app_led_activity_signal (false);
    err_code = app_log_process (&data);
    return err_code;
}
#endif

/**
 * @brief When timer triggers, schedule reading sensors and sending data.
 *
 * @param[in] p_context Always NULL.
 */
#ifndef CEEDLING
static
#endif
void heartbeat (void * p_event, uint16_t event_size)
{
    rd_status_t err_code = RD_SUCCESS;
#if RE_5_ENABLED
    err_code = df_5_process();

    if (RD_SUCCESS == err_code)
    {
        ri_watchdog_feed();
        last_heartbeat_timestamp_ms = ri_rtc_millis();
    }

    RD_ERROR_CHECK (err_code, ~RD_ERROR_FATAL);
#endif
    // Turn accelerometer high-performance mode on
    LOG ("Acceleration collection start\r\n");
    err_code |= app_sensor_fifo_collection_start();
    RD_ERROR_CHECK (err_code, ~RD_ERROR_FATAL);
}

/**
 * @brief When timer triggers, schedule reading sensors and sending data.
 *
 * @param[in] p_context Always NULL.
 */
#ifndef CEEDLING
static
#endif
void schedule_heartbeat_isr (void * const p_context)
{
    ri_scheduler_event_put (NULL, 0U, &heartbeat);
}

rd_status_t app_heartbeat_init (void)
{
    rd_status_t err_code = RD_SUCCESS;

    if ( (!ri_timer_is_init()) || (!ri_scheduler_is_init()))
    {
        err_code |= RD_ERROR_INVALID_STATE;
    }
    else
    {
        err_code |= ri_timer_create (&heart_timer, RI_TIMER_MODE_REPEATED,
                                     &schedule_heartbeat_isr);

        if (RD_SUCCESS == err_code)
        {
            err_code |= ri_timer_start (heart_timer, APP_HEARTBEAT_INTERVAL_MS, NULL);
        }
    }

    return err_code;
}

#if RE_AF_ENABLED
static rd_status_t  acceleration_fft_process (float * const  data, const uint8_t type)
{
    // FFT example
    ri_comm_message_t msg = {0};
    rd_status_t err_code = RD_SUCCESS;
    arm_rfft_fast_instance_f32 S = {0};
    static uint8_t sequence = 0;
    // 0x55 for a sequence that's easy to spot in memory readout
    memset (fft_output, 0x55, sizeof (fft_output));
    arm_rfft_fast_init_f32 (&S, APP_SENSOR_BUFFER_DEPTH);
    float td_energy = 0;
    float fft_energy = 0;
    app_endpoint_af_data_t fft_data;

    // Calculate time-domain energy for sanity check.
    for (size_t ii = 0; ii < APP_SENSOR_BUFFER_DEPTH; ii++)
    {
        td_energy += (data[ii] * data[ii]);
    }

    // NOTE: Changes input buffer.
    arm_rfft_fast_f32 (&S, data, fft_output, 0);
    // Print bins
    // for(size_t ii = 0; ii < FFT_SIZE; ii++)
    // {
    // snprintf(log_msg, sizeof(log_msg), "Bin: %d Magnitude: %.3f \r\n", ii, fft_output[ii]);
    // LOG(log_msg);
    // Delay to let logs process
    // ri_delay_ms(1U);
    // }
    // Normalize power and sum to  bins for broadcast
    const size_t bin_accumulator_size = FFT_SIZE / APP_ENDPOINT_AF_NUM_BUCKETS;
    float accumulator_bins [APP_ENDPOINT_AF_NUM_BUCKETS] = {0};

    for (size_t ii = 0; ii < APP_ENDPOINT_AF_NUM_BUCKETS; ii++)
    {
        for (size_t jj = 0; jj < bin_accumulator_size; jj++)
        {
            float bin_power =  fft_output[ (ii * bin_accumulator_size) + jj] *  fft_output[ (ii *
                               bin_accumulator_size) + jj];
            accumulator_bins[ii] += bin_power;
            // Calculate FFT energy for sanity check
            fft_energy += bin_power;
        }
    }

    // Find max for normalization
    float max_power_value = 0;

    for (size_t ii = 0; ii < APP_ENDPOINT_AF_NUM_BUCKETS; ii++)
    {
        max_power_value = (accumulator_bins[ii] > max_power_value) ? accumulator_bins[ii] :
                          max_power_value;
        // Print accumulator bins
        // snprintf(log_msg, sizeof(log_msg), "Bin: %d Magnitude: %.3f \r\n", ii, accumulator_bins[ii]);
        // LOG(log_msg);
        // Delay to let logs process
        // ri_delay_ms(1U);
    }

    // Normalize FFT
    for (size_t ii = 0; ii < APP_ENDPOINT_AF_NUM_BUCKETS; ii++)
    {
        accumulator_bins[ii] /= max_power_value;
        fft_data.buckets[ii] = accumulator_bins[ii];
    }

    // Print time domain + frequency domain energy as a sanity check
    // https://en.wikipedia.org/wiki/Parseval's_theorem, Discrete case.
    fft_energy /= FFT_SIZE;
    // snprintf(log_msg, sizeof(log_msg), "TD Energy: %.3f FD energy: %.3f \r\n", td_energy, fft_energy);
    // LOG(log_msg);
    // Delay to let logs process
    // ri_delay_ms(1U);
    // Calculate scaling to 8-bit uints
    fft_data.scale = APP_ENDPOINT_AF_RESOLUTION_LEVELS / max_power_value;
    fft_data.type = type;
    // FFT buckets are up to sampling frequency / 2, Nyquist Theorem
    fft_data.frequency = (APP_ACC_HIPERF_SAMPLERATE_HZ / 2);
    fft_data.sequence = sequence;
    sequence++;
    app_endpoint_af_encode_v0 (msg.data, &fft_data);;
    msg.data_length = APP_ENDPOINT_AF_DATA_LENGTH;
    err_code = send_adv (&msg);
    // Print outgoing message
    // ri_log_hex (RI_LOG_LEVEL_INFO, msg.data, msg.data_length);
    // LOG ("\r\n");
    return err_code;
}
#endif

rd_status_t app_heartbeat_acceleration_process (float * const  data_x,
        float * const  data_y, float * const  data_z)
{
    // Sensor read takes a long while, indicate activity once data is read.
    app_led_activity_signal (true);
    ri_comm_message_t msg = {0};
    rd_status_t err_code = RD_SUCCESS;
    rd_sensor_data_t env_data = { 0 };
    static uint16_t sequence = 0;
    // Advertising should always be successful
    RD_ERROR_CHECK (err_code, ~RD_ERROR_FATAL);
    app_endpoint_ac_data_t acceleration_data = {0};

    // Sensor is back in low-power mode, process the acceleration data.
    for (size_t ii = 0; ii < APP_SENSOR_BUFFER_DEPTH; ii++)
    {
        // Print samples
        // snprintf(log_msg, sizeof(log_msg), "X: %.3f Y: %.3f Z: %.3f \r\n", data_x[ii], data_y[ii], data_z[ii]);
        // LOG(log_msg);
        // Delay to let logs process
        // ri_delay_ms(1U);
    }

    acceleration_data.rms[APP_ENDPOINT_AC_X_INDEX] = rl_rms (data_x, APP_SENSOR_BUFFER_DEPTH);
    acceleration_data.p2p[APP_ENDPOINT_AC_X_INDEX] = rl_peak2peak (data_x,
            APP_SENSOR_BUFFER_DEPTH);
    acceleration_data.rms[APP_ENDPOINT_AC_Y_INDEX] = rl_rms (data_y, APP_SENSOR_BUFFER_DEPTH);
    acceleration_data.p2p[APP_ENDPOINT_AC_Y_INDEX] = rl_peak2peak (data_y,
            APP_SENSOR_BUFFER_DEPTH);
    acceleration_data.rms[APP_ENDPOINT_AC_Z_INDEX] = rl_rms (data_z, APP_SENSOR_BUFFER_DEPTH);
    acceleration_data.p2p[APP_ENDPOINT_AC_Z_INDEX] = rl_peak2peak (data_z,
            APP_SENSOR_BUFFER_DEPTH);
    env_data.fields = app_sensor_available_data();
    float data_values[rd_sensor_data_fieldcount (&env_data)];
    env_data.data = data_values;
    app_sensor_get (&env_data);
    acceleration_data.temperature = rd_sensor_data_parse (&env_data, RD_SENSOR_TEMP_FIELD);
    err_code |= rt_adc_vdd_get (&acceleration_data.voltage);
    acceleration_data.sequence = sequence;
    sequence++;
    app_endpoint_ac_encode_v2 (msg.data, &acceleration_data);;
    msg.data_length = APP_ENDPOINT_AC_DATA_LENGTH;
    err_code = send_adv (&msg);
    // Turn LED off before starting lengthy flash operations
    app_led_activity_signal (false);
#if RE_AF_ENABLED
    // NOTE: FFT Processing alters source data.
    err_code |= acceleration_fft_process (data_x, APP_ENDPOINT_AF_X_TYPE);
    err_code |= acceleration_fft_process (data_y, APP_ENDPOINT_AF_Y_TYPE);
    err_code |= acceleration_fft_process (data_z, APP_ENDPOINT_AF_Z_TYPE);
#endif
    return err_code;
}

rd_status_t app_heartbeat_start (void)
{
    rd_status_t err_code = RD_SUCCESS;

    if (NULL == heart_timer)
    {
        err_code |= RD_ERROR_INVALID_STATE;
    }
    else
    {
        heartbeat (NULL, 0);
        err_code |= ri_timer_start (heart_timer, APP_HEARTBEAT_INTERVAL_MS, NULL);
    }

    return err_code;
}

rd_status_t app_heartbeat_stop (void)
{
    rd_status_t err_code = RD_SUCCESS;

    if (NULL == heart_timer)
    {
        err_code |= RD_ERROR_INVALID_STATE;
    }
    else
    {
        err_code |= ri_timer_stop (heart_timer);
    }

    return err_code;
}

bool app_heartbeat_overdue (void)
{
    return ri_rtc_millis() > (last_heartbeat_timestamp_ms +
                              APP_HEARTBEAT_OVERDUE_INTERVAL_MS);
}

#ifdef CEEDLING
// Give CEEDLING a handle to state of module.
ri_timer_id_t * get_heart_timer (void)
{
    return &heart_timer;
}
#endif
