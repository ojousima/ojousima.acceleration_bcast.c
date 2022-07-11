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

static inline void LOG (const char * const msg)
{
    ri_log (RI_LOG_LEVEL_INFO, msg);
}

static ri_timer_id_t heart_timer; //!< Timer for updating data.

static uint64_t last_heartbeat_timestamp_ms; //!< Timestamp for heartbeat refresh.

static app_dataformat_t m_dataformat_state; //!< State of heartbeat.

static app_dataformats_t m_dataformats_enabled =
{
    .DF_3  = APP_DF_3_ENABLED,
    .DF_5  = APP_DF_5_ENABLED,
    .DF_8  = APP_DF_8_ENABLED,
    .DF_FA = APP_DF_FA_ENABLED,
    .DF_AC = APP_DF_AC_ENABLED
}; //!< Flags of enabled formats

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
    bool heartbeat_ok = false;

    // Turn accelerometer high-performance mode on
    LOG("Acceleration collection start\r\n");
    err_code |= app_sensor_fifo_collection_start();

    last_heartbeat_timestamp_ms = ri_rtc_millis();

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

rd_status_t app_heartbeat_acceleration_process(float data[][3])
{
   char msg[128] = {0};
   for(size_t ii = 0; ii < 1024; ii++)
   {
     snprintf(msg, sizeof(msg), "X: %.3f Y: %.3f Z: %.3f \r\n", data[ii][0], data[ii][1], data[ii][2]);
     LOG(msg);
     ri_delay_ms(1U);
   }
    return RD_SUCCESS;
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
