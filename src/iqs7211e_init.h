/*
* This file contains all the necessary settings for the IQS7211E.
* It can be changed from the GUI or edited here.
* File:   IQS7211E_init.h
* Author: Azoteq
*/

#ifndef IQS7211E_INIT_H
#define IQS7211E_INIT_H

/* ALP ATI Compensation */
/* Memory Map Position 0x1F - 0x20 */
#define ALP_COMPENSATION_A_0                     0x51
#define ALP_COMPENSATION_A_1                     0x01
#define ALP_COMPENSATION_B_0                     0x5A
#define ALP_COMPENSATION_B_1                     0x01

/* ATI Settings */
/* Memory Map Position 0x21 - 0x27 */
#define TP_ATI_MULTIPLIERS_DIVIDERS_0            0xE1
#define TP_ATI_MULTIPLIERS_DIVIDERS_1            0x21
#define TP_COMPENSATION_DIV                      0x06
#define TP_REF_DRIFT_LIMIT                       0x32
#define TP_ATI_TARGET_0                          0x2C
#define TP_ATI_TARGET_1                          0x01
#define TP_MIN_COUNT_REATI_0                     0x32
#define TP_MIN_COUNT_REATI_1                     0x00
#define ALP_ATI_MULTIPLIERS_DIVIDERS_0           0x23
#define ALP_ATI_MULTIPLIERS_DIVIDERS_1           0x02
#define ALP_COMPENSATION_DIV                     0x02
#define ALP_LTA_DRIFT_LIMIT                      0x14
#define ALP_ATI_TARGET_0                         0xC8
#define ALP_ATI_TARGET_1                         0x00

/* Report Rates and Timing */
/* Memory Map Position 0x28 - 0x32 */
/*
 * Fast-wake low-power profile:
 * - Keep every report rate unchanged, preserving the existing worst-case
 *   wake latency (LP1 50 ms, LP2 100 ms).
 * - With no touch, enter LP1 after 5 s and defer LP2 until 60 s total. This
 *   avoids expensive Idle scanning early without imposing LP2 latency during
 *   short pauses.
 * - A stationary touch enters Idle-Touch after 5 s. Its 15 ms report rate is
 *   unchanged, so movement response remains as fast as Active mode.
 * - The Idle-Touch timeout is the chip's own stuck-touch guard: once it
 *   expires it reseeds the references, which makes a resting finger disappear.
 *   AZD123 5.2.2 says to set it to 0 when the host takes that job. The driver
 *   therefore writes HOST_VERIFY whenever host touch verification is enabled,
 *   in either report mode, and keeps the 60 s FALLBACK when host verification
 *   is disabled. Leaving both armed lets the chip reseed underneath a touch the
 *   host still believes in; disabling both leaves a stuck touch unbounded.
 */
#define ACTIVE_MODE_REPORT_RATE_0                0x0F
#define ACTIVE_MODE_REPORT_RATE_1                0x00
#define IDLE_TOUCH_MODE_REPORT_RATE_0            0x0F
#define IDLE_TOUCH_MODE_REPORT_RATE_1            0x00
#define IDLE_MODE_REPORT_RATE_0                  0x1E
#define IDLE_MODE_REPORT_RATE_1                  0x00
#define LP1_MODE_REPORT_RATE_0                   0x32
#define LP1_MODE_REPORT_RATE_1                   0x00
#define LP2_MODE_REPORT_RATE_0                   0x64
#define LP2_MODE_REPORT_RATE_1                   0x00
#define ACTIVE_MODE_TIMEOUT_0                    0x05
#define ACTIVE_MODE_TIMEOUT_1                    0x00
#define IDLE_TOUCH_MODE_TIMEOUT_HOST_VERIFY_0    0x00
#define IDLE_TOUCH_MODE_TIMEOUT_HOST_VERIFY_1    0x00
#define IDLE_TOUCH_MODE_TIMEOUT_FALLBACK_0       0x3C
#define IDLE_TOUCH_MODE_TIMEOUT_FALLBACK_1       0x00
#define IDLE_MODE_TIMEOUT_0                      0x05
#define IDLE_MODE_TIMEOUT_1                      0x00
#define LP1_MODE_TIMEOUT_0                       0x37
#define LP1_MODE_TIMEOUT_1                       0x00
#define REATI_RETRY_TIME                         0x05
#define REF_UPDATE_TIME                          0x08
#define I2C_TIMEOUT_0                            0x64
#define I2C_TIMEOUT_1                            0x00

/* System Settings */
/* Memory Map Position 0x33 - 0x35 */
#define SYSTEM_CONTROL_0                         0x00
#define SYSTEM_CONTROL_1                         0x00
#define CONFIG_SETTINGS0                         0x2C
#define CONFIG_SETTINGS1                         0x06
#define OTHER_SETTINGS_0                         0x10
#define OTHER_SETTINGS_1                         0x00

/* ALP Settings */
/* Memory Map Position 0x36 - 0x37 */
#define ALP_SETUP_0                              0x77
#define ALP_SETUP_1                              0x03
#define ALP_TX_ENABLE_0                          0x08
#define ALP_TX_ENABLE_1                          0x05

/* Thresholds and Debounce Settings */
/* Memory Map Position 0x38 - 0x3A */
/*
 * Retuned from the stock SET 0x16 (22) / CLEAR 0x0E (14).
 *
 * Threshold = Reference x (1 + Multiplier/128) (datasheet 5.5.1), so one count
 * is +0.78% of the channel reference and a larger value is LESS sensitive.
 * The hysteresis width is SET - CLEAR.
 *
 * A finger approaching the pad can raise a channel as much as a very light
 * touch does, so a false touch cannot reliably be rejected at the moment it is
 * detected. What separates the two is what happens next: sustained contact
 * stays well clear of the level a hover settles at. Raising CLEAR into that gap
 * lets a false detection release on its own rather than latching, which the
 * stock value - far below SET - could not do.
 *
 * SET therefore cannot also satisfy AZD123 4.3.1, which puts it below the
 * weakest of four channels under a light press placed between them: on a panel
 * where hover reaches that level, the value the procedure asks for falls below
 * CLEAR and the two requirements exclude each other. Whichever way that is
 * resolved is a property of the panel, so both thresholds need re-deriving for
 * a different one rather than carrying over. Raising SET also costs contact
 * fragmentation during taps.
 */
#define TRACKPAD_TOUCH_SET_THRESHOLD             0x20
#define TRACKPAD_TOUCH_CLEAR_THRESHOLD           0x12
#define ALP_THRESHOLD_0                          0x08
#define ALP_THRESHOLD_1                          0x00
#define ALP_SET_DEBOUNCE                         0x04
#define ALP_CLEAR_DEBOUNCE                       0x04

/* Button and ALP count and LTA betas */
/* Memory Map Position 0x3B - 0x3C */
#define ALP_COUNT_BETA_LP1                       0xDC
#define ALP_LTA_BETA_LP1                         0x08
#define ALP_COUNT_BETA_LP2                       0xF0
#define ALP_LTA_BETA_LP2                         0x10

/* Hardware Settings */
/* Memory Map Position 0x3D - 0x40 */
#define TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH   0x02
#define TP_CONVERSION_FREQUENCY_FRACTION_VALUE   0x1A
#define ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH  0x02
#define ALP_CONVERSION_FREQUENCY_FRACTION_VALUE  0x1A
#define TRACKPAD_HARDWARE_SETTINGS_0             0x03
#define TRACKPAD_HARDWARE_SETTINGS_1             0x9C
/* LP1 auto-prox: 8 cycles; LP2 auto-prox: 32 cycles; init delay unchanged. */
#define ALP_HARDWARE_SETTINGS_0                  0x67
#define ALP_HARDWARE_SETTINGS_1                  0x9C

/* Trackpad Settings */
/* Memory Map Position 0x41 - 0x49 */
#define TRACKPAD_SETTINGS_0_0                    0x28
#define TRACKPAD_SETTINGS_0_1                    0x05
#define TRACKPAD_SETTINGS_1_0                    0x05
#define TRACKPAD_SETTINGS_1_1                    0x02
#define X_RESOLUTION_0                           0x00 // 1024 (Max Coordinate N)
#define X_RESOLUTION_1                           0x04
#define Y_RESOLUTION_0                           0x00 // 1024 (Max Coordinate N)
#define Y_RESOLUTION_1                           0x04
/*
 * XY filter values restored to Azoteq's reference set (6 / 124 / 7 / 128).
 *
 * The previous values (bottom 2 / top 16 / bottom beta 1 / static beta 0) left
 * the IIR filter doing nothing useful. The damping factor is beta/256
 * (datasheet 7.8.2) and the dynamic filter stops filtering above "top speed" -
 * but a finger landing moves far faster than a top speed of 16, so exactly the
 * transient the filter exists to smooth passed through with only the 2-sample
 * MAV average applied.
 *
 * These four are chip-generation constants rather than panel tuning, and were
 * changed by a commit that otherwise carried legitimate ATI and CS-cap work.
 */
#define XY_DYNAMIC_FILTER_BOTTOM_SPEED_0         0x06
#define XY_DYNAMIC_FILTER_BOTTOM_SPEED_1         0x00
#define XY_DYNAMIC_FILTER_TOP_SPEED_0            0x7C
#define XY_DYNAMIC_FILTER_TOP_SPEED_1            0x00
#define XY_DYNAMIC_FILTER_BOTTOM_BETA            0x07
#define XY_DYNAMIC_FILTER_STATIC_FILTER_BETA     0x80
/*
 * Movement below this, in output-resolution pixels, counts as a stationary
 * touch. It gates the TP Movement flag and the drop to Idle-Touch mode
 * (datasheet 7.5) without altering the reported coordinates. Keeping it tight
 * lets the chip enter Idle-Touch without delaying deliberate slow motion. The
 * host coordinate filter does not use this flag: captured traces showed that
 * gating on it added latency without materially reducing visible jitter.
 */
#define STATIONARY_TOUCH_MOV_THRESHOLD           0x02
#define FINGER_SPLIT_FACTOR                      0x03
/*
 * Trim removes the dead margin at each edge and rescales what is left back over
 * the full resolution, so the extreme coordinates (0 and X/Y Resolution) become
 * reachable (datasheet 7.9). Both edges of an axis are trimmed by the same
 * amount, so the value has to cover the larger of the two dead margins.
 *
 * The stock 0x14 (20) was short of that on this panel: neither 0 nor the
 * maximum could be reached on either axis. 0x2C (44) brings both within a
 * couple of counts, which also keeps the rotate-cw transform - which computes
 * RESOLUTION - raw, and so assumes a full 0..resolution range - symmetric.
 * The cost is a small saturated band at the far edge, where the coordinate is
 * already clamped to the maximum.
 */
#define X_TRIM_VALUE                             0x2C
#define Y_TRIM_VALUE                             0x2C

/* Settings Version Numbers */
/* Memory Map Position 0x4A - 0x4A */
#define MINOR_VERSION                            0x01
#define MAJOR_VERSION                            0x00

/* Gesture Settings */
/* Memory Map Position 0x4B - 0x55 */
#define GESTURE_ENABLE_0                         0x07
#define GESTURE_ENABLE_1                         0x00
#define TAP_TOUCH_TIME_0                         0x96
#define TAP_TOUCH_TIME_1                         0x00
#define TAP_WAIT_TIME_0                          0x96
#define TAP_WAIT_TIME_1                          0x00
#define TAP_DISTANCE_0                           0x32
#define TAP_DISTANCE_1                           0x00
#define HOLD_TIME_0                              0x2C
#define HOLD_TIME_1                              0x01
#define SWIPE_TIME_0                             0x96
#define SWIPE_TIME_1                             0x00
#define SWIPE_X_DISTANCE_0                       0xC8
#define SWIPE_X_DISTANCE_1                       0x00
#define SWIPE_Y_DISTANCE_0                       0xC8
#define SWIPE_Y_DISTANCE_1                       0x00
#define SWIPE_X_CONS_DIST_0                      0x64
#define SWIPE_X_CONS_DIST_1                      0x00
#define SWIPE_Y_CONS_DIST_0                      0x64
#define SWIPE_Y_CONS_DIST_1                      0x00
#define SWIPE_ANGLE                              0x17
#define PALM_THRESHOLD                           0x1E

/* RxTx Mapping */
/* Memory Map Position 0x56 - 0x5C */
#define RX_TX_MAP_0                              0x00
#define RX_TX_MAP_1                              0x01
#define RX_TX_MAP_2                              0x02
#define RX_TX_MAP_3                              0x04
#define RX_TX_MAP_4                              0x05
#define RX_TX_MAP_5                              0x08
#define RX_TX_MAP_6                              0x09
#define RX_TX_MAP_7                              0x0A
#define RX_TX_MAP_8                              0x0B
#define RX_TX_MAP_9                              0x0C
#define RX_TX_MAP_10                             0x0A
#define RX_TX_MAP_11                             0x0B
#define RX_TX_MAP_12                             0x0C
#define RX_TX_MAP_FILLER                         0x00

/* Allocation of channels into cycles 0-9 */
/* Memory Map Position 0x5D - 0x6B */
#define PLACEHOLDER_0                            0x05
#define CH_1_CYCLE_0                             0x00
#define CH_2_CYCLE_0                             0x03
#define PLACEHOLDER_1                            0x05
#define CH_1_CYCLE_1                             0x01
#define CH_2_CYCLE_1                             0x04
#define PLACEHOLDER_2                            0x05
#define CH_1_CYCLE_2                             0x02
#define CH_2_CYCLE_2                             0xFF
#define PLACEHOLDER_3                            0x05
#define CH_1_CYCLE_3                             0x05
#define CH_2_CYCLE_3                             0x08
#define PLACEHOLDER_4                            0x05
#define CH_1_CYCLE_4                             0x06
#define CH_2_CYCLE_4                             0x09
#define PLACEHOLDER_5                            0x05
#define CH_1_CYCLE_5                             0x07
#define CH_2_CYCLE_5                             0xFF
#define PLACEHOLDER_6                            0x05
#define CH_1_CYCLE_6                             0x0A
#define CH_2_CYCLE_6                             0x0D
#define PLACEHOLDER_7                            0x05
#define CH_1_CYCLE_7                             0x0B
#define CH_2_CYCLE_7                             0x0E
#define PLACEHOLDER_8                            0x05
#define CH_1_CYCLE_8                             0x0C
#define CH_2_CYCLE_8                             0xFF
#define PLACEHOLDER_9                            0x05
#define CH_1_CYCLE_9                             0x0F
#define CH_2_CYCLE_9                             0x12

/* Allocation of channels into cycles 10-19 */
/* Memory Map Position 0x6C - 0x7A */
#define PLACEHOLDER_10                           0x05
#define CH_1_CYCLE_10                            0x10
#define CH_2_CYCLE_10                            0x13
#define PLACEHOLDER_11                           0x05
#define CH_1_CYCLE_11                            0x11
#define CH_2_CYCLE_11                            0xFF
#define PLACEHOLDER_12                           0x05
#define CH_1_CYCLE_12                            0x14
#define CH_2_CYCLE_12                            0x17
#define PLACEHOLDER_13                           0x05
#define CH_1_CYCLE_13                            0x15
#define CH_2_CYCLE_13                            0x18
#define PLACEHOLDER_14                           0x05
#define CH_1_CYCLE_14                            0x16
#define CH_2_CYCLE_14                            0xFF
#define PLACEHOLDER_15                           0x05
#define CH_1_CYCLE_15                            0x19
#define CH_2_CYCLE_15                            0xFF
#define PLACEHOLDER_16                           0x05
#define CH_1_CYCLE_16                            0xFF
#define CH_2_CYCLE_16                            0xFF
#define PLACEHOLDER_17                           0x05
#define CH_1_CYCLE_17                            0xFF
#define CH_2_CYCLE_17                            0xFF
#define PLACEHOLDER_18                           0x05
#define CH_1_CYCLE_18                            0xFF
#define CH_2_CYCLE_18                            0xFF
#define PLACEHOLDER_19                           0x05
#define CH_1_CYCLE_19                            0xFF
#define CH_2_CYCLE_19                            0xFF

/* Allocation of channels into cycles 20 */
/* Memory Map Position 0x7B - 0x7C */
#define PLACEHOLDER_20                           0x05
#define CH_1_CYCLE_20                            0xFF
#define CH_2_CYCLE_20                            0xFF

#endif	/* IQS7211E_INIT_H */
