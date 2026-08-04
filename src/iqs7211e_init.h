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
#define IDLE_TOUCH_MODE_TIMEOUT_0                0x3C
#define IDLE_TOUCH_MODE_TIMEOUT_1                0x00
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
 * Tuned to stop the pointer drifting while a finger merely APPROACHES the pad.
 * Stock was SET 0x16 (22) / CLEAR 0x0E (14).
 *
 * Threshold = Reference x (1 + Multiplier/128)  (datasheet 5.5.1)
 * so one count is +0.78% of the channel reference. A larger value is LESS
 * sensitive. Hysteresis width = SET - CLEAR.
 *
 * What the investigation established, in order of usefulness:
 *
 * 1. Hover and a light touch are NOT separable at touch-down. Measured over
 *    ~50 touch episodes, 7 of 35 deliberate light taps peaked at the same
 *    channel count (area 1-2) as a hover. No instantaneous test - strength,
 *    area, or threshold - can tell them apart, because during an approach the
 *    sensor genuinely cannot know whether the finger will land.
 *
 * 2. They DO diverge once contact is established. Measured with a slow steady
 *    drag (constant force, so unlike tapping this is reproducible):
 *        sustained light contact : never fell below str 315
 *        hover                   : lingered around str 205-217
 *    CLEAR belongs in that gap. This is the real fix: rather than trying to
 *    reject a false touch at detection time, let it self-clear immediately
 *    afterwards. With the stock CLEAR sitting far below SET, a false hover
 *    detection never released - that is what produced an observed 182-frame
 *    (2.7s) 2015px phantom drag.
 *
 *        CLEAR 0x0E (14): drag episodes [87, 147, 169, 183] - nothing releases
 *        CLEAR 0x14 (20): [5,7,10,13,14,16,18,27,36,37,53,148,207,371]
 *                         marginal contact clears in 0.1-0.3s (the point) and
 *                         solid contact holds even longer, but real drags began
 *                         to break slightly -> too eager.
 *        CLEAR 0x12 (18): chosen.
 *
 * 3. SET was swept 22..34 and judged by feel. Note the per-step measurements
 *    are NOT trustworthy: they track how hard the pad happened to be tapped far
 *    more than they track the threshold, so operator variation swamps a single
 *    count. SET was swept up to 0x22 (34) and settled back at 0x20 (32) by
 *    feel, which is also exactly what Azoteq's own example code ships.
 *
 * Re-ATI was ruled out as a cause: zero re-ATI events across every capture.
 *
 * Note the surveyed drivers split into two lineages: Azoteq's own example plus
 * Flipper (SET 32 / 75, ATI coarse multiplier 15), and a "SET 20" group of four
 * ZMK/QMK drivers that all inherit one identical config. This panel's ATI was
 * tuned by the official procedure (AZD123 4.2.1: coarse div 1, mult 15, then
 * narrow the fine divider) and landed in the first lineage.
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
 * XY filter values restored to Azoteq's reference set. The previous values
 * (bottom 2 / top 16 / bottom beta 1 / static beta 0) left the IIR filter doing
 * nothing useful: damping factor = beta/256 (datasheet 7.8.2) and the dynamic
 * filter stops filtering above "top speed", but a finger landing was measured
 * moving 36-47 px/cycle - far above 16 - so the transient passed through with
 * only the 2-sample MAV average applied.
 *
 * 6 / 124 / 7 / 128 is what Azoteq's own example code uses, and every other
 * IQS7211E driver surveyed ships it untouched - including Flipper's, which is
 * the only other independent lineage. Nobody deviates from these four values.
 *
 * They were lost in bc9e6df (2026-01-23), a commit that also carried the
 * legitimate ATI and CS-cap work for this panel; the filter values look like
 * collateral from a full GUI export rather than a deliberate change.
 */
#define XY_DYNAMIC_FILTER_BOTTOM_SPEED_0         0x06
#define XY_DYNAMIC_FILTER_BOTTOM_SPEED_1         0x00
#define XY_DYNAMIC_FILTER_TOP_SPEED_0            0x7C
#define XY_DYNAMIC_FILTER_TOP_SPEED_1            0x00
#define XY_DYNAMIC_FILTER_BOTTOM_BETA            0x07
#define XY_DYNAMIC_FILTER_STATIC_FILTER_BETA     0x80
/*
 * Movement below this (in output-resolution pixels) counts as a stationary
 * touch, which gates the TP Movement flag and the drop to Idle-Touch mode
 * (datasheet 7.5). 0x14 is what every other IQS7211E driver ships; this had
 * been at 0x1E since bc9e6df, the same commit that lost the XY filter values.
 * It does not affect reported coordinates.
 */
#define STATIONARY_TOUCH_MOV_THRESHOLD           0x14
#define FINGER_SPLIT_FACTOR                      0x03
#define X_TRIM_VALUE                             0x14
#define Y_TRIM_VALUE                             0x14

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
