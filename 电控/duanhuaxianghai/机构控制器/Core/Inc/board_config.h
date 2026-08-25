#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Hardware facts and tunable constants live here; no control code owns them. */
#include <stdint.h>

#define CONTROL_PERIOD_S                    0.001f
#define MOTOR_FEEDBACK_TIMEOUT_MS           20U

/* Confirm only the Z-axis ID on the assembled CAN bus before first motion. */
#define CAN_ID_LOADER_MOTOR_A               0x201U
#define CAN_ID_LOADER_MOTOR_B               0x202U
#define CAN_ID_LIFT_MOTOR                   0x203U
#define CAN_ID_ROTATOR_MOTOR                0x206U
#define CAN_ID_M2006_COMMAND_GROUP          0x200U
#define CAN_ID_GM6020_COMMAND_GROUP         0x1FFU

/* Existing loader source used these raw targets; keep them distinct. */
#define LOADER_A_OUT_COUNTS                 (8192L * 20L * 38L)
#define LOADER_B_OUT_COUNTS                 (8192L * 20L * 35L)

#define M2006_CURRENT_LIMIT                 4000.0f
#define GM6020_CURRENT_LIMIT                20000.0f

/* Measured after assembly.  Zero means compensation is deliberately disabled. */
#define LIFT_GRAVITY_CURRENT                0.0f
#define ROTATOR_GRAVITY_CURRENT             0.0f

/* Initial signs are copied from the separate examples; verify on an unloaded rig. */
#define LOADER_A_FEEDBACK_SIGN              1
#define LOADER_B_FEEDBACK_SIGN              1
#define LIFT_FEEDBACK_SIGN                  1
#define ROTATOR_FEEDBACK_SIGN               1
#define LOADER_A_CURRENT_SIGN               1
#define LOADER_B_CURRENT_SIGN               1
#define LIFT_CURRENT_SIGN                   1
#define ROTATOR_CURRENT_SIGN                1

#endif
