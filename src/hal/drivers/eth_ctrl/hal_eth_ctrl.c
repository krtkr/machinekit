/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.
*/

#include "eth_ctrl.h"

#include "rtapi.h"            /* RTAPI realtime OS API */
#include "rtapi_app.h"        /* RTAPI realtime module decls */
#include "hal.h"            /* HAL public API decls */
#include "rtapi_math.h"

#include <string.h>

#define MAX_MODULES                       8

#define MAX_MODULE_ID                     15

/* module information */
MODULE_AUTHOR("Kirill Kranke");
MODULE_DESCRIPTION("Ethernet Control driver for EMC HAL");
MODULE_LICENSE("GPL");

int module_ids[MAX_MODULES] = { [0 ... MAX_MODULES - 1] = -1 } ;
RTAPI_MP_ARRAY_INT(module_ids, MAX_MODULES, "module identifiers for boards");

int base_frequency[MAX_MODULES] = { [0 ... MAX_MODULES - 1] = 125000000 };
RTAPI_MP_ARRAY_INT(base_frequency, MAX_MODULES, "base frequency for boards");

char *nic_name;
RTAPI_MP_STRING(nic_name, "network interface name");

typedef struct {
    hal_bit_t invert_enable;              /* param: 'enable' signal inversion, true to invert */
    hal_bit_t invert_step;                /* param: 'step' signal inversion, true to invert */
    hal_bit_t invert_dir;                 /* param: 'dir' signal inversion, true to invert */
    hal_float_t step_length;              /* param: step signal length, ns */
    uint16_t step_length_t;               /* step signal length, ticks */
    hal_float_t pos_scale;                /* param: steps per position unit */
    hal_s32_t *steps;                     /* pin: captured feedback (steps) */
    hal_float_t vel_cmd_pps_min;          /* param: minimum velocity command to process (position units per second) */
    hal_float_t *vel_cmd_pps;             /* pin: position command (position units per second) */
    hal_float_t *pos_fb;                  /* pin: position feedback (position units) */
    hal_bit_t *enable;                    /* pin: 'enable' signal  */
} hal_eth_step_dir_t;

typedef struct {
    hal_s32_t *steps;                     /* pin: captured feedback (steps) */
    hal_s32_t *z_steps;                   /* pin: captured Z feedback, actual value is 16 bit's signed integer (steps) */
    hal_bit_t invert_a;                   /* param: 'enable' signal inversion, true to invert */
    hal_bit_t invert_b;                   /* param: 'step' signal inversion, true to invert */
    hal_bit_t invert_z;                   /* param: 'dir' signal inversion, true to invert */
    hal_float_t pos_scale;                /* param: steps per position unit */
    hal_float_t *pos_fb;                  /* pin: position feedback (position units) */
} hal_eth_qei_t;

typedef struct {
    hal_eth_step_dir_t step_dir[STEP_DIRs_COUNT];
    hal_eth_qei_t qei[QEIs_COUNT];
    hal_bit_t *data_in[GPIOs_COUNT];      /* pins: input data */
    hal_bit_t *data_out[GPIOs_COUNT];     /* pins: output data */

    hal_bit_t params_update;              /* param: set to true to force params update on next write call */
    hal_bit_t reset_counters;             /* param: set to true to force step/dir and qei counters reset on next write call */

    int module_id;                        /* module id */
    int base_freq;                        /* base frequency */

    hal_float_t tps;                      /* ticks per second */

    int requested;                        /* request has been sent */
} hal_eth_ctrl_t;

static hal_eth_ctrl_t *boards;
static hal_eth_ctrl_t *boards_map[MAX_MODULE_ID];
static int module_idx_map[MAX_MODULE_ID];
static int boards_count = 0;
static int comp_id;                       /* component ID */
static long period_prev = 0;

static eth_ctrl_tx_data_t tx_data;        /* tx data buffer */
static eth_ctrl_rx_data_t rx_data;        /* rx data buffer */
static eth_ctrl_conn_t conn;              /* eth_ctrl connection */

static void read(void *arg, long period);
static void write(void *arg, long period);

static void reset_values(hal_eth_ctrl_t *board);
static int export_board(hal_eth_ctrl_t *board, int num);

int rtapi_app_main(void)
{
    int n, retval = 0;

    // Initialize variables
    for (n = 0; n < MAX_MODULE_ID; n++) {
        boards_map[n] = 0;
        module_idx_map[n] = -1;
    }

    // Check arguments
    for (n = 0; n < MAX_MODULES && module_ids[n] > 0 ; n++) {
        if(module_ids[n] > MAX_MODULE_ID) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "ETH_CTRL: ERROR: bad module id '%i' at module #%i, should be [%i:%i]\n",
                            module_ids[n], n, 0, MAX_MODULE_ID);
            retval = -1;
            goto clean_up_nothing;
        }
        if (module_idx_map[module_ids[n]] >= 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "ETH_CTRL: ERROR: duplicated module '%i' at module #%i, previous module #%i\n",
                            module_ids[n], n, module_idx_map[module_ids[n]]);
            retval = -1;
            goto clean_up_nothing;
        }
        module_idx_map[module_ids[n]] = n;
        if(base_frequency[n] <= 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "ETH_CTRL: ERROR: bad control base frequency '%i' for module %i, expect non zero positive integer\n",
                            base_frequency[n], n);
            retval = -2;
            goto clean_up_nothing;
        }
        boards_count++;
    }

    /* Connect to the HAL */
    comp_id = hal_init("hal_eth_ctrl");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "ETH_CTRL: ERROR: hal_init() failed\n");
        retval = -3;
        goto clean_up_nothing;
    }

    /* Allocate shared memory for data */
    boards = hal_malloc(boards_count * sizeof(hal_eth_ctrl_t));
    if (boards == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "ETH_CTRL: ERROR: hal_malloc() failed\n");
        retval = -4;
        goto clean_up_component;
    }

    // Export pins
    for (n = 0; n < boards_count; n++) {
        retval = export_board(boards + n, n);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "ETH_CTRL: ERROR: failed to export module #%i variables\n", n);
            goto clean_up_memory;
        }
        boards[n].module_id = module_ids[n];
        boards[n].base_freq = base_frequency[n];
        boards[n].requested = 0;
        boards_map[module_ids[n]] = boards;
    }

    // Reset values
    for (n = 0; n < boards_count; n++) {
        reset_values(boards + n);
    }

    /* Export functions */
    retval = hal_export_funct("hal_eth_ctrl.read", read, boards, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "ETH_CTRL: ERROR: read function export failed\n");
        goto clean_up_memory;
    }

    retval = hal_export_funct("hal_eth_ctrl.write", write, boards, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "ETH_CTRL: ERROR: write function export failed\n");
        goto clean_up_memory;
    }

    // We should initialize connection to zero
    conn.connection = 0;

    // Set initial data to zeros
    memset(&tx_data, 0, sizeof(tx_data));
    memset(&rx_data, 0, sizeof(rx_data));

    retval = eth_ctrl_open(&conn, nic_name);
    if (retval != ETH_CTRL_NO_ERROR) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "ETH_CTRL: ERROR: unable to open connection for module #%i, error = %i, msg = %s\n", n, retval, eth_ctrl_error_msgs[retval]);
        goto clean_up_connections;
    }

    return 0;

clean_up_connections:
    for (n = 0; n < boards_count; n++) {
        eth_ctrl_close(&conn);
    }

clean_up_memory:
    // No hal_free, ignore

clean_up_component:
    hal_exit(comp_id);

clean_up_nothing:
    return retval;
}

void rtapi_app_exit(void) {
    eth_ctrl_close(&conn);
    hal_exit(comp_id);
}

static void read(void *arg, long period) {
    int n, i, module_id;
    hal_eth_ctrl_t* board;
    for (n = 0; n < boards_count; n++) {
        board = boards + n;
        if (!board->requested) {
            tx_data.write = 0x00;
            eth_ctrl_send(&conn, &tx_data, board->module_id);
            board->requested = 1;
        }
        module_id = eth_ctrl_recv(&conn, &rx_data);
        if ((module_id >= 0) && (module_id <= MAX_MODULE_ID)) {
            board = boards_map[module_id];
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "ETH_CTRL: Bad module id received or error: %i.\n",
                            module_id);
            continue;
        }
        if (board) {
            board->requested = 0;
            for (i = 0; i < STEP_DIRs_COUNT; i++) {
                *(board->step_dir[i].steps) = rx_data.steps[i];
                *(board->step_dir[i].pos_fb) = (hal_float_t)rx_data.steps[i] / board->step_dir[i].pos_scale;
            }
            for (i = 0; i < QEIs_COUNT; i++) {
                *(board->qei[i].steps) = rx_data.qei[i];
                *(board->qei[i].z_steps) = rx_data.qei_z[i];
                *(board->qei[i].pos_fb) = (hal_float_t)rx_data.qei[i] / board->qei[i].pos_scale;
            }
            for (i = 0; i < GPIOs_COUNT; i++) {
                *(board->data_out[i]) = !!(rx_data.gpi & (1 << i));
            }
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "ETH_CTRL: Board #%i with module id %i is not configured, but data received.\n",
                            n, module_id);
            continue;
        }
    }
}

static void write(void *arg, long period) {
    int n, i;
    hal_eth_ctrl_t* board;

    hal_float_t vel_cmd_sps;

    for (n = 0; n < boards_count; n++) {
        board = boards + n;
        if (board->params_update || (period_prev != period)) {
            period_prev = period;
            board->params_update = 0;

            for (i = 0; i < STEP_DIRs_COUNT; i++) {
                // Update step length
                board->step_dir[i].step_length_t = (uint16_t)((hal_float_t)board->base_freq / 1000000000.0 * board->step_dir[i].step_length);
            }

            // Update tps
            board->tps = (hal_float_t) board->base_freq;
        }

        for (i = 0; i < STEP_DIRs_COUNT; i++) {
            // Set flags
            tx_data.step_dir_control[i] = 0;
            if (*(board->step_dir[i].enable)) {
                tx_data.step_dir_control[i] |= STEP_DIR_CONTROL_ENABLE_MASK;
            }
            if (board->step_dir[i].invert_enable) {
                tx_data.step_dir_control[i] |= STEP_DIR_CONTROL_INVERT_ENABLE_MASK;
            }
            if (board->step_dir[i].invert_step) {
                tx_data.step_dir_control[i] |= STEP_DIR_CONTROL_INVERT_STEP_MASK;
            }
            if (board->step_dir[i].invert_dir) {
                tx_data.step_dir_control[i] |= STEP_DIR_CONTROL_INVERT_DIR_MASK;
            }
            if (board->reset_counters) {
                tx_data.step_dir_control[i] |= STEP_DIR_CONTROL_RESET_MASK;
            }

            tx_data.step_length[i] = board->step_dir[i].step_length_t;

            // calculate velocity command in steps/s
            vel_cmd_sps = *board->step_dir[i].vel_cmd_pps * board->step_dir[i].pos_scale;

            // Check if we need move
            if (vel_cmd_sps < 0) {
                vel_cmd_sps = -vel_cmd_sps;
                tx_data.step_dir_control[i] |= STEP_DIR_CONTROL_DIR_MASK;
            }

            if (vel_cmd_sps > board->step_dir[i].vel_cmd_pps_min) {
                tx_data.step_dir_control[i] |= STEP_DIR_CONTROL_MOVE_MASK;
            }

            // Now convert velocity to ticks per step
            if (vel_cmd_sps > 0) {
                tx_data.period[i] = (uint32_t)(board->tps / vel_cmd_sps);
            }
        }

        for (i = 0; i < QEIs_COUNT; i++) {
            if (board->reset_counters) {
                tx_data.qei_control[i] = QEI_CONTROL_RESET_MASK;
            }
            if (board->qei[i].invert_a) {
                tx_data.qei_control[i] |= QEI_CONTROL_INVERT_A_MASK;
            }
            if (board->qei[i].invert_b) {
                tx_data.qei_control[i] |= QEI_CONTROL_INVERT_B_MASK;
            }
            if (board->qei[i].invert_z) {
                tx_data.qei_control[i] |= QEI_CONTROL_INVERT_Z_MASK;
            }
        }

        tx_data.gpo = 0;
        for (i = 0; i < GPIOs_COUNT; i++) {
            tx_data.gpo |= (!!*(board->data_in[i]) << i);
        }

        // Allow write
        tx_data.write = WRITE_SYMBOL;

        eth_ctrl_send(&conn, &tx_data, board->module_id);
        board->requested = 1;
        board->reset_counters = 0;
    }
}

static void reset_values(hal_eth_ctrl_t *board) {
    int i;
    for (i = 0; i < STEP_DIRs_COUNT; i++) {
        board->step_dir[i].invert_enable   = 0;
        board->step_dir[i].invert_step     = 0;
        board->step_dir[i].invert_dir      = 0;
        board->step_dir[i].step_length     = 20000.0;
        board->step_dir[i].step_length_t   = 0;
        board->step_dir[i].pos_scale       = 0.01;
        *(board->step_dir[i].steps)        = 0.0;
        board->step_dir[i].vel_cmd_pps_min = 0.000001;
        *(board->step_dir[i].vel_cmd_pps)  = 0.0;
        *(board->step_dir[i].pos_fb)       = 0.0;
        *(board->step_dir[i].enable)       = 0;
    }

    for (i = 0; i < QEIs_COUNT; i++) {
        *(board->qei[i].steps)             = 0;
        *(board->qei[i].z_steps)           = 0;
        board->qei[i].pos_scale            = 1000;
        *(board->qei[i].pos_fb)            = 0.0;
    }

    for (i = 0; i < GPIOs_COUNT; i++) {
        *(board->data_in[i])               = 0;
        *(board->data_out[i])              = 0;
    }

    board->params_update                   = 1;
    board->reset_counters                  = 1;

    board->tps                             = 0.0;
}

static int export_board(hal_eth_ctrl_t *board, int num) {
    int retval, n;

    for (n = 0; n < STEP_DIRs_COUNT; n++) {
        retval = hal_param_bit_newf(HAL_RW, &(board->step_dir[n].invert_enable), comp_id, "hal_eth_ctrl.%d.step_dir.%d.invert_enable", num, n);
        if (retval != 0) { return retval; }
        retval = hal_param_bit_newf(HAL_RW, &(board->step_dir[n].invert_step), comp_id, "hal_eth_ctrl.%d.step_dir.%d.invert_step", num, n);
        if (retval != 0) { return retval; }
        retval = hal_param_bit_newf(HAL_RW, &(board->step_dir[n].invert_dir), comp_id, "hal_eth_ctrl.%d.step_dir.%d.invert_dir", num, n);
        if (retval != 0) { return retval; }
        retval = hal_param_float_newf(HAL_RW, &(board->step_dir[n].step_length), comp_id, "hal_eth_ctrl.%d.step_dir.%d.step_length", num, n);
        if (retval != 0) { return retval; }
        retval = hal_param_float_newf(HAL_RW, &(board->step_dir[n].vel_cmd_pps_min), comp_id, "hal_eth_ctrl.%d.step_dir.%d.vel_cmd_min", num, n);
        if (retval != 0) { return retval; }
        retval = hal_param_float_newf(HAL_RW, &(board->step_dir[n].pos_scale), comp_id, "hal_eth_ctrl.%d.step_dir.%d.pos_scale", num, n);
        if (retval != 0) { return retval; }
        retval = hal_pin_s32_newf(HAL_OUT, &(board->step_dir[n].steps), comp_id, "hal_eth_ctrl.%d.step_dir.%d.steps", num, n);
        if (retval != 0) { return retval; }
        retval = hal_pin_float_newf(HAL_IN, &(board->step_dir[n].vel_cmd_pps), comp_id, "hal_eth_ctrl.%d.step_dir.%d.vel_cmd", num, n);
        if (retval != 0) { return retval; }
        retval = hal_pin_float_newf(HAL_OUT, &(board->step_dir[n].pos_fb), comp_id, "hal_eth_ctrl.%d.step_dir.%d.pos_fb", num, n);
        if (retval != 0) { return retval; }
        retval = hal_pin_bit_newf(HAL_IN, &(board->step_dir[n].enable), comp_id, "hal_eth_ctrl.%d.step_dir.%d.enable", num, n);
        if (retval != 0) { return retval; }
    }

    for (n = 0; n < QEIs_COUNT; n++) {
        retval = hal_pin_s32_newf(HAL_OUT, &(board->qei[n].steps), comp_id, "hal_eth_ctrl.%d.qei.%d.steps", num, n);
        if (retval != 0) { return retval; }
        retval = hal_pin_s32_newf(HAL_OUT, &(board->qei[n].z_steps), comp_id, "hal_eth_ctrl.%d.qei.%d.z_steps", num, n);
        if (retval != 0) { return retval; }
        retval = hal_param_float_newf(HAL_RW, &(board->qei[n].pos_scale), comp_id, "hal_eth_ctrl.%d.qei.%d.pos_scale", num, n);
        if (retval != 0) { return retval; }
        retval = hal_pin_float_newf(HAL_OUT, &(board->qei[n].pos_fb), comp_id, "hal_eth_ctrl.%d.qei.%d.pos_fb", num, n);
        if (retval != 0) { return retval; }

    }

    for (n = 0; n < GPIOs_COUNT; n++) {
        retval = hal_pin_bit_newf(HAL_IN, &(board->data_in[n]), comp_id, "hal_eth_ctrl.%d.data_in.%d", num, n);
        if (retval != 0) { return retval; }
        retval = hal_pin_bit_newf(HAL_OUT, &(board->data_out[n]), comp_id, "hal_eth_ctrl.%d.data_out.%d", num, n);
        if (retval != 0) { return retval; }
    }

    retval = hal_param_bit_newf(HAL_RW, &(board->params_update), comp_id, "hal_eth_ctrl.%d.params_update", num);
    if (retval != 0) { return retval; }
    retval = hal_param_bit_newf(HAL_RW, &(board->reset_counters), comp_id, "hal_eth_ctrl.%d.reset_counters", num);
    if (retval != 0) { return retval; }

    return 0;
}
