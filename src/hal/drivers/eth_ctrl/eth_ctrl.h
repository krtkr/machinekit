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

#ifndef ETH_CTRL_H_
#define ETH_CTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define STEP_DIRs_COUNT 4
#define QEIs_COUNT 4
#define GPIOs_COUNT 32

typedef struct __attribute__((packed)) {
	int32_t steps[STEP_DIRs_COUNT];
	int32_t qei[QEIs_COUNT];
	int16_t qei_z[QEIs_COUNT];
	uint32_t gpi;
	uint32_t reserved0;
} eth_ctrl_rx_data_t;

// Control bits masks
#define STEP_DIR_CONTROL_ENABLE_MASK        (0x01)
#define STEP_DIR_CONTROL_MOVE_MASK          (0x02)
#define STEP_DIR_CONTROL_DIR_MASK           (0x04)
#define STEP_DIR_CONTROL_RESET_MASK         (0x08)
#define STEP_DIR_CONTROL_INVERT_ENABLE_MASK (0x10)
#define STEP_DIR_CONTROL_INVERT_STEP_MASK   (0x20)
#define STEP_DIR_CONTROL_INVERT_DIR_MASK    (0x40)

#define QEI_CONTROL_RESET_MASK              (0x01)
#define QEI_CONTROL_INVERT_A_MASK           (0x02)
#define QEI_CONTROL_INVERT_B_MASK           (0x04)
#define QEI_CONTROL_INVERT_Z_MASK           (0x08)

#define WRITE_SYMBOL                        (0x5a)

typedef struct __attribute__((packed)) {
	uint32_t period[STEP_DIRs_COUNT];
	uint16_t step_length[STEP_DIRs_COUNT];
	uint8_t  step_dir_control[STEP_DIRs_COUNT];
	uint8_t  qei_control[QEIs_COUNT];
	uint32_t reserved0;
	uint32_t reserved1;
	uint32_t gpo;
	uint8_t  timeout_scale;
	uint16_t reserved2;
	uint8_t  write;
} eth_ctrl_tx_data_t;

typedef struct {
	void* connection; // Set to 0 on initialization
} eth_ctrl_conn_t;

typedef enum {
	ETH_CTRL_NO_ERROR = 0,
	ETH_CTRL_CONN_IS_NULL,
	ETH_CTRL_ATTEMPT_TO_REOPEN,
	ETH_CTRL_MALLOC_ERROR,
	ETH_CTRL_NIC_NAME_IS_NULL,
	ETH_CTRL_NIC_NAME_TOO_LONG,
	ETH_CTRL_UNABLE_TO_OPEN_SOCKET,
	ETH_CTRL_UNABLE_TO_GET_HWADDR,
	ETH_CTRL_UNABLE_TO_GET_NIC_INDEX,
} eth_ctrl_error_code_t;

extern const char* eth_ctrl_error_msgs[9];

extern const uint8_t eth_ctrl_mac[6];

int eth_ctrl_open(eth_ctrl_conn_t* conn, const char* nic_name);
int eth_ctrl_send(eth_ctrl_conn_t* conn, eth_ctrl_tx_data_t* tx_data, unsigned module_id);
int eth_ctrl_recv(eth_ctrl_conn_t* conn, eth_ctrl_rx_data_t* rx_data);
void eth_ctrl_close(eth_ctrl_conn_t* conn);

int eth_ctrl_version();

#ifdef __cplusplus
}
#endif

#endif /* ETH_CTRL_H_ */
