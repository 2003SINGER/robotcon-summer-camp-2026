#include "can.h"
#include <stdint.h>

extern FDCAN_HandleTypeDef hfdcan1;   

void fdcan_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter_st;

    fdcan_filter_st.FilterConfig  = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter_st.FilterID1     = 0x0000;
    fdcan_filter_st.FilterID2     = 0x0000;
    fdcan_filter_st.FilterIndex   = 0;
    fdcan_filter_st.FilterType    = FDCAN_FILTER_MASK;
    fdcan_filter_st.IdType        = FDCAN_STANDARD_ID;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter_st);

}

void FDCAN_Send_Current(int16_t curr0, int16_t curr1)
{
		uint8_t send_data[8] = {0};
		FDCAN_TxHeaderTypeDef Tx_message;
	
		Tx_message.DataLength = FDCAN_DLC_BYTES_8;
		Tx_message.Identifier = 0x200;
		Tx_message.IdType = FDCAN_STANDARD_ID;
		Tx_message.TxFrameType = FDCAN_DATA_FRAME;
		
		send_data[0] = (curr0 >> 8) & 0xFF;
		send_data[1] = curr0 & 0xFF;
		send_data[2] = (curr1 >> 8) & 0xFF;
    send_data[3] = curr1 & 0xFF;
    send_data[4] = 0;
    send_data[5] = 0;
    send_data[6] = 0;
    send_data[7] = 0;
		HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Tx_message, send_data);
		if(ret != HAL_OK)
		{
			//Error_Handler();
		}
}

