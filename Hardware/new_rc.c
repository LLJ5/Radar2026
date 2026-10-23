#include "new_rc.h"
#include "usart.h"
#include "struct_typedef.h"
#include "string.h"
uint8_t VT13_recieve_1[42],VT13_recieve_2[42];
void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{	
  /* configuare the huart Reception Type TOIDLE */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* configuare the huart Receicve Size */
   huart->RxXferSize = DataLength;
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Disable DMA */
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;

  /* Configure the destination memory Buffer address */
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  huart->hdmarx->Instance->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}




union{
VT13_data use_data;
uint8_t  temp_data[21];
}VT13_USE;
u8 new_live_3; 
void USER_USART6_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
	new_live_3=100;
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
  {
			__HAL_DMA_DISABLE(huart->hdmarx);

			huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

		  if((VT13_recieve_1[0]==0xA9)&&(VT13_recieve_1[1]==0x53))
		  memcpy(VT13_USE.temp_data,VT13_recieve_1,20);
      __HAL_DMA_SET_COUNTER(huart->hdmarx,20*2);

  }
  else
  {
			__HAL_DMA_DISABLE(huart->hdmarx);

			huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
		
		  if((VT13_recieve_2[0]==0xA9)&&(VT13_recieve_2[1]==0x53))
		  memcpy(VT13_USE.temp_data,VT13_recieve_2,20);
      __HAL_DMA_SET_COUNTER(huart->hdmarx,20*2);

  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
if(huart->Instance == USART6)
	{		
		USER_USART6_RxHandler(huart,Size);	
	}

  /* reset the Reception Type */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}

void tele_link_init(void)
{
	USART_RxDMA_MultiBufferStart(&huart6,(uint32_t *)&(huart6.Instance->DR),(uint32_t *)&VT13_recieve_1[0],(uint32_t *)&VT13_recieve_2[1],42U);
}


VT13_ctrl_t VT13_ctrl;
void work_out_init(void)
{
	work_out_tele_data(&VT13_USE.use_data);
}
void work_out_tele_data(VT13_data *data)
{
			VT13_ctrl.R_x = 1024 - data->ch_0; //!< Channel 0
			VT13_ctrl.R_y = 1024 - data->ch_1; //!< Channel 1
			VT13_ctrl.L_x = 1024 - data->ch_3;//!< Channel 2
			VT13_ctrl.L_y = 1024 - data->ch_2;//!< Channel 3
			//rc_ctrl->last_sr = rc_ctrl->sr;
			VT13_ctrl.mouse_x = data->mouse_x; //!< Mouse X axis
			VT13_ctrl.mouse_y = data->mouse_y; ; //!< Mouse Y axis
			VT13_ctrl.mouse_z = data->mouse_z;  //!< Mouse Z axis
			VT13_ctrl.mouse_l = data->mouse_left;  //!< Mouse Left Is Press ?
			VT13_ctrl.mouse_r = data->mouse_right; //!< Mouse Right Is Press ?
			VT13_ctrl.mouse_mid  = data->mouse_middle;
			VT13_ctrl.key = data->key;//!< KeyBoard value
			VT13_ctrl.lun = -(data->wheel-1024); //²¦ÂÖ
			//rc_ctrl->last_lun = rc_ctrl->lun;
			VT13_ctrl.W = (data->key&0x01)>>0;
			VT13_ctrl.S = (data->key&0x02)>>1;
			VT13_ctrl.A = (data->key&0x04)>>2;
			VT13_ctrl.D = (data->key&0x08)>>3;
			VT13_ctrl.Shift = (data->key&0x10)>>4;
			VT13_ctrl.Ctrl = (data->key&0x20)>>5;
			VT13_ctrl.Q = (data->key&0x40)>>6;
			VT13_ctrl.E  = (data->key&0x80)>>7;
			VT13_ctrl.R = (data->key&0x100)>>8;
			VT13_ctrl.F = (data->key&0x200)>>9;
			VT13_ctrl.G = (data->key&0x400)>>10;
			VT13_ctrl.Z = (data->key&0x800)>>11;
			VT13_ctrl.X = (data->key&0x1000)>>12;
			VT13_ctrl.C = (data->key&0x2000)>>13;
			VT13_ctrl.V = (data->key&0x4000)>>14;
			VT13_ctrl.B  = (data->key&0x8000)>>15;
			VT13_ctrl.mode  = data->mode_sw;
			VT13_ctrl.trigger  = data->trigger;
			VT13_ctrl.s_l = data->fn_1;
			VT13_ctrl.s_r = data->fn_2;
		//Ò£¿ØÆ÷ÖµÅ×³öÒì³£
			VT13_ctrl.R_x = VT13_ctrl.R_x > 660 ? 0 : VT13_ctrl.R_x;
			VT13_ctrl.R_x = VT13_ctrl.R_x < -660 ? 0 : VT13_ctrl.R_x;
			
			VT13_ctrl.R_y = VT13_ctrl.R_y > 660 ? 0 : VT13_ctrl.R_y;
			VT13_ctrl.R_y = VT13_ctrl.R_y < -660 ? 0 : VT13_ctrl.R_y;
			
			VT13_ctrl.L_x = VT13_ctrl.L_x > 660 ? 0 : VT13_ctrl.L_x;
			VT13_ctrl.L_x = VT13_ctrl.L_x < -660 ? 0 : VT13_ctrl.L_x;
			
			VT13_ctrl.L_y = VT13_ctrl.L_y > 660 ? 0 : VT13_ctrl.L_y;
			VT13_ctrl.L_y = VT13_ctrl.L_y < -660 ? 0 : VT13_ctrl.L_y;
			
			VT13_ctrl.mouse_x = VT13_ctrl.mouse_x > 1024 ? 1024 : VT13_ctrl.mouse_x;
			VT13_ctrl.mouse_x = VT13_ctrl.mouse_x < -1024 ? -1024 : VT13_ctrl.mouse_x;
			VT13_ctrl.mouse_y = VT13_ctrl.mouse_y > 1024 ? 1024 : VT13_ctrl.mouse_y;
			VT13_ctrl.mouse_y = VT13_ctrl.mouse_y < -1024 ? -1024 : VT13_ctrl.mouse_y;
}