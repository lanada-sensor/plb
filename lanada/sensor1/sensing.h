#ifndef __SENSING_H__
#define __SENSING_H__
#include "contiki.h"

//static int Sensor_start();
void InitSensor(void);

int 			i;
	int 			va;
	int 			sel_num;
	char 			w_char, r_char;


	int32_t 		CWT_i, CWT_j, CWT_k, CWT_delay_k, CWT_delay_cnt, UART_wait_time_index,
					Tx_wait_time_index;
	int32_t 		CWT_hfreq, CWT_lfreq;

	int32_t			CWT_data_x_prev[7], CWT_data_y_prev[7],
					CWT_base_addr,
					CWT_hp_b[7], CWT_hp_a[7],
					CWT_hpf_out_01, CWT_hpf_out,
					CWT_read_data,
					CWT_first_sig_base_addr, CWT_second_sig_base_addr,
					CWT_coef_dw_sel[30], CWT_coef_up_sel[30],
					CWT_in_data[29],
					CWT_X1, CWT_X2, CWT_LRS,
					CWT_out_dw_01, CWT_out_up_01,
					CWT_out_dw, CWT_out_up,
					CWT_dw_square_acc, CWT_up_square_acc,
					CWT_dw_square_mean, CWT_up_square_mean,
					CWT_dw_acc, CWT_up_acc,
					CWT_dw_mean, CWT_up_mean,
					CWT_dw_mean_square, CWT_up_mean_square,
					CWT_dw_variance[33], CWT_up_variance[33];

	char			CWT_coef_dw_mat[33][30], CWT_coef_up_mat[33][30];

	int32_t			CWT_UART_addr;
	char			CWT_UART_addr_L8bit, CWT_UART_addr_M8bit, CWT_UART_addr_H3bit,
					CWT_UART_rdata_L8bit, CWT_UART_rdata_H8bit,
					CWT_UART_wdata_L8bit, CWT_UART_wdata_H8bit,
					CWT_UART_addr_wcmd, CWT_UART_cmd_datard, CWT_UART_cmd_datawr,
					CWT_UART_cmd_HfLf_tx, CWT_UART_cmd_Hf_tx,
					CWT_UART_tx_freq;


	////////////////////////////////////////////////////




#endif /* __SNUST_H__ */
