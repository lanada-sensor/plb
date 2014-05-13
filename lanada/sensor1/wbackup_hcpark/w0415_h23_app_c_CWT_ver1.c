/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         APP layer for sensor network
 * \author
 *         Jinhwan, Jung <jhjun@lanada.kaist.ac.kr>
 */

#include "contiki.h"
//#include "net/rime.h"

#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "sys/etimer.h"


#include "../cpu/msp430/dev/uart0.h" // hcpark,
//#include "../cpu/msp430/dev/uart1.h" // hcpark,

#include "uart.h"


#include <stdio.h>
#include <stdlib.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
PROCESS(app_layer_process, "SNUST Sensor network App layer");
AUTOSTART_PROCESSES(&app_layer_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_layer_process, ev, data)
{
	static struct etimer et;
	int i=0;
	int va;

	int sel_num;
	char w_char, r_char;

	int32_t 		CWT_i, CWT_j, CWT_k;
	int32_t 		CWT_hfreq, CWT_lfreq;

	int32_t			CWT_data_x_prev[7], CWT_data_y_prev[7],
					CWT_base_addr,
					CWT_hp_b[7], CWT_hp_a[7],
					CWT_hpf_out_01, CWT_hpf_out,
					CWT_read_data,
					CWT_first_sig_base_addr, CWT_second_sig_base_addr,
					CWT_coef_dw_sel[30], CWT_coef_up_sel[30],
					CWT_coef_dw_mat[33][30], CWT_coef_up_mat[33][30],
					CWT_in_data[29],
					CWT_X1, CWT_X2, CWT_LRS,
					CWT_out_dw_01, CWT_out_up_01,
					CWT_out_dw, CWT_out_up,
					CWT_dw_square_acc, CWT_up_square_acc,
					CWT_dw_square_mean, CWT_up_square_mean,
					CWT_dw_acc, CWT_up_acc,
					CWT_dw_mean, CWT_up_mean,
					CWT_dw_mean_square, CWT_dw_mean_square,
					CWT_dw_variance[33], CWT_up_variance[33];










	PROCESS_BEGIN();
	SENSORS_ACTIVATE(button_sensor);

	slip_arch_init(115200);
	//uart0_init(115200);

	sel_num = 0;
	w_char = '1';

	if(sel_num)
		printf("snust  BEGINNING if\n");
	else
	{
		//printf("snust  BEGINNING else\n");

		for( va=0; va<10; va++)
			UART_out(w_char); // libuart


	}

	while(1)
	{
		r_char = UCA0RXBUF;

		if( r_char == w_char)
		{
			printf("keyboard pressed : %c \n", r_char);
			w_char ++;
		}

	}



	// ----------------------------------------------------------

	// CWT_read_data <-- USART

	// initialization
	// signed 8 bit
	CWT_hp_b[1] = 11;
	CWT_hp_b[2] = -57;
	CWT_hp_b[3] = 113;
	CWT_hp_b[4] = -113;
	CWT_hp_b[5] = 57;
	CWT_hp_b[6] = -11;

	CWT_hp_a[1] = 32;
	CWT_hp_a[2] = -95;
	CWT_hp_a[3] = 122;
	CWT_hp_a[4] = -81;
	CWT_hp_a[5] = 28;
	CWT_hp_a[6] = -4;


	CWT_coef_dw_mat[][0] = ;

	CWT_coef_up_mat[][0] = ;

	for(CWT_hfreq=0; CWT_hfreq<3; CWT_hfreq++)
	{

		for(CWT_lfreq=0; CWT_lfreq<11; CWT_lfreq++)
		{

			// %% High pass filtering for LF removing
			// X1i = High_filter (5,f_dc, Fs, SI11);

			for(CWT_i=0; CWT_i<2; CWT_i++)	//  two signals
			{

				//[2]-->[3]....
				// [6] is oldest data
				CWT_data_x_prev[2] = 0;
				CWT_data_x_prev[3] = 0;
				CWT_data_x_prev[4] = 0;
				CWT_data_x_prev[5] = 0;
				CWT_data_x_prev[6] = 0;

				CWT_data_y_prev[2] = 0;
				CWT_data_y_prev[3] = 0;
				CWT_data_y_prev[4] = 0;
				CWT_data_y_prev[5] = 0;
				CWT_data_y_prev[6] = 0;


				if( CWT_i==0 )
						CWT_base_addr = 0; // ?? , first signal
				else	CWT_base_addr = 0; // ?? , second signal


				for(CWT_j=0; CWT_j<500000; CWT_j++) // 500,000
				{

					// ?? CPLD data read using USART
					// addr = CWT_base_addr + CWT_j;
					// USART

					CWT_hpf_out_01 = CWT_hp_b[1] * CWT_read_data;

					for(CWT_k=2; CWT_k<7; CWT_k++)
					{
						CWT_hpf_out_01 = CWT_hpf_out_01 + (CWT_hp_b[CWT_k] * CWT_data_x_prev[CWT_k]) - (CWT_hp_a[CWT_k] * CWT_data_y_prev[CWT_k]);
					}

					// HPF output generation
					CWT_hpf_out = ( CWT_hpf_out_01 >> 5); // since CWT_hp_a[1] == 32, 14 bit (from wMOD~4Code.m)

					for(CWT_k=6; CWT_k>2; CWT_k--)
					{
						CWT_data_x_prev[CWT_k] = CWT_data_x_prev[CWT_k - 1];
						CWT_data_y_prev[CWT_k] = CWT_data_y_prev[CWT_k - 1];
					}
					CWT_data_x_prev[2] = CWT_read_data;
					CWT_data_y_prev[2] = CWT_hpf_out;



					if( CWT_j >= 200000 )	// steady
					{
						// ?? CPLD data write using USART
						// USART
					}


				} // for(CWT_j=0; CWT_j<500000; CWT_j++)  // 500,000


			} // for(CWT_i=0; CWT_i<2; CWT_i++)

			// ==== END ===== %% High pass filtering for LF removing



			// %% LRS, length: 300,001
			// LRS_i = X1i_s- X2i_s;

			CWT_first_sig_base_addr = 0; // ?? , first signal
			CWT_second_sig_base_addr = 0; // ?? , second signal

			for(CWT_j=0; CWT_j<30; CWT_j++)
			{
				CWT_coef_dw_sel[CWT_j] = CWT_coef_dw_mat[CWT_hfreq*11 + CWT_lfreq][CWT_j];
				CWT_coef_up_sel[CWT_j] = CWT_coef_up_mat[CWT_hfreq*11 + CWT_lfreq][CWT_j];
			}

			for(CWT_j=0; CWT_j<29; CWT_j++)
			{
				CWT_in_data[CWT_j] = 0;
			}

			CWT_dw_acc = 0;
			CWT_up_acc = 0;
			CWT_dw_square_acc = 0;
			CWT_up_square_acc = 0;

			for(CWT_j=0; CWT_j<262144; CWT_j++) // 300,000, 262144=2^18
			{

				// ?? CPLD data read using USART, first data
				// addr = CWT_first_sig_base_addr + CWT_j;
				// USART
				// CWT_X1

				// ?? CPLD data read using USART, second data
				// addr = CWT_second_sig_base_addr + CWT_j;
				// USART
				// CWT_X2

				CWT_LRS = CWT_X1 - CWT_X2;

				CWT_out_dw_01 = CWT_LRS * CWT_coef_dw_sel[0];
				CWT_out_up_01 = CWT_LRS * CWT_coef_up_sel[0];

				for(CWT_k=0; CWT_k<29; CWT_k++)
				{
					CWT_out_dw_01 = CWT_out_dw_01 + (CWT_in_data[CWT_k] * CWT_coef_dw_sel[CWT_k+1]);
					CWT_out_up_01 = CWT_out_up_01 + (CWT_in_data[CWT_k] * CWT_coef_up_sel[CWT_k+1]);
				}

				for(CWT_k=27; CWT_k>=0; CWT_k--)
				{
					CWT_in_data[CWT_k + 1] = CWT_in_data[CWT_k];
				}

				CWT_in_data[0] = CWT_LRS;

				CWT_out_dw = CWT_out_dw_01; // must be signed 8 bit, -127~127
				CWT_out_up = CWT_out_up_01;


				// standard deviation calculation
				CWT_dw_square_acc = CWT_dw_square_acc + (CWT_out_dw*CWT_out_dw);
				CWT_up_square_acc = CWT_up_square_acc + (CWT_out_up*CWT_out_up);
				CWT_dw_acc = CWT_dw_acc + CWT_out_dw;
				CWT_up_acc = CWT_up_acc + CWT_out_up;

			}


			CWT_dw_square_mean = (CWT_dw_square_acc >> 18);
			CWT_up_square_mean = (CWT_up_square_acc >> 18);

			CWT_dw_mean = (CWT_dw_acc >> 18);
			CWT_up_mean = (CWT_up_acc >> 18);

			CWT_dw_mean_square = CWT_dw_mean * CWT_dw_mean;
			CWT_up_mean_square = CWT_up_mean * CWT_up_mean;

			CWT_dw_variance[CWT_hfreq*11 + CWT_lfreq] = CWT_dw_square_mean - CWT_dw_mean_square;
			CWT_up_variance[CWT_hfreq*11 + CWT_lfreq] = CWT_up_square_mean - CWT_up_mean_square;


		} // for(CWT_lfreq=3000; CWT_lfreq<=4000; CWT_lfreq+=100)


	} // for(CWT_hfreq=202; CWT_hfreq<205; CWT_hfreq++)


	// Outlier analysis
	// CWT_dw_variance[CWT_hfreq*11 + CWT_lfreq],
	// CWT_up_variance[CWT_hfreq*11 + CWT_lfreq]


	// ------------------------------------------------------------



	while (1) {
		// button sensor
		PROCESS_WAIT_EVENT_UNTIL(
				(ev == sensors_event) && (data == &button_sensor));
		leds_on(LEDS_RED);


		etimer_set(&et, CLOCK_SECOND / 1000);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));


	}


	PROCESS_END();
}

