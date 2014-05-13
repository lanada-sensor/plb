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

#include "sys/clock.h"	// hcpark

#include "uart.h"
//#include "spi.h" // core/dev/spi.h
// cpu/msp430/f2xxx/spi.c

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
	static struct 	etimer et;
	int 			i=0;
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






	PROCESS_BEGIN();
	SENSORS_ACTIVATE(button_sensor); // temp

	slip_arch_init(115200);
	//uart0_init(115200);


	sel_num = 0;
	w_char = '1';

	UART_wait_time_index = 100000;
	Tx_wait_time_index = 100000; // 1 sec

	// SPI read : CPLD to MCU test
	// SPI is used for CC2420

	if(1)
	{
		  /* RS232 */
		  UCA1CTL1 |= UCSWRST;            /* Hold peripheral in reset state */
		  UCA1CTL1 |= UCSSEL_2;           /* CLK = SMCLK */

		  UCA1BR0 = 0x45;                 /* 8MHz/115200 = 69 = 0x45 */
		  UCA1BR1 = 0x00;
		  UCA1MCTL = UCBRS_3;             /* Modulation UCBRSx = 3 */

		  P3DIR &= ~0x80;                 /* P3.5 = USCI_A0 RXD as input */
		  //P3DIR |= 0x10;                  /* P3.4 = USCI_A0 TXD as output */
		  P3SEL |= 0x80;                  /* P3.4,5 = USCI_A0 TXD/RXD */
		  /*UCA0CTL1 &= ~UCSWRST;*/       /* Initialize USCI state machine */

		  //transmitting = 0;

		  /* XXX Clear pending interrupts before enable */
		  //IFG2 &= ~UCA0RXIFG;
		  //IFG2 &= ~UCA0TXIFG;
		  UCA1CTL1 &= ~UCSWRST;                   /* Initialize USCI state machine **before** enabling interrupts */
		  //IE2 |= UCA0RXIE;                        /* Enable UCA0 RX interrupt */

	}

	if(0)
	{
		printf("UCA0CTL0 :  %x \n", UCA0CTL0);
		printf("UCA0CTL1 :  %x \n", UCA0CTL1);
		printf("UCA1CTL0 :  %x \n", UCA1CTL0);
		printf("UCA1CTL1 :  %x \n", UCA1CTL1);

	}


	// UART - CPLD test
	if(0)
	{
		// UART CPLD interface test with SRAM write read
		CWT_UART_addr = 0x61234;
		CWT_UART_addr_L8bit = (CWT_UART_addr & 0xFF);
		CWT_UART_addr_M8bit = ((CWT_UART_addr & 0xFF00) >> 8);
		CWT_UART_addr_H3bit = ((CWT_UART_addr & 0x70000) >> 16);

		//
		UART_out( 0xC3 ); // libuart
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;

		UART_out( CWT_UART_addr_L8bit ); // libuart
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;

		UART_out( CWT_UART_addr_M8bit );
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;

		UART_out( CWT_UART_addr_H3bit );
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;

		CWT_UART_wdata_L8bit = 0xCD;
		CWT_UART_wdata_H8bit = 0xAB;

		UART_out( CWT_UART_wdata_L8bit );
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;

		UART_out( CWT_UART_wdata_H8bit );
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;





		UART_out( 0xC2 ); // cmd
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;

		UART_out( CWT_UART_addr_L8bit ); // libuart
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;

		UART_out( CWT_UART_addr_M8bit );
			// wait interval between UART Tx/RX
			for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
				CWT_delay_cnt++;

		UART_out( CWT_UART_addr_H3bit );

		while( (UC1IFG & 0x01)==0 ) ;
		CWT_UART_rdata_L8bit = (UCA1RXBUF & 0xFF);

		while( (UC1IFG & 0x01)==0 ) ;
		CWT_UART_rdata_H8bit = (UCA1RXBUF & 0xFF);




		CWT_read_data = (CWT_UART_rdata_H8bit << 8) + CWT_UART_rdata_L8bit;


		printf(" --- test : %x ----\n", CWT_UART_rdata_L8bit);
		printf(" --- test : %x ----\n", CWT_UART_rdata_H8bit);
		printf(" --- test : %x ----\n", CWT_read_data);



		while(1);
	}



	if(0)
	{
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
	}



	// ----------------------------------------------------------

	// CWT_read_data <-- USART

	//CWT_coef_dw_mat[][0] = ;


	CWT_coef_dw_mat[0][0] = -1;
	CWT_coef_dw_mat[0][1] = -1;
	CWT_coef_dw_mat[0][2] = 2;
	CWT_coef_dw_mat[0][3] = 5;
	CWT_coef_dw_mat[0][4] = 3;
	CWT_coef_dw_mat[0][5] = -9;
	CWT_coef_dw_mat[0][6] = -20;
	CWT_coef_dw_mat[0][7] = -7;
	CWT_coef_dw_mat[0][8] = 31;
	CWT_coef_dw_mat[0][9] = 51;
	CWT_coef_dw_mat[0][10] = 10;
	CWT_coef_dw_mat[0][11] = -65;
	CWT_coef_dw_mat[0][12] = -87;
	CWT_coef_dw_mat[0][13] = -7;
	CWT_coef_dw_mat[0][14] = 94;
	CWT_coef_dw_mat[0][15] = 94;
	CWT_coef_dw_mat[0][16] = -3;
	CWT_coef_dw_mat[0][17] = -85;
	CWT_coef_dw_mat[0][18] = -69;
	CWT_coef_dw_mat[0][19] = 8;
	CWT_coef_dw_mat[0][20] = 51;
	CWT_coef_dw_mat[0][21] = 31;
	CWT_coef_dw_mat[0][22] = -6;
	CWT_coef_dw_mat[0][23] = -20;
	CWT_coef_dw_mat[0][24] = -10;
	CWT_coef_dw_mat[0][25] = 3;
	CWT_coef_dw_mat[0][26] = 5;
	CWT_coef_dw_mat[0][27] = 2;
	CWT_coef_dw_mat[0][28] = -1;
	CWT_coef_dw_mat[0][29] = -1;
	CWT_coef_dw_mat[1][0] = 0;
	CWT_coef_dw_mat[1][1] = -1;
	CWT_coef_dw_mat[1][2] = 0;
	CWT_coef_dw_mat[1][3] = 3;
	CWT_coef_dw_mat[1][4] = 5;
	CWT_coef_dw_mat[1][5] = 1;
	CWT_coef_dw_mat[1][6] = -13;
	CWT_coef_dw_mat[1][7] = -20;
	CWT_coef_dw_mat[1][8] = 1;
	CWT_coef_dw_mat[1][9] = 39;
	CWT_coef_dw_mat[1][10] = 48;
	CWT_coef_dw_mat[1][11] = -9;
	CWT_coef_dw_mat[1][12] = -74;
	CWT_coef_dw_mat[1][13] = -78;
	CWT_coef_dw_mat[1][14] = 18;
	CWT_coef_dw_mat[1][15] = 106;
	CWT_coef_dw_mat[1][16] = 76;
	CWT_coef_dw_mat[1][17] = -23;
	CWT_coef_dw_mat[1][18] = -91;
	CWT_coef_dw_mat[1][19] = -55;
	CWT_coef_dw_mat[1][20] = 19;
	CWT_coef_dw_mat[1][21] = 52;
	CWT_coef_dw_mat[1][22] = 25;
	CWT_coef_dw_mat[1][23] = -11;
	CWT_coef_dw_mat[1][24] = -18;
	CWT_coef_dw_mat[1][25] = -8;
	CWT_coef_dw_mat[1][26] = 4;
	CWT_coef_dw_mat[1][27] = 5;
	CWT_coef_dw_mat[1][28] = 1;
	CWT_coef_dw_mat[1][29] = -1;
	CWT_coef_dw_mat[2][0] = -1;
	CWT_coef_dw_mat[2][1] = -1;
	CWT_coef_dw_mat[2][2] = 0;
	CWT_coef_dw_mat[2][3] = 4;
	CWT_coef_dw_mat[2][4] = 5;
	CWT_coef_dw_mat[2][5] = -3;
	CWT_coef_dw_mat[2][6] = -17;
	CWT_coef_dw_mat[2][7] = -16;
	CWT_coef_dw_mat[2][8] = 10;
	CWT_coef_dw_mat[2][9] = 47;
	CWT_coef_dw_mat[2][10] = 38;
	CWT_coef_dw_mat[2][11] = -26;
	CWT_coef_dw_mat[2][12] = -89;
	CWT_coef_dw_mat[2][13] = -57;
	CWT_coef_dw_mat[2][14] = 43;
	CWT_coef_dw_mat[2][15] = 111;
	CWT_coef_dw_mat[2][16] = 58;
	CWT_coef_dw_mat[2][17] = -45;
	CWT_coef_dw_mat[2][18] = -92;
	CWT_coef_dw_mat[2][19] = -38;
	CWT_coef_dw_mat[2][20] = 31;
	CWT_coef_dw_mat[2][21] = 50;
	CWT_coef_dw_mat[2][22] = 17;
	CWT_coef_dw_mat[2][23] = -14;
	CWT_coef_dw_mat[2][24] = -18;
	CWT_coef_dw_mat[2][25] = -5;
	CWT_coef_dw_mat[2][26] = 4;
	CWT_coef_dw_mat[2][27] = 4;
	CWT_coef_dw_mat[2][28] = 1;
	CWT_coef_dw_mat[2][29] = -1;
	CWT_coef_dw_mat[3][0] = -1;
	CWT_coef_dw_mat[3][1] = -1;
	CWT_coef_dw_mat[3][2] = 1;
	CWT_coef_dw_mat[3][3] = 5;
	CWT_coef_dw_mat[3][4] = 4;
	CWT_coef_dw_mat[3][5] = -7;
	CWT_coef_dw_mat[3][6] = -18;
	CWT_coef_dw_mat[3][7] = -13;
	CWT_coef_dw_mat[3][8] = 19;
	CWT_coef_dw_mat[3][9] = 51;
	CWT_coef_dw_mat[3][10] = 27;
	CWT_coef_dw_mat[3][11] = -44;
	CWT_coef_dw_mat[3][12] = -93;
	CWT_coef_dw_mat[3][13] = -39;
	CWT_coef_dw_mat[3][14] = 65;
	CWT_coef_dw_mat[3][15] = 111;
	CWT_coef_dw_mat[3][16] = 36;
	CWT_coef_dw_mat[3][17] = -64;
	CWT_coef_dw_mat[3][18] = -83;
	CWT_coef_dw_mat[3][19] = -26;
	CWT_coef_dw_mat[3][20] = 42;
	CWT_coef_dw_mat[3][21] = 44;
	CWT_coef_dw_mat[3][22] = 10;
	CWT_coef_dw_mat[3][23] = -16;
	CWT_coef_dw_mat[3][24] = -17;
	CWT_coef_dw_mat[3][25] = -3;
	CWT_coef_dw_mat[3][26] = 5;
	CWT_coef_dw_mat[3][27] = 4;
	CWT_coef_dw_mat[3][28] = 0;
	CWT_coef_dw_mat[3][29] = -1;
	CWT_coef_dw_mat[4][0] = -1;
	CWT_coef_dw_mat[4][1] = 0;
	CWT_coef_dw_mat[4][2] = 2;
	CWT_coef_dw_mat[4][3] = 5;
	CWT_coef_dw_mat[4][4] = 2;
	CWT_coef_dw_mat[4][5] = -10;
	CWT_coef_dw_mat[4][6] = -20;
	CWT_coef_dw_mat[4][7] = -6;
	CWT_coef_dw_mat[4][8] = 31;
	CWT_coef_dw_mat[4][9] = 52;
	CWT_coef_dw_mat[4][10] = 10;
	CWT_coef_dw_mat[4][11] = -65;
	CWT_coef_dw_mat[4][12] = -84;
	CWT_coef_dw_mat[4][13] = -15;
	CWT_coef_dw_mat[4][14] = 82;
	CWT_coef_dw_mat[4][15] = 104;
	CWT_coef_dw_mat[4][16] = 13;
	CWT_coef_dw_mat[4][17] = -79;
	CWT_coef_dw_mat[4][18] = -75;
	CWT_coef_dw_mat[4][19] = -9;
	CWT_coef_dw_mat[4][20] = 45;
	CWT_coef_dw_mat[4][21] = 42;
	CWT_coef_dw_mat[4][22] = 3;
	CWT_coef_dw_mat[4][23] = -19;
	CWT_coef_dw_mat[4][24] = -14;
	CWT_coef_dw_mat[4][25] = -1;
	CWT_coef_dw_mat[4][26] = 5;
	CWT_coef_dw_mat[4][27] = 3;
	CWT_coef_dw_mat[4][28] = 0;
	CWT_coef_dw_mat[4][29] = -1;
	CWT_coef_dw_mat[5][0] = -1;
	CWT_coef_dw_mat[5][1] = -1;
	CWT_coef_dw_mat[5][2] = 0;
	CWT_coef_dw_mat[5][3] = 3;
	CWT_coef_dw_mat[5][4] = 5;
	CWT_coef_dw_mat[5][5] = -1;
	CWT_coef_dw_mat[5][6] = -14;
	CWT_coef_dw_mat[5][7] = -19;
	CWT_coef_dw_mat[5][8] = 3;
	CWT_coef_dw_mat[5][9] = 42;
	CWT_coef_dw_mat[5][10] = 45;
	CWT_coef_dw_mat[5][11] = -5;
	CWT_coef_dw_mat[5][12] = -79;
	CWT_coef_dw_mat[5][13] = -76;
	CWT_coef_dw_mat[5][14] = 10;
	CWT_coef_dw_mat[5][15] = 95;
	CWT_coef_dw_mat[5][16] = 91;
	CWT_coef_dw_mat[5][17] = -10;
	CWT_coef_dw_mat[5][18] = -82;
	CWT_coef_dw_mat[5][19] = -70;
	CWT_coef_dw_mat[5][20] = 5;
	CWT_coef_dw_mat[5][21] = 51;
	CWT_coef_dw_mat[5][22] = 33;
	CWT_coef_dw_mat[5][23] = -3;
	CWT_coef_dw_mat[5][24] = -19;
	CWT_coef_dw_mat[5][25] = -12;
	CWT_coef_dw_mat[5][26] = 1;
	CWT_coef_dw_mat[5][27] = 5;
	CWT_coef_dw_mat[5][28] = 3;
	CWT_coef_dw_mat[5][29] = 0;
	CWT_coef_dw_mat[6][0] = -1;
	CWT_coef_dw_mat[6][1] = -1;
	CWT_coef_dw_mat[6][2] = 1;
	CWT_coef_dw_mat[6][3] = 4;
	CWT_coef_dw_mat[6][4] = 4;
	CWT_coef_dw_mat[6][5] = -5;
	CWT_coef_dw_mat[6][6] = -17;
	CWT_coef_dw_mat[6][7] = -15;
	CWT_coef_dw_mat[6][8] = 14;
	CWT_coef_dw_mat[6][9] = 46;
	CWT_coef_dw_mat[6][10] = 38;
	CWT_coef_dw_mat[6][11] = -27;
	CWT_coef_dw_mat[6][12] = -84;
	CWT_coef_dw_mat[6][13] = -63;
	CWT_coef_dw_mat[6][14] = 35;
	CWT_coef_dw_mat[6][15] = 104;
	CWT_coef_dw_mat[6][16] = 73;
	CWT_coef_dw_mat[6][17] = -33;
	CWT_coef_dw_mat[6][18] = -87;
	CWT_coef_dw_mat[6][19] = -56;
	CWT_coef_dw_mat[6][20] = 20;
	CWT_coef_dw_mat[6][21] = 49;
	CWT_coef_dw_mat[6][22] = 29;
	CWT_coef_dw_mat[6][23] = -8;
	CWT_coef_dw_mat[6][24] = -19;
	CWT_coef_dw_mat[6][25] = -10;
	CWT_coef_dw_mat[6][26] = 2;
	CWT_coef_dw_mat[6][27] = 5;
	CWT_coef_dw_mat[6][28] = 2;
	CWT_coef_dw_mat[6][29] = 0;
	CWT_coef_dw_mat[7][0] = -1;
	CWT_coef_dw_mat[7][1] = -1;
	CWT_coef_dw_mat[7][2] = 2;
	CWT_coef_dw_mat[7][3] = 5;
	CWT_coef_dw_mat[7][4] = 3;
	CWT_coef_dw_mat[7][5] = -9;
	CWT_coef_dw_mat[7][6] = -19;
	CWT_coef_dw_mat[7][7] = -10;
	CWT_coef_dw_mat[7][8] = 22;
	CWT_coef_dw_mat[7][9] = 53;
	CWT_coef_dw_mat[7][10] = 24;
	CWT_coef_dw_mat[7][11] = -44;
	CWT_coef_dw_mat[7][12] = -87;
	CWT_coef_dw_mat[7][13] = -46;
	CWT_coef_dw_mat[7][14] = 58;
	CWT_coef_dw_mat[7][15] = 106;
	CWT_coef_dw_mat[7][16] = 51;
	CWT_coef_dw_mat[7][17] = -52;
	CWT_coef_dw_mat[7][18] = -87;
	CWT_coef_dw_mat[7][19] = -39;
	CWT_coef_dw_mat[7][20] = 32;
	CWT_coef_dw_mat[7][21] = 48;
	CWT_coef_dw_mat[7][22] = 20;
	CWT_coef_dw_mat[7][23] = -11;
	CWT_coef_dw_mat[7][24] = -20;
	CWT_coef_dw_mat[7][25] = -7;
	CWT_coef_dw_mat[7][26] = 3;
	CWT_coef_dw_mat[7][27] = 5;
	CWT_coef_dw_mat[7][28] = 2;
	CWT_coef_dw_mat[7][29] = -1;
	CWT_coef_dw_mat[8][0] = -1;
	CWT_coef_dw_mat[8][1] = 0;
	CWT_coef_dw_mat[8][2] = 3;
	CWT_coef_dw_mat[8][3] = 5;
	CWT_coef_dw_mat[8][4] = 0;
	CWT_coef_dw_mat[8][5] = -12;
	CWT_coef_dw_mat[8][6] = -20;
	CWT_coef_dw_mat[8][7] = -2;
	CWT_coef_dw_mat[8][8] = 33;
	CWT_coef_dw_mat[8][9] = 49;
	CWT_coef_dw_mat[8][10] = 10;
	CWT_coef_dw_mat[8][11] = -66;
	CWT_coef_dw_mat[8][12] = -85;
	CWT_coef_dw_mat[8][13] = -18;
	CWT_coef_dw_mat[8][14] = 78;
	CWT_coef_dw_mat[8][15] = 102;
	CWT_coef_dw_mat[8][16] = 29;
	CWT_coef_dw_mat[8][17] = -71;
	CWT_coef_dw_mat[8][18] = -82;
	CWT_coef_dw_mat[8][19] = -22;
	CWT_coef_dw_mat[8][20] = 38;
	CWT_coef_dw_mat[8][21] = 46;
	CWT_coef_dw_mat[8][22] = 14;
	CWT_coef_dw_mat[8][23] = -15;
	CWT_coef_dw_mat[8][24] = -17;
	CWT_coef_dw_mat[8][25] = -5;
	CWT_coef_dw_mat[8][26] = 4;
	CWT_coef_dw_mat[8][27] = 5;
	CWT_coef_dw_mat[8][28] = 1;
	CWT_coef_dw_mat[8][29] = -1;
	CWT_coef_dw_mat[9][0] = -1;
	CWT_coef_dw_mat[9][1] = -1;
	CWT_coef_dw_mat[9][2] = 1;
	CWT_coef_dw_mat[9][3] = 4;
	CWT_coef_dw_mat[9][4] = 5;
	CWT_coef_dw_mat[9][5] = -3;
	CWT_coef_dw_mat[9][6] = -16;
	CWT_coef_dw_mat[9][7] = -17;
	CWT_coef_dw_mat[9][8] = 6;
	CWT_coef_dw_mat[9][9] = 45;
	CWT_coef_dw_mat[9][10] = 43;
	CWT_coef_dw_mat[9][11] = -11;
	CWT_coef_dw_mat[9][12] = -76;
	CWT_coef_dw_mat[9][13] = -77;
	CWT_coef_dw_mat[9][14] = 5;
	CWT_coef_dw_mat[9][15] = 93;
	CWT_coef_dw_mat[9][16] = 93;
	CWT_coef_dw_mat[9][17] = 5;
	CWT_coef_dw_mat[9][18] = -84;
	CWT_coef_dw_mat[9][19] = -73;
	CWT_coef_dw_mat[9][20] = -5;
	CWT_coef_dw_mat[9][21] = 45;
	CWT_coef_dw_mat[9][22] = 41;
	CWT_coef_dw_mat[9][23] = 6;
	CWT_coef_dw_mat[9][24] = -17;
	CWT_coef_dw_mat[9][25] = -16;
	CWT_coef_dw_mat[9][26] = -3;
	CWT_coef_dw_mat[9][27] = 5;
	CWT_coef_dw_mat[9][28] = 4;
	CWT_coef_dw_mat[9][29] = 1;
	CWT_coef_dw_mat[10][0] = -1;
	CWT_coef_dw_mat[10][1] = -1;
	CWT_coef_dw_mat[10][2] = 2;
	CWT_coef_dw_mat[10][3] = 5;
	CWT_coef_dw_mat[10][4] = 3;
	CWT_coef_dw_mat[10][5] = -7;
	CWT_coef_dw_mat[10][6] = -18;
	CWT_coef_dw_mat[10][7] = -13;
	CWT_coef_dw_mat[10][8] = 17;
	CWT_coef_dw_mat[10][9] = 48;
	CWT_coef_dw_mat[10][10] = 35;
	CWT_coef_dw_mat[10][11] = -28;
	CWT_coef_dw_mat[10][12] = -85;
	CWT_coef_dw_mat[10][13] = -64;
	CWT_coef_dw_mat[10][14] = 29;
	CWT_coef_dw_mat[10][15] = 103;
	CWT_coef_dw_mat[10][16] = 78;
	CWT_coef_dw_mat[10][17] = -18;
	CWT_coef_dw_mat[10][18] = -85;
	CWT_coef_dw_mat[10][19] = -65;
	CWT_coef_dw_mat[10][20] = 8;
	CWT_coef_dw_mat[10][21] = 49;
	CWT_coef_dw_mat[10][22] = 34;
	CWT_coef_dw_mat[10][23] = -2;
	CWT_coef_dw_mat[10][24] = -19;
	CWT_coef_dw_mat[10][25] = -13;
	CWT_coef_dw_mat[10][26] = 0;
	CWT_coef_dw_mat[10][27] = 5;
	CWT_coef_dw_mat[10][28] = 3;
	CWT_coef_dw_mat[10][29] = 0;
	CWT_coef_dw_mat[11][0] = -1;
	CWT_coef_dw_mat[11][1] = -1;
	CWT_coef_dw_mat[11][2] = 1;
	CWT_coef_dw_mat[11][3] = 4;
	CWT_coef_dw_mat[11][4] = 4;
	CWT_coef_dw_mat[11][5] = -5;
	CWT_coef_dw_mat[11][6] = -18;
	CWT_coef_dw_mat[11][7] = -14;
	CWT_coef_dw_mat[11][8] = 19;
	CWT_coef_dw_mat[11][9] = 51;
	CWT_coef_dw_mat[11][10] = 27;
	CWT_coef_dw_mat[11][11] = -49;
	CWT_coef_dw_mat[11][12] = -91;
	CWT_coef_dw_mat[11][13] = -30;
	CWT_coef_dw_mat[11][14] = 78;
	CWT_coef_dw_mat[11][15] = 105;
	CWT_coef_dw_mat[11][16] = 18;
	CWT_coef_dw_mat[11][17] = -71;
	CWT_coef_dw_mat[11][18] = -81;
	CWT_coef_dw_mat[11][19] = -8;
	CWT_coef_dw_mat[11][20] = 48;
	CWT_coef_dw_mat[11][21] = 39;
	CWT_coef_dw_mat[11][22] = 0;
	CWT_coef_dw_mat[11][23] = -20;
	CWT_coef_dw_mat[11][24] = -12;
	CWT_coef_dw_mat[11][25] = 1;
	CWT_coef_dw_mat[11][26] = 5;
	CWT_coef_dw_mat[11][27] = 2;
	CWT_coef_dw_mat[11][28] = 0;
	CWT_coef_dw_mat[11][29] = -1;
	CWT_coef_dw_mat[12][0] = -1;
	CWT_coef_dw_mat[12][1] = -1;
	CWT_coef_dw_mat[12][2] = 2;
	CWT_coef_dw_mat[12][3] = 5;
	CWT_coef_dw_mat[12][4] = 3;
	CWT_coef_dw_mat[12][5] = -9;
	CWT_coef_dw_mat[12][6] = -20;
	CWT_coef_dw_mat[12][7] = -7;
	CWT_coef_dw_mat[12][8] = 31;
	CWT_coef_dw_mat[12][9] = 51;
	CWT_coef_dw_mat[12][10] = 10;
	CWT_coef_dw_mat[12][11] = -65;
	CWT_coef_dw_mat[12][12] = -87;
	CWT_coef_dw_mat[12][13] = -7;
	CWT_coef_dw_mat[12][14] = 94;
	CWT_coef_dw_mat[12][15] = 94;
	CWT_coef_dw_mat[12][16] = -3;
	CWT_coef_dw_mat[12][17] = -85;
	CWT_coef_dw_mat[12][18] = -69;
	CWT_coef_dw_mat[12][19] = 8;
	CWT_coef_dw_mat[12][20] = 51;
	CWT_coef_dw_mat[12][21] = 31;
	CWT_coef_dw_mat[12][22] = -6;
	CWT_coef_dw_mat[12][23] = -20;
	CWT_coef_dw_mat[12][24] = -10;
	CWT_coef_dw_mat[12][25] = 3;
	CWT_coef_dw_mat[12][26] = 5;
	CWT_coef_dw_mat[12][27] = 2;
	CWT_coef_dw_mat[12][28] = -1;
	CWT_coef_dw_mat[12][29] = -1;
	CWT_coef_dw_mat[13][0] = 0;
	CWT_coef_dw_mat[13][1] = -1;
	CWT_coef_dw_mat[13][2] = 0;
	CWT_coef_dw_mat[13][3] = 3;
	CWT_coef_dw_mat[13][4] = 5;
	CWT_coef_dw_mat[13][5] = 1;
	CWT_coef_dw_mat[13][6] = -13;
	CWT_coef_dw_mat[13][7] = -20;
	CWT_coef_dw_mat[13][8] = 1;
	CWT_coef_dw_mat[13][9] = 39;
	CWT_coef_dw_mat[13][10] = 48;
	CWT_coef_dw_mat[13][11] = -9;
	CWT_coef_dw_mat[13][12] = -74;
	CWT_coef_dw_mat[13][13] = -78;
	CWT_coef_dw_mat[13][14] = 18;
	CWT_coef_dw_mat[13][15] = 106;
	CWT_coef_dw_mat[13][16] = 76;
	CWT_coef_dw_mat[13][17] = -23;
	CWT_coef_dw_mat[13][18] = -91;
	CWT_coef_dw_mat[13][19] = -55;
	CWT_coef_dw_mat[13][20] = 19;
	CWT_coef_dw_mat[13][21] = 52;
	CWT_coef_dw_mat[13][22] = 25;
	CWT_coef_dw_mat[13][23] = -11;
	CWT_coef_dw_mat[13][24] = -18;
	CWT_coef_dw_mat[13][25] = -8;
	CWT_coef_dw_mat[13][26] = 4;
	CWT_coef_dw_mat[13][27] = 5;
	CWT_coef_dw_mat[13][28] = 1;
	CWT_coef_dw_mat[13][29] = -1;
	CWT_coef_dw_mat[14][0] = -1;
	CWT_coef_dw_mat[14][1] = -1;
	CWT_coef_dw_mat[14][2] = 0;
	CWT_coef_dw_mat[14][3] = 4;
	CWT_coef_dw_mat[14][4] = 5;
	CWT_coef_dw_mat[14][5] = -3;
	CWT_coef_dw_mat[14][6] = -17;
	CWT_coef_dw_mat[14][7] = -16;
	CWT_coef_dw_mat[14][8] = 10;
	CWT_coef_dw_mat[14][9] = 47;
	CWT_coef_dw_mat[14][10] = 38;
	CWT_coef_dw_mat[14][11] = -26;
	CWT_coef_dw_mat[14][12] = -89;
	CWT_coef_dw_mat[14][13] = -57;
	CWT_coef_dw_mat[14][14] = 43;
	CWT_coef_dw_mat[14][15] = 111;
	CWT_coef_dw_mat[14][16] = 58;
	CWT_coef_dw_mat[14][17] = -45;
	CWT_coef_dw_mat[14][18] = -92;
	CWT_coef_dw_mat[14][19] = -38;
	CWT_coef_dw_mat[14][20] = 31;
	CWT_coef_dw_mat[14][21] = 50;
	CWT_coef_dw_mat[14][22] = 17;
	CWT_coef_dw_mat[14][23] = -14;
	CWT_coef_dw_mat[14][24] = -18;
	CWT_coef_dw_mat[14][25] = -5;
	CWT_coef_dw_mat[14][26] = 4;
	CWT_coef_dw_mat[14][27] = 4;
	CWT_coef_dw_mat[14][28] = 1;
	CWT_coef_dw_mat[14][29] = -1;
	CWT_coef_dw_mat[15][0] = -1;
	CWT_coef_dw_mat[15][1] = -1;
	CWT_coef_dw_mat[15][2] = 1;
	CWT_coef_dw_mat[15][3] = 5;
	CWT_coef_dw_mat[15][4] = 4;
	CWT_coef_dw_mat[15][5] = -7;
	CWT_coef_dw_mat[15][6] = -18;
	CWT_coef_dw_mat[15][7] = -13;
	CWT_coef_dw_mat[15][8] = 19;
	CWT_coef_dw_mat[15][9] = 51;
	CWT_coef_dw_mat[15][10] = 27;
	CWT_coef_dw_mat[15][11] = -44;
	CWT_coef_dw_mat[15][12] = -93;
	CWT_coef_dw_mat[15][13] = -39;
	CWT_coef_dw_mat[15][14] = 65;
	CWT_coef_dw_mat[15][15] = 111;
	CWT_coef_dw_mat[15][16] = 36;
	CWT_coef_dw_mat[15][17] = -64;
	CWT_coef_dw_mat[15][18] = -83;
	CWT_coef_dw_mat[15][19] = -26;
	CWT_coef_dw_mat[15][20] = 42;
	CWT_coef_dw_mat[15][21] = 44;
	CWT_coef_dw_mat[15][22] = 10;
	CWT_coef_dw_mat[15][23] = -16;
	CWT_coef_dw_mat[15][24] = -17;
	CWT_coef_dw_mat[15][25] = -3;
	CWT_coef_dw_mat[15][26] = 5;
	CWT_coef_dw_mat[15][27] = 4;
	CWT_coef_dw_mat[15][28] = 0;
	CWT_coef_dw_mat[15][29] = -1;
	CWT_coef_dw_mat[16][0] = -1;
	CWT_coef_dw_mat[16][1] = 0;
	CWT_coef_dw_mat[16][2] = 2;
	CWT_coef_dw_mat[16][3] = 5;
	CWT_coef_dw_mat[16][4] = 2;
	CWT_coef_dw_mat[16][5] = -10;
	CWT_coef_dw_mat[16][6] = -20;
	CWT_coef_dw_mat[16][7] = -6;
	CWT_coef_dw_mat[16][8] = 31;
	CWT_coef_dw_mat[16][9] = 52;
	CWT_coef_dw_mat[16][10] = 10;
	CWT_coef_dw_mat[16][11] = -65;
	CWT_coef_dw_mat[16][12] = -84;
	CWT_coef_dw_mat[16][13] = -15;
	CWT_coef_dw_mat[16][14] = 82;
	CWT_coef_dw_mat[16][15] = 104;
	CWT_coef_dw_mat[16][16] = 13;
	CWT_coef_dw_mat[16][17] = -79;
	CWT_coef_dw_mat[16][18] = -75;
	CWT_coef_dw_mat[16][19] = -9;
	CWT_coef_dw_mat[16][20] = 45;
	CWT_coef_dw_mat[16][21] = 42;
	CWT_coef_dw_mat[16][22] = 3;
	CWT_coef_dw_mat[16][23] = -19;
	CWT_coef_dw_mat[16][24] = -14;
	CWT_coef_dw_mat[16][25] = -1;
	CWT_coef_dw_mat[16][26] = 5;
	CWT_coef_dw_mat[16][27] = 3;
	CWT_coef_dw_mat[16][28] = 0;
	CWT_coef_dw_mat[16][29] = -1;
	CWT_coef_dw_mat[17][0] = -1;
	CWT_coef_dw_mat[17][1] = -1;
	CWT_coef_dw_mat[17][2] = 0;
	CWT_coef_dw_mat[17][3] = 3;
	CWT_coef_dw_mat[17][4] = 5;
	CWT_coef_dw_mat[17][5] = -1;
	CWT_coef_dw_mat[17][6] = -14;
	CWT_coef_dw_mat[17][7] = -19;
	CWT_coef_dw_mat[17][8] = 3;
	CWT_coef_dw_mat[17][9] = 42;
	CWT_coef_dw_mat[17][10] = 45;
	CWT_coef_dw_mat[17][11] = -5;
	CWT_coef_dw_mat[17][12] = -79;
	CWT_coef_dw_mat[17][13] = -76;
	CWT_coef_dw_mat[17][14] = 10;
	CWT_coef_dw_mat[17][15] = 95;
	CWT_coef_dw_mat[17][16] = 91;
	CWT_coef_dw_mat[17][17] = -10;
	CWT_coef_dw_mat[17][18] = -82;
	CWT_coef_dw_mat[17][19] = -70;
	CWT_coef_dw_mat[17][20] = 5;
	CWT_coef_dw_mat[17][21] = 51;
	CWT_coef_dw_mat[17][22] = 33;
	CWT_coef_dw_mat[17][23] = -3;
	CWT_coef_dw_mat[17][24] = -19;
	CWT_coef_dw_mat[17][25] = -12;
	CWT_coef_dw_mat[17][26] = 1;
	CWT_coef_dw_mat[17][27] = 5;
	CWT_coef_dw_mat[17][28] = 3;
	CWT_coef_dw_mat[17][29] = 0;
	CWT_coef_dw_mat[18][0] = -1;
	CWT_coef_dw_mat[18][1] = -1;
	CWT_coef_dw_mat[18][2] = 1;
	CWT_coef_dw_mat[18][3] = 4;
	CWT_coef_dw_mat[18][4] = 4;
	CWT_coef_dw_mat[18][5] = -5;
	CWT_coef_dw_mat[18][6] = -17;
	CWT_coef_dw_mat[18][7] = -15;
	CWT_coef_dw_mat[18][8] = 14;
	CWT_coef_dw_mat[18][9] = 46;
	CWT_coef_dw_mat[18][10] = 38;
	CWT_coef_dw_mat[18][11] = -27;
	CWT_coef_dw_mat[18][12] = -84;
	CWT_coef_dw_mat[18][13] = -63;
	CWT_coef_dw_mat[18][14] = 35;
	CWT_coef_dw_mat[18][15] = 104;
	CWT_coef_dw_mat[18][16] = 73;
	CWT_coef_dw_mat[18][17] = -33;
	CWT_coef_dw_mat[18][18] = -87;
	CWT_coef_dw_mat[18][19] = -56;
	CWT_coef_dw_mat[18][20] = 20;
	CWT_coef_dw_mat[18][21] = 49;
	CWT_coef_dw_mat[18][22] = 29;
	CWT_coef_dw_mat[18][23] = -8;
	CWT_coef_dw_mat[18][24] = -19;
	CWT_coef_dw_mat[18][25] = -10;
	CWT_coef_dw_mat[18][26] = 2;
	CWT_coef_dw_mat[18][27] = 5;
	CWT_coef_dw_mat[18][28] = 2;
	CWT_coef_dw_mat[18][29] = 0;
	CWT_coef_dw_mat[19][0] = -1;
	CWT_coef_dw_mat[19][1] = -1;
	CWT_coef_dw_mat[19][2] = 2;
	CWT_coef_dw_mat[19][3] = 5;
	CWT_coef_dw_mat[19][4] = 3;
	CWT_coef_dw_mat[19][5] = -9;
	CWT_coef_dw_mat[19][6] = -19;
	CWT_coef_dw_mat[19][7] = -10;
	CWT_coef_dw_mat[19][8] = 22;
	CWT_coef_dw_mat[19][9] = 53;
	CWT_coef_dw_mat[19][10] = 24;
	CWT_coef_dw_mat[19][11] = -44;
	CWT_coef_dw_mat[19][12] = -87;
	CWT_coef_dw_mat[19][13] = -46;
	CWT_coef_dw_mat[19][14] = 58;
	CWT_coef_dw_mat[19][15] = 106;
	CWT_coef_dw_mat[19][16] = 51;
	CWT_coef_dw_mat[19][17] = -52;
	CWT_coef_dw_mat[19][18] = -87;
	CWT_coef_dw_mat[19][19] = -39;
	CWT_coef_dw_mat[19][20] = 32;
	CWT_coef_dw_mat[19][21] = 48;
	CWT_coef_dw_mat[19][22] = 20;
	CWT_coef_dw_mat[19][23] = -11;
	CWT_coef_dw_mat[19][24] = -20;
	CWT_coef_dw_mat[19][25] = -7;
	CWT_coef_dw_mat[19][26] = 3;
	CWT_coef_dw_mat[19][27] = 5;
	CWT_coef_dw_mat[19][28] = 2;
	CWT_coef_dw_mat[19][29] = -1;
	CWT_coef_dw_mat[20][0] = -1;
	CWT_coef_dw_mat[20][1] = 0;
	CWT_coef_dw_mat[20][2] = 3;
	CWT_coef_dw_mat[20][3] = 5;
	CWT_coef_dw_mat[20][4] = 0;
	CWT_coef_dw_mat[20][5] = -12;
	CWT_coef_dw_mat[20][6] = -20;
	CWT_coef_dw_mat[20][7] = -2;
	CWT_coef_dw_mat[20][8] = 33;
	CWT_coef_dw_mat[20][9] = 49;
	CWT_coef_dw_mat[20][10] = 10;
	CWT_coef_dw_mat[20][11] = -66;
	CWT_coef_dw_mat[20][12] = -85;
	CWT_coef_dw_mat[20][13] = -18;
	CWT_coef_dw_mat[20][14] = 78;
	CWT_coef_dw_mat[20][15] = 102;
	CWT_coef_dw_mat[20][16] = 29;
	CWT_coef_dw_mat[20][17] = -71;
	CWT_coef_dw_mat[20][18] = -82;
	CWT_coef_dw_mat[20][19] = -22;
	CWT_coef_dw_mat[20][20] = 38;
	CWT_coef_dw_mat[20][21] = 46;
	CWT_coef_dw_mat[20][22] = 14;
	CWT_coef_dw_mat[20][23] = -15;
	CWT_coef_dw_mat[20][24] = -17;
	CWT_coef_dw_mat[20][25] = -5;
	CWT_coef_dw_mat[20][26] = 4;
	CWT_coef_dw_mat[20][27] = 5;
	CWT_coef_dw_mat[20][28] = 1;
	CWT_coef_dw_mat[20][29] = -1;
	CWT_coef_dw_mat[21][0] = -1;
	CWT_coef_dw_mat[21][1] = -1;
	CWT_coef_dw_mat[21][2] = 1;
	CWT_coef_dw_mat[21][3] = 4;
	CWT_coef_dw_mat[21][4] = 5;
	CWT_coef_dw_mat[21][5] = -3;
	CWT_coef_dw_mat[21][6] = -16;
	CWT_coef_dw_mat[21][7] = -17;
	CWT_coef_dw_mat[21][8] = 6;
	CWT_coef_dw_mat[21][9] = 45;
	CWT_coef_dw_mat[21][10] = 43;
	CWT_coef_dw_mat[21][11] = -11;
	CWT_coef_dw_mat[21][12] = -76;
	CWT_coef_dw_mat[21][13] = -77;
	CWT_coef_dw_mat[21][14] = 5;
	CWT_coef_dw_mat[21][15] = 93;
	CWT_coef_dw_mat[21][16] = 93;
	CWT_coef_dw_mat[21][17] = 5;
	CWT_coef_dw_mat[21][18] = -84;
	CWT_coef_dw_mat[21][19] = -73;
	CWT_coef_dw_mat[21][20] = -5;
	CWT_coef_dw_mat[21][21] = 45;
	CWT_coef_dw_mat[21][22] = 41;
	CWT_coef_dw_mat[21][23] = 6;
	CWT_coef_dw_mat[21][24] = -17;
	CWT_coef_dw_mat[21][25] = -16;
	CWT_coef_dw_mat[21][26] = -3;
	CWT_coef_dw_mat[21][27] = 5;
	CWT_coef_dw_mat[21][28] = 4;
	CWT_coef_dw_mat[21][29] = 1;
	CWT_coef_dw_mat[22][0] = 0;
	CWT_coef_dw_mat[22][1] = -1;
	CWT_coef_dw_mat[22][2] = 0;
	CWT_coef_dw_mat[22][3] = 3;
	CWT_coef_dw_mat[22][4] = 5;
	CWT_coef_dw_mat[22][5] = -2;
	CWT_coef_dw_mat[22][6] = -16;
	CWT_coef_dw_mat[22][7] = -18;
	CWT_coef_dw_mat[22][8] = 10;
	CWT_coef_dw_mat[22][9] = 47;
	CWT_coef_dw_mat[22][10] = 38;
	CWT_coef_dw_mat[22][11] = -32;
	CWT_coef_dw_mat[22][12] = -90;
	CWT_coef_dw_mat[22][13] = -51;
	CWT_coef_dw_mat[22][14] = 57;
	CWT_coef_dw_mat[22][15] = 110;
	CWT_coef_dw_mat[22][16] = 42;
	CWT_coef_dw_mat[22][17] = -63;
	CWT_coef_dw_mat[22][18] = -86;
	CWT_coef_dw_mat[22][19] = -20;
	CWT_coef_dw_mat[22][20] = 44;
	CWT_coef_dw_mat[22][21] = 43;
	CWT_coef_dw_mat[22][22] = 5;
	CWT_coef_dw_mat[22][23] = -19;
	CWT_coef_dw_mat[22][24] = -14;
	CWT_coef_dw_mat[22][25] = 0;
	CWT_coef_dw_mat[22][26] = 5;
	CWT_coef_dw_mat[22][27] = 3;
	CWT_coef_dw_mat[22][28] = 0;
	CWT_coef_dw_mat[22][29] = -1;
	CWT_coef_dw_mat[23][0] = -1;
	CWT_coef_dw_mat[23][1] = -1;
	CWT_coef_dw_mat[23][2] = 1;
	CWT_coef_dw_mat[23][3] = 4;
	CWT_coef_dw_mat[23][4] = 4;
	CWT_coef_dw_mat[23][5] = -5;
	CWT_coef_dw_mat[23][6] = -18;
	CWT_coef_dw_mat[23][7] = -14;
	CWT_coef_dw_mat[23][8] = 19;
	CWT_coef_dw_mat[23][9] = 51;
	CWT_coef_dw_mat[23][10] = 27;
	CWT_coef_dw_mat[23][11] = -49;
	CWT_coef_dw_mat[23][12] = -91;
	CWT_coef_dw_mat[23][13] = -30;
	CWT_coef_dw_mat[23][14] = 78;
	CWT_coef_dw_mat[23][15] = 105;
	CWT_coef_dw_mat[23][16] = 18;
	CWT_coef_dw_mat[23][17] = -71;
	CWT_coef_dw_mat[23][18] = -81;
	CWT_coef_dw_mat[23][19] = -8;
	CWT_coef_dw_mat[23][20] = 48;
	CWT_coef_dw_mat[23][21] = 39;
	CWT_coef_dw_mat[23][22] = 0;
	CWT_coef_dw_mat[23][23] = -20;
	CWT_coef_dw_mat[23][24] = -12;
	CWT_coef_dw_mat[23][25] = 1;
	CWT_coef_dw_mat[23][26] = 5;
	CWT_coef_dw_mat[23][27] = 2;
	CWT_coef_dw_mat[23][28] = 0;
	CWT_coef_dw_mat[23][29] = -1;
	CWT_coef_dw_mat[24][0] = -1;
	CWT_coef_dw_mat[24][1] = -1;
	CWT_coef_dw_mat[24][2] = 2;
	CWT_coef_dw_mat[24][3] = 5;
	CWT_coef_dw_mat[24][4] = 3;
	CWT_coef_dw_mat[24][5] = -9;
	CWT_coef_dw_mat[24][6] = -20;
	CWT_coef_dw_mat[24][7] = -7;
	CWT_coef_dw_mat[24][8] = 31;
	CWT_coef_dw_mat[24][9] = 51;
	CWT_coef_dw_mat[24][10] = 10;
	CWT_coef_dw_mat[24][11] = -65;
	CWT_coef_dw_mat[24][12] = -87;
	CWT_coef_dw_mat[24][13] = -7;
	CWT_coef_dw_mat[24][14] = 94;
	CWT_coef_dw_mat[24][15] = 94;
	CWT_coef_dw_mat[24][16] = -3;
	CWT_coef_dw_mat[24][17] = -85;
	CWT_coef_dw_mat[24][18] = -69;
	CWT_coef_dw_mat[24][19] = 8;
	CWT_coef_dw_mat[24][20] = 51;
	CWT_coef_dw_mat[24][21] = 31;
	CWT_coef_dw_mat[24][22] = -6;
	CWT_coef_dw_mat[24][23] = -20;
	CWT_coef_dw_mat[24][24] = -10;
	CWT_coef_dw_mat[24][25] = 3;
	CWT_coef_dw_mat[24][26] = 5;
	CWT_coef_dw_mat[24][27] = 2;
	CWT_coef_dw_mat[24][28] = -1;
	CWT_coef_dw_mat[24][29] = -1;
	CWT_coef_dw_mat[25][0] = 0;
	CWT_coef_dw_mat[25][1] = -1;
	CWT_coef_dw_mat[25][2] = 0;
	CWT_coef_dw_mat[25][3] = 3;
	CWT_coef_dw_mat[25][4] = 5;
	CWT_coef_dw_mat[25][5] = 1;
	CWT_coef_dw_mat[25][6] = -13;
	CWT_coef_dw_mat[25][7] = -20;
	CWT_coef_dw_mat[25][8] = 1;
	CWT_coef_dw_mat[25][9] = 39;
	CWT_coef_dw_mat[25][10] = 48;
	CWT_coef_dw_mat[25][11] = -9;
	CWT_coef_dw_mat[25][12] = -74;
	CWT_coef_dw_mat[25][13] = -78;
	CWT_coef_dw_mat[25][14] = 18;
	CWT_coef_dw_mat[25][15] = 106;
	CWT_coef_dw_mat[25][16] = 76;
	CWT_coef_dw_mat[25][17] = -23;
	CWT_coef_dw_mat[25][18] = -91;
	CWT_coef_dw_mat[25][19] = -55;
	CWT_coef_dw_mat[25][20] = 19;
	CWT_coef_dw_mat[25][21] = 52;
	CWT_coef_dw_mat[25][22] = 25;
	CWT_coef_dw_mat[25][23] = -11;
	CWT_coef_dw_mat[25][24] = -18;
	CWT_coef_dw_mat[25][25] = -8;
	CWT_coef_dw_mat[25][26] = 4;
	CWT_coef_dw_mat[25][27] = 5;
	CWT_coef_dw_mat[25][28] = 1;
	CWT_coef_dw_mat[25][29] = -1;
	CWT_coef_dw_mat[26][0] = -1;
	CWT_coef_dw_mat[26][1] = -1;
	CWT_coef_dw_mat[26][2] = 0;
	CWT_coef_dw_mat[26][3] = 4;
	CWT_coef_dw_mat[26][4] = 5;
	CWT_coef_dw_mat[26][5] = -3;
	CWT_coef_dw_mat[26][6] = -17;
	CWT_coef_dw_mat[26][7] = -16;
	CWT_coef_dw_mat[26][8] = 10;
	CWT_coef_dw_mat[26][9] = 47;
	CWT_coef_dw_mat[26][10] = 38;
	CWT_coef_dw_mat[26][11] = -26;
	CWT_coef_dw_mat[26][12] = -89;
	CWT_coef_dw_mat[26][13] = -57;
	CWT_coef_dw_mat[26][14] = 43;
	CWT_coef_dw_mat[26][15] = 111;
	CWT_coef_dw_mat[26][16] = 58;
	CWT_coef_dw_mat[26][17] = -45;
	CWT_coef_dw_mat[26][18] = -92;
	CWT_coef_dw_mat[26][19] = -38;
	CWT_coef_dw_mat[26][20] = 31;
	CWT_coef_dw_mat[26][21] = 50;
	CWT_coef_dw_mat[26][22] = 17;
	CWT_coef_dw_mat[26][23] = -14;
	CWT_coef_dw_mat[26][24] = -18;
	CWT_coef_dw_mat[26][25] = -5;
	CWT_coef_dw_mat[26][26] = 4;
	CWT_coef_dw_mat[26][27] = 4;
	CWT_coef_dw_mat[26][28] = 1;
	CWT_coef_dw_mat[26][29] = -1;
	CWT_coef_dw_mat[27][0] = -1;
	CWT_coef_dw_mat[27][1] = -1;
	CWT_coef_dw_mat[27][2] = 1;
	CWT_coef_dw_mat[27][3] = 5;
	CWT_coef_dw_mat[27][4] = 4;
	CWT_coef_dw_mat[27][5] = -7;
	CWT_coef_dw_mat[27][6] = -18;
	CWT_coef_dw_mat[27][7] = -13;
	CWT_coef_dw_mat[27][8] = 19;
	CWT_coef_dw_mat[27][9] = 51;
	CWT_coef_dw_mat[27][10] = 27;
	CWT_coef_dw_mat[27][11] = -44;
	CWT_coef_dw_mat[27][12] = -93;
	CWT_coef_dw_mat[27][13] = -39;
	CWT_coef_dw_mat[27][14] = 65;
	CWT_coef_dw_mat[27][15] = 111;
	CWT_coef_dw_mat[27][16] = 36;
	CWT_coef_dw_mat[27][17] = -64;
	CWT_coef_dw_mat[27][18] = -83;
	CWT_coef_dw_mat[27][19] = -26;
	CWT_coef_dw_mat[27][20] = 42;
	CWT_coef_dw_mat[27][21] = 44;
	CWT_coef_dw_mat[27][22] = 10;
	CWT_coef_dw_mat[27][23] = -16;
	CWT_coef_dw_mat[27][24] = -17;
	CWT_coef_dw_mat[27][25] = -3;
	CWT_coef_dw_mat[27][26] = 5;
	CWT_coef_dw_mat[27][27] = 4;
	CWT_coef_dw_mat[27][28] = 0;
	CWT_coef_dw_mat[27][29] = -1;
	CWT_coef_dw_mat[28][0] = -1;
	CWT_coef_dw_mat[28][1] = 0;
	CWT_coef_dw_mat[28][2] = 2;
	CWT_coef_dw_mat[28][3] = 5;
	CWT_coef_dw_mat[28][4] = 2;
	CWT_coef_dw_mat[28][5] = -10;
	CWT_coef_dw_mat[28][6] = -20;
	CWT_coef_dw_mat[28][7] = -6;
	CWT_coef_dw_mat[28][8] = 31;
	CWT_coef_dw_mat[28][9] = 52;
	CWT_coef_dw_mat[28][10] = 10;
	CWT_coef_dw_mat[28][11] = -65;
	CWT_coef_dw_mat[28][12] = -84;
	CWT_coef_dw_mat[28][13] = -15;
	CWT_coef_dw_mat[28][14] = 82;
	CWT_coef_dw_mat[28][15] = 104;
	CWT_coef_dw_mat[28][16] = 13;
	CWT_coef_dw_mat[28][17] = -79;
	CWT_coef_dw_mat[28][18] = -75;
	CWT_coef_dw_mat[28][19] = -9;
	CWT_coef_dw_mat[28][20] = 45;
	CWT_coef_dw_mat[28][21] = 42;
	CWT_coef_dw_mat[28][22] = 3;
	CWT_coef_dw_mat[28][23] = -19;
	CWT_coef_dw_mat[28][24] = -14;
	CWT_coef_dw_mat[28][25] = -1;
	CWT_coef_dw_mat[28][26] = 5;
	CWT_coef_dw_mat[28][27] = 3;
	CWT_coef_dw_mat[28][28] = 0;
	CWT_coef_dw_mat[28][29] = -1;
	CWT_coef_dw_mat[29][0] = -1;
	CWT_coef_dw_mat[29][1] = -1;
	CWT_coef_dw_mat[29][2] = 0;
	CWT_coef_dw_mat[29][3] = 3;
	CWT_coef_dw_mat[29][4] = 5;
	CWT_coef_dw_mat[29][5] = -1;
	CWT_coef_dw_mat[29][6] = -14;
	CWT_coef_dw_mat[29][7] = -19;
	CWT_coef_dw_mat[29][8] = 3;
	CWT_coef_dw_mat[29][9] = 42;
	CWT_coef_dw_mat[29][10] = 45;
	CWT_coef_dw_mat[29][11] = -5;
	CWT_coef_dw_mat[29][12] = -79;
	CWT_coef_dw_mat[29][13] = -76;
	CWT_coef_dw_mat[29][14] = 10;
	CWT_coef_dw_mat[29][15] = 95;
	CWT_coef_dw_mat[29][16] = 91;
	CWT_coef_dw_mat[29][17] = -10;
	CWT_coef_dw_mat[29][18] = -82;
	CWT_coef_dw_mat[29][19] = -70;
	CWT_coef_dw_mat[29][20] = 5;
	CWT_coef_dw_mat[29][21] = 51;
	CWT_coef_dw_mat[29][22] = 33;
	CWT_coef_dw_mat[29][23] = -3;
	CWT_coef_dw_mat[29][24] = -19;
	CWT_coef_dw_mat[29][25] = -12;
	CWT_coef_dw_mat[29][26] = 1;
	CWT_coef_dw_mat[29][27] = 5;
	CWT_coef_dw_mat[29][28] = 3;
	CWT_coef_dw_mat[29][29] = 0;
	CWT_coef_dw_mat[30][0] = -1;
	CWT_coef_dw_mat[30][1] = -1;
	CWT_coef_dw_mat[30][2] = 1;
	CWT_coef_dw_mat[30][3] = 4;
	CWT_coef_dw_mat[30][4] = 4;
	CWT_coef_dw_mat[30][5] = -5;
	CWT_coef_dw_mat[30][6] = -17;
	CWT_coef_dw_mat[30][7] = -15;
	CWT_coef_dw_mat[30][8] = 14;
	CWT_coef_dw_mat[30][9] = 46;
	CWT_coef_dw_mat[30][10] = 38;
	CWT_coef_dw_mat[30][11] = -27;
	CWT_coef_dw_mat[30][12] = -84;
	CWT_coef_dw_mat[30][13] = -63;
	CWT_coef_dw_mat[30][14] = 35;
	CWT_coef_dw_mat[30][15] = 104;
	CWT_coef_dw_mat[30][16] = 73;
	CWT_coef_dw_mat[30][17] = -33;
	CWT_coef_dw_mat[30][18] = -87;
	CWT_coef_dw_mat[30][19] = -56;
	CWT_coef_dw_mat[30][20] = 20;
	CWT_coef_dw_mat[30][21] = 49;
	CWT_coef_dw_mat[30][22] = 29;
	CWT_coef_dw_mat[30][23] = -8;
	CWT_coef_dw_mat[30][24] = -19;
	CWT_coef_dw_mat[30][25] = -10;
	CWT_coef_dw_mat[30][26] = 2;
	CWT_coef_dw_mat[30][27] = 5;
	CWT_coef_dw_mat[30][28] = 2;
	CWT_coef_dw_mat[30][29] = 0;
	CWT_coef_dw_mat[31][0] = -1;
	CWT_coef_dw_mat[31][1] = -1;
	CWT_coef_dw_mat[31][2] = 2;
	CWT_coef_dw_mat[31][3] = 5;
	CWT_coef_dw_mat[31][4] = 3;
	CWT_coef_dw_mat[31][5] = -9;
	CWT_coef_dw_mat[31][6] = -19;
	CWT_coef_dw_mat[31][7] = -10;
	CWT_coef_dw_mat[31][8] = 22;
	CWT_coef_dw_mat[31][9] = 53;
	CWT_coef_dw_mat[31][10] = 24;
	CWT_coef_dw_mat[31][11] = -44;
	CWT_coef_dw_mat[31][12] = -87;
	CWT_coef_dw_mat[31][13] = -46;
	CWT_coef_dw_mat[31][14] = 58;
	CWT_coef_dw_mat[31][15] = 106;
	CWT_coef_dw_mat[31][16] = 51;
	CWT_coef_dw_mat[31][17] = -52;
	CWT_coef_dw_mat[31][18] = -87;
	CWT_coef_dw_mat[31][19] = -39;
	CWT_coef_dw_mat[31][20] = 32;
	CWT_coef_dw_mat[31][21] = 48;
	CWT_coef_dw_mat[31][22] = 20;
	CWT_coef_dw_mat[31][23] = -11;
	CWT_coef_dw_mat[31][24] = -20;
	CWT_coef_dw_mat[31][25] = -7;
	CWT_coef_dw_mat[31][26] = 3;
	CWT_coef_dw_mat[31][27] = 5;
	CWT_coef_dw_mat[31][28] = 2;
	CWT_coef_dw_mat[31][29] = -1;
	CWT_coef_dw_mat[32][0] = -1;
	CWT_coef_dw_mat[32][1] = 0;
	CWT_coef_dw_mat[32][2] = 3;
	CWT_coef_dw_mat[32][3] = 5;
	CWT_coef_dw_mat[32][4] = 0;
	CWT_coef_dw_mat[32][5] = -12;
	CWT_coef_dw_mat[32][6] = -20;
	CWT_coef_dw_mat[32][7] = -2;
	CWT_coef_dw_mat[32][8] = 33;
	CWT_coef_dw_mat[32][9] = 49;
	CWT_coef_dw_mat[32][10] = 10;
	CWT_coef_dw_mat[32][11] = -66;
	CWT_coef_dw_mat[32][12] = -85;
	CWT_coef_dw_mat[32][13] = -18;
	CWT_coef_dw_mat[32][14] = 78;
	CWT_coef_dw_mat[32][15] = 102;
	CWT_coef_dw_mat[32][16] = 29;
	CWT_coef_dw_mat[32][17] = -71;
	CWT_coef_dw_mat[32][18] = -82;
	CWT_coef_dw_mat[32][19] = -22;
	CWT_coef_dw_mat[32][20] = 38;
	CWT_coef_dw_mat[32][21] = 46;
	CWT_coef_dw_mat[32][22] = 14;
	CWT_coef_dw_mat[32][23] = -15;
	CWT_coef_dw_mat[32][24] = -17;
	CWT_coef_dw_mat[32][25] = -5;
	CWT_coef_dw_mat[32][26] = 4;
	CWT_coef_dw_mat[32][27] = 5;
	CWT_coef_dw_mat[32][28] = 1;
	CWT_coef_dw_mat[32][29] = -1;



	CWT_coef_up_mat[0][0] = 0;
	CWT_coef_up_mat[0][1] = 0;
	CWT_coef_up_mat[0][2] = 0;
	CWT_coef_up_mat[0][3] = 0;
	CWT_coef_up_mat[0][4] = -1;
	CWT_coef_up_mat[0][5] = -1;
	CWT_coef_up_mat[0][6] = 3;
	CWT_coef_up_mat[0][7] = 5;
	CWT_coef_up_mat[0][8] = -10;
	CWT_coef_up_mat[0][9] = -21;
	CWT_coef_up_mat[0][10] = 18;
	CWT_coef_up_mat[0][11] = 57;
	CWT_coef_up_mat[0][12] = -21;
	CWT_coef_up_mat[0][13] = -98;
	CWT_coef_up_mat[0][14] = -2;
	CWT_coef_up_mat[0][15] = 123;
	CWT_coef_up_mat[0][16] = 28;
	CWT_coef_up_mat[0][17] = -99;
	CWT_coef_up_mat[0][18] = -40;
	CWT_coef_up_mat[0][19] = 51;
	CWT_coef_up_mat[0][20] = 31;
	CWT_coef_up_mat[0][21] = -19;
	CWT_coef_up_mat[0][22] = -13;
	CWT_coef_up_mat[0][23] = 4;
	CWT_coef_up_mat[0][24] = 4;
	CWT_coef_up_mat[0][25] = 0;
	CWT_coef_up_mat[0][26] = -1;
	CWT_coef_up_mat[0][27] = 0;
	CWT_coef_up_mat[0][28] = 0;
	CWT_coef_up_mat[0][29] = 0;
	CWT_coef_up_mat[1][0] = 0;
	CWT_coef_up_mat[1][1] = 0;
	CWT_coef_up_mat[1][2] = 0;
	CWT_coef_up_mat[1][3] = 0;
	CWT_coef_up_mat[1][4] = 0;
	CWT_coef_up_mat[1][5] = -1;
	CWT_coef_up_mat[1][6] = 2;
	CWT_coef_up_mat[1][7] = 6;
	CWT_coef_up_mat[1][8] = -6;
	CWT_coef_up_mat[1][9] = -22;
	CWT_coef_up_mat[1][10] = 11;
	CWT_coef_up_mat[1][11] = 58;
	CWT_coef_up_mat[1][12] = -4;
	CWT_coef_up_mat[1][13] = -100;
	CWT_coef_up_mat[1][14] = -20;
	CWT_coef_up_mat[1][15] = 119;
	CWT_coef_up_mat[1][16] = 45;
	CWT_coef_up_mat[1][17] = -93;
	CWT_coef_up_mat[1][18] = -53;
	CWT_coef_up_mat[1][19] = 49;
	CWT_coef_up_mat[1][20] = 34;
	CWT_coef_up_mat[1][21] = -16;
	CWT_coef_up_mat[1][22] = -15;
	CWT_coef_up_mat[1][23] = 3;
	CWT_coef_up_mat[1][24] = 5;
	CWT_coef_up_mat[1][25] = 0;
	CWT_coef_up_mat[1][26] = -1;
	CWT_coef_up_mat[1][27] = 0;
	CWT_coef_up_mat[1][28] = 0;
	CWT_coef_up_mat[1][29] = 0;
	CWT_coef_up_mat[2][0] = 0;
	CWT_coef_up_mat[2][1] = 0;
	CWT_coef_up_mat[2][2] = 0;
	CWT_coef_up_mat[2][3] = 0;
	CWT_coef_up_mat[2][4] = 0;
	CWT_coef_up_mat[2][5] = -1;
	CWT_coef_up_mat[2][6] = 1;
	CWT_coef_up_mat[2][7] = 6;
	CWT_coef_up_mat[2][8] = -4;
	CWT_coef_up_mat[2][9] = -21;
	CWT_coef_up_mat[2][10] = 2;
	CWT_coef_up_mat[2][11] = 55;
	CWT_coef_up_mat[2][12] = 11;
	CWT_coef_up_mat[2][13] = -95;
	CWT_coef_up_mat[2][14] = -37;
	CWT_coef_up_mat[2][15] = 105;
	CWT_coef_up_mat[2][16] = 68;
	CWT_coef_up_mat[2][17] = -84;
	CWT_coef_up_mat[2][18] = -65;
	CWT_coef_up_mat[2][19] = 43;
	CWT_coef_up_mat[2][20] = 39;
	CWT_coef_up_mat[2][21] = -12;
	CWT_coef_up_mat[2][22] = -17;
	CWT_coef_up_mat[2][23] = 2;
	CWT_coef_up_mat[2][24] = 5;
	CWT_coef_up_mat[2][25] = 0;
	CWT_coef_up_mat[2][26] = -1;
	CWT_coef_up_mat[2][27] = 0;
	CWT_coef_up_mat[2][28] = 0;
	CWT_coef_up_mat[2][29] = 0;
	CWT_coef_up_mat[3][0] = 0;
	CWT_coef_up_mat[3][1] = 0;
	CWT_coef_up_mat[3][2] = 0;
	CWT_coef_up_mat[3][3] = 0;
	CWT_coef_up_mat[3][4] = 0;
	CWT_coef_up_mat[3][5] = -1;
	CWT_coef_up_mat[3][6] = 1;
	CWT_coef_up_mat[3][7] = 6;
	CWT_coef_up_mat[3][8] = -1;
	CWT_coef_up_mat[3][9] = -21;
	CWT_coef_up_mat[3][10] = -3;
	CWT_coef_up_mat[3][11] = 52;
	CWT_coef_up_mat[3][12] = 22;
	CWT_coef_up_mat[3][13] = -83;
	CWT_coef_up_mat[3][14] = -60;
	CWT_coef_up_mat[3][15] = 97;
	CWT_coef_up_mat[3][16] = 83;
	CWT_coef_up_mat[3][17] = -73;
	CWT_coef_up_mat[3][18] = -75;
	CWT_coef_up_mat[3][19] = 36;
	CWT_coef_up_mat[3][20] = 45;
	CWT_coef_up_mat[3][21] = -10;
	CWT_coef_up_mat[3][22] = -18;
	CWT_coef_up_mat[3][23] = 1;
	CWT_coef_up_mat[3][24] = 5;
	CWT_coef_up_mat[3][25] = 0;
	CWT_coef_up_mat[3][26] = -1;
	CWT_coef_up_mat[3][27] = 0;
	CWT_coef_up_mat[3][28] = 0;
	CWT_coef_up_mat[3][29] = 0;
	CWT_coef_up_mat[4][0] = 0;
	CWT_coef_up_mat[4][1] = 0;
	CWT_coef_up_mat[4][2] = 0;
	CWT_coef_up_mat[4][3] = 0;
	CWT_coef_up_mat[4][4] = -1;
	CWT_coef_up_mat[4][5] = 0;
	CWT_coef_up_mat[4][6] = 5;
	CWT_coef_up_mat[4][7] = 1;
	CWT_coef_up_mat[4][8] = -18;
	CWT_coef_up_mat[4][9] = -10;
	CWT_coef_up_mat[4][10] = 45;
	CWT_coef_up_mat[4][11] = 36;
	CWT_coef_up_mat[4][12] = -75;
	CWT_coef_up_mat[4][13] = -73;
	CWT_coef_up_mat[4][14] = 83;
	CWT_coef_up_mat[4][15] = 96;
	CWT_coef_up_mat[4][16] = -60;
	CWT_coef_up_mat[4][17] = -84;
	CWT_coef_up_mat[4][18] = 26;
	CWT_coef_up_mat[4][19] = 50;
	CWT_coef_up_mat[4][20] = -6;
	CWT_coef_up_mat[4][21] = -20;
	CWT_coef_up_mat[4][22] = 0;
	CWT_coef_up_mat[4][23] = 5;
	CWT_coef_up_mat[4][24] = 1;
	CWT_coef_up_mat[4][25] = -1;
	CWT_coef_up_mat[4][26] = 0;
	CWT_coef_up_mat[4][27] = 0;
	CWT_coef_up_mat[4][28] = 0;
	CWT_coef_up_mat[4][29] = 0;
	CWT_coef_up_mat[5][0] = 0;
	CWT_coef_up_mat[5][1] = 0;
	CWT_coef_up_mat[5][2] = 0;
	CWT_coef_up_mat[5][3] = 0;
	CWT_coef_up_mat[5][4] = -1;
	CWT_coef_up_mat[5][5] = 0;
	CWT_coef_up_mat[5][6] = 4;
	CWT_coef_up_mat[5][7] = 3;
	CWT_coef_up_mat[5][8] = -15;
	CWT_coef_up_mat[5][9] = -15;
	CWT_coef_up_mat[5][10] = 40;
	CWT_coef_up_mat[5][11] = 43;
	CWT_coef_up_mat[5][12] = -64;
	CWT_coef_up_mat[5][13] = -83;
	CWT_coef_up_mat[5][14] = 68;
	CWT_coef_up_mat[5][15] = 107;
	CWT_coef_up_mat[5][16] = -45;
	CWT_coef_up_mat[5][17] = -92;
	CWT_coef_up_mat[5][18] = 16;
	CWT_coef_up_mat[5][19] = 53;
	CWT_coef_up_mat[5][20] = -1;
	CWT_coef_up_mat[5][21] = -21;
	CWT_coef_up_mat[5][22] = -2;
	CWT_coef_up_mat[5][23] = 6;
	CWT_coef_up_mat[5][24] = 1;
	CWT_coef_up_mat[5][25] = -1;
	CWT_coef_up_mat[5][26] = 0;
	CWT_coef_up_mat[5][27] = 0;
	CWT_coef_up_mat[5][28] = 0;
	CWT_coef_up_mat[5][29] = 0;
	CWT_coef_up_mat[6][0] = 0;
	CWT_coef_up_mat[6][1] = 0;
	CWT_coef_up_mat[6][2] = 0;
	CWT_coef_up_mat[6][3] = 0;
	CWT_coef_up_mat[6][4] = -1;
	CWT_coef_up_mat[6][5] = -1;
	CWT_coef_up_mat[6][6] = 4;
	CWT_coef_up_mat[6][7] = 4;
	CWT_coef_up_mat[6][8] = -13;
	CWT_coef_up_mat[6][9] = -18;
	CWT_coef_up_mat[6][10] = 30;
	CWT_coef_up_mat[6][11] = 51;
	CWT_coef_up_mat[6][12] = -46;
	CWT_coef_up_mat[6][13] = -97;
	CWT_coef_up_mat[6][14] = 51;
	CWT_coef_up_mat[6][15] = 115;
	CWT_coef_up_mat[6][16] = -29;
	CWT_coef_up_mat[6][17] = -97;
	CWT_coef_up_mat[6][18] = 5;
	CWT_coef_up_mat[6][19] = 56;
	CWT_coef_up_mat[6][20] = 4;
	CWT_coef_up_mat[6][21] = -21;
	CWT_coef_up_mat[6][22] = -3;
	CWT_coef_up_mat[6][23] = 6;
	CWT_coef_up_mat[6][24] = 1;
	CWT_coef_up_mat[6][25] = -1;
	CWT_coef_up_mat[6][26] = 0;
	CWT_coef_up_mat[6][27] = 0;
	CWT_coef_up_mat[6][28] = 0;
	CWT_coef_up_mat[6][29] = 0;
	CWT_coef_up_mat[7][0] = 0;
	CWT_coef_up_mat[7][1] = 0;
	CWT_coef_up_mat[7][2] = 0;
	CWT_coef_up_mat[7][3] = 0;
	CWT_coef_up_mat[7][4] = -1;
	CWT_coef_up_mat[7][5] = -1;
	CWT_coef_up_mat[7][6] = 3;
	CWT_coef_up_mat[7][7] = 5;
	CWT_coef_up_mat[7][8] = -9;
	CWT_coef_up_mat[7][9] = -21;
	CWT_coef_up_mat[7][10] = 23;
	CWT_coef_up_mat[7][11] = 55;
	CWT_coef_up_mat[7][12] = -33;
	CWT_coef_up_mat[7][13] = -99;
	CWT_coef_up_mat[7][14] = 31;
	CWT_coef_up_mat[7][15] = 120;
	CWT_coef_up_mat[7][16] = -11;
	CWT_coef_up_mat[7][17] = -100;
	CWT_coef_up_mat[7][18] = -8;
	CWT_coef_up_mat[7][19] = 59;
	CWT_coef_up_mat[7][20] = 8;
	CWT_coef_up_mat[7][21] = -22;
	CWT_coef_up_mat[7][22] = -5;
	CWT_coef_up_mat[7][23] = 6;
	CWT_coef_up_mat[7][24] = 1;
	CWT_coef_up_mat[7][25] = -1;
	CWT_coef_up_mat[7][26] = 0;
	CWT_coef_up_mat[7][27] = 0;
	CWT_coef_up_mat[7][28] = 0;
	CWT_coef_up_mat[7][29] = 0;
	CWT_coef_up_mat[8][0] = 0;
	CWT_coef_up_mat[8][1] = 0;
	CWT_coef_up_mat[8][2] = 0;
	CWT_coef_up_mat[8][3] = 0;
	CWT_coef_up_mat[8][4] = 0;
	CWT_coef_up_mat[8][5] = -1;
	CWT_coef_up_mat[8][6] = 2;
	CWT_coef_up_mat[8][7] = 6;
	CWT_coef_up_mat[8][8] = -7;
	CWT_coef_up_mat[8][9] = -22;
	CWT_coef_up_mat[8][10] = 14;
	CWT_coef_up_mat[8][11] = 57;
	CWT_coef_up_mat[8][12] = -18;
	CWT_coef_up_mat[8][13] = -101;
	CWT_coef_up_mat[8][14] = 7;
	CWT_coef_up_mat[8][15] = 127;
	CWT_coef_up_mat[8][16] = 7;
	CWT_coef_up_mat[8][17] = -101;
	CWT_coef_up_mat[8][18] = -18;
	CWT_coef_up_mat[8][19] = 57;
	CWT_coef_up_mat[8][20] = 14;
	CWT_coef_up_mat[8][21] = -22;
	CWT_coef_up_mat[8][22] = -7;
	CWT_coef_up_mat[8][23] = 6;
	CWT_coef_up_mat[8][24] = 2;
	CWT_coef_up_mat[8][25] = -1;
	CWT_coef_up_mat[8][26] = 0;
	CWT_coef_up_mat[8][27] = 0;
	CWT_coef_up_mat[8][28] = 0;
	CWT_coef_up_mat[8][29] = 0;
	CWT_coef_up_mat[9][0] = 0;
	CWT_coef_up_mat[9][1] = 0;
	CWT_coef_up_mat[9][2] = 0;
	CWT_coef_up_mat[9][3] = 0;
	CWT_coef_up_mat[9][4] = 0;
	CWT_coef_up_mat[9][5] = -1;
	CWT_coef_up_mat[9][6] = 1;
	CWT_coef_up_mat[9][7] = 6;
	CWT_coef_up_mat[9][8] = -4;
	CWT_coef_up_mat[9][9] = -22;
	CWT_coef_up_mat[9][10] = 7;
	CWT_coef_up_mat[9][11] = 56;
	CWT_coef_up_mat[9][12] = -1;
	CWT_coef_up_mat[9][13] = -103;
	CWT_coef_up_mat[9][14] = -11;
	CWT_coef_up_mat[9][15] = 125;
	CWT_coef_up_mat[9][16] = 25;
	CWT_coef_up_mat[9][17] = -99;
	CWT_coef_up_mat[9][18] = -31;
	CWT_coef_up_mat[9][19] = 56;
	CWT_coef_up_mat[9][20] = 20;
	CWT_coef_up_mat[9][21] = -22;
	CWT_coef_up_mat[9][22] = -8;
	CWT_coef_up_mat[9][23] = 6;
	CWT_coef_up_mat[9][24] = 2;
	CWT_coef_up_mat[9][25] = -1;
	CWT_coef_up_mat[9][26] = 0;
	CWT_coef_up_mat[9][27] = 0;
	CWT_coef_up_mat[9][28] = 0;
	CWT_coef_up_mat[9][29] = 0;
	CWT_coef_up_mat[10][0] = 0;
	CWT_coef_up_mat[10][1] = 0;
	CWT_coef_up_mat[10][2] = 0;
	CWT_coef_up_mat[10][3] = 0;
	CWT_coef_up_mat[10][4] = 0;
	CWT_coef_up_mat[10][5] = -1;
	CWT_coef_up_mat[10][6] = 0;
	CWT_coef_up_mat[10][7] = 6;
	CWT_coef_up_mat[10][8] = -1;
	CWT_coef_up_mat[10][9] = -21;
	CWT_coef_up_mat[10][10] = -1;
	CWT_coef_up_mat[10][11] = 56;
	CWT_coef_up_mat[10][12] = 10;
	CWT_coef_up_mat[10][13] = -99;
	CWT_coef_up_mat[10][14] = -28;
	CWT_coef_up_mat[10][15] = 114;
	CWT_coef_up_mat[10][16] = 49;
	CWT_coef_up_mat[10][17] = -94;
	CWT_coef_up_mat[10][18] = -45;
	CWT_coef_up_mat[10][19] = 53;
	CWT_coef_up_mat[10][20] = 27;
	CWT_coef_up_mat[10][21] = -20;
	CWT_coef_up_mat[10][22] = -11;
	CWT_coef_up_mat[10][23] = 5;
	CWT_coef_up_mat[10][24] = 3;
	CWT_coef_up_mat[10][25] = -1;
	CWT_coef_up_mat[10][26] = -1;
	CWT_coef_up_mat[10][27] = 0;
	CWT_coef_up_mat[10][28] = 0;
	CWT_coef_up_mat[10][29] = 0;
	CWT_coef_up_mat[11][0] = 0;
	CWT_coef_up_mat[11][1] = 0;
	CWT_coef_up_mat[11][2] = 0;
	CWT_coef_up_mat[11][3] = 0;
	CWT_coef_up_mat[11][4] = 0;
	CWT_coef_up_mat[11][5] = -1;
	CWT_coef_up_mat[11][6] = 2;
	CWT_coef_up_mat[11][7] = 6;
	CWT_coef_up_mat[11][8] = -6;
	CWT_coef_up_mat[11][9] = -22;
	CWT_coef_up_mat[11][10] = 11;
	CWT_coef_up_mat[11][11] = 58;
	CWT_coef_up_mat[11][12] = -4;
	CWT_coef_up_mat[11][13] = -100;
	CWT_coef_up_mat[11][14] = -20;
	CWT_coef_up_mat[11][15] = 119;
	CWT_coef_up_mat[11][16] = 45;
	CWT_coef_up_mat[11][17] = -93;
	CWT_coef_up_mat[11][18] = -53;
	CWT_coef_up_mat[11][19] = 49;
	CWT_coef_up_mat[11][20] = 34;
	CWT_coef_up_mat[11][21] = -16;
	CWT_coef_up_mat[11][22] = -15;
	CWT_coef_up_mat[11][23] = 3;
	CWT_coef_up_mat[11][24] = 5;
	CWT_coef_up_mat[11][25] = 0;
	CWT_coef_up_mat[11][26] = -1;
	CWT_coef_up_mat[11][27] = 0;
	CWT_coef_up_mat[11][28] = 0;
	CWT_coef_up_mat[11][29] = 0;
	CWT_coef_up_mat[12][0] = 0;
	CWT_coef_up_mat[12][1] = 0;
	CWT_coef_up_mat[12][2] = 0;
	CWT_coef_up_mat[12][3] = 0;
	CWT_coef_up_mat[12][4] = 0;
	CWT_coef_up_mat[12][5] = -1;
	CWT_coef_up_mat[12][6] = 1;
	CWT_coef_up_mat[12][7] = 6;
	CWT_coef_up_mat[12][8] = -4;
	CWT_coef_up_mat[12][9] = -21;
	CWT_coef_up_mat[12][10] = 2;
	CWT_coef_up_mat[12][11] = 55;
	CWT_coef_up_mat[12][12] = 11;
	CWT_coef_up_mat[12][13] = -95;
	CWT_coef_up_mat[12][14] = -37;
	CWT_coef_up_mat[12][15] = 105;
	CWT_coef_up_mat[12][16] = 68;
	CWT_coef_up_mat[12][17] = -84;
	CWT_coef_up_mat[12][18] = -65;
	CWT_coef_up_mat[12][19] = 43;
	CWT_coef_up_mat[12][20] = 39;
	CWT_coef_up_mat[12][21] = -12;
	CWT_coef_up_mat[12][22] = -17;
	CWT_coef_up_mat[12][23] = 2;
	CWT_coef_up_mat[12][24] = 5;
	CWT_coef_up_mat[12][25] = 0;
	CWT_coef_up_mat[12][26] = -1;
	CWT_coef_up_mat[12][27] = 0;
	CWT_coef_up_mat[12][28] = 0;
	CWT_coef_up_mat[12][29] = 0;
	CWT_coef_up_mat[13][0] = 0;
	CWT_coef_up_mat[13][1] = 0;
	CWT_coef_up_mat[13][2] = 0;
	CWT_coef_up_mat[13][3] = 0;
	CWT_coef_up_mat[13][4] = 0;
	CWT_coef_up_mat[13][5] = -1;
	CWT_coef_up_mat[13][6] = 1;
	CWT_coef_up_mat[13][7] = 6;
	CWT_coef_up_mat[13][8] = -1;
	CWT_coef_up_mat[13][9] = -21;
	CWT_coef_up_mat[13][10] = -3;
	CWT_coef_up_mat[13][11] = 52;
	CWT_coef_up_mat[13][12] = 22;
	CWT_coef_up_mat[13][13] = -83;
	CWT_coef_up_mat[13][14] = -60;
	CWT_coef_up_mat[13][15] = 97;
	CWT_coef_up_mat[13][16] = 83;
	CWT_coef_up_mat[13][17] = -73;
	CWT_coef_up_mat[13][18] = -75;
	CWT_coef_up_mat[13][19] = 36;
	CWT_coef_up_mat[13][20] = 45;
	CWT_coef_up_mat[13][21] = -10;
	CWT_coef_up_mat[13][22] = -18;
	CWT_coef_up_mat[13][23] = 1;
	CWT_coef_up_mat[13][24] = 5;
	CWT_coef_up_mat[13][25] = 0;
	CWT_coef_up_mat[13][26] = -1;
	CWT_coef_up_mat[13][27] = 0;
	CWT_coef_up_mat[13][28] = 0;
	CWT_coef_up_mat[13][29] = 0;
	CWT_coef_up_mat[14][0] = 0;
	CWT_coef_up_mat[14][1] = 0;
	CWT_coef_up_mat[14][2] = 0;
	CWT_coef_up_mat[14][3] = 0;
	CWT_coef_up_mat[14][4] = -1;
	CWT_coef_up_mat[14][5] = 0;
	CWT_coef_up_mat[14][6] = 5;
	CWT_coef_up_mat[14][7] = 1;
	CWT_coef_up_mat[14][8] = -18;
	CWT_coef_up_mat[14][9] = -10;
	CWT_coef_up_mat[14][10] = 45;
	CWT_coef_up_mat[14][11] = 36;
	CWT_coef_up_mat[14][12] = -75;
	CWT_coef_up_mat[14][13] = -73;
	CWT_coef_up_mat[14][14] = 83;
	CWT_coef_up_mat[14][15] = 96;
	CWT_coef_up_mat[14][16] = -60;
	CWT_coef_up_mat[14][17] = -84;
	CWT_coef_up_mat[14][18] = 26;
	CWT_coef_up_mat[14][19] = 50;
	CWT_coef_up_mat[14][20] = -6;
	CWT_coef_up_mat[14][21] = -20;
	CWT_coef_up_mat[14][22] = 0;
	CWT_coef_up_mat[14][23] = 5;
	CWT_coef_up_mat[14][24] = 1;
	CWT_coef_up_mat[14][25] = -1;
	CWT_coef_up_mat[14][26] = 0;
	CWT_coef_up_mat[14][27] = 0;
	CWT_coef_up_mat[14][28] = 0;
	CWT_coef_up_mat[14][29] = 0;
	CWT_coef_up_mat[15][0] = 0;
	CWT_coef_up_mat[15][1] = 0;
	CWT_coef_up_mat[15][2] = 0;
	CWT_coef_up_mat[15][3] = 0;
	CWT_coef_up_mat[15][4] = -1;
	CWT_coef_up_mat[15][5] = 0;
	CWT_coef_up_mat[15][6] = 4;
	CWT_coef_up_mat[15][7] = 3;
	CWT_coef_up_mat[15][8] = -15;
	CWT_coef_up_mat[15][9] = -15;
	CWT_coef_up_mat[15][10] = 40;
	CWT_coef_up_mat[15][11] = 43;
	CWT_coef_up_mat[15][12] = -64;
	CWT_coef_up_mat[15][13] = -83;
	CWT_coef_up_mat[15][14] = 68;
	CWT_coef_up_mat[15][15] = 107;
	CWT_coef_up_mat[15][16] = -45;
	CWT_coef_up_mat[15][17] = -92;
	CWT_coef_up_mat[15][18] = 16;
	CWT_coef_up_mat[15][19] = 53;
	CWT_coef_up_mat[15][20] = -1;
	CWT_coef_up_mat[15][21] = -21;
	CWT_coef_up_mat[15][22] = -2;
	CWT_coef_up_mat[15][23] = 6;
	CWT_coef_up_mat[15][24] = 1;
	CWT_coef_up_mat[15][25] = -1;
	CWT_coef_up_mat[15][26] = 0;
	CWT_coef_up_mat[15][27] = 0;
	CWT_coef_up_mat[15][28] = 0;
	CWT_coef_up_mat[15][29] = 0;
	CWT_coef_up_mat[16][0] = 0;
	CWT_coef_up_mat[16][1] = 0;
	CWT_coef_up_mat[16][2] = 0;
	CWT_coef_up_mat[16][3] = 0;
	CWT_coef_up_mat[16][4] = -1;
	CWT_coef_up_mat[16][5] = -1;
	CWT_coef_up_mat[16][6] = 4;
	CWT_coef_up_mat[16][7] = 4;
	CWT_coef_up_mat[16][8] = -13;
	CWT_coef_up_mat[16][9] = -18;
	CWT_coef_up_mat[16][10] = 30;
	CWT_coef_up_mat[16][11] = 51;
	CWT_coef_up_mat[16][12] = -46;
	CWT_coef_up_mat[16][13] = -97;
	CWT_coef_up_mat[16][14] = 51;
	CWT_coef_up_mat[16][15] = 115;
	CWT_coef_up_mat[16][16] = -29;
	CWT_coef_up_mat[16][17] = -97;
	CWT_coef_up_mat[16][18] = 5;
	CWT_coef_up_mat[16][19] = 56;
	CWT_coef_up_mat[16][20] = 4;
	CWT_coef_up_mat[16][21] = -21;
	CWT_coef_up_mat[16][22] = -3;
	CWT_coef_up_mat[16][23] = 6;
	CWT_coef_up_mat[16][24] = 1;
	CWT_coef_up_mat[16][25] = -1;
	CWT_coef_up_mat[16][26] = 0;
	CWT_coef_up_mat[16][27] = 0;
	CWT_coef_up_mat[16][28] = 0;
	CWT_coef_up_mat[16][29] = 0;
	CWT_coef_up_mat[17][0] = 0;
	CWT_coef_up_mat[17][1] = 0;
	CWT_coef_up_mat[17][2] = 0;
	CWT_coef_up_mat[17][3] = 0;
	CWT_coef_up_mat[17][4] = -1;
	CWT_coef_up_mat[17][5] = -1;
	CWT_coef_up_mat[17][6] = 3;
	CWT_coef_up_mat[17][7] = 5;
	CWT_coef_up_mat[17][8] = -9;
	CWT_coef_up_mat[17][9] = -21;
	CWT_coef_up_mat[17][10] = 23;
	CWT_coef_up_mat[17][11] = 55;
	CWT_coef_up_mat[17][12] = -33;
	CWT_coef_up_mat[17][13] = -99;
	CWT_coef_up_mat[17][14] = 31;
	CWT_coef_up_mat[17][15] = 120;
	CWT_coef_up_mat[17][16] = -11;
	CWT_coef_up_mat[17][17] = -100;
	CWT_coef_up_mat[17][18] = -8;
	CWT_coef_up_mat[17][19] = 59;
	CWT_coef_up_mat[17][20] = 8;
	CWT_coef_up_mat[17][21] = -22;
	CWT_coef_up_mat[17][22] = -5;
	CWT_coef_up_mat[17][23] = 6;
	CWT_coef_up_mat[17][24] = 1;
	CWT_coef_up_mat[17][25] = -1;
	CWT_coef_up_mat[17][26] = 0;
	CWT_coef_up_mat[17][27] = 0;
	CWT_coef_up_mat[17][28] = 0;
	CWT_coef_up_mat[17][29] = 0;
	CWT_coef_up_mat[18][0] = 0;
	CWT_coef_up_mat[18][1] = 0;
	CWT_coef_up_mat[18][2] = 0;
	CWT_coef_up_mat[18][3] = 0;
	CWT_coef_up_mat[18][4] = 0;
	CWT_coef_up_mat[18][5] = -1;
	CWT_coef_up_mat[18][6] = 2;
	CWT_coef_up_mat[18][7] = 6;
	CWT_coef_up_mat[18][8] = -7;
	CWT_coef_up_mat[18][9] = -22;
	CWT_coef_up_mat[18][10] = 14;
	CWT_coef_up_mat[18][11] = 57;
	CWT_coef_up_mat[18][12] = -18;
	CWT_coef_up_mat[18][13] = -101;
	CWT_coef_up_mat[18][14] = 7;
	CWT_coef_up_mat[18][15] = 127;
	CWT_coef_up_mat[18][16] = 7;
	CWT_coef_up_mat[18][17] = -101;
	CWT_coef_up_mat[18][18] = -18;
	CWT_coef_up_mat[18][19] = 57;
	CWT_coef_up_mat[18][20] = 14;
	CWT_coef_up_mat[18][21] = -22;
	CWT_coef_up_mat[18][22] = -7;
	CWT_coef_up_mat[18][23] = 6;
	CWT_coef_up_mat[18][24] = 2;
	CWT_coef_up_mat[18][25] = -1;
	CWT_coef_up_mat[18][26] = 0;
	CWT_coef_up_mat[18][27] = 0;
	CWT_coef_up_mat[18][28] = 0;
	CWT_coef_up_mat[18][29] = 0;
	CWT_coef_up_mat[19][0] = 0;
	CWT_coef_up_mat[19][1] = 0;
	CWT_coef_up_mat[19][2] = 0;
	CWT_coef_up_mat[19][3] = 0;
	CWT_coef_up_mat[19][4] = 0;
	CWT_coef_up_mat[19][5] = -1;
	CWT_coef_up_mat[19][6] = 1;
	CWT_coef_up_mat[19][7] = 6;
	CWT_coef_up_mat[19][8] = -4;
	CWT_coef_up_mat[19][9] = -22;
	CWT_coef_up_mat[19][10] = 7;
	CWT_coef_up_mat[19][11] = 56;
	CWT_coef_up_mat[19][12] = -1;
	CWT_coef_up_mat[19][13] = -103;
	CWT_coef_up_mat[19][14] = -11;
	CWT_coef_up_mat[19][15] = 125;
	CWT_coef_up_mat[19][16] = 25;
	CWT_coef_up_mat[19][17] = -99;
	CWT_coef_up_mat[19][18] = -31;
	CWT_coef_up_mat[19][19] = 56;
	CWT_coef_up_mat[19][20] = 20;
	CWT_coef_up_mat[19][21] = -22;
	CWT_coef_up_mat[19][22] = -8;
	CWT_coef_up_mat[19][23] = 6;
	CWT_coef_up_mat[19][24] = 2;
	CWT_coef_up_mat[19][25] = -1;
	CWT_coef_up_mat[19][26] = 0;
	CWT_coef_up_mat[19][27] = 0;
	CWT_coef_up_mat[19][28] = 0;
	CWT_coef_up_mat[19][29] = 0;
	CWT_coef_up_mat[20][0] = 0;
	CWT_coef_up_mat[20][1] = 0;
	CWT_coef_up_mat[20][2] = 0;
	CWT_coef_up_mat[20][3] = 0;
	CWT_coef_up_mat[20][4] = 0;
	CWT_coef_up_mat[20][5] = -1;
	CWT_coef_up_mat[20][6] = 0;
	CWT_coef_up_mat[20][7] = 6;
	CWT_coef_up_mat[20][8] = -1;
	CWT_coef_up_mat[20][9] = -21;
	CWT_coef_up_mat[20][10] = -1;
	CWT_coef_up_mat[20][11] = 56;
	CWT_coef_up_mat[20][12] = 10;
	CWT_coef_up_mat[20][13] = -99;
	CWT_coef_up_mat[20][14] = -28;
	CWT_coef_up_mat[20][15] = 114;
	CWT_coef_up_mat[20][16] = 49;
	CWT_coef_up_mat[20][17] = -94;
	CWT_coef_up_mat[20][18] = -45;
	CWT_coef_up_mat[20][19] = 53;
	CWT_coef_up_mat[20][20] = 27;
	CWT_coef_up_mat[20][21] = -20;
	CWT_coef_up_mat[20][22] = -11;
	CWT_coef_up_mat[20][23] = 5;
	CWT_coef_up_mat[20][24] = 3;
	CWT_coef_up_mat[20][25] = -1;
	CWT_coef_up_mat[20][26] = -1;
	CWT_coef_up_mat[20][27] = 0;
	CWT_coef_up_mat[20][28] = 0;
	CWT_coef_up_mat[20][29] = 0;
	CWT_coef_up_mat[21][0] = 0;
	CWT_coef_up_mat[21][1] = 0;
	CWT_coef_up_mat[21][2] = 0;
	CWT_coef_up_mat[21][3] = 0;
	CWT_coef_up_mat[21][4] = 0;
	CWT_coef_up_mat[21][5] = -1;
	CWT_coef_up_mat[21][6] = 0;
	CWT_coef_up_mat[21][7] = 5;
	CWT_coef_up_mat[21][8] = 1;
	CWT_coef_up_mat[21][9] = -19;
	CWT_coef_up_mat[21][10] = -7;
	CWT_coef_up_mat[21][11] = 49;
	CWT_coef_up_mat[21][12] = 24;
	CWT_coef_up_mat[21][13] = -87;
	CWT_coef_up_mat[21][14] = -50;
	CWT_coef_up_mat[21][15] = 106;
	CWT_coef_up_mat[21][16] = 66;
	CWT_coef_up_mat[21][17] = -87;
	CWT_coef_up_mat[21][18] = -57;
	CWT_coef_up_mat[21][19] = 51;
	CWT_coef_up_mat[21][20] = 30;
	CWT_coef_up_mat[21][21] = -19;
	CWT_coef_up_mat[21][22] = -12;
	CWT_coef_up_mat[21][23] = 5;
	CWT_coef_up_mat[21][24] = 3;
	CWT_coef_up_mat[21][25] = -1;
	CWT_coef_up_mat[21][26] = -1;
	CWT_coef_up_mat[21][27] = 0;
	CWT_coef_up_mat[21][28] = 0;
	CWT_coef_up_mat[21][29] = 0;
	CWT_coef_up_mat[22][0] = 0;
	CWT_coef_up_mat[22][1] = 0;
	CWT_coef_up_mat[22][2] = 0;
	CWT_coef_up_mat[22][3] = 0;
	CWT_coef_up_mat[22][4] = 0;
	CWT_coef_up_mat[22][5] = -1;
	CWT_coef_up_mat[22][6] = 1;
	CWT_coef_up_mat[22][7] = 6;
	CWT_coef_up_mat[22][8] = -4;
	CWT_coef_up_mat[22][9] = -21;
	CWT_coef_up_mat[22][10] = 2;
	CWT_coef_up_mat[22][11] = 55;
	CWT_coef_up_mat[22][12] = 11;
	CWT_coef_up_mat[22][13] = -95;
	CWT_coef_up_mat[22][14] = -37;
	CWT_coef_up_mat[22][15] = 105;
	CWT_coef_up_mat[22][16] = 68;
	CWT_coef_up_mat[22][17] = -84;
	CWT_coef_up_mat[22][18] = -65;
	CWT_coef_up_mat[22][19] = 43;
	CWT_coef_up_mat[22][20] = 39;
	CWT_coef_up_mat[22][21] = -12;
	CWT_coef_up_mat[22][22] = -17;
	CWT_coef_up_mat[22][23] = 2;
	CWT_coef_up_mat[22][24] = 5;
	CWT_coef_up_mat[22][25] = 0;
	CWT_coef_up_mat[22][26] = -1;
	CWT_coef_up_mat[22][27] = 0;
	CWT_coef_up_mat[22][28] = 0;
	CWT_coef_up_mat[22][29] = 0;
	CWT_coef_up_mat[23][0] = 0;
	CWT_coef_up_mat[23][1] = 0;
	CWT_coef_up_mat[23][2] = 0;
	CWT_coef_up_mat[23][3] = 0;
	CWT_coef_up_mat[23][4] = 0;
	CWT_coef_up_mat[23][5] = -1;
	CWT_coef_up_mat[23][6] = 1;
	CWT_coef_up_mat[23][7] = 6;
	CWT_coef_up_mat[23][8] = -1;
	CWT_coef_up_mat[23][9] = -21;
	CWT_coef_up_mat[23][10] = -3;
	CWT_coef_up_mat[23][11] = 52;
	CWT_coef_up_mat[23][12] = 22;
	CWT_coef_up_mat[23][13] = -83;
	CWT_coef_up_mat[23][14] = -60;
	CWT_coef_up_mat[23][15] = 97;
	CWT_coef_up_mat[23][16] = 83;
	CWT_coef_up_mat[23][17] = -73;
	CWT_coef_up_mat[23][18] = -75;
	CWT_coef_up_mat[23][19] = 36;
	CWT_coef_up_mat[23][20] = 45;
	CWT_coef_up_mat[23][21] = -10;
	CWT_coef_up_mat[23][22] = -18;
	CWT_coef_up_mat[23][23] = 1;
	CWT_coef_up_mat[23][24] = 5;
	CWT_coef_up_mat[23][25] = 0;
	CWT_coef_up_mat[23][26] = -1;
	CWT_coef_up_mat[23][27] = 0;
	CWT_coef_up_mat[23][28] = 0;
	CWT_coef_up_mat[23][29] = 0;
	CWT_coef_up_mat[24][0] = 0;
	CWT_coef_up_mat[24][1] = 0;
	CWT_coef_up_mat[24][2] = 0;
	CWT_coef_up_mat[24][3] = 0;
	CWT_coef_up_mat[24][4] = -1;
	CWT_coef_up_mat[24][5] = 0;
	CWT_coef_up_mat[24][6] = 5;
	CWT_coef_up_mat[24][7] = 1;
	CWT_coef_up_mat[24][8] = -18;
	CWT_coef_up_mat[24][9] = -10;
	CWT_coef_up_mat[24][10] = 45;
	CWT_coef_up_mat[24][11] = 36;
	CWT_coef_up_mat[24][12] = -75;
	CWT_coef_up_mat[24][13] = -73;
	CWT_coef_up_mat[24][14] = 83;
	CWT_coef_up_mat[24][15] = 96;
	CWT_coef_up_mat[24][16] = -60;
	CWT_coef_up_mat[24][17] = -84;
	CWT_coef_up_mat[24][18] = 26;
	CWT_coef_up_mat[24][19] = 50;
	CWT_coef_up_mat[24][20] = -6;
	CWT_coef_up_mat[24][21] = -20;
	CWT_coef_up_mat[24][22] = 0;
	CWT_coef_up_mat[24][23] = 5;
	CWT_coef_up_mat[24][24] = 1;
	CWT_coef_up_mat[24][25] = -1;
	CWT_coef_up_mat[24][26] = 0;
	CWT_coef_up_mat[24][27] = 0;
	CWT_coef_up_mat[24][28] = 0;
	CWT_coef_up_mat[24][29] = 0;
	CWT_coef_up_mat[25][0] = 0;
	CWT_coef_up_mat[25][1] = 0;
	CWT_coef_up_mat[25][2] = 0;
	CWT_coef_up_mat[25][3] = 0;
	CWT_coef_up_mat[25][4] = -1;
	CWT_coef_up_mat[25][5] = 0;
	CWT_coef_up_mat[25][6] = 4;
	CWT_coef_up_mat[25][7] = 3;
	CWT_coef_up_mat[25][8] = -15;
	CWT_coef_up_mat[25][9] = -15;
	CWT_coef_up_mat[25][10] = 40;
	CWT_coef_up_mat[25][11] = 43;
	CWT_coef_up_mat[25][12] = -64;
	CWT_coef_up_mat[25][13] = -83;
	CWT_coef_up_mat[25][14] = 68;
	CWT_coef_up_mat[25][15] = 107;
	CWT_coef_up_mat[25][16] = -45;
	CWT_coef_up_mat[25][17] = -92;
	CWT_coef_up_mat[25][18] = 16;
	CWT_coef_up_mat[25][19] = 53;
	CWT_coef_up_mat[25][20] = -1;
	CWT_coef_up_mat[25][21] = -21;
	CWT_coef_up_mat[25][22] = -2;
	CWT_coef_up_mat[25][23] = 6;
	CWT_coef_up_mat[25][24] = 1;
	CWT_coef_up_mat[25][25] = -1;
	CWT_coef_up_mat[25][26] = 0;
	CWT_coef_up_mat[25][27] = 0;
	CWT_coef_up_mat[25][28] = 0;
	CWT_coef_up_mat[25][29] = 0;
	CWT_coef_up_mat[26][0] = 0;
	CWT_coef_up_mat[26][1] = 0;
	CWT_coef_up_mat[26][2] = 0;
	CWT_coef_up_mat[26][3] = 0;
	CWT_coef_up_mat[26][4] = -1;
	CWT_coef_up_mat[26][5] = -1;
	CWT_coef_up_mat[26][6] = 4;
	CWT_coef_up_mat[26][7] = 4;
	CWT_coef_up_mat[26][8] = -13;
	CWT_coef_up_mat[26][9] = -18;
	CWT_coef_up_mat[26][10] = 30;
	CWT_coef_up_mat[26][11] = 51;
	CWT_coef_up_mat[26][12] = -46;
	CWT_coef_up_mat[26][13] = -97;
	CWT_coef_up_mat[26][14] = 51;
	CWT_coef_up_mat[26][15] = 115;
	CWT_coef_up_mat[26][16] = -29;
	CWT_coef_up_mat[26][17] = -97;
	CWT_coef_up_mat[26][18] = 5;
	CWT_coef_up_mat[26][19] = 56;
	CWT_coef_up_mat[26][20] = 4;
	CWT_coef_up_mat[26][21] = -21;
	CWT_coef_up_mat[26][22] = -3;
	CWT_coef_up_mat[26][23] = 6;
	CWT_coef_up_mat[26][24] = 1;
	CWT_coef_up_mat[26][25] = -1;
	CWT_coef_up_mat[26][26] = 0;
	CWT_coef_up_mat[26][27] = 0;
	CWT_coef_up_mat[26][28] = 0;
	CWT_coef_up_mat[26][29] = 0;
	CWT_coef_up_mat[27][0] = 0;
	CWT_coef_up_mat[27][1] = 0;
	CWT_coef_up_mat[27][2] = 0;
	CWT_coef_up_mat[27][3] = 0;
	CWT_coef_up_mat[27][4] = -1;
	CWT_coef_up_mat[27][5] = -1;
	CWT_coef_up_mat[27][6] = 3;
	CWT_coef_up_mat[27][7] = 5;
	CWT_coef_up_mat[27][8] = -9;
	CWT_coef_up_mat[27][9] = -21;
	CWT_coef_up_mat[27][10] = 23;
	CWT_coef_up_mat[27][11] = 55;
	CWT_coef_up_mat[27][12] = -33;
	CWT_coef_up_mat[27][13] = -99;
	CWT_coef_up_mat[27][14] = 31;
	CWT_coef_up_mat[27][15] = 120;
	CWT_coef_up_mat[27][16] = -11;
	CWT_coef_up_mat[27][17] = -100;
	CWT_coef_up_mat[27][18] = -8;
	CWT_coef_up_mat[27][19] = 59;
	CWT_coef_up_mat[27][20] = 8;
	CWT_coef_up_mat[27][21] = -22;
	CWT_coef_up_mat[27][22] = -5;
	CWT_coef_up_mat[27][23] = 6;
	CWT_coef_up_mat[27][24] = 1;
	CWT_coef_up_mat[27][25] = -1;
	CWT_coef_up_mat[27][26] = 0;
	CWT_coef_up_mat[27][27] = 0;
	CWT_coef_up_mat[27][28] = 0;
	CWT_coef_up_mat[27][29] = 0;
	CWT_coef_up_mat[28][0] = 0;
	CWT_coef_up_mat[28][1] = 0;
	CWT_coef_up_mat[28][2] = 0;
	CWT_coef_up_mat[28][3] = 0;
	CWT_coef_up_mat[28][4] = 0;
	CWT_coef_up_mat[28][5] = -1;
	CWT_coef_up_mat[28][6] = 2;
	CWT_coef_up_mat[28][7] = 6;
	CWT_coef_up_mat[28][8] = -7;
	CWT_coef_up_mat[28][9] = -22;
	CWT_coef_up_mat[28][10] = 14;
	CWT_coef_up_mat[28][11] = 57;
	CWT_coef_up_mat[28][12] = -18;
	CWT_coef_up_mat[28][13] = -101;
	CWT_coef_up_mat[28][14] = 7;
	CWT_coef_up_mat[28][15] = 127;
	CWT_coef_up_mat[28][16] = 7;
	CWT_coef_up_mat[28][17] = -101;
	CWT_coef_up_mat[28][18] = -18;
	CWT_coef_up_mat[28][19] = 57;
	CWT_coef_up_mat[28][20] = 14;
	CWT_coef_up_mat[28][21] = -22;
	CWT_coef_up_mat[28][22] = -7;
	CWT_coef_up_mat[28][23] = 6;
	CWT_coef_up_mat[28][24] = 2;
	CWT_coef_up_mat[28][25] = -1;
	CWT_coef_up_mat[28][26] = 0;
	CWT_coef_up_mat[28][27] = 0;
	CWT_coef_up_mat[28][28] = 0;
	CWT_coef_up_mat[28][29] = 0;
	CWT_coef_up_mat[29][0] = 0;
	CWT_coef_up_mat[29][1] = 0;
	CWT_coef_up_mat[29][2] = 0;
	CWT_coef_up_mat[29][3] = 0;
	CWT_coef_up_mat[29][4] = 0;
	CWT_coef_up_mat[29][5] = -1;
	CWT_coef_up_mat[29][6] = 1;
	CWT_coef_up_mat[29][7] = 6;
	CWT_coef_up_mat[29][8] = -4;
	CWT_coef_up_mat[29][9] = -22;
	CWT_coef_up_mat[29][10] = 7;
	CWT_coef_up_mat[29][11] = 56;
	CWT_coef_up_mat[29][12] = -1;
	CWT_coef_up_mat[29][13] = -103;
	CWT_coef_up_mat[29][14] = -11;
	CWT_coef_up_mat[29][15] = 125;
	CWT_coef_up_mat[29][16] = 25;
	CWT_coef_up_mat[29][17] = -99;
	CWT_coef_up_mat[29][18] = -31;
	CWT_coef_up_mat[29][19] = 56;
	CWT_coef_up_mat[29][20] = 20;
	CWT_coef_up_mat[29][21] = -22;
	CWT_coef_up_mat[29][22] = -8;
	CWT_coef_up_mat[29][23] = 6;
	CWT_coef_up_mat[29][24] = 2;
	CWT_coef_up_mat[29][25] = -1;
	CWT_coef_up_mat[29][26] = 0;
	CWT_coef_up_mat[29][27] = 0;
	CWT_coef_up_mat[29][28] = 0;
	CWT_coef_up_mat[29][29] = 0;
	CWT_coef_up_mat[30][0] = 0;
	CWT_coef_up_mat[30][1] = 0;
	CWT_coef_up_mat[30][2] = 0;
	CWT_coef_up_mat[30][3] = 0;
	CWT_coef_up_mat[30][4] = 0;
	CWT_coef_up_mat[30][5] = -1;
	CWT_coef_up_mat[30][6] = 0;
	CWT_coef_up_mat[30][7] = 6;
	CWT_coef_up_mat[30][8] = -1;
	CWT_coef_up_mat[30][9] = -21;
	CWT_coef_up_mat[30][10] = -1;
	CWT_coef_up_mat[30][11] = 56;
	CWT_coef_up_mat[30][12] = 10;
	CWT_coef_up_mat[30][13] = -99;
	CWT_coef_up_mat[30][14] = -28;
	CWT_coef_up_mat[30][15] = 114;
	CWT_coef_up_mat[30][16] = 49;
	CWT_coef_up_mat[30][17] = -94;
	CWT_coef_up_mat[30][18] = -45;
	CWT_coef_up_mat[30][19] = 53;
	CWT_coef_up_mat[30][20] = 27;
	CWT_coef_up_mat[30][21] = -20;
	CWT_coef_up_mat[30][22] = -11;
	CWT_coef_up_mat[30][23] = 5;
	CWT_coef_up_mat[30][24] = 3;
	CWT_coef_up_mat[30][25] = -1;
	CWT_coef_up_mat[30][26] = -1;
	CWT_coef_up_mat[30][27] = 0;
	CWT_coef_up_mat[30][28] = 0;
	CWT_coef_up_mat[30][29] = 0;
	CWT_coef_up_mat[31][0] = 0;
	CWT_coef_up_mat[31][1] = 0;
	CWT_coef_up_mat[31][2] = 0;
	CWT_coef_up_mat[31][3] = 0;
	CWT_coef_up_mat[31][4] = 0;
	CWT_coef_up_mat[31][5] = -1;
	CWT_coef_up_mat[31][6] = 0;
	CWT_coef_up_mat[31][7] = 5;
	CWT_coef_up_mat[31][8] = 1;
	CWT_coef_up_mat[31][9] = -19;
	CWT_coef_up_mat[31][10] = -7;
	CWT_coef_up_mat[31][11] = 49;
	CWT_coef_up_mat[31][12] = 24;
	CWT_coef_up_mat[31][13] = -87;
	CWT_coef_up_mat[31][14] = -50;
	CWT_coef_up_mat[31][15] = 106;
	CWT_coef_up_mat[31][16] = 66;
	CWT_coef_up_mat[31][17] = -87;
	CWT_coef_up_mat[31][18] = -57;
	CWT_coef_up_mat[31][19] = 51;
	CWT_coef_up_mat[31][20] = 30;
	CWT_coef_up_mat[31][21] = -19;
	CWT_coef_up_mat[31][22] = -12;
	CWT_coef_up_mat[31][23] = 5;
	CWT_coef_up_mat[31][24] = 3;
	CWT_coef_up_mat[31][25] = -1;
	CWT_coef_up_mat[31][26] = -1;
	CWT_coef_up_mat[31][27] = 0;
	CWT_coef_up_mat[31][28] = 0;
	CWT_coef_up_mat[31][29] = 0;
	CWT_coef_up_mat[32][0] = 0;
	CWT_coef_up_mat[32][1] = 0;
	CWT_coef_up_mat[32][2] = 0;
	CWT_coef_up_mat[32][3] = 0;
	CWT_coef_up_mat[32][4] = 0;
	CWT_coef_up_mat[32][5] = -1;
	CWT_coef_up_mat[32][6] = 0;
	CWT_coef_up_mat[32][7] = 4;
	CWT_coef_up_mat[32][8] = 3;
	CWT_coef_up_mat[32][9] = -17;
	CWT_coef_up_mat[32][10] = -12;
	CWT_coef_up_mat[32][11] = 45;
	CWT_coef_up_mat[32][12] = 35;
	CWT_coef_up_mat[32][13] = -80;
	CWT_coef_up_mat[32][14] = -65;
	CWT_coef_up_mat[32][15] = 95;
	CWT_coef_up_mat[32][16] = 82;
	CWT_coef_up_mat[32][17] = -83;
	CWT_coef_up_mat[32][18] = -63;
	CWT_coef_up_mat[32][19] = 46;
	CWT_coef_up_mat[32][20] = 36;
	CWT_coef_up_mat[32][21] = -17;
	CWT_coef_up_mat[32][22] = -14;
	CWT_coef_up_mat[32][23] = 4;
	CWT_coef_up_mat[32][24] = 4;
	CWT_coef_up_mat[32][25] = -1;
	CWT_coef_up_mat[32][26] = -1;
	CWT_coef_up_mat[32][27] = 0;
	CWT_coef_up_mat[32][28] = 0;
	CWT_coef_up_mat[32][29] = 0;


	CWT_UART_cmd_HfLf_tx = 0xC0;
	CWT_UART_cmd_Hf_tx   = 0xC1;
	CWT_UART_cmd_datard  = 0xC2;
	CWT_UART_cmd_datawr  = 0xC3;

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


	CWT_delay_cnt = 0;

	for(CWT_hfreq=0; CWT_hfreq<3; CWT_hfreq++)
	{

		for(CWT_lfreq=0; CWT_lfreq<11; CWT_lfreq++)
		{

			// %% High pass filtering for LF removing
			// X1i = High_filter (5,f_dc, Fs, SI11);

			for(CWT_i=0; CWT_i<2; CWT_i++)	//  two signals
			{
				if( CWT_i == 0)	// HF+LF
					UART_out( CWT_UART_cmd_HfLf_tx ); // cmd
				else
					UART_out( CWT_UART_cmd_Hf_tx ); // cmd

				// wait interval between UART Tx/RX
				for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
					CWT_delay_cnt++;

				CWT_UART_tx_freq = ( (CWT_hfreq <<4 ) + CWT_lfreq) & 0xFF;
				UART_out( CWT_UART_tx_freq ); // cmd

				// tx complete wait
				for (CWT_delay_k=0; CWT_delay_k < Tx_wait_time_index; CWT_delay_k++)
					CWT_delay_cnt++;


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
						CWT_base_addr = 0; // first signal
				else	CWT_base_addr = 0x40000; // second signal, 250k


				for(CWT_j=0; CWT_j<250000; CWT_j++) // 250,000
				{

					// USART
					CWT_UART_addr = CWT_base_addr + CWT_j;
					CWT_UART_addr_L8bit = (CWT_UART_addr & 0xFF);
					CWT_UART_addr_M8bit = ((CWT_UART_addr & 0xFF00) >> 8);
					CWT_UART_addr_H3bit = ((CWT_UART_addr & 0x70000) >> 16);

					UART_out( CWT_UART_cmd_datard ); // cmd
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;

					UART_out( CWT_UART_addr_L8bit ); // libuart
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;

					UART_out( CWT_UART_addr_M8bit );
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;

					UART_out( CWT_UART_addr_H3bit );
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;


					// CPLD data read
//					while( (UC1IFG & 0x01)==0 ) ; //kdw
					CWT_UART_rdata_L8bit = (UCA1RXBUF & 0xFF);

//					while( (UC1IFG & 0x01)==0 ) ; //kdw
					CWT_UART_rdata_H8bit = (UCA1RXBUF & 0xFF);


					CWT_read_data = (CWT_UART_rdata_H8bit << 8) + CWT_UART_rdata_L8bit;
					// 2's complement
					if( CWT_read_data >= 2048)
						CWT_read_data = CWT_read_data - 4096;




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



					// CPLD data write using USART
					UART_out( CWT_UART_cmd_datawr ); // cmd
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;

					UART_out( CWT_UART_addr_L8bit ); // libuart
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;

					UART_out( CWT_UART_addr_M8bit );
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;

					UART_out( CWT_UART_addr_H3bit );
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;

					CWT_UART_wdata_L8bit = ( CWT_hpf_out & 0xFF );
					CWT_UART_wdata_H8bit = ( ( CWT_hpf_out & 0xFF00 ) >> 8);

					UART_out( CWT_UART_wdata_L8bit );
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;

					UART_out( CWT_UART_wdata_H8bit );
						// wait interval between UART Tx/RX
						for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
							CWT_delay_cnt++;


				} // for(CWT_j=0; CWT_j<250000; CWT_j++)  // 250,000


			} // for(CWT_i=0; CWT_i<2; CWT_i++)

			// ==== END ===== %% High pass filtering for LF removing



			// %% LRS, length: 300,001
			// LRS_i = X1i_s- X2i_s;


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

			CWT_first_sig_base_addr = 0; // first signal
			CWT_second_sig_base_addr = 0x40000; // second signal, 250k

			for(CWT_j=0; CWT_j<262144; CWT_j++) // 300,000, 262144=2^18
			{

				// CPLD data read using USART, first data
				CWT_UART_addr = CWT_first_sig_base_addr + CWT_j;
				CWT_UART_addr_L8bit = (CWT_UART_addr & 0xFF);
				CWT_UART_addr_M8bit = ((CWT_UART_addr & 0xFF00) >> 8);
				CWT_UART_addr_H3bit = ((CWT_UART_addr & 0x70000) >> 16);

				UART_out( CWT_UART_cmd_datard ); // cmd
					// wait interval between UART Tx/RX
					for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
						CWT_delay_cnt++;

				UART_out( CWT_UART_addr_L8bit ); // libuart
					// wait interval between UART Tx/RX
					for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
						CWT_delay_cnt++;

				UART_out( CWT_UART_addr_M8bit );
					// wait interval between UART Tx/RX
					for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
						CWT_delay_cnt++;

				UART_out( CWT_UART_addr_H3bit );
					// wait interval between UART Tx/RX
					for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
						CWT_delay_cnt++;


				// CPLD data read
//				while( (UC1IFG & 0x01)==0 ) ;//kdw
				CWT_UART_rdata_L8bit = (UCA1RXBUF & 0xFF);

//				while( (UC1IFG & 0x01)==0 ) ;//kdw
				CWT_UART_rdata_H8bit = (UCA1RXBUF & 0xFF);


				CWT_X1 = (CWT_UART_rdata_H8bit << 8) + CWT_UART_rdata_L8bit;
				// 2's complement, HPF_out : 14 bit
				if( CWT_X1 >= 8192)
					CWT_X1 = CWT_X1 - 16384;



				// CPLD data read using USART, second data
				CWT_UART_addr = CWT_second_sig_base_addr + CWT_j;
				CWT_UART_addr_L8bit = (CWT_UART_addr & 0xFF);
				CWT_UART_addr_M8bit = ((CWT_UART_addr & 0xFF00) >> 8);
				CWT_UART_addr_H3bit = ((CWT_UART_addr & 0x70000) >> 16);

				UART_out( CWT_UART_cmd_datard ); // cmd
					// wait interval between UART Tx/RX
					for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
						CWT_delay_cnt++;

				UART_out( CWT_UART_addr_L8bit ); // libuart
					// wait interval between UART Tx/RX
					for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
						CWT_delay_cnt++;

				UART_out( CWT_UART_addr_M8bit );
					// wait interval between UART Tx/RX
					for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
						CWT_delay_cnt++;

				UART_out( CWT_UART_addr_H3bit );
					// wait interval between UART Tx/RX
					for (CWT_delay_k=0; CWT_delay_k < UART_wait_time_index; CWT_delay_k++)
						CWT_delay_cnt++;


				// CPLD data read
				while( (UC1IFG & 0x01)==0 ) ;
				CWT_UART_rdata_L8bit = (UCA1RXBUF & 0xFF);

				while( (UC1IFG & 0x01)==0 ) ;
				CWT_UART_rdata_H8bit = (UCA1RXBUF & 0xFF);


				CWT_X2 = (CWT_UART_rdata_H8bit << 8) + CWT_UART_rdata_L8bit;
				// 2's complement, HPF_out : 14 bit
				if( CWT_X2 >= 8192)
					CWT_X2 = CWT_X2 - 16384;


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

