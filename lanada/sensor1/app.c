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
 *         Deawoo, Kim 	<dwkim@lanada.kast.ac.kr>
 */

#include "contiki.h"
#include "net/rime.h"
#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "sys/etimer.h"
//for clock-scheme
#include "ntp.h"
#include "sclock.h"
#include "random.h"
//sensor
#include "sensing.h" //kdw
#include "../cpu/msp430/dev/uart0.h" // hcpark,
#include "sys/clock.h"	// hcpark
#include "uart.h"

#include <stdio.h>
#include <stdlib.h>
#define DEBUGPRINT 1

// define data type
#define DATA 				0x10	// '00010000'
#define	SYNC_START			0x20	// '00100000'
#define	SYNC_ACK			0x42	// '01000010'
#define	SYNC_END			0x80	// '10000000'

/*---------------------------------------------------------------------------*/
PROCESS(app_layer_process, "Sensor network App layer start");
AUTOSTART_PROCESSES(&app_layer_process);
/*---------------------------------------------------------------------------*/
//static variables
typedef enum {DATA_flag=1, SYNC_flag, END_flag} input_flag;

static uint8_t result_data;
static uint8_t is_data_or_sync;
static input_flag flag;
static int clock_drift;
static uint8_t is_sleep_mode;
static uint8_t data_backup[PACKETBUF_SIZE];
static uint8_t length_backup;

//time sync
static struct etimer et;
static struct ntp *ntp;
static struct sclock *sc;
static struct timestamp sleeptime;

/*---------------------------------------------------------------------------*/
//functions declaration
static void Address_setup();
static int Sensor_start();
static uint8_t Sensor_calc(int);
static uint8_t Plb_on();
static uint8_t Send(uint8_t);
static void Data_aggregation();
static int Sync_calc();
static void Sync_modifying(int);

/*---------------------------------------------------------------------------*/
//callback functions
static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
#if DEBUGPRINT
	printf("[app] recv callback\n");
#endif
	uint8_t* packet_temp;
	uint8_t check_bit;
	packet_temp=(uint8_t*)packetbuf_dataptr();

	check_bit=packet_temp[0]; //6 right shifts
	if(check_bit==DATA) // check_bit == '0x10'
	{
		is_data_or_sync=DATA;
		flag=DATA_flag;
	}
	else if(check_bit==SYNC_ACK) // check_bit == '0x42'
	{
		is_data_or_sync=SYNC_ACK;
		flag=SYNC_flag;
	}
	else if(check_bit==SYNC_END) // check_bit == '0x80'
	{
//		is_data_or_sync=SYNC_END;
		flag=END_flag;
	}
	else
	{
		is_data_or_sync=0;
	}
	length_backup=packetbuf_datalen();
	packetbuf_copyto(data_backup);

	return;
}

static void
sent_uc(struct unicast_conn *c, int status, int num_tx)
{
#if DEBUGPRINT
	printf("sent callback [status : %d]\n",status);
#endif
	if(is_data_or_sync == SYNC_ACK){
		flag=END_flag;
		is_data_or_sync=SYNC_END;
	}
}
 //it is no longer useful
static const struct unicast_callbacks unicast_callbacks = {recv_uc,sent_uc};//sent_uc is not used at the present stage
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
//Address function
static void
Address_setup()
{
	rimeaddr_t next_node_addr;
	rimeaddr_t prev_node_addr;
	if(rimeaddr_node_addr.u8[0]==1)
	{
		packetbuf_set_datalen(0);

	}
	next_node_addr.u8[0]=rimeaddr_node_addr.u8[0]+1;
	next_node_addr.u8[1]=rimeaddr_node_addr.u8[1];
	prev_node_addr.u8[0]=rimeaddr_node_addr.u8[0]-1;
	prev_node_addr.u8[1]=rimeaddr_node_addr.u8[1]; //it only can handle the case of MAX_NODE <256

	packetbuf_set_addr(PACKETBUF_ADDR_NOW,&rimeaddr_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_NEXT,&next_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_PREVIOUS,&prev_node_addr);

#if DEBUGPRINT
	printf("[app] Setting addresses\n");
#endif
	//address setting using packetbuf_set_addr
}
/*---------------------------------------------------------------------------*/
//Sensor functions
static int //return type check
Sensor_start()
{
	//sending a signal to Sensor to operate
#if DEBUGPRINT
	printf("[app] Sensing complete\n");
#endif

	{
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


					 InitSensor();





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
			}

	return 1; //return sensing result
}
static uint8_t
Sensor_calc(int data)
{
	//calculate sensing data and return result value 0 or 1
#if DEBUGPRINT
	printf("[app] Sensing calculation complete\n");
#endif
	return 1;
}
/*---------------------------------------------------------------------------*/
//plb signals
static uint8_t
Plb_on()
{

	//sending a signal to plb layer
#if DEBUGPRINT
	printf("[app] plb on request\n");
#endif
	NETSTACK_RDC.on();
	return 1;
}

static uint8_t
Send(uint8_t type)
{

	if(type==DATA) //if type is DATA
	{
#if DEBUGPRINT
	printf("[app] send data (%d)\n",result_data);
#endif
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,DATA);
		//send DATA to NEXT node
		return unicast_send(&uc, packetbuf_addr(PACKETBUF_ADDR_NEXT));//TR result return
	}
	else//if type is SYNC
	{
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,SYNC_START);
		//send SYNC_start to PREV node
		return unicast_send(&uc, packetbuf_addr(PACKETBUF_ADDR_PREVIOUS));//TR result return
	}
}
/*---------------------------------------------------------------------------*/
//data function
static void
Data_aggregation()
{
	uint16_t length=packetbuf_datalen();
	uint16_t node_num=(uint16_t)rimeaddr_node_addr.u8[0]; //this length means bit level, the bit position that is result data locates
	uint8_t shift_amt,index; //ptr index value, the number of shift operations to apply
	uint8_t *dataptr_temp;
	dataptr_temp=(uint8_t *)packetbuf_dataptr();
//	printf("Data_aggregation %d %x %x, total length %d\n",length,dataptr_temp[0],dataptr_temp[1],packetbuf_totlen());
	if(node_num==1) //it means START node case, set DATA bits at first byte
	{
		dataptr_temp[0]=DATA;// first byte = 0x10
		length=1;
	}
	if(node_num%8)
	{
		shift_amt=8-node_num%8;
	}
	else //node_num%8 == 0
	{
		shift_amt=0;
	}

	if(node_num%8==1)
	{
			length++;
	}
	index=length-1;
	dataptr_temp[index]|=result_data<<shift_amt;

	packetbuf_set_datalen(length);
	//data aggregation using dataptr
#if DEBUGPRINT
	printf("[app] received data aggregation\n");
#endif
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(app_layer_process, ev, data)
{
	int sensor_value; // type check
	static struct etimer et;
	//data_backup=(uint8_t*)malloc(PACKETBUF_SIZE*sizeof(uint8_t));
	PROCESS_EXITHANDLER(unicast_close(&uc);)

	PROCESS_BEGIN();

	//sync init : KJY
	sclock_create(&sc, TYPE_CLOCK);
	ntp_create(&ntp);

#if DEBUGPRINT
	printf("PROCESS BEGINNING\n");
#endif
	SENSORS_ACTIVATE(button_sensor);
	slip_arch_init(115200);
	unicast_open(&uc, 146, &unicast_callbacks);
#if DEBUGPRINT
	printf("[app] Setting a unicast channel\n");
#endif
	Address_setup();



	while(1) {
		is_sleep_mode=0;
		///////////////
		// sensing part
		///////////////



		sensor_value=Sensor_start();
		result_data=Sensor_calc(sensor_value);

//		PROCESS_WAIT_EVENT_UNTIL((ev==sensors_event) && (data == &button_sensor));

		Plb_on();
		if(rimeaddr_node_addr.u8[0]==1 && rimeaddr_node_addr.u8[1]==0)
		{
			packetbuf_clear();
			Data_aggregation();
			Send(DATA);
		}
#if DEBUGPRINT
		printf("[app] waiting until intterupt\n");
#endif
		//PROCESS_WAIT_EVENT_UNTIL(is_sleep_mode);
		while(!is_sleep_mode) // can i replace it with PROCESS_WAIT_UNTIL or some PROCESS function?
		{
			  etimer_set(&et, CLOCK_SECOND/1000);
			  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			  if(flag)
			  {
#if DEBUGPRINT
				  printf("[app] flag value : %d\n",flag);
#endif
				  switch(flag)
				  {
				  case DATA_flag:
					  flag=0;
					  packetbuf_copyfrom(data_backup,length_backup);
					  Data_aggregation();
					  Send(DATA);
					  break;
				  case SYNC_flag:
					  flag=0;
					  //sync calc
					  packetbuf_clear();
					  packetbuf_copyfrom(data_backup,length_backup);
					  ntp_handle_ack(ntp, packetbuf_dataptr());//KJY
					  if (rimeaddr_node_addr.u8[0]==1 && rimeaddr_node_addr.u8[1]==0){
						  is_sleep_mode=1;
						  NETSTACK_RDC.off(0);
						  break;
					  }
					  packetbuf_clear();
#if DEBUGPRINT
					  printf("[app] Synchronized!!");
#endif
//					  print_ntp(ntp);
					  Send(SYNC_START);
					  break;
				  case END_flag:
					  flag=0;
					  is_sleep_mode=1;
					  NETSTACK_RDC.off(0);
					  break;
				  }
			  }
			//waiting
		}
#if DEBUGPRINT
		printf("[app] goto sleep mode, wake up after 21days\n");
#endif
		NETSTACK_RDC.off(0);

	    leds_on(LEDS_RED);
	    etimer_set(&et, CLOCK_SECOND);
	    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	    leds_off(LEDS_RED);

	    timestamp_init(&sleeptime);
		sleeptime.sec = (uint8_t)((random_rand() % 5) +1);
	    while(!timestamp_is_empty(&sleeptime)){
	      sclock_etimer_set_long(sc, &et, &sleeptime);
	      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

	    }
#if DEBUGPRINT
	    printf("[app] sleep end\n");
#endif
	    leds_on(LEDS_YELLOW);
	    etimer_set(&et, CLOCK_SECOND );
	    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	    leds_off(7);
	}



	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
