/*
 * Bismillahirrohmaanirrohiim
 * ABINARA-1 KRPAI ITS JUARA NASIONAL
 * Doa Usaha Tawakal Tetap Semangat
 * Sesungguhnya bersama kesulitan ada kemudahan
 *
 */
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_dma.h>
#include <misc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "delay.h"
#include "lcd_biasa.h"
#include "invers.h"
#include "RX28.h"
#include "pengolahanKakiRX28.h"
#include "motion.h"
#include "interruptTxUsart.h"
#include "ADC_DMA2.h"
#include "initIO.h"
#include "initBluetooth.h";
#include "initSlave.h"
#include "stdbool.h"

#define buka_co	500
#define tutup_co 950

#define LURUSKAN_KANAN	555
#define LURUSKAN_KIRI 	554
#define LURUSKAN_CMPS 	545

#define TELUSURSCAN_KANAN	551
#define TELUSURSCAN_KIRI	550
#define TELUSURSCAN_KIRI_UV1	549
#define TELUSURSCAN_KIRI_UV2	548
#define TELUSURSCAN_KANAN_UV1	547
#define TELUSURSCAN_KANAN_UV2	546

#define kalibrasi_utara		33 //cmps kalibrasi
#define kalibrasi_selatan	199
#define kalibrasi_barat		301
#define kalibrasi_timur		108

#define kalibrasi_serong_timur 146
#define kalibrasi_serong_selatan 242
#define kalibrasi_serong_barat 343
#define kalibrasi_serong_utara 70

#define acuan_1a 37 //cmps tpa
#define acuan_1b 121
#define acuan_3_4a 210
#define acuan_4b 76

#define kecepatan_edf TIM4->CCR1 //timer EDF
int edf = 1000;

short int state_telusur;

short int ruangArr[20];
char tampil[16];
int i, j;
int penanda_ganti_muka;
short int posisi_anjing;
short int penanda_buzzer;
short int status_api = 0;
short int eksistansi_api2;
int eksistansi_api1;
int ruang_api;
int ruang_start;
int state_jalan;
int next_state_jalan;
int before_state_jalan;
int arah_deteksi_lost;
int state_kembalidarilost;
int langkahScan;
int ruangScan;
int konversi_scan;
int konversi_loncat;
int keluar_kanan;
int keluar_kiri;
int penanda205;
int penanda_keluar_4a_4b_1b;
int penanda_keluar_1a;

float kape1 = 5.504, kape2 = 0.045;
float deriv1 = 0.22, deriv2 = 0.1;
float p_rot1 = 0, p_rot2 = 0;
float deriv_rot1 = 0, deriv_rot2 = 0;

int ganti_muka;
int ganti_kanan;
int ganti_kiri;
int setPoint0z = 160; //160 250 200
//int setPoint1z = 250; //120
int setPoint2z = 80; //90

//KP2z=0.045/0.02, KP1z=0, KPrz=0.014/0.008, KIrz=0;
float KP2z = 0.02, KP1z = 0, KPrz = 0.002, KIrz = 0;
//KDz=0.1,KDrz=0;
float KDz = 0.2, KDrz = 0 ;

//kontrol telusur ruangan
float KP2zR = 0.04, KP1zR = 0.01, KPrzR = 0.02, KP1rzR = 0.0001;
float KDzR = 0.1, KDrzR = 0.1; //KDz=5,KDrz=15;
int sensor[8];
int srf[4];

#define UTARA	0
#define BARAT	1
#define TIMUR	2
#define	SELATAN	3

#define DEADZONE 4
#define SEARAH_JARUM_JAM	0
#define LAWAN_JARUM_JAM		1

#define BUFFER_state_jalanArr 500

int mataangin;
int upper[4];
int lower[4];
int bataskhusus_up[4];
int bataskhusus_low[4];
int aktivasi_up[4];
int aktivasi_low[4];
int atas[4];
int bawah[4];

int khusus = 330;

int lock_acuan;
int menu;
int pewaktu;
int state_start;

int sudah_lurus;
int setcmps;
int setOrientasi;
int orientasi_Now;
int fungsi_orientasi;
int gunakan_cmps = 1;
int change;

int kan=1, kir=0;
int posisi_tpa;

int api3=0;

//bluetooth mode
int state_jalanArr[BUFFER_state_jalanArr];
int pos_state_jalanArr=0;
unsigned char state_USART2;
/*state_jalanSingle terdiri data HIGH dan LOW*/
unsigned char state_jalanSingle[2];
int temp_state_jalanArr = 60000;

//void inisialisasi_timer4() //PWM output
//{
//	GPIO_InitTypeDef configIo;
//	TIM_OCInitTypeDef configTimerOC;
//	TIM_TimeBaseInitTypeDef configTimer;
//
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //aktifkan clock timer 4
//
//	configIo.GPIO_Pin = GPIO_Pin_12;
//	configIo.GPIO_Mode = GPIO_Mode_AF;
//	configIo.GPIO_OType = GPIO_OType_PP;
//	configIo.GPIO_Speed = GPIO_Speed_50MHz;
//	configIo.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOD, &configIo);
//
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
//
//	// definisi timer 2 //
//
//
//
//	configTimer.TIM_Prescaler = 84 - 1; //frekuensi = 84Mhz /(prescaler + 1)= 1Mhz
//	configTimer.TIM_Period =  20000 - 1; //periode-1; freqPWM=1Mhz/20000=50hz
//	configTimer.TIM_ClockDivision = TIM_CKD_DIV1; 	// lihat reference manual hal.367
//	configTimer.TIM_CounterMode = TIM_CounterMode_Up;
//	//configTimer.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(TIM4, &configTimer);
//
//	//== Enable TIM2 Preload register on ARR ==//
//	TIM_ARRPreloadConfig(TIM4, ENABLE);
//
//	//== definisi OC timer 2 ==/
//
//	configTimerOC.TIM_OCMode = TIM_OCMode_PWM1;
//	configTimerOC.TIM_OutputState = TIM_OutputState_Enable;
//	//	configTimerOC.TIM_Pulse = 1000;
//	configTimerOC.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OC1Init(TIM4, &configTimerOC);
//
//	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
//
//	TIM_CtrlPWMOutputs(TIM4, ENABLE);
//
//	TIM_Cmd(TIM4, ENABLE);
//}

void inisialisasi_timer6() //Cartesian Trajectory Planning
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseInitTypeDef definisi_timebase;

	definisi_timebase.TIM_Prescaler = 84 - 1;
	definisi_timebase.TIM_Period = 40000 - 1; //periode-1; periode = 84x40000/168000000x2 = 0.04
	definisi_timebase.TIM_ClockDivision = TIM_CKD_DIV1; 	// lihat reference manual hal.367
	definisi_timebase.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &definisi_timebase);

	/* Enable TIM3 Preload register on ARR */
	TIM_ARRPreloadConfig(TIM6, ENABLE);

	TIM_Cmd(TIM6, ENABLE);

	NVIC_InitTypeDef definisi_NVIC;

	definisi_NVIC.NVIC_IRQChannel = TIM6_DAC_IRQn;
	definisi_NVIC.NVIC_IRQChannelCmd = ENABLE;
	definisi_NVIC.NVIC_IRQChannelPreemptionPriority = 1;
	//definisi_NVIC.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&definisi_NVIC);
	//
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}


void USART2_IRQHandler(void) //USART Bluetooth
{
	switch(state_USART2){
	case 0:
		if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET){
			dataKontrol = uart_receive_data(USART2);
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			state_USART2=1;
		}
		break;
	case 1:
		if(dataKontrol == 's'){
			uart_send_data(USART2,'r');
			pos_state_jalanArr=0;
			state_USART2=2;
		}
		else state_USART2 = 0;
		break;
	case 2:
		if(USART_GetFlagStatus(USART2,USART_IT_RXNE) != RESET){
			dataKontrol = uart_receive_data(USART2);
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			if(dataKontrol == 't'){
				state_USART2=3;
			}
			else state_USART2=2;
		}
		break;
	case 3:
		for (i=0 ; i<pos_state_jalanArr; i++){
				uart_send_data(USART2,state_jalanArr[i] & 0xFF); //data L
				uart_send_data(USART2,state_jalanArr[i] >> 8); //data H
		}

		state_USART2 = 4;
		break;
	case 4:
		for(i=0 ; i<pos_state_jalanArr ; i++){
			state_jalanArr[i] ='\0';
		}
		pos_state_jalanArr=0;
		state_USART2=0;
		while(1){
			berhenti();
		}
		break;
	}
}
char dataKompas, sign, dataSudut[3], *join_sudut;
int flag_data, kompas, statusKompas, statusKalibrasi, kalibrasi;


void USART3_IRQHandler(void) //USART Kompas
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		dataKompas = USART_ReceiveData(USART3);
		if (sign == 0)
		{
			if (dataKompas == 'x')sign = 1;
			else sign = 0;
		}
		else if (sign == 1)
		{
			if (dataKompas == 'y')sign = 2;
			else sign = 0;
		}
		else if (sign == 2)
		{
			switch (flag_data)
			{
			case 0 : dataSudut[0] = dataKompas; flag_data = 1; break;
			case 1 : dataSudut[1] = dataKompas; flag_data = 2; break;
			case 2 :
				dataSudut[2] = dataKompas;
				flag_data = 0;
				sign = 0;
				kompas = strtold(dataSudut, &join_sudut);
				kalibrasi = 1;
				    			//count_compas++;
				break;
			}
		}
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}

int sensor_gp[8];
unsigned char rx_buffer[4], rx_index;
char dataSlave;
unsigned char data_srf[8], posisi_api, data_api, data_max_api, data_min_api, data_cmpsL, data_cmpsH, tpa[8];
int nilai_api, data_cmpsgabung;
int penanda, data_ke;
float arf_srf0[100], arf_srf1[100], arf_srf2[100], arf_srf3[100], arf_srf4[100], arf_srf5[100], arf_srf6[100], arf_srf7[100];

void USART1_IRQHandler(void) //USART Slave
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		dataSlave = USART_ReceiveData(USART1);
		if (penanda == 0)
		{
			if (dataSlave == 0x90)
			{
				penanda = 1;
			}

			else penanda = 0;
		}
		else if (penanda == 1)
		{
			switch (data_ke)
			{
			case 0:
				data_srf[0] = dataSlave;
				sensor_gp[0] = data_srf[0] * 10;
				data_ke = 1;
				break;
			case 1:
				data_srf[1] = dataSlave;
				sensor_gp[2] = data_srf[1] * 10;
				data_ke = 2;
				break;
			case 2:
				data_srf[2] = dataSlave;
				sensor_gp[4] = data_srf[2] * 10;
				data_ke = 3;
				break;
			case 3:
				data_srf[3] = dataSlave;
				sensor_gp[6] = data_srf[3] * 10;
				data_ke = 4;
				break;
			case 4:
				data_api = dataSlave;
				nilai_api = data_api;
				data_ke = 5;
				break;
			case 5:
				posisi_api = dataSlave;
				data_ke = 6;
				break;
			case 6:
				data_max_api = dataSlave;
				data_ke = 7;
				break;
			case 7:
				data_min_api = dataSlave;
				data_ke = 8;
				break;
			case 8:
				data_cmpsH = dataSlave;
				data_ke = 9;
				break;
			case 9:
				data_cmpsL = dataSlave;
				kalibrasi = 1;
				data_ke = 0;
				penanda = 0;
				break;
			/*case 10:
				tpa[0] = dataSlave;
				data_ke = 11;
				break;
			case 11:
				tpa[1] = dataSlave;
				data_ke = 12;
				break;
			case 12:
				tpa[2] = dataSlave;
				data_ke = 13;
				break;
			case 13:
				tpa[3] = dataSlave;
				data_ke = 14;
				break;
			case 14:
				tpa[4] = dataSlave;
				data_ke = 15;
				break;
			case 15:
				tpa[5] = dataSlave;
				data_ke = 16;
				break;
			case 16:
				tpa[6] = dataSlave;
				data_ke = 17;
				break;
			case 17:
				tpa[7] = dataSlave;
				data_ke = 0;
				penanda = 0;
				break;*/
			}
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

int thd_tpa = 50;//75
int status_tpa = 0;
void tampil_tpa()
{
	//tinggi lilin 18-20
	//kondisi malam, >110 aman
	lcd_gotoxy(0,1);
	sprintf(tampil, "%3d %3d %3d %3d", tpa[0],tpa[1],tpa[2],tpa[3]);
	lcd_puts(tampil);
	lcd_gotoxy(0,2);
	sprintf(tampil, "%3d %3d %3d %3d", tpa[4],tpa[5],tpa[6],tpa[7]);
	lcd_puts(tampil);
}

int nilai_cmps()
{
	data_cmpsgabung = (data_cmpsH << 8);
	data_cmpsgabung += data_cmpsL;
	data_cmpsgabung /= 10;
	return data_cmpsgabung;
}

int setPointK;
void setKompas()
{
	setPointK = nilai_cmps();
}


int errorK;
void tampil_kompas(int x, int y) //Sudut Kompas
{
	errorK = -(setPointK - kompas);
	if (errorK > 180)errorK -= 360;
	else if (errorK < -180)errorK += 360;

	lcd_gotoxy(x, y);
	sprintf(tampil, "CMPS: %3d", nilai_cmps());
	lcd_puts(tampil);
}

unsigned char pewaktu_l, pewaktu_h;
int count_garis;
char str[50];

int tanda_case, state_jalan_lama;

void TIM6_DAC_IRQHandler()
{
	trajectoriLinier(speedServo);
	kirimInstruksiGerak(1023); //652

	//	GPIO_ToggleBits(GPIOD,GPIO_Pin_12);
	if (penanda_buzzer == 0)(GPIO_SetBits(GPIOE, GPIO_Pin_4));
	else if (penanda_buzzer >= 5)(GPIO_ResetBits(GPIOE, GPIO_Pin_4));
	penanda_ganti_muka++;
	pewaktu++;
	penanda_buzzer++;
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}


int sensor_garis;
float nilai_garis;
float rata_garis = 6.3; //malam:6.5 siang:5 garis kalibrasi
int baca_garis()//garis
{
	nilai_garis = (float)hasil_konversi_adc[8];
	nilai_garis = nilai_garis * 10 / 255;
	if (nilai_garis < rata_garis)sensor_garis = 1;
	else sensor_garis = 0;
	return sensor_garis;
}


void tampil_garis(int x, int y)
{
	baca_garis();
	lcd_gotoxy(x, y);
	sprintf(tampil, "%i", baca_garis());
	lcd_puts(tampil);
}

float max_garis = 0;
float min_garis = 10;

void kalibrasi_garis()
{
	nilai_garis = (float)hasil_konversi_adc[8] * 10 / 255;
	if (nilai_garis > max_garis)max_garis = nilai_garis;
	if (nilai_garis < min_garis)min_garis = nilai_garis;
	rata_garis = (max_garis + min_garis) / 2;
	lcd_gotoxy(0, 1);
	sprintf(tampil, "max : %1.1f", max_garis);
	lcd_puts(tampil);
	lcd_gotoxy(0, 2);
	sprintf(tampil, "min : %1.1f", min_garis);
	lcd_puts(tampil);
	lcd_gotoxy(0, 3);
	sprintf(tampil, "rata : %1.1f", rata_garis);
	lcd_puts(tampil);
}

int sensor_gp_asli[8];
float data_sensor_gp[8];
void baca_gp(int muka)
{
	int i;
	for (i = 0; i < 4; i++)
	{
//			data_sensor_gp[i]=(float)hasil_konversi_adc[i]*3/255;
//			if(data_sensor_gp[i]<0.4)data_sensor_gp[i]=0.4;	/////jangan sampai pembacaan tegangan = 0
//			else if (data_sensor_gp[i]>3)data_sensor_gp[i]=3;
//			data_sensor_gp[i]=((0.0875*data_sensor_gp[i]-0.035)/1.9)+0.0125;
//			data_sensor_gp[i]=(1/data_sensor_gp[i])*10;
//			if(data_sensor_gp[i]>600)data_sensor_gp[i]=600;

		data_sensor_gp[i] = (float)hasil_konversi_adc[i] * 3 / 255;
		if (data_sensor_gp[i] < 0.1)data_sensor_gp[i] = 0.1;	/////jangan sampai pembacaan tegangan = 0
		data_sensor_gp[i] = (2.35 / (0.2*data_sensor_gp[i] - 0.0125)) - 0.42;
		data_sensor_gp[i] = data_sensor_gp[i] * 10;
		if (data_sensor_gp[i] > 500)data_sensor_gp[i] = 500;

	}

	if (muka == 0)
	{
		sensor_gp_asli[0] = data_sensor_gp[1];
		sensor_gp_asli[1] = data_sensor_gp[0];
		sensor_gp_asli[2] = data_sensor_gp[3];
		sensor_gp_asli[3] = data_sensor_gp[2];
	}
	else if (muka == 1)
	{
		sensor_gp_asli[0] = data_sensor_gp[0];
		sensor_gp_asli[1] = data_sensor_gp[3];
		sensor_gp_asli[2] = data_sensor_gp[2];
		sensor_gp_asli[3] = data_sensor_gp[1];
	}
	else if (muka == 2)
	{
		sensor_gp_asli[0] = data_sensor_gp[3];
		sensor_gp_asli[1] = data_sensor_gp[2];
		sensor_gp_asli[2] = data_sensor_gp[1];
		sensor_gp_asli[3] = data_sensor_gp[0];
	}
	else if (muka == 3)
	{
		sensor_gp_asli[0] = data_sensor_gp[2];
		sensor_gp_asli[1] = data_sensor_gp[1];
		sensor_gp_asli[2] = data_sensor_gp[0];
		sensor_gp_asli[3] = data_sensor_gp[3];
	}
}

void gp_serong()
{
	int a;
	for (a = 4; a < 8; a++)
	{

		data_sensor_gp[a] = (float)hasil_konversi_adc[a] * 3 / 255;
		if (data_sensor_gp[a] < 0.1)data_sensor_gp[a] = 0.1;	/////jangan sampai pembacaan tegangan = 0
		data_sensor_gp[a] = (2.35 / (0.2*data_sensor_gp[a] - 0.0125)) - 0.42;
		data_sensor_gp[a] = data_sensor_gp[a] * 10;
		if (data_sensor_gp[a] > 500)data_sensor_gp[a] = 500;
	}

	 sensor_gp[7] = data_sensor_gp[4];
	 sensor_gp[5] = data_sensor_gp[5];
	 sensor_gp[3] = data_sensor_gp[6];
	 sensor_gp[1] = data_sensor_gp[7];
}

void tampil_gp(int y)
{
	lcd_gotoxy(0, y);
	sprintf(tampil, "%4d", sensor_gp[0]);
	lcd_puts(tampil);

	lcd_gotoxy(5, y);
	sprintf(tampil, "%4d", sensor_gp[2]);
	lcd_puts(tampil);

	lcd_gotoxy(10, y);
	sprintf(tampil, "%4d", sensor_gp[4]);
	lcd_puts(tampil);

	lcd_gotoxy(0, y + 1);
	sprintf(tampil, "%4d", sensor_gp[6]);
	lcd_puts(tampil);
}

void tampil_gp_asli()
{
	baca_gp(0);
	gp_serong();
	lcd_gotoxy(0, 1);
	sprintf(tampil, "%4d", sensor_gp_asli[0]);
	lcd_puts(tampil);

	lcd_gotoxy(5, 1);
	sprintf(tampil, "%4d", sensor_gp_asli[1]);
	lcd_puts(tampil);

	lcd_gotoxy(10, 1);
	sprintf(tampil, "%4d", sensor_gp_asli[2]);
	lcd_puts(tampil);

	lcd_gotoxy(0, 2);
	sprintf(tampil, "%4d", sensor_gp_asli[3]);
	lcd_puts(tampil);

	lcd_gotoxy(5, 2);
	sprintf(tampil, "%4d", sensor_gp[1]);
	lcd_puts(tampil);

	lcd_gotoxy(10, 2);
	sprintf(tampil, "%4d", sensor_gp[3]);
	lcd_puts(tampil);

	lcd_gotoxy(0, 3);
	sprintf(tampil, "%4d", sensor_gp[5]);
	lcd_puts(tampil);

	lcd_gotoxy(5, 3);
	sprintf(tampil, "%4d", sensor_gp[7]);
	lcd_puts(tampil);
}

void tampil_api()
{
	lcd_gotoxy(10, 0);
	sprintf(tampil, "%3d", posisi_api);
	lcd_puts(tampil);

	lcd_gotoxy(10, 1);
	sprintf(tampil, "%3d", nilai_api);
	lcd_puts(tampil);

	lcd_gotoxy(10, 2);
	sprintf(tampil, "%3d", data_max_api);
	lcd_puts(tampil);

	lcd_gotoxy(10, 3);
	sprintf(tampil, "%3d", data_min_api);
	lcd_puts(tampil);
}

void buzzer()
{
	int i = 0;
	for (i = 0; i < 30; i++)
	{
		GPIO_ToggleBits(GPIOE, GPIO_Pin_4);
		delay_ms(30);
	}
}


float error0, error1, error2, error1s, error2s, error3s, rotasij, rotasii;
float sudutBelok, kecepatan, langkah, tinggi;

float KPK = 0.2; //0.8 untuk kompensasi telusur biasa
void PIDk()
{
	kompas = nilai_cmps();
	errorK = (setPointK - kompas);
	if (errorK > 180)errorK -= 360;
	else if (errorK < -180)errorK += 360;

	sudutBelok = 0.5*errorK;

	if (sudutBelok >= 3)sudutBelok = 3; //15.2
	if (sudutBelok <= -3)sudutBelok = -3;
}

void PIDkr()
{
	kompas = nilai_cmps();
	errorK = (setPointK - kompas);
	if (errorK > 180)errorK -= 360;
	else if (errorK < -180)errorK += 360;

	sudutBelok = 0.5*errorK;

	if (sudutBelok >= 3)sudutBelok = 3; //15.2
	if (sudutBelok <= -3)sudutBelok = -3;
}

int lock = 0;

unsigned char miring=2; //0 = kanan , 1 = kiri, 2 = lurus

void PIDkr2()
{
	switch(lock)
	{
	case 0:
		setPointK = nilai_cmps();
		lock = 1;
		break;
	case 1:
		kompas = nilai_cmps();
		errorK = (setPointK - kompas);
		if (errorK > 180)errorK -= 360;
		else if (errorK < -180)errorK += 360;

		sudutBelok = 0.5*errorK;

		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		break;
	}

}


int sensor0, sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7, kanan, kiri, serong_kanan, serong_kiri;

int setPoint0 = 115;
int setPoint1 = 250;
int setPoint2 = 70;

float KP2 = 0.0575, KP1 = 0.0225, KPr = 0.0175; //KP2=0.0425, KP1=0.0425; KPr=0.0175
float KD = 0.3, KDr = 2; //0.5
float sensor_gp_sebelum[8];
short int lock_state_telusur;
int rotasi, rotasis;
int selisih_serong;
int state_telusur_biasa;
int batas_count;
int aktivasi_count1B;
int elast;
unsigned int perempatan;
unsigned int dekat;
float belokan = 0;
float belokan2 = 0;

void telusurKananPipih()//edit saat 90 ganti muka
{
	kecepatan = 64-perempatan; //33//64
	langkah = 2.5-dekat-belokan; //2//3.6// paling enak buat belok 2.5
	tinggi = 1; //1
	//float p=5.504, d=p*0.02;
	switch (state_telusur)
	{
		case 0:
		sensor0 = 0; sensor1 = 1; sensor2 = 2; sensor3 = 3; sensor4 = 4; sensor5 = 5; sensor6 = 6; sensor7 = 7;
		baca_gp(state_telusur);
		gp_serong();

		error0 = setPoint0z - sensor_gp[sensor0];
		if(sensor_gp[sensor1] > sensor_gp_asli[1])
		{
			error2 = setPoint2z - (sensor_gp[sensor1] - sensor_gp_asli[1]);
		}
		else if (sensor_gp[sensor1] < sensor_gp_asli[1])
		{
			error2 = setPoint2z - (sensor_gp_asli[1]-sensor_gp[sensor1]);
		}
		rotasi = (sensor_gp[sensor3] - sensor_gp[sensor1]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor1]<200)
		//{
			sudutBelok = (kape1/100)*error2 + (deriv1/100)*(error2 - error2s)  + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			if(sudutBelok > 5 || sudutBelok < -5)
			{
				belokan2 = 0.2;
				belokan = 1;
			}
			else
			{
				belokan2 = 0;
				belokan = 0;
			}
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;

		//}
/*		if(sensor_gp[sensor1]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kanan++;
		}*/
		if (sensor_gp_asli[0]<setPoint0z-100) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 3; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		break;
		case 1:
		sensor0 = 2; sensor1 = 3; sensor2 = 4; sensor3 = 5; sensor4 = 6; sensor5 = 7; sensor6 = 0; sensor7 = 1;
		baca_gp(state_telusur);
		gp_serong();

		error0 = setPoint0z - sensor_gp[sensor0];
		if(sensor_gp[sensor1] > sensor_gp_asli[1])
		{
			error2 = setPoint2z - (sensor_gp[sensor1] - sensor_gp_asli[1]);
		}
		else if (sensor_gp[sensor1] < sensor_gp_asli[1])
		{
			error2 = setPoint2z - (sensor_gp_asli[1]-sensor_gp[sensor1]);
		}
		rotasi = (sensor_gp[sensor3] - sensor_gp[sensor1]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor1]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;

			if(sudutBelok > 5 || sudutBelok < -5)
			{
				belokan2 = 0.2;
				belokan = 1;
			}
			else
			{
				belokan2 = 0;
				belokan = 0;
			}
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor1]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kanan++;
		}*/
		if (sensor_gp_asli[0]<setPoint0z-100) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 0; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
		case 2:
		sensor0 = 4; sensor1 = 5; sensor2 = 6; sensor3 = 7; sensor4 = 0; sensor5 = 1; sensor6 = 2; sensor7 = 3;
		baca_gp(state_telusur);
		gp_serong();

		error0 = setPoint0z - sensor_gp[sensor0];
		if(sensor_gp[sensor1] > sensor_gp_asli[1])
		{
			error2 = setPoint2z - (sensor_gp[sensor1] - sensor_gp_asli[1]);
		}
		else if (sensor_gp[sensor1] < sensor_gp_asli[1])
		{
			error2 = setPoint2z - (sensor_gp_asli[1]-sensor_gp[sensor1]);
		}
		rotasi = (sensor_gp[sensor3] - sensor_gp[sensor1]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor1]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;

			if(sudutBelok > 5 || sudutBelok < -5)
			{
				belokan2 = 0.2;
				belokan = 1;
			}
			else
			{
				belokan2 = 0;
				belokan = 0;
			}
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor1]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kanan++;
		}*/
		if (sensor_gp_asli[0]<setPoint0z-100) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 1; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		break;
		case 3:
		sensor0 = 6; sensor1 = 7; sensor2 = 0; sensor3 = 1; sensor4 = 2; sensor5 = 3; sensor6 = 4; sensor7 = 5;
		baca_gp(state_telusur);
		gp_serong();

		error0 = setPoint0z - sensor_gp[sensor0];
		if(sensor_gp[sensor1] > sensor_gp_asli[1])
		{
			error2 = setPoint2z - (sensor_gp[sensor1] - sensor_gp_asli[1]);
		}
		else if (sensor_gp[sensor1] < sensor_gp_asli[1])
		{
			error2 = setPoint2z - (sensor_gp_asli[1]-sensor_gp[sensor1]);
		}
		rotasi = (sensor_gp[sensor3] - sensor_gp[sensor1]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor1]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;

			if(sudutBelok > 5 || sudutBelok < -5)
			{
				belokan2 = 0.2;
				belokan = 1;
			}
			else
			{
				belokan2 = 0;
				belokan = 0;
			}
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor1]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kanan++;
		}*/
		if (sensor_gp_asli[0]<setPoint0z-100) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 2; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	}
}

void telusurKiriPipih()//edited mas jafar 2019
{
	kecepatan = 64; //55.2
	langkah = 3.5-belokan;//3//3.2
	tinggi = 0.8+belokan2; //1

	switch (state_telusur)
	{
		case 0:
		sensor0 = 0; sensor1 = 1; sensor2 = 2; sensor3 = 3; sensor4 = 4; sensor5 = 5; sensor6 = 6; sensor7 = 7;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0z - sensor_gp[sensor0];

		if(sensor_gp[sensor7] > sensor_gp_asli[3])
		{
			error2 = 90 - (sensor_gp[sensor7] - sensor_gp_asli[3]); //90
		}
		else if (sensor_gp[sensor7] < sensor_gp_asli[3])
		{
			error2 = 90 - (sensor_gp_asli[3]-sensor_gp[sensor7]);
		}
		if(sensor_gp[sensor7]>300 && sensor_gp_asli[3]>300){
			error2=-200;
		}
		rotasi = (sensor_gp[sensor5] - sensor_gp[sensor7]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor7]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2 20 awal 8
			if (sudutBelok <= -8)sudutBelok = -8;
			if(sudutBelok > 5 || sudutBelok < -5)
			{
				belokan2 = 0.2;
				belokan = 1;
			}
			else
			{
				belokan2 = 0;
				belokan = 0;
			}
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor7]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kiri++;
		}*/
		if (sensor_gp_asli[0]<setPoint0z) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 1; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kanan++;
		}
		gerak(0, langkah, -sudutBelok, tinggi, kecepatan);
		break;
		case 1:
		sensor0 = 2; sensor1 = 3; sensor2 = 4; sensor3 = 5; sensor4 = 6; sensor5 = 7; sensor6 = 0; sensor7 = 1;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0z - sensor_gp[sensor0];
		if(sensor_gp[sensor7] > sensor_gp_asli[3])
		{
			error2 = 90 - (sensor_gp[sensor7] - sensor_gp_asli[3]);
		}
		else if (sensor_gp[sensor7] < sensor_gp_asli[3])
		{
			error2 = 90 - (sensor_gp_asli[3]-sensor_gp[sensor7]);
		}
		if(sensor_gp[sensor7]>300 && sensor_gp_asli[3]>300){
			error2=-200;
		}
		rotasi = (sensor_gp[sensor5] - sensor_gp[sensor7]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor7]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2 20 awal 8
			if (sudutBelok <= -8)sudutBelok = -8;
			if(sudutBelok > 5 || sudutBelok < -5)
			{
				belokan2 = 0.2;
				belokan = 1;
			}
			else
			{
				belokan2 = 0;
				belokan = 0;
			}
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor7]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kiri++;
		}*/
		if (sensor_gp_asli[0]<setPoint0z) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 2; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kanan++;
		}
		gerak(langkah, 0, -sudutBelok, tinggi, kecepatan);
		break;
		case 2:
		sensor0 = 4; sensor1 = 5; sensor2 = 6; sensor3 = 7; sensor4 = 0; sensor5 = 1; sensor6 = 2; sensor7 = 3;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0z - sensor_gp[sensor0];
		if(sensor_gp[sensor7] > sensor_gp_asli[3])
		{
			error2 = 90 - (sensor_gp[sensor7] - sensor_gp_asli[3]);
		}
		else if (sensor_gp[sensor7] < sensor_gp_asli[3])
		{
			error2 = 90 - (sensor_gp_asli[3]-sensor_gp[sensor7]);
		}
		if(sensor_gp[sensor7]>300 && sensor_gp_asli[3]>300){
			error2=-200;
		}
		rotasi = (sensor_gp[sensor5] - sensor_gp[sensor7]);
		if (error2 > 200)error2 = 200;
		//else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor7]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2 20 awal 8
			if (sudutBelok <= -8)sudutBelok = -8;
			if(sudutBelok > 5 || sudutBelok < -5)
			{
				belokan2 = 0.2;
				belokan = 1;
			}
			else
			{
				belokan2 = 0;
				belokan = 0;
			}
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor7]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kiri++;
		}*/
		if (sensor_gp_asli[0]<setPoint0z) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 3; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kanan++;
		}
		gerak(0, -langkah, -sudutBelok, tinggi, kecepatan);
		break;
		case 3:
		sensor0 = 6; sensor1 = 7; sensor2 = 0; sensor3 = 1; sensor4 = 2; sensor5 = 3; sensor6 = 4; sensor7 = 5;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0z - sensor_gp[sensor0];
		if(sensor_gp[sensor7] > sensor_gp_asli[3])
		{
			error2 = 90 - (sensor_gp[sensor7] - sensor_gp_asli[3]);
		}
		else if (sensor_gp[sensor7] < sensor_gp_asli[3])
		{
			error2 = 90 - (sensor_gp_asli[3]-sensor_gp[sensor7]);
		}
		if(sensor_gp[sensor7]>300 && sensor_gp_asli[3]>300){
			error2=-200;
		}
		rotasi = (sensor_gp[sensor5] - sensor_gp[sensor7]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor7]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2 20 awal 8
			if (sudutBelok <= -8)sudutBelok = -8;
			if(sudutBelok > 5 || sudutBelok < -5)
			{
				belokan2 = 0.2;
				belokan = 1;
			}
			else
			{
				belokan2 = 0;
				belokan = 0;
			}
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor7]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kiri++;
		}*/
		if (sensor_gp_asli[0]<setPoint0z) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 0; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kanan++;
		}
		gerak(-langkah, 0, -sudutBelok, tinggi, kecepatan);
		break;
	}
}

void telusurKanan()//edit saat 90 ganti muka
{
	kecepatan = 64; //33
	langkah = 3.6; //2
	tinggi = 1; //1
	//float p=5.504, d=p*0.02;
	switch (state_telusur)
	{
		case 0:
		sensor0 = 0; sensor1 = 1; sensor2 = 2; sensor3 = 3; sensor4 = 4; sensor5 = 5; sensor6 = 6; sensor7 = 7;
		baca_gp(state_telusur);
		gp_serong();

		error0 = setPoint0z - sensor_gp[sensor0];
		error2 = setPoint2z - (sensor_gp[sensor1] - sensor_gp_asli[1]);
		rotasi = (sensor_gp[sensor3] - sensor_gp[sensor1]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor1]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
/*		if(sensor_gp[sensor1]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kanan++;
		}*/
		gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		break;
		case 1:
		sensor0 = 2; sensor1 = 3; sensor2 = 4; sensor3 = 5; sensor4 = 6; sensor5 = 7; sensor6 = 0; sensor7 = 1;
		baca_gp(state_telusur);
		gp_serong();

		error0 = setPoint0z - sensor_gp[sensor0];
		error2 = setPoint2z - (sensor_gp[sensor1] - sensor_gp_asli[1]);
		rotasi = (sensor_gp[sensor3] - sensor_gp[sensor1]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor1]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
		case 2:
		sensor0 = 4; sensor1 = 5; sensor2 = 6; sensor3 = 7; sensor4 = 0; sensor5 = 1; sensor6 = 2; sensor7 = 3;
		baca_gp(state_telusur);
		gp_serong();

		error0 = setPoint0z - sensor_gp[sensor0];
		error2 = setPoint2z - (sensor_gp[sensor1] - sensor_gp_asli[1]);
		rotasi = (sensor_gp[sensor3] - sensor_gp[sensor1]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor1]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor1]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kanan++;
		}*/
		gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		break;
		case 3:
		sensor0 = 6; sensor1 = 7; sensor2 = 0; sensor3 = 1; sensor4 = 2; sensor5 = 3; sensor6 = 4; sensor7 = 5;
		baca_gp(state_telusur);
		gp_serong();

		error0 = setPoint0z - sensor_gp[sensor0];
		error2 = setPoint2z - (sensor_gp[sensor1] - sensor_gp_asli[1]);
		rotasi = (sensor_gp[sensor3] - sensor_gp[sensor1]);
		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor1]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor1]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kanan++;
		}*/
		gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	}
}

void telusurKiri()//editing
{
	kecepatan = 64; //55.2
	langkah = 3.2; //3
	tinggi = 1; //1

	switch (state_telusur)
	{
		case 0:
		sensor0 = 0; sensor1 = 1; sensor2 = 2; sensor3 = 3; sensor4 = 4; sensor5 = 5; sensor6 = 6; sensor7 = 7;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0z - sensor_gp[sensor0];
		error2 = 90 - (sensor_gp[sensor7] - sensor_gp_asli[3]);
		rotasi = (sensor_gp[sensor5] - sensor_gp[sensor7]);
		if (error2 > 100)error2 = 100;
		else if (error2 < -100)error2 = -100;
		//if(sensor_gp[sensor7]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor7]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kiri++;
		}*/
		gerak(0, langkah, -sudutBelok, tinggi, kecepatan);
		break;
		case 1:
		sensor0 = 2; sensor1 = 3; sensor2 = 4; sensor3 = 5; sensor4 = 6; sensor5 = 7; sensor6 = 0; sensor7 = 1;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0z - sensor_gp[sensor0];
		error2 = 90 - (sensor_gp[sensor7] - sensor_gp_asli[3]);
		rotasi = (sensor_gp[sensor5] - sensor_gp[sensor7]);
		if (error2 > 100)error2 = 100;
		else if (error2 < -100)error2 = -100;
		//if(sensor_gp[sensor7]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor7]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kiri++;
		}*/
		gerak(langkah, 0, -sudutBelok, tinggi, kecepatan);
		break;
		case 2:
		sensor0 = 4; sensor1 = 5; sensor2 = 6; sensor3 = 7; sensor4 = 0; sensor5 = 1; sensor6 = 2; sensor7 = 3;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0z - sensor_gp[sensor0];
		error2 = 90 - (sensor_gp[sensor7] - sensor_gp_asli[3]);
		rotasi = (sensor_gp[sensor5] - sensor_gp[sensor7]);
		if (error2 > 100)error2 = 100;
		else if (error2 < -100)error2 = -100;
		//else if (error2 < -200)error2 = -200;
		//if(sensor_gp[sensor7]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor7]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kiri++;
		}*/
		gerak(0, -langkah, -sudutBelok, tinggi, kecepatan);
		break;
		case 3:
		sensor0 = 6; sensor1 = 7; sensor2 = 0; sensor3 = 1; sensor4 = 2; sensor5 = 3; sensor6 = 4; sensor7 = 5;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0z - sensor_gp[sensor0];
		error2 = 90 - (sensor_gp[sensor7] - sensor_gp_asli[3]);
		rotasi = (sensor_gp[sensor5] - sensor_gp[sensor7]);
		if (error2 > 100)error2 = 100;
		else if (error2 < -100)error2 = -100;
		//if(sensor_gp[sensor7]<200)
		//{
			sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
			if (sudutBelok >= 8)sudutBelok = 8; //15.2
			if (sudutBelok <= -8)sudutBelok = -8;
			error2s = error2;
			rotasis = rotasi;
			rotasij = rotasij + rotasi;
		//}
		/*if(sensor_gp[sensor7]>200)
		{
			kecepatan = 50;
			sudutBelok = (p/100*error0) + (d/100*(error0-elast));
			elast = error0;
			if (sudutBelok >= 13.5)sudutBelok = 13.5;
			if (sudutBelok <= -13.5)sudutBelok = -13.5;
			ganti_muka++;
			ganti_kiri++;
		}*/
		gerak(-langkah, 0, -sudutBelok, tinggi, kecepatan);
		break;
	}
}

int konversi_state_telusur(int i)
{
	state_telusur = state_telusur + i;
	if (state_telusur == -1)state_telusur = 3;
	else if (state_telusur == 4)state_telusur = 0;
	else if (state_telusur == -2)state_telusur = 2;
	penanda_ganti_muka = 0;
	hitung_zigzag = 0;
	return state_telusur;
}

void konversi_gp(int state_telusur)
{
	if (state_telusur == 0)
	{
		sensor[0] = 0; sensor[1] = 1; sensor[2] = 2; sensor[3] = 3; sensor[4] = 4; sensor[5] = 5; sensor[6] = 6; sensor[7] = 7;
	}
	else if (state_telusur == 1)
	{
		sensor[0] = 2; sensor[1] = 3; sensor[2] = 4; sensor[3] = 5; sensor[4] = 6; sensor[5] = 7; sensor[6] = 0; sensor[7] = 1;
	}
	else if (state_telusur == 2)
	{
		sensor[0] = 4; sensor[1] = 5; sensor[2] = 6; sensor[3] = 7; sensor[4] = 0; sensor[5] = 1; sensor[6] = 2; sensor[7] = 3;
	}
	else if (state_telusur == 3)
	{
		sensor[0] = 6; sensor[1] = 7; sensor[2] = 0; sensor[3] = 1; sensor[4] = 2; sensor[5] = 3; sensor[6] = 4; sensor[7] = 5;
	}
}

void konversi_srf(int state_telusur)
{
	if (state_telusur == 0)
	{
		srf[0] = 0; srf[1] = 1; srf[2] = 2; srf[3] = 3;
	}
	else if (state_telusur == 1)
	{
		srf[0] = 1; srf[1] = 2; srf[2] = 3; srf[3] = 0;
	}
	else if (state_telusur == 2)
	{
		srf[0] = 2; srf[1] = 3; srf[2] = 0; srf[3] = 1;
	}
	else if (state_telusur == 3)
	{
		srf[0] = 3; srf[1] = 0; srf[2] = 1; srf[3] = 2;
	}
}

int thd_api = 150;
int deteksi_api(int arah)
{
	for (i = 0; i < arah; i++)
	{
		if (uv2_on || uv1_on || nilai_api > thd_api)
		{
			eksistansi_api2 = 1;
			break;
		}
		else eksistansi_api2 = 0;
		delay_ms(1);
	}
	return eksistansi_api2;
}

void jalan_langkah(int arah, int jumlah_langkah)
{
	kecepatan = 64; //40
	langkah = 3.2; //3
	tinggi = 1;
	//statusKompas = 0;
	hitung_langkah = 0;
	konversi_gp(arah);
	while (1)
	{
		if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
		else sudutBelok = 0;
		//sudutBelok = 0;
		if (arah == 0)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 1)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (arah == 2)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 3)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		if (hitung_langkah == jumlah_langkah || (sensor_gp[sensor[0]]<100))break;
	}
	hitung_langkah = 0;
	//state_telusur=arah;
	penanda_ganti_muka = 0;
	hitung_zigzag = 0;
}

void jalan_langkah_scan12(int arah, int jumlah_langkah) //sensor 2 peka
{
	kecepatan = 64; //40
	langkah = 3.2; //3
	tinggi = 1;
	hitung_langkah = 0;
	konversi_gp(arah);
	while (1)
	{
		if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
		else sudutBelok = 0;
		if (arah == 0)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 1)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (arah == 2)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 3)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		if (uv2_on)eksistansi_api2 = 1;
		if (uv1_on)eksistansi_api1 = 1;
		if (hitung_langkah == jumlah_langkah || (sensor_gp[sensor[0]]<100))break;
	}
	hitung_langkah = 0;
	penanda_ganti_muka = 0;
	hitung_zigzag = 0;
}

void jalan_langkah_scan2(int arah, int jumlah_langkah) //sensor 2 peka
{
	kecepatan = 64; //40
	langkah = 3.2; //3
	tinggi = 1;
	hitung_langkah = 0;
	konversi_gp(arah);
	while (1)
	{
		if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
		else sudutBelok = 0;
		if (arah == 0)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 1)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (arah == 2)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 3)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		if (uv2_on)eksistansi_api2 = 1;
		if (hitung_langkah == jumlah_langkah|| (sensor_gp[sensor[0]]<100))break;
	}
	hitung_langkah = 0;
	penanda_ganti_muka = 0;
	hitung_zigzag = 0;
}

void jalan_langkah_scan1(int arah, int jumlah_langkah) //sensor 2 peka
{
	kecepatan = 64; //40
	langkah = 3.2; //3
	tinggi = 1;
	hitung_langkah = 0;
	konversi_gp(arah);
	while (1)
	{
		if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
		else sudutBelok = 0;
		if (arah == 0)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 1)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (arah == 2)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 3)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		if (uv1_on)eksistansi_api1 = 1;
		if (hitung_langkah == jumlah_langkah|| (sensor_gp[sensor[0]]<100))break;
	}
	hitung_langkah = 0;
	penanda_ganti_muka = 0;
	hitung_zigzag = 0;
}

void geser_kanan(int muka, int jumlah_langkah)
{
	kecepatan = 64; //40
	langkah = 3.2; //3
	tinggi = 1;
	hitung_langkah = 0;
	konversi_gp(muka);

	while(1)
	{
		if (muka == 0)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (muka == 1)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		else if (muka == 2)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (muka == 3)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		if (hitung_langkah == jumlah_langkah)break;
	}

	hitung_langkah = 0;
	penanda_ganti_muka = 0;
	hitung_zigzag = 0;
}

void geser_kiri(int muka, int jumlah_langkah)
{
	kecepatan = 64; //40
	langkah = 3.2; //3
	tinggi = 1;
	hitung_langkah = 0;
	konversi_gp(muka);

	while(1)
	{
		if (muka == 0)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (muka == 1)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		else if (muka == 2)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (muka == 3)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		if (hitung_langkah == jumlah_langkah)break;
	}

	hitung_langkah = 0;
	penanda_ganti_muka = 0;
	hitung_zigzag = 0;
}

// setpoint telusur kanan muka0 80 muka1 70 muka2 80 muka3 100
// setpoint telusur kiri muka0 100 muka1 80 muka2 70 muka3 80
int speed_backup;
void telusurKanan_zigzag()
{

	kecepatan = 64; //33  37 34
	langkah = 3.2; //2  2.3 3.6 3.2 3.4
	tinggi = 1; //1.3
	//statusKompas = 0;

	switch (state_telusur)
	{
	case 0:
		sensor[0] = 0; sensor[1] = 1; sensor[2] = 2; sensor[3] = 3; sensor[4] = 4; sensor[5] = 5; sensor[6] = 6; sensor[7] = 7;
		baca_gp(state_telusur);
		gp_serong();

		selisih_serong = (sensor_gp[sensor[1]] - sensor_gp_asli[1]);
		error0 = setPoint0z - sensor_gp[sensor[0]];
		error2 = setPoint2z - selisih_serong;

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
		if (hitung_zigzag <= 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 2)) //3
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}

		if (sudutBelok >= 5)sudutBelok = 5; //15.2
		if (sudutBelok <= -5)sudutBelok = -5;

		if (sensor_gp[sensor[2]] <= 220)penanda_ganti_muka = 0;

		error2s = error2;
		rotasis = rotasi;
		rotasij = rotasij + rotasi;

		if(aktivasi_count1B == 1)
		{
			batas_count = 1;
		}
		else
		{
			batas_count = 2;
		}


		if ((error0>0&&sensor_gp_asli[0]<setPoint0z)) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 3; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		else if (penanda_ganti_muka > batas_count) //15
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 1;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka != 0 || (sensor_gp[sensor[1]]>400&&sensor_gp[sensor[2]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 3:
		sensor[0] = 6; sensor[1] = 7; sensor[2] = 0; sensor[3] = 1; sensor[4] = 2; sensor[5] = 3; sensor[6] = 4; sensor[7] = 5;
		baca_gp(state_telusur);
		gp_serong();

		selisih_serong = (sensor_gp[sensor[1]] - sensor_gp_asli[1]);
		error0 = setPoint0z - sensor_gp[sensor[0]];
		error2 = setPoint2z - selisih_serong;

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 2))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 5)sudutBelok = 5; //15.2
		if (sudutBelok <= -5)sudutBelok = -5;

		if (sensor_gp[sensor[2]] <= 220)penanda_ganti_muka = 0;

		error2s = error2;
		rotasis = rotasi;
		rotasij = rotasij + rotasi;

		if(aktivasi_count1B == 1)
		{
			batas_count = 1;
		}
		else
		{
			batas_count = 2;
		}

		if ((error0>0&&sensor_gp_asli[0]<setPoint0z)) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 2; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		else if (penanda_ganti_muka > batas_count) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka != 0 || (sensor_gp[sensor[1]]>400&&sensor_gp[sensor[2]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	case 2:
		sensor[0] = 4; sensor[1] = 5; sensor[2] = 6; sensor[3] = 7; sensor[4] = 0; sensor[5] = 1; sensor[6] = 2; sensor[7] = 3;
		baca_gp(state_telusur);
		gp_serong();

		selisih_serong = (sensor_gp[sensor[1]] - sensor_gp_asli[1]);
		error0 = setPoint0z - sensor_gp[sensor[0]];
		error2 = setPoint2z - selisih_serong;

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
		if (hitung_zigzag <= 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 2))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 5)sudutBelok = 5; //15.2
		if (sudutBelok <= -5)sudutBelok = -5;

		if (sensor_gp[sensor[2]] <= 220)penanda_ganti_muka = 0;

		error2s = error2;
		rotasis = rotasi;
		rotasij = rotasij + rotasi;

		if(aktivasi_count1B == 1)
		{
			batas_count = 1;
		}
		else
		{
			batas_count = 2;
		}

		if ((error0>0&&sensor_gp_asli[0]<setPoint0z)) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 1; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		else if (penanda_ganti_muka > batas_count) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka != 0 || (sensor_gp[sensor[1]]>400&&sensor_gp[sensor[2]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 1:
		sensor[0] = 2; sensor[1] = 3; sensor[2] = 4; sensor[3] = 5; sensor[4] = 6; sensor[5] = 7; sensor[6] = 0; sensor[7] = 1;
		baca_gp(state_telusur);
		gp_serong();

		selisih_serong = (sensor_gp[sensor[1]] - sensor_gp_asli[1]);
		error0 = setPoint0z - sensor_gp[sensor[0]];
		error2 = setPoint2z - selisih_serong;

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (error2 > 200)error2 = 200;
		else if (rotasi < -200)error2 = -200;

		sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 2))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 5)sudutBelok = 5; //15.2
		if (sudutBelok <= -5)sudutBelok = -5;

		if (sensor_gp[sensor[2]] <= 220)penanda_ganti_muka = 0;

		error2s = error2;
		rotasis = rotasi;
		rotasij = rotasij + rotasi;

		if(aktivasi_count1B == 1)
		{
			batas_count = 1;
		}
		else
		{
			batas_count = 2;
		}

		if ((error0>0&&sensor_gp_asli[0]<setPoint0z)) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 0; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		else if (penanda_ganti_muka > batas_count) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka != 0 || (sensor_gp[sensor[1]]>400&&sensor_gp[sensor[2]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	}

}

void telusurKiri_zigzag()
{
	kecepatan = 64; //55.2
	langkah = 3.2; //3
	tinggi = 1; //1
	//statusKompas = 0;

	switch (state_telusur)
	{
	case 0:
		sensor[0] = 0; sensor[1] = 1; sensor[2] = 2; sensor[3] = 3; sensor[4] = 4; sensor[5] = 5; sensor[6] = 6; sensor[7] = 7;
		baca_gp(state_telusur);
		gp_serong();

		selisih_serong = sensor_gp[sensor[7]] - sensor_gp_asli[3];
		error0 = setPoint0z - sensor_gp[sensor[0]];
		error2 = setPoint2z - selisih_serong;

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 2)) //5
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 5)sudutBelok = 5; //15.2
		if (sudutBelok <= -5)sudutBelok = -5;

		if (sensor_gp[sensor[6]] <= 220)penanda_ganti_muka = 0;

		error2s = error2;
		rotasis = rotasi;

		if (error0>0&&(sensor_gp_asli[0]<(setPoint0z)))
		{
			hitung_zigzag = 0;
			state_telusur = 1; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 2) //8
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka != 0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 3:
		sensor[0] = 6; sensor[1] = 7; sensor[2] = 0; sensor[3] = 1; sensor[4] = 2; sensor[5] = 3; sensor[6] = 4; sensor[7] = 5;
		baca_gp(state_telusur);
		gp_serong();

		selisih_serong = sensor_gp[sensor[7]] - sensor_gp_asli[3];
		error0 = setPoint0z - sensor_gp[sensor[0]];
		error2 = setPoint2z - selisih_serong;

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 2))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 5)sudutBelok = 5; //15.2
		if (sudutBelok <= -5)sudutBelok = -5;

		if (sensor_gp[sensor[6]] <= 220)penanda_ganti_muka = 0;

		error1s = error1;
		error2s = error2;

		if (error0>0&&(sensor_gp_asli[0]<(setPoint0z)))
		{
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 2) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka != 0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	case 2:
		sensor[0] = 4; sensor[1] = 5; sensor[2] = 6; sensor[3] = 7; sensor[4] = 0; sensor[5] = 1; sensor[6] = 2; sensor[7] = 3;
		baca_gp(state_telusur);
		gp_serong();

		selisih_serong = sensor_gp[sensor[7]] - sensor_gp_asli[3];
		error0 = setPoint0z - sensor_gp[sensor[0]];
		error2 = 70 - selisih_serong;

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 2))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 5)sudutBelok = 5; //15.2
		if (sudutBelok <= -5)sudutBelok = -5;

		if (sensor_gp[sensor[6]] <= 220)penanda_ganti_muka = 0;

		error1s = error1;
		error2s = error2;

		if (error0>0&&(sensor_gp_asli[0]<(setPoint0z)))
		{
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 2) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 1;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka != 0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 1:
		sensor[0] = 2; sensor[1] = 3; sensor[2] = 4; sensor[3] = 5; sensor[4] = 6; sensor[5] = 7; sensor[6] = 0; sensor[7] = 1;
		baca_gp(state_telusur);
		gp_serong();

		selisih_serong = sensor_gp[sensor[7]] - sensor_gp_asli[3];
		error0 = setPoint0z - sensor_gp[sensor[0]];
		error2 = setPoint2z - selisih_serong;

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 2))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 5)sudutBelok = 5; //15.2
		if (sudutBelok <= -5)sudutBelok = -5;

		if (sensor_gp[sensor[6]] <= 220)penanda_ganti_muka = 0;

		error1s = error1;
		error2s = error2;

		if (error0>0&&(sensor_gp_asli[0]<(setPoint0z)))
		{
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 2) //70
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka != 0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	}

}

int count_srf_muka;
int setPoint0zr = 120;
int setPoint2zr = 60;
int tanda_coba;
void kananruang_baru()
{
	kecepatan = 50; //33  37 34
	langkah = 2.5; //2  2.3
	tinggi = 1; //1.3

	switch (state_telusur)
	{
	case 0:
		sensor[0] = 0; sensor[1] = 1; sensor[2] = 2; sensor[3] = 3; sensor[4] = 4; sensor[5] = 5; sensor[6] = 6; sensor[7] = 7;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - sensor_gp[sensor[2]];

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (rotasi > 200)rotasi = 200;
		else if (rotasi < -200)rotasi = -200;

		if (statusKompas == 1&&statusKalibrasi == 1)
		{
			errorK = -(setPointK - kompas);
			if (errorK > 180)errorK -= 360;
			else if (errorK < -180)errorK += 360;

			sudutBelok = KP2zR*error2  + KDzR*(error2 - error2s) + KPrzR*rotasi + KDrzR*(rotasi - rotasis) + KPK*errorK;
		}
		else if((penanda_ganti_muka > 1 && penanda_ganti_muka <= 4) || (abs(rotasi) >= 200 && abs(error2) < 50))
		{
			sudutBelok = KP1zR*error2  + KDzR*(error2 - error2s) + KP1rzR*rotasi + KDrzR*(rotasi - rotasis);
//			tanda_coba=1;
//			led_api_on;
		}
		else
		{
			sudutBelok = KP2zR*error2  + KDzR*(error2 - error2s) + KPrzR*rotasi + KDrzR*(rotasi - rotasis);
//			tanda_coba=2;
//			led_api_off;
		}

		if (hitung_zigzag < 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 1)) //3
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}

		if (sudutBelok >= 2.5)sudutBelok = 2.5; //15.2
		if (sudutBelok <= -2.5)sudutBelok = -2.5;

		if (sensor_gp[sensor[2]] <= 180 || sensor_gp_asli[1] <= 180)penanda_ganti_muka = 0;

		//if (sensor_gp_asli[1]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;

		//if (sensor_gp_asli[0]<setPoint0zr)
		if (error0 > 0)count_srf_muka++;
		else count_srf_muka = 0;

		if (count_srf_muka > 15 || sensor_gp_asli[0] < setPoint0zr) //2
		{
			count_srf_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 3; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
			penanda_buzzer=0;
		}
//		if (sensor_gp_asli[0]<setPoint0zr&&(sensor_gp_asli[1]<150||sensor_gp_asli[1]>399))
//		{
//			hitung_zigzag=0;
//			state_telusur=3; //ganti telusur jika depan mepet
//			ganti_muka++;
//			ganti_kiri++;
//		}
//		else if (sensor_gp_asli[0]<setPoint0zr&&(sensor_gp_asli[1]>=150&&sensor_gp_asli[1]<=399))
//		{
//			penanda_ganti_muka=0;
//			hitung_zigzag=0;
//			state_telusur=1;
//			ganti_muka++;
//			ganti_kanan++;
//		}
		else if (penanda_ganti_muka > 6) //15
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 1;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka != 0)
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
			else sudutBelok = 0;
		}
		gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 3:
		sensor[0] = 6; sensor[1] = 7; sensor[2] = 0; sensor[3] = 1; sensor[4] = 2; sensor[5] = 3; sensor[6] = 4; sensor[7] = 5;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - sensor_gp[sensor[2]];

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);
		if (rotasi > 200)rotasi = 200;
		else if (rotasi < -200)rotasi = -200;

		if (statusKompas == 1&&statusKalibrasi == 1)
		{
			errorK = -(setPointK - kompas);
			if (errorK > 180)errorK -= 360;
			else if (errorK < -180)errorK += 360;

			sudutBelok = KP2zR*error2  + KDzR*(error2 - error2s) + KPrzR*rotasi + KDrzR*(rotasi - rotasis) + KPK*errorK;
		}
		else if((penanda_ganti_muka > 1 && penanda_ganti_muka < 4) || (abs(rotasi) > 280 && abs(error2) < 50))
		{
			sudutBelok = KP1zR*error2  + KDzR*(error2 - error2s) + KP1rzR*rotasi + KDrzR*(rotasi - rotasis);
//			tanda_coba=1;
//			led_api_on;
		}
		else
		{
			sudutBelok = KP2zR*error2  + KDzR*(error2 - error2s) + KPrzR*rotasi + KDrzR*(rotasi - rotasis);
//			tanda_coba=2;
//			led_api_off;
		}


		if (hitung_zigzag < 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}

		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[2]] <= 180 || sensor_gp_asli[1] <= 180)penanda_ganti_muka = 0;
		//if (sensor_gp_asli[1]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;

		//if (sensor_gp_asli[0]<setPoint0zr)
		//if (error0>0)
		if (error0 > 0)count_srf_muka++;
		else count_srf_muka = 0;

		if (count_srf_muka > 15 || sensor_gp_asli[0] < setPoint0zr)
		{
			count_srf_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 2; //ganti telusur jika depan mepet
			penanda_buzzer = 0;
			ganti_muka++;
			ganti_kiri++;
		}
//		if (sensor_gp_asli[0]<setPoint0zr&&(sensor_gp_asli[1]<150||sensor_gp_asli[1]>399))
//		{
//			hitung_zigzag=0;
//			state_telusur=2; //ganti telusur jika depan mepet
//			ganti_muka++;
//			ganti_kiri++;
//		}
//		else if (sensor_gp_asli[0]<setPoint0zr&&(sensor_gp_asli[1]>=150&&sensor_gp_asli[1]<=399))
//		{
//			penanda_ganti_muka=0;
//			hitung_zigzag=0;
//			state_telusur=0;
//			ganti_muka++;
//			ganti_kanan++;
//		}
		else if (penanda_ganti_muka > 6) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka != 0)
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
			else sudutBelok = 0;
		}
		gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	case 2:
		sensor[0] = 4; sensor[1] = 5; sensor[2] = 6; sensor[3] = 7; sensor[4] = 0; sensor[5] = 1; sensor[6] = 2; sensor[7] = 3;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - sensor_gp[sensor[2]];

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (rotasi > 200)rotasi = 200;
		else if (rotasi < -200)rotasi = -200;

		if (statusKompas == 1&&statusKalibrasi == 1)
		{
			errorK = -(setPointK - kompas);
			if (errorK > 180)errorK -= 360;
			else if (errorK < -180)errorK += 360;

			sudutBelok = KP2zR*error2  + KDzR*(error2 - error2s) + KPrzR*rotasi + KDrzR*(rotasi - rotasis) + KPK*errorK;
		}
		else if((penanda_ganti_muka > 1 && penanda_ganti_muka < 4) || (abs(rotasi) > 280 && abs(error2) < 50))
		{
			sudutBelok = KP1zR*error2  + KDzR*(error2 - error2s) + KP1rzR*rotasi + KDrzR*(rotasi - rotasis);
//			tanda_coba=1;
//			led_api_on;
		}
		else
		{
			sudutBelok = KP2zR*error2  + KDzR*(error2 - error2s) + KPrzR*rotasi + KDrzR*(rotasi - rotasis);
//			tanda_coba=2;
//			led_api_off;
		}


		if (hitung_zigzag < 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}

		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[2]] <= 180 || sensor_gp_asli[1] <= 180)penanda_ganti_muka = 0;
		//if (sensor_gp_asli[1]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;

		//if (sensor_gp_asli[0]<setPoint0zr)
		//if (error0>0)
		if (error0 > 0)count_srf_muka++;
		else count_srf_muka = 0;

		if (count_srf_muka > 15 || sensor_gp_asli[0] < setPoint0zr)
		{
			count_srf_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 1; //ganti telusur jika depan mepet
			penanda_buzzer = 0;
			ganti_muka++;
			ganti_kiri++;
		}
//		if (sensor_gp_asli[0]<setPoint0zr&&(sensor_gp_asli[1]<150||sensor_gp_asli[1]>399))
//		{
//			hitung_zigzag=0;
//			state_telusur=1; //ganti telusur jika depan mepet
//			ganti_muka++;
//			ganti_kiri++;
//		}
//		else if (sensor_gp_asli[0]<setPoint0zr&&(sensor_gp_asli[1]>=150&&sensor_gp_asli[1]<=399))
//		{
//			penanda_ganti_muka=0;
//			hitung_zigzag=0;
//			state_telusur=3;
//			ganti_muka++;
//			ganti_kanan++;
//		}
		else if (penanda_ganti_muka > 6) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka != 0)
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
			else sudutBelok = 0;
		}
		gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 1:
		sensor[0] = 2; sensor[1] = 3; sensor[2] = 4; sensor[3] = 5; sensor[4] = 6; sensor[5] = 7; sensor[6] = 0; sensor[7] = 1;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];

		error2 = setPoint2zr - sensor_gp[sensor[2]];

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (rotasi > 200)rotasi = 200;
		else if (rotasi < -200)rotasi = -200;

		if (statusKompas == 1&&statusKalibrasi == 1)
		{
			errorK = -(setPointK - kompas);
			if (errorK > 180)errorK -= 360;
			else if (errorK < -180)errorK += 360;

			sudutBelok = KP2zR*error2  + KDzR*(error2 - error2s) + KPrzR*rotasi + KDrzR*(rotasi - rotasis) + KPK*errorK;
		}
		else if((penanda_ganti_muka > 1 && penanda_ganti_muka < 4) || (abs(rotasi) > 280 && abs(error2) < 50))
		{
			sudutBelok = KP1zR*error2  + KDzR*(error2 - error2s) + KP1rzR*rotasi + KDrzR*(rotasi - rotasis);
//			tanda_coba = 1;
//			led_api_on;
		}
		else
		{
			sudutBelok = KP2zR*error2  + KDzR*(error2 - error2s) + KPrzR*rotasi + KDrzR*(rotasi - rotasis);
//			tanda_coba=2;
//			led_api_off;
		}

		if (hitung_zigzag < 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}

		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[2]] <= 180 || sensor_gp_asli[1] <= 180)penanda_ganti_muka = 0;
		//if (sensor_gp_asli[1]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;

		//if (sensor_gp_asli[0]<setPoint0zr)
		//if (error0>0)
		if (error0 > 0)count_srf_muka++;
		else count_srf_muka = 0;

		if (count_srf_muka > 15 || sensor_gp_asli[0] < setPoint0zr)
		{
			count_srf_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 0; //ganti telusur jika depan mepet
			penanda_buzzer = 0;
			ganti_muka++;
			ganti_kiri++;
		}

//		if (sensor_gp_asli[0]<setPoint0zr&&(sensor_gp_asli[1]<150||sensor_gp_asli[1]>399))
//		{
//			hitung_zigzag=0;
//			state_telusur=0; //ganti telusur jika depan mepet
//			ganti_muka++;
//			ganti_kiri++;
//		}
//		else if (sensor_gp_asli[0]<setPoint0zr&&(sensor_gp_asli[1]>=150&&sensor_gp_asli[1]<=399))
//		{
//			penanda_ganti_muka=0;
//			hitung_zigzag=0;
//			state_telusur=2;
//			ganti_muka++;
//			ganti_kanan++;
//		}
		else if (penanda_ganti_muka > 6) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka != 0)
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
			else sudutBelok = 0;
		}
		gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	}
}

void telusurKiri_zigzag_ruang()
{
	kecepatan = 50; //55.2
	langkah = 2.5; //3
	tinggi = 1; //1
	//statusKompas = 0;

	switch (state_telusur)
	{
	case 0:
		sensor[0] = 0; sensor[1] = 1; sensor[2] = 2; sensor[3] = 3; sensor[4] = 4; sensor[5] = 5; sensor[6] = 6; sensor[7] = 7;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[7]] - sensor_gp_asli[3]);

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 1)) //5
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[6]]<=180||sensor_gp_asli[3]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;

		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 1; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 3) //8
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 3:
		sensor[0] = 6; sensor[1] = 7; sensor[2] = 0; sensor[3] = 1; sensor[4] = 2; sensor[5] = 3; sensor[6] = 4; sensor[7] = 5;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[7]] - sensor_gp_asli[3]);

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[6]]<=180||sensor_gp_asli[3]<=180)penanda_ganti_muka=0;

		error1s = error1;
		error2s = error2;


		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 3) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	case 2:
		sensor[0] = 4; sensor[1] = 5; sensor[2] = 6; sensor[3] = 7; sensor[4] = 0; sensor[5] = 1; sensor[6] = 2; sensor[7] = 3;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[7]] - sensor_gp_asli[3]);

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[6]]<=180||sensor_gp_asli[3]<=180)penanda_ganti_muka=0;

		error1s = error1;
		error2s = error2;


		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 3) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 1;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 1:
		sensor[0] = 2; sensor[1] = 3; sensor[2] = 4; sensor[3] = 5; sensor[4] = 6; sensor[5] = 7; sensor[6] = 0; sensor[7] = 1;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[7]] - sensor_gp_asli[3]);

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[6]]<=180||sensor_gp_asli[3]<=180)penanda_ganti_muka=0;

		error1s = error1;
		error2s = error2;



		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 3) //70
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	}

}

void telusurKanan_zigzag_ruang()
{
	kecepatan = 50; //33  37 34
	langkah = 2.5; //2  2.3 3.6 3.2 3.4
	tinggi = 1; //1.3
	//statusKompas = 0;

	switch (state_telusur)
	{
	case 0:
		sensor[0] = 0; sensor[1] = 1; sensor[2] = 2; sensor[3] = 3; sensor[4] = 4; sensor[5] = 5; sensor[6] = 6; sensor[7] = 7;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[1]] - sensor_gp_asli[1]);

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
		if (hitung_zigzag <= 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 1)) //3
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}

		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[2]]<=180||sensor_gp_asli[1]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;
		rotasij = rotasij + rotasi;

		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr))) //sensor_gp_asli[0]<(setPoint0z+60)
		{
			hitung_zigzag = 0;
			state_telusur = 3; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		else if (penanda_ganti_muka > 3) //15
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 1;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[1]]>400&&sensor_gp[sensor[2]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 3:
		sensor[0] = 6; sensor[1] = 7; sensor[2] = 0; sensor[3] = 1; sensor[4] = 2; sensor[5] = 3; sensor[6] = 4; sensor[7] = 5;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[1]] - sensor_gp_asli[1]);

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[2]]<=180||sensor_gp_asli[1]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;
		rotasij = rotasij + rotasi;


		if(error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 2; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		else if (penanda_ganti_muka > 3) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[1]]>400&&sensor_gp[sensor[2]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	case 2:
		sensor[0] = 4; sensor[1] = 5; sensor[2] = 6; sensor[3] = 7; sensor[4] = 0; sensor[5] = 1; sensor[6] = 2; sensor[7] = 3;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[1]] - sensor_gp_asli[1]);

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);
		if (hitung_zigzag <= 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[2]]<=180||sensor_gp_asli[1]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;
		rotasij = rotasij + rotasi;


		if (error0>0&&(sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 1; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		else if (penanda_ganti_muka > 3) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[1]]>400&&sensor_gp[sensor[2]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 1:
		sensor[0] = 2; sensor[1] = 3; sensor[2] = 4; sensor[3] = 5; sensor[4] = 6; sensor[5] = 7; sensor[6] = 0; sensor[7] = 1;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[1]] - sensor_gp_asli[1]);

		//if (sensor_gp[sensor[2]]>240)error2=0;

		rotasi = (sensor_gp[sensor[3]] - sensor_gp[sensor[1]]);

		if (error2 > 200)error2 = 200;
		else if (rotasi < -200)error2 = -200;

		sudutBelok = (kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[2]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[2]]<=180||sensor_gp_asli[1]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;
		rotasij = rotasij + rotasi;


		if (error0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 0; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kiri++;
		}
		else if (penanda_ganti_muka > 3) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kanan++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[1]]>400&&sensor_gp[sensor[2]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	}
}

int lurus_tembok;
int errorT;
void luruskan_tembok_kiri()
{
	baca_gp(state_telusur);
	gp_serong();
	if(state_telusur == 0)
	{
		errorT = (sensor_gp[7] - sensor_gp[5]);
	}
	else if (state_telusur == 1)
	{
		errorT = (sensor_gp[1] - sensor_gp[7]);
	}
	else if (state_telusur == 2)
	{
		errorT = (sensor_gp[3] - sensor_gp[1]);
	}
	else if(state_telusur == 3)
	{
		errorT = (sensor_gp[5] - sensor_gp[3]);
	}


	if(errorT<=10 && errorT>=-10)
	{
		gerakStatic(0, 0, 0, 0, 0, 0, 25);
		lurus_tembok = 1;
		hitung_langkah = 0;
	}

	if(errorT>20)
	{
		gerak(0, 0, 5, 1, 25);
		lurus_tembok = 0;
	}
	else if(errorT<-20)
	{
		gerak(0, 0, -5, 1, 25);
		lurus_tembok = 0;
	}


	if(errorT<=20 && errorT>10)
	{
		gerak(0, 0, 1, 1, 15);
		lurus_tembok = 0;
	}
	else if(errorT>=-20 && errorT<-10)
	{
		gerak(0, 0, -1, 1, 15);
		lurus_tembok = 0;
	}
}

void luruskan_tembok_kanan()
{
	baca_gp(state_telusur);
	gp_serong();
	if(state_telusur == 0)
	{
		errorT = (sensor_gp[3] - sensor_gp[1]);
	}
	else if (state_telusur == 1)
	{
		errorT = (sensor_gp[5] - sensor_gp[3]);
	}
	else if (state_telusur == 2)
	{
		errorT = (sensor_gp[7] - sensor_gp[5]);
	}
	else if(state_telusur == 3)
	{
		errorT = (sensor_gp[1] - sensor_gp[7]);
	}


	if(errorT<=10 && errorT>=-10)
	{
		gerakStatic(0, 0, 0, 0, 0, 0, 25);
		hitung_langkah = 0;
		lurus_tembok = 1;
	}

	if(errorT>20)
	{
		gerak(0, 0, 5, 1, 25);
		lurus_tembok = 0;
	}
	else if(errorT<-20)
	{
		gerak(0, 0, -5, 1, 25);
		lurus_tembok = 0;
	}


	if(errorT<=20 && errorT>10)
	{
		gerak(0, 0, 1, 1, 15);
		lurus_tembok = 0;
	}
	else if(errorT>=-20 && errorT<-10)
	{
		gerak(0, 0, -1, 1, 15);
		lurus_tembok = 0;
	}

}


void telusurKiri_zigzag_detek_floor()
{
//	kecepatan = 50; //55.2
//	langkah = 1.2; //3
//	tinggi = 0.8; //1
	//statusKompas = 0;

	switch (state_telusur)
	{
	case 0:
		sensor[0] = 0; sensor[1] = 1; sensor[2] = 2; sensor[3] = 3; sensor[4] = 4; sensor[5] = 5; sensor[6] = 6; sensor[7] = 7;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[7]] - sensor_gp_asli[3]);

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 1)) //5
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[6]]<=180||sensor_gp_asli[3]<=180)penanda_ganti_muka=0;

		error2s = error2;
		rotasis = rotasi;

		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 1; //ganti telusur jika depan mepet
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 3) //8
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 3:
		sensor[0] = 6; sensor[1] = 7; sensor[2] = 0; sensor[3] = 1; sensor[4] = 2; sensor[5] = 3; sensor[6] = 4; sensor[7] = 5;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[7]] - sensor_gp_asli[3]);

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[6]]<=180||sensor_gp_asli[3]<=180)penanda_ganti_muka=0;

		error1s = error1;
		error2s = error2;


		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 3) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	case 2:
		sensor[0] = 4; sensor[1] = 5; sensor[2] = 6; sensor[3] = 7; sensor[4] = 0; sensor[5] = 1; sensor[6] = 2; sensor[7] = 3;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[7]] - sensor_gp_asli[3]);

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[6]]<=180||sensor_gp_asli[3]<=180)penanda_ganti_muka=0;

		error1s = error1;
		error2s = error2;


		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 3;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 3) //80
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 1;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		break;
	case 1:
		sensor[0] = 2; sensor[1] = 3; sensor[2] = 4; sensor[3] = 5; sensor[4] = 6; sensor[5] = 7; sensor[6] = 0; sensor[7] = 1;
		baca_gp(state_telusur);
		gp_serong();
		error0 = setPoint0zr - sensor_gp[sensor[0]];
		error2 = setPoint2zr - (sensor_gp[sensor[7]] - sensor_gp_asli[3]);

		//if (sensor_gp[sensor[6]]>240)error2=0;

		rotasi = (sensor_gp[sensor[5]] - sensor_gp[sensor[7]]);

		if (error2 > 200)error2 = 200;
		else if (error2 < -200)error2 = -200;

		sudutBelok = -(kape1/100)*error2  + (deriv1/100)*(error2 - error2s) + p_rot1*rotasi + deriv_rot1*(rotasi - rotasis);

		if (hitung_zigzag <= 1 || (sensor_gp[sensor[6]] > 220&&hitung_zigzag <= 1))
		{
			penanda_ganti_muka = 0;
			sudutBelok = 0;
		}
		if (sudutBelok >= 3)sudutBelok = 3; //15.2
		if (sudutBelok <= -3)sudutBelok = -3;

		if (sensor_gp[sensor[6]]<=180||sensor_gp_asli[3]<=180)penanda_ganti_muka=0;

		error1s = error1;
		error2s = error2;



		if (error0>0 && (sensor_gp_asli[0]<(setPoint0zr)))
		{
			hitung_zigzag = 0;
			state_telusur = 2;
			ganti_muka++;
			ganti_kanan++;
		}
		else if (penanda_ganti_muka > 3) //70
		{
			penanda_ganti_muka = 0;
			hitung_zigzag = 0;
			state_telusur = 0;
			ganti_muka++;
			ganti_kiri++;
		}
		if (penanda_ganti_muka!=0 || (sensor_gp[sensor[7]]>400&&sensor_gp[sensor[6]]<200))
		{
			if (statusKompas == 1&&statusKalibrasi == 1)PIDk();
			else sudutBelok = 0;
		}
		gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		break;
	}

}


void gerak_lewat_tangga(int arah){
	kecepatan = 64-perempatan-50; //40
	langkah = 1.2;//1.2;//1.0;//1.2; //3
	tinggi = 2.4;//2.4;//2.5;//2.1;
	PIDkr2();

	if (arah == 0)lewat_tangga(0, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1)lewat_tangga(langkah, 0, sudutBelok, tinggi, kecepatan);
	else if (arah == 2)lewat_tangga(0, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3)lewat_tangga(-langkah, 0, sudutBelok, tinggi, kecepatan);
	penanda_ganti_muka = 0;

}

void maju(int arah)
{
	kecepatan = 64-perempatan; //40
	langkah = 3.2; //3
	tinggi = 1;
//	if (statusKompas==1&&statusKalibrasi==1)PIDkr();
//	else sudutBelok=0;
//	setPointK = 300;
	PIDkr2();
	//PIDkr();

		if (arah == 0)gerak(0, langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 1)gerak(langkah, 0, sudutBelok, tinggi, kecepatan);
		else if (arah == 2)gerak(0, -langkah, sudutBelok, tinggi, kecepatan);
		else if (arah == 3)gerak(-langkah, 0, sudutBelok, tinggi, kecepatan);
		penanda_ganti_muka = 0;



}

void serong(int arah, int arah_serong)
{
	kecepatan = 40.2; //60.2
	langkah = 2.1; //3
	tinggi = 1.3;
	state_telusur = arah;

	if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
	else sudutBelok = 0;

	if (arah == 0&&arah_serong == -1)gerak(-langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1&&arah_serong == -1)gerak(langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 2&&arah_serong == -1)gerak(langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3&&arah_serong == -1)gerak(-langkah, -langkah, sudutBelok, tinggi, kecepatan);

	else if (arah == 0&&arah_serong == 1)gerak(langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1&&arah_serong == 1)gerak(langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 2&&arah_serong == 1)gerak(-langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3&&arah_serong == 1)gerak(-langkah, langkah, sudutBelok, tinggi, kecepatan);
}

unsigned char unfloor;
unsigned char state_floor;
void serong_floor(int arah, int arah_serong)
{

	state_telusur = arah;

	if(unfloor == 0){
		if(nilai_cmps() < kalibrasi_serong_selatan-10 || nilai_cmps() > kalibrasi_serong_selatan+10){
			muter_acuankompas(kalibrasi_serong_selatan);
		}
	}

	else if(unfloor == 1){
		if(nilai_cmps() < kalibrasi_serong_timur-10 || nilai_cmps() > kalibrasi_serong_timur+10){
			muter_acuankompas(kalibrasi_serong_timur);
		}
	}

	else if(unfloor == 2){
		if(nilai_cmps() < kalibrasi_serong_utara-10 || nilai_cmps() > kalibrasi_serong_utara+10){
			muter_acuankompas(kalibrasi_serong_utara);
		}
	}


	if (statusKompas == 1&&statusKalibrasi == 1)PIDkr();
	else sudutBelok = 0;

	if (arah == 0&&arah_serong == -1)gerak(-langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1&&arah_serong == -1)gerak(langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 2&&arah_serong == -1)gerak(langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3&&arah_serong == -1)gerak(-langkah, -langkah, sudutBelok, tinggi, kecepatan);

	else if (arah == 0&&arah_serong == 1)gerak(langkah, langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 1&&arah_serong == 1)gerak(langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 2&&arah_serong == 1)gerak(-langkah, -langkah, sudutBelok, tinggi, kecepatan);
	else if (arah == 3&&arah_serong == 1)gerak(-langkah, langkah, sudutBelok, tinggi, kecepatan);
}

int tujuan_putar;
void putar(int sudut_tujuan,int w, int speed)
{
	tujuan_putar=kompas+sudut_tujuan;
	if (tujuan_putar>360)tujuan_putar=tujuan_putar-360;
	else if(tujuan_putar<0)tujuan_putar=360+tujuan_putar;

	hitung_langkah=0;
	while(1)
	{
		if (sudut_tujuan>0)gerak(0,0,-w,1,speed);
		else gerak(0,0,w,1,speed);
		if(hitung_langkah==2)break;
	}
	while(1)
	{
		if (sudut_tujuan>0)gerak(0,0,-w,1,speed);
		else gerak(0,0,w,1,speed);

		if(kompas<=tujuan_putar+1&&kompas>=tujuan_putar-1)
		{
			gerakStatic(0,0,0,0,0,0,30);
			break;
		}
	}
}

void putar_biasa(int sudutputar)
{
	int i;
	float arah_putar;
	if(sudutputar==90){i=40;arah_putar=13.5;}
	else if (sudutputar==180){i=80;arah_putar=9;}
	else if (sudutputar==270){i=120;arah_putar=9;}
	else if (sudutputar==-90){i=40;arah_putar=-9;}
	else if (sudutputar==-180){i=80;arah_putar=-9;}
	else if (sudutputar==-270){i=120;arah_putar=-9;}
	else if (sudutputar==360){i=155;arah_putar=9;}

	if(pewaktu>=i)goto label;
	pewaktu=0;
	while(pewaktu<i)
	{
		gerak(0,0,arah_putar,1,25);
	}
	label:
	gerakStatic(0,0,0,0,0,0,30);
}

int tembak_cmps()//tembak cmps
{
	atas[UTARA] = 78;
	atas[BARAT] = 346;
	atas[TIMUR] = 153;
	atas[SELATAN] = 244;

	bawah[UTARA] = 0;
	bawah[BARAT] = 256;
	bawah[TIMUR] = 63;
	bawah[SELATAN] = 154;

	if((nilai_cmps()<=atas[SELATAN]&&nilai_cmps()>=bawah[SELATAN]))
	{
		mataangin = SELATAN;
	}
	else if((nilai_cmps()<=atas[TIMUR]&&nilai_cmps()>=bawah[TIMUR]) || nilai_cmps()>=khusus)
	{
		mataangin = TIMUR;
	}
	else if(nilai_cmps()<=atas[UTARA]&&nilai_cmps()>=bawah[UTARA])
	{
		mataangin = UTARA;
	}
	else if(nilai_cmps()<=atas[BARAT]&&nilai_cmps()>=bawah[BARAT])
	{
		mataangin = BARAT;
	}
	else
	{
		mataangin = DEADZONE;
	}
	return mataangin;
}


void arah_mana()
{
	//UTARA
	upper[UTARA] = kalibrasi_utara + 45;
	lower[UTARA] = kalibrasi_utara - 45;
	if (upper[UTARA] > 360)
	{
		bataskhusus_up[UTARA] = upper[UTARA] - 360;
		upper[UTARA] = 360;
		aktivasi_up[UTARA] = 1;
	}
	else if (lower[UTARA] < 0)
	{
		bataskhusus_low[UTARA] = lower[UTARA] + 360;
		lower[UTARA] = 0;
		aktivasi_low[UTARA] = 1;
	}

	if (aktivasi_up[UTARA] == 1)
	{
		if ((nilai_cmps() > lower[UTARA] && nilai_cmps() < upper[UTARA]) || nilai_cmps() < bataskhusus_up[UTARA])
		{
			mataangin = UTARA;
		}
	}
	else if (aktivasi_low[UTARA] == 1)
	{
		if ((nilai_cmps() > lower[UTARA] && nilai_cmps() < upper[UTARA]) || nilai_cmps() > bataskhusus_low[UTARA])
		{
			mataangin = UTARA;
		}
	}
	else if (nilai_cmps()>lower[UTARA] && nilai_cmps() < upper[UTARA])
	{
		mataangin = UTARA;
	}

//SELATAN
	upper[SELATAN] = kalibrasi_selatan + 45;
	lower[SELATAN] = kalibrasi_selatan - 45;
	if (upper[SELATAN] > 360)
	{
		bataskhusus_up[SELATAN] = upper[SELATAN] - 360;
		upper[SELATAN] = 360;
		aktivasi_up[SELATAN] = 1;
	}
	else if (lower[SELATAN] < 0)
	{
		bataskhusus_low[SELATAN] = lower[SELATAN] + 360;
		lower[SELATAN] = 0;
		aktivasi_low[SELATAN] = 1;
	}

	if (aktivasi_up[SELATAN] == 1)
	{
		if ((nilai_cmps() > lower[SELATAN] && nilai_cmps() < upper[SELATAN]) || nilai_cmps() < bataskhusus_up[SELATAN])
		{
			mataangin = SELATAN;
		}
	}
	else if (aktivasi_low[SELATAN] == 1)
	{
		if ((nilai_cmps() > lower[SELATAN] && nilai_cmps() < upper[SELATAN]) || nilai_cmps() > bataskhusus_low[SELATAN])
		{
			mataangin = SELATAN;
		}
	}
	else if (nilai_cmps()>lower[SELATAN] && nilai_cmps() < upper[SELATAN])
	{
		mataangin = SELATAN;
	}

//BARAT
	upper[BARAT] = kalibrasi_barat + 45;
	lower[BARAT] = kalibrasi_barat - 45;
	if (upper[BARAT] > 360)
	{
		bataskhusus_up[BARAT] = upper[BARAT] - 360;
		upper[BARAT] = 360;
		aktivasi_up[BARAT] = 1;
	}
	else if (lower[BARAT] < 0)
	{
		bataskhusus_low[BARAT] = lower[BARAT] + 360;
		lower[BARAT] = 0;
		aktivasi_low[BARAT] = 1;
	}

	if (aktivasi_up[BARAT] == 1)
	{
		if ((nilai_cmps() > lower[BARAT] && nilai_cmps() < upper[BARAT]) || nilai_cmps() < bataskhusus_up[BARAT])
		{
			mataangin = BARAT;
		}
	}
	else if (aktivasi_low[BARAT] == 1)
	{
		if ((nilai_cmps() > lower[BARAT] && nilai_cmps() < upper[BARAT]) || nilai_cmps() > bataskhusus_low[BARAT])
		{
			mataangin = BARAT;
		}
	}
	else if (nilai_cmps()>lower[BARAT] && nilai_cmps() < upper[BARAT])
	{
		mataangin = BARAT;
	}
//TIMUR
	upper[TIMUR] = kalibrasi_timur + 45;
	lower[TIMUR] = kalibrasi_timur - 45;
	if (upper[TIMUR] > 360)
	{
		bataskhusus_up[TIMUR] = upper[TIMUR] - 360;
		upper[TIMUR] = 360;
		aktivasi_up[TIMUR] = 1;
	}
	else if (lower[TIMUR] < 0)
	{
		bataskhusus_low[TIMUR] = lower[TIMUR] + 360;
		lower[TIMUR] = 0;
		aktivasi_low[TIMUR] = 1;
	}

	if (aktivasi_up[TIMUR] == 1)
	{
		if ((nilai_cmps() > lower[TIMUR] && nilai_cmps() < upper[TIMUR]) || nilai_cmps() < bataskhusus_up[TIMUR])
		{
			mataangin = TIMUR;
		}
	}
	else if (aktivasi_low[TIMUR] == 1)
	{
		if ((nilai_cmps() > lower[TIMUR] && nilai_cmps() < upper[TIMUR]) || nilai_cmps() > bataskhusus_low[TIMUR])
		{
			mataangin = TIMUR;
		}
	}
	else if (nilai_cmps()>lower[TIMUR] && nilai_cmps() < upper[TIMUR])
	{
		mataangin = TIMUR;
	}
}


void muter_acuankompas(int acuan)
{
	setcmps = acuan;

	int errorC;
	errorC = (setcmps - nilai_cmps());
	if (errorC> 180)errorC -= 360;
	else if (errorC < -180)errorC += 360;
	if(errorC<=2 && errorC>=-2)
	{
		gerakStatic(0, 0, 0, 0, 0, 0, 25);
		sudah_lurus = 1;
		hitung_langkah = 0;
	}

	if(errorC>10)
	{
		gerak(0, 0, 5, 1, 25);
		sudah_lurus = 0;
	}
	else if(errorC<-10)
	{
		gerak(0, 0, -5, 1, 25);
		sudah_lurus = 0;
	}


	if(errorC<=10 && errorC>2)
	{
		gerak(0, 0, 1, 1, 15);
		sudah_lurus = 0;
	}
	else if(errorC>=-10 && errorC<-2)
	{
		gerak(0, 0, -1, 1, 15);
		sudah_lurus = 0;
	}
}

void muter_kompas(int acuan)
{
	if(acuan == UTARA)
	{
		setcmps = kalibrasi_utara;//202
	}
	else if(acuan == SELATAN)
	{
		setcmps = kalibrasi_selatan;//37
	}
	else if(acuan == TIMUR)
	{
		setcmps = kalibrasi_timur;//115
	}
	else if(acuan == BARAT)
	{
		setcmps = kalibrasi_barat;//309
	}

	int errorC;
	errorC = (setcmps - nilai_cmps());
	if (errorC> 180)errorC -= 360;
	else if (errorC < -180)errorC += 360;
	if(errorC<=2 && errorC>=-2)
	{
		gerakStatic(0, 0, 0, 0, 0, 0, 25);
		sudah_lurus = 1;
		hitung_langkah = 0;
	}

	if(errorC>10)
	{
		gerak(0, 0, 5, 1, 25);
		sudah_lurus = 0;
	}
	else if(errorC<-10)
	{
		gerak(0, 0, -5, 1, 25);
		sudah_lurus = 0;
	}


	if(errorC<=10 && errorC>2)
	{
		gerak(0, 0, 1, 1, 15);
		sudah_lurus = 0;
	}
	else if(errorC>=-10 && errorC<-2)
	{
		gerak(0, 0, -1, 1, 15);
		sudah_lurus = 0;
	}
}

void luruskan_cmps()
{
	if(mataangin == UTARA)
	{
		setcmps = kalibrasi_utara;
	}
	else if(mataangin == SELATAN)
	{
		setcmps = kalibrasi_selatan;
	}
	else if(mataangin == TIMUR)
	{
		setcmps = kalibrasi_timur;
	}
	else if(mataangin == BARAT)
	{
		setcmps = kalibrasi_barat;
	}
	else if(fungsi_orientasi == 1)
	{
		setcmps = setOrientasi;
	}

	int errorC;
	errorC = (setcmps - nilai_cmps());
	if (errorC> 180)errorC -= 360;
	else if (errorC < -180)errorC += 360;
	if(errorC<=2 && errorC>=-2)
	{
		gerakStatic(0, 0, 0, 0, 0, 0, 25);
		sudah_lurus = 1;
		hitung_langkah = 0;
	}

	if(errorC>10)
	{
		gerak(0, 0, 5, 1, 25);
		sudah_lurus = 0;
	}
	else if(errorC<-10)
	{
		gerak(0, 0, -5, 1, 25);
		sudah_lurus = 0;
	}


	if(errorC<=10 && errorC>2)
	{
		gerak(0, 0, 1, 1, 15);
		sudah_lurus = 0;
	}
	else if(errorC>=-10 && errorC<-2)
	{
		gerak(0, 0, -1, 1, 15);
		sudah_lurus = 0;
	}
}


int nilai_error_cmps()
{
	if(mataangin == UTARA)
	{
		setcmps = kalibrasi_utara;
	}
	else if(mataangin == SELATAN)
	{
		setcmps = kalibrasi_selatan;
	}
	else if(mataangin == TIMUR)
	{
		setcmps = kalibrasi_timur;
	}
	else if(mataangin == BARAT)
	{
		setcmps = kalibrasi_barat;
	}

	int errorC;
	errorC = (setcmps - nilai_cmps());
	if (errorC> 180)errorC -= 360;
	else if (errorC < -180)errorC += 360;

	return errorC;
}

void tampil_mataangin()
{

	lcd_gotoxy(0,3);
	sprintf(tampil,"ERROR =%3d",nilai_error_cmps());
	lcd_puts(tampil);
	if(mataangin == UTARA)
	{
		lcd_gotoxy(0,2);
		lcd_puts("UTARA");
	}
	else if(mataangin == SELATAN)
	{
		lcd_gotoxy(0,2);
		lcd_puts("SELATAN");
	}
	else if(mataangin == TIMUR)
	{
		lcd_gotoxy(0,2);
		lcd_puts("TIMUR");
	}
	else if(mataangin == BARAT)
	{
		lcd_gotoxy(0,2);
		lcd_puts("BARAT");
	}
	else if(mataangin == DEADZONE)
	{
		lcd_gotoxy(0,2);
		lcd_puts("DEADZONE");
	}
}

void cmps_sekarang()
{
	tembak_cmps();
	orientasi_Now = mataangin;
}

void perubahan_mataangin(int arah_jam, int banyak_berubah)
{
	int k;
	change = banyak_berubah;
	switch(arah_jam)
	{
		case 0:
			for(k =0;k<change;k++)
			{
				mataangin = mataangin + 1;
				if(mataangin>3)mataangin-=4;
			}
		break;
		case 1:
			for(k = 0;k<change;k++)
			{
				mataangin = mataangin - 1;
				if(mataangin<0)mataangin+=4;
			}
		break;
	}
}

int penanda_stack_ruang4;
int jumlah_lost;
void deteksi_lost_acuan(int arah, int state)
{
	arah_deteksi_lost = arah;
	state_kembalidarilost = state;
	switch (arah)
	{
	case 0: //telusur kiri
		if (ganti_kanan >= 1)
		{
			ganti_kiri = 0;
			ganti_kanan = 0;
		}
		if (ganti_kiri >= 4) //lost
		{
			jumlah_lost++;
			state_jalan = 5000;

		}
		break;
	case 1: //telusur kanan
		if (ganti_kiri >= 1)
		{
			ganti_kanan = 0;
			ganti_kiri = 0;
		}
		if (ganti_kanan >= 4) //lost
		{
			perubahan_mataangin(SEARAH_JARUM_JAM,1);
			state_telusur = 0;
			state_jalan = LURUSKAN_CMPS;
			next_state_jalan = 5000;
			jumlah_lost++;
		}
		break;
	}
}

int api_siap;
int setPoint_api = 24;
int error_api;
float sudutBelok_api;
float sudutBelok_api_statis;
int putar_api;
int PID_api()
{
	if (posisi_api == 25)
	{
		gerakStatic(0, 0, 0, 0, 0, 0, 30);
		api_siap = 1;
	}

	else if ((posisi_api > 8&&posisi_api <= 25)&&nilai_api > 180)
	{
		gerak(0, 0, -9, 1, 25);
		putar_api = 1;
	}
	else if ((posisi_api<8 || posisi_api>25)&&nilai_api > 180)
	{
		gerak(0, 0, 9, 1, 25);
		putar_api = 0;
	}
	else
	{
		if (putar_api == 0)
		{
			gerak(0, 0, 9, 1, 25);
		}
		else if (putar_api == 1)
		{
			gerak(0, 0, -9, 1, 25);
		}
	}

	return api_siap;
}

int putar_kepala_semprot;
int count_pompa;
void padamkan_api()
{
	posisico = buka_co-55;
	while (deteksi_api(500) == 1 || count_pompa < 2) //&&
	{
		gerakStatic(0, 0, 0, 10, 0, 0, 10);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;

		gerakStatic(0, 0, 0, -10, 0, 0, 10);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;

		count_pompa++;
	}
	eksistansi_api2 = 0;
	count_pompa = 0;
	posisico = tutup_co;
	penanda_buzzer = 0;
	gerakStatic(0, 0, 0, 0, 0, 0, 10);
	delay_ms(5000); //9000
}


void tampil_ruang_start()
{
	if (ruang_start == 1)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Start 1A");
	}
	else if (ruang_start == 2)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Start 2");
	}
	else if (ruang_start == 3)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Start 3");
	}
	else if (ruang_start == 4)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Start 4A");
	}
	else if (ruang_start == 5)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Start 1B");
	}
	else if (ruang_start == 6)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Start 1C");
	}
	else if (ruang_start == 7)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("Start 4B");
	}
	else if (ruang_start == 0)
	{
		lcd_gotoxy(0, 1);
		lcd_puts("error");
	}
}

float speed_b = 42.2;
float speed_s = 25;
float langkah_b = 2.5;

int state_b = 1;

void modeBluetooth()
{
	lcd_gotoxy(0, 1);
	sprintf(tampil, "Vd : %2.0f", speed_b);
	lcd_puts(tampil);

	lcd_gotoxy(8, 1);
	sprintf(tampil, "Vs : %2.0f", speed_s);
	lcd_puts(tampil);

	lcd_gotoxy(0, 2);
	sprintf(tampil, "L : %1.1f", langkah_b);
	lcd_puts(tampil);

	lcd_gotoxy(8, 2);
	sprintf(tampil, "w : %2.2f", belok);
	lcd_puts(tampil);

	tampil_kompas(0, 3);

	if (state_b == 0)
	{
		if (direksi == 5)
		{
			lcd_gotoxy(10, 3);
			lcd_puts(" -X ");
		}
		else if (direksi == 6)
		{
			lcd_gotoxy(10, 3);
			lcd_puts(" Y  ");
		}
		else if (direksi == 7)
		{
			lcd_gotoxy(10, 3);
			lcd_puts(" X  ");
		}
		else if (direksi == 8)
		{
			lcd_gotoxy(10, 3);
			lcd_puts(" -Y ");
		}
		else if (direksi == 9)
		{
			lcd_gotoxy(10, 3);
			lcd_puts("----");
		}
	}
	else if (state_b == 2)
	{
		if (direksi == 5)
		{
			lcd_gotoxy(10, 3);
			lcd_puts(" Z  ");
		}
		else if (direksi == 6)
		{
			lcd_gotoxy(10, 3);
			lcd_puts("YAW ");
		}
		else if (direksi == 7)
		{
			lcd_gotoxy(10, 3);
			lcd_puts("PTCH");
		}
		else if (direksi == 8)
		{
			lcd_gotoxy(10, 3);
			lcd_puts("ROLL");
		}
		else if (direksi == 9)
		{
			lcd_gotoxy(10, 3);
			lcd_puts("----");
		}
	}
	else
	{
		lcd_gotoxy(10, 3);
		lcd_puts("----");
	}

//	lcd_gotoxy(0,3);
//	sprintf(tampil,"data : %2c",dataKontrol);
//	lcd_puts(tampil);

	if (dataKontrol == '9')
	{
		if (state_b == 1)state_b = 0;
		else state_b = 1;

		belok = 0;

		gerakStatic(0, 0, 0, belok, 0, 0, 30);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;

		direksi = 9;
		setKompas();
		dataKontrol = '0';
		lcd_clear();
	}
	if (dataKontrol == 'A')
	{
		if (state_b == 3)state_b = 2;
		else state_b = 3;

		belok = 0;

		gerakStatic(0, 0, 0, belok, 0, 0, 30);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;

		direksi = 9;
		setKompas();
		dataKontrol = '0';
		lcd_clear();
	}


	switch (state_b)
	{
	case 0:
		lcd_gotoxy(0, 0);
		lcd_puts("GERAK DINAMIS");

		if (dataKontrol >= '5' && dataKontrol <= '8')
		{
			if (dataKontrol == '5')direksi = 5;
			else if (dataKontrol == '6')direksi = 6;
			else if (dataKontrol == '7')direksi = 7;
			else if (dataKontrol == '8')direksi = 8;
			dataKontrol = '0';
			//lcd_clear();
		}

		if (dataKontrol >= '1' && dataKontrol <= '4')
		{
			if (dataKontrol == '1')
			{
				belok = 0;
			}
			else if (dataKontrol == '2')
			{
				belok = belok - 2;
				if (belok < -26)belok = -26;
			}
			else if (dataKontrol == '3')
			{
				belok = 0;
			}
			else if (dataKontrol == '4')
			{
				belok = belok + 2;
				if (belok > 26)belok = 26;
			}
			dataKontrol = '0';
			//lcd_clear();
		}

		if (direksi == 6)
		{
			gerak(0, langkah_b, belok, 1, speed_b);

		}
		else if (direksi == 8)
		{
			gerak(0, -langkah_b, belok, 1, speed_b);
		}
		else if (direksi == 7)
		{
			gerak(langkah_b, 0, belok, 1, speed_b);
		}
		else if (direksi == 5)
		{
			gerak(-langkah_b, 0, belok, 1, speed_b);
		}
		else if (direksi == 9) //untuk stop
		{
			belok = 0;
			gerak(0, 0, belok, 0, speed_b);
			gerakStatic(0, 0, 0, belok, 0, 0, 30);
		}
		break;
	case 1:
		lcd_gotoxy(0, 0);
		lcd_puts("SETTING DINAMIS");
		if (dataKontrol >= '1' && dataKontrol <= '4')
		{
			if (dataKontrol == '1')
			{
				speed_b += 1;
			}
			else if (dataKontrol == '2')
			{
				langkah_b += 0.1;
			}
			else if (dataKontrol == '3')
			{
				speed_b -= 1;
			}
			else if (dataKontrol == '4')
			{
				langkah_b -= 0.1;
			}
			dataKontrol = '0';
			//lcd_clear();
		}
		break;

	case 2:
		lcd_gotoxy(0, 0);
		lcd_puts("GERAK STATIS");

		if (dataKontrol >= '5' && dataKontrol <= '8')
		{
			if (dataKontrol == '5')direksi = 5;
			else if (dataKontrol == '6')direksi = 6;
			else if (dataKontrol == '7')direksi = 7;
			else if (dataKontrol == '8')direksi = 8;
			dataKontrol = '0';
			//lcd_clear();
		}

		if (dataKontrol >= '1' && dataKontrol <= '4')
		{
			if (dataKontrol == '1')
			{
				belok = 0;
			}
			else if (dataKontrol == '2')
			{
				belok = belok - 0.5;
			}
			else if (dataKontrol == '3')
			{
				belok = 0;
			}
			else if (dataKontrol == '4')
			{
				belok = belok + 0.5;
			}
			dataKontrol = '0';
			//lcd_clear();
		}
//		if(direksi==10)
//		{
//			//kecepatan_edf=edf;
//			gerakStatic(0,0,0,-belok,0,0,SPEED_DEMO);
//			dataKontrol='0';
//		}
		if (direksi == 6) //yaw
		{
			if (belok < -20)belok = -20;
			else if (belok > 20)belok = 20;
			gerakStatic(0, 0, 0, belok, 0, 0, speed_s);
		}
		else if (direksi == 8) //roll
		{
			if (belok > 9)belok = 9;
			else if (belok < -9)belok = -9;
			gerakStatic(0, 0, 0, 0, 0, belok, speed_s);
		}
		else if (direksi == 7) //pitch
		{
			if (belok > 9)belok = 9;
			else if (belok < -9)belok = -9;
			gerakStatic(0, 0, 0, 0, belok, 0, speed_s);
		}
		else if (direksi == 5) //z
		{
			if (belok > 1)belok = 1;
			else if (belok < -2.5)belok = -2.5;
			gerakStatic(0, 0, belok, 0, 0, 0, speed_s);
		}
		else if (direksi == 9) //untuk stop
		{
			belok = 0;
			gerakStatic(0, 0, 0, belok, 0, 0, 30);
		}
		break;
	case 3:
		lcd_gotoxy(0, 0);
		lcd_puts("SETTING STATIS");
		if (dataKontrol >= '1' && dataKontrol <= '4')
		{
			if (dataKontrol == '1')
			{
				speed_s += 1;
			}
			else if (dataKontrol == '2')
			{
				//langkah_b+=0.1;
			}
			else if (dataKontrol == '3')
			{
				speed_s -= 1;
			}
			else if (dataKontrol == '4')
			{
				//langkah_b-=0.1;
			}
			dataKontrol = '0';
			//lcd_clear();
		}
		break;
	}
}


float count_demo;
int state_demo;
void demo_quadruped()
{
	statusKompas = 1;
	switch (state_demo)
	{
	case 0:
		buzzer();
		penanda_buzzer = 0;
		jalan_langkah(state_telusur, 7);
		penanda_buzzer = 0;
		konversi_state_telusur(1);
		jalan_langkah(state_telusur, 7);
		penanda_buzzer = 0;
		konversi_state_telusur(1);
		jalan_langkah(state_telusur, 7);
		penanda_buzzer = 0;
		konversi_state_telusur(1);
		jalan_langkah(state_telusur, 7);
		penanda_buzzer = 0;
		konversi_state_telusur(1);
		state_demo = 1;
		break;
	case 1:
		putar(360, 9, 25);
		penanda_buzzer = 0;
		state_demo = 102;
		hitung_langkah = 0;
		break;
	case 102:
		PIDkr();
		gerak(0, 0, sudutBelok, 1, 30);
		if (kompas == setPointK)
		{
			gerakStatic(0, 0, 0, 0, 0, 0, 30);
			penanda_buzzer = 0;
			state_demo = 2;
		}
		break;
	case 2:
		serong(state_telusur, 1);
		if (hitung_langkah > 7)
		{
			state_demo = 3;
			hitung_langkah = 0;
			penanda_buzzer = 0;
		}
		break;
	case 3:
		serong(state_telusur, -1);
		if (hitung_langkah > 7)
		{
			state_demo = 4;
			hitung_langkah = 0;
			konversi_state_telusur(-2);
			penanda_buzzer = 0;
		}
		break;
	case 4:
		serong(state_telusur, 1);
		if (hitung_langkah > 7)
		{
			state_demo = 5;
			hitung_langkah = 0;
			penanda_buzzer = 0;
		}
		break;
	case 5:
		serong(state_telusur, -1);
		if (hitung_langkah > 7)
		{
			state_demo = 6;
			hitung_langkah = 0;
			konversi_state_telusur(-2);
			penanda_buzzer = 0;
		}
		break;
	case 6:
		PIDkr();
		gerak(0, 0, sudutBelok, 1, 30);
		if (kompas == setPointK)
		{
			gerakStatic(0, 0, 0, 0, 0, 0, 30);
			penanda_buzzer = 0;
			state_demo = 7;
		}
		break;
	case 7:
		gerakStatic(0, 0, -3.5, 0, 0, 0, 2.5);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;
		gerakStatic(0, 0, -1, 0, 0, 0, 2.5);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;
		gerakStatic(-5, 0, -1, 0, 0, 0, 2.5);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;
		gerakStatic(5, 0, -1, 0, 0, 0, 2.5);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;
		gerakStatic(0, -5, -1, 0, 0, 0, 2.5);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;
		gerakStatic(0, 5, -1, 0, 0, 0, 2.5);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;
		gerakStatic(0, 0, 0, 0, 0, 0, 2.5);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;
		state_demo = 8;
		break;
	case 8:
		for (count_demo = 0; count_demo < 3; count_demo++) //yaw
		{
			gerakStatic(0, 0, -1, 18, 0, 0, 5);
			while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
				;
			gerakStatic(0, 0, -1, -18, 0, 0, 5);
			while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
				;
		}
		gerakStatic(0, 0, -1, 0, 0, 0, 7);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;

		for (count_demo = 0; count_demo < 3; count_demo++) //pitch
		{
			gerakStatic(0, 0, -1, 0, 5, 0, 4);
			while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
				;
			gerakStatic(0, 0, -1, 0, -5, 0, 4);
			while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
				;
		}
		gerakStatic(0, 0, -1, 0, 0, 0, 7);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;

		for (count_demo = 0; count_demo < 3; count_demo++) //roll
		{
			gerakStatic(0, 0, -1, 0, 0, 5, 4);
			while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
				;
			gerakStatic(0, 0, -1, 0, 0, -5, 4);
			while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
				;
		}
		gerakStatic(0, 0, -1, 0, 0, 0, 7);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;

		gerakStatic(0, 3.5, -1, 0, 0, 0, 7);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;

		for (i = 0; i < 1; i++)
		{
			for (count_demo = 0; count_demo <= 1; count_demo = count_demo + 0.1)
			{
				gerakStatic(3.5*sin(2 * PI * count_demo), 3.5*cos(2*PI*count_demo), -1.5, 0, 0, 0, 5);
				while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
					;
			}
			penanda_buzzer = 0;
		}

		gerakStatic(3.5, 0, -1, 0, 0, 0, 7);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;

		for (i = 0; i < 1; i++)
		{
			for (count_demo = 0; count_demo <= 1; count_demo = count_demo + 0.1)
			{
				gerakStatic(3.5*cos(2 * PI * count_demo), 3.5*sin(2*PI*count_demo), -1.5, 0, 0, 0, 5);
				while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
					;
			}
			penanda_buzzer = 0;
		}

		for (i = 0; i < 1; i++)
		{
			for (count_demo = 0; count_demo <= 1; count_demo = count_demo + 0.1)
			{
				gerakStatic(0, 3.5*cos(2 * PI * count_demo), 2.5*sin(2*PI*count_demo) - 1.5, 0, 0, 0, 5);
				while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
					;
			}
			penanda_buzzer = 0;
		}

		for (i = 0; i < 1; i++)
		{
			for (count_demo = 0; count_demo <= 1; count_demo = count_demo + 0.1)
			{
				gerakStatic(0, 3.5*sin(2 * PI * count_demo), 2.5*cos(2*PI*count_demo) - 1.5, 0, 0, 0, 5);
				while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
					;
			}
			penanda_buzzer = 0;
		}


		for (i = 0; i < 1; i++)
		{
			for (count_demo = 0; count_demo <= 1; count_demo = count_demo + 0.1)
			{
				gerakStatic(3.5*cos(2 * PI * count_demo), 0, 2.5*sin(2*PI*count_demo) - 1.5, 0, 0, 0, 5);
				while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
					;
			}
			penanda_buzzer = 0;
		}

		for (i = 0; i < 1; i++)
		{
			for (count_demo = 0; count_demo <= 1; count_demo = count_demo + 0.1)
			{
				gerakStatic(3.5*sin(2 * PI * count_demo), 0, 2.5*cos(2*PI*count_demo) - 1.5, 0, 0, 0, 5);
				while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
					;
			}
			penanda_buzzer = 0;
		}


		gerakStatic(0, 0, -1, 0, 0, 0, 7);
		while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
			;
		penanda_buzzer = 0;

		state_demo = 9;
		buzzer();
		delay_ms(1000);
		buzzer();
		delay_ms(1000);
		buzzer();
		break;
	case 9:
		gerakStatic(0, 0, 0, 0, 0, 0, 7);
		break;
	}
}


/*
 * Variable Ruang Start
 * 1A	=	1
 * 2	=	2
 * 3	=	3
 * 4A	=	4
 * 1B	=	5
 * 1C	=	6
 * 4B	=	7
 */

int count_penanda_dinding2; //penanda dinding dekat anjing 2
int count_penanda_anjing3;
int count_penanda_anjing1;
int penanda_api_1; // 0 jika api di dinding luar, 1 jika api di dinding dalam
short int penanda_khusus_1a;
int penanda_khusus_2;
int penanda_khusus_2api;
int penanda_telusur_api;
short int penanda_pintu1; //0 = 1a, 1=1b
int penanda_langkah;
int thd_dinding = 400;
int batas_serong;

void bismillah()//2019
{
	lcd_gotoxy(0, 0);
	lcd_puts("<<<BISMILLAH>>>");
	lcd_gotoxy(0, 1);
	sprintf(tampil, "State=%4d", state_jalan);
	lcd_puts(tampil);
	//tampil_mataangin();
	lcd_gotoxy(0,2);
	sprintf(tampil,"muka=%d %1d ",ganti_muka,state_telusur);
	lcd_puts(tampil);
	lcd_gotoxy(0,3);
	sprintf(tampil,"%d   %d %d",mataangin,ruang_start,ruang_api);
	lcd_puts(tampil);
	lcd_gotoxy(9,3);
	sprintf(tampil,"L R %d %d",ganti_kiri, ganti_kanan);
	lcd_puts(tampil);

	switch(state_jalan)
	{
	//case ruang
	case 0:
		baca_gp(0);
		konversi_gp(0);
		gp_serong();
//		muter_acuankompas(kalibrasi_timur);//acuan start
//		if(sudah_lurus==1)
//		{
//			baca_gp(0);
//			konversi_gp(0);
//			gp_serong();
//			mataangin=TIMUR;
//			setKompas();
//			tembak_cmps();
//
//			state_jalan = 50;
//		}
		state_jalan = 50;
		break;
	case 50:
		konversi_gp(state_telusur);
		gp_serong();
		baca_gp(state_telusur);
		telusurKanan_zigzag_ruang();
		if(baca_garis()==1)
		{
			penanda_buzzer=0;
			pewaktu=0;
			setKompas();
			tembak_cmps();
			state_jalan = LURUSKAN_CMPS;
			next_state_jalan = 1000;
		}
		break;
	//case fungsi
	case 545:
		if(gunakan_cmps == 1)
		{
			luruskan_cmps();
			if(sudah_lurus == 1)
			{
				state_jalan = next_state_jalan;
				ganti_muka = 0;
			}
		}
		else if(gunakan_cmps == 0)
		{
			state_jalan = next_state_jalan;
			ganti_muka = 0;
		}
		break;
	case 555:
		luruskan_tembok_kanan();
		if(lurus_tembok == 1 || pewaktu>60)
		{
			state_jalan = next_state_jalan;
			hitung_langkah = 0;
			hitung_zigzag = 0;
			ganti_muka = 0;
			ganti_kiri = 0;
			ganti_kanan = 0;
			pewaktu = 0;
		}
		break;
	case 554:
		luruskan_tembok_kiri();
		if(lurus_tembok == 1)
		{
			state_jalan = next_state_jalan;
			hitung_langkah = 0;
			hitung_zigzag = 0;
			ganti_muka = 0;
			ganti_kiri = 0;
			ganti_kanan = 0;
		}
	//case deteksi ruang
	case 1000:
		if(state_telusur==0 && mataangin==TIMUR)ruang_start=2;
		else if(state_telusur==1 && mataangin==SELATAN)ruang_start=2;
		else if(state_telusur==2 && mataangin==BARAT)ruang_start=2;
		else if(state_telusur==3 && mataangin==UTARA)ruang_start=2;

		else if(state_telusur==0 && mataangin==UTARA)ruang_start=34;
		else if(state_telusur==1 && mataangin==TIMUR)ruang_start=34;
		else if(state_telusur==2 && mataangin==SELATAN)ruang_start=34;
		else if(state_telusur==3 && mataangin==BARAT)ruang_start=34;

		else if(state_telusur==2 && mataangin==UTARA)ruang_start=67;
		else if(state_telusur==3 && mataangin==TIMUR)ruang_start=67;
		else if(state_telusur==0 && mataangin==SELATAN)ruang_start=67;
		else if(state_telusur==1 && mataangin==BARAT)ruang_start=67;

		else if(state_telusur==1 && mataangin==UTARA)ruang_start=15;
		else if(state_telusur==2 && mataangin==TIMUR)ruang_start=15;
		else if(state_telusur==3 && mataangin==SELATAN)ruang_start=15;
		else if(state_telusur==0 && mataangin==BARAT)ruang_start=15;

		else ruang_start=99;
		state_jalan=1001;
		break;
	case 1001:
		if(ruang_start==2)state_jalan=2002;
		else if(ruang_start==34)state_jalan=1002;
		else if(ruang_start==67)state_jalan=1003;
//		else if(ruang_start==15)state_jalan=1004;//dilanjut setelah algoritma ruang selesai
		break;
	case 1002:
		konversi_gp(state_telusur);
		gp_serong();
		baca_gp(state_telusur);
		if(sensor_gp[sensor[1]]<400)
		{
			state_jalan = LURUSKAN_KANAN;
			next_state_jalan = 2003;
			ruang_start = 3;
		}
		else
		{
			state_jalan = LURUSKAN_CMPS;
			next_state_jalan = 2004;
			ruang_start = 4;
		}
		break;
	case 1003:
		konversi_gp(state_telusur);
		gp_serong();
		baca_gp(state_telusur);
		if(sensor_gp[sensor[1]]<400)
		{
			state_jalan = LURUSKAN_KANAN;
			next_state_jalan = 2006;
			ruang_start = 6;
		}
		else
		{
			state_jalan = LURUSKAN_CMPS;
			next_state_jalan = 2007;
			ruang_start = 7;
		}
		break;
	case 1004:
		konversi_gp(state_telusur);
		gp_serong();
		baca_gp(state_telusur);
		if(sensor_gp[sensor[0]]<70)
		{
			konversi_state_telusur(-1);
			state_jalan = 1005;
		}
		else
		{
			state_jalan = 1005;
		}
		break;
	case 1005:
		konversi_gp(state_telusur);
		gp_serong();
		baca_gp(state_telusur);
		jalan_langkah(state_telusur,2);
		if(sensor_gp_asli[0]<200)
		{
			ruang_start = 5;
			state_jalan = 2005;
		}
		else
		{
			ruang_start = 1;
			state_jalan = 2001;
		}
		break;
	//case navigasi
	case 2001:

		break;
	case 2002:

		break;
	case 2003:

		break;
	case 2004:

		break;
	case 2005:

		break;
	case 2006:

		break;
	case 2007:

		break;
	}
}

int thd_wall = 100, k;
int minWall()
{
	baca_gp(state_telusur);
	for (i =0 ; i <4 ; i++)
	{
		if (sensor_gp_asli[i] < thd_wall){
			thd_wall = sensor_gp_asli[i];
			k=i;
		} else{
			sensor_gp_asli[i] = thd_wall;
			j = sensor_gp_asli[i] ;
			thd_wall = j;
		}
	}
	return k;
}

void led_muka_mati(){
	led_muka0_off;
	led_muka1_off;
	led_muka3_off;
}

void led_muka(int telusur){
	led_muka_mati();
	switch(telusur){
	case 0:
		led_muka0_on;
		break;
	case 1:
		led_muka1_on;
		break;
	case 3:
		led_muka3_on;
		break;
	}
}

unsigned char penanda_limit_switch;

void jalan()
{
	//lcd_clear();
	lcd_gotoxy(0, 0);
	lcd_puts("<<<BISMILLAH>>>");
	lcd_gotoxy(0, 1);
	sprintf(tampil, "State : %4d", state_jalan);
	lcd_puts(tampil);
	//tampil_mataangin();
	lcd_gotoxy(0,2);
	sprintf(tampil,"muka: %d %d %d",ganti_muka,state_telusur,lock_state_telusur);
	lcd_puts(tampil);
	lcd_gotoxy(0,3);
	sprintf(tampil,"%d %d %d %d",mataangin,ruang_start,ruang_api,posisi_anjing);
	lcd_puts(tampil);
	lcd_gotoxy(9,3);
	sprintf(tampil,"L R %d %d",ganti_kiri, ganti_kanan);
	lcd_puts(tampil);
	if(state_USART2 == 2 && pos_state_jalanArr < BUFFER_state_jalanArr){
		state_jalanArr[pos_state_jalanArr] = state_jalan;
		if(state_jalanArr[pos_state_jalanArr]!= temp_state_jalanArr){
			temp_state_jalanArr = state_jalan;
			pos_state_jalanArr++;
		}
		else{
			state_jalanArr[pos_state_jalanArr] = '\0';
		}
	}

	led_muka(state_telusur);
	switch (state_jalan)
	{
	case 0:
		state_telusur = 0;
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		muter_acuankompas(kalibrasi_selatan);
		if(sudah_lurus == 1){
			sudah_lurus = 0;
			state_jalan = 1;
		}

		break;
	case 1:
		maju(state_telusur);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		/**
		 * (+) Syauqi
		 */
		if (sensor_gp[sensor[0]] < setPoint0z + 20){
			lock = 0;
			muter_acuankompas(kalibrasi_serong_selatan);
			sudah_lurus == 0;
			konversi_state_telusur(1);
			ganti_muka = 0;
			state_jalan = 4;
			kecepatan = 55;
			langkah = 1.0;
			tinggi = 0.8;
		}
		break;

	case 2:
		telusurKiri_zigzag_detek_floor();
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();

		if( (limit_switch0_on || limit_switch1_on) && (limit_switch2_off && limit_switch3_off) ){
			penanda_buzzer = 0;
			gerakStatic(0,0,0,0,0,0,30);
			pewaktu = 0;
			state_jalan = 3;
			konversi_state_telusur(-2);

		}

//		if( (limit_switch2_on || limit_switch3_on) && (limit_switch0_off && limit_switch1_off) ){
//			penanda_limit_switch = 2;
//			penanda_buzzer = 0;
//			gerakStatic(0,0,0,0,0,0,30);
//			pewaktu = 0;
//			konversi_state_telusur(-2);
//			state_jalan = 3;
//
//		}
//		if(ganti_muka >= 1){
//			gerakStatic(0,0,0,0,0,0,30);
//			state_jalan = 4;
//		}
		break;

	case 3:
		telusurKanan_zigzag_ruang();
		if(pewaktu > 2){
			konversi_state_telusur(-2);
			state_jalan = LURUSKAN_KIRI;
			next_state_jalan = 4;
			ganti_kanan = 0;
			ganti_kiri = 0;
			ganti_muka = 0;
			sudah_lurus = 0;
		}

		break;

	case 4:
		/**
		 * persiapan jalan serong compas ke selatan
		 */
		muter_acuankompas(kalibrasi_serong_selatan);
		if(sudah_lurus == 1){
			pewaktu = 0;
			sudah_lurus = 0;
			konversi_state_telusur(-2);
			state_jalan = 50;
		}
		break;

	case 50:
		serong_floor(state_telusur,1);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		if(sensor_gp[sensor[1]] < 70){
			kecepatan = 40.2; //60.2
			langkah = 2.5; //3
			tinggi = 1.5;
			konversi_state_telusur(-2);
			state_jalan = 5;
		}

		break;

	case 5:
		serong_floor(state_telusur,1);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();

		if(limit_switch1_on && limit_switch3_off){
			penanda_limit_switch = 1;
			pewaktu = 0;
			penanda_buzzer = 0;
			gerakStatic(0,0,0,0,0,0,30);
			state_jalan = 6;
			konversi_state_telusur(-1);
		}
		if(limit_switch3_on && limit_switch1_off || (limit_switch0_on) || (limit_switch2_on)){
			penanda_limit_switch = 2;
			pewaktu = 0;
			penanda_buzzer = 0;
			gerakStatic(0,0,0,0,0,0,30);
			state_jalan = 6;
			konversi_state_telusur(-1);
		}
		if(state_telusur == 1 && sensor_gp[sensor[1]] < 90){
			gerakStatic(0,0,0,0,0,0,30);
			unfloor = 1;
			konversi_state_telusur(-1);
			sudah_lurus = 0;
			ganti_muka = 0;
			ganti_kanan = 0;
			ganti_kiri = 0;
			state_jalan = 20;

		}
		break;

	case 6:
		serong_floor(state_telusur,-1);
		if(pewaktu>8){
			if(penanda_limit_switch == 1){
				kecepatan = 40.2; //60.2
				langkah = 2.0; //3
				tinggi = 2.0;
			}
			else{
				kecepatan = 40.2; //60.2
				langkah = 1.2; //3
				tinggi = 1.8;
			}
			ganti_muka = 0;
			gerakStatic(0,0,0,0,0,0,30);
			konversi_state_telusur(1);
			state_jalan = 5;
		}
		break;

	case 20:
		/**
		 * persiapan jalan serong compas ke timur
		 */
		muter_acuankompas(kalibrasi_timur);

		if(sudah_lurus == 1){
			sudah_lurus == 0;
			state_jalan = 21;
		}
		break;

	case 21:
		maju(state_telusur);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		/**
		 * (+) Syauqi
		 */
		if (sensor_gp[sensor[0]] < setPoint0z + 20){
			lock = 0;
			muter_acuankompas(kalibrasi_serong_timur);
			konversi_state_telusur(1);
			ganti_muka = 0;
			sudah_lurus == 0;
			state_jalan = 22;
			kecepatan = 55;
			langkah = 1.0;
			tinggi = 0.8;
		}
		break;

	case 22:
		/**
		 * persiapan jalan serong compas ke serong timur
		 */
		muter_acuankompas(kalibrasi_serong_timur);
		if(sudah_lurus == 1){
			konversi_state_telusur(-2);
			sudah_lurus = 0;
			state_jalan = 51;
		}
		break;

	case 51:
		serong_floor(state_telusur,1);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		if(sensor_gp[sensor[1]] < 70){
			konversi_state_telusur(-2);
			kecepatan = 40.2; //60.2
			langkah = 2.5; //3
			tinggi = 1.5;
			state_jalan = 23;
		}
		break;

	case 23:
		serong_floor(state_telusur,1);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();

		if(limit_switch1_on && limit_switch3_off){
			penanda_limit_switch = 1;
			pewaktu = 0;
			penanda_buzzer = 0;
			gerakStatic(0,0,0,0,0,0,30);
			state_jalan = 25;
			konversi_state_telusur(-1);
		}
		if(limit_switch3_on && limit_switch1_off || (limit_switch0_on) || (limit_switch2_on)){
			penanda_limit_switch = 2;
			pewaktu = 0;
			penanda_buzzer = 0;
			gerakStatic(0,0,0,0,0,0,30);
			state_jalan = 25;
			konversi_state_telusur(-1);
		}
		if(state_telusur == 1 && sensor_gp[sensor[1]] < 100){
			gerakStatic(0,0,0,0,0,0,30);
			unfloor = 2;
			ganti_muka = 0;
			konversi_state_telusur(-1);
			sudah_lurus = 0;
			ganti_kanan = 0;
			ganti_kiri = 0;
			state_jalan = 30;

		}
		break;

	case 25:
		serong_floor(state_telusur,-1);
		if(pewaktu>3){
			if(penanda_limit_switch == 1){
				kecepatan = 40.2; //60.2
				langkah = 1.6; //3
				tinggi = 2.0;
			}
			else{
				kecepatan = 40.2; //60.2
				langkah = 1.2; //3
				tinggi = 1.8;
			}
			ganti_muka = 0;
			gerakStatic(0,0,0,0,0,0,30);
			konversi_state_telusur(1);
			state_jalan = 23;
		}
		break;

	case 30:
		/**
		 * persiapan jalan serong compas ke utara
		 */
		muter_acuankompas(kalibrasi_utara);

		if(sudah_lurus == 1){
			sudah_lurus == 0;
			state_jalan = 31;
		}
		break;


	case 31:
		maju(state_telusur);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		/**
		 * (+) Syauqi
		 */
		if (sensor_gp[sensor[0]] < 80){
			lock = 0;
			muter_acuankompas(kalibrasi_serong_utara);
			konversi_state_telusur(1);
			ganti_muka = 0;
			sudah_lurus == 0;
			state_jalan = 32;
			kecepatan = 55;
			langkah = 1.0;
			tinggi = 0.8;
		}
		break;

	case 32:
		/**
		 * persiapan jalan serong compas ke serong utara
		 */
		muter_acuankompas(kalibrasi_serong_utara);
		if(sudah_lurus == 1){
			state_floor = 0;
			sudah_lurus = 0;
			konversi_state_telusur(-2);
			state_jalan = 52;
		}
		break;

	case 52:
		serong_floor(state_telusur,1);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		if(sensor_gp[sensor[1]] < 70){
			kecepatan = 40.2; //60.2
			langkah = 2.5; //3
			tinggi = 1.5;
			konversi_state_telusur(-2);
			state_jalan = 33;
		}
		break;

	case 33:
		serong_floor(state_telusur,1);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();

		if(limit_switch1_on && limit_switch3_off){
			penanda_limit_switch = 1;
			pewaktu = 0;
			penanda_buzzer = 0;
			gerakStatic(0,0,0,0,0,0,30);
			state_jalan = 35;
			konversi_state_telusur(-1);
		}
		if(limit_switch3_on && (limit_switch1_off) || (limit_switch0_on) || (limit_switch2_on)){
			penanda_limit_switch = 2;
			pewaktu = 0;
			penanda_buzzer = 0;
			gerakStatic(0,0,0,0,0,0,30);
			state_jalan = 35;
			konversi_state_telusur(-1);
		}
//		if(state_telusur == 1 && sensor_gp[sensor[7]] < 400 && state_floor == 0){
//			state_floor = 1;
//		}

		if(state_telusur == 1 && sensor_gp[sensor[7]] > 400 && sensor_gp[sensor[3]] > 400){
			gerakStatic(0,0,0,0,0,0,30);
			unfloor = 3;
			ganti_muka = 0;
			konversi_state_telusur(-1);
			sudah_lurus = 0;
			ganti_kanan = 0;
			ganti_kiri = 0;
			state_jalan = 37;

		}
		break;

	case 35:
		serong_floor(state_telusur,-1);
		if(pewaktu>5){
			if(penanda_limit_switch == 1){
				kecepatan = 40.2; //60.2
				langkah = 1.6; //3
				tinggi = 2.0;
			}
			else{
				kecepatan = 50.2; //60.2
				langkah = 1.2; //3
				tinggi = 1.8;
			}
			ganti_muka = 0;
			gerakStatic(0,0,0,0,0,0,30);
			konversi_state_telusur(1);
			state_jalan = 33;
		}
		break;

	case 37:
		if(unfloor == 3){
			unfloor = 0;
		}
		/**
		 * persiapan jalan serong compas ke barat
		 */
		muter_acuankompas(kalibrasi_barat);

		if(sudah_lurus == 1){
			sudah_lurus == 0;
			state_jalan = 38;
			state_telusur = 0;
			konversi_state_telusur(-2);
		}
		break;

	case 38:
		maju(state_telusur);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		/**
		 * (+) Syauqi
		 */
		if (sensor_gp[sensor[3]]<400 && sensor_gp[sensor[1]] < 400){
			penanda_buzzer = 0;
			lock = 0;
			konversi_state_telusur(1);
			ganti_muka = 0;
			ganti_kanan= 0;
			ganti_kiri = 0;
			setKompas();
			mataangin = tembak_cmps();
			state_jalan = LURUSKAN_CMPS;
			next_state_jalan = 39;
		}
		break;

	case 39:
		maju(state_telusur);
		baca_gp(state_telusur);
		konversi_gp(state_telusur);
		gp_serong();
		if(sensor_gp_asli[0]<70)
		{
			konversi_state_telusur(1);
			state_jalan = LURUSKAN_KIRI;
			next_state_jalan = 40;
			ganti_muka = 0;
			ganti_kiri = 0;
			ganti_kanan = 0;
		}
		break;

	case 40:
		telusurKiri_zigzag();

		if(ganti_muka == 1){
			state_jalan = 41;
		}
		break;

	case 41:
		telusurKiriPipih();
		if(baca_garis()==1){
			penanda_buzzer = 0;
			state_jalan =999;
		}
		break;



//	case 0 : //awal
//		baca_gp(0);
//		konversi_gp(0);
//		gp_serong();
//		muter_acuankompas(kalibrasi_timur);//acuan start
//		//muter_kompas(TIMUR);
//		if(sudah_lurus==1)
//		{
//			baca_gp(0);
//			konversi_gp(0);
//			gp_serong();
//			mataangin=TIMUR;
//			setKompas();
//			tembak_cmps();
//			if(minWall()==0){
//				state_telusur = 3;
//				baca_gp(state_telusur);
//				konversi_gp(state_telusur);
//				gp_serong();
//				if((sensor_gp[1]&&sensor_gp[3])<thd_wall)
//				{
//					pewaktu = 0;
//					state_jalan = LURUSKAN_KANAN;
//					next_state_jalan = 2002;
//				}else state_jalan = 2002;
//			}
//			else if(minWall()==1){
//				state_telusur = 0;
//				baca_gp(state_telusur);
//				konversi_gp(state_telusur);
//				gp_serong();
//				if((sensor_gp[1]&&sensor_gp[3])<thd_wall)
//				{
//					pewaktu = 0;
//					state_jalan = LURUSKAN_KANAN;
//					next_state_jalan = 2002;
//				}else state_jalan = 2002;
//			}
//			else if(minWall()==2){
//				state_telusur = 1;
//				baca_gp(state_telusur);
//				konversi_gp(state_telusur);
//				gp_serong();
//				if((sensor_gp[1]&&sensor_gp[3])<thd_wall)
//				{
//					pewaktu = 0;
//					state_jalan = LURUSKAN_KANAN;
//					next_state_jalan = 2002;
//				}else state_jalan = 2002;
//			}
//			else if(minWall()==3){
//				state_telusur = 2;
//				baca_gp(state_telusur);
//				konversi_gp(state_telusur);
//				gp_serong();
//				if((sensor_gp[1]&&sensor_gp[3])<thd_wall)
//				{
//					pewaktu = 0;
//					state_jalan = LURUSKAN_KANAN;
//					next_state_jalan = 2002;
//				}else state_jalan = 2002;
//			}
//			else state_jalan = 2002;
//		}
//		break;
//	case 500:
//		telusurKiri_zigzag_ruang();
//		if(sensor_gp[sensor[7]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[1]]<120)
//		{
//			if(sensor_gp[sensor[6]]<100)
//			{
//				geser_kanan(state_telusur,1);
//			}
//			else if(sensor_gp[sensor[2]]<200)
//			{
//				konversi_state_telusur(1);
//			}
//			else
//			{
//				konversi_state_telusur(-2);
//			}
//		}
//
//		deteksi_lost_acuan(0, 500);
//
//		if(baca_garis()==1)
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			setKompas();
//			tembak_cmps();
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 499;
//			konversi_state_telusur(1);
//		}
//		break;
//	case 499:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<120)
//		{
//			konversi_state_telusur(-1);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			hitung_langkah = 0;
//			//state_jalan = 999;
//			if(sensor_gp[sensor[1]]<400)
//			{
//				state_jalan = 667;
//			}
//			else
//			{
//				gunakan_cmps = 1;
//				state_jalan = 666;
//			}
//		}
//		break;
//	case 518:
//		luruskan_tembok_kanan();
//		if(lurus_tembok == 1)
//		{
//			penanda_buzzer = 0;
//			tembak_cmps();
//			lock_acuan = tembak_cmps();
//			state_telusur = 1;
//			hitung_langkah = 0;
//			count_garis = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 2002;
//		}
//		else if(pewaktu>100)
//		{
//			geser_kiri(0,1);
//			pewaktu = 0;
//			state_jalan = 506;
//		}
//		break;
//	case 506:
//		luruskan_tembok_kiri();
//		if(lurus_tembok == 1)
//		{
//			penanda_buzzer = 0;
//			tembak_cmps();
//			lock_acuan = tembak_cmps();
//			state_telusur = 3;
//			hitung_langkah = 0;
//			count_garis = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 2002;
//		}
//		break;
//	case 1:
//		konversi_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < 160)
//		{
//			lcd_clear();
//			state_jalan = 2001;
//			pewaktu = 0;
//		}
//
//		if (baca_garis() == 1)
//		{
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			lcd_clear();
//			jalan_langkah(state_telusur, 3);
//			count_garis = 0;
//		}
//
//		break;
//	case 2001:
//		baca_gp(state_telusur);
//		gp_serong();
//		konversi_gp(state_telusur);
//		gerak(0, 0, 9, 1, 25);
//		if (sensor_gp_asli[1]<150&&sensor_gp[sensor[1]]>100)
//		{
//			penanda_buzzer = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 2002;
//			setKompas();
//			tembak_cmps();
//			statusKompas = 1;
//			konversi_state_telusur(1);
//		}
//		else if (pewaktu > 115)
//		{
//			state_jalan = 1;
//		}
//		break;
//	case 999: //khusus untuk robot berhenti
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		if (start_on)
//		{
//			state_start = 0;
//			state_jalan = 0;
//		}
//		break;
//	case 2002:
//		konversi_gp(state_telusur);
//		gp_serong();
//		baca_gp(state_telusur);
//
//		telusurKanan_zigzag_ruang();
//
//		if (ganti_kiri >= 1)
//		{
//			penanda_stack_ruang4++;
//		}
//		else if (ganti_kanan >= 1)
//		{
//			penanda_stack_ruang4 = 0;
//		}
//
//		if (penanda_stack_ruang4 >= 7)
//		{
//			state_jalan = 2003;
//			ganti_kiri = 0;
//			ganti_kanan = 0;
//			konversi_gp(0);
//			penanda_buzzer = 0;
//			penanda_stack_ruang4 = 0;
//		}
//
//		deteksi_lost_acuan(1, 2002);
//
//		if (baca_garis() == 1)
//		{
//			lock_state_telusur = state_telusur;//lock state
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			setKompas();
//			tembak_cmps();//mulai lock acuan kompas
//			//lock_acuan = tembak_cmps();//lock
//			statusKompas = 1;
//			hitung_langkah = 0;
//			//state_jalan = 999;
//
//			if(sensor_gp[sensor[1]]<400)
//			{
//				state_jalan = 667;
//			}
//			else
//			{
//				state_jalan = 666;
//			}
//
//		}
//
//		if(sensor_gp[sensor[7]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[1]]<120)
//		{
//			if(sensor_gp[sensor[2]]<100)
//			{
//				geser_kiri(state_telusur,1);
//			}
//			else if(sensor_gp[sensor[6]]<200)
//			{
//				konversi_state_telusur(-1);
//			}
//			else
//			{
//				konversi_state_telusur(-2);
//			}
//
//		}
//		break;
//	case 2003:
//		gerak(0, 0, -9, 1, 25);
//		baca_gp(0);
//		gp_serong();
//		if (sensor_gp_asli[3]<150&&sensor_gp[sensor[7]]>100)
//		{
//			state_jalan = 2004;
//			penanda_buzzer = 0;
//			state_telusur = 3;
//		}
//		break;
//	case 2004:
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2003);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			setKompas();
//			tembak_cmps();//mulai lock acuan kompas
//			lock_acuan = tembak_cmps();//lock
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur, 1);
//			konversi_state_telusur(-1);
//			statusKompas = 1;
//			ganti_muka = 0;
//			penanda_keluar_4a_4b_1b = 1;
//			lock_state_telusur = state_telusur;
//
//			if(sensor_gp[sensor[1]]<400)
//			{
//				state_jalan = 667;
//			}
//			else
//			{
//				state_jalan = 666;
//			}
//		}
//
//		break;
//	case 2://menentukan Ruang Start
//		//jalan_langkah(state_telusur, 3);
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		if (sensor_gp[sensor[0]]<250&&sensor_gp_asli[1]<thd_dinding&&sensor_gp[sensor[6]]>thd_dinding) //start = 3 atau 2
//		{
//			penanda_buzzer = 0;
//			state_jalan = 3100;
//			ganti_muka = 0;
//		}
//		else if (sensor_gp[sensor[0]] > 300&&sensor_gp_asli[1] < thd_dinding) //start = 2 atau 1c
//		{
//			baca_gp(state_telusur);
//			penanda_buzzer = 0;
//			state_jalan = 3100;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//		}
//		else if (sensor_gp_asli[0]<thd_dinding&&sensor_gp_asli[1]>thd_dinding) //start = 1b atau 4a atau 4b
//		{
//			state_jalan = 3;
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
////		//else if (sensor_gp[sensor[0]]>thd_dinding&&sensor_gp[sensor[6]]<thd_dinding) //start = 1a, posisi sekarang=lorong
////		else if (sensor_gp[sensor[6]]<thd_dinding || penanda_keluar_1a == 1) //start = 1a, posisi sekarang=lorong
////		{
////			penanda_buzzer = 0;
////			ruang_start = 1;
////			geser_kiri(state_telusur,1);
////			penanda_pintu1=0;
////			konversi_state_telusur(-1);
////			state_jalan = 2093; //langsung menuju ke tidak ada pintu di 1a
////		}
//		break;
//
//
//		//deteksi ruang start 3, 2
//	case 3100:
//
//		switch(lock_state_telusur){
//		case 0: //muka 0
//			//r.start 3
//			if (lock_acuan == UTARA){
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 3101;
//				ruang_start = 3;
//				ruangArr[0] = 3;
//			}
//			//r.start 2
//			else if (lock_acuan == TIMUR){
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 2100; //ke case konfig ruang 2
//				ruang_start = 2;
//				ruangArr[0] = 2;
//			}
//			break;
//		case 1: //muka 1
//			//r.start 3
//			if (lock_acuan == TIMUR){
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 3101;
//				ruang_start = 3;
//				ruangArr[0] = 3;
//			}
//			//r.start 2
//			else if (lock_acuan == SELATAN){
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 2100; //ke case konfig ruang 2
//				ruang_start = 2;
//				ruangArr[0] = 2;
//			}
//			break;
//		case 2:
//			//r.start 3
//			if (lock_acuan == SELATAN){
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 3101;
//				ruang_start = 3;
//				ruangArr[0] = 3;
//			}
//			//r.start 2
//			else if (lock_acuan == BARAT){
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 2100; //ke case konfig ruang 2
//				ruang_start = 2;
//				ruangArr[0] = 2;
//			}
//
//			break;
//		case 3:
//			//r.start 3
//			if (lock_acuan == BARAT){
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 3101;
//				ruang_start = 3;
//				ruangArr[0] = 3;
//			}
//			//r.start 2
//			else if (lock_acuan == UTARA){
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 2100;//ke case konfig ruang 2
//				ruang_start = 2;
//				ruangArr[0] = 2;
//			}
//			break;
//		}
//		break;
//
//
//
//
//		//================Ruang start 3 =================//
//	case 3101:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//mendeteksi siku di ruang 3
//		if(sensor_gp_asli[3]<thd_dinding && sensor_gp_asli[1]<300) {
//			state_jalan=3111;
//			hitung_zigzag = 0;
//			hitung_langkah = 0;
//		}
//		telusurKanan_zigzag();
//		break;
//
//	case 3111:
//		if(hitung_zigzag == 1){
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 3201;
//			hitung_zigzag = 0;
//			hitung_langkah = 0;
//			konversi_state_telusur(-1);
//		}
//		else telusurKanan_zigzag();
//		break;
//
//
//
//	case 3201:
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		state_jalan=3305;
//		break;
//
//	case 3305: //acuan case 2329 3301 2270
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp_asli[0]<300){
//			lock=0;
//			next_state_jalan=LURUSKAN_CMPS;
//			state_jalan = 3306; //3298
//		}
//		else{
//			if(sensor_gp_asli[3]<thd_dinding){
//				geser_kanan(state_telusur,1);
//				lock=0;
//			}
//			else{
//				geser_kiri(state_telusur,1);
//				lock=0;
//			}
//		}
//		break;
//
//	case 3306:
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		state_jalan=3301;
//		break;
//
//	case 3301:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		//takut masuk ruang 3 lagi
//		if (baca_garis() == 1) {
//			lock_state_telusur = state_telusur;//lock state
//			penanda_buzzer = 0;
//			pewaktu=0;
//			setKompas();
//			lock_acuan=tembak_cmps();//mulai lock acuan kompas
//			hitung_langkah = 0;
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan=999;
//		}
//
//		if(sensor_gp_asli[0]<140){
//			state_jalan=LURUSKAN_CMPS;
//			konversi_state_telusur(1);
//			next_state_jalan = 3298;
//			lock=0;
//		}
//
//		break;
//
//
//
//	case 3298:
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		state_jalan=3302;
//		break;
//
//	case 3302:
//
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiriPipih();
//		//back-up takut masuk ruang 1,2,1C
//		if(baca_garis() == 1){
//			penanda_buzzer=0;
//			setKompas();
//			//masuk ruang 1A 3310
//			if(((tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) || (tembak_cmps()==TIMUR && state_telusur==0) || (tembak_cmps()==SELATAN && state_telusur==1))){
//				state_jalan = LURUSKAN_CMPS;
//				next_state_jalan=3310;
//				konversi_state_telusur(-1);
//			}
//			//masuk ruang 2
//			else if(((tembak_cmps()==BARAT && state_telusur==0) || (tembak_cmps()==UTARA && state_telusur==1) || (tembak_cmps()==TIMUR && state_telusur==2) || (tembak_cmps()==SELATAN && state_telusur==3))){
//
//			}//masuk ruang 1C anjing di posisi 1
//			else if ((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2)){
//
//			}
//			//masuk 4a
//			else if(tembak_cmps()==BARAT && state_telusur==1 || tembak_cmps()==UTARA && state_telusur==2 || tembak_cmps()==TIMUR && state_telusur==3 || tembak_cmps()==SELATAN && state_telusur==0){
//
//			}
//		}
//		//back-up takut detek anjing
//		if(detek_anjing() == 1){
//			setKompas();
//			//anjing di posisi 3
//			if((tembak_cmps()==TIMUR && state_telusur==0) ||(tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) ||(tembak_cmps()==SELATAN && state_telusur==1)){
//
//			}
//			//anjing di posisi 2
//			else if ((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2)){
//
//			}
//		}
//
//		//verifikasi lorong antara ruang 3 dan 4
//		if(((tembak_cmps()==BARAT && state_telusur==1) || (tembak_cmps()==UTARA && state_telusur==2) || (tembak_cmps()==TIMUR && state_telusur==3) || (tembak_cmps()==SELATAN && state_telusur==0)) && sensor_gp[sensor[6]]<thd_dinding && sensor_gp[sensor[5]]<thd_dinding && sensor_gp[sensor[2]]<thd_dinding){
//
//			state_jalan=3204;
//			ganti_muka=0;
//			ganti_kiri=0;
//			ganti_kanan=0;
//		}
//
//		break;
//
//	case 3204: //state_telusur menghadap ke timur
//		//kemungkinan anjing 3 atau 2
//		//udah ganti kanan
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//
//		if((tembak_cmps()==TIMUR && state_telusur==0 ||tembak_cmps()==BARAT && state_telusur==2 || tembak_cmps()==UTARA && state_telusur==3 ||tembak_cmps()==SELATAN && state_telusur==1)){
//			//anjing di posisi 3 cek diawal biar langsung memutar ruang 4
//			if(detek_anjing() == 1){
//				state_jalan=3206;
//				posisi_anjing=3;
//				konversi_state_telusur(-2);
//			}
//			else{
//				state_jalan=3205;
//				ganti_muka=0;
//				ganti_kiri=0;
//				ganti_kanan=0;
//			}
//
//		}
//
//		else telusurKiri_zigzag();
//		break;
//	case 3205:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//		//tidak ada anjing alias anjing di posisi antara 2 dan 1
//		if(ganti_kanan == 1){
//			state_jalan = 3209; //3208
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			posisi_anjing=0; //antara 1 atau 2
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_muka=0;
//		}
//
//		//jika robot deteksi anjing di posisi 3 depan ruang 4B **dicek lagi**
//		else if(detek_anjing()==1 && ((tembak_cmps()==TIMUR && state_telusur==0) ||(tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) ||(tembak_cmps()==SELATAN && state_telusur==1))){
//			posisi_anjing=3;
//			konversi_state_telusur(-2);
//			state_jalan=3206;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//		}
//		break;
//
//	//================posisi anjing = 3 ==============
//	case 3206:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKanan_zigzag();
//		if((tembak_cmps()==TIMUR && state_telusur==1 ||tembak_cmps()==BARAT && state_telusur==3 || tembak_cmps()==UTARA && state_telusur==0 ||tembak_cmps()==SELATAN && state_telusur==2)){
//			if(sensor_gp_asli[1]<thd_dinding && sensor_gp[sensor[6]]<thd_dinding && sensor_gp[sensor[5]]<thd_dinding){
//				penanda_buzzer = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//				ganti_muka=0;
//				state_jalan =3304;
//				konversi_state_telusur(-1);
//			}
//		}
//		break;
//	case 3304:
//		//mendekat ke ruang 4 dengan cara maju
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130){
//			konversi_state_telusur(1);
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			state_jalan=3207;
//			lock=0;
//		}
//		break;
//	case 3207:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//masuk 4A atau 4B anjing=3
//		telusurKiriPipih();
//
//		if(tembak_cmps()==BARAT && state_telusur==1 || tembak_cmps()==UTARA && state_telusur==2 || tembak_cmps()==TIMUR && state_telusur==3 || tembak_cmps()==SELATAN && state_telusur==0){
//			if (baca_garis() == 1) {
//					// RUANG 4A DIMASUKI
//					pewaktu=0;
//					hitung_zigzag = 0;
//					hitung_langkah=0;
//					penanda_buzzer = 0;
//					state_jalan = 3214;
//					ruangArr[1] = 4; //4A
//					ruangScan = 4; //4A
//			}
//
//			//khusus anjing 3
//			//verifikasi lorong ruang 4 dan dinding atas mengarah ke 4B
//			if(sensor_gp[sensor[6]]<thd_dinding && sensor_gp[sensor[5]]<thd_dinding && sensor_gp[sensor[0]] > 500){
//				state_jalan=3883;  //pasti 4B
//			}
//
//		}
//		break;
//
//	case 3883:
//		//masuk ruang 4B anjing == 3
//		telusurKiriPipih();
//		if(baca_garis()==1){
//			penanda_buzzer=0;
//			pewaktu=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			ruangArr[1] = 7; //4B
//			ruangScan = 7; //4B
//			state_jalan=3210;
//		}
//
//		break;
////========================================
//
//
//		//konfirmasi dinding ruang 4 jika anjing tidak sama dengan 3
//
//	case 3208:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//		if(ganti_kanan == 1){
//			ganti_muka = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			state_jalan=3209;
//		}
//
//		break;
//		//===========START case kalo misalkan gak detek ruang 4========
//	case 3355:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<120){
//			lock=0;
//			konversi_state_telusur(1);
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			state_jalan=3356;
//		}
//		break;
//	case 3356: //2136
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		telusurKiriPipih();
//		if((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2)){
//			if(baca_garis()==1){ //masuk ke ruang 4B
//				penanda_buzzer=0;
//				state_jalan=3210;
//				lock_acuan = tembak_cmps();
//				lock_state_telusur = state_telusur;
//				pewaktu=0;
//				hitung_langkah=0;
//				hitung_zigzag=0;
//				ruangArr[1] = 7; //4B
//				ruangScan = 7; //4B
//			}
//			//sepertinya konfigurasi 4A
//			//robot mengarah ke utara atau verifikasi lorong di antara ruang 3&4
//			if(sensor_gp_asli[3]<thd_dinding && sensor_gp_asli[1]<thd_dinding && sensor_gp[sensor[5]]<thd_dinding && sensor_gp[sensor[0]] > 500){
//				state_jalan = 3213; //pasti konfig 4A
//				hitung_zigzag=0;
//				hitung_langkah=0;
//				ganti_kanan=0;
//				ganti_muka=0;
//				ganti_kiri=0;
//			}
//		}
//		break;
//
//		//===========END case kalo misalkan gak detek ruang 4========
//
//
//	case 3210:
//		if(hitung_langkah == 1){
//			jalan_langkah_scan12(state_telusur, 1);
//			pewaktu=0;
//			hitung_zigzag = 0;
//			hitung_langkah=0;
//			state_jalan=3200;
//		}
//		telusurKiriPipih();
//		break;
//
//	case 3200:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp[sensor[6]]<100){
//			state_jalan=3225;
//			hitung_zigzag = 0;
//			hitung_langkah=0;
//		}
//		else geser_kiri(lock_state_telusur,1);
//		break;
//
//
//	case 3209:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//
//		if(sensor_gp[sensor[6]]<thd_dinding && ((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2))){
//			//posisi anjing 2
//			if(detek_anjing() == 1){
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan=3333;
//				posisi_anjing=2;
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//			}
//			//posisi anjing 1
//			else if(detek_anjing() !=1 && sensor_gp[sensor[3]]<thd_dinding && sensor_gp_asli[1]<thd_dinding){
//				posisi_anjing=1;
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan=3333;
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//			}
//		}
//		break;
//	case 3333:
//		konversi_state_telusur(1);
//		state_jalan=3355;
//		break;
//
//	case 3212:
//
//		//===========KONFIGURASI 4A =====================
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp_asli[3]<140){
//			state_jalan=3213;
//			ganti_muka=0;
//			ganti_kiri=0;
//			ganti_kanan=0;
//			pewaktu = 0;
//		}
//		else geser_kiri(state_telusur,1);
//		break;
//
//	case 3213:
//		//deteksi garis ruang 4A anjing 1 atau 2
//		telusurKiriPipih();
//		if(baca_garis() == 1){
//			pewaktu=0;
//			hitung_zigzag = 0;
//			hitung_langkah=0;
//			penanda_buzzer = 0;
//			state_jalan = 3214;
//		}
//		break;
//
//		//masuk ke dalam ruang 4A
//	case 3214:
//		//pengecekan apakah ruang 4A yang dimasuki?
//		telusurKiri_zigzag_ruang();
//		if(hitung_zigzag == 2){
//			if((tembak_cmps()==BARAT && state_telusur==1) || (tembak_cmps()==UTARA && state_telusur==2) || (tembak_cmps()==TIMUR && state_telusur==3) || (tembak_cmps()==SELATAN && state_telusur==0)){
//				pewaktu=0;
//				hitung_zigzag = 0;
//				hitung_langkah=0;
//				state_jalan=3215;
//				ruangArr[1] = 4; //4A
//				ruangScan = 4; //4A
//			}
//
//		}
//
//		break;
//	case 3215:
//		state_jalan=3216;
//		break;
//	case 3216: //acuan case 107
//		jalan_langkah_scan12(state_telusur, 1);
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			ruang_api = ruangScan; //4A
//			konversi_state_telusur(-1);
//			state_jalan = 3217; //201->2220
//		}
//		else
//		{
//			jalan_langkah(state_telusur, 1);
//			konversi_state_telusur(-1);
//			state_jalan = 3226; //tidak ada api di 4a
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//
//	case 3217: //acuan case 2220
//		konversi_gp(state_telusur);
//		gp_serong();
//		baca_gp(state_telusur);
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 3217);
//		if (baca_garis() == 1)
//		{
//			penanda_buzzer=0;
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(-1);
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			ganti_muka=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			state_jalan=3218; //2221
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 3222;
//		}
//		break;
//
//
//	case 3218: //acuan case 2221
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2221);
//		if (baca_garis() == 1)
//		{
//			penanda_buzzer=0;
//			pewaktu=0;
//			state_jalan = 3219; //telusur kanan selain IA dan IB 2222
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			ganti_muka = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 3222;
//		}
//		break;
//
//	case 3219: //acuan case 2222 ->121
//		//tidak ada api
//		eksistansi_api1 = 0;
//		eksistansi_api2 = 0;
//		led_api_off;
//		ruang_api = 0;
//		ganti_muka = 1;
//		state_jalan=3226;
//
//		break;
//
//	case 3222://acuan case 202
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 3223;
//			api_siap = 0;
//		}
//		break;
//
//	case 3223: //acuan case 203
//		padamkan_api();
//		//state_jalan = 204; returning acuan case 204
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//	//=========KONFIG 4B ================
//	case 3225:
//		jalan_langkah_scan12(lock_state_telusur,1);
//		if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 3217;
//			ruang_api = ruangScan;
//		}
//		else{
//			jalan_langkah(state_telusur, 1);
//			konversi_state_telusur(-1);
//			state_jalan=3226;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//
//	case 3226: //acuan case 2110
//		//mau keluar ruang 4 case global bisa 4A atau 4B
//		telusurKanan_zigzag_ruang();
//		if(baca_garis()==1)
//		{
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//			pewaktu=0;
//			tembak_cmps();
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 3230;
//		}
//		break;
//
//	case 3230:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130)
//		{
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			ganti_kanan = 0;
//			ganti_kiri=0;
//			lock=0;
//			//keluar 4B
//			if(ruangArr[1] == 7){
//				ruangArr[2] = ruangArr[1];
//				//putar balik
//				if(posisi_anjing == 2){
//					konversi_state_telusur(-1);
//					state_jalan = LURUSKAN_KANAN;
//					next_state_jalan=3232; //puter balik
//					ganti_muka = 0;
//					ganti_kanan=0;
//					ganti_kiri=0;
//				}
//				//sudah detek anjing dulu
//				else if(posisi_anjing == 1){
//					konversi_state_telusur(1);
//					ganti_muka = 0;
//					ganti_kanan=0;
//					ganti_kiri=0;
//					state_jalan=3252;
//				}
//				//===============
//				else{
//					//cek anjing dulu
//					state_jalan = 3231;
//					konversi_state_telusur(1);
//				}
//
//			}
//			//keluar 4A
//			else if (ruangArr[1] == 4){
//				ruangArr[2] = ruangArr[1];
//				state_jalan = 3240;
//			}
//		}
//		break;
//	case 3231:
//		//setelah keluar ruang 4B ***belum tau anjing di 1 atau 2
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//
//		if(((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2))){
//			//POSISI ANJING 2 KONFIG RUANG 4B
//			if(detek_anjing() == 1){
//				konversi_state_telusur(-2);
//				posisi_anjing=2;
//				state_jalan = LURUSKAN_KANAN;
//				next_state_jalan=3232; //puter balik
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//			}
//			//POSISI ANJING 1 atau 2 KONFIG RUANG 4B langsung masuk 1C
//			if(detek_anjing() != 1 && sensor_gp[sensor[2]]<thd_dinding){
//				//posisi anjing 3 sudah ditetapin sebelumnya
//				if(posisi_anjing == 3){
//					posisi_anjing=3;
//				}
//				else{
//					posisi_anjing=1;
//				}
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//				state_jalan=3252;
//			}
//		}
//		telusurKiri_zigzag();
//		break;
//
//		/*=======================================*/
//	case 3252: //ingin masuk 1C
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//		//cek anjing lagi
//		if (detek_anjing() == 1) {
//			konversi_state_telusur(-2);
//			posisi_anjing=2;
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan=3232; //puter balik
//			ganti_muka = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//		}
//		//konfig 4B masuk 1C apabila anjing 1
//		if(baca_garis() == 1){
//			if((tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) ||(tembak_cmps()==SELATAN && state_telusur==2)){
//				lock_state_telusur = state_telusur;
//				penanda_buzzer = 0;
//				pewaktu=0;
//				setKompas();
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				lock_acuan = tembak_cmps();
//				state_jalan = LURUSKAN_KIRI;
//				ruangArr[3] = 6; //1C
//				ruangScan = 6; //1C
//				next_state_jalan = 3253;
//				jalan_langkah_scan12(state_telusur,2);
//			}
//		}
//		break;
//
//	case 3253: //acuan case 3206 masuk 1C
//		//masuk ruang 1C
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (hitung_langkah == 3 || sensor_gp[sensor[0]]<100)
//		{
//			lock=0;
//			//deteksi api
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//			{
//				//dicek lagi apinya
//				state_jalan = 3254;
//				pewaktu=0;
//				konversi_state_telusur(1);
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			//tidak deteksi api
//			else
//			{
//				hitung_langkah=0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//				state_jalan = 3267;
//				konversi_state_telusur(-1); //supaya state telusur ke tembok (ke arah timur);
//			}
//		}
//		break;
//
//	case 3254: //acuan case 3211
//		if(nilai_api<thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 3255;
//			ruang_api = ruangScan;
//		}
//		else if(eksistansi_api1 == 1 || eksistansi_api2 == 1){
//			maju(state_telusur);
//			state_jalan=3258;
//		}
//		else jalan_langkah_scan12(state_telusur,1);
//		break;
//
//	case 3255: //acuan case 3208
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 3256;
//			api_siap = 0;
//		}
//		break;
//
//	case 3256: //acuan case 3209
//		padamkan_api();
//		state_jalan = 3257; // cek apakah api sudah mati apa belum
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//	case 3257: //acuan case 3210
//		if (deteksi_api(4000) == 1)
//		{
//			//api masih nyala
//			//state_jalan = 2201;
//
//			break;
//		}
//		else{
//			led_api_off;
//			//return belum diset
//			state_jalan=999; //sementara diset berhenti
//		}
//		break;
//
//	case 3258: //acuan case 3207
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		//obstacle
//		if(sensor_gp_asli[0]<200)
//		{
//			lock=0;
//			//state_jalan = 115; belum diset
//		}
//		else if (nilai_api > thd_api)
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 3255; //kembali ke case sebelumnya mencari keberadaan api
//		}
//		else maju(state_telusur);
//		break;
//
//		//cek apakah ada api lagi
//	case 3267:
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api2 == 1||eksistansi_api1 == 1){
//			eksistansi_api1 = 0;
//			eksistansi_api2 = 0;
//			state_jalan=3254; //3254
//		}
//		if (baca_garis() == 1)
//		{
//			if(ruangArr[3] == 6 && ruangScan == 6){
//				pewaktu = 0;
//				setKompas();
//				lock_acuan = tembak_cmps();
//				lock_state_telusur = state_telusur;
//				state_jalan = 3268;
//				jalan_langkah(lock_state_telusur,2);
//			}
//		}
//		else telusurKiri_zigzag_ruang();
//		break;
//
//	case 3268:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<thd_dinding){
//			state_jalan = LURUSKAN_KIRI;
//			ruangArr[4] = 1; //keluar ruang 1A
//			next_state_jalan = 3283;
//			penanda_pintu1=0;
//			pewaktu=0;
//			setKompas();
//			while (baca_garis() == 1)telusurKiri_zigzag();
//
//		}
//		//ruang 1b
//		else{
//			penanda_pintu1=1;
//			ruangArr[4] = 5; //1B
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			setKompas();
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 3263;
//			lock_acuan = tembak_cmps();
//		}
//		break;
//		/*=======================================*/
//
//	case 3232:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//posisi anjing 2 konfig 4B putar balik
//		telusurKanan_zigzag();
//		if((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2)){
//			if((sensor_gp_asli[3]<thd_dinding) && sensor_gp_asli[1]<thd_dinding){ //330
//				state_jalan = 3233;
//				hitung_zigzag=0;
//				hitung_langkah=0;
//				ganti_kanan=0;
//				ganti_muka=0;
//				ganti_kiri=0;
//			}
//		}
//		break;
//
////==== start ingin masuk 1C
//	case 3233:
//		//mendekat ke ruang 4 dengan cara maju **konfig 4B anjing 2
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp_asli[3]<140){
//			state_jalan=3234;
//			ganti_muka=0;
//			ganti_kiri=0;
//			ganti_kanan=0;
//			pewaktu = 0;
//		}
//		else geser_kiri(state_telusur,1);
//		break;
//
//	case 3234:
//		//konfig 4B anjing 2
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//
//		telusurKiri_zigzag();
//		if(ganti_kiri==2){
//			konversi_state_telusur(1);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			//posisi anjing 2 dan harus masuk 1C
//			if(sensor_gp[sensor[7]]>thd_dinding && sensor_gp[sensor[1]]>thd_dinding && ((tembak_cmps()==TIMUR && state_telusur==0) ||(tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) ||(tembak_cmps()==SELATAN && state_telusur==1))){
//				state_jalan = 3235;
//			}
//		}
//		//antisipasi takut detek anjing lagi
//		if(detek_anjing() == 1){
//			posisi_anjing = 1;
//			state_jalan = 3230;
//		}
//
//		break;
//
//	case 3235:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130){
//			lock=0;
//			konversi_state_telusur(1);
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 3236;
//		}
//		break;
//	case 3236:
//		//konfig 4B masuk 1C apabila anjing 1
//		telusurKiri_zigzag();
//		if(baca_garis() == 1){
//			if((tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) ||(tembak_cmps()==SELATAN && state_telusur==2)){
//				lock_state_telusur = state_telusur;
//				penanda_buzzer = 0;
//				pewaktu=0;
//				setKompas();
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				lock_acuan = tembak_cmps();
//				state_jalan = LURUSKAN_KIRI;
//				ruangArr[3] = 6; //1C
//				ruangScan = 6; //1C
//				next_state_jalan = 3253;
//				jalan_langkah_scan12(state_telusur,2);
//			}
//		}
//		break;
//
//		//=======================
//
//	case 3240:
//		//konfigurasi 4A
//		if(posisi_anjing==1){
//			//siap masuk ke ruang 1
//			konversi_state_telusur(1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=3241; //masuk antara 1A atau 1B
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			pewaktu=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//		}
//		//posisi anjing 2 dan 3 siap masuk 1 C **sementara dibuat masuk 1A atau 1B dulu
//		else if(posisi_anjing == 2 || posisi_anjing==3){
//			konversi_state_telusur(-1); //cek kembali apakah ada anjing di posisi 1
//			state_jalan=LURUSKAN_KANAN;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			next_state_jalan=3320;
//			pewaktu=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//		}
//		break;
//
//	case 3320:
//		if(detek_anjing() == 1){
//			posisi_anjing = 1;
//			konversi_state_telusur(-2);
//			state_jalan=3241;
//		}
//		//anjing di antara posisi 1 atau 2
//		else {
//			state_jalan=3321;
//			pewaktu=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//		}
//		break;
//
//		//mau masuk menuju ruang 1C atau 6
//	case 3321:
//		telusurKanan_zigzag();
//		if(ganti_kanan == 1){
//			konversi_state_telusur(-1);
//			state_jalan = 3322;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//		}
//
//		break;
//
//	case 3322:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130){
//			lock=0;
//			konversi_state_telusur(1);
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 3236;
//		}
//		break;
//
//
//	//masuk antara 1A atau 1B
//	case 3241:
//		telusurKiriPipih();
//		//masih belum bisa menentukan ruang 1A atau 1B setelah detek garis
//		if(baca_garis() == 1){
//			setKompas();
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 3242;
//			penanda_buzzer=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			lock_acuan=tembak_cmps();
//		}
//		break;
//
//	case 3242:
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 2;
//		}
//		jalan_langkah(state_telusur,3);
//		state_jalan=3243;
//		break;
//
//	case 3243: //acuan case 12
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//persyaratan ruang 1B
//		if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<300){
//			ruangScan = 5; //1B = 5
//			ruangArr[3] = 5;
//			konversi_state_telusur(-1);
//			ganti_muka=0;
//			ganti_kiri=0;
//			hitung_langkah=0;
//			state_jalan=3245;
//		}
//		//persyaratan ruang 1A
//		else if(sensor_gp[sensor[2]]<thd_dinding){
//			ruangScan = 1; //1A = 1
//			ruangArr[3] = 1;
//			konversi_state_telusur(1);
//			ganti_muka=0;
//			ganti_kiri=0;
//			hitung_langkah=0;
//			state_jalan=3271;
//		}
//
//		break;
//
//
//
////	case 3244:
////		konversi_state_telusur(-1);
////		ganti_muka=0;
////		ganti_kiri=0;
////		hitung_langkah=0;
////		state_jalan=3245;
////		break;
//
//		//Konfigurasi 1B
//	case 3245: //acuan case 1110
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1 = 1;
//			state_jalan = 3246;
//			hitung_langkah = 0;
//		}
//		break;
//
//	case 3246: //acuan case 201
//		telusurKiri_zigzag_ruang();
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			state_jalan = 3247;
//			ruang_api = ruangScan;
//		}
//		//tidak deteksi api
//		else
//		{
//			state_jalan = 3260;
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		break;
//	case 3247: //acuan case 9006
//		muter_acuankompas(acuan_1b);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=3248;
//		}
//		break;
//
//	case 3248: //acuan case 9001
//		muter_(kan,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			state_jalan=3250;
//		}
//		else if(pewaktu>110)
//		{
//			if(ruang_api==5)
//			{
//				pewaktu=0;
//				state_jalan=3249;
//			}
//
//		}
//		break;
//
//	case 3249: //acuan case 9009
//		muter_(kir,5,25);
//		if(pewaktu>50 && ruang_api==5)
//		{
//			state_telusur=0;
//			lock_state_telusur=state_telusur;
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			geser_kiri(state_telusur,1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=999; //berhenti deteksi error
//		}
//		break;
//
//	case 3250: //acuan case 9003
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=3251;
//		}
//		break;
//
//	case 3251: //acuan case 9004
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 3247;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//			if(ruang_api==5 && status_tpa==1)
//			{
//				mataangin = SELATAN;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 999; //berhenti dulu
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//		}
//		break;
//
//	//tidak deteksi api di 1B
//	case 3260: //acuan case 2107
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			lock=0;
//			konversi_state_telusur(-1);
//			state_jalan = 3261;
//		}
//		break;
//
//	case 3261: //acuan case 98
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KANAN;
//		next_state_jalan = 3262;
//		break;
//
//	case 3262: //acuan case 99 keluar 1B
//		telusurKanan_zigzag_ruang();
//		if (baca_garis() == 1)
//		{
//			ruangArr[4] = 5; //1B
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//			setKompas();
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 3263;
//			lock_acuan = tembak_cmps();
//		}
//		break;
//
//	case 3263:
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 2;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 0;
//		}
//		state_jalan=3264;
//		break;
//
//	case 3264:
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<130)
//		{
//			lock=0;
//			next_state_jalan = 3265;
//			konversi_state_telusur(-1);
//			state_jalan=LURUSKAN_KANAN;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//
//	case 3265: //acuan case 4
//		telusurKanan_zigzag();
//		if (ganti_kanan == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 3266;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//
//	case 3266: //acuan case 90
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 130)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 3283; //siap ingin masuk ruang 2
//		}
//		break;
//
//
//
//		//konfigurasi 1A
//
//	case 3271: //acuan case 1011
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1 = 1;
//			state_jalan = 3272;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			penanda_pintu1=0;
//		}
//		break;
//
//	case 3272: //acuan case 1012
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (hitung_langkah == 1)//anti detect diruang 2
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			konversi_gp(state_telusur);
//			jalan_langkah_scan12(state_telusur, 2);
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 3273;
//				ruang_api = ruangScan;
//			}
//			else if (eksistansi_api2 == 1)
//			{
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-1);
//				jalan_langkah(state_telusur, 2);
//				jalan_langkah_scan1(state_telusur, 2);
//				if (eksistansi_api1 == 1)
//				{
//					penanda_buzzer = 0;
//					konversi_state_telusur(-2);
//					jalan_langkah(state_telusur, 3);
//					state_jalan = 3273;
//					ruang_api = ruangScan;
//				}
//				else
//				{
//					eksistansi_api2 = 0;
//					eksistansi_api1 = 0;
//					konversi_state_telusur(1); //-2x`
//					state_jalan = 3280;
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//
//				}
//			}
//			else if(hitung_langkah<=5&&sensor_gp[sensor[0]]<120)
//			{
//				konversi_state_telusur(-1);
//				hitung_langkah = 0;
//			}
//			else
//			{
//				konversi_state_telusur(1);
//				state_jalan = 3280;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//		}
////		else if(sensor_gp[sensor[0]]<120){
////
////		}
//		else if (nilai_api > thd_api&&hitung_langkah<3)
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			//state_jalan = 202;
//			ruang_api = ruangScan;
//		}
//		break;
//
//	case 3273: //acuan case 9005
//		muter_acuankompas(acuan_1a);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=3274;
//		}
//		break;
//
//	case 3274: //acuan case 9002
//		muter_(kir,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			jalan_langkah(state_telusur,1);
//			state_jalan=3275;
//		}
//		else if(pewaktu>110)
//		{
//			if(ruang_api==1)//tpa tdk dapat
//			{
//				pewaktu=0;
//				//state_jalan=9010;
//			}
//		}
//		break;
//
//	case 3275: //acuan case 9003
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=3276;
//		}
//		break;
//
//	case 3276: //acuan case 9004
//		//jika api nyala lagi
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 3272;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//			if(ruang_api == 1 && status_tpa == 1)//acuan sesudah tpa
//			{
//				mataangin = UTARA;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 999; //return belum diset
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//		}
//		break;
//
//	//tak deteksi api di ruang 1A
//	case 3280: //acuan case 96
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan = 3281;
//		}
//		break;
//
//	case 3281: //acuan case 97
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KIRI;
//		next_state_jalan = 3282;
//		break;
//
//	case 3282: //acuan case 2092
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //garis di 1a
//		{
//			pewaktu=0;
//			setKompas();
//			while (baca_garis() == 1)telusurKiri_zigzag();
//			next_state_jalan = 3283;
//			state_jalan = LURUSKAN_KIRI;
//			ruangArr[4] = 1; //keluar ruang 1A
//			penanda_pintu1=0;
//		}
//		break;
//
//
////sudah keluar ruang 1A atau 1B
//	case 3283: //acuan case 2093
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //garis di 2 masuk ruang 2
//		{
//			if(ruangArr[3] == 5 && ruangArr[4] ==5){
//				penanda_pintu1=1;
//			}
//			pewaktu=0;
//			penanda_buzzer=0;
//			setKompas();
//			hitung_zigzag = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = 3284;
//			ruangArr[5] = 2;
//			ruangScan=2;
//		}
//		break;
//
//	case 3284: //acuan case 93
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]]<100) //scan di ruang 2 memang harus maju dikit
//		{
//			lock=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 3285;  //padamkan api di ruang II belum 663
//				eksistansi_api2 = 0;
//				hitung_langkah = 0;
//			}
//			else
//			{
//				konversi_state_telusur(-1);
//				state_jalan = 3290; //94
//				ganti_muka = 0;
//			}
//		}
//		break;
//
//	case 3285: //acuan case 663
//		konversi_gp(state_telusur);
//		jalan_langkah(state_telusur,2);
//		jalan_langkah_scan12(state_telusur,2);
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 3286; //538
//			ruang_api = ruangScan;
//		}
//		else if(sensor_gp[sensor[0]]<120)
//		{
//			geser_kanan(state_telusur,2);
//			//state_jalan = 517; belum diset
//		}
//		else
//		{
//			baca_gp(state_telusur);
//			if(sensor_gp_asli[1]<sensor_gp_asli[3])
//			{
//				konversi_state_telusur(1);
//				lock_state_telusur = state_telusur;
//				//state_jalan = 540; belum diset
//				keluar_kiri = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			else
//			{
//				konversi_state_telusur(-1);
//				lock_state_telusur = state_telusur - 2;
//				if(lock_state_telusur<0)lock_state_telusur+=4;
//				//state_jalan = 540; belum diset
//				keluar_kanan = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//		}
//		break;
//
//	case 3286: //acuan case 538
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			state_jalan = 3287;
//		}
//		break;
//
//	case 3287: //acuan case 2220
//		gp_serong();
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 3287);
//		if (baca_garis() == 1) //antisipasi
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			//state_jalan = ; //2221 belum diset
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 3288;
//		}
//		break;
//
//	case 3288: //acuan case 202
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 3289; //203
//			api_siap = 0;
//		}
//		break;
//
//	case 3289: //acuan case 203
//		padamkan_api();
//		state_jalan = 999; //belum diset
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//
//	//mau keluar ruang 2 tidak detek api
//	case 3290: //acuan case 94
//		telusurKanan_zigzag_ruang();
//		if (baca_garis() == 1)
//		{
//			pewaktu=0;
//			setKompas();
//			tembak_cmps();
//			lock_state_telusur = state_telusur;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			penanda_buzzer=0;
//			state_jalan=3291;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			ruangArr[6] = 2;
//		}
//		break;
//
//	case 3291: //acuan case 2007
//		telusurKanan_zigzag();
//		if (hitung_zigzag == 2)
//		{
//			//konversi_state_telusur(-1);
//			next_state_jalan=3292;
//			state_jalan = LURUSKAN_KANAN;
//			ganti_muka = 0;
//			if(ruangArr[3] == 5 && ruangArr[4] ==5){
//				penanda_khusus_1a = 0;
//			}
//		}
//
//		break;
//
//	case 3292:
//		konversi_state_telusur(-1);
//		state_jalan=3294;
//		break;
//
//	case 3294: //acuan case 2106
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp_asli[1] < thd_dinding) //4
//		{
//			lock=0;
//			jalan_langkah(state_telusur, 2);
//			penanda_buzzer = 0;
//			state_jalan = 3295;
//		}
//		break;
//
//
//	case 3295: //acuan case 2108
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiriPipih();
//		if((tembak_cmps()==TIMUR && state_telusur==2) ||(tembak_cmps()==BARAT && state_telusur==0) || (tembak_cmps()==UTARA && state_telusur==1) ||(tembak_cmps()==SELATAN && state_telusur==3)){
//			if(sensor_gp_asli[3]<thd_dinding && sensor_gp_asli[1]<thd_dinding) {
//				ganti_kanan=0;
//				ganti_kiri=0;
//				ganti_muka=0;
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				penanda_buzzer=0;
//				state_jalan=3296;
//			}
//		}
//		break;
//
//	case 3296:
//		if(hitung_zigzag == 1){
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 3301; //sudah selesai konfigurasi start ruang 2
//			hitung_zigzag = 0;
//			hitung_langkah = 0;
//			konversi_state_telusur(1);
//		}
//		else telusurKiri_zigzag();
//		break;
////============== MUTER LAGI :))
//
////============== KHUSUS BACK-UP JIKA TERJADI ERROR pada ruang start 3
//
//		//jika masuk ruang 1A
//	case 3310:
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 2;
//		}
//		jalan_langkah(state_telusur,3);
//		state_jalan=3311;
//		break;
//
//	case 3311:
//		if(sensor_gp[sensor[2]]<thd_dinding){
//			ruangScan = 1; //1A = 1
//			ruangArr[1] = 1;
//			konversi_state_telusur(1);
//			ganti_muka=0;
//			ganti_kiri=0;
//			hitung_langkah=0;
//			state_jalan=3312;
//		}
//		break;
//
//	case 3312:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp_asli[0] < 100) //1
//		{
//			lock=0;
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1 = 1;
//			state_jalan = 3313;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			penanda_pintu1=0;
//		}
//		break;
//
//
//
//
//
//
//
//
//	case 3:
////		telusurKiri_zigzag();
////		if (ganti_muka == 0&&(hitung_zigzag == 0 || hitung_zigzag == 1))
////		{
////			if (sensor_gp_asli[3] < 400)count_penanda_anjing3++;
////			else count_penanda_anjing3 = 0;
////
////			if (count_penanda_anjing3 > 2)
////			{
////				penanda_buzzer = 0;
////				posisi_anjing = 3;
////				count_penanda_anjing3 = 0;
////			}
////		}
////		if (ganti_kanan == 2 && ganti_kiri == 0)
////		{
////			penanda_buzzer = 0;
////			ruang_start = 7; //start = 4b, posisi sekarang lorong
////			if (posisi_anjing == 3)
////			{
////				state_jalan = 8;
////			}
////			else
////			{
////				//gerakStatic(0,0,0,0,0,0,30);
////				konversi_state_telusur(-2);
////				state_jalan = 133; //dari IVb menuju ke III
////				ganti_muka = 0;
////			}
////		}
////		if (ganti_kiri == 1&&ganti_kanan == 2)
////		{
////			penanda_buzzer = 0;
////			ruang_start = 5; //start = 1b, posisi sekarang lorong dekat pintu 3
////			konversi_state_telusur(-2);
////			state_jalan = 6;
////			posisi_anjing = 0;
////			penanda_pintu1 = 1;
////		}
//
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//ruang start 4A | 4
//		if (((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2))){
//			if(sensor_gp[sensor[2]]>thd_dinding){
//				penanda_buzzer = 0;
//				ruang_start = 4; //start = 4A
//				ruangArr[0] = 4; //start 4A
//				setKompas();
//				konversi_state_telusur(-1);
//				next_state_jalan=4002;
//				state_jalan=LURUSKAN_KANAN;
//				ganti_kanan=0;
//				ganti_kiri=0;
//				ganti_muka=0;
//			}
//			//kemungkinan start ruang 3 jaga-jaga
//			if(sensor_gp[sensor[2]]<thd_dinding){
//
//			}
//		}
//		//ruang start 4B | 7
//		if (((tembak_cmps()==BARAT && state_telusur==1) || (tembak_cmps()==UTARA && state_telusur==2) || (tembak_cmps()==TIMUR && state_telusur==3) || (tembak_cmps()==SELATAN && state_telusur==0))){
//			//ruang start 4B
//			if(sensor_gp[sensor[2]]>thd_dinding){
//				penanda_buzzer=0;
//				ruang_start = 7; //4B
//				ruangArr[0] = 7;
//				konversi_state_telusur(1);
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan = 7004; //4B
//				ganti_kanan=0;
//				ganti_kiri=0;
//				ganti_muka=0;
//				setKompas();
//			}
//			//kemungkinan start ruang 1C jaga-jaga
//			if(sensor_gp[sensor[2]]<thd_dinding){
//
//			}
//		}
//			//state_jalan = 700;
//
////			ganti_kanan = 0;
////			ganti_kiri	= 0;
//			//posisi_anjing = 0;
//
//		//{
//			//kirim_case();
////			penanda_buzzer = 0;
////			pewaktu=0;
////			ruang_start = 4; //start = 4a, posisi sekarang 1a/1b
////			state_jalan = 700;
////			setKompas();
////			ganti_kanan = 0;
////			ganti_kiri	= 0;
////			posisi_anjing = 0;
////		}
////		if(ganti_kanan == 2 && ganti_kiri == 2 )//lost dari 4a ke 1a
////		{
////			state_jalan = LURUSKAN_CMPS;
////			next_state_jalan = 528;
////			state_telusur = lock_state_telusur;
////		}
//		break;
//
//
//
//		//====================== START RUANG 4A ========================//
//	case 4002: //ingin ngecek anjing di posisi 1
//		telusurKanan_zigzag();
//		//posisi anjing 1
//		if(detek_anjing() == 1){
//			posisi_anjing=1;
//			konversi_state_telusur(-2);
//			state_jalan=LURUSKAN_KIRI;
//			//langsung ke ruang 1A atau 1B
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_muka=0;
//			next_state_jalan=4003;
//		}
//		//posisi anjing bukan 1 kemungkinan 2 atau 3 masuk lewat 1C
//		if(ganti_kanan == 1){
//			konversi_state_telusur(-1);
//			state_jalan=4066; //4066
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//		}
//
//		break;
//
////============= POSISI ANJING 1 ==================
//	case 4003: //acuan case 3241
//		telusurKiriPipih();
//		if(baca_garis() == 1){
//			setKompas();
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 4004;
//			penanda_buzzer=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			lock_acuan=tembak_cmps();
//		}
//		break;
//
//	case 4004: //acuan case 3242
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 2;
//		}
//		jalan_langkah_scan12(state_telusur,3);
//		state_jalan=LURUSKAN_CMPS;
//		next_state_jalan=4005;
//		break;
//
//	case 4005: //acuan case 3243
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//persyaratan ruang 1B
//		if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<300){
//			ruangScan = 5; //1B = 5
//			ruangArr[1] = 5;
//			konversi_state_telusur(-1);
//			ganti_muka=0;
//			ganti_kiri=0;
//			hitung_langkah=0;
//			state_jalan=4035;
//		}
//		//persyaratan ruang 1A
//		else{
//			ruangScan = 1; //1A = 1
//			ruangArr[1] = 1;
//			konversi_state_telusur(1);
//			ganti_muka=0;
//			ganti_kiri=0;
//			hitung_langkah=0;
//			state_jalan=4006;//state_jalan=3271;
//		}
//		break;
//
//	case 4006: //acuan case 3271 1011
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(1);
//			state_jalan = 4008;
//			ruang_api = ruangScan;
//		}
//		else if (eksistansi_api2 == 1)
//		{
//			eksistansi_api1 = 0;
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 2);
//			jalan_langkah_scan1(state_telusur, 2);
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(-2);
//				jalan_langkah(state_telusur, 3);
//				state_jalan = 4008;
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				konversi_state_telusur(1); //-2x`
//				state_jalan = 4012;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//
//			}
//		}
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1 = 1;
//			state_jalan=4007;//state_jalan = 3272;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			penanda_pintu1=0;
//		}
//		break;
//
//	case 4007: //acuan case 3272 1012
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if (eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(1);
//			state_jalan = 4008;
//			ruang_api = ruangScan;
//		}
//		else if (eksistansi_api2 == 1)
//		{
//			eksistansi_api1 = 0;
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 2);
//			jalan_langkah_scan1(state_telusur, 2);
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(-2);
//				jalan_langkah(state_telusur, 3);
//				state_jalan = 4008;
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				konversi_state_telusur(1); //-2x`
//				state_jalan = 4012;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//
//			}
//		}
//		else if(hitung_langkah<=5&&sensor_gp[sensor[0]]<120)
//		{
//			konversi_state_telusur(-1);
//			hitung_langkah = 0;
//		}
//		else
//		{
//			konversi_state_telusur(1);
//			state_jalan = 4012;
//			ganti_muka = 0;
//			hitung_zigzag = 0;
//		}
//
//
//		if (nilai_api > thd_api&&hitung_langkah<3)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			//state_jalan = 202; belum diset
//			ruang_api = ruangScan;
//		}
//		break;
//
//	//deteksi api di ruang 1A
//	case 4008: //acuan case 3273 9005
//		muter_acuankompas(acuan_1a);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=4009;//state_jalan=3274;
//		}
//		break;
//
//	case 4009 : //acuan case 3274 9002
//		muter_(kir,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			jalan_langkah(state_telusur,1);
//			state_jalan=4010;//state_jalan=3275;
//		}
//		else if(pewaktu>110)
//		{
//			if(ruang_api==1)//tpa tdk dapat
//			{
//				pewaktu=0;
//				//state_jalan=9010; belum diset
//			}
//		}
//		break;
//	case 4010: //acuan case 3275 9003
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=4011;
//		}
//		break;
//
//	case 4011: //acuan case 3276 9004
//		//jika api nyala lagi
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 4007;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//			if(ruang_api == 1 && status_tpa == 1)//acuan sesudah tpa
//			{
//				mataangin = UTARA;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 999; //return belum diset
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//		}
//		break;
//
//		//tidak deteksi api di ruang 1A
//	case 4012: //acuan case 3280 96
//		baca_gp(state_telusur);
//		maju(state_telusur); //mentokin tembok
//		if(sensor_gp_asli[0]<75)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan = 4014;
//		}
//		break;
//
//	case 4014: //acuan case 3281 97
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KIRI;
//		next_state_jalan = 4015;
//		break;
//
//	case 4015: //acuan case 3282 2092
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //garis di 1a
//		{
//			pewaktu=0;
//			setKompas();
//			while (baca_garis() == 1)telusurKiri_zigzag();
//			next_state_jalan = 4016;
//			state_jalan = LURUSKAN_KIRI;
//			ruangArr[2] = 1; //keluar ruang 1A
//			penanda_pintu1=0;
//		}
//		break;
//
//
////============== konfigurasi 1B
//	case 4035: //acuan case 3245 1110
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1 = 1; //1B
//			state_jalan = 4036;
//			hitung_langkah = 0;
//			penanda_khusus_1a=0;
//		}
//		break;
//
//	case 4036: //acuan case 3246 201 cek api ruang 1B
//		telusurKiri_zigzag_ruang();
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			state_jalan = 4037;
//			ruang_api = ruangScan;
//		}
//		//tidak deteksi api ruang 1B
//		else
//		{
//			state_jalan = 4045;
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		break;
//	case 4037: //acuan case 3247 9006
//		muter_acuankompas(acuan_1b);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=4038;
//		}
//		break;
//
//	case 4038: //acuan case 3248 9001
//		muter_(kan,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			state_jalan=4040;
//		}
//		else if(pewaktu>110)
//		{
//			if(ruang_api==5) //khusus di 1B
//			{
//				pewaktu=0;
//				state_jalan=4039;
//			}
//
//		}
//		break;
//
//	case 4039: //acuan case 3249 9009
//		muter_(kir,5,25);
//		if(pewaktu>50 && ruang_api==5) //api di 1B
//		{
//			state_telusur=0;
//			lock_state_telusur=state_telusur;
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			geser_kiri(state_telusur,1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=999; //berhenti deteksi error asli case 2220 belum diset
//		}
//		break;
//
//	case 4040: //acuan case 3250 9003
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=4041;
//		}
//		break;
//
//	case 4041: //acuan case 3251 9004
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 4037;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//			if(ruang_api==5 && status_tpa==1)
//			{
//				mataangin = SELATAN;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 999; //berhenti dulu belum diset case 498
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//		}
//		break;
//
//	case 4045: //acuan case 3260 2107
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			lock=0;
//			konversi_state_telusur(-1);
//			state_jalan = 4046;
//		}
//		break;
//
//	case 4046: //acuan case 3261 98
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KANAN;
//		next_state_jalan = 4047;
//		break;
//
//	case 4047: //acuan case 3262 99 keluar ruang 1B
//		telusurKanan_zigzag_ruang();
//		if (baca_garis() == 1)
//		{
//			if(ruangArr[3] == 5){
//				ruangArr[4] = 5;
//			}
//			else if (ruangArr[1] == 5){
//				ruangArr[2] = 5; //1B
//			}
//
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//			setKompas();
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 4048;
//			lock_acuan = tembak_cmps();
//		}
//		break;
//
//	case 4048: //acuan case 3263
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 2;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 0;
//		}
//		state_jalan=4049;
//		break;
//
//	case 4049: //acuan case 3264
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<130)
//		{
//			lock=0;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			//putaran pertama
//			if(ruangArr[2] == 5){
//				next_state_jalan = 4050;
//				konversi_state_telusur(-1);
//				state_jalan=LURUSKAN_KANAN;
//			}
//			//putaran kedua
//			else if(ruangArr[3] == 5 && ruangArr[4] == 5){
//				konversi_state_telusur(1);
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan = 4029; //siap masuk ruang 3 kembali utk putaran ketiga
//			}
//
//		}
//		break;
//
//	case 4050: //acuan case 3265 4
//		telusurKanan_zigzag();
//		if (ganti_kanan == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 4051;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//
//	case 4051: //acuan case 3266 90
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 130)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 4016; //siap ingin masuk ruang 2
//		}
//		break;
//
//		//====================================
//
//
//		//sudah keluar ruang 1A atau 1B dan ingin masuk ruang 2
//	case 4016: //acuan case 3283 2093
//		//perputaran pertama
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //garis di 2 masuk ruang 2
//		{
//			if(ruangArr[2] ==5){
//				penanda_pintu1=1;
//			}
//			else{
//				penanda_pintu1=0;
//			}
//			pewaktu=0;
//			penanda_buzzer=0;
//			setKompas();
//			hitung_zigzag = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = 4017;
//			ruangArr[3] = 2;
//			ruangScan=2;
//		}
//		break;
//
//	case 4017://acuan case 3284 93
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]]<100) //scan di ruang 2 memang harus maju dikit
//		{
//			lock=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 4018;  //padamkan api di ruang II belum 663
//				eksistansi_api2 = 0;
//				hitung_langkah = 0;
//			}
//			else //tidak deteksi api
//			{
//				konversi_state_telusur(-1);
//				state_jalan = 4025; //94 belum diset
//				ganti_muka = 0;
//			}
//		}
//		break;
//
//
//	case 4018://acuan case 3285 663 deteksi api di ruang 2
//		konversi_gp(state_telusur);
//		jalan_langkah(state_telusur,2);
//		jalan_langkah_scan12(state_telusur,2);
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 4019; //538
//			ruang_api = ruangScan;
//		}
//		else if(sensor_gp[sensor[0]]<120)
//		{
//			geser_kanan(state_telusur,2);
//			//state_jalan = 517; belum diset
//		}
//		else
//		{
//			baca_gp(state_telusur);
//			if(sensor_gp_asli[1]<sensor_gp_asli[3])
//			{
//				konversi_state_telusur(1);
//				lock_state_telusur = state_telusur;
//				//state_jalan = 540; belum diset
//				keluar_kiri = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			else
//			{
//				konversi_state_telusur(-1);
//				lock_state_telusur = state_telusur - 2;
//				if(lock_state_telusur<0)lock_state_telusur+=4;
//				//state_jalan = 540; belum diset
//				keluar_kanan = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//		}
//
//		break;
//
//	case 4019: //acuan case 3286 538
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			state_jalan = 4020;
//		}
//		break;
//
//	case 4020: //acuan case 3287 2220
//		gp_serong();
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 4020);
//		if (baca_garis() == 1) //antisipasi
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			//state_jalan = ; //2221 belum diset
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 4021;
//		}
//		break;
//
//	case 4021: //acuan case 3288 202
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 4022; //203
//			api_siap = 0;
//		}
//		break;
//
//	case 4022: //acuan case 3289 203
//		padamkan_api();
//		state_jalan = 999; //belum diset
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//		//mau keluar ruang 2 tidak deteksi api
//
//	case 4025: //acuan case 3290 94
//		//ingin keluar ruang 2
//		telusurKanan_zigzag_ruang();
//		if (baca_garis() == 1)
//		{
//			pewaktu=0;
//			setKompas();
//			tembak_cmps();
//			lock_state_telusur = state_telusur;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			penanda_buzzer=0;
//
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			//putaran kedua
//			if(ruangArr[1] == 2){
//				ruangArr[2] = 2;
//			}
//			//putaran pertama
//			else if (ruangArr[3] == 2){
//				ruangArr[4] = 2;
//			}
//			state_jalan=4026;
//
//		}
//		break;
//
//	case 4026: //acuan case 3291 2007
//		//maju langkah keluar ruang 2
//		telusurKanan_zigzag();
//		if (hitung_zigzag == 2)
//		{
//
//			ganti_muka = 0;
//
//			//putaran kedua misal 1B
//			if(ruangArr[1] == 2 && ruangArr[2] == 2){
//				state_jalan=4060;
//				//konversi_state_telusur(1);
//			}
//
//			//puataran pertama 1 1B
//			if(ruangArr[2] ==5){
//				ganti_muka=0;
//				ganti_kiri=0;
//				ganti_kanan=0;
//				penanda_khusus_1a = 0; //1B
//				penanda_pintu1 = 1;
//				state_jalan=4095;
//			}
//
//			//putaran pertama 1A
//			else if(ruangArr[2] == 1){ //1A
//				state_jalan = LURUSKAN_KANAN;
//				penanda_pintu1 = 0; //1A
//				penanda_khusus_1a = 1;
//				next_state_jalan=4027;
//			}
//			//posisi anjing 2 dan 3
//			else{
//				next_state_jalan=4027;
//			}
//		}
//		break;
//
//	case 4027: //acuan case 3292
//		konversi_state_telusur(-1);
//		state_jalan = 4028;
//		break;
//
//	case 4028: //acuan case 3294 2106
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp_asli[1] < thd_dinding) //4
//		{
//			lock=0;
//			jalan_langkah(state_telusur, 2);
//			penanda_buzzer = 0;
//			state_jalan = 4029;
//		}
//		break;
//
//
//	case 4029: //acuan case 2012 masuk ruang 3
//		telusurKiri_zigzag();
//		if(baca_garis() == 1){
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = 4030; //102; //jalan lagi dari awal
//			ruangScan = 3;
//			ruangArr[5] = 3;
//			//next_state_jalan = 2103
//			//langkahScan = 3;
//			ganti_muka = 0;
//			//konversi_scan = -1;
//			//konversi_loncat = -2;
//		}
//		break;
//
//
//
//	case 4030: //acuan case 550
//		//menyusuri ruang 3 mencari api
//		telusurKiri_zigzag_ruang();
//
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//
//		if(ganti_muka>=1)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 4031;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		else if(sensor_gp[sensor[0]]<100)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 4031;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//		{
//			konversi_state_telusur(1);
//			//jalan_langkah(state_telusur,2);
//			//state_jalan = 136; belum diset
//			state_jalan = 4080; //belum diset
//			ruang_api = ruangScan;
//		}
//		else
//		{
//			state_jalan = 4031;
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		break;
//
//	case 4080: //acuan case 136
//		//uv deteksi api
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		if(sensor_gp[sensor[0]]<100||sensor_gp_asli[0]<70)
//		{
//			lock=0;
//			state_jalan=4081;
//		}
//		break;
//
//	case 4081: //acuan case 137
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 4082;
//			api3=1;
//			ruang_api = ruangScan;
//		}
//		else
//		{
//			api3=0;
//			konversi_state_telusur(-2);
//			state_jalan = 4082;
//		}
//		break;
//
//	case 4082: //acuan case 138
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp[sensor[0]]<100||sensor_gp_asli[0]<70)
//		{
//			if(api3==1)
//			{
//				state_jalan=4083;
//			}
//			else
//			{
//				//state_jalan = 7025;
//				konversi_state_telusur(-1);
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//		}
//		break;
//	case 4083: //acuan case 201 ->9007
//		statusKompas = 1;
//		jalan_langkah(state_telusur,2);
//		ganti_kanan = 0;
//		ganti_kiri = 0;
//		state_jalan = 4084;
//		break;
//
//	case 4084://acuan case 9007
//		muter_acuankompas(acuan_3_4a);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=4085;
//		}
//		break;
//
//	case 4085: //acuan case 9001
//		muter_(kan,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			state_jalan=4088;
//		}
//		else if(pewaktu>85 && (ruang_api==3))
//		{
//			pewaktu=0;
//			state_jalan=4086;
//		}
//
//		else if(pewaktu>110)
//		{
//			state_jalan=999; // belum diset cek error
//		}
//		break;
//
//	case 4086: //acuan case 9009
//		muter_(kir,5,25);
//		if(pewaktu>45 && ruang_api==3)
//		{
//			state_telusur=0;
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			geser_kiri(state_telusur,1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=4087;
//		}
//		break;
//
//	case 4087: //acuan case 2220
//		gp_serong();
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 4087);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			//state_jalan = 2221; belum diset
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 4090; //7021
//		}
//		break;
//
//	case 4088: //acuan case 9003
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=4089;
//		}
//		break;
//
//	case 4089: //acuan case 9004
//		//jika api nyala lagi
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 4084;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//
//			/* belum diset
//			 if(ruang_api==3 && status_tpa==1)
//			{
//				mataangin = BARAT;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 498;
//				state_telusur = 0;
//				konversi_gp(0);
//			}*/
//		}
//
//		break;
//
//	case 4090: //acuan case 202
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 4091;
//			api_siap = 0;
//		}
//		break;
//
//	case 4091:
//		padamkan_api();
//		//state_jalan = 204; return belum diset
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		state_jalan=999;
//		break;
//
//
//
//	case 4031: //acuan case 2103 tidak ada api
//		telusurKanan_zigzag(); //gak ada api
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			while (baca_garis() == 1)telusurKanan_zigzag();
//			ganti_muka = 0;
//			state_jalan = 4032;//89; //tadi diganti disini
//			setKompas();
//			ruangArr[6]=3;
//		}
//		break;
//
//	case 4032: //acuan case 89
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 4033;
//		}
//		break;
//
//	case 4033: //acuan case 90
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 120)
//		{
//			lock=0;
//			tembak_cmps();//edit
//			konversi_state_telusur(-1);
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 4055;
//		}
//		break;
//
//	case 4055:
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[0] > thd_dinding) //ada pintu di 1a
//		{
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1=0;
//			state_jalan = 4056;
//		}
//		else //tidak ada pintu di 1a
//		{
//			penanda_pintu1=1;
//			konversi_state_telusur(-2); //siap masuk ruang 2
//			ganti_muka = 0;
//			state_jalan = 4058;
//
//		}
//		break;
//
//
//
//		//apabila konfig = 1A
//	case 4056: //acuan case 2091
//		telusurKanan_zigzag();
//		if (baca_garis() == 1) //sampai di ruang 1a
//		{
//			pewaktu=0;
//			setKompas();
//			hitung_zigzag = 0;
//			hitung_langkah = 0;
//			state_jalan = 4057;
//			ruangArr[1] = 1;
//			ruangScan = 1;
//
//		}
//		break;
//
//	case 4057: //acuan case 91
//
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 2;
//		}
//		jalan_langkah(state_telusur,2);
//		state_jalan=4005; //balik ke case sebelumnya
//		break;
//
//
//
//		//apabila konfig 1B, masuk ruang 2 dahulu
//	case 4058: //acuan case 92
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //sampai di ruang 2
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			hitung_zigzag = 0;
//			hitung_langkah =0;
//			state_jalan = 4017;
//			setKompas();
//			ruangScan=2;
//			ruangArr[1]=2;
//		}
//		break;
//
//	case 4095:
//		telusurKanan_zigzag();
//		if(ganti_kanan == 1){
//			konversi_state_telusur(-2);
//			state_jalan = 7096;
//		}
//		break;
//
//	case 4096:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp_asli[0] < 130) //4
//		{
//			konversi_state_telusur(1);
//			lock=0;
//			penanda_buzzer = 0;
//			state_jalan = 4029;
//		}
//		break;
//
//	case 4060:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 120)
//		{
//			lock=0;
//			tembak_cmps();//edit
//			konversi_state_telusur(-1);
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 4061;
//		}
//		break;
//
//	case 4061:
//		//ingin masuk ruang 1B
//		telusurKanan_zigzag();
//		if(baca_garis() == 1){
//			pewaktu=0;
//			setKompas();
//			lock_acuan=tembak_cmps();
//			lock_state_telusur = state_telusur;
//			state_jalan=4062;
//		}
//		break;
//
//	case 4062:
//		if(penanda_pintu1 == 1)
//		{
//			penanda_buzzer=0;
//			ruangArr[3]=5;
//			ruangScan=5;
//			state_jalan = 4063;
//			ganti_muka = 0;
//		}
//		//salah detek ternyata 1A
//		else{
//			state_jalan = 4057;
//		}
//		break;
//
//	case 4063: //acuan case 4057 91
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 2;
//		}
//		jalan_langkah(state_telusur,3);
//		state_jalan=4064;
//		break;
//
//	case 4064:
//		konversi_state_telusur(-1);
//		state_jalan=4065;
//		ganti_muka=0;
//		break;
//
//	case 4065: //acuan case 1110
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			hitung_langkah=0;
//			konversi_state_telusur(1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 4036;
//
//		}
//		break;
//
//
////=========================================
//
//		//===Konfig Anjing di posisi 2 atau 3
//
//	case 4066:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			hitung_langkah=0;
//			konversi_state_telusur(-1);
//			state_jalan=LURUSKAN_KANAN;
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			next_state_jalan = 4067;
//			ganti_kiri=0;
//			ganti_kanan=0;
//		}
//		break;
//
//	case 4067:
//		telusurKanan_zigzag();
//		if(detek_anjing() == 1){
//			//posisi anjing 2
//			if((tembak_cmps()==BARAT && state_telusur==1) || (tembak_cmps()==UTARA && state_telusur==2) || (tembak_cmps()==TIMUR && state_telusur==3) || (tembak_cmps()==SELATAN && state_telusur==0)){
//				posisi_anjing=2;
//				konversi_state_telusur(1);
//				ganti_kanan=0;
//				ganti_muka=0;
//				ganti_kiri=0;
//				state_jalan=4068;
//			}
//			//posisi anjing 3
//			if(ganti_kiri==1 && ((tembak_cmps()==BARAT && state_telusur==0) || (tembak_cmps()==UTARA && state_telusur==1) || (tembak_cmps()==TIMUR && state_telusur==2) || (tembak_cmps()==SELATAN && state_telusur==3))){
//				posisi_anjing=3;
//				konversi_state_telusur(1);
//				ganti_kanan=0;
//				ganti_muka=0;
//				ganti_kiri=0;
//				state_jalan=4068;
//			}
//		}
//
//		break;
//
//	case 4068: //konfig anjing 1 atau siap masuk 1C
//		telusurKiri_zigzag();
//		if(baca_garis() == 1){
//			if((tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) ||(tembak_cmps()==SELATAN && state_telusur==2)){
//				lock_state_telusur = state_telusur;
//				penanda_buzzer = 0;
//				pewaktu=0;
//				setKompas();
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				lock_acuan = tembak_cmps();
//				state_jalan = LURUSKAN_KIRI;
//				ruangArr[1] = 6; //1C
//				ruangScan = 6; //1C
//				next_state_jalan = 4069;
//				jalan_langkah_scan12(state_telusur,2);
//			}
//		}
//		break;
//
//	case 4069: //acuan case 3253 3206
//		//masuk ruang 1C
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (hitung_langkah == 3 || sensor_gp[sensor[0]]<100)
//		{
//			lock=0;
//			//deteksi api
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//			{
//				//dicek lagi apinya
//				state_jalan = 4070;
//				pewaktu=0;
//				konversi_state_telusur(1);
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			//tidak deteksi api
//			else
//			{
//				hitung_langkah=0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//				state_jalan = 4075;
//				konversi_state_telusur(-1); //supaya state telusur ke tembok (ke arah timur);
//			}
//		}
//		break;
//
//	case 4070: //acuan case 3254 3211
//		if(nilai_api<thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 4071;
//			ruang_api = ruangScan;
//		}
//		else if(eksistansi_api1 == 1 || eksistansi_api2 == 1){
//			maju(state_telusur);
//			state_jalan=4074;
//		}
//		else jalan_langkah_scan12(state_telusur,1);
//		break;
//
//	case 4071: //acuan case 3208 3255
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 4072;
//			api_siap = 0;
//		}
//		break;
//
//	case 4072: //acuan case 3256 3209
//		padamkan_api();
//		state_jalan = 4073; // cek apakah api sudah mati apa belum
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//	case 4073: //acuan case 3257 3210
//		if (deteksi_api(4000) == 1)
//		{
//			//api masih nyala
//			//state_jalan = 2201;
//
//			break;
//		}
//		else{
//			led_api_off;
//			//return belum diset
//			state_jalan=999; //sementara diset berhenti
//		}
//		break;
//
//	case 4074: //case 3258 3207
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		//obstacle
//		if(sensor_gp_asli[0]<200)
//		{
//			lock=0;
//			//state_jalan = 115; belum diset
//		}
//		else if (nilai_api > thd_api)
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 4071; //kembali ke case sebelumnya mencari keberadaan api
//		}
//		else maju(state_telusur);
//		break;
//
//	case 4075: //acuan case 3267
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api2 == 1||eksistansi_api1 == 1){
//			eksistansi_api1 = 0;
//			eksistansi_api2 = 0;
//			state_jalan=4070; //3254
//		}
//		if (baca_garis() == 1)
//		{
//			penanda_buzzer=0;
//			if(ruangArr[1] == 6 && ruangScan == 6){
//				pewaktu = 0;
//				setKompas();
//				lock_acuan = tembak_cmps();
//				lock_state_telusur = state_telusur;
//				state_jalan = 4076;
//				jalan_langkah(lock_state_telusur,2);
//			}
//		}
//		else telusurKiri_zigzag_ruang();
//		break;
//
//	case 4076: //acuan case 3268
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		//keluar ruang 1A
//		if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<thd_dinding){
//			state_jalan = LURUSKAN_KIRI;
//			ruangArr[2] = 1;
//			next_state_jalan = 4016;
//			penanda_pintu1=0;
//			pewaktu=0;
//			setKompas();
//			while (baca_garis() == 1)telusurKiri_zigzag();
//
//		}
//		//ruang 1b
//		else{
//			penanda_pintu1=1;
//			ruangArr[2] = 5; //1B
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			setKompas();
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 4048;
//			lock_acuan = tembak_cmps();
//		}
//		break;
//
//		//SELESAI RUANG START 4A
//
//
//
//
//
//
//
//
//
//
//		//===============Konfigurasi Ruang start 4B===============
//	case 7004: //acuan case 3208
//
//		telusurKiri_zigzag();
//		if(ganti_kanan == 1){
//			ganti_muka = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			state_jalan=7005;
//		}
//		break;
//
//	case 7005: //acuan case 3209
//
//		telusurKiri_zigzag();
//
//		if(sensor_gp[sensor[6]]<thd_dinding && ((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2))){
//			//posisi anjing 2
//			if(detek_anjing() == 1){
//				konversi_state_telusur(-1);
//				state_jalan=7006;
//				posisi_anjing=2;
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//			}
//			//posisi anjing 1 atau 3
//			else if(detek_anjing() !=1 && sensor_gp[sensor[3]]<thd_dinding && sensor_gp_asli[1]<300){
//				state_jalan=7006;
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//				konversi_state_telusur(-1);
//				posisi_anjing=1; //sementara posisi anjing 1 padahal bisa 1 atau 3
//			}
//		}
//		break;
//
//	case 7006:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<120){
//			lock=0;
//			konversi_state_telusur(-1);
//			state_jalan=LURUSKAN_KANAN;
//			next_state_jalan=7007;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_muka=0;
//		}
//		break;
//
//	case 7007:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		telusurKanan_zigzag(); //acuan case 11
//		//cek jaga-jaga jika posisi anjing ternyata di posisi 3
//		if(ganti_kiri == 1 && detek_anjing() == 1){
//			state_jalan = 7008; //belum diset
//			posisi_anjing=3;
//			konversi_state_telusur(-2);
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_muka=0;
//		}
//
//		if(((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2))){
//
//			if(ganti_kiri == 2 && sensor_gp[sensor[5]]<thd_dinding && sensor_gp_asli[3]<330){
//				hitung_zigzag=0;
//				hitung_langkah=0;
//				next_state_jalan=7035;
//				state_jalan=LURUSKAN_KANAN;
//			}
//		}
//
//		if(ganti_kanan == 1){
//			state_jalan = 100000; //check error
//		}
//		break;
//
//	case 7008:
//		telusurKiri_zigzag();
//		if(ganti_kanan == 1){
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan = 7072;
//			ganti_kiri=0;
//			ganti_kanan=0;
//			hitung_langkah=0;
//		}
//		break;
//
//	case 7035:
//		penanda_buzzer=0;
//		konversi_state_telusur(-1);
//		state_jalan=7036;
//		break;
//
//	case 7036:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<120){
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan=7037;
//			setKompas();
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_muka=0;
//		}
//		break;
//
//	case 7037:
//		telusurKiriPipih();
//		if(((tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) || (tembak_cmps()==TIMUR && state_telusur==0) || (tembak_cmps()==SELATAN && state_telusur==1))){
//			if(sensor_gp[sensor[5]]<thd_dinding && sensor_gp_asli[1]<300){
//				hitung_zigzag=0;
//				hitung_langkah=0;
//				next_state_jalan=7038;
//				state_jalan=LURUSKAN_KIRI;
//			}
//		}
//		break;
//	case 7038:
//		//cek posisi anjing untuk menentukan robot jalan ke 1C atau 1A/1B
//		if(posisi_anjing == 1){
//			konversi_state_telusur(1);
//			state_jalan=7039;
//		}
//		else if(posisi_anjing == 2){
//			state_jalan=7070;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_muka=0;
//		}
//		break;
//
//	case 7039:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<120){
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan=7040;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_kanan=0;
//		}
//		break;
//
//
//	case 7040: //acuan case 3241
//		//masih belum bisa menentukan ruang 1A atau 1B setelah detek garis
//		telusurKiriPipih();
//		if(baca_garis() == 1){
//			setKompas();
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 7041;
//			penanda_buzzer=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			lock_acuan=tembak_cmps();
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_muka=0;
//		}
//		break;
//
//	case 7041: //acuan case 3242
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 2;
//		}
//		jalan_langkah_scan12(state_telusur,3);
//		state_jalan=LURUSKAN_CMPS;
//		next_state_jalan=7042;
//		break;
//
//	case 7042: //acuan case 4005 3243 12
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//persyaratan ruang 1B
//		if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<300){
//			ruangScan = 5; //1B = 5
//			ruangArr[1] = 5;
//			konversi_state_telusur(-1);
//			ganti_muka=0;
//			ganti_kiri=0;
//			hitung_langkah=0;
//			state_jalan=7043;
//		}
//		//persyaratan ruang 1A
//		else if(sensor_gp[sensor[2]]<thd_dinding){
//			ruangScan = 1; //1A = 1
//			ruangArr[1] = 1;
//			konversi_state_telusur(1);
//			ganti_muka=0;
//			ganti_kiri=0;
//			hitung_langkah=0;
//			state_jalan=7060;
//		}
//		break;
//
//	case 7043: //acuan case 3245 1110
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1 = 1;
//			state_jalan = 7044;
//			hitung_langkah = 0;
//		}
//		break;
//
//	//case 7245-7249 proses deteksi api hingga madamin api
//	case 7044: //acuan case 3246 201 (case global)
//
//		telusurKiri_zigzag_ruang();
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			state_jalan = 7045; //3247
//			ruang_api = ruangScan;
//		}
//		//tidak deteksi api
//		else
//		{
//			state_jalan = 7050;
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		break;
//
//	case 7045: //acuan case 3247 9006
//		muter_acuankompas(acuan_1b);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=7046; //3248
//		}
//		break;
//
//	case 7046: //case 3248 9001
//		muter_(kan,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			state_jalan=7048;
//		}
//		else if(pewaktu>110)
//		{
//			if(ruang_api==5)
//			{
//				pewaktu=0;
//				state_jalan=7047;
//			}
//
//		}
//		break;
//
//	case 7047: //acuan case 3249 9009
//		muter_(kir,5,25);
//		if(pewaktu>50 && ruang_api==5)
//		{
//			state_telusur=0;
//			lock_state_telusur=state_telusur;
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			geser_kiri(state_telusur,1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=999; //berhenti deteksi error 2220
//		}
//		break;
//
//	case 7048://acuan case 9003
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=7049; //3251
//		}
//		break;
//
//	case 7049: //acuan case 3251
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 7045;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//			if(ruang_api==5 && status_tpa==1)
//			{
//				mataangin = SELATAN;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 999; //berhenti dulu
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//		}
//		break;
//
//	case 7050://acuan case 4045 3260 2107
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			lock=0;
//			konversi_state_telusur(-1);
//			state_jalan = 7051;
//		}
//		break;
//
//	case 7051: //acuan case 4046 3261 98
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KANAN;
//		next_state_jalan = 7052;
//		break;
//
//	case 7052: //acuan case 3262 99 keluar 1B
//		telusurKanan_zigzag_ruang();
//		if (baca_garis() == 1)
//		{
//			//putaran 2
//			if(ruangArr[3] == 5){
//				ruangArr[4] = 5;
//			}
//			//putaran 1
//			else if (ruangArr[1] == 5){
//				ruangArr[2] = 5; //1B
//			}
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//			setKompas();
//			state_jalan=7053;
//			lock_acuan = tembak_cmps();
//		}
//		break;
//
//	case 7053: //acuan case 4048 3263
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 2;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 0;
//		}
//		state_jalan=LURUSKAN_CMPS;
//		next_state_jalan=7054;
//		break;
//
//	case 7054: //acuan case 4049 3264
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<130)
//		{
//			lock=0;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			//putaran pertama
//			if(ruangArr[2] == 5){
//				next_state_jalan = 7055;
//				konversi_state_telusur(-1);
//				state_jalan=LURUSKAN_KANAN;
//			}
//			//putaran kedua
//			else if(ruangArr[3] == 5 && ruangArr[4] == 5){
//				konversi_state_telusur(1);
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan = 7094; //siap masuk ruang 3 kembali utk putaran ketiga
//			}
//		}
//		break;
//
//	case 7055: //acuan case 3265 4
//		telusurKanan_zigzag();
//		if (ganti_kanan == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 7056;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//
//	case 7056: //acuan case 3266 90
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 130)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 7081; //siap ingin masuk ruang 2
//		}
//		break;
//
//
//		//konfigurasi ruang 1A
//	case 7060: //case 4060 3271 1011
//		baca_gp(state_telusur);
//		jalan_langkah_scan12(state_telusur,1);
//		if (eksistansi_api1 == 1)
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			konversi_state_telusur(1);
//			state_jalan = 7062;
//			ruang_api = ruangScan;
//		}
//		else if (eksistansi_api2 == 1)
//		{
//			lock=0;
//			eksistansi_api1 = 0;
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 2);
//			jalan_langkah_scan1(state_telusur, 2);
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(-2);
//				jalan_langkah(state_telusur, 3);
//				state_jalan = 7062;
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				konversi_state_telusur(1); //-2x`
//				state_jalan = 7066;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//
//			}
//		}
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1 = 1;
//			state_jalan = 7061;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			penanda_pintu1=0;
//		}
//		break;
//
//	case 7061: //acuan case 3272 1012
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if (eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(1);
//			state_jalan = 7062;
//			ruang_api = ruangScan;
//		}
//		else if (eksistansi_api2 == 1)
//		{
//			eksistansi_api1 = 0;
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 2);
//			jalan_langkah_scan1(state_telusur, 2);
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(-2);
//				jalan_langkah(state_telusur, 3);
//				state_jalan = 7062;
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				konversi_state_telusur(1); //-2x`
//				state_jalan = 7066;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//
//			}
//		}
//		else if(hitung_langkah<=5&&sensor_gp[sensor[0]]<120)
//		{
//			konversi_state_telusur(-1);
//			hitung_langkah = 0;
//		}
//		else
//		{
//			konversi_state_telusur(1);
//			state_jalan = 7066;
//			ganti_muka = 0;
//			hitung_zigzag = 0;
//		}
//
//
//		if (nilai_api > thd_api&&hitung_langkah<3)
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			//state_jalan = 202;
//			ruang_api = ruangScan;
//		}
//		break;
//
//	case 7062: //acuan case 3273 9005
//		muter_acuankompas(acuan_1a);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=7063;
//		}
//		break;
//
//	case 7063: //acuan case 3274 9002
//		muter_(kir,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			jalan_langkah(state_telusur,1);
//			state_jalan=7064; //3275
//		}
//		else if(pewaktu>110)
//		{
//			if(ruang_api==1)//tpa tdk dapat
//			{
//				pewaktu=0;
//				//state_jalan=9010;
//			}
//		}
//		break;
//
//	case 7064: //acuan case 3275 9003
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=7065; //3276
//		}
//		break;
//
//	case 7065: //acuan case 3276 9004
//		//jika api nyala lagi
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 7061;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//			if(ruang_api == 1 && status_tpa == 1)//acuan sesudah tpa
//			{
//				mataangin = UTARA;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 999; //return belum diset
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//		}
//		break;
//
//		//tak deteksi api di ruang 1A
//	case 7066: //acuan case 4012 3280 96
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan = 7067;
//		}
//		break;
//
//	case 7067: //acuan cae 3281 97
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KIRI;
//		next_state_jalan = 7068; //3282
//		break;
//
//	case 7068: //acuan cae 4015 3282 2092
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //garis di 1a
//		{
//			pewaktu=0;
//			setKompas();
//			while (baca_garis() == 1)telusurKiri_zigzag();
//			next_state_jalan = 7081;
//			state_jalan = LURUSKAN_KIRI;
//			ruangArr[2] = 1; //keluar ruang 1A
//			penanda_pintu1=0;
//		}
//		break;
//
//
//		//konfig anjing di posisi 2
//	case 7070:
//		telusurKiri_zigzag();
//		if(ganti_kiri == 1){
//			konversi_state_telusur(1);
//			state_jalan=7071;
//		}
//		break;
//
//	case 7071: //acuan case 4066
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			hitung_langkah=0;
//			konversi_state_telusur(1);
//			state_jalan=LURUSKAN_KIRI;
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			next_state_jalan = 7072;
//			ganti_kiri=0;
//			ganti_kanan=0;
//		}
//		break;
////siap masuk 1C konfig anjing di posisi 2 dan 3
//	case 7072: //acuan case 4068
//		telusurKiri_zigzag();
//		if(baca_garis() == 1){
//			if((tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) ||(tembak_cmps()==SELATAN && state_telusur==2)){
//				lock_state_telusur = state_telusur;
//				penanda_buzzer = 0;
//				pewaktu=0;
//				setKompas();
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				lock_acuan = tembak_cmps();
//				state_jalan = LURUSKAN_KIRI;
//				ruangArr[1] = 6; //1C
//				ruangScan = 6; //1C
//				next_state_jalan = 7073;
//				jalan_langkah_scan12(state_telusur,2);
//			}
//		}
//		break;
//
//	case 7073: //acuan case 4069 3253 3206
//		//masuk ruang 1C
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (hitung_langkah == 3 || sensor_gp[sensor[0]]<100)
//		{
//			lock=0;
//			//deteksi api
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//			{
//				//dicek lagi apinya
//				state_jalan = 7074;
//				pewaktu=0;
//				konversi_state_telusur(1);
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			//tidak deteksi api
//			else
//			{
//				hitung_langkah=0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//				state_jalan = 7079;
//				konversi_state_telusur(-1); //supaya state telusur ke tembok (ke arah timur);
//			}
//		}
//
//		break;
//
//	case 7074: //acuan case 4070 3254 3211
//		if(nilai_api<thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 7075;
//			ruang_api = ruangScan;
//		}
//		else if(eksistansi_api1 == 1 || eksistansi_api2 == 1){
//			maju(state_telusur);
//			state_jalan=7078;
//		}
//		else jalan_langkah_scan12(state_telusur,1);
//		break;
//
//
//	case 7075: //acuan case 4071 3208 3255
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 7076;
//			api_siap = 0;
//		}
//		break;
//
//	case 7076: //acuan case 4072 3256 3209
//		padamkan_api();
//		state_jalan = 7077; // cek apakah api sudah mati apa belum
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//	case 7077: //acuan case 4073 3257 3210
//		if (deteksi_api(4000) == 1)
//		{
//			//api masih nyala
//			//state_jalan = 2201;
//
//			break;
//		}
//		else{
//			led_api_off;
//			//return belum diset
//			state_jalan=999; //sementara diset berhenti
//		}
//		break;
//
//	case 7078: //acuan case 4074 3258 3207
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		//obstacle
//		if(sensor_gp_asli[0]<200)
//		{
//			lock=0;
//			//state_jalan = 115; belum diset
//		}
//		else if (nilai_api > thd_api)
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 7075; //kembali ke case sebelumnya mencari keberadaan api
//		}
//		else maju(state_telusur);
//		break;
//
//		//tidak detek api
//	case 7079: //acuan case 4075 3267
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api2 == 1||eksistansi_api1 == 1){
//			eksistansi_api1 = 0;
//			eksistansi_api2 = 0;
//			state_jalan=7074;//4070; //3254
//		}
//		if (baca_garis() == 1)
//		{
//			penanda_buzzer=0;
//			if(ruangArr[1] == 6 && ruangScan == 6){
//				pewaktu = 0;
//				setKompas();
//				lock_acuan = tembak_cmps();
//				lock_state_telusur = state_telusur;
//				state_jalan = 7080;
//				jalan_langkah(lock_state_telusur,2);
//			}
//		}
//		else telusurKiri_zigzag_ruang();
//		break;
//
//	case 7080: //acuan case 4076 3268
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		//keluar ruang 1A
//		if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<thd_dinding){
//			state_jalan = LURUSKAN_KIRI;
//			ruangArr[2] = 1;
//			next_state_jalan = 7081;
//			penanda_pintu1=0;
//			pewaktu=0;
//			setKompas();
//			while (baca_garis() == 1)telusurKiri_zigzag();
//
//		}
//		//ruang 1b
//		else{
//			penanda_pintu1=1;
//			ruangArr[2] = 5; //1B
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			setKompas();
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 7053;
//			lock_acuan = tembak_cmps();
//		}
//		break;
//
//
//
//		//sudah keluar ruang 1A atau 1B dan ingin masuk ruang 2
//	case 7081: //acuan case 4016 3283 2093
//		//perputaran pertama
//
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //garis di 2 masuk ruang 2
//		{
//			if(ruangArr[2] ==5){
//				//1B
//				penanda_pintu1=1;
//			}
//			else{
//				//1A
//				penanda_pintu1=0;
//			}
//			pewaktu=0;
//			penanda_buzzer=0;
//			setKompas();
//			hitung_zigzag = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = 7082;
//			ruangArr[3] = 2;
//			ruangScan=2;
//		}
//
//		break;
//
//	case 7082: //acuan case 4017 3284 93
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]]<100) //scan di ruang 2 memang harus maju dikit
//		{
//			lock=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 7083;  //padamkan api di ruang II belum 663
//				eksistansi_api2 = 0;
//				hitung_langkah = 0;
//			}
//			else //tidak deteksi api
//			{
//				konversi_state_telusur(-1);
//				state_jalan = 7090; //94
//				ganti_muka = 0;
//			}
//		}
//		break;
//
//	case 7083: //acuan case 4018 3285 663
//		konversi_gp(state_telusur);
//		jalan_langkah(state_telusur,2);
//		jalan_langkah_scan12(state_telusur,2);
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 7084; //538
//			ruang_api = ruangScan;
//		}
//		else if(sensor_gp[sensor[0]]<120)
//		{
//			geser_kanan(state_telusur,2);
//			//state_jalan = 517; belum diset
//		}
//		else
//		{
//			baca_gp(state_telusur);
//			if(sensor_gp_asli[1]<sensor_gp_asli[3])
//			{
//				konversi_state_telusur(1);
//				lock_state_telusur = state_telusur;
//				//state_jalan = 540; belum diset
//				keluar_kiri = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			else
//			{
//				konversi_state_telusur(-1);
//				lock_state_telusur = state_telusur - 2;
//				if(lock_state_telusur<0)lock_state_telusur+=4;
//				//state_jalan = 540; belum diset
//				keluar_kanan = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//		}
//		break;
//
//	case 7084: //acuan case 4019 3286 538
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			state_jalan = 7085;
//		}
//
//		break;
//
//	case 7085: //acuan case 4020 3287 2220
//		gp_serong();
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 7085);
//		if (baca_garis() == 1) //antisipasi
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			//state_jalan = ; //2221 belum diset
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 7086;
//		}
//		break;
//
//	case 7086: //acuan case 4021 3288 202
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 4022; //203
//			api_siap = 0;
//		}
//		break;
//
//	case 7087: //acuan case 4022 3289 203
//		padamkan_api();
//		state_jalan = 999; //belum diset
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//
//		//mau keluar ruang 2 tidak deteksi api
//	case 7090: //acuan case 4025 3290 94
//		telusurKanan_zigzag_ruang();
//		if (baca_garis() == 1)
//		{
//			pewaktu=0;
//			setKompas();
//			tembak_cmps();
//			lock_state_telusur = state_telusur;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			penanda_buzzer=0;
//
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			//putaran kedua
//			if(ruangArr[1] == 2){
//				ruangArr[2] = 2;
//			}
//			//putaran pertama
//			else if (ruangArr[3] == 2){
//				ruangArr[4] = 2;
//			}
//			state_jalan=7091;
//
//		}
//
//		break;
//
//	case 7091: //acuan case 4026 3291 2007
//		//maju langkah keluar ruang 2
//		telusurKanan_zigzag();
//		if (hitung_zigzag == 2)
//		{
//
//			ganti_muka = 0;
//
//			//putaran kedua 1B
//			if(ruangArr[1] == 2 && ruangArr[2] == 2){
//				state_jalan=7032;
//				//konversi_state_telusur(1);
//			}
//
//			//putaran pertama 1B
//			if(ruangArr[2] ==5){
//				ganti_muka=0;
//				ganti_kiri=0;
//				ganti_kanan=0;
//				penanda_khusus_1a = 0; //1B
//				penanda_pintu1 = 1;
//				state_jalan=7095;
//			}
//
//			//putaran pertama 1A
//			else if(ruangArr[2] == 1){
//				state_jalan = LURUSKAN_KANAN;
//				penanda_pintu1 = 0; //1A
//				penanda_khusus_1a = 1;
//				next_state_jalan=7092;
//			}
//
//		}
//
//		break;
//
//	case 7092: //acuan case 4027 3292
//		konversi_state_telusur(-1);
//		state_jalan = 7093;
//		break;
//
//	case 7093: //acuan case 4028 3294 2106
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp_asli[1] < thd_dinding) //4
//		{
//			lock=0;
//			jalan_langkah(state_telusur, 2);
//			penanda_buzzer = 0;
//			state_jalan = 7094;
//		}
//		break;
//
//	case 7094: //acuan case 4029 3294 2012
//		telusurKiri_zigzag();
//		if(baca_garis() == 1){
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = LURUSKAN_KIRI; //102; //jalan lagi dari awal
//			ruangScan = 3;
//			ruangArr[5] = 3;
//			next_state_jalan = 7010;
//			//langkahScan = 3;
//			ganti_muka = 0;
//			//konversi_scan = -1;
//			//konversi_loncat = -2;
//		}
//		break;
//
//	case 7095:
//		telusurKanan_zigzag();
//		if(ganti_kanan == 1){
//			konversi_state_telusur(-2);
//			state_jalan = 7096;
//		}
//		break;
//
//	case 7096:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp_asli[0] < 130) //4
//		{
//			konversi_state_telusur(1);
//			lock=0;
//			penanda_buzzer = 0;
//			state_jalan = 7094;
//		}
//		break;
//
//
//	case 7010: //acuan case 550 case global dengan next state jalan 2103
//		//scanning ruang 3
//		telusurKiri_zigzag_ruang();
//
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//
//		if(ganti_muka>=1)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 7025;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		else if(sensor_gp[sensor[0]]<100)//edit 25-04-2018
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 7025;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//
//
//		if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//		{
//			konversi_state_telusur(1);
//			//jalan_langkah(state_telusur,2);
//			state_jalan = 7011;
//			ruang_api = ruangScan;
//		}
//		else
//		{
//			state_jalan = 7025;
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		break;
//
//	case 7011: //acuan case 136
//		//uv deteksi api
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		if(sensor_gp[sensor[0]]<100||sensor_gp_asli[0]<70)
//		{
//			lock=0;
//			state_jalan=7012;
//		}
//		break;
//
//	case 7012: //acuan case 137
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 7013;
//			api3=1;
//			ruang_api = ruangScan;
//		}
//		else
//		{
//			api3=0;
//			konversi_state_telusur(-2);
//			state_jalan = 7013;
//		}
//		break;
//
//	case 7013: //acuan case 138
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp[sensor[0]]<100||sensor_gp_asli[0]<70)
//		{
//			lock = 0;
//			if(api3==1)
//			{
//				state_jalan=7014;
//			}
//			else
//			{
//				state_jalan = 7025;
//				konversi_state_telusur(-1);
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//		}
//		break;
//
//	case 7014: //case global 201 -> 9007
//		statusKompas = 1;
//		jalan_langkah(state_telusur,2);
//		ganti_kanan = 0;
//		ganti_kiri = 0;
//		state_jalan = 7015;
//		break;
//
//	case 7015: //acuan case 9007
//		muter_acuankompas(acuan_3_4a);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=7016;
//		}
//		break;
//
//	case 7016: //acuan case 9001
//		muter_(kan,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			state_jalan=7019;
//		}
//		else if(pewaktu>85 && (ruang_api==3))
//		{
//			pewaktu=0;
//			state_jalan=7017;
//		}
//
//		else if(pewaktu>110)
//		{
//			state_jalan=999; // belum diset cek error
//		}
//		break;
//
//	case 7017: //acuan case 9009
//		muter_(kir,5,25);
//		if(pewaktu>45 && ruang_api==3)
//		{
//			state_telusur=0;
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			geser_kiri(state_telusur,1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=7018;
//		}
//		break;
//
//	case 7018: //acuan case 2220
//		gp_serong();
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 7018);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			//state_jalan = 2221; belum diset
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 7021;
//		}
//		break;
//
//	case 7019: //acuan case 9003
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=7020;
//		}
//		break;
//
//	case 7020: //acuan case 9004
//		//jika api nyala lagi
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 7015;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//
//			/* belum diset
//			 if(ruang_api==3 && status_tpa==1)
//			{
//				mataangin = BARAT;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 498;
//				state_telusur = 0;
//				konversi_gp(0);
//			}*/
//		}
//
//		break;
//
//	case 7021: //acuan case 202
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 7022;
//			api_siap = 0;
//		}
//		break;
//
//	case 7022: //acuan case 203
//		padamkan_api();
//		//state_jalan = 204; return belum diset
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		state_jalan=999;//belum diset
//		break;
//
//	case 7025: //acuan case 2103 tidak ada api
//		//ingin keluar ruang 3
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			while (baca_garis() == 1)telusurKanan_zigzag();
//			ganti_muka = 0;
//			state_jalan = 7026; //tadi diganti disini
//			setKompas();
//			ruangArr[6]=3;
//		}
//		break;
//
//
//		//==========PUTARAN KEDUA=============
//	case 7026: //acuan case 4033 89
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 7027;
//		}
//		break;
//
//	case 7027: //acuan case 4033 90
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 120)
//		{
//			lock=0;
//			tembak_cmps();//edit
//			konversi_state_telusur(-1);
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 7028;
//		}
//		break;
//
//	case 7028: //acuan case 4055
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[0] > thd_dinding) //ada pintu di 1a
//		{
//			//masuk ruang 1a dahulu baru ruang 2
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1=0;
//			state_jalan = 7029;
//		}
//		else //tidak ada pintu di 1a berarti konfig 1b
//		{
//			//masuk ruang 2 dahulu baru ruang 1b
//			penanda_pintu1=1;
//			konversi_state_telusur(-2); //siap masuk ruang 2
//			ganti_muka = 0;
//			state_jalan = 7031;
//
//		}
//		break;
//
//	case 7029: //acuan case 4056 2091
//		telusurKanan_zigzag();
//		if (baca_garis() == 1) //sampai di ruang 1a
//		{
//			pewaktu=0;
//			setKompas();
//			hitung_zigzag = 0;
//			hitung_langkah = 0;
//			state_jalan = 7030;
//			ruangArr[1] = 1;
//			ruangScan = 1;
//		}
//		break;
//
//	case 7030: //acuan case 4057 91
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 2;
//		}
//		jalan_langkah(state_telusur,2);
//		state_jalan=7042; //balik ke case sebelumnya
//		break;
//
//		//apabila konfig 1B, masuk ruang 2 dahulu
//	case 7031:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //sampai di ruang 2
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			hitung_zigzag = 0;
//			hitung_langkah =0;
//			state_jalan = 7082;
//			setKompas();
//			ruangScan=2;
//			ruangArr[1]=2;
//		}
//		break;
//
//	case 7032: //acuan case 4060
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 120)
//		{
//			lock=0;
//			tembak_cmps();//edit
//			konversi_state_telusur(-1);
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 7033;
//		}
//		break;
//
//	case 7033: //acuan case 4061
//		//ingin masuk ruang 1B
//		telusurKanan_zigzag();
//		if(baca_garis() == 1){
//			pewaktu=0;
//			setKompas();
//			lock_acuan=tembak_cmps();
//			lock_state_telusur = state_telusur;
//			state_jalan=7034;
//		}
//		break;
//
//	case 7034: //acuan case 4062
//		if(penanda_pintu1 == 1)
//		{
//			penanda_buzzer=0;
//			ruangArr[3]=5;
//			ruangScan=5;
//			state_jalan = 7099;
//			ganti_muka = 0;
//		}
//		//salah detek ternyata 1A
//		else{
//			state_jalan = 7030;
//		}
//		break;
//
//	case 7099: //acuan case 4063 4057 91
//		if(lock_acuan == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(lock_acuan == UTARA)
//		{
//			state_telusur = 3;
//		}
//		else if(lock_acuan == SELATAN)
//		{
//			state_telusur = 1;
//		}
//		else if(lock_acuan == BARAT)
//		{
//			state_telusur = 2;
//		}
//		jalan_langkah(state_telusur,3);
//		state_jalan=7100;
//		break;
//
//	case 7100: //acuan case 6064
//		konversi_state_telusur(-1);
//		state_jalan=7101;
//		ganti_muka=0;
//		break;
//
//	case 7101://acuan case 4065
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			hitung_langkah=0;
//			konversi_state_telusur(1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 7044; //mepet tembok
//
//		}
//		break;
//
//
//
//
//
//
//
//
//		//========= KONFIG RUANG START 2 ==================
//
//
// //sering error menentukan konfig 1a atau 1b sehingga dibuat case ini
//	case 2100:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp[sensor[1]] < thd_dinding && sensor_gp[sensor[3]] < thd_dinding){
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 2070;
//		}
//		//back-up takut salah detek
//		else if(sensor_gp_asli[1]<thd_dinding){
//			state_jalan = 2071;
//		}
//		break;
//
//	case 2071:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//
//		if(sensor_gp_asli[1]<100){
//			penanda_buzzer=0;
//			next_state_jalan = 2070;
//			state_jalan = LURUSKAN_KANAN;
//		}
//		else{
//			geser_kanan(state_telusur,1);
//		}
//		break;
//
//	case 2070: //acuan case 4055 7028
//		//masih ada kemungkinan ruang start 3
//		//sehingga perlu backup antara
//		//start ruang 3 dan 2
//
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		if (sensor_gp_asli[0] > thd_dinding) //ada pintu di 1a
//		{
//
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1=0;
//			next_state_jalan = 2050;
//			hitung_zigzag=0;
//			state_jalan = LURUSKAN_KANAN;
//
//		}
//		else //tidak ada pintu di 1a berarti konfig 1b
//		{
//
//			penanda_pintu1=1; //1B
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 2055;
//
//		}
//		break;
//		//====== konfig ruang 1B=========
//
//	case 2055: //acuan case 7095
//		telusurKanan_zigzag();
//		if(ganti_kiri == 1 && ganti_kanan == 1){
//			konversi_state_telusur(-2);
//			state_jalan = 2056;
//		}
//		break;
//
//	case 2056: //acuan case 7096
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp_asli[0] < 130) //4
//		{
//			konversi_state_telusur(1);
//			lock=0;
//			penanda_buzzer = 0;
//			state_jalan = 2112;//7094;
//		}
//		break;
//
//		//=======konfig ruang 1A=========
//	case 2050:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//
//		if(sensor_gp_asli[1]<=110){
//			penanda_buzzer=0;
//			next_state_jalan = 2101;
//			state_jalan = LURUSKAN_KANAN;
//		}
//		else{
//			geser_kanan(state_telusur,1);
//		}
//		break;
//
//	case 2101:
//		konversi_state_telusur(-1);
//		state_jalan = 2109;
//		pewaktu = 0;
//		break;
//
//
//	case 2109: //acuan case 3294 2106
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp_asli[1] < thd_dinding) //1
//		{
//			lock=0;
//			jalan_langkah(state_telusur, 2);
//			penanda_buzzer = 0;
//			state_jalan = 2272;
//			ganti_kiri=0;
//		}
//		break;
//
//	case 2272:
//		konversi_state_telusur(-1);
//		//state_jalan = LURUSKAN_CMPS;
//		state_jalan = 2273;
//		break;
//
//	case 2273:
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		state_jalan = 2274;
//		break;
//
//	case 2274:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 100) //1
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			konversi_state_telusur(1);
//			state_jalan = 2112;
//			ganti_kiri=0;
//		}
//		break;
//
//		//===================================
//
//	case 2112://acuan case 3295 2108
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiriPipih();
//		if((tembak_cmps()==TIMUR && state_telusur==2) ||(tembak_cmps()==BARAT && state_telusur==0) || (tembak_cmps()==UTARA && state_telusur==1) ||(tembak_cmps()==SELATAN && state_telusur==3)){
//			if(sensor_gp_asli[3]<300 && sensor_gp_asli[1]<300) {
//				ganti_kanan=0;
//				ganti_kiri=0;
//				ganti_muka=0;
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				penanda_buzzer=0;
//				jalan_langkah(state_telusur,2);
//				state_jalan=2113;
//			}
//		}
//		break;
//
//	case 2113: //acuan case 3296
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		state_jalan = 2275;
//
//		break;
//
//	case 2275:
//		state_jalan = LURUSKAN_CMPS;
//		next_state_jalan = 2276;
//		hitung_zigzag = 0;
//		hitung_langkah = 0;
//		konversi_state_telusur(1);
//		break;
//
//	case 2276:
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		state_jalan = 2270;
//		break;
//
//	case 2270:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp_asli[0]<300){
//			lock=0;
//			next_state_jalan=LURUSKAN_CMPS;
//			state_jalan = 2271;
//		}
//		else{
//			if(sensor_gp_asli[3]<thd_dinding){
//				geser_kanan(state_telusur,1);
//				lock=0;
//			}
//			else{
//				geser_kiri(state_telusur,1);
//				lock=0;
//			}
//		}
//		break;
//
//	case 2271:
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		state_jalan=2114;
//		break;
//
//	case 2114://acuan case 3301
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		//takut masuk ruang 3 lagi
//		if (baca_garis() == 1) {
//			lock=0;
//			lock_state_telusur = state_telusur;//lock state
//			penanda_buzzer = 0;
//			pewaktu=0;
//			setKompas();
//			lock_acuan=tembak_cmps();//mulai lock acuan kompas
//			hitung_langkah = 0;
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan=999; //deteksi error belum diset (berhenti)
//		}
//
//		//persimpangan siku ruang 3
//		if(sensor_gp_asli[0]<140){
//			state_jalan=LURUSKAN_CMPS;
//			konversi_state_telusur(1);
//			next_state_jalan = 2115;
//			lock=0;
//		}
//		break;
//
//	case 2115: //acuan case 3298
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		state_jalan=2116;
//		break;
//
//	case 2116: //acuan case 3302
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiriPipih();
//		//back-up takut masuk ruang 1,2,1C
//		if(baca_garis() == 1){
//			penanda_buzzer=0;
//			setKompas();
//			//masuk ruang 1A 3310
//			if(((tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) || (tembak_cmps()==TIMUR && state_telusur==0) || (tembak_cmps()==SELATAN && state_telusur==1))){
//				state_jalan = 999; //belum di-set
//				//next_state_jalan=3310;
//				//konversi_state_telusur(-1);
//			}
//			//masuk ruang 2 belum di-set
//			else if(((tembak_cmps()==BARAT && state_telusur==0) || (tembak_cmps()==UTARA && state_telusur==1) || (tembak_cmps()==TIMUR && state_telusur==2) || (tembak_cmps()==SELATAN && state_telusur==3))){
//				state_jalan = 999; //belum di-set
//			}//masuk ruang 1C anjing di posisi 1 belum di-set
//			else if ((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2)){
//				state_jalan = 999; //belum di-set
//			}
//		}
//		//back-up takut detek anjing belum di-set
//		if(detek_anjing() == 1){
//			setKompas();
//			//anjing di posisi 3
//			if((tembak_cmps()==TIMUR && state_telusur==0) ||(tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) ||(tembak_cmps()==SELATAN && state_telusur==1)){
//				state_jalan = 999; //belum di-set
//			}
//			//anjing di posisi 2 belum di-set
//			else if ((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2)){
//				state_jalan = 999; //belum di-set
//			}
//		}
//
//		//verifikasi lorong antara ruang 3 dan 4 menghadap ke selatan
//		if(((tembak_cmps()==BARAT && state_telusur==1) || (tembak_cmps()==UTARA && state_telusur==2) || (tembak_cmps()==TIMUR && state_telusur==3) || (tembak_cmps()==SELATAN && state_telusur==0)) && sensor_gp[sensor[6]]<thd_dinding && sensor_gp[sensor[5]]<thd_dinding && sensor_gp[sensor[2]]<thd_dinding){
//			state_jalan=2117;
//			ganti_muka=0;
//			ganti_kiri=0;
//			ganti_kanan=0;
//		}
//		break;
//
//	case 2117: //acuan case 3204
//		//kemungkinan anjing 3 atau 2
//		//udah ganti kanan
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//
//		if((tembak_cmps()==TIMUR && state_telusur==0 ||tembak_cmps()==BARAT && state_telusur==2 || tembak_cmps()==UTARA && state_telusur==3 ||tembak_cmps()==SELATAN && state_telusur==1)){
//			//anjing di posisi 3 cek diawal biar langsung memutar ruang 4
//			if(detek_anjing() == 1){
//				state_jalan=2119;//3206 //posisi anjing = 3
//				posisi_anjing=3;
//				konversi_state_telusur(-1);
//				ganti_muka=0;
//				ganti_kiri=0;
//				ganti_kanan=0;
//			}
//			else{ //posisi anjing tidak di 3 kemungkinan 1 atau 2
//				state_jalan=2118; //3205
//				ganti_muka=0;
//				ganti_kiri=0;
//				ganti_kanan=0;
//			}
//
//		}
//		else telusurKiri_zigzag();
//		break;
//
//	case 2118: //acuan case 3205
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//		//tidak ada anjing alias anjing di posisi antara 2 dan 1
//		if(ganti_kanan == 1){
//			state_jalan = 2240; //3208
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			posisi_anjing=0; //antara 1 atau 2
//			ganti_kanan=0;
//			ganti_kiri=0;
//			ganti_muka=0;
//		}
//
//		//jika robot deteksi anjing di posisi 3 depan ruang 4B **dicek lagi posisi anjing di 3**
//		else if(detek_anjing()==1 && ((tembak_cmps()==TIMUR && state_telusur==0) ||(tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) ||(tembak_cmps()==SELATAN && state_telusur==1))){
//			posisi_anjing=3;
//			konversi_state_telusur(-1);
//			state_jalan=2119; //3206
//			hitung_langkah=0;
//			hitung_zigzag=0;
//		}
//		break;
//
//		//================posisi anjing = 3 ==============
//	case 2119: //acuan case 3206
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKanan_zigzag();
//		if((tembak_cmps()==TIMUR && state_telusur==1 ||tembak_cmps()==BARAT && state_telusur==3 || tembak_cmps()==UTARA && state_telusur==0 ||tembak_cmps()==SELATAN && state_telusur==2)){
//			if(sensor_gp_asli[1]<thd_dinding && sensor_gp[sensor[6]]<thd_dinding && sensor_gp[sensor[5]]<thd_dinding){
//				penanda_buzzer = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//				ganti_muka=0;
//				state_jalan = 2120; //3210
//				konversi_state_telusur(-1);
//			}
//		}
//		break;
//	case 2120: //acuan case 3210
//		//mendekat ke ruang 4 dengan cara maju
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130){
//			konversi_state_telusur(1);
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			state_jalan=2121; //3207
//			lock=0;
//		}
//		break;
//	case 2121: //acuan case 3207
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//masuk 4A atau 4B anjing=3
//		telusurKiriPipih();
//		if (baca_garis() == 1) {
//			if(tembak_cmps()==BARAT && state_telusur==1 || tembak_cmps()==UTARA && state_telusur==2 || tembak_cmps()==TIMUR && state_telusur==3 || tembak_cmps()==SELATAN && state_telusur==0){ // RUANG 4A DIMASUKI
//				pewaktu=0;
//				hitung_zigzag = 0;
//				hitung_langkah=0;
//				penanda_buzzer = 0;
//				state_jalan = 2251; //3214
//				ruangArr[1] = 4; //4A
//				ruangScan = 4; //4A
//			}
//		}
//		//khusus anjing 3
//		//verifikasi lorong ruang 4 dan dinding atas mengarah ke 4B
//		if(sensor_gp[sensor[0]] > 500 && sensor_gp[sensor[6]]<thd_dinding && sensor_gp[sensor[5]]<thd_dinding &&((tembak_cmps()==BARAT && state_telusur==1 || tembak_cmps()==UTARA && state_telusur==2 || tembak_cmps()==TIMUR && state_telusur==3 || tembak_cmps()==SELATAN && state_telusur==0))){
//			state_jalan=2130; //3883  //pasti 4B
//		}
//		break;
//
//	case 2130: //acuan case 3883
//		//masuk ruang 4B anjing == 3
//		telusurKiriPipih();
//		if(baca_garis()==1){
//			penanda_buzzer=0;
//			pewaktu=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			ruangArr[1] = 7; //4B
//			ruangScan = 7; //4B
//			state_jalan=2136; //3210
//		}
//
//		break;
//
//
////========================================
//
//		//posisi anjing di 2 atau 3 //persimpangan jalan ruang 3 dan 4
//	case 2131: //acuan case 3208
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//		if(ganti_kanan == 1){
//			ganti_muka = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			state_jalan=2240; //3209
//		}
//		break;
//
//	case 2132: //acuan case 3209 persimpangan jalan pojok
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//		if(ganti_kanan == 1){
//			ganti_muka = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			state_jalan=2240; //3209
//		}
//
//		break;
//	case 2240:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//		if(sensor_gp[sensor[6]]<thd_dinding && sensor_gp[sensor[2]]<thd_dinding && ((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2))){
//			//posisi anjing 2
//			if(detek_anjing() == 1){
//				if(sensor_gp_asli[0]<220){
//					state_jalan=LURUSKAN_KIRI;
//					next_state_jalan=2134; //3333
//					posisi_anjing=2;
//				}
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//			}
//			//posisi anjing 1
//			else if(detek_anjing() !=1 && sensor_gp[sensor[3]]<thd_dinding && sensor_gp_asli[1]<thd_dinding){
//				posisi_anjing=1;
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan=2134; //3333
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//			}
//		}
//		break;
//	case 2134: //acuan case 3333
//		konversi_state_telusur(1);
//		state_jalan=2135; //3355
//		break;
//
//	case 2135: //acuan case 3355
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<120){
//			lock=0;
//			konversi_state_telusur(1);
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			state_jalan=2136; //3356
//		}
//		break;
//	case 2136: //acuan case 3356
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		telusurKiriPipih();
//		if((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2)){
//			if(baca_garis()==1){ //masuk ke ruang 4B
//				penanda_buzzer=0;
//				state_jalan=2137; //3210
//				lock_acuan = tembak_cmps();
//				lock_state_telusur = state_telusur;
//				pewaktu=0;
//				hitung_langkah=0;
//				hitung_zigzag=0;
//				ruangArr[1] = 7; //4B
//				ruangScan = 7; //4B
//			}
//			//sepertinya konfigurasi 4A
//			//robot mengarah ke utara atau verifikasi lorong di antara ruang 3&4
//			else if(sensor_gp_asli[3]<thd_dinding && sensor_gp_asli[1]<thd_dinding && sensor_gp[sensor[5]]<thd_dinding && sensor_gp[sensor[0]] > 500){
//				state_jalan = 2250;//3213 //pasti konfig 4A
//				hitung_zigzag=0;
//				hitung_langkah=0;
//				ganti_kanan=0;
//				ganti_muka=0;
//				ganti_kiri=0;
//			}
//		}
//		break;
////==========================SELESAI kalo misalkan gak detek ruang 4=============================
//
//
//		//===Konfigurasi menuju 4A===========
//	case 2250: //acuan case 3213
//		telusurKiriPipih();
//		//deteksi garis ruang 4A anjing 1 atau 2
//		if(baca_garis() == 1){
//			pewaktu=0;
//			hitung_zigzag = 0;
//			hitung_langkah=0;
//			penanda_buzzer = 0;
//			state_jalan = 2251; //3214
//		}
//		break;
//
//		//masuk ke dalam ruang 4A
//	case 2251: //acuan case 3214
//		//pengecekan apakah ruang 4A yang dimasuki?
//		telusurKiri_zigzag_ruang();
//		if(hitung_zigzag == 2){
//			if((tembak_cmps()==BARAT && state_telusur==1) || (tembak_cmps()==UTARA && state_telusur==2) || (tembak_cmps()==TIMUR && state_telusur==3) || (tembak_cmps()==SELATAN && state_telusur==0)){
//				pewaktu=0;
//				hitung_zigzag = 0;
//				hitung_langkah=0;
//				state_jalan=2252; //3216
//				ruangArr[1] = 4; //4A
//				ruangScan = 4; //4A
//			}
//		}
//		break;
//
//	case 2252: //acuan case 3216 scanning ruang 4A
//		jalan_langkah_scan12(state_telusur, 1);
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			ruang_api = ruangScan; //4A
//			konversi_state_telusur(-1);
//			state_jalan = 2253; //201->2220
//		}
//		else
//		{
//			jalan_langkah(state_telusur, 1);
//			konversi_state_telusur(-1);
//			state_jalan = 2140; //tidak ada api di 4a
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//		//========================
//
//	case 2253: //acuan case 3217 2220
//		konversi_gp(state_telusur);
//		gp_serong();
//		baca_gp(state_telusur);
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2253);
//		if (baca_garis() == 1)
//		{
//			penanda_buzzer=0;
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(-1);
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			ganti_muka=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			state_jalan=2254; //3218
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 2256;
//		}
//		break;
//
//	case 2254: //acuan case 2221 3218
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2254);
//		if (baca_garis() == 1)
//		{
//			penanda_buzzer=0;
//			pewaktu=0;
//			state_jalan = 2255; //telusur kanan selain IA dan IB 2222
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			ganti_muka = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 2256;
//		}
//		break;
//
//	case 2255: //acuan case 3219
//		//tidak ada api
//		eksistansi_api1 = 0;
//		eksistansi_api2 = 0;
//		led_api_off;
//		ruang_api = 0;
//		ganti_muka = 1;
//		state_jalan=2140; //keluar ruang 4
//
//		break;
//
//	case 2256: //acuan case 3222 ada api
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 2257; //3223 memadamkan api
//			api_siap = 0;
//		}
//		break;
//
//	case 2257: //acuan case 3223
//		padamkan_api();
//		//state_jalan = 204; returning acuan case 204
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//
//		//sudah masuk ruang 4B dan mulai jalan beberapa langkah
//	case 2137: //acuan case 3210
//		if(hitung_langkah == 1){
//			jalan_langkah_scan12(state_telusur, 1);
//			pewaktu=0;
//			hitung_zigzag = 0;
//			hitung_langkah=0;
//			state_jalan=2138; //3200
//		}
//		telusurKiriPipih();
//		break;
//
//	case 2138: //acuan case 3200
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp[sensor[6]]<100){
//			state_jalan= 2139; //3225
//			hitung_zigzag = 0;
//			hitung_langkah=0;
//		}
//		else geser_kiri(lock_state_telusur,1);
//		break;
//
//		//=== KONFIG 4B ===
//	case 2139: //acuan case 3225
//		jalan_langkah_scan12(lock_state_telusur,1);
//		if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 2253; //3217
//			ruang_api = ruangScan;
//		}
//		else{
//			jalan_langkah(state_telusur, 1);
//			konversi_state_telusur(-1);
//			state_jalan=2140; //3226 mau keluar ruang 4A/4B
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//
//	case 2140: //acuan case 3226
//		//mau keluar ruang 4 jika tidak detek api case global bisa 4A atau 4B
//		telusurKanan_zigzag_ruang();
//		if(baca_garis()==1)
//		{
//			penanda_buzzer = 0;
//			pewaktu=0;
//			//khusus anjing 1 konfig 4A -> masuk ke ruang 3 dahulu lalu ruang 1
//			if(posisi_anjing  == 1 && ruangArr[1] == 4){
//				ruangArr[2] = 4; //keluar dari 4A
//				state_jalan = 2260;
//			}
//			else{
//				state_jalan=LURUSKAN_CMPS;
//				next_state_jalan = 2145; //3230 case  global
//			}
//		}
//		break;
////===============================================// Khusus konfig 4A anjing 1
//	case 2260: //acuan case 118
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		telusurKanan_zigzag();
//		if(baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer=0;
//		}
//		if (sensor_gp[sensor[5]]<thd_dinding && ganti_muka >= 2&&hitung_zigzag >= 5)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 2261;
//		}
//		break;
//
//	case 2261: //acuan case 4122
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			lock=0;
//			konversi_state_telusur(-1);
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 2263;
//		}
//		break;
//
//	case 2262:
//		telusurKanan_zigzag();
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		//mau masuk ruang 3
//		if(sensor_gp_asli[1]<thd_dinding && ((tembak_cmps()==BARAT && state_telusur==0) || (tembak_cmps()==UTARA && state_telusur==1) || (tembak_cmps()==TIMUR && state_telusur==2) || (tembak_cmps()==SELATAN && state_telusur==3))){
//			state_jalan = 2263;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//
//	case 2263:
//		telusurKanan_zigzag();
//		if(ganti_kanan == 2){
//			konversi_state_telusur(-1);
//			state_jalan = 2264;
//		}
//		break;
//
//	case 2264: //acuan case 134
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			lock=0;
//			konversi_state_telusur(1);
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 2265;
//		}
//		break;
//
//	case 2265: //acuan case 135
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)//sampai di Ruang III
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = TELUSURSCAN_KIRI; //102; //jalan lagi dari awal
//			ruangScan = 3;
//			ruangArr[3] = 3;
//			//langkahScan = 3;
//			next_state_jalan = 2266;
//			ganti_muka = 0;
//			//konversi_scan = -1;
//			//konversi_loncat = -2;
//		}
//		break;
//	case 2266: // 2301 7010 case 550 case global dengan next state jalan 2103
//		state_jalan = 2301;
//		break;
//
//
//		//===========================///
//
//
//
//		//=========== CASE GLOBAL PENENTUAN CASE SETELAH KELUAR RUANG 4A/4B==============//
//	case 2145: //acuan case 3230
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130)
//		{
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			ganti_kanan = 0;
//			ganti_kiri=0;
//			lock=0;
//			//keluar 4B
//			if(ruangArr[1] == 7){
//				ruangArr[2] = ruangArr[1];
//				//putar balik
//				if(posisi_anjing == 2){
//					konversi_state_telusur(-1);
//					state_jalan = LURUSKAN_KANAN;
//					next_state_jalan=2158; //puter balik
//					ganti_muka = 0;
//					ganti_kanan=0;
//					ganti_kiri=0;
//				}
//				//sudah detek anjing dahulu
//				else if(posisi_anjing == 1){
//					konversi_state_telusur(1);
//					ganti_muka = 0;
//					ganti_kanan=0;
//					ganti_kiri=0;
//					state_jalan=2147; //3252
//				}
//				//===============
//				else{
//					//cek anjing dulu
//					state_jalan = 2146; //3231
//					konversi_state_telusur(1);
//				}
//
//			}
//			//keluar 4A
//			else if (ruangArr[1] == 4){
//				if(posisi_anjing == 2 || posisi_anjing == 3){
//					ruangArr[2] = ruangArr[1];
//					state_jalan = 2165;//3240;
//				}
//			}
//		}
//		break;
//
//	case 2146: //acuan case 3231
//		//setelah keluar ruang 4B ***belum tau anjing di 1 atau 2
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//
//		if(((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2))){
//			//POSISI ANJING 2 KONFIG RUANG 4B
//			if(detek_anjing() == 1){
//				konversi_state_telusur(-2);
//				posisi_anjing=2;
//				state_jalan = LURUSKAN_KANAN;
//				next_state_jalan=2158;//3232 //puter balik
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//			}
//			//POSISI ANJING 1 atau 2 KONFIG RUANG 4B langsung masuk 1C
//			if(detek_anjing() != 1 && sensor_gp[sensor[2]]<thd_dinding){
//				//posisi anjing 3 sudah ditetapin sebelumnya
//				if(posisi_anjing == 3){
//					posisi_anjing=3;
//				}
//				else{
//					posisi_anjing=1;
//				}
//				ganti_muka = 0;
//				ganti_kanan=0;
//				ganti_kiri=0;
//				state_jalan=2147; //3252
//			}
//		}
//		telusurKiri_zigzag();
//		break;
//
//		//============= ingin masuk 1C ================//
//	case 2147: //acuan case 3252
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKiri_zigzag();
//		//cek anjing lagi di posisi 2
//		if (detek_anjing() == 1) {
//			konversi_state_telusur(-2);
//			posisi_anjing=2;
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan=2158; //puter balik
//			ganti_muka = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//		}
//		//konfig 4B masuk 1C apabila anjing 1 atau 3
//		if(baca_garis() == 1){
//			if((tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) ||(tembak_cmps()==SELATAN && state_telusur==2)){
//				lock_state_telusur = state_telusur;
//				penanda_buzzer = 0;
//				pewaktu=0;
//				setKompas();
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				lock_acuan = tembak_cmps();
//				state_jalan = LURUSKAN_KIRI;
//				ruangArr[3] = 6; //1C
//				ruangScan = 6; //1C
//				next_state_jalan = 2148; //3253
//				jalan_langkah_scan12(state_telusur,2);
//			}
//		}
//		break;
//
//	case 2148: //acuan case 3253 3206 sudah masuk 1C
//		//masuk ruang 1C
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (hitung_langkah == 3 || sensor_gp[sensor[0]]<100)
//		{
//			lock=0;
//			//deteksi api
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//			{
//				//dicek lagi apinya
//				state_jalan = 2149; //3254
//				pewaktu=0;
//				konversi_state_telusur(1);
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			//tidak deteksi api
//			else
//			{
//				hitung_langkah=0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//				state_jalan = 2154; //3267
//				konversi_state_telusur(-1); //supaya state telusur ke tembok (ke arah timur);
//			}
//		}
//		break;
//
//	case 2149: //acuan case 3254 ngecek api lagi di ruang 1C
//		if(nilai_api<thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 2150; //3255
//			ruang_api = ruangScan;
//		}
//		else if(eksistansi_api1 == 1 || eksistansi_api2 == 1){
//			maju(state_telusur);
//			state_jalan=2153;
//		}
//		else jalan_langkah_scan12(state_telusur,1);
//		break;
//
//	case 2150: //acuan case 3255 3208
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 2151; //3256
//			api_siap = 0;
//		}
//		break;
//
//	case 2151: //acuan case 3256 memadamkan api
//		padamkan_api();
//		state_jalan = 2152; //3257 cek apakah api sudah mati apa belum khusus di ruang 1
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//
//	case 2152: //acuan case 3257
//		if (deteksi_api(4000) == 1)
//		{
//			//api masih nyala
//			//state_jalan = 2201;
//
//			break;
//		}
//		else{
//			led_api_off;
//			//return belum diset
//			state_jalan=999; //sementara diset berhenti
//		}
//		break;
//
//	case 2153: //acuan case 3258 3207
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		//obstacle
//		if(sensor_gp_asli[0]<200)
//		{
//			lock=0;
//			//state_jalan = 115; belum diset
//		}
//		else if (nilai_api > thd_api)
//		{
//			lock=0;
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 2150;//3255 //kembali ke case sebelumnya mencari keberadaan api
//		}
//		else maju(state_telusur);
//		break;
//
//		//cek apakah ada api nyala
//	case 2154: //acuan case 3267
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//		if(eksistansi_api2 == 1||eksistansi_api1 == 1){
//			eksistansi_api1 = 0;
//			eksistansi_api2 = 0;
//			state_jalan=2149; //3254
//		}
//		if (baca_garis() == 1)
//		{
//			if(ruangArr[3] == 6 && ruangScan == 6){
//				pewaktu = 0;
//				setKompas();
//				lock_acuan = tembak_cmps();
//				lock_state_telusur = state_telusur;
//				state_jalan = 2155; //3268
//				//jalan_langkah(lock_state_telusur,2);
//			}
//		}
//		else telusurKiri_zigzag_ruang();
//		break;
//
//	case 2155: //acuan case 3268
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		//ruang 1a
//		if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<thd_dinding){
//			state_jalan = LURUSKAN_KIRI;
//			ruangArr[4] = 1; //keluar ruang 1A
//			next_state_jalan = 2195;//3283; //3283
//			penanda_pintu1=0;
//			pewaktu=0;
//			setKompas();
//			penanda_khusus_1a = 1;
//			//while (baca_garis() == 1)telusurKiri_zigzag();
//
//		}
//		//ruang 1b
//		else{
//			penanda_pintu1=1;
//			ruangArr[4] = 5; //1B
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//			ganti_kanan=0;
//			ganti_kiri=0;
//			setKompas();
//			state_jalan=LURUSKAN_CMPS;
//			next_state_jalan = 2200;//2173; //3263
//			lock_acuan = tembak_cmps();
//		}
//		break;
//
//		//=============================================
//
//	case 2158: //acuan case 3232
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		//posisi anjing 2 konfig 4B putar balik
//		telusurKanan_zigzag();
//		if((tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) || (tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==SELATAN && state_telusur==2)){
//			if((sensor_gp_asli[3]<thd_dinding) && sensor_gp_asli[1]<thd_dinding){ //330
//				state_jalan = 2160; //3233
//				hitung_zigzag=0;
//				hitung_langkah=0;
//				ganti_kanan=0;
//				ganti_muka=0;
//				ganti_kiri=0;
//			}
//		}
//		break;
//
//
//		//====START ingin masuk 1C
//	case 2160: //acuan case 3233
//		//mendekat ke ruang 4 dengan cara maju **konfig 4B anjing 2
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		if(sensor_gp_asli[3]<140){
//			state_jalan=2161; //3234
//			ganti_muka=0;
//			ganti_kiri=0;
//			ganti_kanan=0;
//			pewaktu = 0;
//		}
//		else geser_kiri(state_telusur,1);
//		break;
//
//	case 2161: //acuan case 3234
//		//konfig 4B anjing 2
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//
//		telusurKiri_zigzag();
//		if(ganti_kiri==2){
//			konversi_state_telusur(1);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			//posisi anjing 2 dan harus masuk 1C
//			if(sensor_gp[sensor[7]]>thd_dinding && sensor_gp[sensor[1]]>thd_dinding && ((tembak_cmps()==TIMUR && state_telusur==0) ||(tembak_cmps()==BARAT && state_telusur==2) || (tembak_cmps()==UTARA && state_telusur==3) ||(tembak_cmps()==SELATAN && state_telusur==1))){
//				state_jalan = 2162; //3235
//			}
//		}
//		//antisipasi takut detek anjing lagi
//		if(detek_anjing() == 1){
//			posisi_anjing = 1;
//			state_jalan = 2145;
//		}
//		break;
//
//	case 2162: //acuan case 3235
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130){
//			lock=0;
//			konversi_state_telusur(1);
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 2163; //3236
//		}
//		break;
//
//	case 2163: //acuan case 3236
//		//konfig 4B masuk 1C apabila anjing 1
//		telusurKiri_zigzag();
//		if(baca_garis() == 1){
//			if((tembak_cmps()==TIMUR && state_telusur==1) || (tembak_cmps()==BARAT && state_telusur==3) || (tembak_cmps()==UTARA && state_telusur==0) ||(tembak_cmps()==SELATAN && state_telusur==2)){
//				lock_state_telusur = state_telusur;
//				penanda_buzzer = 0;
//				pewaktu=0;
//				setKompas();
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				lock_acuan = tembak_cmps();
//				state_jalan = LURUSKAN_KIRI;
//				ruangArr[3] = 6; //1C
//				ruangScan = 6; //1C
//				next_state_jalan = 2148; //3253
//				jalan_langkah_scan12(state_telusur,2);
//			}
//		}
//		break;
//
//		//==================================
//
//
//
//	case 2165: //acuan case 3240
//		//konfigurasi 4A
//		if(posisi_anjing==1){
//			//siap masuk ke ruang 1
//			konversi_state_telusur(1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=2170; //masuk antara 1A atau 1B
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			pewaktu=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//		}
//		//posisi anjing 2 dan 3 siap masuk 1 C **sementara dibuat masuk 1A atau 1B dulu
//		else if(posisi_anjing == 2 || posisi_anjing==3){
//			konversi_state_telusur(-1); //cek kembali apakah ada anjing di posisi 1
//			state_jalan=LURUSKAN_KANAN;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			next_state_jalan=2166; //3320
//			pewaktu=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//		}
//		break;
//
//	case 2166: //acuan case 3320
//		if(detek_anjing() == 1){
//			posisi_anjing = 1;
//			konversi_state_telusur(-2);
//			state_jalan=2170; //3241
//		}
//		//anjing di antara posisi 1 atau 2
//		else {
//			state_jalan=2167; //3321
//			pewaktu=0;
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//		}
//		break;
//
//	case 2167: //acuan case 3321
//		telusurKanan_zigzag();
//		if(ganti_kanan == 1){
//			konversi_state_telusur(-1);
//			state_jalan = 2168; //3322
//			hitung_langkah=0;
//			hitung_zigzag=0;
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//		}
//		break;
//
//	case 2168: //3322
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<130){
//			lock=0;
//			konversi_state_telusur(1);
//			ganti_kanan=0;
//			ganti_muka=0;
//			ganti_kiri=0;
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 2163; //3236
//		}
//		break;
//
//		//masuk antara 1A atau 1B
//		case 2170: //acuan case 3241
//			telusurKiriPipih();
//			//masih belum bisa menentukan ruang 1A atau 1B setelah detek garis
//			if(baca_garis() == 1){
//				setKompas();
//				state_jalan = LURUSKAN_CMPS;
//				next_state_jalan = 2171; //3242
//				penanda_buzzer=0;
//				hitung_langkah=0;
//				hitung_zigzag=0;
//				lock_acuan=tembak_cmps();
//			}
//			break;
//
//		case 2171: //acuan case 3242
//			if(lock_acuan == TIMUR)
//			{
//				state_telusur = 0;
//			}
//			else if(lock_acuan == UTARA)
//			{
//				state_telusur = 3;
//			}
//			else if(lock_acuan == SELATAN)
//			{
//				state_telusur = 1;
//			}
//			else if(lock_acuan == BARAT)
//			{
//				state_telusur = 2;
//			}
//			jalan_langkah(state_telusur,3);
//			state_jalan=2172;
//			break;
//
//		case 2172: //acuan case 3243 12
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			//persyaratan ruang 1B
//			if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<300){
//				ruangScan = 5; //1B = 5
//				ruangArr[3] = 5;
//				konversi_state_telusur(-1);
//				ganti_muka=0;
//				ganti_kiri=0;
//				hitung_langkah=0;
//				state_jalan=2173; //3245
//			}
//			//persyaratan ruang 1A
//			else if(sensor_gp[sensor[2]]<thd_dinding){
//				ruangScan = 1; //1A = 1
//				ruangArr[3] = 1;
//				konversi_state_telusur(1);
//				ganti_muka=0;
//				ganti_kiri=0;
//				hitung_langkah=0;
//				state_jalan=2187;
//			}
//			break;
//
//			//Konfigurasi 1B atau 5
//		case 2173: //acuan case 3245
//			baca_gp(state_telusur);
//			maju(state_telusur);
//			if(sensor_gp_asli[0]<130)
//			{
//				lock=0;
//				konversi_state_telusur(1);
//				penanda_buzzer = 0;
//				ganti_muka = 0;
//				penanda_pintu1 = 1;
//				state_jalan = 2174; //3246
//				hitung_langkah = 0;
//			}
//			break;
//
//		case 2174: //acuan case 3246
//			telusurKiri_zigzag_ruang();
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//			{
//				state_jalan = 2175; //3247
//				ruang_api = ruangScan;
//			}
//			//tidak deteksi api
//			else
//			{
//				state_jalan = 2180; //3260
//				konversi_state_telusur(-1);
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//			break;
//
//		case 2175: //acuan case 3247 9006
//			muter_acuankompas(acuan_1b);
//			if(sudah_lurus==1)
//			{
//				pewaktu=0;
//				state_jalan=2176; //3248
//			}
//			break;
//
//		case 2176: //acuan case 3248
//			muter_(kan,5,20);
//			if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//			{
//				state_jalan=2178; //3250
//			}
//			else if(pewaktu>110)
//			{
//				if(ruang_api==5) //konfigurasi 1B
//				{
//					pewaktu=0;
//					state_jalan=2177; //3249
//				}
//
//			}
//			break;
//
//		case 2177: //acuan case 3249
//			muter_(kir,5,25);
//			if(pewaktu>50 && ruang_api==5)
//			{
//				state_telusur=0;
//				lock_state_telusur=state_telusur;
//				konversi_gp(state_telusur);
//				baca_gp(state_telusur);
//				gp_serong();
//				geser_kiri(state_telusur,1);
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan=999; //berhenti deteksi error
//			}
//			break;
//
//		case 2178: //acuan case 3250 9003
//			led_api_on;
//			status_tpa=1;
//			PID_tpa();
//			if(PID_tpa()==1)
//			{
//				padamkan_api();
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				state_jalan=2179; //3251
//			}
//			break;
//
//		case 2179://acuan case 3251 9004
//			if (deteksi_api(4000) == 1)
//				{
//					state_telusur=0;
//					baca_gp(state_telusur);
//					gp_serong();
//					state_jalan = 2175;//balik ke awal, padamin pake lagoritma lama 3247
//					break;
//				}
//				else
//				{
//					led_api_off;
//					if(ruang_api==5 && status_tpa==1) //1B
//					{
//						mataangin = SELATAN;
//						state_jalan =LURUSKAN_CMPS;
//						next_state_jalan = 999; //berhenti dulu
//						state_telusur = 0;
//						konversi_gp(0);
//					}
//				}
//			break;
//
//			//tidak deteksi api di 1B
//		case 2180: //acuan case 3260
//			baca_gp(state_telusur);
//			maju(state_telusur);
//			if(sensor_gp_asli[0]<75)
//			{
//				lock=0;
//				konversi_state_telusur(-1);
//				state_jalan = 2181;
//			}
//			break;
//
//		case 2181: //acuan case 3261 98
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 3262;
//			break;
//
//		case 2182: //acuan case 3262 99 keluar ruang 1B
//			telusurKanan_zigzag_ruang();
//			if (baca_garis() == 1)
//			{
//				ruangArr[4] = 5; //1B
//				penanda_buzzer = 0;
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//				setKompas();
//				state_jalan=LURUSKAN_CMPS;
//				next_state_jalan = 2183; //3263 keluar 1B
//				lock_acuan = tembak_cmps();
//			}
//			break;
//
//		case 2183: //acuan case 3263
//			if(lock_acuan == TIMUR)
//			{
//				state_telusur = 2;
//			}
//			else if(lock_acuan == UTARA)
//			{
//				state_telusur = 1;
//			}
//			else if(lock_acuan == SELATAN)
//			{
//				state_telusur = 3;
//			}
//			else if(lock_acuan == BARAT)
//			{
//				state_telusur = 0;
//			}
//			state_jalan=2184; //3264
//			break;
//
//		case 2184: //acuan case 3264'
//			//keluar ruang 1B
//			maju(state_telusur);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			if(sensor_gp_asli[0]<100)
//			{
//				lock=0;
//				next_state_jalan = 2112;//2185; //3265
//				konversi_state_telusur(1);
//				state_jalan=LURUSKAN_KIRI;
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//			}
//			break;
//
//			//Konfigurasi 1A
//		case 2187: //acuan case 3271 1011
//			baca_gp(state_telusur);
//			maju(state_telusur);
//			if(sensor_gp_asli[0]<100)
//			{
//				lock=0;
//				konversi_state_telusur(-1);
//				penanda_buzzer = 0;
//				ganti_muka = 0;
//				penanda_pintu1 = 1;
//				state_jalan = 2188; //3272
//				hitung_langkah=0;
//				hitung_zigzag=0;
//				penanda_pintu1=0;
//			}
//			break;
//
//		case 2188: //acuan case 3272 1012
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			maju(state_telusur);
//
//			if (hitung_langkah == 1)//anti detect diruang 2
//			{
//				lock=0;
//				penanda_buzzer = 0;
//				konversi_gp(state_telusur);
//				jalan_langkah_scan12(state_telusur, 2);
//				if (eksistansi_api1 == 1)
//				{
//					penanda_buzzer = 0;
//					konversi_state_telusur(1);
//					state_jalan = 2189; //3273
//					ruang_api = ruangScan;
//				}
//				else if (eksistansi_api2 == 1)
//				{
//					eksistansi_api1 = 0;
//					konversi_state_telusur(-1);
//					jalan_langkah(state_telusur, 2);
//					jalan_langkah_scan1(state_telusur, 2);
//					if (eksistansi_api1 == 1)
//					{
//						penanda_buzzer = 0;
//						konversi_state_telusur(-2);
//						jalan_langkah(state_telusur, 3);
//						state_jalan = 2189; //3273
//						ruang_api = ruangScan;
//					}
//					else
//					{
//						eksistansi_api2 = 0;
//						eksistansi_api1 = 0;
//						konversi_state_telusur(1); //-2x`
//						state_jalan = 2193; //3280
//						ganti_muka = 0;
//						hitung_zigzag = 0;
//
//					}
//				}
//				else if(hitung_langkah<=5&&sensor_gp[sensor[0]]<120)
//				{
//					konversi_state_telusur(-1);
//					hitung_langkah = 0;
//				}
//				else
//				{
//					konversi_state_telusur(1);
//					state_jalan = 3280; //3280
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//				}
//			}
//	//		else if(sensor_gp[sensor[0]]<120){
//	//
//	//		}
//			else if (nilai_api > thd_api&&hitung_langkah<3)
//			{
//				lock=0;
//				penanda_buzzer = 0;
//				gerakStatic(0, 0, 0, 0, 0, 0, 30);
//				//state_jalan = 202;
//				ruang_api = ruangScan;
//			}
//			break;
//
//		case 2189: //acuan case 3273 9005
//			muter_acuankompas(acuan_1a);
//			if(sudah_lurus==1)
//			{
//				pewaktu=0;
//				state_jalan=2190; //3274
//			}
//			break;
//
//		case 2190: //acuan case 3274 9002
//			muter_(kir,5,20);
//			if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//			{
//				jalan_langkah(state_telusur,1);
//				state_jalan=2191; //3275
//			}
//			else if(pewaktu>110)
//			{
//				if(ruang_api==1)//tpa tdk dapat
//				{
//					pewaktu=0;
//					//state_jalan=9010;
//				}
//			}
//			break;
//
//		case 2191: //acuan case 3275 9003
//			led_api_on;
//			status_tpa=1;
//			PID_tpa();
//			if(PID_tpa()==1)
//			{
//				padamkan_api();
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				state_jalan=2192; //3276
//			}
//			break;
//
//		case 2192: //acuan case 3276 9004
//			//jika api nyala lagi
//			if (deteksi_api(4000) == 1)
//			{
//				state_telusur=0;
//				baca_gp(state_telusur);
//				gp_serong();
//				state_jalan = 2188;//3272 balik ke awal, padamin pake lagoritma lama
//				break;
//			}
//			else
//			{
//				led_api_off;
//				if(ruang_api == 1 && status_tpa == 1)//acuan sesudah tpa
//				{
//					mataangin = UTARA;
//					state_jalan =LURUSKAN_CMPS;
//					next_state_jalan = 999; //return belum diset
//					state_telusur = 0;
//					konversi_gp(0);
//				}
//			}
//			break;
//
//			//tak deteksi api di ruang 1A
//		case 2193: //acuan case 3280 96
//			baca_gp(state_telusur);
//			maju(state_telusur);
//			if(sensor_gp_asli[0]<75)
//			{
//				lock=0;
//				konversi_state_telusur(1);
//				state_jalan = 2194; //3281
//			}
//			break;
//
//		case 2194: //acuan case 3281 97
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 2195; //3282
//			break;
//
//		case 2195: //acuan case 2105
//			telusurKiri_zigzag();
//			if (baca_garis() == 1) //garis di 1a
//			{
//				pewaktu=0;
//				hitung_zigzag = 0;
//				penanda_khusus_1a = 1;
//				//penanda_pintu1=0;
//			}
//			//telusurKiri_zigzag();
//			if (hitung_zigzag == 2&&penanda_khusus_1a == 1)
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				ruangArr[4] = 1; //keluar ruang 1A
//				konversi_state_telusur(1);
//				state_jalan = 2196;
//				ganti_muka = 0;
//				penanda_khusus_1a = 0;
//			}
//			break;
//
//		case 2196: //acuan cae 2106 konfig 1A
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			maju(state_telusur);
//			if (sensor_gp_asli[1] < thd_dinding) //4
//			{
//				lock = 0;
//				jalan_langkah(state_telusur, 2); //tadi 1
//				penanda_buzzer = 0;
//
//				state_jalan = 2197; //2272
//			}
//			break;
//
//		case 2197: //acuan case 2272
//			konversi_state_telusur(-1); //ingin mepet ke ruang 2
//			//state_jalan = LURUSKAN_CMPS;
//			state_jalan = 2198; //2273
//			break;
//
//		case 2198: //2273 konfig 1A
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 2199;// 2274;
//			break;
//
//		case 2199: //2274 konfig 1A
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			maju(state_telusur);
//			if (sensor_gp_asli[0] < 100) //1
//			{
//				lock=0;
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 2200; //2112
//				ganti_kiri=0;
//			}
//			break;
//
//		case 2200: //konfig 1A dan 1B ingin masuk ruang 3
//
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			telusurKiriPipih();
//			if((tembak_cmps()==TIMUR && state_telusur==2) ||(tembak_cmps()==BARAT && state_telusur==0) || (tembak_cmps()==UTARA && state_telusur==1) ||(tembak_cmps()==SELATAN && state_telusur==3)){
//				if(sensor_gp_asli[3]<thd_dinding && sensor_gp_asli[1]<thd_dinding) {
//					ganti_kanan=0;
//					ganti_kiri=0;
//					ganti_muka=0;
//					hitung_zigzag = 0;
//					hitung_langkah = 0;
//					penanda_buzzer=0;
//					state_jalan=2300; //2300
//				}
//			}
//			break;
//
//		case 2300: //acuan case 4029 3294 2012 7094
//			telusurKiri_zigzag();
//			if(baca_garis() == 1){
//				//kirim_case();
//				penanda_buzzer = 0;
//				pewaktu=0;
//				jalan_langkah(state_telusur,2);
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				state_jalan = LURUSKAN_KIRI; //102; //jalan lagi dari awal
//				ruangScan = 3;
//				ruangArr[5] = 3;
//				next_state_jalan = 2301; //7010
//				//langkahScan = 3;
//				ganti_muka = 0;
//				//konversi_scan = -1;
//				//konversi_loncat = -2;
//			}
//			break;
//
//		case 2301: //acuan case 7010 case 550 case global dengan next state jalan 2103
//			//scanning ruang 3
//			telusurKiri_zigzag_ruang();
//
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//
//			if(ganti_muka>=1)
//			{
//				konversi_state_telusur(-2);
//				state_jalan = 2320;//7025;
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//			else if(sensor_gp[sensor[0]]<100)//edit 25-04-2018
//			{
//				konversi_state_telusur(-1);
//				state_jalan = 2320;//7025;
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//
//			if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//			{
//				konversi_state_telusur(1);
//				//jalan_langkah(state_telusur,2);
//				state_jalan = 2302; //7011
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				state_jalan = 2320;//7025;
//				konversi_state_telusur(-1);
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//			break;
//
//		case 2302:// acuan case 7011 136
//			//uv deteksi api
//			maju(state_telusur);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			if(sensor_gp[sensor[0]]<100||sensor_gp_asli[0]<70)
//			{
//				lock=0;
//				state_jalan=2303; //7012
//			}
//			break;
//
//		case 2303: //7012 137
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//			{
//				konversi_state_telusur(-2);
//				state_jalan = 2304; //7013
//				api3=1;
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				api3=0;
//				konversi_state_telusur(-2);
//				state_jalan = 2304; //7013
//			}
//			break;
//
//		case 2304: //7013 138
//			maju(state_telusur);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			if(sensor_gp[sensor[0]]<100||sensor_gp_asli[0]<70)
//			{
//				lock = 0;
//				if(api3==1)
//				{
//					state_jalan=2310; //7014
//				}
//				else
//				{
//					state_jalan = 2320;
//					konversi_state_telusur(-1);
//					ganti_muka = 0;
//					hitung_langkah = 0;
//					hitung_zigzag = 0;
//				}
//			}
//			break;
//
//		case 2310://case global 201 -> 9007 2310
//			statusKompas = 1;
//			jalan_langkah(state_telusur,2);
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 2311; //7015
//			break;
//
//		case 2311: //acuan case 9007 7015
//			muter_acuankompas(acuan_3_4a);
//			if(sudah_lurus==1)
//			{
//				pewaktu=0;
//				state_jalan=2312; //7016
//			}
//			break;
//
//		case 2312: //acuan case 7016 9001
//			muter_(kan,5,20);
//			if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//			{
//				state_jalan=2315; //7019
//			}
//			else if(pewaktu>85 && (ruang_api==3))
//			{
//				pewaktu=0;
//				state_jalan=2313; //7017
//			}
//
//			else if(pewaktu>110)
//			{
//				state_jalan=999; // belum diset cek error
//			}
//			break;
//
//		case 2313: //acuan case 7017 9009
//			muter_(kir,5,25);
//			if(pewaktu>45 && ruang_api==3)
//			{
//				state_telusur=0;
//				konversi_gp(state_telusur);
//				baca_gp(state_telusur);
//				gp_serong();
//				geser_kiri(state_telusur,1);
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan=2314; //7018
//			}
//			break;
//
//		case 2314: //acuan case 7018 2220
//			gp_serong();
//			telusurKiri_zigzag_ruang();
//			deteksi_lost_acuan(0, 2313);
//			if (baca_garis() == 1)
//			{
//				//kirim_case();
//				penanda_buzzer=0;
//				pewaktu=0;
//				konversi_state_telusur(-2);
//				jalan_langkah(state_telusur, 3);
//				konversi_state_telusur(1);
//				//state_jalan = 2221; belum diset
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//			}
//			if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//			{
//				geser_kanan(state_telusur,1);
//			}
//			if (nilai_api > thd_api)
//			{
//				penanda_buzzer = 0;
//				gerakStatic(0, 0, 0, 0, 0, 0, 30);
//				state_jalan = 2317;//7021;
//			}
//
//			break;
//
//		case 2315: //acuan case 7019 9003
//			led_api_on;
//			status_tpa=1;
//			PID_tpa();
//			if(PID_tpa()==1)
//			{
//				padamkan_api();
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				state_jalan=2316; //7020
//			}
//			break;
//
//		case 2316: //acuan case 7020 9004
//			//jika api nyala lagi
//			if (deteksi_api(4000) == 1)
//			{
//				state_telusur=0;
//				baca_gp(state_telusur);
//				gp_serong();
//				state_jalan = 2311;//7015;//balik ke awal, padamin pake lagoritma lama
//				break;
//			}
//			else
//			{
//				led_api_off;
//
//				/* belum diset
//				 if(ruang_api==3 && status_tpa==1)
//				{
//					mataangin = BARAT;
//					state_jalan =LURUSKAN_CMPS;
//					next_state_jalan = 498;
//					state_telusur = 0;
//					konversi_gp(0);
//				}*/
//			}
//
//			break;
//
//		case 2317: //acuan case 7021 202
//			led_api_on;
//			PID_api();
//			if (PID_api() == 1)
//			{
//				penanda_buzzer = 0;
//				state_jalan = 2318; //7022
//				api_siap = 0;
//			}
//
//			break;
//
//		case 2318: //acuan case 7022 203
//			padamkan_api();
//			//state_jalan = 204; return belum diset
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			status_api = 1;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			statusKompas = 0;
//			state_jalan=999;//belum diset
//			break;
//
//		case 2320: //acuan case 7025 2013 tidak ada api
//			//ingin keluar ruang 3
//			telusurKanan_zigzag();
//			if (baca_garis() == 1)
//			{
//				//kirim_case();
//				penanda_buzzer=0;
//				pewaktu=0;
//				while (baca_garis() == 1)telusurKanan_zigzag();
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				setKompas();
//				if(ruangArr[5] == 3){
//					ruangArr[6]=3;
//					state_jalan = 2335;
//				}
//				//konfig ruang start 2, 4a, anjing 1, khusus
//				else if(ruangArr[3] == 3){
//					ruangArr[4]=3;
//					state_jalan = 2350; //khusus
//				}
//
//			}
//			break;
//		case 2335:
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			if(ganti_kiri == 1){
//				state_jalan = 2325;
//			}
//			telusurKanan_zigzag();
//			break;
//
//		case 2325: //acuan case 3101
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			//mendeteksi siku di ruang 3
//			if(sensor_gp_asli[3]<thd_dinding && sensor_gp_asli[1]<300) {
//				state_jalan=2326; //3111
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//			}
//			telusurKanan_zigzag();
//
//			break;
//
//
//
//		case 2326: //acuan case 3111
//			if(hitung_zigzag == 1){
//				state_jalan=LURUSKAN_CMPS;
//				next_state_jalan = 2327;//3201;
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				konversi_state_telusur(-1);
//			}
//			else telusurKanan_zigzag();
//			break;
//
//		case 2327://acuan case 3201
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			//state_jalan = LURUSKAN_CMPS;
//			state_jalan=2329; //3301
//			break;
//
//		case 2328:
////			gerakStatic(0, 0, 0, 0, 0, 0, 30);
////			state_jalan = 2329;
//			break;
//
//		case 2329: //acuan case 3301 2270
////			baca_gp(state_telusur);
////			konversi_gp(state_telusur);
////			gp_serong();
//			//maju(state_telusur);
//
////			//takut masuk ruang 3 lagi
////			if (baca_garis() == 1) {
////				lock_state_telusur = state_telusur;//lock state
////				penanda_buzzer = 0;
////				pewaktu=0;
////				setKompas();
////				lock_acuan=tembak_cmps();//mulai lock acuan kompas
////				hitung_langkah = 0;
////				state_jalan=LURUSKAN_CMPS;
////				next_state_jalan=999;
////			}
////
////			if(sensor_gp_asli[0]<140){
////				state_jalan=LURUSKAN_CMPS;
////				konversi_state_telusur(1);
////				next_state_jalan = 2329;//3298;
////				lock=0;
////			}
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			if(sensor_gp_asli[0]<300){
//				lock=0;
//				next_state_jalan=LURUSKAN_CMPS;
//				state_jalan = 2340; //3298
//			}
//			else{
//				if(sensor_gp_asli[3]<thd_dinding){
//					geser_kanan(state_telusur,1);
//					lock=0;
//				}
//				else{
//					geser_kiri(state_telusur,1);
//					lock=0;
//				}
//			}
//			break;
//
//		case 2340: //acuan case 3298
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan=2341; //3302
//			break;
//
//		case 2341:
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			maju(state_telusur);
//
//			//takut masuk ruang 3 lagi
//			if (baca_garis() == 1) {
//				lock=0;
//				lock_state_telusur = state_telusur;//lock state
//				penanda_buzzer = 0;
//				pewaktu=0;
//				setKompas();
//				lock_acuan=tembak_cmps();//mulai lock acuan kompas
//				hitung_langkah = 0;
//				state_jalan=LURUSKAN_CMPS;
//				next_state_jalan=999; //deteksi error belum diset (berhenti)
//			}
//
//			//persimpangan siku ruang 3
//			if(sensor_gp_asli[0]<140){
//				state_jalan=LURUSKAN_CMPS;
//				konversi_state_telusur(1);
//				next_state_jalan = 2345 ;
//				lock=0;
//			}
//			break;
//
//		case 2345: //acuan case 3302
//			state_jalan = 2116; //ngulang di awal lagi :))
//			break;
//
//			//=============== case khusus konfig 4a anjing 1
//		case 2350: //acuan case 7026 4033 89
//			//khusus konfig 4a anjing 1
//			telusurKanan_zigzag();
//			if (ganti_muka == 3l)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(-1);
//				state_jalan = 2351;//7027;
//			}
//			break;
//
//		case 2351: //acuan case 7027 4033 90
//			//ingin masuk ruang 1
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			maju(state_telusur);
//			if (sensor_gp_asli[0] < 120)
//			{
//				lock=0;
//				tembak_cmps();//edit
//				//konfig 1a
//				if(penanda_pintu1 == 0){
//					konversi_state_telusur(-1);
//					state_jalan = LURUSKAN_KANAN;
//					next_state_jalan = 2352; //7029;
//
//				}
//				//konfig 1b
//				else{
//					konversi_state_telusur(-1);
//					state_jalan = 2365;
//				}
//
//			}
//			break;
//
//			//=========konfigurasi 1a=======
//		case 2352: //acuan case 7029 4056 2091
//			telusurKanan_zigzag();
//			if (baca_garis() == 1) //sampai di ruang 1a
//			{
//				pewaktu=0;
//				setKompas();
//				hitung_zigzag = 0;
//				hitung_langkah = 0;
//				state_jalan = 2353;//7030;
//				ruangArr[5] = 1;
//				lock_acuan = tembak_cmps();
//				ruangScan = 1;
//			}
//			break;
//
//		case 2353: //acuan case 7030 4057 91
//			if(lock_acuan == TIMUR)
//			{
//				state_telusur = 0;
//			}
//			else if(lock_acuan == UTARA)
//			{
//				state_telusur = 3;
//			}
//			else if(lock_acuan == SELATAN)
//			{
//				state_telusur = 1;
//			}
//			else if(lock_acuan == BARAT)
//			{
//				state_telusur = 2;
//			}
//			jalan_langkah(state_telusur,3);
//
//			state_jalan = 2355;
// 			break;
//
//		case 2354: //walaupun sudah ditentukan konfig 1a tp tetap dicek lagi
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			//persyaratan ruang 1B
//			if(sensor_gp[sensor[7]]<thd_dinding && sensor_gp_asli[3]<300){
//				ruangScan = 5; //1B = 5
//				ruangArr[1] = 5;
//				konversi_state_telusur(-1);
//				ganti_muka=0;
//				ganti_kiri=0;
//				hitung_langkah=0;
//				state_jalan=2365;
//			}
//			//persyaratan ruang 1A
//			if(sensor_gp[sensor[2]]<thd_dinding){
//				ruangScan = 1; //1A = 1
//				ruangArr[1] = 1;
//				konversi_state_telusur(1);
//				ganti_muka=0;
//				ganti_kiri=0;
//				hitung_langkah=0;
//				state_jalan=2355;
//			}
//			break;
//
//
//			//==konfigurasi 1A========================
//		case 2355: //acuan case 7060
//			//scanning 1A
//			baca_gp(state_telusur);
//			jalan_langkah_scan12(state_telusur,1);
//			if (eksistansi_api1 == 1)
//			{
//				lock=0;
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 2357; //7062
//				ruang_api = ruangScan;
//			}
//			else if (eksistansi_api2 == 1)
//			{
//				lock=0;
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-1);
//				jalan_langkah(state_telusur, 2);
//				jalan_langkah_scan1(state_telusur, 2);
//				if (eksistansi_api1 == 1)
//				{
//					penanda_buzzer = 0;
//					konversi_state_telusur(-2);
//					jalan_langkah(state_telusur, 3);
//					state_jalan = 2357; //7062
//					ruang_api = ruangScan;
//				}
//				else
//				{
//					eksistansi_api2 = 0;
//					eksistansi_api1 = 0;
//					konversi_state_telusur(1); //-2x`
//					state_jalan = 2361; //7066;
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//
//				}
//			}
//			if(sensor_gp_asli[0]<100)
//			{
//				lock=0;
//				konversi_state_telusur(-1);
//				penanda_buzzer = 0;
//				ganti_muka = 0;
//				//penanda_pintu1 = 1;
//				state_jalan = 2356;
//				hitung_langkah=0;
//				hitung_zigzag=0;
//				penanda_pintu1=0;
//			}
//			break;
//
//		case 2356: //acuan case 7061 3272 1012
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 2357;//7062;
//				ruang_api = ruangScan;
//			}
//			else if (eksistansi_api2 == 1)
//			{
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-1);
//				jalan_langkah(state_telusur, 2);
//				jalan_langkah_scan1(state_telusur, 2);
//				if (eksistansi_api1 == 1)
//				{
//					penanda_buzzer = 0;
//					konversi_state_telusur(-2);
//					jalan_langkah(state_telusur, 3);
//					state_jalan = 2357;//7062;
//					ruang_api = ruangScan;
//				}
//				else
//				{
//					eksistansi_api2 = 0;
//					eksistansi_api1 = 0;
//					konversi_state_telusur(1); //-2x`
//					state_jalan = 2361;
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//
//				}
//			}
//			else if(hitung_langkah<=5&&sensor_gp[sensor[0]]<120)
//			{
//				konversi_state_telusur(-1);
//				hitung_langkah = 0;
//			}
//			else
//			{
//				konversi_state_telusur(1);
//				state_jalan = 2361;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//
//
//			if (nilai_api > thd_api&&hitung_langkah<3)
//			{
//				lock=0;
//				penanda_buzzer = 0;
//				gerakStatic(0, 0, 0, 0, 0, 0, 30);
//				//state_jalan = 202;
//				ruang_api = ruangScan;
//			}
//			break;
//
//		case 2357: //acuan case 7062 3273 9005
//			muter_acuankompas(acuan_1a);
//			if(sudah_lurus==1)
//			{
//				pewaktu=0;
//				state_jalan=2358;//7063;
//			}
//			break;
//
//		case 2358: //acuan case 2358 7063 3274 9002
//			muter_(kir,5,20);
//			if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//			{
//				jalan_langkah(state_telusur,1);
//				state_jalan=2359;//7064; //3275
//			}
//			else if(pewaktu>110)
//			{
//				if(ruang_api==1)//tpa tdk dapat
//				{
//					pewaktu=0;
//					//state_jalan=9010;
//				}
//			}
//			break;
//
//		case 2359: //acuan case 7064 3275 9003
//			led_api_on;
//			status_tpa=1;
//			PID_tpa();
//			if(PID_tpa()==1)
//			{
//				padamkan_api();
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				state_jalan=2360;//7065; //3276
//			}
//			break;
//
//		case 2360: //acuan case 7065 3276 9004
//			//jika api nyala lagi
//			if (deteksi_api(4000) == 1)
//			{
//				state_telusur=0;
//				baca_gp(state_telusur);
//				gp_serong();
//				state_jalan = 2356;//balik ke awal, padamin pake lagoritma lama
//				break;
//			}
//			else
//			{
//				led_api_off;
//				if(ruang_api == 1 && status_tpa == 1)//acuan sesudah tpa
//				{
//					mataangin = UTARA;
//					state_jalan =LURUSKAN_CMPS;
//					next_state_jalan = 999; //return belum diset
//					state_telusur = 0;
//					konversi_gp(0);
//				}
//			}
//			break;
//
//			//tak deteksi api di ruang 1A
//		case 2361: //7066 4012 3280 96
//			baca_gp(state_telusur);
//			maju(state_telusur);
//			if(sensor_gp_asli[0]<75)
//			{
//				lock=0;
//				konversi_state_telusur(1);
//				state_jalan = 2362;//7067;
//			}
//			break;
//
//		case 2362: //7067 3281 97
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 2363;//7068; //3282
//			break;
//
//		case 2363: //acuan case 2195 2105
//			telusurKiri_zigzag();
//			if (baca_garis() == 1) //garis di 1a
//			{
//				pewaktu=0;
//				hitung_zigzag = 0;
//				penanda_khusus_1a = 1;
//			}
//			if (hitung_zigzag == 2&&penanda_khusus_1a == 1)
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				ruangArr[4] = 1; //keluar ruang 1A
//				konversi_state_telusur(1);
//				state_jalan = 2378; //2196 2106
//				ganti_muka = 0;
//				penanda_khusus_1a = 0;
//			}
//			break;
//			//===========================================
//
//
//			//=========konfigurasi 1B ==========
//		case 2365: //acuan case 7033 4061
//			telusurKanan_zigzag();
//			if(baca_garis() == 1){
//				pewaktu=0;
//				setKompas();
//				lock_acuan=tembak_cmps();
//				lock_state_telusur = state_telusur;
//				state_jalan=2366;//7034;
//				ruangArr[5] = 5; //1B
//			}
//			break;
//
//		case 2366: //acuan case 7099 4063 4057 91
//			if(lock_acuan == TIMUR)
//			{
//				state_telusur = 0;
//			}
//			else if(lock_acuan == UTARA)
//			{
//				state_telusur = 3;
//			}
//			else if(lock_acuan == SELATAN)
//			{
//				state_telusur = 1;
//			}
//			else if(lock_acuan == BARAT)
//			{
//				state_telusur = 2;
//			}
//			jalan_langkah(state_telusur,2 );
//			state_jalan=2367;//7100;
//			break;
//
//		case 2367: //acuan case 7100 6064
//			konversi_state_telusur(-1);
//			state_jalan=2368; // 7101
//			ganti_muka=0;
//			break;
//
//		case 2368: //acuan case 7101 4065
//			baca_gp(state_telusur);
//			maju(state_telusur);
//			if(sensor_gp_asli[0]<100)
//			{
//				lock=0;
//				hitung_langkah=0;
//				konversi_state_telusur(1);
//				penanda_buzzer = 0;
//				ganti_muka = 0;
//				state_jalan = 2369;//7044; //mepet tembok
//
//			}
//			break;
//
//		case 2369://acuan case 7044 scanning api di ruang 1B
//			telusurKiri_zigzag_ruang();
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//			{
//				state_jalan = 2370;//7045; //3247
//				ruang_api = ruangScan;
//			}
//			//tidak deteksi api
//			else
//			{
//				state_jalan = 2375;//7050;
//				konversi_state_telusur(-1);
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//			break;
//
//		case 2370: //acuan case 7045 3247 9006
//			muter_acuankompas(acuan_1b);
//			if(sudah_lurus==1)
//			{
//				pewaktu=0;
//				state_jalan=2371;//7046; //3248
//			}
//			break;
//
//		case 2371: //acuan case 7046 3248 9001
//			muter_(kan,5,20);
//			if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//			{
//				state_jalan=2373;//7048;
//			}
//			else if(pewaktu>110)
//			{
//				if(ruang_api==5)
//				{
//					pewaktu=0;
//					state_jalan=2372; //7047;
//				}
//
//			}
//
//			break;
//
//		case 2372: //acuan case 7047 3249 9009
//			muter_(kir,5,25);
//			if(pewaktu>50 && ruang_api==5)
//			{
//				state_telusur=0;
//				lock_state_telusur=state_telusur;
//				konversi_gp(state_telusur);
//				baca_gp(state_telusur);
//				gp_serong();
//				geser_kiri(state_telusur,1);
//				state_jalan=LURUSKAN_KIRI;
//				next_state_jalan=999; //berhenti deteksi error 2220
//			}
//			break;
//
//		case 2373: //acuan case 7048 9003
//			led_api_on;
//			status_tpa=1;
//			PID_tpa();
//			if(PID_tpa()==1)
//			{
//				padamkan_api();
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				state_jalan=2374; //3251
//			}
//			break;
//
//		case 2374: //acuan case 7049 3251
//			if (deteksi_api(4000) == 1)
//			{
//				state_telusur=0;
//				baca_gp(state_telusur);
//				gp_serong();
//				state_jalan = 2370;//7045;//balik ke awal, padamin pake lagoritma lama
//				break;
//			}
//			else
//			{
//				led_api_off;
//				if(ruang_api==5 && status_tpa==1)
//				{
//					mataangin = SELATAN;
//					state_jalan =LURUSKAN_CMPS;
//					next_state_jalan = 999; //berhenti dulu belum diset
//					state_telusur = 0;
//					konversi_gp(0);
//				}
//			}
//			break;
//
//		case 2375: //acuan case 7050 4045 3260 2107
//			baca_gp(state_telusur);
//			maju(state_telusur);
//			if(sensor_gp_asli[0]<75)
//			{
//				lock=0;
//				konversi_state_telusur(-1);
//				state_jalan = 2376;//7051;
//			}
//			break;
//
//		case 2376: //acuan case 7051 4046 3261 98
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 2377;//7052;
//
//			break;
//
//		case 2377: //acuan case 7052 3262 99 keluar 1B
//			telusurKanan_zigzag_ruang();
//			if (baca_garis() == 1)
//			{
//				penanda_buzzer = 0;
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//				setKompas();
//				state_jalan=2385;//7053; ke ruang III konfig bisa 1A atau 1B
//				lock_acuan = tembak_cmps();
//				ruangArr[6] = 5; //1B
//			}
//			break;
//
//
//			//=====1A
//		case 2378: //acuan case 2196 2106 konfig 1A
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			maju(state_telusur);
//			if (sensor_gp_asli[1] < thd_dinding) //4
//			{
//				lock = 0;
//				jalan_langkah(state_telusur, 2); //tadi 1
//				penanda_buzzer = 0;
//
//				state_jalan = 2379;//2197; //2272
//			}
//			break;
//
//		case 2379: //acuan case 2197 2272
//			konversi_state_telusur(-1); //ingin mepet ke ruang 2
//			//state_jalan = LURUSKAN_CMPS;
//			state_jalan = 2380;//2198; //2273
//			break;
//
//		case 2380: //acuan case 2198 2273 konfig 1A
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 2381;//2199;// 2274;
//			break;
//
//		case 2381: //acuan case 2199 2274 konfig 1A
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			maju(state_telusur);
//			if (sensor_gp_asli[0] < 100) //1
//			{
//				lock=0;
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 2385;//2200; //2112 ingin ke ruang III
//				ganti_kiri=0;
//			}
//			break;
//
//		case 2385: //ingin ke ruang 4 diulang seperti dari keluar ruang start
//			state_jalan = 2112;
//			break;
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//	case 528:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			if(sensor_gp_asli[3]>thd_dinding)
//			{
//				ruang_start = 4;
//				ganti_muka = 0;
//				state_jalan = 527;
//			}
//			else
//			{
//				ruang_start = 5;
//				ganti_muka = 0;
//				state_jalan = 6;
//			}
//		}
//		break;
//	case 527:
//		telusurKanan_zigzag();
//		if(ganti_muka == 1 && baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			konversi_state_telusur(1);
//			state_jalan = 1011;
//			ganti_muka = 0;
//		}
//		else if(ganti_muka == 3 && baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			state_jalan = 1110;
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//		}
//		break;
//	case 668:
//		telusurKiri_zigzag();
//		if (ganti_kiri == 2)
//		{
//			state_jalan = 669;
//		}
//		break;
//	case 669:
//		if(gunakan_cmps == 1)
//		{
//			luruskan_cmps();
//			if(sudah_lurus == 1)
//			{
//				state_jalan = 3;
//				//jalan_langkah(state_telusur,1);
//				ganti_kiri = 2;
//				ganti_kanan = 1;
//			}
//		}
//		else if(gunakan_cmps == 0)
//		{
//			state_jalan = 3;
//			//jalan_langkah(state_telusur,1);
//			ganti_kiri = 2;
//			ganti_kanan = 1;
//		}
//		break;
//	case 700:
//		if(gunakan_cmps == 1)
//		{
//			tembak_cmps();
//			luruskan_cmps();
//			if(sudah_lurus == 1)
//			{
//				state_jalan = 12;
//				lock_state_telusur = state_telusur;
//			}
//		}
//		else if(gunakan_cmps == 0)
//		{
//			state_jalan = 12;
//		}
//		break;
//	case 664:
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			maju(state_telusur);
//			if(sensor_gp_asli[0]<120)
//			{
//				konversi_state_telusur(1);
//				konversi_gp(state_telusur);
//				baca_gp(state_telusur);
//				gp_serong();
//				state_jalan = LURUSKAN_KIRI;
//				next_state_jalan = 542;
//			}
//		break;
//	case 666:
//		state_telusur = lock_state_telusur;
//		if(gunakan_cmps == 1)
//		{
//			luruskan_cmps();
//			if(sudah_lurus==1)
//			{
//				setKompas();
//				tembak_cmps();//mulai lock acuan kompas
//				lock_acuan = tembak_cmps();//lock
//				hitung_langkah = 0;
//				state_jalan = 520;
//				lock_state_telusur = state_telusur;
//			}
//		}
//		else if(gunakan_cmps == 0)
//		{
//			hitung_langkah = 0;
//			lock_state_telusur = state_telusur;
//			state_jalan = 542;
//		}
//		break;
//	case 542:
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		if(penanda_keluar_4a_4b_1b == 1)
//		{
//			if(sensor_gp_asli[0]<100)
//			{
//				lock=0;
//				state_jalan = 3;
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//			}
//		}
//		else
//		{
//			if(hitung_langkah == 3) //ini awalnya 3 dirubah menjadi 2
//			{
//				gerakStatic(0, 0, 0, 0, 0, 0, 25);
//				lock=0;
//				hitung_langkah = 0;
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				state_jalan = 2;
//			}
//		}
//		break;
//	case 520:
//		baca_gp(state_telusur);
//		gp_serong();
//		konversi_gp(state_telusur);
//		if(sensor_gp[sensor[0]]<100 && (sensor_gp[sensor[1]]>500||sensor_gp[sensor[7]]>500))
//		{
//			konversi_state_telusur(-1);
//			geser_kiri(state_telusur,2);
//			state_jalan = 542;
//		}
//		else if(sensor_gp[sensor[2]]<400 && sensor_gp[sensor[1]]>400)//di garis ruang
//		{
//			geser_kiri(state_telusur,2);
//			if(sensor_gp[sensor[0]]<500)
//			{
//				penanda_keluar_4a_4b_1b = 1;
//				//geser_kiri(state_telusur,1);
//				state_jalan = 542;
//			}
//			else
//			{
//				penanda_keluar_1a = 1;
//				state_jalan = 542;
//			}
//		}
//		else if(sensor_gp[sensor[2]]<400)
//		{
//			state_jalan = 542;
//			hitung_langkah = 0;
//		}
//		else//cek terus
//		{
//			state_jalan=542;//masuk back up
//			hitung_langkah = 0;
//		}
//		break;
//	case 667:
//		luruskan_tembok_kanan();
//		if(lurus_tembok == 1)
//		{
//			setKompas();
//			tembak_cmps();//mulai lock acuan kompas
//			lock_acuan = tembak_cmps();//lock
//			//lock_state_telusur = state_telusur;//lock state
//			hitung_langkah = 0;
//			state_jalan = 542;
//		}
//		break;
//	case 777:
//		if(gunakan_cmps == 1)
//		{
//			luruskan_cmps();
//			if(sudah_lurus==1)
//			{
//				state_jalan = 2;
//				//sudah_lurus = 0;
//			}
//		}
//		else if(gunakan_cmps == 0)
//		{
//			state_jalan = 2;
//		}
//		break;
//	case 4:
//		telusurKanan_zigzag();
//		if (ganti_muka == 2&&baca_garis() == 1) //start=2, posisi sekarang 1b
//		{
//			//kirim_case();
//			pewaktu=0;
//			state_jalan = LURUSKAN_CMPS;
//			lock_state_telusur = state_telusur;
//			next_state_jalan = 544;
//		}
//		else if (ganti_muka == 3) //start=3, posisi sekarang=depan pintu 2
//		{
//			penanda_buzzer = 0;
//			ruang_start = 3; //start = 3, posisi sekarang = 2
//			konversi_state_telusur(-1);
//			state_jalan = 90;
//		}
//		break;
//	case 544:
//		penanda_buzzer = 0;
//		penanda_pintu1=1;
//		ruang_start = 2; //start = 2, posisi sekarang = 1b
//		jalan_langkah(state_telusur,2);
//		konversi_state_telusur(-1);
//		ganti_muka = 0;
//		state_jalan = 1110;
//		break;
//	case 5:
//		//telusurKanan_zigzag();
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		detek_anjing();
//		//if(detek_anjing()==1){state_jalan=999;}
//		if (hitung_langkah < 4 && detek_anjing()==1) //ada anjing
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			delay_ms(100);
//			ruang_start = 6;
//			konversi_state_telusur(-1);
//			state_jalan = 1124;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			hitung_langkah = 0;
//			posisi_anjing = 2;
//		}
//		if (hitung_langkah < 4&&baca_garis() == 1) //5
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer = 0;
//			setKompas();
//			ruang_start = 2; //start = 2, posisi sekarang = 1a
//			ruangScan = 1;
//			ganti_muka = 0;
//			konversi_loncat = -2;
//			langkahScan = 5;
//			hitung_langkah = 0;
//			state_jalan = TELUSURSCAN_KANAN;
//			next_state_jalan = 112;//2105
//			konversi_scan = 1;
//		}
//		if (hitung_langkah == 4)
//		{
//			penanda_buzzer = 0;
//			ruang_start = 6;
//			konversi_state_telusur(-1);
//			state_jalan = 123;
//			ganti_muka = 0;
//		}
//		break;
//	case 6:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = TELUSURSCAN_KIRI; //102; //jalan lagi dari awal
//			ruangScan = 3;
//			langkahScan = 3;
//			next_state_jalan = 2103;
//			ganti_muka = 0;
//			konversi_scan = -1;
//			konversi_loncat = -2;
//		}
//		break;
//	case 7:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			while (baca_garis() == 1)telusurKiri_zigzag();
//			hitung_zigzag = 0;
//			state_jalan = 2006;
//		}
//		break;
//	case 551:
//		maju(state_telusur);
//		penanda_pintu1=0;
//		if (hitung_langkah == 1)//anti detect diruang 2
//		{
//			penanda_buzzer = 0;
//			konversi_gp(state_telusur);
//			jalan_langkah_scan12(state_telusur, 2);
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 201;
//				ruang_api = 1;
//			}
//			else if(eksistansi_api2 == 1)
//			{
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-1);
//				jalan_langkah(state_telusur, 1);
//				jalan_langkah_scan1(state_telusur, 2);
//				if (eksistansi_api1 == 1)
//				{
//					penanda_buzzer = 0;
//					konversi_state_telusur(-2);
//					jalan_langkah(state_telusur, 3);
//					state_jalan = 201;
//					ruang_api = 1;
//				}
//				else
//				{
//					eksistansi_api2 = 0;
//					eksistansi_api1 = 0;
//					konversi_state_telusur(-2); //-2
//					jalan_langkah(state_telusur, 3);
//					state_jalan = next_state_jalan;
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//				}
//			}
//			else if(hitung_langkah<=5&&sensor_gp[sensor[0]]<120)
//			{
//				//geser_kiri(state_telusur,2);
//				konversi_state_telusur(-1);
//				hitung_langkah = 0;
//				state_jalan = 114;
//			}
//			else
//			{
//				konversi_state_telusur(1); //-2
//				state_jalan = next_state_jalan;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//		}
//		break;
//	case 9005://scan tpa 1a
//		muter_acuankompas(acuan_1a);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=9002;
//		}
//		break;
//	case 549:
//		telusurKiri_zigzag_ruang();
//
//		if (uv1_on)eksistansi_api1 = 1;
//
////		if(ganti_muka>=1)
////		{
////			konversi_state_telusur(konversi_loncat);
////			state_jalan = next_state_jalan;
////			ganti_muka = 0;
////			hitung_langkah = 0;
////			hitung_zigzag = 0;
////		}
////		else if(sensor_gp[sensor[0]]<100)
////		{
////			konversi_state_telusur(konversi_loncat+1);
////			state_jalan = next_state_jalan;
////			ganti_muka = 0;
////			hitung_langkah = 0;
////			hitung_zigzag = 0;
////		}
//
//		if(hitung_langkah < langkahScan)
//		{
//			if(eksistansi_api1 == 1)
//			{
//				state_jalan = 201;
//				ruang_api = ruangScan;
//			}
//			else if(sensor_gp[sensor[0]]<100)//edit 25-04-2018
//			{
//				geser_kanan(state_telusur,2);
//				jalan_langkah_scan12(state_telusur,2);
//				if(eksistansi_api1 == 1)
//				{
//					ruang_api = ruangScan;
//					state_jalan = 201;
//				}
//				else
//				{
//					state_jalan = next_state_jalan;
//					konversi_state_telusur(konversi_scan);
//					ganti_muka = 0;
//					hitung_langkah = 0;
//					hitung_zigzag = 0;
//				}
//			}
//		}
//		else if(hitung_langkah == langkahScan)
//		{
//			if(eksistansi_api1 == 1)
//			{
//				state_jalan = 201;
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				state_jalan = next_state_jalan;
//				konversi_state_telusur(konversi_scan);
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//		}
//
//		break;
//	case 550:
//		telusurKiri_zigzag_ruang();
//
//		if (uv2_on)eksistansi_api2 = 1;
//		if (uv1_on)eksistansi_api1 = 1;
//
//		if(ganti_muka>=1)
//		{
//			konversi_state_telusur(konversi_loncat);
//			state_jalan = next_state_jalan;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//		else if(sensor_gp[sensor[0]]<100)//edit 25-04-2018
//		{
//			konversi_state_telusur(konversi_loncat+1);
//			state_jalan = next_state_jalan;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			hitung_zigzag = 0;
//		}
//
//		if(ruangScan == 3)
//		{
//			if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//			{
//				konversi_state_telusur(1);
//				//jalan_langkah(state_telusur,2);
//				state_jalan = 136;
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				state_jalan = next_state_jalan;
//				konversi_state_telusur(konversi_scan);
//				ganti_muka = 0;
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//			}
//		}
//		else
//		{
//			if(hitung_langkah < langkahScan)
//			{
//				if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//				{
//					//jalan_langkah(state_telusur,3);
//					state_jalan = 201;
//					ruang_api = ruangScan;
//				}
//			}
//			else if(hitung_langkah == langkahScan)
//			{
//				if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//				{
//					//jalan_langkah(state_telusur,3);
//					state_jalan = 201;
//					ruang_api = ruangScan;
//				}
//				else
//				{
//					state_jalan = next_state_jalan;
//					konversi_state_telusur(konversi_scan);
//					ganti_muka = 0;
//					hitung_langkah = 0;
//					hitung_zigzag = 0;
//				}
//			}
//		}
//		break;
//	case 136:
//			maju(state_telusur);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			if(sensor_gp[sensor[0]]<100||sensor_gp_asli[0]<70)
//			{
//				state_jalan=137;
//			}
//			break;
//		case 137:
//			if (uv2_on)eksistansi_api2 = 1;
//			if (uv1_on)eksistansi_api1 = 1;
//			if(eksistansi_api2 == 1||eksistansi_api1 == 1)
//			{
//				konversi_state_telusur(-2);
//				state_jalan = 138;
//				api3=1;
//				ruang_api = ruangScan;
//			}
//			else
//			{
//				api3=0;
//				konversi_state_telusur(-2);
//				state_jalan = 138;
//			}
//			break;
//		case 138:
//			maju(state_telusur);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			gp_serong();
//			if(sensor_gp[sensor[0]]<100||sensor_gp_asli[0]<70)
//			{
//				if(api3==1)
//				{
//					state_jalan=201;
//				}
//				else
//				{
//					state_jalan = next_state_jalan;
//					konversi_state_telusur(konversi_scan);
//					ganti_muka = 0;
//					hitung_langkah = 0;
//					hitung_zigzag = 0;
//				}
//			}
//			break;
//	case 9006://scan tpa 1b
//		muter_acuankompas(acuan_1b);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=9001;
//		}
//		break;
//	case 9007://scan tpa 3_4a
//		muter_acuankompas(acuan_3_4a);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=9001;
//		}
//		break;
	case 545:
		if(gunakan_cmps == 1)
		{
			luruskan_cmps();
			if(sudah_lurus == 1)
			{
				state_jalan = next_state_jalan;
				ganti_muka = 0;
			}
		}
		else if(gunakan_cmps == 0)
		{
			state_jalan = next_state_jalan;
			ganti_muka = 0;
		}
		break;
//	case 2006:
//		telusurKiri_zigzag();
//		if (hitung_zigzag == 3) //dari 1a sampai di ruang II
//		{
//			penanda_buzzer = 0;
//			jalan_langkah_scan12(state_telusur, 2);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,2);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;  //padamkan api di ruang II
//				ruang_api = 2;
//			}
//			else
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 2007;
//				ganti_muka = 0;
//			}
//			//deteksi_ruang_masuk(0,2);
//		}
//		break;
//	case 2007:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_khusus_2 = 1;
//			hitung_zigzag = 0;
//		}
//		if (penanda_khusus_2 == 1&&hitung_zigzag == 3)
//		{
//			state_jalan = 2106;
//			konversi_state_telusur(-1);
//			penanda_khusus_2 = 0;
//		}
//		break;
//	//case 8-11 untuk jalan dari IVb ke III
//	case 8:
//		telusurKiri_zigzag();
//		baca_gp(state_telusur);
////		if (hitung_zigzag==3)
////		{
////			if (sensor_gp_asli[0]<320) //terdapat anjing di 2
////			{
////				posisi_anjing=2;
////				konversi_state_telusur(-1);
////				ganti_muka=-1;
////				state_jalan=133;
////			}
////		}
//		if (hitung_zigzag == 5)
//		{
//			state_jalan = 9;
//			konversi_state_telusur(1);
//		}
//		break;
//	case 9:
//		//serong(state_telusur, 1);
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<70)
//		{
//			konversi_state_telusur(-1);
//			ganti_muka=0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 10;
//		}
//		break;
//	case 10:
//		telusurKanan_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 5) //sebelumnya ini
//		{
//			ganti_muka = 0;
//			mataangin=TIMUR;
//			state_jalan = LURUSKAN_CMPS;
//			state_telusur=2;
//			next_state_jalan = 4124;
//		}
////		if(ganti_kanan==3)
////		{
////			//gerakStatic(0,0,0,0,0,0,30);
////			konversi_state_telusur(-2); //sebelumnya-1
////			state_jalan=11;
////			ganti_muka=0;
////		}
////		else if (ganti_kiri==1)
////		{
////			//konversi_state_telusur(-1); //sebelumnya-1
////			state_jalan=11;
////			ganti_muka=0;
////		}
//		break;
//	case 4124:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<70)
//		{
//			state_jalan = 4125;
//			konversi_state_telusur(1);
//			ganti_muka=0;
//		}
//		break;
//	case 4125:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<70)
//		{
//			konversi_state_telusur(-1);
//			ganti_muka=0;
//			state_jalan = 11;
//		}
//		break;
//	case 11:
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 2011;
//		}
//		break;
//	case 2011:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 2012;
//		}
//		break;
//	case 2012:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = TELUSURSCAN_KIRI; //102; //jalan lagi dari awal
//			ruangScan = 3;
//			langkahScan = 3;
//			next_state_jalan = 2103;
//			ganti_muka = 0;
//			konversi_scan = -1;
//			konversi_loncat = -2;
//		}
//		break;
//
//	case 12:
//		//telusurKiri_zigzag_ruang();
//		//maju(state_telusur);
//		//baca_gp(state_telusur);
//		//if (baca_garis() == 1)
//		//{
//			//penanda_buzzer = 0;
//			jalan_langkah(state_telusur, 2); //untuk menentukan 1a/1b
//			baca_gp(state_telusur);
//			if (sensor_gp_asli[1] < 300) //sampai 1a dari 4a
//			{
//				konversi_state_telusur(1);
//				state_jalan = 1011;
//				ganti_muka = 0;
//			}
//			else  //sampai 1b dari 4a
//			{
//				state_jalan = 1110;
//				konversi_state_telusur(-1);
//				ganti_muka = 0;
//			}
//		break;
//	case 1110:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			//geser_kanan(state_telusur,1);//penambahan
//			konversi_state_telusur(1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1 = 1;
//			state_jalan = 13;
//		}
//		break;
//	case 13:
//		muter_acuankompas(kalibrasi_barat);
//		if(sudah_lurus==1)
//		{
//			state_telusur = 2;
//			state_jalan = TELUSURSCAN_KIRI;
//			langkahScan = 2;
//			hitung_langkah = 0;
//			ruangScan = 5;
//			konversi_scan = -1;
//			konversi_loncat = -2;
//			next_state_jalan = 2107;
//		}
//		break;
//	case 1011:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		//telusurKanan_zigzag();
//		if (sensor_gp_asli[0]<100)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 1012;
//			hitung_langkah = 0;
//
//		}
//		break;
//	case 1012:
//		maju(state_telusur);
//		penanda_pintu1=0;
//		if (hitung_langkah == 1)//anti detect diruang 2
//		{
//			penanda_buzzer = 0;
//			konversi_gp(state_telusur);
//			jalan_langkah_scan12(state_telusur, 2);
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 201;
//				ruang_api = 1;
//			}
//			else if (eksistansi_api2 == 1)
//			{
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-1);
//				jalan_langkah(state_telusur, 2);
//				jalan_langkah_scan1(state_telusur, 2);
//				if (eksistansi_api1 == 1)
//				{
//					penanda_buzzer = 0;
//					konversi_state_telusur(-2);
//					jalan_langkah(state_telusur, 3);
//					state_jalan = 201;
//					ruang_api = 1;
//				}
//				else
//				{
//					if (ruang_start == 2)
//					{
//						eksistansi_api2 = 0;
//						eksistansi_api1 = 0;
//						konversi_state_telusur(1); //-2
//						//jalan_langkah(state_telusur, 3);
//						state_jalan = 96;
//						ganti_muka = 0;
//					}
//					else
//					{
//						eksistansi_api2 = 0;
//						eksistansi_api1 = 0;
//						konversi_state_telusur(1); //-2
//						//jalan_langkah(state_telusur, 3);
//						state_jalan = 96;//2092
//						ganti_muka = 0;
//						hitung_zigzag = 0;
//					}
//				}
//			}
//			else if(hitung_langkah<=5&&sensor_gp[sensor[0]]<120)
//			{
//				//geser_kiri(state_telusur,2);
//				konversi_state_telusur(-1);
//				hitung_langkah = 0;
//				state_jalan = 114;
//			}
//			else
//			{
//				if (ruang_start == 2)
//				{
//					konversi_state_telusur(1);
//					state_jalan = 96;
//					ganti_muka = 0;
//				}
//				else
//				{
//					konversi_state_telusur(1);
//					state_jalan = 96;
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//				}
//			}
//		}
//		else if (nilai_api > thd_api&&hitung_langkah<3)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			ruang_api = 1;
//		}
//		break;
//	case 509:
//		jalan_langkah_scan1(state_telusur,2);
//		if(eksistansi_api1 == 1)
//		{
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur,2);
//			ruang_api = 1;
//			state_jalan = 201;
//		}
//		else if(nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			ruang_api = 1;
//		}
//		else
//		{
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur,2);
//			ganti_muka = 0;
//			state_jalan = 2092;
//		}
//		break;
//	case 530:
//		state_jalan = LURUSKAN_KANAN;
//		next_state_jalan = 529;
//		break;
//	case 529:
//		konversi_state_telusur(-1);
//		jalan_langkah(state_telusur, 2);
//		jalan_langkah_scan1(state_telusur, 2);
//		if (eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 201;
//			ruang_api = 1;
//		}
//		else if (nilai_api > thd_api&&hitung_langkah<=4)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			ruang_api = 1;
//		}
//		else
//		{
//			if (ruang_start == 2)
//			{
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-2); //-2
//				jalan_langkah(state_telusur, 3);
//				state_jalan = 2105;
//				ganti_muka = 0;
//			}
//			else
//			{
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-2); //-2
//				jalan_langkah(state_telusur, 3);
//				state_jalan = 2092;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//		}
//	break;
//	//Case 3-11 Start dari Ruang III
//	case 89:
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 90;
//		}
//		break;
//	case 90:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 120)
//		{
//			//tembak_cmps();//edit
//			konversi_state_telusur(-1);
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 511;
//		}
//		break;
//	case 511:
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[0] > thd_dinding) //ada pintu di 1a
//		{
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			penanda_pintu1=0;
//			state_jalan = 2091;
//		}
//		else //tidak ada pintu di 1a
//		{
//			penanda_pintu1=1;
//			if (ruang_start == 2)
//			{
//				konversi_state_telusur(1);
//				ganti_muka = 0;
//				state_jalan = 95;
//				lock_state_telusur = state_telusur-1;
//				if(lock_state_telusur<0)lock_state_telusur+=4;
//			}
//			else
//			{
//				konversi_state_telusur(1);
//				ganti_muka = 0;
//				state_jalan = 92;
//				penanda_pintu1 = 1;
//			}
//		}
//		if (ruang_start == 6 || ruang_start == 1 || ruang_start == 5)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 92;
//		}
//		break;
//	case 2091:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1) //sampai di ruang 1a
//		{
//			//kirim_case();
//			pewaktu=0;
//			//while (baca_garis() == 1)telusurKanan_zigzag();
//			setKompas();
//			hitung_zigzag = 0;
//			hitung_langkah = 0;
//			state_jalan = 91;
//		}
//		break;
//	case 91:
//		//telusurKanan_zigzag();
//		maju(state_telusur);
//		penanda_pintu1=0;
//		if (hitung_langkah == 1)//anti detect diruang 2
//		{
//			penanda_buzzer = 0;
//			konversi_gp(state_telusur);
//			jalan_langkah_scan12(state_telusur, 2);
//			if (eksistansi_api1 == 1 || eksistansi_api2 == 1)
//			{
//				konversi_state_telusur(-1);
//				hitung_langkah = 0;
//				state_jalan = 114;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			else if (eksistansi_api2 == 1)
//			{
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-1);
//				jalan_langkah(state_telusur, 2);
//				jalan_langkah_scan1(state_telusur, 2);
//				if (eksistansi_api1 == 1)
//				{
//					penanda_buzzer = 0;
//					konversi_state_telusur(-2);
//					jalan_langkah(state_telusur, 3);
//					state_jalan = 201;
//					ruang_api = 1;
//				}
//				else
//				{
//					if (ruang_start == 2)
//					{
//						eksistansi_api2 = 0;
//						eksistansi_api1 = 0;
//						konversi_state_telusur(1); //-2
//						//jalan_langkah(state_telusur, 3);
//						state_jalan = 96;
//						ganti_muka = 0;
//					}
//					else
//					{
//						eksistansi_api2 = 0;
//						eksistansi_api1 = 0;
//						konversi_state_telusur(1); //-2
//						//jalan_langkah(state_telusur, 3);
//						state_jalan = 96;//2092
//						ganti_muka = 0;
//						hitung_zigzag = 0;
//					}
//				}
//			}
//			else if(hitung_langkah<=5&&sensor_gp[sensor[0]]<120)
//			{
//				//geser_kiri(state_telusur,2);
//				konversi_state_telusur(-1);
//				hitung_langkah = 0;
//				state_jalan = 114;
//			}
//			else
//			{
//				if (ruang_start == 2)
//				{
//					konversi_state_telusur(1);
//					state_jalan = 96;
//					ganti_muka = 0;
//				}
//				else
//				{
//					konversi_state_telusur(1);
//					state_jalan = 96;
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//				}
//			}
//		}
//		else if (nilai_api > thd_api&&hitung_langkah<3)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			ruang_api = 1;
//		}
//		break;
//	case 114:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<200)
//		{
//			state_jalan = 115;
//		}
//		else if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			ruang_api = 1;
//		}
//		break;
//	case 115:
//		jalan_langkah_scan12(state_telusur,1);
//		konversi_state_telusur(-2);
//		state_jalan = 116;
//		break;
//	case 116:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			if (eksistansi_api1 == 1 || eksistansi_api2 == 1)
//			{
//				penanda_buzzer = 0;
//				state_jalan = 201;
//				ruang_api = 1;
//			}
//			else
//			{
//				eksistansi_api2 = 0;
//				eksistansi_api1 = 0;
//				konversi_state_telusur(1); //-2
//				state_jalan = 97;//2092
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//		}
//		break;
//	case 96:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 97;
//		}
//		break;
//	case 97:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KIRI;
//		if(ruang_start==2)
//		{
//			next_state_jalan = 2105;
//		}
//		else next_state_jalan = 2092;
//		break;
//	case 508:
//		jalan_langkah_scan1(state_telusur,2);
//		if(eksistansi_api1 == 1)
//		{
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur,2);
//			ruang_api = 1;
//			state_jalan = 201;
//		}
//		else if(nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			ruang_api = 1;
//		}
//		else
//		{
//			if (ruang_start == 2)
//			{
//				konversi_state_telusur(1);
//				jalan_langkah(state_telusur,2);
//				state_jalan = 2105;
//				ganti_muka = 0;
//			}
//			else
//			{
//				konversi_state_telusur(1);
//				jalan_langkah(state_telusur,2);
//				state_jalan = 2092;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//		}
//		break;
//	case 2092:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //garis di 1a
//		{
//			//kirim_case();
//			pewaktu=0;
//			while (baca_garis() == 1)telusurKiri_zigzag();
//			state_jalan = 2093;
//			setKompas();
//		}
//		break;
//	case 2093:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //garis di 2
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer=0;
//			penanda_pintu1=0;
//			//while (baca_garis() == 1)telusurKiri_zigzag();
//			setKompas();
//			hitung_zigzag = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = 93;
//		}
//		break;
//	case 2094:
//		//telusurKiri_zigzag();
//		maju(state_telusur);
//		//if (hitung_zigzag==3) //dari 1a ke 2, scan di ruang 2 memang harus maju dikit
//		konversi_gp(state_telusur);
//		if (hitung_langkah == 3 || sensor_gp[sensor[0]]<100) //scan di ruang 2 memang harus maju dikit
//		{
//			//state_jalan = 999;
//			penanda_buzzer = 0;
//			jalan_langkah_scan12(state_telusur, 2);
//			//konversi_state_telusur(1);
//			//jalan_langkah_scan2(state_telusur,1);
//			//gerakStatic(0,0,0,0,0,0,30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,2);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(1);
//				state_jalan = 537;  //padamkan api di ruang II
//				//ruang_api = 2;
//				eksistansi_api2 = 0;
//				hitung_langkah = 0;
//			}
//			else
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 2095;
//				ganti_muka = 0;
//			}
//			//deteksi_ruang_masuk(0,2);
//		}
//
//		break;
//	case 537:
//		jalan_langkah(state_telusur,2);
//		jalan_langkah_scan12(state_telusur,2);
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 536;
//			ruang_api = 2;
//		}
//		else if(hitung_langkah<4&&sensor_gp[sensor[0]]<100)
//		{
//			geser_kanan(state_telusur,2);
//			state_jalan = 510;
//		}
//		else
//		{
//			//gerakStatic(0,0,0,0,0,0,30);
//			//state_jalan = 999;
//			baca_gp(state_telusur);
//			if(sensor_gp_asli[1]<sensor_gp_asli[3])
//			{
//				konversi_state_telusur(1);
//				lock_state_telusur = state_telusur;
//				state_jalan = 535;
//				keluar_kiri = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			else
//			{
//				konversi_state_telusur(-1);
//				lock_state_telusur = state_telusur - 2;
//				if(lock_state_telusur<0)lock_state_telusur+=4;
//				state_jalan = 535;
//				keluar_kanan = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//		}
//		break;
//	case 510:
//		jalan_langkah_scan12(state_telusur,2);
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 538;
//			ruang_api = 2;
//		}
//		else if(nilai_api<thd_api)
//		{
//			ruang_api = 2;
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//		}
//		else
//		{
//			baca_gp(state_telusur);
//			if(sensor_gp_asli[1]<sensor_gp_asli[3])
//			{
//				konversi_state_telusur(1);
//				lock_state_telusur = state_telusur;
//				state_jalan = 535;
//				keluar_kiri = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			else
//			{
//				konversi_state_telusur(-1);
//				lock_state_telusur = state_telusur - 2;
//				if(lock_state_telusur<0)lock_state_telusur+=4;
//				state_jalan = 535;
//				keluar_kanan = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//		}
//		break;
//	case 536:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			state_jalan = 201;
//		}
//		break;
//	case 535:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<100 || sensor_gp[sensor[0]]<100)
//		{
//			if(keluar_kiri == 1)
//			{
//				konversi_state_telusur(1);
//				state_jalan = 534;
//			}
//			else if(keluar_kanan == 1)
//			{
//				state_jalan = 2095;
//			}
//		}
//		break;
//	case 534:
//		telusurKiri_zigzag();
//		if(baca_garis()==1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			state_telusur = lock_state_telusur;
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur,2);
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 1);
//			state_jalan = LURUSKAN_KANAN;//95;
//			next_state_jalan = 2096;
//			ganti_muka = 0;
//
//		}
//		break;
//	case 2095:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			//while (baca_garis() == 1)telusurKanan_zigzag();
//			hitung_zigzag = 0;
//			state_jalan = 2096;
//		}
//		break;
//	case 2096:
//		telusurKanan_zigzag();
//		if (hitung_zigzag == 2)
//		{
//			hitung_langkah = 0;
//			state_jalan = 552;
//			ganti_muka = 0;
//			penanda_khusus_1a = 0;
//		}
//		break;
//	case 552:
//		luruskan_tembok_kanan();
//		if(lurus_tembok == 1)
//		{
//			state_jalan = 2206;
//			konversi_state_telusur(-1);
//		}
//		break;
//	case 92:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //sampai di ruang 2
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			//while (baca_garis() == 1)telusurKiri_zigzag();
//			hitung_zigzag = 0;
//			hitung_langkah =0;
//			state_jalan = 93;
//			setKompas();
//		}
//		break;
//	case 93:
//		//telusurKiri_zigzag();
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]]<100) //scan di ruang 2 memang harus maju dikit
//		{
//			//penanda_buzzer = 0;
//			jalan_langkah_scan12(state_telusur, 2);
//			//konversi_state_telusur(1);
//			//jalan_langkah_scan2(state_telusur,1);
//			//gerakStatic(0,0,0,0,0,0,30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,2);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(1);
//				state_jalan = 663;  //padamkan api di ruang II
//				//ruang_api = 2;
//				eksistansi_api2 = 0;
//				hitung_langkah = 0;
//			}
//			else
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 94;
//				ganti_muka = 0;
//			}
//			//deteksi_ruang_masuk(0,2);
//		}
//		break;
//	case 663:
//		konversi_gp(state_telusur);
//		jalan_langkah(state_telusur,2);
//		jalan_langkah_scan12(state_telusur,2);
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 538;
//			ruang_api = 2;
//		}
//		else if(sensor_gp[sensor[0]]<120)
//		{
//			geser_kanan(state_telusur,2);
//			state_jalan = 517;
//		}
//		else
//		{
//			//gerakStatic(0,0,0,0,0,0,30);
//			baca_gp(state_telusur);
//			if(sensor_gp_asli[1]<sensor_gp_asli[3])
//			{
//				konversi_state_telusur(1);
//				lock_state_telusur = state_telusur;
//				state_jalan = 540;
//				keluar_kiri = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			else
//			{
//				konversi_state_telusur(-1);
//				lock_state_telusur = state_telusur - 2;
//				if(lock_state_telusur<0)lock_state_telusur+=4;
//				state_jalan = 540;
//				keluar_kanan = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//		}
//		break;
//	case 517:
//		jalan_langkah_scan12(state_telusur,2);
//		if(eksistansi_api1 == 1 || eksistansi_api2 == 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 538;
//			ruang_api = 2;
//		}
//		else
//		{
//			baca_gp(state_telusur);
//			if(sensor_gp_asli[1]<sensor_gp_asli[3])
//			{
//				konversi_state_telusur(1);
//				lock_state_telusur = state_telusur;
//				state_jalan = 540;
//				keluar_kiri = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//			else
//			{
//				konversi_state_telusur(-1);
//				lock_state_telusur = state_telusur - 2;
//				if(lock_state_telusur<0)lock_state_telusur+=4;
//				state_jalan = 540;
//				keluar_kanan = 1;
//				ganti_muka = 0;
//				eksistansi_api1 = 0;
//				eksistansi_api2 = 0;
//			}
//		}
//		break;
//	case 540:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<100 || sensor_gp[sensor[0]]<100)
//		{
//			if(keluar_kiri == 1)
//			{
//				konversi_state_telusur(1);
//				state_jalan = 539;
//			}
//			else if(keluar_kanan == 1)
//			{
//				state_jalan = 94;
//			}
//		}
//		break;
//	case 539:
//		telusurKiri_zigzag();
//		if(baca_garis()==1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			state_telusur = lock_state_telusur;
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur,2);
//			konversi_state_telusur(-1);
//			if ((ruang_start == 6 || ruang_start == 1 || ruang_start == 5)&&penanda_pintu1 == 0)
//				{
//					state_jalan = 2096;
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//				}
//				else
//				{
//					state_jalan = LURUSKAN_KANAN;//95;
//					next_state_jalan = 95;
//					ganti_muka = 0;
//					jalan_langkah(state_telusur, 1);
//				}
//		}
//		break;
//	case 538:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			state_jalan = 201;
//		}
//	break;
//	case 94:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			//jalan_langkah(state_telusur, 1);
//			//while (baca_garis() == 1)telusurKanan_zigzag();
//			setKompas();
//			tembak_cmps();
//			lock_state_telusur = state_telusur;
//			if ((ruang_start == 6 || ruang_start == 5 )&&penanda_pintu1 == 0)
//			{
//				state_jalan = 2096;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//			if (ruang_start == 3 && penanda_pintu1 == 0)
//			{
//				state_jalan = 2096;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//			if ((ruang_start == 4 || ruang_start == 7) &&penanda_pintu1 == 0)
//			{
//				state_jalan = 2096;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//			if (ruang_start == 1)
//			{
//				state_jalan = 2096;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//			if (ruang_start == 6 || ruang_start == 5 )
//			{
//				state_jalan = 2096;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//			if(penanda_pintu1 == 1)
//			{
//				state_jalan = 95;
//				ganti_muka = 0;
//				jalan_langkah(state_telusur, 2);
//			}
//		}
//		break;
//	case 95:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1&&ganti_muka>1) //sampai ruang 1b
//		{
//			//kirim_case();
//			pewaktu=0;
//			//state_telusur = lock_state_telusur;
//			state_jalan = 660;
//		}
//		else if(baca_garis()==1&&ganti_muka<=1)//sampai ruang 1a
//		{
//			//kirim_case();
//			pewaktu=0;
//			state_jalan = 91;
//			ganti_muka = 0;
//			hitung_zigzag = 0;
//		}
//		break;
//	case 660:
//		if(gunakan_cmps == 1)
//		{
//			luruskan_cmps();
//			if(sudah_lurus == 1)
//			{
//				state_jalan = 559;
//				hitung_langkah = 0;
//				ganti_muka = 0;
//			}
//		}
//		else if(gunakan_cmps == 0)
//		{
//			state_jalan = 559;
//			hitung_langkah = 0;
//			ganti_muka = 0;
//		}
//		break;
//	case 559:
//		penanda_buzzer = 0;
//		if(ruang_start==5||ruang_start==6)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 560;
//			ganti_muka = 0;
//		}
//		else
//		{
//			penanda_buzzer = 0;
//			jalan_langkah(state_telusur,2);
//			konversi_state_telusur(-1);
//			state_jalan = 1110;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//		}
//		break;
//	case 560:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<80 || sensor_gp[sensor[0]]<70)
//		{
//			geser_kanan(state_telusur,2);
//			state_jalan = 2107;
//		}
//		break;
//	case 102:
//		penanda_buzzer = 0;
//		jalan_langkah_scan12(state_telusur, 3);
//		//gerakStatic(0,0,0,0,0,0,30);
//		if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			//jalan_langkah(state_telusur,2);
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(-1);
//			state_jalan = 201;  //padamkan api di ruang III
//			ruang_api = 3;
//		}
//		else
//		{
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(-1);
//			state_jalan = 2103;
//			ganti_muka = 0;
//		}
//		break;
//	case 2103:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			while (baca_garis() == 1)telusurKanan_zigzag();
//			ganti_muka = 0;
//			state_jalan = 89; //tadi diganti disini
//			setKompas();
//		}
//		break;
//	case 103:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_khusus_2api = 1;
//			ganti_muka = 0;
//		}
//		if (penanda_khusus_2api == 1&&hitung_zigzag > 4&&ganti_muka == 1) //sampai di ruang II
//		{
//			penanda_buzzer = 0;
//			jalan_langkah_scan12(state_telusur, 1);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,2);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(1);
//				state_jalan = 201;  //padamkan api di ruang II
//				ruang_api = 2;
//			}
//			else
//			{
//				penanda_khusus_2api = 0;
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(1);
//				state_jalan = 2104;
//				ganti_muka = 0;
//			}
//			//deteksi_ruang_masuk(0,2);
//		}
//		break;
//	case 2104:
//		telusurKiri_zigzag();
//		if (hitung_zigzag > 3&&ganti_muka == 1)
//		{
//			state_jalan = 4105;
//		}
////		if (baca_garis()==1)
////		{
////			jalan_langkah(state_telusur,1);
////			konversi_state_telusur(1);
////			state_jalan=104;
////			ganti_muka=0;
////		}
//		break;
//	case 4105:
//		konversi_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z)
//		{
//			//konversi_state_telusur(-1);
//			state_jalan = 104;
//			ganti_muka = 0;
//		}
//
//		break;
//	case 104:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1&&penanda_khusus_2 == 0)
//		{
//			//kirim_case();
//			pewaktu=0;
//			while (baca_garis() == 1)telusurKanan_zigzag();
//			hitung_zigzag = 0;
//			penanda_khusus_2 = 1;
//		}
////		if (ganti_muka==1&&baca_garis()==1&&hitung_zigzag>3&&penanda_khusus_2==1) // sampai di ruang IA
//		if (ganti_muka < 2&&baca_garis() == 1&&penanda_khusus_2 == 1) // sampai di ruang IA
//		{
//			pewaktu=0;
//			penanda_buzzer = 0;
//			penanda_khusus_2 = 0;
//			jalan_langkah_scan12(state_telusur, 3);
//			if (eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 201;
//				ruang_api = 1;
//			}
//			else if (eksistansi_api2 == 1)
//			{
//				eksistansi_api1 = 0;
//				konversi_state_telusur(-1);
//				jalan_langkah(state_telusur, 2);
//				jalan_langkah_scan1(state_telusur, 2);
//				if (eksistansi_api1 == 1)
//				{
//					penanda_buzzer = 0;
//					konversi_state_telusur(-2);
//					jalan_langkah(state_telusur, 3);
//					konversi_state_telusur(1);
//					state_jalan = 201;
//					ruang_api = 1;
//				}
//				else
//				{
//					eksistansi_api2 = 0;
//					eksistansi_api1 = 0;
//					konversi_state_telusur(-2);
//					jalan_langkah(state_telusur, 3);
//					state_jalan = 2105;
//					ganti_muka = 0;
//					hitung_zigzag = 0;
//				}
//			}
//			else
//			{
//				konversi_state_telusur(1);
//				state_jalan = 2105;
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//		}
//		else if (ganti_muka > 2&&baca_garis() == 1) // sampai di ruang IB
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer = 0;
//			penanda_khusus_2 = 0;
//			jalan_langkah_scan12(state_telusur, 3);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,2);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 5;
//			}
//			else
//			{
//				//jalan_langkah(state_telusur,1);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 2107;
//				ganti_muka = 0;
//			}
//		}
//		break;
//	case 112:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 113;
//		}
//		break;
//	case 113:
//		konversi_gp(state_start);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KIRI;
//		next_state_jalan = 2105;
//		break;
//	case 2105:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			hitung_zigzag = 0;
//			penanda_khusus_1a = 1;
//		}
//		if (hitung_zigzag == 2&&penanda_khusus_1a == 1)
//		{
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(1);
//			state_jalan = 2106;
//			ganti_muka = 0;
//			penanda_khusus_1a = 0;
//		}
//		break;
//	case 2206:
//			//konversi_state_telusur(-1);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			maju(state_telusur);
//			if (sensor_gp_asli[1] < thd_dinding) //4
//			{
//				//penanda_buzzer = 0;
//				jalan_langkah(state_telusur, 2); //tadi 1
//				if (ruang_start == 4 || ruang_start == 7)
//				{
//					penanda_buzzer = 0;
//					//gerakStatic(0,0,0,0,0,0,30);
//					konversi_state_telusur(-1);
//					state_jalan = 2108;
//				}
//				else
//				{
//					//gerakStatic(0,0,0,0,0,0,30);
//					konversi_state_telusur(1);
//					state_jalan = 100;
//					ganti_muka = 0;
//				}
//			}
//			break;
//	case 100:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 101;
//		}
//		break;
//	case 101:
//		konversi_gp(state_start);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KANAN;
//		next_state_jalan = 105;
//		break;
//	case 2106:
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[1] < thd_dinding) //4
//		{
//			//penanda_buzzer = 0;
//			jalan_langkah(state_telusur, 2); //tadi 1
//			if (ruang_start == 4 || ruang_start == 7)
//			{
//				penanda_buzzer = 0;
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 2108;
//			}
//			else
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(1);
//				state_jalan = 100;
//				ganti_muka = 0;
//			}
//		}
//		break;
//	case 2107:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 98;
//		}
//		break;
//	case 98:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KANAN;
//		next_state_jalan = 99;
//		break;
//	case 99:
//		aktivasi_count1B = 0;
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			lock_state_telusur = state_telusur;
//			pewaktu=0;
//		}
//		if (ganti_muka == 1&&(ruang_start == 4 || ruang_start == 7))
//		{
//			penanda_buzzer = 0;
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(-1);
//			aktivasi_count1B = 0;
//			state_jalan = 2108;
//		}
//		if (ganti_muka == 2) //deteksi perempatan
//		{
//			penanda_buzzer = 0;
//			state_telusur = lock_state_telusur - 2;
//			if(state_telusur<0)state_telusur+=4;
//			state_jalan = 106;
//			aktivasi_count1B = 0;
//		}
//		break;
//	case 661:
//		luruskan_tembok_kanan();
//		if(lurus_tembok == 1)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 2107;
//		}
//		break;
//	case 105:
//		telusurKanan_zigzag_ruang();
//		if (ganti_muka == 1) //deteksi perempatan
//		{
//			penanda_buzzer = 0;
//			state_jalan = 106;
//		}
//		break;
//	case 2108:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			state_jalan = 102;
//			setKompas();
//		}
//		break;
//	case 106:
//		gp_serong();
//		baca_gp(state_telusur);
//		telusurKanan_zigzag_ruang();
//		if((sensor_gp[sensor[3]]<300 || sensor_gp_asli[0]<300)&&hitung_zigzag>1)
//		{
//			konversi_state_telusur(1);
//			//state_jalan = 557;
//
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 4126;
//		}
//		break;
//	case 4126:
//		if(mataangin == TIMUR)
//		{
//			state_telusur = 0;
//		}
//		else if(mataangin == UTARA)
//		{
//			state_telusur = 1;
//		}
//		else if(mataangin == SELATAN)
//		{
//			state_telusur = 3;
//		}
//		else if(mataangin == BARAT)
//		{
//			state_telusur = 2;
//		}
//
//		state_jalan = 4127;
//		break;
//	case 4127:
//		gp_serong();
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		if(sensor_gp[sensor[3]]<300 || sensor_gp_asli[0]<300)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 557;
//		}
//		else
//		{
//			state_jalan = 4128;
//
//		}
//		break;
//	case 4128:
//		perempatan=20;
//		gp_serong();
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp[sensor[3]]<300 || sensor_gp_asli[0]<300)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 557;
//		}
//		break;
//	case 557:
//		perempatan=0;
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			//kirim_case();
//			state_jalan = 558;
//			pewaktu = 0;
//			konversi_state_telusur(-1);
//		}
//		break;
//	case 558:
//		luruskan_tembok_kanan();
//		if(lurus_tembok == 1)
//		{
//			if(sensor_gp_asli[3] > 450)
//			{
//				penanda_buzzer = 0;
//				state_jalan = 107; //deteksi ada pintu ruang IV A
//				konversi_state_telusur(-1);
//				hitung_langkah = 0;
//			}
//			else
//			{
//				//konversi_state_telusur(-1);
//				hitung_langkah = 0;
//				hitung_zigzag = 0;
//				state_jalan = 662;
//				ganti_muka = 0;
//			}
//		}
//		else if(pewaktu>75)
//		{
//			jalan_langkah(state_telusur,1);
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 558;
//			//kirim_case();
//			pewaktu = 0;
//		}
//		break;
//	case 662:
////		telusurKiri_zigzag();
////		if(hitung_zigzag<1&&ganti_muka>=1)
////		{
//			if (sensor_gp_asli[0] < 450) //Ada anjing di I
//			{
//				penanda_buzzer = 0;
//				posisi_anjing = 1;
//				gerak(0, 0, 0, 0, 0);
//				delay_ms(100);
//				konversi_state_telusur(-1);
//				//jalan_langkah(state_telusur,2);
//				state_jalan = 109; //tidak ada pintu di ruang IV A
//				count_penanda_anjing1 = 0;
//			}
//			else if (sensor_gp_asli[0] > 450)
//			{
//				state_jalan = 110; //Tidak ada anjing di I
//				konversi_state_telusur(-1);
//				ganti_muka = 0;
//				hitung_zigzag = 0;
//			}
//		//}
//		break;
//	case 107: //sampai di ruang IVa
//		maju(state_telusur);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer = 0;
//			jalan_langkah_scan12(state_telusur, 2);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				ruang_api = 4;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//			}
//			else
//			{
//				jalan_langkah(state_telusur, 1);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 121; //tidak ada api di IVa dan menuju ke 3
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//			}
//		}
//		else if (hitung_langkah > 6) //jika terjadi nyangkut di IVa
//		{
////			konversi_state_telusur(-1);
////			jalan_langkah(state_telusur, 2);
////			konversi_state_telusur(1);
////			state_jalan = 2608;
////			hitung_langkah = 0;
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 543;
//		}
//		break;
//	case 543:
//		konversi_state_telusur(-1);
//		jalan_langkah(state_telusur, 2);
//		konversi_state_telusur(1);
//		state_jalan = 2608;
//		hitung_langkah = 0;
//		break;
//	case 2608:
//		maju(state_telusur);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer = 0;
//			jalan_langkah_scan12(state_telusur, 2);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 4;
//			}
//			else
//			{
//				jalan_langkah(state_telusur, 1);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 121; //tidak ada api di IVa dan menuju ke 3
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//			}
//		}
//		else if (hitung_langkah > 5)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 2609;
//		}
//		break;
//	case 2609:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 4;
//			}
//			else
//			{
//				jalan_langkah(state_telusur, 1);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 121; //tidak ada api di IVa dan menuju ke 3
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//			}
//		}
//		break;
//	case 108:
//		serong(state_telusur, -1);
//		if (sensor_gp[sensor[6]] < setPoint0z + 40)
//		{
//			state_jalan = 110;
//			ganti_muka = 0;
//		}
//		break;
//	case 109:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1) //sampai di ruang IVB dengan anjing I
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			//gerakStatic(0,0,0,0,0,0,30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 7;
//			}
//			else
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 131;
//			}
//		}
//		break;
//	case 1111:
//		jalan_langkah(state_telusur, 1);
//		konversi_state_telusur(-1);
//		ganti_muka = 1;
//		state_jalan = 110;
//		break;
//	case 110:
//		telusurKiri_zigzag();
//		baca_gp(state_telusur);
//		if (ganti_muka == 2&&sensor_gp_asli[0] < 200&&hitung_zigzag < 8) //ada anjing posisi II
//		{
//			penanda_buzzer = 0;
//			posisi_anjing = 2;
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(-2);
//			state_jalan = 2111;
//		}
//		if (baca_garis() == 1) //sampai di ruang IVB dengan posisi Anjing III
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			posisi_anjing = 3;
//			jalan_langkah_scan12(state_telusur, 2);
//			//gerakStatic(0,0,0,0,0,0,30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 7;
//			}
//			else
//			{
//				jalan_langkah(state_telusur, 1);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 2110; //pulang dari IVB ke 3 dengan anjing 3
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//			}
//		}
//		break;
//	case 2111:
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		//if (sensor_gp[sensor[2]]>300)
//		if (sensor_gp_asli[1] > 400)count_penanda_dinding2++;
//		else count_penanda_dinding2 = 0;
//
//		if (count_penanda_dinding2 > 10)
//		{
//			//jalan_langkah(state_telusur,1);
//			konversi_state_telusur(1);
//			state_jalan = 111;
//			count_penanda_dinding2 = 0;
//		}
//		break;
//	case 2110:
//		telusurKanan_zigzag();
//		if(baca_garis()==1)
//		{
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = 4220;
//		}
////		if(ganti_kanan==5)
////		{
////			konversi_state_telusur(-2); //menuju ke ruang 3
////			ganti_muka=0;
////			state_jalan=122;
////		}
////		else if (ganti_kiri==2)
////		{
////			//konversi_state_telusur(-2); //menuju ke ruang 3
////			ganti_muka=0;
////			state_jalan=122;
////		}
//		break;
//	case 4220:
//		if(mataangin == TIMUR)
//		{
//			state_telusur = 3;
//		}
//		else if(mataangin == UTARA)
//		{
//			state_telusur = 0;
//		}
//		else if(mataangin == SELATAN)
//		{
//			state_telusur = 2;
//		}
//		else if(mataangin == BARAT)
//		{
//			state_telusur = 1;
//		}
//
//		state_jalan = 4221;
//		break;
//	case 4221:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<70)
//		{
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = 4222;
//		}
//		break;
//	case 4222:
//		telusurKiri_zigzag();
//		if (ganti_muka == 2)
//		{
//			state_jalan = 4223;
//		}
//		break;
//	case 4223:
//		baca_gp(state_telusur);
//		telusurKiri_zigzag();
//		if(hitung_zigzag == 5)
//		{
//			state_jalan = 4224;
//			konversi_state_telusur(1);
//		}
//		break;
//	case 4224:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<70)
//		{
//			konversi_state_telusur(-1);
//			ganti_muka=0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 4225;
//		}
//		break;
//	case 4225:
//		telusurKanan_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 5) //sebelumnya ini
//		{
//			ganti_muka = 0;
//			mataangin=TIMUR;
//			state_jalan = LURUSKAN_CMPS;
//			state_telusur=2;
//			next_state_jalan = 4122;
//		}
//		break;
//	case 111:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1) //sampai di Ruang IVB dengan posisi Anjing II
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			posisi_anjing = 2;
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 7;
//				break;
//			}
//			else
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 131; //pulang dari IV B anjing II ke III
//			}
//		}
//		break;
//	case 121://dari IVa menuju ke 3
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<75)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 117;
//		}
//		break;
//	case 117:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = LURUSKAN_KANAN;
//		next_state_jalan = 118;
//		break;
//	case 118:
//		telusurKanan_zigzag();
//		if(baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer=0;
//		}
//		if (ganti_muka >= 2&&hitung_zigzag == 5)
//		{
//			//konversi_state_telusur(-1);
//			ganti_muka = 0;
//			mataangin=TIMUR;
//			state_jalan = LURUSKAN_CMPS;
//			state_telusur=2;
//			next_state_jalan = 4122;
//		}
//		break;
//	case 665:
//		telusurKanan();
//		if (baca_garis() == 1) //sampai IVa dengan anjing 3 start 1c
//		{
//			//kirim_case();
//			pewaktu=0;
//			perubahan_mataangin(SEARAH_JARUM_JAM,2);
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 512;
//			if(mataangin==UTARA)state_telusur=2;
//			else if(mataangin==BARAT)state_telusur=1;
//			else if(mataangin==TIMUR)state_telusur=3;
//			else if(mataangin==SELATAN)state_telusur=0;
//		}
//		break;
//	case 512:
//		penanda_buzzer = 0;
//		jalan_langkah_scan12(state_telusur, 2);
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			//jalan_langkah(state_telusur,3);
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(-1);
//			state_jalan = 201;
//			ruang_api = 4;
//		}
//		else
//		{
//			jalan_langkah(state_telusur,2);
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(-1);
//			//state_jalan = 121; //tidak ada api di IVa dan menuju ke 3
//			state_jalan = 4121;
//			aktivasi_count1B = 1;
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//	case 4121:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<70)
//		{
//			state_jalan = 121;
//			ganti_muka=0;
//		}
//		break;
//	case 4122:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<70)
//		{
//			konversi_state_telusur(1);
//			konversi_gp(state_telusur);
//			ganti_muka=0;
//			pewaktu = 0;
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 122;
//		}
//		break;
//	case 122:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<70)
//		{
//			konversi_state_telusur(-1);
//			ganti_muka=0;
//			state_jalan = 4123;
//		}
//		break;
//	case 4123:
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 2122;
//		}
//		break;
//	case 2122:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			if (ruang_start == 3)
//			{
//				ganti_muka = 0;
//				konversi_state_telusur(-1);
//				state_jalan = 89;
//			}
//			else
//			{
//				konversi_state_telusur(1);
//				state_jalan = LURUSKAN_KIRI;
//				next_state_jalan = 2123;
//			}
//		}
//		break;
//	case 2123:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1) //sampai di III
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = TELUSURSCAN_KIRI; //102; //jalan lagi dari awal
//			ruangScan = 3;
//			langkahScan = 3;
//			next_state_jalan = 2103;
//			ganti_muka = 0;
//			konversi_scan = -1;
//			konversi_loncat = -2;
//		}
//		break;
//	case 123:
//		telusurKiri_zigzag(); //dari ruang 1c
//		baca_gp(state_telusur);
//		if (baca_garis() == 1 && ganti_muka<=3) //sampai ruang IVb dengan anjing 1/3
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			//gerakStatic(0,0,0,0,0,0,30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				ruang_api = 7;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//			}
//			else
//			{
//				jalan_langkah(state_telusur, 1);
//				konversi_state_telusur(-1);
//				state_jalan = 2124;
//				ganti_muka = 0;
//			}
//		}
//		else if(ganti_muka>4 && baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			state_jalan = 128;
//		}
//
//		if (ganti_muka == 2 && hitung_zigzag<=7)
//		{
////			state_jalan = 999;
//			//baca_gp(state_telusur);
//			if (sensor_gp_asli[0] < 300&&sensor_gp[sensor[6]] < 400) //anjing yes, pintu no
//			{
//				penanda_buzzer = 0;
//				state_jalan = 127;
//				gerakStatic(0,0,0,0,0,0,30);
//				delay_ms(100);
//				posisi_anjing = 3;
//			}
////			else if (sensor_gp_asli[0] > 400&&sensor_gp[sensor[6]] < 400) //anjing no, pintu no
////			{
////				//penanda_buzzer=0;
////				state_jalan = 128;
////				posisi_anjing = 1;
////			}
//		}
//		break;
//	case 9008://scan tpa 4b
//		muter_acuankompas(acuan_4b);
//		if(sudah_lurus==1)
//		{
//			pewaktu=0;
//			state_jalan=9001;
//		}
//		break;
//	case 2124:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1) //detect garis 4b
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 2125;
//			ganti_muka = 0;
//		}
//		break;
//	case 2125:
//		telusurKanan_zigzag();
//		baca_gp(state_telusur);
//		if (ganti_muka == 1&&hitung_zigzag < 3)
//		{
//			if (sensor_gp_asli[0] < thd_dinding)count_penanda_anjing3++; //ada anjing di 3
//			else count_penanda_anjing3 = 0;
//			if (count_penanda_anjing3 > 2)
//			{
//				count_penanda_anjing3 = 0;
//				posisi_anjing = 3;
//				konversi_state_telusur(1);
//				state_jalan = 2126;
//				ganti_muka = 0;
//			}
//		}
//		if (ganti_muka == 4)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 134; //menuju ruang III
//		}
//		break;
//	case 2126:
//		telusurKiri_zigzag();
//		if (ganti_muka == 2)
//		{
//			state_jalan = 8; //sama seperti dari 4b jika ada anjing 3
//		}
//		break;
//	case 1124:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp_asli[0] < 120 || sensor_gp[sensor[0]]<100)
//		{
//			//jalan_langkah(state_telusur, 2);
//			//konversi_state_telusur(1);
//			state_jalan = 124;
//			konversi_state_telusur(-1);
//			hitung_langkah = 0;
//		}
////		else if(hitung_langkah>3)
////		{
////			geser_kiri(state_telusur,1);
////			if(sensor_gp_asli[3]<thd_dinding)
////			{
////				if(sensor_gp_asli[1]<thd_dinding)
////				{
////					state_jalan = 516;
////					konversi_state_telusur(1);
////				}
////				else
////				{
////					konversi_state_telusur(-2);
////					state_jalan	= LURUSKAN_KANAN;
////					next_state_jalan = 515;
////				}
////			}
////			else
////			{
////				state_jalan = 1124;
////				hitung_langkah = 2;
////			}
////		}
//		break;
//	case 516:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<200)
//		{
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			perubahan_mataangin(LAWAN_JARUM_JAM,1);
//			state_jalan = 124;
//		}
//		break;
//	case 515:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[3]<thd_dinding)
//		{
//			jalan_langkah(state_telusur,1);
//			konversi_state_telusur(-1);
//			state_jalan = 516;
//		}
//		break;
//	case 124:
//		//telusurKanan_zigzag();
//		telusurKanan();
//		if (hitung_langkah<10&&baca_garis()==1)
//		{ //sampai ruang IVA dengan anjing II
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			perubahan_mataangin(SEARAH_JARUM_JAM,2);
//			if(abs(nilai_error_cmps())>15)
//			{
//				state_jalan = LURUSKAN_CMPS;
//				next_state_jalan = 514;
//				hitung_langkah = 0;
//				if(mataangin==UTARA)state_telusur=2;
//				else if(mataangin==BARAT)state_telusur=1;
//				else if(mataangin==TIMUR)state_telusur=3;
//				else if(mataangin==SELATAN)state_telusur=0;
//			}
//			else
//			{
//				if(mataangin==UTARA)state_telusur=2;
//				else if(mataangin==BARAT)state_telusur=1;
//				else if(mataangin==TIMUR)state_telusur=3;
//				else if(mataangin==SELATAN)state_telusur=0;
//				state_jalan = 514;
//			}
//
//		}
//		else if(hitung_langkah>=13)
//		{
//			perubahan_mataangin(SEARAH_JARUM_JAM,2);
//			state_jalan = 1125;
//		}
//		break;
//	case 514:
//		jalan_langkah_scan12(state_telusur, 3);
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//		{
//			penanda_buzzer = 0;
//			//jalan_langkah(state_telusur,3);
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(-1);
//			state_jalan = 201;
//			ruang_api = 4;
//		}
//		else
//		{
//			jalan_langkah(state_telusur, 3);
//			eksistansi_api1 = 0;
//			eksistansi_api2 = 0;
//			aktivasi_count1B == 1;
//			//gerakStatic(0,0,0,0,0,0,30);
//			konversi_state_telusur(-1);
//			state_jalan = 121; //tidak ada api di IVa dan menuju ke 3
//			ganti_muka = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//	case 1125:
//		telusurKanan_zigzag();
//		//telusurKanan();
//		if(baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 7;
//			}
//			else
//			{
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 131; //pulang dari IVb dengan anjing !=3 ke 3
//			}
//		}
//		break;
//	case 125:
////		telusurKanan_zigzag();
////		if (ganti_muka == 2)
////		{
//			baca_gp(state_telusur);
//			state_jalan = 665;
////		}
//		break;
//	case 126:
//		maju(state_telusur);
//		if (baca_garis() == 1) //sampai IVb dengan anjing 3/1 start 1c
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah_scan12(state_telusur, 2);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 7;
//			}
//			else
//			{
//				if (posisi_anjing == 3)
//				{
//					//jalan_langkah(state_telusur,1);
//					//gerakStatic(0,0,0,0,0,0,30);
//					konversi_state_telusur(-1);
//					state_jalan = 2110; //pulang dari IVB ke 3 dengan anjing 3
//					ganti_muka = 0;
//
//				}
//				else
//				{
//					//gerakStatic(0,0,0,0,0,0,30);
//					konversi_state_telusur(-1);
//					state_jalan = 131; //pulang dari IVb dengan anjing !=3 ke 3
//				}
//			}
//
//		}
//		break;
//	case 127:
//		luruskan_tembok_kiri();
//		if(lurus_tembok == 1)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 125;
//			ganti_muka = 0;
//			hitung_zigzag = 0;
//		}
//		break;
//	case 128:
////		telusurKiri_zigzag();
////		if (baca_garis() == 1) //sampai ruang IVa dengan anjing I dari 1c
////		{
//			penanda_buzzer = 0;
//			jalan_langkah_scan12(state_telusur, 2);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			if (eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//jalan_langkah(state_telusur,3);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 201;
//				ruang_api = 4;
//			}
//			else
//			{
//				jalan_langkah(state_telusur,2);
//				//gerakStatic(0,0,0,0,0,0,30);
//				konversi_state_telusur(-1);
//				state_jalan = 121; //tidak ada api di IVa dan menuju ke 3
//				ganti_muka = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//			}
//		//}
//		break;
////	case 2131:
////		jalan_langkah(state_telusur,3);
////		state_jalan=2133;
////		ganti_muka=0;
////		break;
////	case 2133:
////		break;
//
//	case 131: //Return IVB ke III
//		//if (posisi_anjing==1||posisi_anjing==2)state_jalan=132;
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 1133;
//			ganti_muka = 0;
//		}
//		break;
//	case 132:
//		jalan_langkah(state_telusur, 3);
//		state_jalan = 1133;
//		ganti_muka = 0;
//		break;
//	case 1133: //untuk membedakan, ini start bukan dari 4b
//		telusurKanan_zigzag();
//		if (ganti_muka == 4)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 134;
//		}
//		break;
//	case 133: //untuk membedakan, ini start dari 4b
//		telusurKanan_zigzag();
//		baca_gp(state_telusur);
//		if (ganti_muka == 1&&hitung_zigzag<6&&hitung_zigzag>3) // u/ pengecekan kembali anjing 3
//		{
//			if (sensor_gp_asli[0] < 400) //ada anjing di 3 , thd sebelumnya 400
//			{
//				posisi_anjing = 3;
//				penanda_buzzer = 0;
//				konversi_state_telusur(1);
//				state_jalan = 2133;
//				ganti_muka = 0;
//			}
//		}
//		if (ganti_muka == 4)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 134;
//		}
//		break;
//	case 2133:
//		telusurKiri_zigzag();
//		if (ganti_muka == 2)
//		{
//			state_jalan = 8;
//			hitung_zigzag = 0;
//		}
//		break;
//	case 134:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			if (ruang_start == 3)
//			{
//				ganti_muka = 0;
//				konversi_state_telusur(-1);
//				state_jalan = 89;
//			}
//			else
//			{
//				konversi_state_telusur(1);
//				state_jalan = LURUSKAN_KIRI;
//				next_state_jalan = 135;
//			}
//		}
//		break;
//	case 135:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)//sampai di Ruang III
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			jalan_langkah(state_telusur,2);
//			ganti_muka = 0;
//			hitung_langkah = 0;
//			state_jalan = TELUSURSCAN_KIRI; //102; //jalan lagi dari awal
//			ruangScan = 3;
//			langkahScan = 3;
//			next_state_jalan = 2103;
//			ganti_muka = 0;
//			konversi_scan = -1;
//			konversi_loncat = -2;
//		}
//		break;
//	case 533:
//
//		break;
//	case 5000: //untuk eksekusi saat lost acuan
//		maju(state_telusur);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//		}
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[0] < 120)
//		{
//			state_jalan = 5001;
//		}
//		break;
//	case 5001:
//		baca_gp(0);
//		gp_serong();
//		konversi_gp(0);
//		gerak(0, 0, 9, 1, 25);
//		if (arah_deteksi_lost == 1)
//		{
//			if (sensor_gp_asli[1]<150&&sensor_gp[sensor[1]]>100)
//			{
//				penanda_buzzer = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				ganti_muka = 0;
//				state_telusur = 1;
//				state_jalan = state_kembalidarilost;
//				setKompas();
//			}
//		}
//		else if (arah_deteksi_lost == 0)
//		{
//			if (sensor_gp_asli[3]<150&&sensor_gp[sensor[7]]>100)
//			{
//				penanda_buzzer = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				ganti_muka = 0;
//				state_telusur = 3;
//				state_jalan = state_kembalidarilost;
//				setKompas();
//			}
//		}
//
//
//		break;
//	case 201: //201 khusus madamin api
//		//led_api_on;
//		statusKompas = 1;
//		if (ruang_api == 1)
//		{
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			if(ruang_start==2)
//			{
//				jalan_langkah(state_telusur,2);
//				state_jalan = 9005;
//			}
//			else state_jalan = 9005; //khusus ruang 1a
//		}
//		else if (ruang_api == 5)
//		{
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			if(ruang_start==2)
//			{
//				jalan_langkah(state_telusur,2);
//				state_jalan = 9006;
//			}
//			else state_jalan = 9006; //khusus ruang 1b
//
//		}
//		else if(ruang_api == 3)
//		{
//			jalan_langkah(state_telusur,2);
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 9007;
//		}
//		else
//		{
//			state_jalan = 2220; //selain 1a/1b
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//	case 2210:
//		gp_serong();
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2210);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(-1);//jatah diedit
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			state_jalan = 2211;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			penanda_api_1 = 0;
//		}
//		if(sensor_gp[sensor[2]]<200&&sensor_gp[sensor[7]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[1]]<150)
//		{
//			geser_kiri(state_telusur,1);
//		}
//		break;
//	case 2211:
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2211);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(-1);
//			state_jalan = 2212;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			penanda_api_1 = 1;
//		}
//		break;
//	case 2212:
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2212);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur, 5);
//			//konversi_state_telusur(-1);
//			state_jalan = 2213;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			penanda_api_1 = 1;
//		}
//		break;
//	case 2213:
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2213);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
////			konversi_state_telusur(-2);
////			jalan_langkah(state_telusur,3);
////			state_jalan=2210; //telusur kiri api IA
////			ganti_kanan=0;
////			ganti_kiri=0;
//			ruang_api=0;
//			state_jalan = 2214; //telusur kiri api IA 2092
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			ganti_muka = 0;
//			eksistansi_api1 = 0;
//			eksistansi_api2 = 0;
//			led_api_off;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			penanda_api_1 = 0;
//		}
//		break;
//	case 2214:
//		if(ruang_start==2)
//		{
//			jalan_langkah(state_telusur,2);
//			konversi_state_telusur(1);
//			state_jalan = 2106;
//		}
//		else {
//			state_jalan = 2093;
//		}
//		break;
//	case 2220:
//		gp_serong();
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2220);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			state_jalan = 2221;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//		}
//		break;
//	case 2221:
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2221);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer=0;
//			pewaktu=0;
////			konversi_state_telusur(-2);
////			jalan_langkah(state_telusur,3);
////			konversi_state_telusur(-1);
////			state_jalan=2220; //telusur kanan selain IA dan IB
////			ganti_kanan=0;
////			ganti_kiri=0;
//			state_jalan = 2222; //telusur kanan selain IA dan IB
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			ganti_muka = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//		}
//		break;
//	case 2222:
//		eksistansi_api1 = 0;
//		eksistansi_api2 = 0;
//		led_api_off;
//		if (ruang_api == 2&&penanda_pintu1 == 0)
//		{
//			state_jalan = 2095;
//			ruang_api = 0;
//		}
//		else if (ruang_api == 2&&penanda_pintu1 == 1)
//		{
//			state_jalan = 94;
//			ruang_api = 0;
//		}
//		else if (ruang_api == 3)
//		{
//			state_jalan = 89;
//			ruang_api = 0;
//		}
//		else if (ruang_api == 4)
//		{
//			state_jalan = 121;
//			ganti_muka = 1;
//			ruang_api = 0;
//		}
//		else if (ruang_api == 7)
//		{
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 2125;
//			ganti_muka = 0;
//			ruang_api = 0;
//		}
//		break;
//	case 2230:
//		gp_serong();
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2230);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			penanda_buzzer = 0;
//			pewaktu=0;
//			//konversi_state_telusur(1);
//			state_telusur = lock_state_telusur;
////			jalan_langkah(state_telusur, 2);
////			konversi_state_telusur(1);
////			jalan_langkah(state_telusur, 3);
////			konversi_state_telusur(-1);
//			state_jalan = 556;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			penanda_api_1 = 1;
//		}
//		if (ganti_kanan >= 1)
//		{
//			ganti_kiri = 0;
//			ganti_kanan = 0;
//		}
//		if (ganti_kiri >= 4) //lost
//		{
//			mataangin = TIMUR;
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 505;
//
//		}
//		if(sensor_gp[sensor[6]]<200&&sensor_gp[sensor[1]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[7]]<150)
//		{
//			geser_kanan(state_telusur,1);
//		}
//		break;
//	case 505:
//		baca_gp(0);
//		maju(0);
//		if(sensor_gp_asli[0]<100)
//		{
//			ganti_muka = 0;
//			ganti_kiri = 0;
//			ganti_kanan = 0;
//			state_jalan = 2231;
//		}
//		else if(nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//		}
//		break;
//	case 556:
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[0]<100)
//		{
//			konversi_state_telusur(1);
//			state_jalan = LURUSKAN_KIRI;
//			next_state_jalan = 2231;
//			jalan_langkah(state_telusur,2);
//		}
//		break;
	case 555:
		luruskan_tembok_kanan();
		if(lurus_tembok == 1 || pewaktu>40)
		{
			state_jalan = next_state_jalan;
			hitung_langkah = 0;
			hitung_zigzag = 0;
			ganti_muka = 0;
			ganti_kiri = 0;
			ganti_kanan = 0;
			pewaktu = 0;
		}
		break;
	case 554:
		luruskan_tembok_kiri();
		if(lurus_tembok == 1 || pewaktu>40)
		{
			state_jalan = next_state_jalan;
			hitung_langkah = 0;
			hitung_zigzag = 0;
			ganti_muka = 0;
			ganti_kiri = 0;
			ganti_kanan = 0;
			pewaktu = 0;
		}
		break;
//	case 2231:
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2231);
//		if (baca_garis() == 1 && ganti_muka>1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			state_jalan = 2232;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			penanda_api_1 = 0;
//		}
//		break;
//	case 2232:
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2232);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 3);
//			konversi_state_telusur(1);
//			state_jalan = 2233;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		if (nilai_api > thd_api)
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			penanda_api_1 = 0;
//		}
//		break;
//	case 2233:
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2233);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
////			konversi_state_telusur(-2);
////			jalan_langkah(state_telusur,3);
////			state_jalan=2230; //telusur kanan khusus IB
////			ganti_kanan=0;
////			ganti_kiri=0;
//			state_jalan = 2107;
//			ganti_muka = 1;
//			ruang_api = 0;
//			ruang_api = 0;
//			led_api_off;
//		}
//		if (nilai_api > thd_api) //245
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//			penanda_api_1 = 1;
//		}
//		break;
//
//	case 202:
//		led_api_on;
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 203;
//			api_siap = 0;
//		}
//		break;
//	case 203:
//		padamkan_api();
//		state_jalan = 204;
//		eksistansi_api2 = 0;
//		eksistansi_api1 = 0;
//		status_api = 1;
//		gerakStatic(0, 0, 0, 0, 0, 0, 30);
//		statusKompas = 0;
//		break;
//	case 204:
//		if (deteksi_api(4000) == 1)
//		{
//			state_jalan = 2201;
//			break;
//		}
//		else
//		{
//			led_api_off;
//			if(ruang_api == 1 && gunakan_cmps == 1)
//			{
//				mataangin = UTARA;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 400;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//			else if(ruang_api == 5 && gunakan_cmps == 1)
//			{
//				mataangin = SELATAN;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 410;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//			else if(ruang_api == 2&&gunakan_cmps == 1)
//			{
//				mataangin = BARAT;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 2205;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//			else if(ruang_api == 3 &&gunakan_cmps == 1)
//			{
//				mataangin = BARAT;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 498;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//			else if(ruang_api == 7)
//			{
//				mataangin = TIMUR;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 2205;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//			else if(ruang_api == 4)
//			{
//				mataangin = BARAT;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 2205;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//			else
//			{
//				state_jalan = 2205; //2205
//				konversi_gp(0); //3
//				state_telusur = 0;
//			}
//
//		}
//		break;
//	case 400:
//		maju(0);
//		gp_serong();
//		konversi_gp(0);
//		if (sensor_gp[sensor[0]] < 120)
//		{
//			konversi_gp(0);
//			baca_gp(0);
//			if(sensor_gp[sensor[1]] > 250)
//			{
//				geser_kanan(0,1);
//				//kirim_case();
//				pewaktu = 0;
//				state_jalan = 400;
//			}
//			else
//			{
//				//kirim_case();
//				state_jalan = 2305;
//				pewaktu = 0;
//			}
//		}
//		break;
//	case 401:
//		maju(0);
//		gp_serong();
//		konversi_gp(0);
//		if (sensor_gp[sensor[0]] < 120)
//		{
//			konversi_gp(0);
//			state_jalan = 402;
//		}
//		break;
//	case 402:
//		gerak(0, 0, 9, 1, 25);
//		gp_serong();
//		baca_gp(0);
//		if (sensor_gp_asli[1]<150&&sensor_gp[sensor[1]]>100)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 403;//209;
//			state_telusur = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			setKompas();
//		}
//		break;
//	case 403:
//		gp_serong();
//		baca_gp(state_telusur);
//		telusurKanan_zigzag_ruang();
//		if(baca_garis() == 1)
//		{
//			pewaktu=0;
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			penanda_buzzer = 0;
//			state_jalan=210;
//			ganti_muka = 0;
//			setKompas();
//		}
//	case 404:
//		jalan_langkah(0,4);
//		konversi_state_telusur(1);
//		state_jalan = 2205;
//		break;
//	case 405:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		gp_serong();
//		if (sensor_gp[sensor[0]] < 120)
//		{
//			state_jalan = 406;
//		}
//		break;
//	case 406:
//		gerak(0, 0, 9, 1, 25);
//		gp_serong();
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[1]<150&&sensor_gp[sensor[1]]>100)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 210;//209;
//			state_telusur = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			setKompas();
//		}
//		break;
//	case 407:
//		maju(0);
//		gp_serong();
//		konversi_gp(0);
//		if (sensor_gp[sensor[0]] < 120)
//		{
//			konversi_gp(0);
//			state_jalan = 408;
//		}
//		break;
//	case 408:
//		gerak(0, 0, 9, 1, 25);
//		gp_serong();
//		baca_gp(0);
//		if (sensor_gp_asli[1]<150&&sensor_gp[sensor[1]]>100)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 409;//209;
//			state_telusur = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			setKompas();
//		}
//		break;
//	case 409:
//		telusurKanan_zigzag_ruang();
//		if(baca_garis() == 1)
//		{
//			pewaktu=0;
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			penanda_buzzer = 0;
//			state_jalan=210;
//			ganti_muka = 0;
//			setKompas();
//		}
//		break;
//	case 410:
//		maju(0);
//		gp_serong();
//		konversi_gp(0);
//		if (sensor_gp[sensor[0]] < 120)
//		{
//			konversi_gp(0);
//			state_jalan = 408;
//		}
//		if(baca_garis() == 1)
//		{
//			pewaktu=0;
//			konversi_state_telusur(2);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			penanda_buzzer = 0;
//			state_jalan=411;
//			ganti_muka = 0;
//			setKompas();
//		}
//		break;
//	case 411:
//		jalan_langkah(state_telusur,5);
//		state_jalan = 412;
//		break;
//	case 412:
//		geser_kanan(state_telusur,5);
//		konversi_state_telusur(2);
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		state_jalan = 410;
//		break;
//	case 2201:
//		PID_api();
//		if (PID_api() == 1)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 2202;
//			api_siap = 0;
//		}
//		break;
//	case 2202:
//		maju(0);
//		if (nilai_api > (thd_api + 5))
//		{
//			penanda_buzzer = 0;
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			state_jalan = 202;
//		}
//		break;
//	case 2205:
//		maju(0);
//		if (sensor_gp[sensor[0]] < 120)
//		{
//			konversi_gp(0);
//			baca_gp(0);
//			if (ruang_api==1||ruang_api==5)
//			{
//				//kirim_case();
//				pewaktu = 0;
//				state_jalan=2305;
//			}
//			else state_jalan = 205;
////			if (ruang_api == 1)
////			{
////				if (penanda_api_1 == 0)state_jalan = 2305;
////				else state_jalan = 2405;
////			}
////			else if (ruang_api == 5)
////			{
////				if (penanda_api_1 == 0)state_jalan = 2505;
////				else state_jalan = 205;
////			}
//		}
//		break;
//	case 2505: //khusus ruang api : 5 , posisi api : 0
//		gerak(0, 0, 9, 1, 25);
//		baca_gp(0);
//		gp_serong();
//		if (sensor_gp_asli[1]<150&&sensor_gp[sensor[1]]>100)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 2506;
//			state_telusur = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			setKompas();
//			state_telusur = 1;
//		}
//		break;
//	case 2506:
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2506);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(1);
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			state_jalan = 2507;
//		}
//		break;
//	case 2507:
//		telusurKanan_zigzag_ruang();
//		deteksi_lost_acuan(1, 2507);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			//while(baca_garis()==1)telusurKanan_zigzag_ruang();
//			//state_jalan=210;
//			ganti_muka = 0;
//			setKompas();
//
//			jalan_langkah(state_telusur, 2);
//			baca_gp(state_telusur);
//			if (sensor_gp_asli[1] > thd_dinding)
//			{
//				//konversi_state_telusur(-1);
//				state_jalan = 210;
//			}
//			else
//			{
//				konversi_state_telusur(-2);
//				state_jalan = 208;
//			}
//		}
//		break;
//	case 2405: //khusus ruang api : 1 , posisi api : 1
//		gerak(0, 0, -9, 1, 25);
//		gp_serong();
//		baca_gp(0);
//		if (sensor_gp_asli[3]<150&&sensor_gp[sensor[7]]>100)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 2406;
//			state_telusur = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			setKompas();
//			state_telusur = 3;
//		}
//		break;
//	case 2406: //lanjutan 2407
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2406);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur, 4);
//			konversi_state_telusur(-1);
//			state_jalan = 2407;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//		}
//		break;
//	case 2407: //lanjutan 2406
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2407);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			//while(baca_garis()==1)telusurKiri_zigzag_ruang();
//			//state_jalan=210;
//			ganti_muka = 0;
//			setKompas();
//
//			jalan_langkah(state_telusur, 2);
//			baca_gp(state_telusur);
//			if (sensor_gp_asli[1] > thd_dinding)
//			{
//				//konversi_state_telusur(-1);
//				state_jalan = 210;
//			}
//			else
//			{
//				konversi_state_telusur(-2);
//				state_jalan = 208;
//			}
//		}
//		break;
//	case 2305:
//		gerak(0, 0, -9, 1, 25);
//		gp_serong();
//		baca_gp(0);
//		if (sensor_gp_asli[3]<150&&sensor_gp[sensor[7]]>100)
//		{
//			tembak_cmps();
//			if(mataangin != BARAT)
//			{
//				mataangin = BARAT;
//				state_jalan = LURUSKAN_CMPS;
//				next_state_jalan = 2305;
//				//kirim_case();
//				pewaktu = 0;
//			}
//			else
//			{
//				penanda_buzzer = 0;
//				state_jalan = 541;//209;
//				state_telusur = 0;
//				ganti_kanan = 0;
//				ganti_kiri = 0;
//				setKompas();
//			}
//		}
//		else if(pewaktu>100)
//		{
//			mataangin = UTARA;
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 519;
//			state_telusur = 0;
//		}
//		break;
//	case 519:
//		geser_kanan(0,2);
//		konversi_gp(0);
//		state_jalan = 400;
//		break;
//	case 541:
//		telusurKiri_zigzag_ruang();
//		//deteksi_lost_acuan(0,541);
//		if (ganti_kanan >= 1)
//		{
//			ganti_kiri = 0;
//			ganti_kanan = 0;
//		}
//		if (ganti_kiri >= 4) //lost
//		{
//			mataangin = UTARA;
//			state_jalan =LURUSKAN_CMPS;
//			next_state_jalan = 400;
//			state_telusur = 0;
//			konversi_gp(0);
//		}
//		if(baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			//state_telusur = 0;
//			//jalan_langkah(state_telusur, 2);
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			penanda_buzzer = 0;
//			state_jalan=210;
//			ganti_muka = 0;
//			setKompas();
//		}
//		break;
//	case 205:
//		gerak(0, 0, 9, 1, 25);
//		gp_serong();
//		baca_gp(0);
//		if (sensor_gp_asli[1]<150&&sensor_gp[sensor[1]]>100)
//		{
//			penanda_buzzer = 0;
//			state_jalan = 206;
//			state_telusur = 0;
//			ganti_kanan = 0;
//			ganti_kiri = 0;
//			setKompas();
//			tembak_cmps();
//			state_telusur = 1;
//		}
//		break;
//	case 206:
//		telusurKanan_zigzag_ruang();
//		gp_serong();
//		deteksi_lost_acuan(1, 206);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			while (baca_garis() == 1)telusurKanan_zigzag_ruang();
//			//state_jalan=210;
//			ganti_muka = 0;
//			setKompas();
//
//			if (ruang_api == 5) //cari tahu apakah keluar di 1b
//			{
//				jalan_langkah(state_telusur, 2);
//				baca_gp(state_telusur);
//				if (sensor_gp_asli[1] > thd_dinding)
//				{
//					//konversi_state_telusur(-1);
//					state_jalan = 210;
//				}
//				else
//				{
//					konversi_state_telusur(-2);
//					state_jalan = 208;
//				}
//			}
//			else
//			{
//				state_jalan = 210;
//				ganti_muka = 0;
//				konversi_gp(state_telusur);
//			}
//		}
//		if(sensor_gp[sensor[2]]<200&&sensor_gp[sensor[7]]<120&&sensor_gp[sensor[0]]>setPoint0zr&&sensor_gp[sensor[1]]<150)
//		{
//			geser_kiri(state_telusur,1);
//		}
//		break;
//	case 207:
//		maju(state_telusur);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			konversi_state_telusur(-2);
//			state_jalan = 210;
//			ganti_muka = 0;
//			setKompas();
//		}
//		break;
//	case 208:
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 208);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			while (baca_garis() == 1)telusurKiri_zigzag_ruang();
//			state_jalan = 2209;
//		}
//		break;
//	case 2209:
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 2209);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			//while(baca_garis()==1)telusurKiri_zigzag_ruang();
//			jalan_langkah(state_telusur, 2);
//			state_jalan = 210;
//			ganti_muka = 0;
//			setKompas();
//		}
//		break;
//	case 209:
//		telusurKiri_zigzag_ruang();
//		deteksi_lost_acuan(0, 209);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			//while(baca_garis()==1)telusurKiri_zigzag_ruang();
//			//state_telusur = 0;
//			jalan_langkah(state_telusur, 2);
//			baca_gp(state_telusur);
//			penanda_buzzer = 0;
//			//state_jalan=210;
//			ganti_muka = 0;
//			setKompas();
//
//			//jalan_langkah(state_telusur,1);
//			if (sensor_gp_asli[1] > thd_dinding)
//			{
//				//konversi_state_telusur(-1);
//				state_jalan = 210;
//			}
//			else
//			{
//				konversi_state_telusur(-2);
//				state_jalan = 208;
//			}
//
//		}
//		break;
//	case 211:
//		gerak(0, 0, 7, 1, 25);
//		if (kompas <= setPointK + 1&&kompas >= setPointK - 1)
//		{
//			penanda_buzzer = 0;
//			konversi_state_telusur(-2);
//			state_jalan = 211;
//		}
//		break;
//	case 210:
//		if(ruang_api == 2 || ruang_api == 3)
//		{
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 532;
//		}
//		else if(ruang_api == 5)
//		{
//			gp_serong();
//			geser_kanan(state_telusur,2);
//			mataangin = BARAT;
//			state_jalan = LURUSKAN_CMPS;
//			state_telusur = 0;
//			konversi_gp(state_telusur);
//			next_state_jalan = 532;
//		}
//		else if(ruang_api == 1)
//		{
//			gp_serong();
//			konversi_gp(state_telusur);
//			if(sensor_gp[sensor[7]]<300)
//			{
//				state_jalan = LURUSKAN_KIRI;
//				next_state_jalan = 532;
//				state_telusur = 0;
//			}
//			else
//			{
//				mataangin = UTARA;
//				state_jalan = LURUSKAN_CMPS;
//				next_state_jalan = 400;
//			}
//
//		}
//		else
//		{
//			if(ruang_api == 4)
//			{
//				mataangin = UTARA;
//				state_jalan = LURUSKAN_CMPS;
//				state_telusur = 0;
//				next_state_jalan = 532;
//				konversi_gp(state_telusur);
//			}
//			else if(ruang_api == 7)
//			{
//				mataangin = SELATAN;
//				state_jalan = LURUSKAN_CMPS;
//				state_telusur = 0;
//				next_state_jalan = 532;
//				konversi_gp(state_telusur);
//			}
//		}
//
//		break;
//	case 532: //menentukan case pulang
//		/*
//		#return dari 3 ke 1 (3007)/
//		#return dari 3 ke 4A (3054)/
//		#return dari 3 ke 4B anjing1&2 (3084)/
//		#return dari 3 ke 4B anjing3 (3089)/
//		#return 1A Ke 2 (3014)/
//		#return 1A ke 3 (3015)/
//		#return 1A ke 4A (3069)/
//		#return 1A&B ke 4B anjing1&2 (3093)/
//		#return 1A ke 4B anjing3 (3096)/
//		#return 1B ke 4B anjing3 (3101)/
//		#return 1B ke 2 (3019)/
//		#return 1B ke 3 (3022)/
//		#return 1B ke 4A (3073)/
//		#return 4B anjing 1&3 ke 1 (3025)/
//		#return 4B anjing2 ke 1 (3028)/
//		#return 4B anjing 1&2 ke 3 (3034)/
//		#return 4B anjing3 ke 3 (3037)/
//		#return 4B ke 2 anjing1&2 (3102)/
//		#return 4B ke 2 anjing3 (3105)/
//		#return 4A anjing2&3 ke 1 (3045)/
//		#return 4A anjing1 ke 1 (3047)/
//		#return 4A ke 2 (3048)/
//		#return 4A ke 3 (3050)/
//		*/
//		statusKompas = 1;
//		if ((ruang_start == 1 || ruang_start == 5 || ruang_start == 6)&&ruang_api == 2)
//		{
//			state_jalan = 3000;
//		}
//		else if (ruang_start == 3&&ruang_api == 2)
//		{
//			state_jalan = 3002;
//		}
//		else if (ruang_start == 2&&ruang_api == 3)
//		{
//			state_jalan = 3006;
//		}
//		else if ((ruang_start == 1 || ruang_start == 5 || ruang_start == 6)&&ruang_api == 3)
//		{
//			state_jalan = 3007;
//		}
//		else if (ruang_start == 2&&ruang_api == 1)
//		{
//			state_jalan = 3014;
//		}
//		else if (ruang_start == 3&&ruang_api == 1)
//		{
//			state_jalan = 3015;
//		}
//		else if (ruang_start == 2&&ruang_api == 5)
//		{
//			state_jalan = 3019;
//		}
//		else if (ruang_start == 3&&ruang_api == 5)
//		{
//			state_jalan = 3022;
//		}
//		else if ((ruang_start == 1 || ruang_start == 5 || ruang_start == 6)&&ruang_api == 7&&(posisi_anjing == 1 || posisi_anjing == 3))
//		{
//			state_jalan = 3025;
//		}
//		else if ((ruang_start == 1 || ruang_start == 5 || ruang_start == 6)&&ruang_api == 7&&posisi_anjing == 0)
//		{
//			state_jalan = 3025;
//		}
//		else if ((ruang_start == 1 || ruang_start == 5 || ruang_start == 6)&&ruang_api == 7&&posisi_anjing == 2)
//		{
//			state_jalan = 3028;
//		}
//		else if (ruang_start == 3&&ruang_api == 7&&(posisi_anjing == 1 || posisi_anjing == 2))
//		{
//			state_jalan = 3034;
//		}
//		else if (ruang_start == 3&&ruang_api == 7&&posisi_anjing == 3)
//		{
//			state_jalan = 3037;
//		}
//		else if ((ruang_start == 1 || ruang_start == 5 || ruang_start == 6)&&ruang_api == 4&&posisi_anjing != 1)
//		{
//			state_jalan = 3001;
//		}
//		else if ((ruang_start == 1 || ruang_start == 5 || ruang_start == 6)&&ruang_api == 4&&posisi_anjing == 1)
//		{
//			state_jalan = 3013;
//		}
//		else if (ruang_start == 2&&ruang_api == 4)
//		{
//			state_jalan = 3012;
//		}
//		else if (ruang_start == 3&&ruang_api == 4)
//		{
//			state_jalan = 3050;
//		}
//		else if (ruang_start == 4&&ruang_api == 3)
//		{
//			state_jalan = 3054;
//		}
//		else if (ruang_start == 4&&ruang_api == 2)
//		{
//			state_jalan = 3058;
//		}
//		else if (ruang_start == 4&&ruang_api == 1)
//		{
//			state_jalan = 3069;
//		}
//		else if (ruang_start == 7&&ruang_api == 2&&posisi_anjing == 3)
//		{
//			state_jalan = 3071;
//		}
//		else if (ruang_start == 4&&ruang_api == 5)
//		{
//			state_jalan = 3073;
//		}
//		else if (ruang_start == 7&&ruang_api == 2&&posisi_anjing != 3)
//		{
//			state_jalan = 3075;
//		}
//		else if (ruang_start == 7&&ruang_api == 3&&posisi_anjing != 3)
//		{
//			state_jalan = 3084;
//		}
//		else if (ruang_start == 7&&ruang_api == 3&&posisi_anjing == 3)
//		{
//			state_jalan = 3089;
//		}
//		else if (ruang_start == 7&&(ruang_api == 1 || ruang_api == 5)&&posisi_anjing != 3)
//		{
//			state_jalan = 3093;
//		}
//		else if (ruang_start == 7&&(ruang_api == 1 || ruang_api == 5)&&posisi_anjing == 3)
//		{
//			state_jalan = 3096;
//		}
//		//return 4B ke 2 Anjing1&2
//		else if (ruang_start == 2&&ruang_api == 7&&posisi_anjing != 3)
//		{
//			state_jalan = 3102;
//		}
//		else if (ruang_start == 2&&ruang_api == 7&&posisi_anjing == 3)
//		{
//			state_jalan = 3105;
//		}
//		else
//		{
//			state_jalan = 999;
//		}
//		break;
//
//	case 3000://return dari 2 ke 1A dan 1B
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer = 0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3002://return dari 2 ke 3
//		jalan_langkah(state_telusur, 2);
//		konversi_state_telusur(-1);
//		hitung_langkah = 0;
//		state_jalan = 507;
//		break;
//	case 507:
//		baca_gp(state_telusur);//edit 25-04-2018
//		maju(state_telusur);
//		if(sensor_gp_asli[3]<250)
//		{
//			konversi_state_telusur(-1);
//			hitung_langkah = 0;
//			state_jalan = 3004;
//		}
//		else if(hitung_langkah > 3)
//		{
//			geser_kanan(state_telusur,1);
//			hitung_langkah = 0;
//			state_jalan = 507;
//		}
//		break;
//	case 3001:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 304;
//		}
//		break;
//	case 3003:
//		konversi_state_telusur(-1);
//		maju(state_telusur);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3004:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer = 0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3005:
//		telusurKanan_zigzag();
//		{
//			state_jalan = 3003;
//		}
//		break;
//	case 3006: //return dari 3 ke 2
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3007: //return dari 3 ke 1
//		ganti_muka = 0;
//		state_jalan = 3008;
//		break;
//	case 3008:
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 3009;
//		}
//		break;
//	case 3009:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3010:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3014://return 1A Ke 2
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3015://return 1A ke 3
//		jalan_langkah(state_telusur,2);
//		konversi_state_telusur(1);
//		hitung_langkah = 0;
//		state_jalan = 3086;
//		break;
//	case 3086:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[3] < setPoint2z + 190)
//		{
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 3018;
//		}
//		else if (hitung_langkah>6&&sensor_gp_asli[3]>setPoint2z + 200)
//		{
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 3086;
//			geser_kiri(state_telusur,1);
//			//state_jalan = 3086;
//			hitung_langkah = 0;
//		}
//		break;
//	case 3016:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 100)
//		{
//			ganti_muka = 0;
//			state_jalan = 3076;
//		}
//		break;
//	case 3017:
//		konversi_gp(state_telusur);
//		maju(state_telusur);
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[3] < 300)
//		{
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 3068;
//		}
//		break;
//	case 3018:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3019://return 1B ke 2
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z)
//		{
//			state_jalan = 3020;
//		}
//		break;
//	case 3020:
//		konversi_state_telusur(-1);
//		state_jalan = 3021;
//		break;
//	case 3021:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3022://return 1B ke 3
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint2z + 40)
//		{
//			state_jalan = 3023;
//		}
//		break;
//	case 3023:
//		konversi_state_telusur(1);
//		state_jalan = 3024;
//		break;
//	case 3024:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3025://return 4B anjing 1&3 ke 1
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z)
//		{
//			state_jalan = 3026;
//		}
//		break;
//	case 3026:
//		konversi_state_telusur(1);
//		state_jalan = 3027;
//		break;
//	case 3027:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3028: //4B anjing2 ke 1
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z)
//		{
//			state_jalan = 3029;
//		}
//		break;
//	case 3029:
//		konversi_state_telusur(-1);
//		state_jalan = 3030;
//		break;
//	case 3030:
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 3031;
//		}
//		break;
//	case 3031:
//		telusurKanan_zigzag();
//		if (ganti_muka == 6)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 3032;
//		}
//		break;
//	case 3032:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3033:
//		ganti_muka = 0;
//		state_jalan = 3038;
//		break;
//	case 3034: //return 4B anjing 1&2 ke 3
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			state_jalan = 3035;
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//		}
//		break;
//	case 3035:
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 3036;
//		}
//		break;
//	case 3036:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			state_jalan = 4013;
//		}
//		break;
//	case 4013:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3037://return 4B anjing3 ke 3
//		ganti_muka = 0;
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint2z + 40)
//		{
//			state_jalan = 3038;
//		}
//		break;
//	case 3038:
//		konversi_state_telusur(1);
//		state_jalan = 3043;
//		break;
//	case 3039:
//		konversi_state_telusur(1);
//		state_jalan = 3044;
//		break;
//	case 3044:
//		telusurKanan_zigzag();
//		if (ganti_muka >= 3&&hitung_zigzag == 5)
//		{
//			state_jalan = 3041;
//		}
//		break;
//	case 3041:
//		konversi_state_telusur(-1);
//		state_jalan = 3042;
//		ganti_muka = 0;
//		break;
//	case 3042:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 4043:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			state_jalan = 4044;
//		}
//		break;
//	case 4044:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3043:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 1&&hitung_zigzag == 5)
//		{
//			ganti_muka = 0;
//			state_jalan = 3039;
//		}
//		break;
//	case 304:
//		state_jalan = LURUSKAN_KIRI;
//		next_state_jalan = 3045;
//		break;
//	case 3045://return 4A anjing2&3 ke 1
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3046:
//		telusurKiri_zigzag();
//		if (hitung_zigzag > 4)
//		{
//			state_jalan = 999;
//		}
//		break;
//	case 3047: //return 4A anjing1 ke 1
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3013:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			state_jalan = 3047;
//			konversi_state_telusur(1);
//		}
//		break;
//	case 3012:
//		maju(state_telusur);
//		konversi_gp(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 3048;
//		}
//		break;
//	case 3048://return 4A ke 2
//		telusurKiri_zigzag();
//		if (ganti_muka == 2)
//		{
//			konversi_state_telusur(-2);
//			jalan_langkah(state_telusur, 2);
//			state_jalan = 3049;
//		}
//		else if (ganti_muka == 1&&hitung_zigzag == 3)
//		{
//			konversi_state_telusur(1);
//			jalan_langkah(state_telusur, 2);
//			state_jalan = 3049;
//		}
//		break;
//	case 3049:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3050://return 4A ke 3
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			ganti_muka = 0;
//			konversi_state_telusur(1);
//			state_jalan = 3051;
//		}
//		break;
//	case 3051:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 1&&hitung_zigzag == 4)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 3052;
//		}
//		else if (ganti_muka == 2)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 3052;
//		}
//		break;
//	case 3052:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3053:
//		ganti_muka = 0;
//		state_jalan = 3048;
//		break;
//	case 3054://return dari 3 ke 4A
//		ganti_muka = 0;
//		konversi_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 3055;
//		}
//
//		break;
//	case 3055:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		telusurKanan_zigzag();
//		if (sensor_gp_asli[3] < 300)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 3056;
//		}
//		break;
//	case 3056:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 5)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 3057;
//		}
//		break;
//	case 3057:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3058://return 2 ke 4A
//		ganti_muka = 0;
//		jalan_langkah(state_telusur, 2);
//		konversi_state_telusur(-1);
//		state_jalan = 3059;
//		break;
//	case 3059:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if(sensor_gp_asli[1] < 200)
//		{
//			jalan_langkah(state_telusur,2);
//			geser_kanan(state_telusur,2);
//			konversi_state_telusur(1);
//			state_jalan = 3060;
//		}
//		break;
//	case 3060:
//		telusurKanan_zigzag();
//		if (ganti_muka == 2)
//		{
//			state_jalan = 3061;
//		}
//		break;
//	case 3061:
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if (sensor_gp[sensor[3]]<200)
//		{
//			pewaktu=0;
//			jalan_langkah(state_telusur,3);
//			gerakStatic(0, 0, 0, 0, 0, 0, 30);
//			setKompas();
//			tembak_cmps();
//			state_jalan = LURUSKAN_CMPS;
//			next_state_jalan = 3162;
//			konversi_state_telusur(-1);
//		}
//		break;
//	case 3162:
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(pewaktu>100&&baca_garis()==0)
//		{
//			if(sensor_gp[sensor[0]]<200&&sensor_gp[sensor[7]]>200)
//			{
//				geser_kanan(state_telusur,3);
//				state_jalan = 3163;
//			}
//			if(sensor_gp[sensor[1]]<200)
//			{
//				geser_kiri(state_telusur,2);
//				state_jalan = 3163;
//			}
//			else state_jalan = 3162;
//		}
//		if(baca_garis()==1)
//		{
//			jalan_langkah(state_telusur,3);
//			state_jalan = 999;
//		}
//		break;
//	case 3163:
//		baca_gp(state_telusur);
//		gp_serong();
//		maju(state_telusur);
//		if(baca_garis()==1)
//		{
//			jalan_langkah(state_telusur,3);
//			state_jalan = 999;
//		}
//		break;
//	case 3062:
//		konversi_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] <= setPoint0z + 20)
//		{
//			ganti_muka = 0;
//			state_jalan = 6000;
//		}
//		break;
//	case 6000:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 4)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 3064;
//		}
//		break;
//	case 3063:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[3] < 300)
//		{
//			jalan_langkah(state_telusur, 2);
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			state_jalan = 3082;
//		}
//		else if(hitung_langkah>3&&sensor_gp_asli[3]>300)
//		{
//			geser_kanan(state_telusur,1);
//			hitung_langkah = 0;
//			state_jalan = 3063;
//		}
//		break;
//	case 3064:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 5)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 3065;
//		}
//		break;
//	case 3065:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3067:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[3] < 300)
//		{
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 3076;
//		}
//		else if(hitung_langkah == 10)
//		{
//			geser_kanan(state_telusur,1);
//			hitung_langkah = 0;
//			state_telusur = 3067;
//		}
//		break;
//	case 3068:
//		telusurKiri_zigzag();
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		if (ganti_muka > 1&&sensor_gp_asli[0] < setPoint0z + 20)
//		{
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 5002;
//		}
//		break;
//
//	case 5002:
//		telusurKanan_zigzag();
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[3] < 300)
//		{
//			konversi_state_telusur(-1);
//			jalan_langkah(state_telusur, 1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 3064;
//		}
//		break;
//	case 3069://return 1A Ke 4A
//		jalan_langkah(state_telusur,2);
//		konversi_state_telusur(1);
//		state_jalan = 5003;
//		break;
//	case 5003:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[3] < setPoint2z + 190)
//		{
//			jalan_langkah(state_telusur, 2);
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 3070;
//		}
//		break;
//	case 3070:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		telusurKiri_zigzag();
//		if (ganti_muka > 1&&sensor_gp_asli[0] < setPoint0z + 20)
//		{
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 5002;
//		}
//		break;
//	case 3071://return 2 Ke 4B anjing3
//		jalan_langkah(state_telusur, 2);
//		konversi_state_telusur(-1);
//		hitung_langkah = 0;
//		state_jalan = 3063;
//		break;
//	case 3072:
//		maju(state_telusur);
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3073://return 1B ke 4A
//		ganti_muka = 0;
//		//konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 100)
//		{
//			state_jalan = 3074;
//		}
//
//		break;
//	case 3074:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		telusurKiri_zigzag();
//		if (ganti_muka > 1&&sensor_gp_asli[0] < setPoint0z + 20)
//		{
//			ganti_muka = 0;
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 5002;
//		}
//		break;
//	case 3075://return 2 ke 4B anjing1&2
//		jalan_langkah(state_telusur, 2);
//		konversi_state_telusur(-1);
//		hitung_langkah = 0;
//		state_jalan = 3067;
//		break;
//	case 3076:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		telusurKiri_zigzag();
//		if (ganti_muka > 1&&sensor_gp_asli[0] < setPoint0z + 20)
//		{
//			ganti_muka = 0;
//			penanda_buzzer = 0;
//			konversi_state_telusur(-1);
//			state_jalan = 4200;
//		}
//		break;
//	case 4200:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < setPoint0z)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 3078;
//		}
//		break;
//	case 3077:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 4)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 3079;
//		}
//
//		break;
//	case 3078:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		telusurKanan_zigzag();
//		if (sensor_gp_asli[3] < 300)
//		{
//			jalan_langkah(state_telusur, 1);
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			penanda_buzzer = 0;
//			state_jalan = 3077;
//		}
//		break;
//	case 3079:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3080:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3081:
//		telusurKiri_zigzag();
//		if (ganti_muka == 2&&hitung_zigzag == 4)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 3079;
//		}
//		break;
//	case 3082:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			penanda_buzzer = 0;
//			konversi_state_telusur(-2);
//			ganti_muka = 0;
//			state_jalan = LURUSKAN_KANAN;
//			next_state_jalan = 4102;//3085;
//		}
//		break;
//	case 3085:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < 100)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 4102;
//		}
//		break;
//	case 4101:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 5)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 4100;
//		}
//		break;
//	case 4102:
//		telusurKanan_zigzag();
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[3] < 300&&ganti_muka>0)
//		{
//			jalan_langkah(state_telusur, 1);
//			konversi_state_telusur(-1);
//			penanda_buzzer = 0;
//			ganti_muka = 0;
//			state_jalan = 4101;
//		}
//		break;
//	case 3083:
//		telusurKanan_zigzag();
//		if (ganti_muka >= 1&&hitung_zigzag == 5)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 4100;
//		}
//		break;
//	case 4100:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3084://return dari 3 ke 4B anjing1&2
//		ganti_muka = 0;
//		state_jalan = 4200;
//		break;
//	case 3088:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3089://return dari 3 ke 4B anjing3
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < setPoint0z)
//		{
//			konversi_state_telusur(-1);
//			state_jalan = 3090;
//		}
//		break;
//	case 3090:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		telusurKanan_zigzag();
//		if (sensor_gp_asli[3] < 300)
//		{
//			jalan_langkah(state_telusur, 1);
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			penanda_buzzer = 0;
//			state_jalan = 3091;
//		}
//		break;
//	case 3091:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 5)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 3092;
//		}
//		break;
//	case 3092:
//		telusurKiri_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3093://return 1 ke 4B anjing1&2
//		ganti_muka = 0;
//		state_jalan = 3095;
//		break;
//	case 3094:
//		telusurKiri_zigzag();
//		if (ganti_muka >= 2&&hitung_zigzag == 4)
//		{
//			konversi_state_telusur(1);
//			state_jalan = 4000;
//		}
//		break;
//	case 4000:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 4001:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[3] < 300)
//		{
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			state_jalan = 3076;
//		}
//		break;
//	case 3095:
//		jalan_langkah(state_telusur,2);
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		if (ruang_api == 1)//sensor_gp_asli[0] > thd_dinding)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 4001;
//		}
//		else if (ruang_api == 5)//sensor_gp[sensor[0]] < thd_dinding)
//		{
//			ganti_muka = 0;
//			state_jalan = 3016;
//		}
//		break;
//	case 3096://return 1A ke 4B anjing3
//		ganti_muka = 0;
//		jalan_langkah(state_telusur,2);
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		if (sensor_gp_asli[0] > thd_dinding)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 3097;
//		}
//		else if (sensor_gp[sensor[0]] < thd_dinding)
//		{
//			ganti_muka = 0;
//			state_jalan = 3098;
//		}
//		break;
//	case 3097:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[3] < setPoint2z + 190)
//		{
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			state_jalan = 3082;
//		}
//		break;
//	case 3098:
//		konversi_gp(state_telusur);
//		baca_gp(state_telusur);
//		maju(state_telusur);
//		if (sensor_gp_asli[0] < setPoint0z)
//		{
//			ganti_muka = 0;
//			state_jalan = 3082;
//		}
//		break;
//	case 3102://return 4B ke 2 Anjing1&2
//		ganti_muka = 0;
//		jalan_langkah(state_telusur, 3);
//		state_jalan = 3103;
//		break;
//	case 3103:
//		telusurKanan_zigzag();
//		if (ganti_muka == 4)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 3104;
//		}
//		break;
//	case 3104:
//		telusurKanan_zigzag();
//		if (baca_garis() == 1)
//		{
//			//kirim_case();
//			pewaktu=0;
//			jalan_langkah(state_telusur, 3);
//			state_jalan = 999;
//		}
//		break;
//	case 3105://return 4B ke 2 Anjing3
//		ganti_muka = 0;
//		maju(state_telusur);
//		if (sensor_gp[sensor[0]] < setPoint0z + 20)
//		{
//			state_jalan = 3106;
//		}
//		break;
//	case 3106:
//		telusurKiri_zigzag();
//		if ((ganti_muka >= 2)&&hitung_zigzag == 5)
//		{
//			konversi_state_telusur(1);
//			ganti_muka = 0;
//			state_jalan = 3107;
//		}
//		break;
//	case 3107:
//		telusurKanan_zigzag();
//		if (ganti_muka >= 3&&hitung_zigzag == 6)
//		{
//			konversi_state_telusur(-1);
//			ganti_muka = 0;
//			state_jalan = 3108;
//		}
//		break;
//	case 3108:
//		telusurKanan_zigzag();
//		if (ganti_muka == 3)
//		{
//			konversi_state_telusur(-2);
//			state_jalan = 3104;
//		}
//		break;
//	case 3109://return 3 ke 1C anjing3&2
//
//		break;
//	case 9000://back up
//		baca_gp(state_telusur);
//		konversi_gp(state_telusur);
//		gp_serong();
//		telusurKanan_zigzag();
//		detek_anjing();
//		if(detek_anjing()==1)
//		{
//			konversi_state_telusur(-1);
//		}
//		if(baca_garis()==1)
//		{
//			penanda_buzzer=0;
//			if(eksistansi_api2 == 1 || eksistansi_api1 == 1)
//			{
//				penanda_buzzer = 0;
//				//pemadam api
//			}
//		}
////		if(pewaktu>2000)
////		{
////			state_jalan=999;
////		}
//		break;
//	case 9001://deket tembok kiri
//		muter_(kan,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			state_jalan=9003;
//		}
//		else if(pewaktu>85 && (ruang_api==3||ruang_api==4))
//		{
//			pewaktu=0;
//			state_jalan=9009;
////			muter_kompas(UTARA);
////			if(sudah_lurus==1)
////			{
////				state_telusur=0;
////				baca_gp(state_telusur);
////				gp_serong();
////				state_jalan=2220;
////			}
//		}
//		else if(pewaktu>85 && ruang_api==7)
//		{
//			muter_kompas(SELATAN);
//			if(sudah_lurus==1)
//			{
//				state_telusur=0;
//				baca_gp(state_telusur);
//				gp_serong();
//				state_jalan=2220;
//			}
//		}
//		else if(pewaktu>110)
//		{
//			if(ruang_api==5)
//			{
//				pewaktu=0;
//				state_jalan=9009;
////				muter_kompas(TIMUR);
////				if(sudah_lurus==1)
////				{
////					state_telusur=0;
////					baca_gp(state_telusur);
////					gp_serong();
////					state_jalan=2230;
////				}
//			}
//			//else state_jalan=999;
//		}
//		break;
//	case 9009:
//		muter_(kir,5,25);
//		if(pewaktu>45 && ruang_api==3)
//		{
//			state_telusur=0;
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			geser_kiri(state_telusur,1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=2220;
//			//state_jalan=999;
//		}
//		if(pewaktu>50 && ruang_api==5)
//		{
//			state_telusur=0;
//			lock_state_telusur=state_telusur;
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			geser_kiri(state_telusur,1);
//			state_jalan=LURUSKAN_KIRI;
//			next_state_jalan=2230;
//		}
//		break;
//	case 9002://deket tembok kanan
//		muter_(kir,5,20);
//		if(tpa[0]>thd_tpa||tpa[1]>thd_tpa||tpa[2]>thd_tpa||tpa[3]>thd_tpa||tpa[4]>thd_tpa||tpa[5]>thd_tpa||tpa[6]>thd_tpa||tpa[7]>thd_tpa)
//		{
//			jalan_langkah(state_telusur,1);
//			state_jalan=9003;
//		}
//		else if(pewaktu>110)
//		{
//			if(ruang_api==1)//tpa tdk dapat
//			{
//				pewaktu=0;
//				state_jalan=9010;
//			}
//
//		}
//		break;
//	case 9010:
//		muter_(kan,5,25);
//		if(pewaktu>50)
//		{
//			state_telusur=0;
//			konversi_gp(state_telusur);
//			baca_gp(state_telusur);
//			gp_serong();
//			geser_kanan(state_telusur,1);
//			state_jalan=LURUSKAN_KANAN;
//			next_state_jalan=2210;
//			//state_jalan=999;
//		}
//		break;
//	case 9003:
//		led_api_on;
//		status_tpa=1;
//		PID_tpa();
//		if(PID_tpa()==1)
//		{
//			padamkan_api();
//			eksistansi_api2 = 0;
//			eksistansi_api1 = 0;
//			state_jalan=9004;
//		}
//		break;
//	case 9004:
//		if (deteksi_api(4000) == 1)
//		{
//			state_telusur=0;
//			baca_gp(state_telusur);
//			gp_serong();
//			state_jalan = 201;//balik ke awal, padamin pake lagoritma lama
//			break;
//		}
//		else
//		{
//			led_api_off;
//			if(ruang_api == 1 && status_tpa == 1)//acuan sesudah tpa
//			{
//				mataangin = UTARA;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 400;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//			if(ruang_api==5 && status_tpa==1)
//			{
//				mataangin = SELATAN;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 498;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//			if(ruang_api==3 && status_tpa==1)
//			{
//				mataangin = BARAT;
//				state_jalan =LURUSKAN_CMPS;
//				next_state_jalan = 498;
//				state_telusur = 0;
//				konversi_gp(0);
//			}
//		}
//		break;
//	case 498:
//		maju(0);
//		gp_serong();
//		konversi_gp(0);
//		if (sensor_gp[sensor[0]] < 80)
//		{
//			konversi_gp(0);
//			baca_gp(0);
//			//kirim_case();
//			state_jalan = 497;
//			pewaktu = 0;
//		}
//		break;
//	case 497:
//		baca_gp(state_telusur);
//		gp_serong();
//		konversi_gp(state_telusur);
//		gerak(0, 0, 9, 1, 25);
//		if (sensor_gp_asli[1]<150&&sensor_gp[sensor[1]]>100)
//		{
//			state_jalan = 496;
//			setKompas();
//			tembak_cmps();
//			statusKompas = 1;
//		}
//		break;
//	case 496:
//		telusurKanan_zigzag_ruang();
//		if(baca_garis() == 1)
//		{
//			pewaktu=0;
//			baca_gp(state_telusur);
//			konversi_gp(state_telusur);
//			penanda_buzzer = 0;
//			jalan_langkah(state_telusur,3);
//			state_jalan=532;
//			ganti_muka = 0;
//			setKompas();
//		}
//		break;
	}
}

int kunjungan;
void demo_kunjungan()
{
	switch (kunjungan) {
		case 0:
			telusurKanan_zigzag_ruang();
			if (nilai_api > thd_api) //245
			{
				penanda_buzzer = 0;
				gerakStatic(0, 0, 0, 0, 0, 0, 30);
				penanda_api_1 = 1;
				kunjungan = 1;
			}
			break;
		case 1:
			led_api_on;
			PID_api();
			if (PID_api() == 1)
			{
				penanda_buzzer = 0;
				kunjungan = 2;
				api_siap = 0;
			}
			break;
		case 2:
			padamkan_api();
			kunjungan = 3;
			state_telusur = 0;
			eksistansi_api2 = 0;
			eksistansi_api1 = 0;
			break;
		case 3:
			gerakStatic(0, 0, 0, 0, 0, 0, 30);
			if (start_on)
			{
				state_start = 0;
				kunjungan = 0;
			}
			break;
	}
}

int anjing;
int detek_anjing()
{
	switch(state_telusur)
	{
		case 0:
			baca_gp(state_telusur);
			konversi_gp(state_telusur);
			if(sensor_gp_asli[0]<150 && sensor_gp[sensor[0]]>200)
			{
				anjing=1;
				penanda_buzzer = 0;
			}
			else
				anjing=0;
			break;
		case 1:
			baca_gp(state_telusur);
			konversi_gp(state_telusur);
			if(sensor_gp_asli[0]<150 && sensor_gp[sensor[0]]>200)
			{
				anjing=1;
				penanda_buzzer = 0;
			}
			else
				anjing=0;
			break;
		case 2:
			baca_gp(state_telusur);
			konversi_gp(state_telusur);
			if(sensor_gp_asli[0]<150 && sensor_gp[sensor[0]]>200)
			{
				anjing=1;
				penanda_buzzer = 0;
			}
			else
				anjing=0;
			break;
		case 3:
			baca_gp(state_telusur);
			konversi_gp(state_telusur);
			if(sensor_gp_asli[0]<150 && sensor_gp[sensor[0]]>160)
			{
				anjing=1;
				penanda_buzzer = 0;
			}
			else
				anjing=0;
			break;
	}
	return anjing;
}

void berhenti()
{
	gerakStatic(0, 0, 0, 0, 0, 0, 30);
}

void muter_(int arah, int sudut, int kec)
{
	if(arah==kan){
		gerak(0, 0, -sudut, 1, kec);
	}
	if(arah==kir){
		gerak(0, 0, sudut, 1, kec);
	}
}

int PID_tpa()
{
	int i;
	for(i=0;i<8;i++)
	{
		if(tpa[i]>thd_tpa)posisi_tpa=i;
	}
	if(posisi_tpa==3||4)
	{
		berhenti();
		api_siap=1;
	}
	else if(posisi_tpa>4)
	{
		putar_api=1;
	}
	else if(posisi_tpa<3)
	{
		putar_api=0;
	}
	else
	{
		if(putar_api==1)
		{
			muter_(kir,5,20);
		}
		else if(putar_api==0)
		{
			muter_(kan,5,20);
		}
	}
	return api_siap;
}

float errorKompas, pKompas, dKompas, lerrorKompas, sudutKompas;
void jalanKompas(int acuan, float kp, float kd)
{
	errorKompas = nilai_cmps() - acuan;
	if (errorKompas >= 180) errorKompas = errorKompas - 360;
	if (errorKompas <= -180) errorKompas = errorKompas + 360;
	if(errorKompas>20)errorKompas=20;
	if(errorKompas<-20)errorKompas=-20;
	pKompas = kp * errorKompas;
	dKompas = kd * (errorKompas - lerrorKompas);
	lerrorKompas = errorKompas;
	sudutKompas = pKompas + dKompas;
	if(sudutKompas>5)sudutKompas=5;
	if(sudutKompas<-5)sudutKompas=-5;
	if(sudutKompas<1 && sudutKompas>-1)sudutKompas=0;

	gerak(0, 2, -sudutKompas, 1, 50);

	lcd_gotoxy(0,1);
	sprintf(tampil,"%.2f %.2f",errorKompas,sudutKompas);
	lcd_puts(tampil);
}

int count;
int count_pompa;
int count_ceper;
int count_putar_kalibrasi;
int count_garis_tinggi;
void start()
{
	if ((start_on||sound_on || state_USART2 == 2)&&state_start != 3 )
	{
		while (start_on)
			;
		if (menu != 2)
		{
			state_start = 3;
			menu = 8;
			lcd_clear();
			pewaktu = 0;
			led_sound_on;
			gerakStatic(0, 0, 0, 0, 0, 0, 30);
			while (statusGerak[0] == 1 || statusGerak[1] == 1 || statusGerak[2] == 1 || statusGerak[3] == 1)
				;
		}
	}
	else if (tombol2_on)
	{
		while (tombol2_on)
		{
			count_ceper++;
			delay_ms(10);
			if (count_ceper > 300)
			{
				gerakStatic(0, 0, 0.7, 0, 0, 0, 30);
			}
		}

		if (state_start == 1&&count_ceper <= 300)
		{
			state_start = 5;
			lcd_clear();
		}
		else if (state_start == 5&&count_ceper <= 300)
		{
			state_start = 1;
			lcd_clear();
		}
		else
		{
			menu++;
			if (menu > 11)menu = 0;
			lcd_clear();
			setKompas();
		}
		count_ceper = 0;
	}
	else if (tombol1_on)
	{

		while (tombol1_on)
		{
			count_putar_kalibrasi++;
			delay_ms(10);
			if (count_putar_kalibrasi > 300)
			{
				pewaktu=0;
				putar_biasa(360);
			}
		}
		count_putar_kalibrasi = 0;

		if (state_start == 1)
		{
			state_start = 4;
			lcd_clear();
		}
		else if (state_start == 4)
		{
			state_start = 1;
			lcd_clear();
		}
		else
		{
			menu--;
			if (menu < 0)menu = 11;
			lcd_clear();
			setKompas();
		}
	}
	else if (tombol3_on)
	{
		while (tombol3_on)
		{
			count_pompa++;
			delay_ms(10);
			if (count_pompa > 300)pompa_on;
		}
		pompa_off;
		count_pompa = 0;
		if (state_start == 1)state_start = 2;
		else if (state_start == 2)state_start = 1;
		else if (state_start == 5)state_start = 1;

		lcd_clear();
	}
	switch (state_start)
	{
	case 0 :
		kalibrasi = 1;
		if (kalibrasi == 1)
		{
			statusKalibrasi = 0;
			setKompas();
			state_start = 1;
			buzzer();
		}
		break;
	case 1 :
		lcd_gotoxy(0, 0);
		lcd_puts("<<<BISMILLAH>>>");
		lcd_gotoxy(0, 1);
		lcd_puts("<<BERKAKI 2018>>");
		break;
	case 2:
		switch (menu)
		{
		case 0 :
			lcd_gotoxy(0, 0);
			lcd_puts("SRF");
			tampil_gp(1);
			break;
		case 1 :
			lcd_gotoxy(0, 0);
			lcd_puts("UV");
			if (uv1_on || uv2_on)
			{
				(GPIO_SetBits(GPIOE, GPIO_Pin_4)); //buzzer on
				led_api_on;
			}
			else
			{
				(GPIO_ResetBits(GPIOE, GPIO_Pin_4)); //buzzer off
				led_api_off;
			}
			lcd_gotoxy(0, 1);
			sprintf(tampil, "UV1 : %3d", GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15));
			lcd_puts(tampil);

			lcd_gotoxy(0, 2);
			sprintf(tampil, "UV2 : %3d", GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9));
			lcd_puts(tampil);
			break;
		case 2 :
			lcd_gotoxy(0, 0);
			lcd_puts("SOUND ACTIVATOR");
			if (sound_on)
			{
				lcd_gotoxy(0, 1);
				lcd_puts("////ON////");
				led_sound_on;
			}
			else
			{
				lcd_gotoxy(0, 1);
				lcd_puts("===OFF=== ");
				led_sound_off;
			}
			break;
		case 3 :
			lcd_gotoxy(0, 0);
			lcd_puts("GARIS");
			lcd_gotoxy(0, 1);
			lcd_puts("Out :");
			tampil_garis(6, 1);
			lcd_gotoxy(0, 2);
			lcd_puts("THD :");
			lcd_gotoxy(6, 2);
			sprintf(tampil, "%2.1f", rata_garis);
			lcd_puts(tampil);
			lcd_gotoxy(0, 3);
			lcd_puts("Nilai :");
			lcd_gotoxy(8, 3);
			sprintf(tampil, "%2.1f", nilai_garis);
			lcd_puts(tampil);
			break;
		case 4 :
			modeBluetooth();
			break;
		case 5 :
			tampil_api();
			break;
		case 6:
			lcd_gotoxy(0, 0);
			lcd_puts("GP");
			tampil_gp_asli();
			break;
		case 7:
			lcd_gotoxy(0, 0);
			lcd_puts("CMPS");
			lcd_gotoxy(0,1);
			sprintf(tampil,"%3d",nilai_cmps());
			lcd_puts(tampil);
			break;
		case 8:
			lcd_gotoxy(0,0);
			lcd_puts("TAMPIL ARAH");
			tembak_cmps();
			tampil_mataangin();
			break;
		case 9:
			lcd_gotoxy(0,0);
			lcd_puts("TAMPIL TPA");
			tampil_tpa();
			break;
		case 10:
			lcd_gotoxy(0, 0);
			lcd_puts("Cek Servo CO");
			while (tombol2_on)
			{
				count_pompa++;
				delay_ms(10);
				if (count_pompa > 300);posisico = buka_co;
			}
			posisico = tutup_co;
			count_pompa = 0;
			break;

		case 11:
			lcd_gotoxy(0, 0);
			sprintf(tampil, "K1 : %3d", GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13));
			lcd_puts(tampil);

			lcd_gotoxy(0, 1);
			sprintf(tampil, "K2 : %3d", GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13));
			lcd_puts(tampil);

			lcd_gotoxy(0, 2);
			sprintf(tampil, "K3 : %3d", GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15));
			lcd_puts(tampil);

			lcd_gotoxy(0, 3);
			sprintf(tampil, "K4 : %3d", GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5));
			lcd_puts(tampil);

			break;
		}
		break;
	case 3 :
		//gerak(0, 0, 5, 1, 25);
		jalan();
		//telusurKiriPipih();
		//maju_tangga(0);
//		gerak_lewat_tangga(1);
//		maju_detek_floor(1);
//		delay_ms(100);
//		gerakStatic(0,0,0,0,0,0,30);
		break;
	case 4 :
		lcd_gotoxy(0, 0);
		lcd_puts("KALIBRASI GARIS");
		kalibrasi_garis();
		break;
	case 5 :
		gerak(0, 1, 0, 0.5, 30);
		break;
	}
}




int main(void)
{
	SystemInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);   //enable interrupt
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	inisialisasiIO();
	inisialisasiRX28(); //Inisialisasi TX RX Servo USART6
	inisialisasiPosisiKaki();
	inisialisasi_ADC_dengan_DMA2();
	inisialisasi_timer6(); //Cartesian Trajectory Planning
	inisialisasi_USART2(); //USART Bluetooth
	inisialisasi_USART1(); //USART Slave
	kecepatan_edf = edf;
	lcd_init(16);
	lcd_clear();
	pewaktuDemo = 0;
	belok = 0;
	direksi = 0;
	posisico = tutup_co;
	buzzer();
	gerakStatic(0, 0, 0.7, 0, 0, 0, 15);
	delay_ms(3000); //delay waktu u/ edf & kompas
	led_muka_mati();
	while (1)
	{
		start();
	}
}
