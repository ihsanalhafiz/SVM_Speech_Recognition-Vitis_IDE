/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <math.h>
#include "complex.h"
#include "platform.h"
#include "xil_printf.h"
#include "xgpio.h"
#include "xparameters.h"
#include "xsvm_speech_30.h"
#include "xadcps.h"
#include "xstatus.h"
#include "sleep.h"
#include "xaxidma.h"

#define printf xil_printf /* Small foot-print printf function */
#define LED_CHANNEL 1
#define BUTTON_CHANNEL 2
#define LED 0x0F   /* Assumes bit 0 of GPIO is connected to an LED  */

XSvm_speech_30 svmSpeech30;
static XAdcPs XAdcInst; /* XADC driver instance */
XGpio Gpio; /* The Instance of the GPIO Driver */
XAxiDma AxiDma;

u32 readButton;
u8 triggerUart = 0;
u8 triggerAdc = 0;
u8 triggerFft = 0;
u8 triggertraindata = 0;
float complex adccomplex[16384];
float complex fftoutput[16384];
float fftFinalOutputReal[4096];
u32 dataFeedSvm[32];
u32 N = 16384;
float adcfloat;
u16 adcdata;
u16 dataAdc[4096] = { };
int svmresult;

float in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14,
		in15, in16, in17, in18, in19, in20, in21, in22, in23, in24, in25, in26,
		in27, in28, in29, in30, in31, in32;

u32 float_to_u32(float val) {
	union {
		float f;
		u32 u;
	} data = { val };
	return data.u;
}

float u32_to_float(u32 val) {
	union {
		u32 u;
		float f;
	} data = { val };
	return data.f;
}

int XAdcFractionToInt(float FloatNum) {
	float Temp;
	Temp = FloatNum;
	if (FloatNum < 0) Temp = -(FloatNum);
	return (((int) ((Temp - (float) ((int) Temp)) * (1000.0f))));
}

void SvmInit(){
	XSvm_speech_30_Config *svmSpeech30_cfg;
	svmSpeech30_cfg = XSvm_speech_30_LookupConfig(
			XPAR_SVM_SPEECH_30_0_DEVICE_ID);
	if (!svmSpeech30_cfg){
		xil_printf("error load config svmspeech30\n");
	}
	int status = XSvm_speech_30_CfgInitialize(&svmSpeech30,
			svmSpeech30_cfg);
	if (status != XST_SUCCESS)
		xil_printf("error initialize svmspeech30\n");
}

void AdcInit(){
	XAdcPs_Config *adc_cfg;
	adc_cfg = XAdcPs_LookupConfig(XPAR_XADCPS_0_DEVICE_ID);
	if (!adc_cfg) {
		xil_printf("error load config adc\n");
	}
	int status = XAdcPs_CfgInitialize(&XAdcInst, adc_cfg, adc_cfg->BaseAddress);
	if (status != XST_SUCCESS){
		xil_printf("error initialize adc\n");
	}
	/*
	 * Self Test the XADC/ADC device
	 */
	status = XAdcPs_SelfTest(&XAdcInst);
	if (status != XST_SUCCESS) {
	}
}

void DmaInit(){
	XAxiDma_Config *dma_cfg;
	dma_cfg = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
	if (!dma_cfg) {
		xil_printf("No config found for dma\r\n");
	}
	int status = XAxiDma_CfgInitialize(&AxiDma, dma_cfg);
	if (status != XST_SUCCESS) {
		xil_printf("DMA initialize failed\r\n");
	}
	if (XAxiDma_HasSg(&AxiDma)) {
		xil_printf("Device configuration as SG mode\r\n");
	}
	// Reset DMA
	XAxiDma_Reset(&AxiDma);
	while (!XAxiDma_ResetIsDone(&AxiDma)) {
	}
}

void GpioInit(){
	/* Initialize the GPIO driver */
	int Status = XGpio_Initialize(&Gpio, XPAR_GPIO_0_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("Gpio Initialization Failed\r\n");
	}
}

int SvmProcess(float in1,float in2,float in3,float in4,
		float in5,float in6,float in7,float in8,
		float in9,float in10,float in11,float in12,
		float in13,float in14,float in15,float in16,
		float in17,float in18,float in19,float in20,
		float in21,float in22,float in23,float in24,
		float in25,float in26,float in27,float in28,
		float in29,float in30,float in31,float in32){
	float result;
	XSvm_speech_30_Set_in1(&svmSpeech30, float_to_u32(in1));
	XSvm_speech_30_Set_in2(&svmSpeech30, float_to_u32(in2));
	XSvm_speech_30_Set_in3(&svmSpeech30, float_to_u32(in3));
	XSvm_speech_30_Set_in4(&svmSpeech30, float_to_u32(in4));
	XSvm_speech_30_Set_in5(&svmSpeech30, float_to_u32(in5));
	XSvm_speech_30_Set_in6(&svmSpeech30, float_to_u32(in6));
	XSvm_speech_30_Set_in7(&svmSpeech30, float_to_u32(in7));
	XSvm_speech_30_Set_in8(&svmSpeech30, float_to_u32(in8));
	XSvm_speech_30_Set_in9(&svmSpeech30, float_to_u32(in9));
	XSvm_speech_30_Set_in10(&svmSpeech30, float_to_u32(in10));
	XSvm_speech_30_Set_in11(&svmSpeech30, float_to_u32(in11));
	XSvm_speech_30_Set_in12(&svmSpeech30, float_to_u32(in12));
	XSvm_speech_30_Set_in13(&svmSpeech30, float_to_u32(in13));
	XSvm_speech_30_Set_in14(&svmSpeech30, float_to_u32(in14));
	XSvm_speech_30_Set_in15(&svmSpeech30, float_to_u32(in15));
	XSvm_speech_30_Set_in16(&svmSpeech30, float_to_u32(in16));
	XSvm_speech_30_Set_in17(&svmSpeech30, float_to_u32(in17));
	XSvm_speech_30_Set_in18(&svmSpeech30, float_to_u32(in18));
	XSvm_speech_30_Set_in19(&svmSpeech30, float_to_u32(in19));
	XSvm_speech_30_Set_in20(&svmSpeech30, float_to_u32(in20));
	XSvm_speech_30_Set_in21(&svmSpeech30, float_to_u32(in21));
	XSvm_speech_30_Set_in22(&svmSpeech30, float_to_u32(in22));
	XSvm_speech_30_Set_in23(&svmSpeech30, float_to_u32(in23));
	XSvm_speech_30_Set_in24(&svmSpeech30, float_to_u32(in24));
	XSvm_speech_30_Set_in25(&svmSpeech30, float_to_u32(in25));
	XSvm_speech_30_Set_in26(&svmSpeech30, float_to_u32(in26));
	XSvm_speech_30_Set_in27(&svmSpeech30, float_to_u32(in27));
	XSvm_speech_30_Set_in28(&svmSpeech30, float_to_u32(in28));
	XSvm_speech_30_Set_in29(&svmSpeech30, float_to_u32(in29));
	XSvm_speech_30_Set_in30(&svmSpeech30, float_to_u32(in30));
	XSvm_speech_30_Set_in31(&svmSpeech30, float_to_u32(in31));
	XSvm_speech_30_Set_in32(&svmSpeech30, float_to_u32(in32));

	XSvm_speech_30_Start(&svmSpeech30);
	while (!XSvm_speech_30_IsDone(&svmSpeech30));

	result = u32_to_float(XSvm_speech_30_Get_return(&svmSpeech30));
	return (int)result;
}

u32 checkIdle(u32 baseAddress, u32 offset) {
	u32 status;
	status = (XAxiDma_ReadReg(baseAddress, offset)) & XAXIDMA_IDLE_MASK;
	return status;
}

int main()
{
    init_platform();

    SvmInit();
    AdcInit();
    DmaInit();
    GpioInit();

	/* Set the direction for all signals as inputs except the LED output */
	XGpio_SetDataDirection(&Gpio, LED_CHANNEL, ~LED);
	/* Set the direction for all signals to be inputs */
	XGpio_SetDataDirection(&Gpio, BUTTON_CHANNEL, 0xFF);


    print("Hello World\n\r");
    print("Successfully ran Hello World application");

    while(1){
		readButton = XGpio_DiscreteRead(&Gpio, BUTTON_CHANNEL);
		if ((readButton >> 0) & 0x01) {
			XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x01);
			triggerAdc = 1;
		}
		if ((readButton >> 1) & 0x01) {
			triggerAdc = 0;
			triggerUart = 1;
			XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x02);
		}
		if ((readButton >> 2) & 0x01) {
			triggerFft = 1;
			XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x04);
		}
		if ((readButton >> 3) & 0x01) {
			triggertraindata = 1;
			XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x08);
		}

		if (triggerAdc == 1) {
//			xil_printf("sampling adc start\n");
			for (u32 i = 0; i < N; i++) {
				adcdata = XAdcPs_GetAdcData(&XAdcInst, XADCPS_CH_AUX_MIN + 1); //channel 1
				adcfloat = (float) (adcdata - 32268.0);
				adccomplex[i] = adcfloat + 0 * I;
				usleep(40);
			}
//			xil_printf("sampling adc done\n");
			triggerAdc = 0;
			triggerFft = 1;
		}
		if (triggerUart == 1) {
			for (u32 i = 0; i < 16384; i++) {
				xil_printf("%0d\n", (int) crealf(adccomplex[i]));
			}
			memset(dataAdc, 0, sizeof(dataAdc));
			triggerUart = 0;
		}
		if (triggerFft == 1) {

			/* Disable interrupts, we use polling mode
			 */
			XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
					XAXIDMA_DEVICE_TO_DMA);
			XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
					XAXIDMA_DMA_TO_DEVICE);

			Xil_DCacheFlushRange((UINTPTR) adccomplex,
					sizeof(float complex) * 16384);
			Xil_DCacheFlushRange((UINTPTR) fftoutput,
					sizeof(float complex) * 16384);

			int status_transfer;
			status_transfer = XAxiDma_SimpleTransfer(&AxiDma,
					(UINTPTR) fftoutput, sizeof(float complex) * 16384,
					XAXIDMA_DEVICE_TO_DMA);
			if (status_transfer == XST_INVALID_PARAM) {
				xil_printf(
						"writing data to fft_IP via DMA XST_INVALID_PARAM\r\n");
			} else if (status_transfer == XST_FAILURE) {
				xil_printf("writing data to fft_IP via DMA XST_FAILURE\r\n");
			}
			//else
//				xil_printf("writing data to fft_IP via DMA success\r\n");
			//checkIdle(XPAR_AXI_DMA_1_BASEADDR,0x34); //device to dma
			status_transfer = XAxiDma_SimpleTransfer(&AxiDma,
					(UINTPTR) adccomplex, sizeof(float complex) * 16384,
					XAXIDMA_DMA_TO_DEVICE);
			if (status_transfer == XST_INVALID_PARAM) {
				xil_printf(
						"writing data from fft_IP via DMA XST_INVALID_PARAM\r\n");
			} else if (status_transfer == XST_FAILURE) {
				xil_printf("writing data from fft_IP via DMA XST_FAILURE\r\n");
			}
			//else
//				xil_printf("writing data from fft_IP via DMA success\r\n");

//			xil_printf("check status %d and %d\r\n", checkIdle(XPAR_AXI_DMA_1_BASEADDR,0x4),checkIdle(XPAR_AXI_DMA_1_BASEADDR,0x34));

			int statuscek = checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x4); //dma to device
			while (statuscek != 2) {
				statuscek = checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x4); //dma to device
			}

			statuscek = checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x34); //dma to device
			while (statuscek != 2) {
				statuscek = checkIdle(XPAR_AXI_DMA_0_BASEADDR, 0x34); //dma to device
			}

//			xil_printf("FFT done\r\n");
			// print the result of FFT
			for (u16 i = 0; i < (4096); i++) {
				fftFinalOutputReal[i] =
						sqrt(
								(crealf(fftoutput[i]) * crealf(fftoutput[i]))
										+ (cimagf(fftoutput[i])
												* cimagf(fftoutput[i])));
//				xil_printf("%0d.%03d\n", (int)fftFinalOutputReal[i], XAdcFractionToInt(fftFinalOutputReal[i]));
			}
			triggerFft = 0;
			triggertraindata = 1;
		}
		if (triggertraindata == 1) {
//			xil_printf("start data train svm\r\n");
			for (u8 i = 0; i < 32; i++) {
				//xil_printf("start loop %d\r\n", i);
				for (u32 j = (i * 128); j < ((i * 128) + 128); j++) {
					dataFeedSvm[i] = dataFeedSvm[i]
							+ (u32) fftFinalOutputReal[j];
				}
				xil_printf("%d ", dataFeedSvm[i]);
				//dataFeedSvm[i] = 0;
			}
//			xil_printf("\n end data train svm\r\n");
			xil_printf(";\n");
			triggertraindata = 0;

			in1 = (float)dataFeedSvm[0];
			in2 = (float)dataFeedSvm[1];
			in3 = (float)dataFeedSvm[2];
			in4 = (float)dataFeedSvm[3];
			in5 = (float)dataFeedSvm[4];
			in6 = (float)dataFeedSvm[5];
			in7 = (float)dataFeedSvm[6];
			in8 = (float)dataFeedSvm[7];
			in9 = (float)dataFeedSvm[8];
			in10 = (float)dataFeedSvm[9];
			in11 = (float)dataFeedSvm[10];
			in12 = (float)dataFeedSvm[11];
			in13 = (float)dataFeedSvm[12];
			in14 = (float)dataFeedSvm[13];
			in15 = (float)dataFeedSvm[14];
			in16 = (float)dataFeedSvm[15];
			in17 = (float)dataFeedSvm[16];
			in18 = (float)dataFeedSvm[17];
			in19 = (float)dataFeedSvm[18];
			in20 = (float)dataFeedSvm[19];
			in21 = (float)dataFeedSvm[20];
			in22 = (float)dataFeedSvm[21];
			in23 = (float)dataFeedSvm[22];
			in24 = (float)dataFeedSvm[23];
			in25 = (float)dataFeedSvm[24];
			in26 = (float)dataFeedSvm[25];
			in27 = (float)dataFeedSvm[26];
			in28 = (float)dataFeedSvm[27];
			in29 = (float)dataFeedSvm[28];
			in30 = (float)dataFeedSvm[29];
			in31 = (float)dataFeedSvm[30];
			in32 = (float)dataFeedSvm[31];

			svmresult = SvmProcess(in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14,
					in15, in16, in17, in18, in19, in20, in21, in22, in23, in24, in25, in26,
					in27, in28, in29, in30, in31, in32);
			xil_printf("Svm result HW : %d\n", svmresult);

			for (u8 i = 0; i < 32; i++) {
				dataFeedSvm[i] = 0;
			}
		}

		/* Clear the LED bit */
		XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x00);
    }

    cleanup_platform();
    return 0;
}
