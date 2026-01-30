#include "xparameters.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xi2stx.h" // I2S TX library
#include "xi2srx.h" // I2S RX library
#include "xaxidma.h" // DMA library
#include <unistd.h>

// Definitions
#define I2S_TRANSMITTER_ID 	XPAR_I2S_TRANSMITTER_0_DEVICE_ID
#define I2S_RECEIVER_ID XPAR_I2S_RECEIVER_0_DEVICE_ID
#define ERR_MIC_DMA_ID XPAR_AXI_DMA_0_DEVICE_ID
#define N_SAMPLES           4095
#define BYTES_PER_SAMPLE    4
#define RX_BYTES            (N_SAMPLES * BYTES_PER_SAMPLE)

//Instances
XI2s_Tx I2Stx_inst;
XI2stx_Config *I2Stx_ConfigPtr;
XI2s_Rx I2Srx_inst;
XI2srx_Config *I2Srx_ConfigPtr;
XAxiDma DMA_inst;
XAxiDma_Config *DMA_ConfigPtr;

static u32 RxBuffer[N_SAMPLES] __attribute__((aligned(64)));
static int bufferVal = 0;
int main()
{
    int Status;
    // Find configs
    I2Stx_ConfigPtr = XI2s_Tx_LookupConfig(I2S_TRANSMITTER_ID);
    I2Srx_ConfigPtr = XI2s_Rx_LookupConfig(I2S_RECEIVER_ID);
    DMA_ConfigPtr = XAxiDma_LookupConfig(ERR_MIC_DMA_ID);
	Status = XI2s_Tx_CfgInitialize(&I2Stx_inst, I2Stx_ConfigPtr, I2Stx_ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: TX CfgInitialize Failed\r\n");
		return XST_FAILURE;
	}
	Status = XI2s_Rx_CfgInitialize(&I2Srx_inst, I2Srx_ConfigPtr, I2Srx_ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: RX CfgInitialize Failed\r\n");
		return XST_FAILURE;
	}
	Status = XAxiDma_CfgInitialize(&DMA_inst, DMA_ConfigPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: DMA CfgInitialize Failed\r\n");
		return XST_FAILURE;
	}
	Status = XI2s_Tx_SetSclkOutDiv(&I2Stx_inst, 12288000, 48000);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: TX SetSclkOutDiv Failed\r\n");
		return XST_FAILURE;
	}
	Status = XI2s_Rx_SetSclkOutDiv(&I2Srx_inst, 12288000, 48000);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: RX SetSclkOutDiv Failed\r\n");
		return XST_FAILURE;
	}
	// For Receiver (Mic) - Assuming xi2srx.h has the matching function
	XI2s_Rx_JustifyEnable(&I2Srx_inst, 0);
	// For Transmitter (Speaker)
	XI2s_Tx_JustifyEnable(&I2Stx_inst, 0);
	XI2s_Tx_SetChMux(&I2Stx_inst, 0, XI2S_TX_CHMUX_AXIS_01);
	XI2s_Rx_SetChMux(&I2Srx_inst, 0, XI2S_RX_CHMUX_XI2S_01);
    // Must be Simple mode for this code
    if (XAxiDma_HasSg(&DMA_inst)) {
        xil_printf("ERROR: DMA is SG-enabled. Disable SG in Vivado for bring-up.\r\n");
        return XST_FAILURE;
    }
    xil_printf("DMA HasMm2S=%d HasS2Mm=%d HasSg=%d AddrWidth=%d\r\n",
               DMA_inst.HasMm2S, DMA_inst.HasS2Mm, DMA_inst.HasSg, DMA_inst.AddrWidth);
    // Initially, speaker off, mic on
	XI2s_Rx_Enable(&I2Srx_inst, 1);
	XI2s_Tx_Enable(&I2Stx_inst, 1);
	xil_printf("Speaker UNMUTED. Audio passing through.\r\n");
    // 2) Cache: invalidate destination buffer before DMA writes into it
    Xil_DCacheInvalidateRange((UINTPTR)RxBuffer, RX_BYTES);
    // 3) Start S2MM transfer (Device->DMA)
    Status = XAxiDma_SimpleTransfer(&DMA_inst,
                                   (UINTPTR)RxBuffer,
                                   RX_BYTES,
								   XAXIDMA_DEVICE_TO_DMA);
    if (Status != XST_SUCCESS) {
        xil_printf("ERROR: S2MM transfer failed %d\r\n", Status);
        return XST_FAILURE;
    }
    // 4) Wait for completion
    while (XAxiDma_Busy(&DMA_inst, XAXIDMA_DEVICE_TO_DMA)) {
        // spin
    }
    // 5) Cache: invalidate again so CPU reads fresh data
    Xil_DCacheInvalidateRange((UINTPTR)RxBuffer, RX_BYTES);

    xil_printf("BEGIN_SAMPLES\r\n");
    for (int i = 0; i < N_SAMPLES; i++) {
        xil_printf("%ld\r\n", (long)RxBuffer[i]);
    }
    xil_printf("END_SAMPLES\r\n");
    // Let the passthrough run indefinitely
    while(1);
    return 0;
}

