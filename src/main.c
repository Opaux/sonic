#include "xparameters.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xi2stx.h" // I2S TX library
#include "xi2srx.h" // I2S RX library
#include <unistd.h>

// I2S definitions
#define I2S_TRANSMITTER_ID 	XPAR_I2S_TRANSMITTER_0_DEVICE_ID
#define I2S_RECEIVER_ID XPAR_I2S_RECEIVER_0_DEVICE_ID

//I2S Instances
XI2s_Tx I2Stx_inst;
XI2stx_Config *I2Stx_ConfigPtr;
XI2s_Rx I2Srx_inst;
XI2srx_Config *I2Srx_ConfigPtr;

int main()
{
    int Status;
    // Find config for transmitter/receiver
    I2Stx_ConfigPtr = XI2s_Tx_LookupConfig(I2S_TRANSMITTER_ID);
    I2Srx_ConfigPtr = XI2s_Rx_LookupConfig(I2S_RECEIVER_ID);
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
    // Initially, speaker off, mic on
	XI2s_Rx_Enable(&I2Srx_inst, 1);
	XI2s_Tx_Enable(&I2Stx_inst, 1);
	xil_printf("Speaker UNMUTED. Audio passing through.\r\n");
    // Let the passthrough run indefinitely
    while(1);
    return 0;
}

