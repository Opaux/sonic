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
#define ERR_RECEIVER_ID XPAR_I2S_RECEIVER_0_DEVICE_ID
#define REF_RECEIVER_ID XPAR_I2S_RECEIVER_1_DEVICE_ID
#define ERR_MIC_DMA_ID XPAR_AXI_DMA_0_DEVICE_ID
#define REF_MIC_DMA_ID XPAR_AXI_DMA_1_DEVICE_ID

#define N_SAMPLES           4096
#define BYTES_PER_SAMPLE    4
#define RX_BYTES            (N_SAMPLES * BYTES_PER_SAMPLE)
#define DISCARDED_SAMPLES   256
#define TLAST_SAMPLES 64
#define CHUNK_BYTES (TLAST_SAMPLES * 4)

//Instances
XI2s_Tx I2Stx_inst;
XI2stx_Config *I2Stx_ConfigPtr;
XI2s_Rx I2SrxErr_inst;
XI2srx_Config *I2SrxErr_ConfigPtr;
XI2s_Rx I2SrxRef_inst;
XI2srx_Config *I2SrxRef_ConfigPtr;
XAxiDma ERRDMA_inst;
XAxiDma_Config *ERRDMA_ConfigPtr;
XAxiDma REFDMA_inst;
XAxiDma_Config *REFDMA_ConfigPtr;

static u32 ErrBuf[N_SAMPLES] __attribute__((aligned(64)));
static u32 RefBuf[N_SAMPLES] __attribute__((aligned(64)));

static inline int32_t signext24(uint32_t x24)
{
    x24 &= 0xFFFFFF;
    if (x24 & 0x800000) x24 |= 0xFF000000;
    return (int32_t)x24;
}

static inline int32_t word_to_s24(uint32_t w)
{
    uint32_t x24 = (w >> 4) & 0xFFFFFF;   // tdata[27:4]
    return signext24(x24);
}


int main()
{
    int Status;
    // Find configs
    I2Stx_ConfigPtr = XI2s_Tx_LookupConfig(I2S_TRANSMITTER_ID);
    I2SrxErr_ConfigPtr = XI2s_Rx_LookupConfig(ERR_RECEIVER_ID);
    I2SrxRef_ConfigPtr = XI2s_Rx_LookupConfig(REF_RECEIVER_ID);
    ERRDMA_ConfigPtr = XAxiDma_LookupConfig(ERR_MIC_DMA_ID);
    REFDMA_ConfigPtr = XAxiDma_LookupConfig(REF_MIC_DMA_ID);
	Status = XI2s_Tx_CfgInitialize(&I2Stx_inst, I2Stx_ConfigPtr, I2Stx_ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: TX CfgInitialize Failed\r\n");
		return XST_FAILURE;
	}
	Status = XI2s_Rx_CfgInitialize(&I2SrxErr_inst, I2SrxErr_ConfigPtr, I2SrxErr_ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Error RX CfgInitialize Failed\r\n");
		return XST_FAILURE;
	}
	Status = XI2s_Rx_CfgInitialize(&I2SrxRef_inst, I2SrxRef_ConfigPtr, I2SrxRef_ConfigPtr->BaseAddress);
		if (Status != XST_SUCCESS) {
			xil_printf("Error: Ref RX CfgInitialize Failed\r\n");
			return XST_FAILURE;
		}
	Status = XAxiDma_CfgInitialize(&ERRDMA_inst, ERRDMA_ConfigPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Error Mic DMA CfgInitialize Failed\r\n");
		return XST_FAILURE;
	}
	Status = XAxiDma_CfgInitialize(&REFDMA_inst, REFDMA_ConfigPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Ref Mic DMA CfgInitialize Failed\r\n");
		return XST_FAILURE;
	}
	XAxiDma_IntrDisable(&REFDMA_inst, XAXIDMA_IRQ_ALL_MASK,
				    XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&REFDMA_inst, XAXIDMA_IRQ_ALL_MASK,
				XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrDisable(&ERRDMA_inst, XAXIDMA_IRQ_ALL_MASK,
					    XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&ERRDMA_inst, XAXIDMA_IRQ_ALL_MASK,
				XAXIDMA_DMA_TO_DEVICE);
	Status = XI2s_Tx_SetSclkOutDiv(&I2Stx_inst, 12288000, 48000);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: TX SetSclkOutDiv Failed\r\n");
		return XST_FAILURE;
	}
	Status = XI2s_Rx_SetSclkOutDiv(&I2SrxErr_inst, 12288000, 48000);
	if (Status != XST_SUCCESS) {
		xil_printf("Error: Error RX SetSclkOutDiv Failed\r\n");
		return XST_FAILURE;
	}
	Status = XI2s_Rx_SetSclkOutDiv(&I2SrxRef_inst, 12288000, 48000);
		if (Status != XST_SUCCESS) {
			xil_printf("Error: Ref RX SetSclkOutDiv Failed\r\n");
			return XST_FAILURE;
		}
	// For Receiver (Mic) - Assuming xi2srx.h has the matching function
	XI2s_Rx_JustifyEnable(&I2SrxErr_inst, 0);
	XI2s_Rx_JustifyEnable(&I2SrxRef_inst, 0);
	// For Transmitter (Speaker)
	XI2s_Tx_JustifyEnable(&I2Stx_inst, 0);
	Status = XI2s_Tx_SetChMux(&I2Stx_inst, XI2S_TX_CHID0, XI2S_TX_CHMUX_AXIS_01);
	if (Status != XST_SUCCESS) { xil_printf("TX CH0 mux fail\r\n"); return XST_FAILURE; }

	Status = XI2s_Tx_SetChMux(&I2Stx_inst, XI2S_TX_CHID1, XI2S_TX_CHMUX_DISABLED);
	if (Status != XST_SUCCESS) { xil_printf("TX CH1 mux fail\r\n"); return XST_FAILURE; }
	XI2s_Rx_SetChMux(&I2SrxErr_inst, 0, XI2S_RX_CHMUX_XI2S_01);
	XI2s_Rx_SetChMux(&I2SrxRef_inst, 0, XI2S_RX_CHMUX_XI2S_01);
    xil_printf("Error DMA HasMm2S=%d HasS2Mm=%d HasSg=%d AddrWidth=%d\r\n",
    		ERRDMA_inst.HasMm2S, ERRDMA_inst.HasS2Mm, ERRDMA_inst.HasSg, ERRDMA_inst.AddrWidth);
    xil_printf("Ref DMA HasMm2S=%d HasS2Mm=%d HasSg=%d AddrWidth=%d\r\n",
			REFDMA_inst.HasMm2S, REFDMA_inst.HasS2Mm, REFDMA_inst.HasSg, REFDMA_inst.AddrWidth);

    // Initially, speaker off, mic on
	XI2s_Rx_Enable(&I2SrxErr_inst, 1);
	XI2s_Rx_Enable(&I2SrxRef_inst, 1);
	XI2s_Tx_Enable(&I2Stx_inst, 1);
	xil_printf("Speaker UNMUTED. Audio passing through.\r\n");
    // 2) Cache: invalidate destination buffer before DMA writes into it
	for (int i = 0; i < N_SAMPLES; i++) {
	    ErrBuf[i] = 0xAAAAAAAA;
	    RefBuf[i] = 0x55555555;
	}
	Xil_DCacheInvalidateRange((UINTPTR)ErrBuf, RX_BYTES); // then invalidate for S2MM
	Xil_DCacheInvalidateRange((UINTPTR)RefBuf, RX_BYTES);
	usleep(200000);
    // 3) Start S2MM transfer (Device->DMA)
	for (int off = 0; off < N_SAMPLES; off += TLAST_SAMPLES) {
	    XAxiDma_SimpleTransfer(&ERRDMA_inst, (UINTPTR)&ErrBuf[off], CHUNK_BYTES, XAXIDMA_DEVICE_TO_DMA);
	    XAxiDma_SimpleTransfer(&REFDMA_inst, (UINTPTR)&RefBuf[off], CHUNK_BYTES, XAXIDMA_DEVICE_TO_DMA);

	    while (XAxiDma_Busy(&ERRDMA_inst, XAXIDMA_DEVICE_TO_DMA) ||
	           XAxiDma_Busy(&REFDMA_inst, XAXIDMA_DEVICE_TO_DMA)) {}

	    Xil_DCacheInvalidateRange((UINTPTR)&ErrBuf[off], CHUNK_BYTES);
	    Xil_DCacheInvalidateRange((UINTPTR)&RefBuf[off], CHUNK_BYTES);
    }
    u32 err_sr = XAxiDma_ReadReg(ERRDMA_inst.RegBase, XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET);
    u32 ref_sr = XAxiDma_ReadReg(REFDMA_inst.RegBase, XAXIDMA_RX_OFFSET + XAXIDMA_SR_OFFSET);
    xil_printf("ERR S2MM SR=0x%08lx  REF S2MM SR=0x%08lx\r\n", (long)err_sr, (long)ref_sr);

    xil_printf("BEGIN_SAMPLES\r\n");
    xil_printf("ref_s24,err_s24\r\n");
    for (int i = 256; i < N_SAMPLES; i++) {
        int32_t ref = word_to_s24(RefBuf[i]);
        int32_t err = word_to_s24(ErrBuf[i]);
        xil_printf("%ld,%ld\r\n", (long)ref, (long)err);
    }

    xil_printf("END_SAMPLES\r\n");
    // Let the passthrough run indefinitely
    while(1);
    return 0;
}
