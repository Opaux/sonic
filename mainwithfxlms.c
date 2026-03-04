/*
 * fxlms_sg_controller.c
 * ======================
 * Filtered-X LMS Active Noise Cancellation
 * Zynq PS bare-metal with Scatter-Gather DMA (Serial Telemetry Version)
 */

#include "xparameters.h"
#include "xil_printf.h"
#include "xi2stx.h"
#include "xi2srx.h"
#include "xaxidma.h"
#include "xaxidma_bd.h"
#include "xaxidma_bdring.h"
#include "xil_cache.h"
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdio.h>

// Hardware IDs
#define I2S_NOISE_ID     XPAR_I2S_TRANSMITTER_0_DEVICE_ID
#define I2S_ANTINOISE_ID XPAR_I2S_TRANSMITTER_1_DEVICE_ID
#define I2S_RX_ERR_ID    XPAR_I2S_RECEIVER_0_DEVICE_ID
#define DMA_ERR_TX_ID    XPAR_AXI_DMA_0_DEVICE_ID
#define DMA_REF_ID       XPAR_AXI_DMA_1_DEVICE_ID

// Algorithm parameters
#define FS              48000
#define BLOCK_SIZE      32
#define W_TAPS          256
#define SZ_TAPS         417

#define MU      0.00005f
#define LEAKY   0.999999f
#define MU_SPM            1e-3f
#define AUX_GAIN          0.02f
#define SPM_TRAIN_BLOCKS  24000
#define SPM_ENERGY_THRESH 1e-6f

// OCM addresses for BD rings
#define BD_ERR_RX_BASE    0x01F00000
#define BD_OUT_TX_BASE    0x01F01000
#define BD_NOISE_TX_BASE  0x01F02000
#define N_BD            4

// DMA buffer layout
#define WORDS_PER_BLOCK (BLOCK_SIZE * 2)
#define BYTES_PER_BLOCK (WORDS_PER_BLOCK * 4)
#define SNAPSHOT_LEN 512

// Audio buffers in DDR
static u32 NoiseBuf[2][WORDS_PER_BLOCK] __attribute__((aligned(64)));
static u32 ErrBuf[2][WORDS_PER_BLOCK] __attribute__((aligned(64)));
static u32 OutBuf[2][WORDS_PER_BLOCK] __attribute__((aligned(64)));

static float x_energy_accum = 0.0f;
static float e_energy_accum = 0.0f;
static u32   sample_count   = 0;
static float ref_snap[SNAPSHOT_LEN];
static float err_snap[SNAPSHOT_LEN];
static int snap_state = 0;
static int snap_idx = 0;
static int tx_idx = 0;

// Peripheral instances
static XI2s_Tx  I2StxNoise;
static XI2s_Tx  I2StxAntinoise;
static XI2s_Rx  I2SrxErr;
static XAxiDma  DmaErrTx;
static XAxiDma  DmaRef;

// Signal conversion helpers
static inline float s24_to_float(int32_t s24) {
    return (float)s24 / 8388608.0f;
}
static inline int32_t float_to_s24(float f) {
    if (f >  1.0f) f =  1.0f;
    if (f < -1.0f) f = -1.0f;
    return (int32_t)(f * 8388607.0f);
}
static inline uint32_t s24_to_i2s_word(int32_t s24) {
    return (uint32_t)((s24 & 0xFFFFFF) << 4);
}
static inline int32_t signext24(uint32_t x24) {
    x24 &= 0xFFFFFF;
    if (x24 & 0x800000) x24 |= 0xFF000000;
    return (int32_t)x24;
}
static inline int32_t word_to_s24(uint32_t w) {
    return ((int32_t)(w << 4)) >> 8;
}

static uint32_t aux_lfsr = 0xDEADBEEF;
static inline float aux_noise_next(void) {
    aux_lfsr ^= aux_lfsr << 13;
    aux_lfsr ^= aux_lfsr >> 17;
    aux_lfsr ^= aux_lfsr << 5;
    return (float)(int32_t)aux_lfsr * (AUX_GAIN / 2147483648.0f);
}
static uint32_t noise_lfsr = 0xACE1ACE1;
static inline int32_t noise_next_s24(void) {
    noise_lfsr ^= noise_lfsr << 13;
    noise_lfsr ^= noise_lfsr >> 17;
    noise_lfsr ^= noise_lfsr << 5;
    return (int32_t)noise_lfsr >> 10;
}

//FXLMS state (Standard FXLMS)
typedef struct {
    float w[W_TAPS];
    float x_buf[W_TAPS * 2];
    float sz_buf[SZ_TAPS * 2];
    float fx[W_TAPS * 2];
    int   x_idx;
    int   sz_idx;
    int   fx_idx;
    float power;

    // Secondary Path (Speaker -> Error Mic)
    float shat[SZ_TAPS];
    float aux_buf[SZ_TAPS * 2];
    int   aux_idx;

    int   train_blocks;
    int   anc_active;
} fxlms_t;

static fxlms_t anc;

static void fxlms_init(void) {
    memset(&anc, 0, sizeof(anc));
    anc.power = 1e-6f;
    anc.x_idx = anc.sz_idx = anc.fx_idx = anc.aux_idx = 0;
    anc.train_blocks = 0;
    anc.anc_active = 0;
    xil_printf("FXLMS init: W=%d SZ=%d (Standard FXLMS)\r\n", W_TAPS, SZ_TAPS);
    xil_printf("Training SPM for %d blocks...\r\n", SPM_TRAIN_BLOCKS);
}

static int32_t fxlms_process(int32_t x_raw, int32_t e_raw, float aux) {
    float x = s24_to_float(x_raw);
    float e = s24_to_float(e_raw);

    if (!anc.anc_active) {
        anc.aux_idx = (anc.aux_idx == 0) ? (SZ_TAPS-1) : (anc.aux_idx - 1);
        anc.aux_buf[anc.aux_idx]           = aux;
        anc.aux_buf[anc.aux_idx + SZ_TAPS] = aux;

        float shat_out = 0.0f;
        for (int k = 0; k < SZ_TAPS; k++)
            shat_out += anc.shat[k] * anc.aux_buf[anc.aux_idx + k];

        float e_spm = e - shat_out;
        for (int k = 0; k < SZ_TAPS; k++)
            anc.shat[k] += MU_SPM * e_spm * anc.aux_buf[anc.aux_idx + k];

        return 0;
    }

    anc.x_idx = (anc.x_idx == 0) ? (W_TAPS-1) : (anc.x_idx - 1);
    anc.x_buf[anc.x_idx]          = x;
    anc.x_buf[anc.x_idx + W_TAPS] = x;

    anc.sz_idx = (anc.sz_idx == 0) ? (SZ_TAPS-1) : (anc.sz_idx - 1);
    anc.sz_buf[anc.sz_idx]           = x;
    anc.sz_buf[anc.sz_idx + SZ_TAPS] = x;

    float fx_now = 0.0f;
    for (int k = 0; k < SZ_TAPS; k++)
        fx_now += anc.shat[k] * anc.sz_buf[anc.sz_idx + k];

    anc.fx_idx = (anc.fx_idx == 0) ? (W_TAPS-1)  : (anc.fx_idx - 1);
    anc.fx[anc.fx_idx]          = fx_now;
    anc.fx[anc.fx_idx + W_TAPS] = fx_now;

    anc.power = 0.999f * anc.power + 0.001f * fx_now * fx_now;
    float mu_norm = MU / (anc.power * W_TAPS + 1e-8f);

    for (int k = 0; k < W_TAPS; k++)
        anc.w[k] = LEAKY * anc.w[k] + mu_norm * e * anc.fx[anc.fx_idx + k];

    float y = 0.0f;
    for (int k = 0; k < W_TAPS; k++)
        y += anc.w[k] * anc.x_buf[anc.x_idx + k];

    float out_gain = 16.0f;
    float actual_output = -(y * out_gain);

    return float_to_s24(actual_output);
}

// Process one ping-pong buffer
static void process_block(int buf_idx) {
    Xil_DCacheInvalidateRange((UINTPTR)ErrBuf[buf_idx], BYTES_PER_BLOCK);

    for (int i = 0; i < BLOCK_SIZE; i++) {
        int32_t x_raw = noise_next_s24();
        int32_t e_raw = word_to_s24(ErrBuf[buf_idx][i*2 + 0]);

        float x_float = s24_to_float(x_raw);
        float e_float = s24_to_float(e_raw);
        x_energy_accum += (x_float * x_float);
        e_energy_accum += (e_float * e_float);
        sample_count++;

        NoiseBuf[buf_idx][i*2 + 0] = s24_to_i2s_word(x_raw);
        NoiseBuf[buf_idx][i*2 + 1] = 0;

        float  aux = 0.0f;
        int32_t out = 0;

        if (!anc.anc_active) {
            aux = aux_noise_next();
            fxlms_process(x_raw, e_raw, aux);
            out = float_to_s24(aux);
        } else {
            out = fxlms_process(x_raw, e_raw, 0.0f);
        }

        if (out >  8388607) out =  8388607;
        if (out < -8388608) out = -8388608;

        OutBuf[buf_idx][i*2 + 0] = s24_to_i2s_word(out);
        OutBuf[buf_idx][i*2 + 1] = 0;
    }

    Xil_DCacheFlushRange((UINTPTR)NoiseBuf[buf_idx], BYTES_PER_BLOCK);
    Xil_DCacheFlushRange((UINTPTR)OutBuf[buf_idx], BYTES_PER_BLOCK);
}

// Scatter-Gather ring setup
static int setup_sg_rx_ring(XAxiDma *dma, UINTPTR bd_base, u32 bufs[][WORDS_PER_BLOCK], int n_bufs) {
    XAxiDma_BdRing *ring = XAxiDma_GetRxRing(dma);
    XAxiDma_Bd *bd_ptr, *bd_cur;
    XAxiDma_Bd bd_template;
    int status;

    XAxiDma_BdRingIntDisable(ring, XAXIDMA_IRQ_ALL_MASK);
    status = XAxiDma_BdRingSetCoalesce(ring, 1, 1);
    if (status != XST_SUCCESS) return status;

    status = XAxiDma_BdRingCreate(ring, bd_base, bd_base, XAXIDMA_BD_MINIMUM_ALIGNMENT, n_bufs);
    if (status != XST_SUCCESS) return status;
    Xil_DCacheFlushRange(bd_base, n_bufs * XAXIDMA_BD_MINIMUM_ALIGNMENT * 4);

    XAxiDma_BdClear(&bd_template);
    status = XAxiDma_BdRingClone(ring, &bd_template);
    if (status != XST_SUCCESS) return status;

    status = XAxiDma_BdRingAlloc(ring, n_bufs, &bd_ptr);
    if (status != XST_SUCCESS) return status;

    bd_cur = bd_ptr;
    for (int i = 0; i < n_bufs; i++) {
        Xil_DCacheFlushRange((UINTPTR)bufs[i], BYTES_PER_BLOCK);
        Xil_DCacheInvalidateRange((UINTPTR)bufs[i], BYTES_PER_BLOCK);
        status = XAxiDma_BdSetBufAddr(bd_cur, (UINTPTR)bufs[i]);
        status = XAxiDma_BdSetLength(bd_cur, BYTES_PER_BLOCK, ring->MaxTransferLen);
        XAxiDma_BdSetCtrl(bd_cur, 0);
        XAxiDma_BdSetId(bd_cur, i);
        bd_cur = (XAxiDma_Bd *)XAxiDma_BdRingNext(ring, bd_cur);
    }
    Xil_DCacheFlushRange((UINTPTR)bd_ptr, n_bufs * XAXIDMA_BD_MINIMUM_ALIGNMENT * 4);
    status = XAxiDma_BdRingToHw(ring, n_bufs, bd_ptr);
    return XAxiDma_BdRingStart(ring);
}

static int setup_sg_tx_ring(XAxiDma *dma, UINTPTR bd_base, u32 bufs[][WORDS_PER_BLOCK], int n_bufs) {
    XAxiDma_BdRing *ring = XAxiDma_GetTxRing(dma);
    int status;

    XAxiDma_BdRingIntDisable(ring, XAXIDMA_IRQ_ALL_MASK);
    XAxiDma_BdRingSetCoalesce(ring, 1, 1);

    status = XAxiDma_BdRingCreate(ring, bd_base, bd_base, XAXIDMA_BD_MINIMUM_ALIGNMENT, n_bufs);
    if (status != XST_SUCCESS) return status;

    XAxiDma_Bd bd_template;
    XAxiDma_BdClear(&bd_template);
    XAxiDma_BdRingClone(ring, &bd_template);
    return XAxiDma_BdRingStart(ring);
}

// Submit one output buffer to the TX ring
static int submit_tx_block(XAxiDma *dma, int buf_idx) {
    XAxiDma_BdRing *ring = XAxiDma_GetTxRing(dma);
    XAxiDma_Bd *bd_ptr;
    static int first_call = 1;

    if (!first_call) {
        int n_done = 0;
        while (n_done == 0) n_done = XAxiDma_BdRingFromHw(ring, 1, &bd_ptr);
        XAxiDma_BdRingFree(ring, 1, bd_ptr);
    }
    first_call = 0;

    int status = XAxiDma_BdRingAlloc(ring, 1, &bd_ptr);
    if (status != XST_SUCCESS) return status;

    XAxiDma_BdSetBufAddr(bd_ptr, (UINTPTR)OutBuf[buf_idx]);
    XAxiDma_BdSetLength(bd_ptr, BYTES_PER_BLOCK, ring->MaxTransferLen);
    XAxiDma_BdSetCtrl(bd_ptr, XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK);
    XAxiDma_BdSetId(bd_ptr, buf_idx);
    return XAxiDma_BdRingToHw(ring, 1, bd_ptr);
}

static int submit_noise_block(XAxiDma *dma, int buf_idx) {
    XAxiDma_BdRing *ring = XAxiDma_GetTxRing(dma);
    XAxiDma_Bd *bd_ptr;
    static int first_call = 1;

    if (!first_call) {
        int n_done = 0;
        while (n_done == 0) n_done = XAxiDma_BdRingFromHw(ring, 1, &bd_ptr);
        XAxiDma_BdRingFree(ring, 1, bd_ptr);
    }
    first_call = 0;

    int status = XAxiDma_BdRingAlloc(ring, 1, &bd_ptr);
    if (status != XST_SUCCESS) return status;

    XAxiDma_BdSetBufAddr(bd_ptr, (UINTPTR)NoiseBuf[buf_idx]);
    XAxiDma_BdSetLength(bd_ptr, BYTES_PER_BLOCK, ring->MaxTransferLen);
    XAxiDma_BdSetCtrl(bd_ptr, XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK);
    XAxiDma_BdSetId(bd_ptr, buf_idx);
    return XAxiDma_BdRingToHw(ring, 1, bd_ptr);
}

// Wait for one RX block to complete, return which buf index filled
static int wait_rx_block(XAxiDma *dma) {
    XAxiDma_BdRing *ring = XAxiDma_GetRxRing(dma);
    XAxiDma_Bd *bd_ptr;
    int n_done = 0;

    while (n_done == 0) {
        // xemacif_input is GONE from here!
        n_done = XAxiDma_BdRingFromHw(ring, 1, &bd_ptr);
    }

    int buf_idx = (int)XAxiDma_BdGetId(bd_ptr);
    XAxiDma_BdRingFree(ring, 1, bd_ptr);

    XAxiDma_Bd *new_bd;
    if (XAxiDma_BdRingAlloc(ring, 1, &new_bd) == XST_SUCCESS) {
        XAxiDma_BdSetBufAddr(new_bd, (UINTPTR)ErrBuf[buf_idx]);
        XAxiDma_BdSetLength(new_bd, BYTES_PER_BLOCK, ring->MaxTransferLen);
        XAxiDma_BdSetCtrl(new_bd, 0);
        XAxiDma_BdSetId(new_bd, buf_idx);
        XAxiDma_BdRingToHw(ring, 1, new_bd);
    }
    return buf_idx;
}

// Hardware init
static int hw_init(void) {
    int s;

    s = XI2s_Tx_CfgInitialize(&I2StxNoise, XI2s_Tx_LookupConfig(I2S_NOISE_ID), XI2s_Tx_LookupConfig(I2S_NOISE_ID)->BaseAddress);
    if (s != XST_SUCCESS) return s;
    s = XI2s_Tx_CfgInitialize(&I2StxAntinoise, XI2s_Tx_LookupConfig(I2S_ANTINOISE_ID), XI2s_Tx_LookupConfig(I2S_ANTINOISE_ID)->BaseAddress);
    if (s != XST_SUCCESS) return s;
    s = XI2s_Rx_CfgInitialize(&I2SrxErr, XI2s_Rx_LookupConfig(I2S_RX_ERR_ID), XI2s_Rx_LookupConfig(I2S_RX_ERR_ID)->BaseAddress);
    if (s != XST_SUCCESS) return s;

    s = XAxiDma_CfgInitialize(&DmaErrTx, XAxiDma_LookupConfig(DMA_ERR_TX_ID));
    if (s != XST_SUCCESS) return s;
    s = XAxiDma_CfgInitialize(&DmaRef, XAxiDma_LookupConfig(DMA_REF_ID));
    if (s != XST_SUCCESS) return s;

    XAxiDma_IntrDisable(&DmaErrTx, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrDisable(&DmaErrTx, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    XAxiDma_IntrDisable(&DmaRef, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

    XI2s_Tx_SetSclkOutDiv(&I2StxNoise,     12288000, FS);
    XI2s_Tx_SetSclkOutDiv(&I2StxAntinoise, 12288000, FS);
    XI2s_Rx_SetSclkOutDiv(&I2SrxErr, 12288000, FS);
    XI2s_Rx_JustifyEnable(&I2SrxErr, 0);
    XI2s_Tx_JustifyEnable(&I2StxNoise,    0);
    XI2s_Tx_JustifyEnable(&I2StxAntinoise,    0);
    XI2s_Tx_SetChMux(&I2StxNoise, XI2S_TX_CHID0, XI2S_TX_CHMUX_AXIS_01);
    XI2s_Tx_SetChMux(&I2StxAntinoise, XI2S_TX_CHID0, XI2S_TX_CHMUX_AXIS_01);
    XI2s_Rx_SetChMux(&I2SrxErr, 0, XI2S_RX_CHMUX_XI2S_01);
    XI2s_Rx_Enable(&I2SrxErr, 1);
    XI2s_Tx_Enable(&I2StxNoise,    1);
    XI2s_Tx_Enable(&I2StxAntinoise,    1);
    xil_printf("HW init OK\r\n");
    return XST_SUCCESS;
}

int main(void) {
    xil_printf("\r\n=== FXLMS ANC (Serial Telemetry Version) ===\r\n");

    if (hw_init() != XST_SUCCESS) return XST_FAILURE;
    fxlms_init();

    xil_printf("Setting up ERR RX ring...\r\n");
    if (setup_sg_rx_ring(&DmaErrTx, BD_ERR_RX_BASE, ErrBuf, 2) != XST_SUCCESS) return XST_FAILURE;

    xil_printf("Setting up antinoise TX ring...\r\n");
    if (setup_sg_tx_ring(&DmaRef, BD_OUT_TX_BASE, OutBuf, 2) != XST_SUCCESS) return XST_FAILURE;

    xil_printf("Setting up noise TX ring...\r\n");
    if (setup_sg_tx_ring(&DmaErrTx, BD_NOISE_TX_BASE, NoiseBuf, 2) != XST_SUCCESS) return XST_FAILURE;

    for (int b = 0; b < 2; b++) {
        for (int i = 0; i < BLOCK_SIZE; i++) {
            int32_t s = noise_next_s24();
            NoiseBuf[b][i*2 + 0] = s24_to_i2s_word(s);
            NoiseBuf[b][i*2 + 1] = 0;
        }
        Xil_DCacheFlushRange((UINTPTR)NoiseBuf[b], BYTES_PER_BLOCK);
    }

    memset(OutBuf, 0, sizeof(OutBuf));
    Xil_DCacheFlushRange((UINTPTR)OutBuf, sizeof(OutBuf));
    submit_tx_block(&DmaRef, 0);
    submit_tx_block(&DmaRef, 1);
    submit_noise_block(&DmaErrTx, 0);
    submit_noise_block(&DmaErrTx, 1);

    xil_printf("All rings running — entering ANC loop\r\n");

    u32 block_count = 0;

	while (1) {
		int proc_idx = wait_rx_block(&DmaErrTx);
		process_block(proc_idx);

		if (snap_state == 0) {
			if (block_count % 500 == 0) {
				snap_state = 1;
				snap_idx = 0;
			}
		}
		else if (snap_state == 1) {
			for(int i=0; i < BLOCK_SIZE; i++) {
				if (snap_idx < SNAPSHOT_LEN) {
					ref_snap[snap_idx] = s24_to_float(word_to_s24(NoiseBuf[proc_idx][i*2]));
					err_snap[snap_idx] = s24_to_float(word_to_s24(ErrBuf[proc_idx][i*2]));
					snap_idx++;
				}
			}
			if (snap_idx >= SNAPSHOT_LEN) {
				snap_state = 2;
				tx_idx = 0;
			}
		}
		else if (snap_state == 2) {
			if (block_count % 3 == 0) {
				if (tx_idx == 0) {
					printf("SNAP_START\n");
				} else if (tx_idx <= SNAPSHOT_LEN) {
					printf("DATA:%.4f,%.4f\n", ref_snap[tx_idx-1], err_snap[tx_idx-1]);
				} else if (tx_idx == SNAPSHOT_LEN + 1) {
					printf("SNAP_END\n");
					snap_state = 0;
				}
				tx_idx++;
			}
		}

		if (submit_tx_block(&DmaRef, proc_idx) != XST_SUCCESS) xil_printf("Antinoise TX fail\r\n");
		if (submit_noise_block(&DmaErrTx, proc_idx) != XST_SUCCESS) xil_printf("Noise TX fail\r\n");

		block_count++;

        if (!anc.anc_active) {
            anc.train_blocks++;
            if (anc.train_blocks >= SPM_TRAIN_BLOCKS) {
                float shat_energy = 0.0f;
                for (int k = 0; k < SZ_TAPS; k++) shat_energy += anc.shat[k] * anc.shat[k];

                if (shat_energy > SPM_ENERGY_THRESH) {
                    anc.anc_active = 1;
                    xil_printf("SPM converged ANC ON\r\n");
                } else {
                    anc.train_blocks = 0;
                }
            }
        }
    }

    return 0;
}
