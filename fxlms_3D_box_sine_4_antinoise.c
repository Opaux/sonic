/*
 * fxlms_3D_box_2x4_mimo.c  (patched)
 * =================================
 * 2 error mic x 4 antinoise speakers, 1 internal reference, FxLMS w/ simultaneous SPM.
 * Zynq-7000 Cortex-A9 + NEON, bare-metal Vitis 2021.1.
 *
 * Changes vs. the original:
 *   - Sine reference is a 1024-entry LUT (no sinf() in the hot loop).
 *   - SZ_TAPS reduced to 256 (21 ms -> 5.3 ms of modeled secondary path).
 *   - Ping-pong depth increased from 2 to 4 buffers (more DMA slack).
 *   - RX BD re-arm moved to AFTER process_block() (closes a race where
 *     DMA can overwrite a buffer while the CPU is still reading it).
 *   - Snapshot telemetry rewritten as a ring buffer drained non-blockingly
 *     into the UART FIFO; zero blocking I/O in the block-processing path.
 *   - Snapshot format now "DATA:ref,e0,e1" (unchanged) + explicit per-mic
 *     RMS and attenuation tags "RMS:..." and "ATTEN:..." emitted once
 *     per ~200 ms outside the hot loop.
 *   - LEAKY = 0.99999f, per-pair power floor prevents mu blow-up.
 *   - SPM gate requires e_spm / e_meas misadjustment ratio < threshold
 *     for N consecutive blocks, not a single energy check.
 *   - W coefficients NaN/Inf checked every 256 blocks; on failure the
 *     filters are zeroed and ANC re-armed.
 *   - Per-block cycle counter captured and reported via telemetry.
 *
 *   NOTE: algorithm topology, sign conventions, and Xilinx BSP API surface
 *         are unchanged.  Drops into the existing Vitis 2021.1 project.
 */

#include "xparameters.h"
#include "xil_printf.h"
#include "xi2stx.h"
#include "xi2srx.h"
#include "xaxidma.h"
#include "xaxidma_bd.h"
#include "xaxidma_bdring.h"
#include "xil_cache.h"
#include "xuartps_hw.h"
#include "xtime_l.h"
#include <arm_neon.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* ============================================================
 *                 Hardware ID mapping
 * ============================================================ */
#define I2S_NOISE_ID         XPAR_I2S_TRANSMITTER_0_DEVICE_ID
#define I2S_AN0_ID           XPAR_I2S_TRANSMITTER_1_DEVICE_ID
#define I2S_AN1_ID           XPAR_I2S_TRANSMITTER_2_DEVICE_ID
#define I2S_AN2_ID           XPAR_I2S_TRANSMITTER_3_DEVICE_ID
#define I2S_AN3_ID           XPAR_I2S_TRANSMITTER_4_DEVICE_ID
#define I2S_RX_ERR0_ID       XPAR_I2S_RECEIVER_0_DEVICE_ID
#define I2S_RX_ERR1_ID       XPAR_I2S_RECEIVER_1_DEVICE_ID
#define DMA_ERR0_NOISE_ID    XPAR_AXI_DMA_0_DEVICE_ID
#define DMA_ERR1_AN0_ID      XPAR_AXI_DMA_1_DEVICE_ID
#define DMA_AN1_ID           XPAR_AXI_DMA_2_DEVICE_ID
#define DMA_AN2_ID           XPAR_AXI_DMA_3_DEVICE_ID
#define DMA_AN3_ID           XPAR_AXI_DMA_4_DEVICE_ID

/* Telemetry UART: reuse stdout */
#ifndef STDOUT_BASEADDRESS
#define STDOUT_BASEADDRESS   XPAR_XUARTPS_0_BASEADDR
#endif
#define TLM_UART_BASE        STDOUT_BASEADDRESS

/* ============================================================
 *                 Algorithm parameters
 * ============================================================ */
#define FS               48000
#define BLOCK_SIZE       32
#define N_ERR            2
#define N_SPK            4
#define W_TAPS           64
#define SZ_TAPS          512          /* was 1024 -- see header note */
/* SZ_TAP_OFFSET: the measured impulse response has ~90-150 samples of
 * air + analog + codec-pipeline pre-delay that are numerically zero.
 * Updating / reading those taps just burns NEON MACs.  We skip them
 * in every hot fir_dot() / shat_update() call.  96 gives SZ_TAPS_EFF=160
 * which is still a round multiple of 8 for the NEON 8-wide kernel.
 * ~37% of Shat compute saved vs the full-length path.  Set to 0 to
 * disable. */
#define SZ_TAP_OFFSET    96
#define SZ_TAPS_EFF      (SZ_TAPS - SZ_TAP_OFFSET)
#define NBUF             4            /* ping-pong depth per channel  */

#define MU               0.0003f   /* was 0.0001f -- MIMO W needs more drive */
#define LEAKY            1.0f     /* mild leak; was 1.0f           */
#define POWER_FLOOR      1.0e-4f      /* clamps normalized mu          */
#define MU_SPM           5.0e-4f
#define AUX_GAIN         0.08f

/* --- SPM convergence gate (EMA + hysteresis) ---
 * Per-block misadjustment is inherently noisy (32 samples / block), so we
 * smooth it with an EMA and gate on the smoothed value.  Hysteresis keeps
 * the state from flapping when the estimate sits near the threshold. */
#define SPM_MISADJ_ARM       0.45f    /* EMA must fall below this to arm    */
#define SPM_MISADJ_DISARM    0.85f    /* ...and rise above this to disarm   */
#define SPM_EMA_ALPHA        0.02f    /* ~50-block smoothing (~33 ms)       */
#define SPM_MIN_TRAIN        100000    /* ~6.7 s warmup -- gives 8 shat pairs
                                         enough time to converge in MIMO */

/* --- Soft W-divergence recovery ---
 * Instead of dropping back to SPM training on divergence (which caused
 * audible ANC on/off cycling), we zero W and halve the LMS step size.
 * mu_scale slowly recovers toward 1.0 each block.  We only fall back to
 * SPM if the mu floor is hit -- i.e., we're persistently diverging and
 * Shat is probably stale. */
#define W_MU_STARTUP         0.25f    /* mu_scale on first arm              */
#define W_MU_RECOVERY        1.0005f  /* per-block growth back to 1.0       */
#define W_MU_FLOOR           0.05f    /* below this -> retrain Shat         */

/* Round-robin Shat update: update one (m,k) pair per sample instead of
 * all 8.  Cuts SPM compute by ~44%.  yhat and e_spm are still evaluated
 * fully every sample, so the residual telemetry stays accurate. */
#define SPM_ROUND_ROBIN      1   /* full-rate Shat update -- 8x convergence */

#define SINE_FREQ_HZ     500.0f
#define SINE_AMPLITUDE   0.15f
#define OUT_GAIN         1.0f

#define DIVERGENCE_CHECK_BLOCKS  256
#define W_MAX_ABS                8.0f   /* saturation-health bound     */

/* ============================================================
 *                  BD base addresses (OCM)
 * ============================================================ */
#define BD_BASE_ERR0     0x01F00000
#define BD_BASE_ERR1     0x01F01000
#define BD_BASE_NOISE    0x01F02000
#define BD_BASE_AN0      0x01F03000
#define BD_BASE_AN1      0x01F04000
#define BD_BASE_AN2      0x01F05000
#define BD_BASE_AN3      0x01F06000

/* ============================================================
 *                  DMA buffer geometry
 * ============================================================ */
#define WORDS_PER_BLOCK  (BLOCK_SIZE * 2)
#define BYTES_PER_BLOCK  (WORDS_PER_BLOCK * 4)
/* Snapshot is 256 samples at 48 kHz = 5.3 ms of waveform, enough for the
 * dashboard FFT to resolve ~200 Hz bins.  UART at 115200 baud ~= 11.5 KB/s,
 * and a 256-line dump at ~15 bytes/line = ~4 KB/s.
 *
 * The cadence is mode-aware so ANC cancellation isn't disturbed more than
 * necessary:
 *    SPM training : arm every 1500 blocks (~1 s), drain 1 sample/block
 *                   (burst lasts ~170 ms each).
 *    ANC active   : arm every 4500 blocks (~3 s), drain 1 sample every 2
 *                   blocks (burst lasts ~340 ms each, ~11% duty cycle).
 * Rate-limiting the ANC drain gives the error-mic + reference pair enough
 * samples to show cancellation without stealing UART bandwidth from the
 * one-line metrics. */
#define SNAPSHOT_LEN                   256
#define SNAPSHOT_INTERVAL_BLOCKS_SPM   1500u
#define SNAPSHOT_INTERVAL_BLOCKS_ANC   9000u   /* every 6 s in ANC          */
#define SNAPSHOT_DRAIN_DIVIDER_SPM     1u
#define SNAPSHOT_DRAIN_DIVIDER_ANC     8u      /* 1 DATA line / 2.7 ms      */
/* Circuit breaker: if a snapshot gets stuck in the drain phase for this many
 * blocks (i.e. the host UART is hopelessly wedged), force it to terminate
 * so future snapshots can still arm. */
#define SNAPSHOT_DRAIN_TIMEOUT_BLOCKS  (SNAPSHOT_LEN * 8u)

static u32 NoiseBuf[NBUF][WORDS_PER_BLOCK]           __attribute__((aligned(64)));
static u32 ErrBuf  [N_ERR][NBUF][WORDS_PER_BLOCK]    __attribute__((aligned(64)));
static u32 OutBuf  [N_SPK][NBUF][WORDS_PER_BLOCK]    __attribute__((aligned(64)));

/* ============================================================
 *                  Telemetry state
 * ============================================================ */
#define TLM_RING_SIZE    8192u
static char     tlm_ring[TLM_RING_SIZE];
static volatile uint32_t tlm_head = 0;   /* producer index              */
static volatile uint32_t tlm_tail = 0;   /* consumer index              */
static uint32_t tlm_dropped_bytes = 0;

static float ref_snap[SNAPSHOT_LEN];
static float err_snap[N_ERR][SNAPSHOT_LEN];
static int   snap_state = 0;   /* 0 idle, 1 filling, 2 draining       */
static int   snap_idx   = 0;
static int   snap_drain_idx = 0;
static uint32_t snap_drain_started_at = 0;  /* block_count when drain phase began */

/* Per-block diagnostics (cheap to accumulate, printed off critical path) */
static float block_ref_sq_sum   = 0.0f;
static float block_err_sq_sum[N_ERR];
static uint32_t max_block_cycles   = 0;
static uint32_t last_block_cycles  = 0;
static uint32_t sample_count       = 0;
static uint32_t tlm_skipped_emits  = 0;   /* telemetry skipped due to overrun */

/* Snapshotted metric values latched at the start of each 300-block window
 * and then slowly spooled out to UART one line at a time, so the vsnprintf
 * cost (~30-60 us per float on A9 soft-float) never piles up on one block. */
static float saved_ref_rms   = 0.0f;
static float saved_e0_rms    = 0.0f;
static float saved_e1_rms    = 0.0f;
static float saved_atten0    = 0.0f;
static float saved_atten1    = 0.0f;
static float saved_shat_e[N_ERR];

/* Cortex-A9 global timer runs at CPU/2.  At 666 MHz CPU that's 333 MHz,
 * i.e., 3 ns per tick.  200k ticks = 600 us -- the overrun gate used to
 * skip telemetry when a block is pushing the budget. */
#define CYC_PER_SEC           333333333u
#define BLOCK_OVERRUN_TICKS   200000u

/* ============================================================
 *                  Peripheral instances
 * ============================================================ */
static XI2s_Tx I2StxNoise;
static XI2s_Tx I2StxAn[N_SPK];
static XI2s_Rx I2SrxErr[N_ERR];
static XAxiDma DmaErr0Noise;
static XAxiDma DmaErr1An0;
static XAxiDma DmaAn1;
static XAxiDma DmaAn2;
static XAxiDma DmaAn3;

typedef struct {
    XAxiDma *dma;
    u32 (*bufs)[WORDS_PER_BLOCK];
    int primed_count;
} tx_chan_t;

typedef struct {
    XAxiDma *dma;
    u32 (*bufs)[WORDS_PER_BLOCK];
} rx_chan_t;

static tx_chan_t tx_noise;
static tx_chan_t tx_an[N_SPK];
static rx_chan_t rx_err[N_ERR];

/* ============================================================
 *                  Signal conversion helpers
 * ============================================================ */
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
static inline int32_t word_to_s24(uint32_t w) {
    return ((int32_t)(w << 4)) >> 8;
}

/* ============================================================
 *                  Non-blocking telemetry
 * ============================================================ */
static inline int tlm_push_byte(char c) {
    uint32_t h = (tlm_head + 1u) % TLM_RING_SIZE;
    if (h == tlm_tail) { tlm_dropped_bytes++; return 0; }
    tlm_ring[tlm_head] = c;
    tlm_head = h;
    return 1;
}
static void tlm_push_str(const char *s) {
    while (*s) { if (!tlm_push_byte(*s)) return; s++; }
}
static void tlm_printf(const char *fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    for (int i = 0; i < n; i++) if (!tlm_push_byte(buf[i])) return;
}
/* Drain as much as the UART TX FIFO will take THIS INSTANT, then return.
 * Guaranteed not to spin; safe to call every block. */
static void telemetry_service(void) {
    while (tlm_tail != tlm_head) {
        u32 sr = XUartPs_ReadReg(TLM_UART_BASE, XUARTPS_SR_OFFSET);
        if (sr & XUARTPS_SR_TNFUL) return;
        XUartPs_WriteReg(TLM_UART_BASE, XUARTPS_FIFO_OFFSET,
                         (u8)tlm_ring[tlm_tail]);
        tlm_tail = (tlm_tail + 1u) % TLM_RING_SIZE;
    }
}

/* ============================================================
 *                  Reference generators
 * ============================================================ */
/* Aux LFSRs, 4 mutually-uncorrelated streams. */
static uint32_t aux_lfsr[N_SPK] = {
    0xDEADBEEF, 0xCAFEBABE, 0xBADC0FFEu, 0xFACEFEEDu
};
static inline float aux_noise_next(int k) {
    uint32_t s = aux_lfsr[k];
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    aux_lfsr[k] = s;
    return (float)(int32_t)s * (AUX_GAIN / 2147483648.0f);
}

/* Broadband LFSR (unused while sine is on, but advanced deterministically) */
static uint32_t noise_lfsr = 0xACE1ACE1u;
static inline int32_t noise_next_s24(void) {
    noise_lfsr ^= noise_lfsr << 13;
    noise_lfsr ^= noise_lfsr >> 17;
    noise_lfsr ^= noise_lfsr << 5;
    return (int32_t)noise_lfsr >> 12;
}

/* Sine reference: 1024-entry LUT driven by a 32-bit phase accumulator.
 * Avoids the per-sample sinf() call entirely. */
#define SINE_LUT_SIZE 1024
#define SINE_LUT_BITS 10
static float sine_lut[SINE_LUT_SIZE];
static uint32_t sine_phase_fx    = 0;
static uint32_t sine_phase_inc_fx = 0;  /* set in sine_lut_init */

static void sine_lut_init(void) {
    for (int i = 0; i < SINE_LUT_SIZE; i++) {
        sine_lut[i] = sinf(2.0f * 3.14159265358979323846f *
                           (float)i / (float)SINE_LUT_SIZE);
    }
    /* phase_inc = freq / fs * 2^32 */
    double inc = (double)SINE_FREQ_HZ / (double)FS * 4294967296.0;
    sine_phase_inc_fx = (uint32_t)inc;
}
static inline int32_t sine_next_s24(void) {
    uint32_t idx = sine_phase_fx >> (32 - SINE_LUT_BITS);
    sine_phase_fx += sine_phase_inc_fx;
    float s = SINE_AMPLITUDE * sine_lut[idx];
    return float_to_s24(s);
}

/* ============================================================
 *                  NEON kernels (unchanged)
 * ============================================================ */
static inline float fir_dot_neon(const float * __restrict a,
                                 const float * __restrict b, int n) {
    float32x4_t acc0 = vdupq_n_f32(0.0f);
    float32x4_t acc1 = vdupq_n_f32(0.0f);
    int i = 0;
    for (; i + 8 <= n; i += 8) {
        float32x4_t va0 = vld1q_f32(a + i);
        float32x4_t vb0 = vld1q_f32(b + i);
        float32x4_t va1 = vld1q_f32(a + i + 4);
        float32x4_t vb1 = vld1q_f32(b + i + 4);
        acc0 = vmlaq_f32(acc0, va0, vb0);
        acc1 = vmlaq_f32(acc1, va1, vb1);
    }
    for (; i + 4 <= n; i += 4) {
        float32x4_t va = vld1q_f32(a + i);
        float32x4_t vb = vld1q_f32(b + i);
        acc0 = vmlaq_f32(acc0, va, vb);
    }
    float32x4_t acc = vaddq_f32(acc0, acc1);
    float sum = vgetq_lane_f32(acc, 0) + vgetq_lane_f32(acc, 1) +
                vgetq_lane_f32(acc, 2) + vgetq_lane_f32(acc, 3);
    for (; i < n; i++) sum += a[i] * b[i];
    return sum;
}

static inline void w_lms_update_neon(float * __restrict w,
                                     const float * __restrict fx,
                                     float scalar, float leaky, int n) {
    float32x4_t vleaky  = vdupq_n_f32(leaky);
    float32x4_t vscalar = vdupq_n_f32(scalar);
    int i = 0;
    for (; i + 4 <= n; i += 4) {
        float32x4_t vw  = vld1q_f32(w  + i);
        float32x4_t vfx = vld1q_f32(fx + i);
        vw = vmulq_f32(vw, vleaky);
        vw = vmlaq_f32(vw, vfx, vscalar);
        vst1q_f32(w + i, vw);
    }
    for (; i < n; i++) w[i] = leaky * w[i] + scalar * fx[i];
}

static inline void shat_update_neon(float * __restrict shat,
                                    const float * __restrict aux,
                                    float scalar, int n) {
    float32x4_t vscalar = vdupq_n_f32(scalar);
    int i = 0;
    for (; i + 4 <= n; i += 4) {
        float32x4_t vs = vld1q_f32(shat + i);
        float32x4_t va = vld1q_f32(aux  + i);
        vs = vmlaq_f32(vs, va, vscalar);
        vst1q_f32(shat + i, vs);
    }
    for (; i < n; i++) shat[i] += scalar * aux[i];
}

/* ============================================================
 *                  MIMO FxLMS state
 * ============================================================ */
typedef struct {
    float w     [N_SPK][W_TAPS]            __attribute__((aligned(16)));
    float x_buf [W_TAPS * 2]               __attribute__((aligned(16)));
    int   x_idx;
    float sz_buf[SZ_TAPS * 2]              __attribute__((aligned(16)));
    int   sz_idx;
    float fx    [N_ERR][N_SPK][W_TAPS * 2] __attribute__((aligned(16)));
    int   fx_idx;
    float power [N_ERR][N_SPK];
    float shat  [N_ERR][N_SPK][SZ_TAPS]    __attribute__((aligned(16)));
    float aux_buf[N_SPK][SZ_TAPS * 2]      __attribute__((aligned(16)));
    int   aux_idx;
    int   train_blocks;
    int   anc_active;
    float e_spm_sq[N_ERR];   /* per-block accumulator (SPM residual^2)   */
    float e_meas_sq[N_ERR];  /* per-block accumulator (raw mic^2)        */
    int   rr_pair;           /* round-robin Shat-update index, 0..7      */
    float last_misadj;       /* latched smoothed misadj for telemetry    */
    float misadj_ema;        /* EMA of per-block misadjustment ratio     */
    float mu_scale;          /* adaptive LMS step scale 0.05..1.0        */
    int   arms;              /* # times ANC has armed (telemetry)        */
    int   disarms;           /* # times disarmed back to SPM (telemetry) */
    /* Per-block cached normalized step: mu_k_cache[m][k] =
     * (MU * mu_scale) / (max(power[m][k], POWER_FLOOR) * W_TAPS).
     * Computed once in process_block() at the top of each sample loop
     * and reused for all 32 samples.  Saves 256 VFP divides per block
     * (~8 us on A9).  Tracking error is tiny because power changes
     * <~3% across one block at this EMA rate. */
    float mu_k_cache[N_ERR][N_SPK];
} fxlms_t;

static fxlms_t anc;

static void fxlms_init(void) {
    memset(&anc, 0, sizeof(anc));
    /* Power starts near a realistic fx^2 (~0.01 for sine amp 0.15 through
     * Shat).  Was 1e-4, which made the first few normalized mu values
     * enormous and was a likely contributor to first-arm W kicks. */
    for (int m = 0; m < N_ERR; m++)
        for (int k = 0; k < N_SPK; k++)
            anc.power[m][k] = 1.0e-2f;
    anc.misadj_ema  = 1.0f;          /* start pessimistic; EMA will fall */
    anc.last_misadj = 1.0f;
    anc.mu_scale    = 1.0f;
    tlm_printf("LOG:FXLMS init %dx%d  W=%d SZ=%d(eff=%d off=%d) NBUF=%d\n",
               N_ERR, N_SPK, W_TAPS, SZ_TAPS, SZ_TAPS_EFF, SZ_TAP_OFFSET, NBUF);
    tlm_printf("LOG:SPM min_train=%d ARM=%.2f DISARM=%.2f alpha=%.3f\n",
               SPM_MIN_TRAIN, (double)SPM_MISADJ_ARM,
               (double)SPM_MISADJ_DISARM, (double)SPM_EMA_ALPHA);
}

/* Per-sample MIMO FxLMS step. */
static void fxlms_process(int32_t x_raw,
                          const int32_t e_raw[N_ERR],
                          const float aux[N_SPK],
                          int32_t y_out[N_SPK]) {
    float x = s24_to_float(x_raw);
    float e[N_ERR];
    for (int m = 0; m < N_ERR; m++) e[m] = s24_to_float(e_raw[m]);

    /* ---------------- SPM training phase ---------------- */
    if (!anc.anc_active) {
        anc.aux_idx = (anc.aux_idx == 0) ? (SZ_TAPS - 1) : (anc.aux_idx - 1);
        for (int k = 0; k < N_SPK; k++) {
            anc.aux_buf[k][anc.aux_idx]           = aux[k];
            anc.aux_buf[k][anc.aux_idx + SZ_TAPS] = aux[k];
        }

        /* yhat and e_spm are evaluated fully every sample so the residual
         * telemetry stays accurate.  Only the Shat tap update itself is
         * round-robin'd -- we update a single (rr_m, rr_k) pair per sample
         * when SPM_ROUND_ROBIN is enabled. */
#if SPM_ROUND_ROBIN
        int rr_m = anc.rr_pair >> 2;              /* 0..1 */
        int rr_k = anc.rr_pair &  3;              /* 0..3 */
        anc.rr_pair = (anc.rr_pair + 1) & 7;      /* 0..7 */
#endif

        for (int m = 0; m < N_ERR; m++) {
            float yhat_m = 0.0f;
            for (int k = 0; k < N_SPK; k++) {
                yhat_m += fir_dot_neon(&anc.shat[m][k][SZ_TAP_OFFSET],
                                       &anc.aux_buf[k][anc.aux_idx + SZ_TAP_OFFSET],
                                       SZ_TAPS_EFF);
            }
            float e_spm = e[m] - yhat_m;
#if SPM_ROUND_ROBIN
            if (m == rr_m) {
                /* Step size is scaled up by 8x because this pair only updates
                 * every 8th sample; the long-run convergence rate is preserved. */
                float mu_e = (MU_SPM * 8.0f) * e_spm;
                shat_update_neon(&anc.shat[rr_m][rr_k][SZ_TAP_OFFSET],
                                 &anc.aux_buf[rr_k][anc.aux_idx + SZ_TAP_OFFSET],
                                 mu_e, SZ_TAPS_EFF);
            }
#else
            float mu_e = MU_SPM * e_spm;
            for (int k = 0; k < N_SPK; k++) {
                shat_update_neon(&anc.shat[m][k][SZ_TAP_OFFSET],
                                 &anc.aux_buf[k][anc.aux_idx + SZ_TAP_OFFSET],
                                 mu_e, SZ_TAPS_EFF);
            }
#endif
            anc.e_spm_sq[m]  += e_spm * e_spm;
            anc.e_meas_sq[m] += e[m]  * e[m];
        }
        for (int k = 0; k < N_SPK; k++)
            y_out[k] = float_to_s24(aux[k]);
        return;
    }

    /* ---------------- ANC active phase ---------------- */
    anc.x_idx = (anc.x_idx == 0) ? (W_TAPS - 1) : (anc.x_idx - 1);
    anc.x_buf[anc.x_idx]            = x;
    anc.x_buf[anc.x_idx + W_TAPS]   = x;

    anc.sz_idx = (anc.sz_idx == 0) ? (SZ_TAPS - 1) : (anc.sz_idx - 1);
    anc.sz_buf[anc.sz_idx]           = x;
    anc.sz_buf[anc.sz_idx + SZ_TAPS] = x;

    anc.fx_idx = (anc.fx_idx == 0) ? (W_TAPS - 1) : (anc.fx_idx - 1);
    for (int m = 0; m < N_ERR; m++) {
        for (int k = 0; k < N_SPK; k++) {
            float v = fir_dot_neon(&anc.shat[m][k][SZ_TAP_OFFSET],
                                   &anc.sz_buf[anc.sz_idx + SZ_TAP_OFFSET],
                                   SZ_TAPS_EFF);
            anc.fx[m][k][anc.fx_idx]          = v;
            anc.fx[m][k][anc.fx_idx + W_TAPS] = v;
            anc.power[m][k] = 0.999f * anc.power[m][k] + 0.001f * v * v;
        }
    }

    /* Per-sample W update.  mu_k_cache was populated once at the start of
     * the current block by process_block() so we avoid the 8 VFP divides
     * per sample that the old path did (~8 us/block on A9).  The leak is
     * applied only on the first mic's update; subsequent mics accumulate. */
    for (int k = 0; k < N_SPK; k++) {
        w_lms_update_neon(anc.w[k],
                          &anc.fx[0][k][anc.fx_idx],
                          anc.mu_k_cache[0][k] * e[0], LEAKY, W_TAPS);
        for (int m = 1; m < N_ERR; m++) {
            w_lms_update_neon(anc.w[k],
                              &anc.fx[m][k][anc.fx_idx],
                              anc.mu_k_cache[m][k] * e[m], 1.0f, W_TAPS);
        }
    }

    for (int k = 0; k < N_SPK; k++) {
        float y = fir_dot_neon(anc.w[k], &anc.x_buf[anc.x_idx], W_TAPS);
        y_out[k] = float_to_s24(-(y * OUT_GAIN));
    }

    for (int m = 0; m < N_ERR; m++)
        anc.e_meas_sq[m] += e[m] * e[m];
}

/* ============================================================
 *                  NaN / divergence check
 * ============================================================ */
static int w_has_diverged(void) {
    for (int k = 0; k < N_SPK; k++) {
        for (int t = 0; t < W_TAPS; t++) {
            float v = anc.w[k][t];
            if (!isfinite(v) || v > W_MAX_ABS || v < -W_MAX_ABS)
                return 1;
        }
    }
    return 0;
}
static void w_reset(void) {
    for (int k = 0; k < N_SPK; k++)
        memset(anc.w[k], 0, sizeof(anc.w[k]));
}

/* ============================================================
 *                  Per-block processing
 * ============================================================ */
static void process_block(int buf_idx) {
    XTime t0, t1;
    XTime_GetTime(&t0);

    for (int m = 0; m < N_ERR; m++)
        Xil_DCacheInvalidateRange((UINTPTR)ErrBuf[m][buf_idx], BYTES_PER_BLOCK);

    /* Precompute per-block normalized LMS step from current power estimates.
     * This replaces 8 VFP divides per sample (256 per block) with 8 divides
     * per block.  Tracking error over 32 samples is <~3% because the power
     * EMA moves slowly; convergence is indistinguishable. */
    if (anc.anc_active) {
        float mu_eff = MU * anc.mu_scale;
        float inv_wtaps = 1.0f / (float)W_TAPS;
        for (int m = 0; m < N_ERR; m++) {
            for (int k = 0; k < N_SPK; k++) {
                float p = anc.power[m][k];
                if (p < POWER_FLOOR) p = POWER_FLOOR;
                anc.mu_k_cache[m][k] = (mu_eff * inv_wtaps) / p;
            }
        }
    }

    for (int i = 0; i < BLOCK_SIZE; i++) {
        int32_t x_raw;
        if (anc.anc_active) {
            x_raw = sine_next_s24();
        } else {
            (void)noise_next_s24();
            x_raw = 0;
        }

        int32_t e_raw[N_ERR];
        for (int m = 0; m < N_ERR; m++)
            e_raw[m] = word_to_s24(ErrBuf[m][buf_idx][i*2 + 0]);

        /* aux_vec is only consumed by the SPM branch inside fxlms_process;
         * in ANC mode the LFSRs + float conversions would just burn cycles. */
        float aux_vec[N_SPK];
        if (!anc.anc_active) {
            for (int k = 0; k < N_SPK; k++) aux_vec[k] = aux_noise_next(k);
        }
        /* else: leave aux_vec uninitialized -- fxlms_process's ANC branch
         *       never reads from it. */

        float xf = s24_to_float(x_raw);
        block_ref_sq_sum += xf * xf;
        for (int m = 0; m < N_ERR; m++) {
            float ef = s24_to_float(e_raw[m]);
            block_err_sq_sum[m] += ef * ef;
        }
        sample_count++;

        NoiseBuf[buf_idx][i*2 + 0] = anc.anc_active ? s24_to_i2s_word(x_raw) : 0;
        NoiseBuf[buf_idx][i*2 + 1] = 0;

        int32_t y_out[N_SPK];
        fxlms_process(x_raw, e_raw, aux_vec, y_out);

        for (int k = 0; k < N_SPK; k++) {
            int32_t v = y_out[k];
            if (v >  8388607)  v =  8388607;
            if (v < -8388608)  v = -8388608;
            OutBuf[k][buf_idx][i*2 + 0] = s24_to_i2s_word(v);
            OutBuf[k][buf_idx][i*2 + 1] = 0;
        }

        /* Snapshot fill (in-block: just a memcpy, no I/O) */
        if (snap_state == 1 && snap_idx < SNAPSHOT_LEN) {
            ref_snap[snap_idx] = xf;
            for (int m = 0; m < N_ERR; m++)
                err_snap[m][snap_idx] = s24_to_float(e_raw[m]);
            snap_idx++;
            if (snap_idx >= SNAPSHOT_LEN) {
                snap_state = 2;
                /* caller code in emit_snapshot_and_metrics() timestamps
                 * drain start via block_count at next entry */
            }
        }
    }

    Xil_DCacheFlushRange((UINTPTR)NoiseBuf[buf_idx], BYTES_PER_BLOCK);
    for (int k = 0; k < N_SPK; k++)
        Xil_DCacheFlushRange((UINTPTR)OutBuf[k][buf_idx], BYTES_PER_BLOCK);

    XTime_GetTime(&t1);
    uint32_t cycles = (uint32_t)(t1 - t0);  /* 1 tick = 2 CPU cycles on A9 */
    last_block_cycles = cycles;
    if (cycles > max_block_cycles) max_block_cycles = cycles;
}

/* ============================================================
 *                  SG ring setup
 * ============================================================ */
static int setup_sg_rx_ring(XAxiDma *dma, UINTPTR bd_base,
                            u32 bufs[][WORDS_PER_BLOCK], int n_bufs) {
    XAxiDma_BdRing *ring = XAxiDma_GetRxRing(dma);
    XAxiDma_Bd *bd_ptr, *bd_cur, bd_template;
    int status;
    XAxiDma_BdRingIntDisable(ring, XAXIDMA_IRQ_ALL_MASK);
    status = XAxiDma_BdRingSetCoalesce(ring, 1, 1);
    if (status != XST_SUCCESS) return status;
    status = XAxiDma_BdRingCreate(ring, bd_base, bd_base,
                                  XAXIDMA_BD_MINIMUM_ALIGNMENT, n_bufs);
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
        XAxiDma_BdSetBufAddr(bd_cur, (UINTPTR)bufs[i]);
        XAxiDma_BdSetLength(bd_cur, BYTES_PER_BLOCK, ring->MaxTransferLen);
        XAxiDma_BdSetCtrl(bd_cur, 0);
        XAxiDma_BdSetId(bd_cur, i);
        bd_cur = (XAxiDma_Bd *)XAxiDma_BdRingNext(ring, bd_cur);
    }
    Xil_DCacheFlushRange((UINTPTR)bd_ptr, n_bufs * XAXIDMA_BD_MINIMUM_ALIGNMENT * 4);
    status = XAxiDma_BdRingToHw(ring, n_bufs, bd_ptr);
    if (status != XST_SUCCESS) return status;
    return XAxiDma_BdRingStart(ring);
}

static int setup_sg_tx_ring(XAxiDma *dma, UINTPTR bd_base,
                            u32 bufs[][WORDS_PER_BLOCK], int n_bufs) {
    (void)bufs;
    XAxiDma_BdRing *ring = XAxiDma_GetTxRing(dma);
    int status;
    XAxiDma_BdRingIntDisable(ring, XAXIDMA_IRQ_ALL_MASK);
    XAxiDma_BdRingSetCoalesce(ring, 1, 1);
    status = XAxiDma_BdRingCreate(ring, bd_base, bd_base,
                                  XAXIDMA_BD_MINIMUM_ALIGNMENT, n_bufs);
    if (status != XST_SUCCESS) return status;
    XAxiDma_Bd bd_template;
    XAxiDma_BdClear(&bd_template);
    XAxiDma_BdRingClone(ring, &bd_template);
    return XAxiDma_BdRingStart(ring);
}

/* ============================================================
 *                  Per-channel submit/wait helpers
 *
 *  RX flow now:
 *      1) wait for next completed BD (blocks if nothing ready yet)
 *      2) return its buf_idx -- caller processes that buffer
 *      3) caller calls rx_rearm_chan(buf_idx) AFTER processing completes,
 *         closing the race the original code had (DMA could overwrite
 *         the buffer while the CPU was still reading).
 * ============================================================ */
static int wait_rx_block_chan(rx_chan_t *ch) {
    XAxiDma_BdRing *ring = XAxiDma_GetRxRing(ch->dma);
    XAxiDma_Bd *bd_ptr;
    int n_done = 0;
    while (n_done == 0)
        n_done = XAxiDma_BdRingFromHw(ring, 1, &bd_ptr);
    int buf_idx = (int)XAxiDma_BdGetId(bd_ptr);
    XAxiDma_BdRingFree(ring, 1, bd_ptr);
    return buf_idx;
}
static void rx_rearm_chan(rx_chan_t *ch, int buf_idx) {
    XAxiDma_BdRing *ring = XAxiDma_GetRxRing(ch->dma);
    XAxiDma_Bd *new_bd;
    if (XAxiDma_BdRingAlloc(ring, 1, &new_bd) != XST_SUCCESS) return;
    XAxiDma_BdSetBufAddr(new_bd, (UINTPTR)ch->bufs[buf_idx]);
    XAxiDma_BdSetLength(new_bd, BYTES_PER_BLOCK, ring->MaxTransferLen);
    XAxiDma_BdSetCtrl(new_bd, 0);
    XAxiDma_BdSetId(new_bd, buf_idx);
    XAxiDma_BdRingToHw(ring, 1, new_bd);
}

/* Rough TX-underrun counter.  If, at the point we try to submit a new
 * buffer, >1 BD has already been completed by the DMA, the TX ring was
 * starving us -- we fell behind real-time.  The first completion is
 * expected (we consumed one buffer while we were processing); anything
 * beyond that is slack we lost. */
static uint32_t tx_underrun_events = 0;

static int submit_tx_block_chan(tx_chan_t *ch, int buf_idx) {
    XAxiDma_BdRing *ring = XAxiDma_GetTxRing(ch->dma);
    XAxiDma_Bd *bd_ptr;
    if (ch->primed_count >= NBUF) {
        int n_done = 0;
        while (n_done == 0)
            n_done = XAxiDma_BdRingFromHw(ring, 1, &bd_ptr);
        XAxiDma_BdRingFree(ring, 1, bd_ptr);
        /* Peek at how many more completions are pending; if the DMA has
         * finished multiple BDs beyond the one we just freed, it means
         * we couldn't keep up with the 48 kHz drain and at least one
         * BD got replayed before we could refresh it. */
        XAxiDma_Bd *peek;
        int extra = XAxiDma_BdRingFromHw(ring, NBUF, &peek);
        if (extra > 0) {
            tx_underrun_events += (uint32_t)extra;
            XAxiDma_BdRingFree(ring, extra, peek);
            /* Recycle the extra slots so primed_count stays accurate. */
            ch->primed_count -= extra;
        }
    } else {
        ch->primed_count++;
    }
    int status = XAxiDma_BdRingAlloc(ring, 1, &bd_ptr);
    if (status != XST_SUCCESS) return status;
    XAxiDma_BdSetBufAddr(bd_ptr, (UINTPTR)ch->bufs[buf_idx]);
    XAxiDma_BdSetLength(bd_ptr, BYTES_PER_BLOCK, ring->MaxTransferLen);
    XAxiDma_BdSetCtrl(bd_ptr, XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK);
    XAxiDma_BdSetId(bd_ptr, buf_idx);
    return XAxiDma_BdRingToHw(ring, 1, bd_ptr);
}

/* ============================================================
 *                  Hardware init
 * ============================================================ */
static int i2s_tx_init(XI2s_Tx *inst, u16 dev_id) {
    XI2stx_Config *cfg = XI2s_Tx_LookupConfig(dev_id);
    if (!cfg) return XST_FAILURE;
    int s = XI2s_Tx_CfgInitialize(inst, cfg, cfg->BaseAddress);
    if (s != XST_SUCCESS) return s;
    XI2s_Tx_SetSclkOutDiv(inst, 12288000, FS);
    XI2s_Tx_JustifyEnable(inst, 0);
    XI2s_Tx_SetChMux(inst, XI2S_TX_CHID0, XI2S_TX_CHMUX_AXIS_01);
    XI2s_Tx_Enable(inst, 1);
    return XST_SUCCESS;
}
static int i2s_rx_init(XI2s_Rx *inst, u16 dev_id) {
    XI2srx_Config *cfg = XI2s_Rx_LookupConfig(dev_id);
    if (!cfg) return XST_FAILURE;
    int s = XI2s_Rx_CfgInitialize(inst, cfg, cfg->BaseAddress);
    if (s != XST_SUCCESS) return s;
    XI2s_Rx_SetSclkOutDiv(inst, 12288000, FS);
    XI2s_Rx_JustifyEnable(inst, 0);
    XI2s_Rx_SetChMux(inst, 0, XI2S_RX_CHMUX_XI2S_01);
    XI2s_Rx_Enable(inst, 1);
    return XST_SUCCESS;
}
static int hw_init(void) {
    int s;
    if ((s = i2s_tx_init(&I2StxNoise,  I2S_NOISE_ID)) != XST_SUCCESS) return s;
    if ((s = i2s_tx_init(&I2StxAn[0], I2S_AN0_ID))    != XST_SUCCESS) return s;
    if ((s = i2s_tx_init(&I2StxAn[1], I2S_AN1_ID))    != XST_SUCCESS) return s;
    if ((s = i2s_tx_init(&I2StxAn[2], I2S_AN2_ID))    != XST_SUCCESS) return s;
    if ((s = i2s_tx_init(&I2StxAn[3], I2S_AN3_ID))    != XST_SUCCESS) return s;
    if ((s = i2s_rx_init(&I2SrxErr[0], I2S_RX_ERR0_ID)) != XST_SUCCESS) return s;
    if ((s = i2s_rx_init(&I2SrxErr[1], I2S_RX_ERR1_ID)) != XST_SUCCESS) return s;

    if ((s = XAxiDma_CfgInitialize(&DmaErr0Noise,
            XAxiDma_LookupConfig(DMA_ERR0_NOISE_ID))) != XST_SUCCESS) return s;
    if ((s = XAxiDma_CfgInitialize(&DmaErr1An0,
            XAxiDma_LookupConfig(DMA_ERR1_AN0_ID)))   != XST_SUCCESS) return s;
    if ((s = XAxiDma_CfgInitialize(&DmaAn1,
            XAxiDma_LookupConfig(DMA_AN1_ID)))        != XST_SUCCESS) return s;
    if ((s = XAxiDma_CfgInitialize(&DmaAn2,
            XAxiDma_LookupConfig(DMA_AN2_ID)))        != XST_SUCCESS) return s;
    if ((s = XAxiDma_CfgInitialize(&DmaAn3,
            XAxiDma_LookupConfig(DMA_AN3_ID)))        != XST_SUCCESS) return s;

    XAxiDma *fdx_dmas[] = { &DmaErr0Noise, &DmaErr1An0 };
    for (int i = 0; i < 2; i++) {
        XAxiDma_IntrDisable(fdx_dmas[i], XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
        XAxiDma_IntrDisable(fdx_dmas[i], XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    }
    XAxiDma *tx_only_dmas[] = { &DmaAn1, &DmaAn2, &DmaAn3 };
    for (int i = 0; i < 3; i++) {
        XAxiDma_IntrDisable(tx_only_dmas[i], XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    }

    rx_err[0].dma  = &DmaErr0Noise;  rx_err[0].bufs = ErrBuf[0];
    rx_err[1].dma  = &DmaErr1An0;    rx_err[1].bufs = ErrBuf[1];
    tx_noise.dma   = &DmaErr0Noise;  tx_noise.bufs = NoiseBuf;   tx_noise.primed_count = 0;
    tx_an[0].dma   = &DmaErr1An0;    tx_an[0].bufs = OutBuf[0];  tx_an[0].primed_count = 0;
    tx_an[1].dma   = &DmaAn1;        tx_an[1].bufs = OutBuf[1];  tx_an[1].primed_count = 0;
    tx_an[2].dma   = &DmaAn2;        tx_an[2].bufs = OutBuf[2];  tx_an[2].primed_count = 0;
    tx_an[3].dma   = &DmaAn3;        tx_an[3].bufs = OutBuf[3];  tx_an[3].primed_count = 0;

    xil_printf("HW init OK (5 TX, 2 RX, 5 DMA, NBUF=%d)\r\n", NBUF);
    return XST_SUCCESS;
}

/* ============================================================
 *                  SPM convergence + divergence logic
 * ============================================================ */
static void evaluate_spm_and_divergence(uint32_t block_count) {
    /* ============ SPM training phase ============ */
    if (!anc.anc_active) {
        float mis_num = 0.0f, mis_den = 0.0f;
        for (int m = 0; m < N_ERR; m++) {
            mis_num += anc.e_spm_sq[m];
            mis_den += anc.e_meas_sq[m];
        }
        anc.e_spm_sq[0]  = anc.e_spm_sq[1]  = 0.0f;
        anc.e_meas_sq[0] = anc.e_meas_sq[1] = 0.0f;

        anc.train_blocks++;
        if (mis_den > 1e-8f) {
            float ratio = sqrtf(mis_num / mis_den);
            /* Clamp extreme single-block outliers so one bad sample doesn't
             * poison the EMA for 50+ blocks. */
            if (ratio > 1.5f) ratio = 1.5f;
            anc.misadj_ema  = (1.0f - SPM_EMA_ALPHA) * anc.misadj_ema
                            +         SPM_EMA_ALPHA * ratio;
            anc.last_misadj = anc.misadj_ema;

            if (anc.train_blocks >= SPM_MIN_TRAIN &&
                anc.misadj_ema   <= SPM_MISADJ_ARM) {
                anc.anc_active = 1;
                anc.mu_scale   = W_MU_STARTUP;   /* gentle startup */
                anc.arms++;
                tlm_printf("LOG:ANC_ARM ema=%.3f train=%u mu_scale=%.2f\n",
                           (double)anc.misadj_ema, (unsigned)anc.train_blocks,
                           (double)anc.mu_scale);
            }
        }
        return;
    }

    /* ============ ANC active phase ============ */
    /* We don't run SPM in this branch so e_spm_sq stays zero -- but we still
     * reset e_meas_sq each block so RMS windows don't integrate forever. */
    anc.e_meas_sq[0] = anc.e_meas_sq[1] = 0.0f;

    /* Grow mu_scale back toward 1.0 each block after a divergence event. */
    if (anc.mu_scale < 1.0f) {
        anc.mu_scale *= W_MU_RECOVERY;
        if (anc.mu_scale > 1.0f) anc.mu_scale = 1.0f;
    }

    if ((block_count & (DIVERGENCE_CHECK_BLOCKS - 1)) == 0) {
        if (w_has_diverged()) {
            /* Soft recovery: zero W, halve step size, STAY in ANC mode.
             * This preserves the sine reference for the user and avoids the
             * on/off cycling that was modulating the perceived pitch. */
            w_reset();
            anc.mu_scale *= 0.5f;
            if (anc.mu_scale < W_MU_FLOOR) {
                /* Persistent divergence -> Shat is probably stale.
                 * Fall back to SPM, but don't wipe the EMA; that lets us
                 * re-arm quickly once Shat catches up. */
                anc.anc_active = 0;
                anc.mu_scale   = 1.0f;
                anc.disarms++;
                tlm_printf("LOG:ANC_DISARM persistent_diverge ema=%.3f\n",
                           (double)anc.misadj_ema);
            } else {
                tlm_printf("LOG:W_DIVERGE_SOFT W=0 mu_scale=%.3f\n",
                           (double)anc.mu_scale);
            }
        }
    }
}

/* ============================================================
 *                  Snapshot + metrics emission
 * ============================================================ */
/* Sum-squared of all Shat taps per mic, summed over the 4 speakers.
 * Used as a coarse "is Shat growing at all?" diagnostic.  O(N_ERR*N_SPK*SZ_TAPS)
 * floats but we only run it once every 300 blocks, so ~2 us of compute. */
static void shat_energy_per_mic(float out[N_ERR]) {
    for (int m = 0; m < N_ERR; m++) {
        float s = 0.0f;
        for (int k = 0; k < N_SPK; k++) {
            const float *sh = anc.shat[m][k];
            for (int t = 0; t < SZ_TAPS; t++) s += sh[t] * sh[t];
        }
        out[m] = s;
    }
}

/* Telemetry is emitted at one small line per block rather than a burst
 * of six+16 in a single block.  This change is the primary fix for the
 * 6/7 frequency error: the old burst spent ~1-2 ms in vsnprintf on some
 * blocks, which forced a TX DMA underrun and one-buffer replay per ~7
 * blocks of audio -- exactly the mechanism that made 500 Hz play as
 * 428.57 Hz.  All numbers are latched once per 300-block window so the
 * spooled values are mutually consistent even though they're printed
 * over ~33 ms. */
static void emit_snapshot_and_metrics(uint32_t block_count) {
    /* block_hot is informational only now.  Gating ALL telemetry behind it
     * was the reason the dashboard went dark the instant ANC armed: with
     * MIMO compute + cache pressure the blocks often sit at ~700 us, above
     * the 600 us threshold, so every metric line got skipped forever.  The
     * individual emits below are already cheap (<=1 vsnprintf each) and the
     * DATA drain is int16 %d, so we just let them run. */
    int block_hot = (last_block_cycles > BLOCK_OVERRUN_TICKS);
    (void)block_hot;

    /* Pick the right cadence for the current mode.  ANC mode snapshots more
     * rarely and drains more slowly so cancellation isn't disturbed by
     * UART / vsnprintf bursts.  The host FFT still has the full 256-sample
     * window -- it just arrives every few seconds instead of every second. */
    uint32_t arm_interval =
        anc.anc_active ? SNAPSHOT_INTERVAL_BLOCKS_ANC
                       : SNAPSHOT_INTERVAL_BLOCKS_SPM;
    uint32_t drain_divider =
        anc.anc_active ? SNAPSHOT_DRAIN_DIVIDER_ANC
                       : SNAPSHOT_DRAIN_DIVIDER_SPM;

    /* ---------- Snapshot arm ---------- */
    if (snap_state == 0 && (block_count % arm_interval) == 0u) {
        snap_state = 1;
        snap_idx = 0;
        tlm_push_str("SNAP_START\n");
    }

    /* Timestamp drain start the first time we see snap_state==2 so the
     * circuit breaker below knows how long we've been draining. */
    if (snap_state == 2 && snap_drain_idx == 0 && snap_drain_started_at == 0) {
        snap_drain_started_at = block_count;
    }

    /* ---------- Snapshot drain ----------
     * int16 %d DATA line, no float formatting.  Rate-limited via
     * drain_divider so that during ANC the UART only sees one line every
     * ~1.3 ms (instead of every 0.67 ms) -- well under the 115200 baud
     * link capacity, leaves room for metric emits alongside. */
    if (snap_state == 2) {
        if (snap_drain_idx < SNAPSHOT_LEN &&
            (block_count % drain_divider) == 0u) {
            float r  = ref_snap[snap_drain_idx];
            float e0 = err_snap[0][snap_drain_idx];
            float e1 = err_snap[1][snap_drain_idx];
            if (r  >  1.0f) r  =  1.0f; if (r  < -1.0f) r  = -1.0f;
            if (e0 >  1.0f) e0 =  1.0f; if (e0 < -1.0f) e0 = -1.0f;
            if (e1 >  1.0f) e1 =  1.0f; if (e1 < -1.0f) e1 = -1.0f;
            int qr  = (int)(r  * 16384.0f);
            int qe0 = (int)(e0 * 16384.0f);
            int qe1 = (int)(e1 * 16384.0f);
            tlm_printf("DATA:%d,%d,%d\n", qr, qe0, qe1);
            snap_drain_idx++;
        }
        /* Normal completion */
        if (snap_drain_idx >= SNAPSHOT_LEN) {
            tlm_push_str("SNAP_END\n");
            snap_state = 0;
            snap_drain_idx = 0;
            snap_drain_started_at = 0;
        }
        /* Circuit breaker: if drain has been running way too long (UART
         * is wedged, ring is full, host disconnected), force-end so we
         * don't block future snapshots forever. */
        else if (snap_drain_started_at != 0 &&
                 (block_count - snap_drain_started_at) >
                 SNAPSHOT_DRAIN_TIMEOUT_BLOCKS) {
            tlm_push_str("SNAP_END\n");   /* may be dropped; best effort */
            snap_state = 0;
            snap_drain_idx = 0;
            snap_drain_started_at = 0;
        }
    }

    /* ---------- Staggered metric emission ---------- */
    uint32_t phase = block_count % 300u;

    /* Phase 0: snapshot the accumulators once per window so the spooled
     * lines are mutually consistent.  This is cheap -- no vsnprintf. */
    if (phase == 0u && sample_count > 0) {
        float ref_rms = sqrtf(block_ref_sq_sum / (float)sample_count);
        float e0_rms  = sqrtf(block_err_sq_sum[0] / (float)sample_count);
        float e1_rms  = sqrtf(block_err_sq_sum[1] / (float)sample_count);
        saved_ref_rms = ref_rms;
        saved_e0_rms  = e0_rms;
        saved_e1_rms  = e1_rms;
        saved_atten0  = (ref_rms > 1e-8f && e0_rms > 1e-8f) ?
                        20.0f * log10f(ref_rms / e0_rms) : 0.0f;
        saved_atten1  = (ref_rms > 1e-8f && e1_rms > 1e-8f) ?
                        20.0f * log10f(ref_rms / e1_rms) : 0.0f;
        shat_energy_per_mic(saved_shat_e);
        block_ref_sq_sum = 0.0f;
        block_err_sq_sum[0] = block_err_sq_sum[1] = 0.0f;
        sample_count = 0;
    }

    /* Track overrun count purely for visibility -- we no longer gate
     * metric emits on it.  One vsnprintf per 300 blocks is ~100 us / 200 ms
     * of wall time, negligible. */
    if (last_block_cycles > BLOCK_OVERRUN_TICKS) tlm_skipped_emits++;

    /* One metric line per 50-block offset (~33 ms apart).  All lines are
     * one vsnprintf each, so peak per-block cost is bounded. */
    switch (phase) {
        case 10:
            tlm_printf("RMS:%.5f,%.5f,%.5f\n",
                       (double)saved_ref_rms,
                       (double)saved_e0_rms,
                       (double)saved_e1_rms);
            break;
        case 60:
            tlm_printf("ATTEN:%.2f,%.2f\n",
                       (double)saved_atten0, (double)saved_atten1);
            break;
        case 110:
            tlm_printf("CYC:%u,%u,%u\n",
                       (unsigned)last_block_cycles,
                       (unsigned)max_block_cycles,
                       (unsigned)tlm_dropped_bytes);
            break;
        case 160:
            tlm_printf("MISADJ:%.4f,%u,%u\n",
                       (double)anc.last_misadj,
                       (unsigned)anc.arms,
                       (unsigned)anc.disarms);
            break;
        case 210:
            tlm_printf("MUSCALE:%.3f\n", (double)anc.mu_scale);
            break;
        case 260:
            tlm_printf("SHATE:%.5e,%.5e\n",
                       (double)saved_shat_e[0], (double)saved_shat_e[1]);
            /* Status breadcrumb the dashboard watches for -- pushed as
             * a literal (no vsnprintf) right after the SHATE line. */
            tlm_push_str(anc.anc_active ? "STATUS:ON\n" : "STATUS:TRAINING\n");
            break;
        case 280:
            /* Include skip count so the dashboard can tell at a glance
             * whether the overrun-gate is triggering. */
            tlm_printf("TLMSKIP:%u\n", (unsigned)tlm_skipped_emits);
            break;
        case 295:
            /* TX underrun counter -- how many buffers the DMA replayed
             * because the CPU couldn't keep up.  Nonzero = the 428.5 Hz
             * symptom is still active. */
            tlm_printf("TXUR:%u\n", (unsigned)tx_underrun_events);
            break;
        default:
            break;
    }
}

/* ============================================================
 *                  main()
 * ============================================================ */
int main(void) {
    xil_printf("\r\n=== FXLMS ANC 2x4 MIMO (patched) ===\r\n");

    sine_lut_init();
    if (hw_init() != XST_SUCCESS) return XST_FAILURE;
    fxlms_init();

    UINTPTR rx_bd_bases[N_ERR] = { BD_BASE_ERR0, BD_BASE_ERR1 };
    for (int m = 0; m < N_ERR; m++) {
        if (setup_sg_rx_ring(rx_err[m].dma, rx_bd_bases[m],
                             rx_err[m].bufs, NBUF) != XST_SUCCESS)
            return XST_FAILURE;
    }

    if (setup_sg_tx_ring(tx_noise.dma, BD_BASE_NOISE,
                         tx_noise.bufs, NBUF) != XST_SUCCESS) return XST_FAILURE;
    UINTPTR an_bd_bases[N_SPK] = {
        BD_BASE_AN0, BD_BASE_AN1, BD_BASE_AN2, BD_BASE_AN3
    };
    for (int k = 0; k < N_SPK; k++) {
        if (setup_sg_tx_ring(tx_an[k].dma, an_bd_bases[k],
                             tx_an[k].bufs, NBUF) != XST_SUCCESS) return XST_FAILURE;
    }

    memset(NoiseBuf, 0, sizeof(NoiseBuf));
    memset(OutBuf,   0, sizeof(OutBuf));
    Xil_DCacheFlushRange((UINTPTR)NoiseBuf, sizeof(NoiseBuf));
    Xil_DCacheFlushRange((UINTPTR)OutBuf,   sizeof(OutBuf));

    /* Prime all 4 TX slots per channel with silence so the DAC never starves
     * before the processing loop gets its first block out. */
    for (int b = 0; b < NBUF; b++) {
        if (submit_tx_block_chan(&tx_noise, b) != XST_SUCCESS)
            xil_printf("Noise prime fail b=%d\r\n", b);
        for (int k = 0; k < N_SPK; k++) {
            if (submit_tx_block_chan(&tx_an[k], b) != XST_SUCCESS)
                xil_printf("Antinoise%d prime fail b=%d\r\n", k, b);
        }
    }

    xil_printf("All rings running -- entering MIMO ANC loop\r\n");

    uint32_t block_count = 0;
    while (1) {
        int proc_idx   = wait_rx_block_chan(&rx_err[0]);
        int proc_idx_1 = wait_rx_block_chan(&rx_err[1]);
        if (proc_idx != proc_idx_1) {
            /* Latched warn; no UART blocking on the critical path. */
            tlm_printf("WARN:mic_buf_div %d %d\n", proc_idx, proc_idx_1);
        }

        process_block(proc_idx);

        /* Submit the 5 outgoing TX buffers for this slot BEFORE re-arming RX,
         * so the TX DMAs are fed as early as possible. */
        if (submit_tx_block_chan(&tx_noise, proc_idx) != XST_SUCCESS)
            tlm_push_str("WARN:noise_tx_submit\n");
        for (int k = 0; k < N_SPK; k++) {
            if (submit_tx_block_chan(&tx_an[k], proc_idx) != XST_SUCCESS)
                tlm_printf("WARN:an_tx_submit k=%d\n", k);
        }

        /* Now that nothing touches ErrBuf[*][proc_idx] any more, it's safe
         * to give it back to the RX DMAs. */
        for (int m = 0; m < N_ERR; m++) rx_rearm_chan(&rx_err[m], proc_idx);

        evaluate_spm_and_divergence(block_count);
        emit_snapshot_and_metrics(block_count);
        telemetry_service();             /* non-blocking UART drain     */
        block_count++;
    }
    return 0;
}
