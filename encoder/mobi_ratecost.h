/*****************************************************************************
 * mobi_ratecost.h: retail Mobiclip's SAD-rate RD-cost primitives
 *****************************************************************************
 * Ported verbatim from Helwettpackardenterprise's MODS_Encoder_v43_2
 * (mods_encoder.c:145-421 and 6694-6984), a translation of the retail
 * Mobiclip VfW encoder verified against the original DLL under Unicorn.
 *
 * Verified bit-exact against that reference at the point of this port: 52000
 * synthetic 4x4/8x8 blocks across all 52 QPs, using the exact quant-table
 * addresses/shifts from tests/verify_sad_rate_4x4.py and
 * tests/verify_sad_rate_8x8.py (Unicorn-verified against the real DLL --
 * the doc comment near mods_setup_quant_tables in mods_encoder.c disagrees
 * with those scripts on both the MF/IQ table addresses and the 4x4 inverse
 * shift; the scripts are ground truth, the comment is not).
 *
 * This is the cost model retail's own mode decision uses internally to
 * decide, per 4x4/8x8 sub-block, whether coding a residual beats leaving it
 * as raw SSE -- NOT yet wired into x264's own ME/mode-decision search (see
 * memory note mobiclip-native-mode-decision-port for why that's a separate,
 * larger phase). This header only exposes the verified primitive so it can
 * be called for cost comparison/re-scoring experiments.
 *****************************************************************************/
#ifndef X264_MOBI_RATECOST_H
#define X264_MOBI_RATECOST_H

#include <stdint.h>
#include "common/common.h"

/* Every symbol here is compiled twice (once per BIT_DEPTH, like the rest of
 * x264) even though the logic itself doesn't vary with pixel depth -- so it
 * needs the same x264_template() renaming as everything else in SRCS_X, or
 * the 8-bit and 10-bit objects collide at link time with duplicate symbols. */
#define mobi_sad_rate_4x4_core x264_template(mobi_sad_rate_4x4_core)
#define mobi_sad_rate_8x8_core x264_template(mobi_sad_rate_8x8_core)
#define mobi_rescore_4x4       x264_template(mobi_rescore_4x4)
#define mobi_rescore_8x8       x264_template(mobi_rescore_8x8)

/* mf/iq are the flat per-QP forward-MF / inverse-quant tables (16 entries
 * for 4x4, 64 for 8x8) at the exact retail DLL addresses:
 *   4x4: mf = 0x1009C940 + (qp%6)*32, iq = 0x1009CD00 + (qp%6)*32
 *        fwd_shift = qp/6 + 15, fwd_offset = (1<<fwd_shift)/3, inv_shift = qp/6
 *   8x8: mf = 0x1009C640 + (qp%6)*128, iq = 0x1009CA00 + (qp%6)*128
 *        fwd_shift = qp/6 + 16, fwd_offset = (1<<fwd_shift)/3, inv_shift = qp/6 - 2
 * Callers must supply these tables themselves (e.g. sourced from the
 * mobi_quant4_scale/mobi_dequant4_scale family already in macroblock.h, or
 * read directly from a mapped DLL image as the verify_sad_rate_*.py scripts
 * do) -- this module intentionally has no ctx-struct dependency. */

int mobi_sad_rate_4x4_core(
    const uint8_t *src, const uint8_t *pred, int stride, uint8_t *workspace,
    int qp, int lambda, int is_keyframe, const uint16_t *mf, int fwd_offset,
    int fwd_shift, const int16_t *iq, int inv_shift, int sub_idx, int32_t *flag_ptr );

int mobi_sad_rate_8x8_core(
    const uint8_t *src, const uint8_t *pred, int stride, uint8_t *workspace,
    int qp, int lambda, int is_keyframe, const uint16_t *mf, int fwd_offset,
    int fwd_shift, const int16_t *iq, int inv_shift, int sub_idx, int32_t *flag_ptr );

/* Convenience wrappers for shadow-scoring/logging call sites: build the
 * quant tables internally (see mobi_build_quant_4x4/8x8 in mobi_ratecost.c)
 * and take src/pred at a single shared stride -- callers with mismatched
 * FENC_STRIDE/FDEC_STRIDE must copy into tightly-packed local buffers first,
 * since the retail kernel itself is only ever called with matched strides. */
int mobi_rescore_4x4( const uint8_t *src, const uint8_t *pred, int stride,
                       int qp, int lambda, int is_keyframe );
int mobi_rescore_8x8( const uint8_t *src, const uint8_t *pred, int stride,
                       int qp, int lambda, int is_keyframe );

#endif
