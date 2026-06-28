/*****************************************************************************
 * macroblock.h: macroblock encoding
 *****************************************************************************
 * Copyright (C) 2003-2025 x264 project
 *
 * Authors: Loren Merritt <lorenm@u.washington.edu>
 *          Laurent Aimar <fenrir@via.ecp.fr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/

#ifndef X264_ENCODER_MACROBLOCK_H
#define X264_ENCODER_MACROBLOCK_H

#include "common/macroblock.h"

/* Mobiclip quantization tables — used by custom quantizer to match decoder.
 * quant4x4_tab/q8 match the decoder's dequant tables exactly. */
static const uint8_t mobi_zigzag4x4[16] =
    { 0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15 };
static const uint8_t mobi_zigzag8x8[64] =
    { 0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
     12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
     35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
     58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63 };
static const uint8_t mobi_q4[6][16] = {
    {10,13,13,10,16,10,13,13,13,13,16,10,16,13,13,16},
    {11,14,14,11,18,11,14,14,14,14,18,11,18,14,14,18},
    {13,16,16,13,20,13,16,16,16,16,20,13,20,16,16,20},
    {14,18,18,14,23,14,18,18,18,18,23,14,23,18,18,23},
    {16,20,20,16,25,16,20,20,20,20,25,16,25,20,20,25},
    {18,23,23,18,29,18,23,23,23,23,29,18,29,23,23,29},
};
static const uint8_t mobi_q8[6][64] = {
    {0x14,0x13,0x13,0x19,0x12,0x19,0x13,0x18,0x18,0x13,0x14,0x12,0x20,0x12,0x14,0x13,0x13,0x18,0x18,0x13,0x13,0x19,0x12,0x19,0x12,0x19,0x12,0x19,0x13,0x18,0x18,0x13,0x13,0x18,0x18,0x13,0x12,0x20,0x12,0x14,0x12,0x20,0x12,0x18,0x18,0x13,0x13,0x18,0x18,0x12,0x19,0x12,0x19,0x12,0x13,0x18,0x18,0x13,0x12,0x20,0x12,0x18,0x18,0x12},
    {0x16,0x15,0x15,0x1C,0x13,0x1C,0x15,0x1A,0x1A,0x15,0x16,0x13,0x23,0x13,0x16,0x15,0x15,0x1A,0x1A,0x15,0x15,0x1C,0x13,0x1C,0x13,0x1C,0x13,0x1C,0x15,0x1A,0x1A,0x15,0x15,0x1A,0x1A,0x15,0x13,0x23,0x13,0x16,0x13,0x23,0x13,0x1A,0x1A,0x15,0x15,0x1A,0x1A,0x13,0x1C,0x13,0x1C,0x13,0x15,0x1A,0x1A,0x15,0x13,0x23,0x13,0x1A,0x1A,0x13},
    {0x1A,0x18,0x18,0x21,0x17,0x21,0x18,0x1F,0x1F,0x18,0x1A,0x17,0x2A,0x17,0x1A,0x18,0x18,0x1F,0x1F,0x18,0x18,0x21,0x17,0x21,0x17,0x21,0x17,0x21,0x18,0x1F,0x1F,0x18,0x18,0x1F,0x1F,0x18,0x17,0x2A,0x17,0x1A,0x17,0x2A,0x17,0x1F,0x1F,0x18,0x18,0x1F,0x1F,0x17,0x21,0x17,0x21,0x17,0x18,0x1F,0x1F,0x18,0x17,0x2A,0x17,0x1F,0x1F,0x17},
    {0x1C,0x1A,0x1A,0x23,0x19,0x23,0x1A,0x21,0x21,0x1A,0x1C,0x19,0x2D,0x19,0x1C,0x1A,0x1A,0x21,0x21,0x1A,0x1A,0x23,0x19,0x23,0x19,0x23,0x19,0x23,0x1A,0x21,0x21,0x1A,0x1A,0x21,0x21,0x1A,0x19,0x2D,0x19,0x1C,0x19,0x2D,0x19,0x21,0x21,0x1A,0x1A,0x21,0x21,0x19,0x23,0x19,0x23,0x19,0x1A,0x21,0x21,0x1A,0x19,0x2D,0x19,0x21,0x21,0x19},
    {0x20,0x1E,0x1E,0x28,0x1C,0x28,0x1E,0x26,0x26,0x1E,0x20,0x1C,0x33,0x1C,0x20,0x1E,0x1E,0x26,0x26,0x1E,0x1E,0x28,0x1C,0x28,0x1C,0x28,0x1C,0x28,0x1E,0x26,0x26,0x1E,0x1E,0x26,0x26,0x1E,0x1C,0x33,0x1C,0x20,0x1C,0x33,0x1C,0x26,0x26,0x1E,0x1E,0x26,0x26,0x1C,0x28,0x1C,0x28,0x1C,0x1E,0x26,0x26,0x1E,0x1C,0x33,0x1C,0x26,0x26,0x1C},
    {0x24,0x22,0x22,0x2E,0x20,0x2E,0x22,0x2B,0x2B,0x22,0x24,0x20,0x3A,0x20,0x24,0x22,0x22,0x2B,0x2B,0x22,0x22,0x2E,0x20,0x2E,0x20,0x2E,0x20,0x2E,0x22,0x2B,0x2B,0x22,0x22,0x2B,0x2B,0x22,0x20,0x3A,0x20,0x24,0x20,0x3A,0x20,0x2B,0x2B,0x22,0x22,0x2B,0x2B,0x20,0x2E,0x20,0x2E,0x20,0x22,0x2B,0x2B,0x22,0x20,0x3A,0x20,0x2B,0x2B,0x20},
};
static const uint8_t mobi_q8_chroma[6][64] = {
    {0x14,0x13,0x13,0x19,0x12,0x19,0x13,0x18,0x18,0x13,0x14,0x12,0x20,0x12,0x14,0x13,0x13,0x18,0x18,0x13,0x13,0x19,0x12,0x19,0x12,0x19,0x12,0x19,0x13,0x18,0x18,0x13,0x13,0x18,0x18,0x13,0x12,0x20,0x12,0x14,0x12,0x20,0x12,0x18,0x18,0x13,0x13,0x18,0x18,0x12,0x19,0x12,0x19,0x12,0x13,0x18,0x18,0x13,0x12,0x20,0x12,0x18,0x18,0x12},
    {0x16,0x15,0x15,0x1C,0x13,0x1C,0x15,0x1A,0x1A,0x15,0x16,0x13,0x23,0x13,0x16,0x15,0x15,0x1A,0x1A,0x15,0x15,0x1C,0x13,0x1C,0x13,0x1C,0x13,0x1C,0x15,0x1A,0x1A,0x15,0x15,0x1A,0x1A,0x15,0x13,0x23,0x13,0x16,0x13,0x23,0x13,0x1A,0x1A,0x15,0x15,0x1A,0x1A,0x13,0x1C,0x13,0x1C,0x13,0x15,0x1A,0x1A,0x15,0x13,0x23,0x13,0x1A,0x1A,0x13},
    {0x1A,0x18,0x18,0x21,0x17,0x21,0x18,0x1F,0x1F,0x18,0x1A,0x17,0x2A,0x17,0x1A,0x18,0x18,0x1F,0x1F,0x18,0x18,0x21,0x17,0x21,0x17,0x21,0x17,0x21,0x18,0x1F,0x1F,0x18,0x18,0x1F,0x1F,0x18,0x17,0x2A,0x17,0x1A,0x17,0x2A,0x17,0x1F,0x1F,0x18,0x18,0x1F,0x1F,0x17,0x21,0x17,0x21,0x17,0x18,0x1F,0x1F,0x18,0x17,0x2A,0x17,0x1F,0x1F,0x17},
    {0x1C,0x1A,0x1A,0x23,0x19,0x23,0x1A,0x21,0x21,0x1A,0x1C,0x19,0x2D,0x19,0x1C,0x1A,0x1A,0x21,0x21,0x1A,0x1A,0x23,0x19,0x23,0x19,0x23,0x19,0x23,0x1A,0x21,0x21,0x1A,0x1A,0x21,0x21,0x1A,0x19,0x2D,0x19,0x1C,0x19,0x2D,0x19,0x21,0x21,0x1A,0x1A,0x21,0x21,0x19,0x23,0x19,0x23,0x19,0x1A,0x21,0x21,0x1A,0x19,0x2D,0x19,0x21,0x21,0x19},
    {0x20,0x1E,0x1E,0x28,0x1C,0x28,0x1E,0x26,0x26,0x1E,0x20,0x1C,0x33,0x1C,0x20,0x1E,0x1E,0x26,0x26,0x1E,0x1E,0x28,0x1C,0x28,0x1C,0x28,0x1C,0x28,0x1E,0x26,0x26,0x1E,0x1E,0x26,0x26,0x1E,0x1C,0x33,0x1C,0x20,0x1C,0x33,0x1C,0x26,0x26,0x1E,0x1E,0x26,0x26,0x1C,0x28,0x1C,0x28,0x1C,0x1E,0x26,0x26,0x1E,0x1C,0x33,0x1C,0x26,0x26,0x1C},
    {0x24,0x22,0x22,0x2E,0x20,0x2E,0x22,0x2B,0x2B,0x22,0x24,0x20,0x3A,0x20,0x24,0x22,0x22,0x2B,0x2B,0x22,0x22,0x2E,0x20,0x2E,0x20,0x2E,0x20,0x2E,0x22,0x2B,0x2B,0x22,0x22,0x2B,0x2B,0x22,0x20,0x3A,0x20,0x24,0x20,0x3A,0x20,0x2B,0x2B,0x22,0x22,0x2B,0x2B,0x20,0x2E,0x20,0x2E,0x20,0x22,0x2B,0x2B,0x22,0x20,0x3A,0x20,0x2B,0x2B,0x20},
};

/* Mobiclip-aware quantizer: rounds to nearest, matching decoder's dequant.
 * The decoder does: mat[ztab[pos]] = level * (quant_tab[qx][pos] << qy)
 * So quantizer step = quant_tab[qx][pos] << qy. */
/* Extra quantizer coarseness for Mobiclip: total qy = 2 + MOBI_QYX.  qy=2
 * (MOBI_QYX=0) is the finest tier and exactly reproduces the original
 * bit-exact behaviour.  Higher values coarsen the quantization (smaller
 * files, lower quality) and are applied identically to the forward quant,
 * the reconstruction, and the header quantizer so enc/dec stay bit-exact.
 * The DEFAULT is 1: combined with the inter deadzone (see mobi_round_off)
 * this keeps temporal flicker near source level at a size comparable to
 * coarser settings.  Raise MOBI_QYX for smaller files (more flicker) or set
 * MOBI_QYX=0 for visually-lossless/bit-exact. */
static ALWAYS_INLINE int mobi_qyx( x264_t *h )
{
    int v = h->param.i_mobi_qyx;
    if( v == -1 ) { const char *e = getenv("MOBI_QYX"); v = e ? atoi(e) : 1; if( v < 0 ) v = 0; }
    return v;
}
/* Rounding offset for quantization.  level = (coef + off)/step, so the
 * threshold to reach level 1 is (step - off).  Round-to-nearest uses off =
 * step/2 (threshold step/2).  For INTER residual we instead use a deadzone
 * (smaller off -> higher threshold) so the small, noisy coefficients of
 * near-static blocks round to zero *consistently* every frame instead of
 * flipping between 0 and +/-1 -> kills temporal flicker/jitter and shrinks
 * files.  This is purely an encoder decision: the decoder reconstructs
 * level*step regardless, so it stays fully bit-compatible.
 *   MOBI_DZ in [1..8] scales the inter offset as step*MOBI_DZ/16
 *   (8 = round-to-nearest/no deadzone, default 5 ~= threshold 0.69*step). */
static ALWAYS_INLINE int mobi_round_off( x264_t *h, int step, int b_intra )
{
    if( b_intra )
        return step >> 1;
    static int dz = -1;
    if( dz < 0 ) { const char *e = getenv("MOBI_DZ"); dz = e ? atoi(e) : 5; if( dz < 1 ) dz = 1; if( dz > 8 ) dz = 8; }
    return step * dz / 16;
}

static ALWAYS_INLINE int mobi_quant_4x4( x264_t *h, dctcoef dct[16], int qp, int b_intra )
{
    int qx = qp % 6;
    int shift = mobi_qyx( h );
    int nz = 0;
    for( int zz = 0; zz < 16; zz++ )
    {
        int raster = mobi_zigzag4x4[zz];
        /* x264's 2D 4x4 DCT DC coefficient = 16*R for constant residual R.
         * The Mobiclip IDCT (>>6 after 2D butterfly) needs DC = 64*R to recover R.
         * Using unshifted mobi_q4[qx] as step compensates: level = 16R/step,
         * reconstruction = level * (step<<2) / 64 = 16R * 4 / 64 = R. */
        int step = (int)mobi_q4[qx][zz] << shift;
        int off  = mobi_round_off( h, step, b_intra );
        int coef = dct[raster];
        int level;
        if( coef >= 0 )
            level = (coef + off) / step;
        else
            level = -((-coef + off) / step);
        dct[raster] = level;
        nz |= level;
    }
    return !!nz;
}

static ALWAYS_INLINE int mobi_quant_8x8( x264_t *h, dctcoef dct[64], int qp, int b_intra )
{
    int qx = qp % 6;
    int shift = mobi_qyx( h );
    int nz = 0;
    for( int zz = 0; zz < 64; zz++ )
    {
        int raster = mobi_zigzag8x8[zz];
        int step = (int)mobi_q8[qx][zz] << shift;
        int off  = mobi_round_off( h, step, b_intra );
        int coef = dct[raster];
        int level;
        if( coef >= 0 )
            level = (coef + off) / step;
        else
            level = -((-coef + off) / step);
        dct[raster] = level;
        nz |= level;
    }
    return !!nz;
}

#define x264_rdo_init x264_template(rdo_init)
void x264_rdo_init( void );

#define x264_macroblock_probe_skip x264_template(macroblock_probe_skip)
int x264_macroblock_probe_skip( x264_t *h, int b_bidir );

#define x264_macroblock_probe_pskip( h )\
    x264_macroblock_probe_skip( h, 0 )
#define x264_macroblock_probe_bskip( h )\
    x264_macroblock_probe_skip( h, 1 )

#define x264_predict_lossless_4x4 x264_template(predict_lossless_4x4)
void x264_predict_lossless_4x4( x264_t *h, pixel *p_dst, int p, int idx, int i_mode );
#define x264_predict_lossless_8x8 x264_template(predict_lossless_8x8)
void x264_predict_lossless_8x8( x264_t *h, pixel *p_dst, int p, int idx, int i_mode, pixel edge[36] );
#define x264_predict_lossless_16x16 x264_template(predict_lossless_16x16)
void x264_predict_lossless_16x16( x264_t *h, int p, int i_mode );
#define x264_predict_lossless_chroma x264_template(predict_lossless_chroma)
void x264_predict_lossless_chroma( x264_t *h, int i_mode );

#define x264_macroblock_encode x264_template(macroblock_encode)
void x264_macroblock_encode      ( x264_t *h );
#define x264_macroblock_write_cabac x264_template(macroblock_write_cabac)
void x264_macroblock_write_cabac ( x264_t *h, x264_cabac_t *cb );
#define x264_macroblock_write_cavlc x264_template(macroblock_write_cavlc)
void x264_macroblock_write_cavlc ( x264_t *h );

#define x264_macroblock_encode_p8x8 x264_template(macroblock_encode_p8x8)
void x264_macroblock_encode_p8x8( x264_t *h, int i8 );
#define x264_macroblock_encode_p4x4 x264_template(macroblock_encode_p4x4)
void x264_macroblock_encode_p4x4( x264_t *h, int i4 );
#define x264_mb_encode_chroma x264_template(mb_encode_chroma)
void x264_mb_encode_chroma( x264_t *h, int b_inter, int i_qp );

#define x264_cabac_mb_skip x264_template(cabac_mb_skip)
void x264_cabac_mb_skip( x264_t *h, int b_skip );
#define x264_cabac_block_residual_c x264_template(cabac_block_residual_c)
void x264_cabac_block_residual_c( x264_t *h, x264_cabac_t *cb, int ctx_block_cat, dctcoef *l );
#define x264_cabac_block_residual_8x8_rd_c x264_template(cabac_block_residual_8x8_rd_c)
void x264_cabac_block_residual_8x8_rd_c( x264_t *h, x264_cabac_t *cb, int ctx_block_cat, dctcoef *l );
#define x264_cabac_block_residual_rd_c x264_template(cabac_block_residual_rd_c)
void x264_cabac_block_residual_rd_c( x264_t *h, x264_cabac_t *cb, int ctx_block_cat, dctcoef *l );

#define x264_quant_luma_dc_trellis x264_template(quant_luma_dc_trellis)
int x264_quant_luma_dc_trellis( x264_t *h, dctcoef *dct, int i_quant_cat, int i_qp,
                                int ctx_block_cat, int b_intra, int idx );
#define x264_quant_chroma_dc_trellis x264_template(quant_chroma_dc_trellis)
int x264_quant_chroma_dc_trellis( x264_t *h, dctcoef *dct, int i_qp, int b_intra, int idx );
#define x264_quant_4x4_trellis x264_template(quant_4x4_trellis)
int x264_quant_4x4_trellis( x264_t *h, dctcoef *dct, int i_quant_cat,
                             int i_qp, int ctx_block_cat, int b_intra, int b_chroma, int idx );
#define x264_quant_8x8_trellis x264_template(quant_8x8_trellis)
int x264_quant_8x8_trellis( x264_t *h, dctcoef *dct, int i_quant_cat,
                             int i_qp, int ctx_block_cat, int b_intra, int b_chroma, int idx );

#define x264_noise_reduction_update x264_template(noise_reduction_update)
void x264_noise_reduction_update( x264_t *h );

static ALWAYS_INLINE int x264_quant_4x4( x264_t *h, dctcoef dct[16], int i_qp, int ctx_block_cat, int b_intra, int p, int idx )
{
    if( h->param.i_mobiclip )
        return mobi_quant_4x4( h, dct, i_qp, b_intra );
    int i_quant_cat = b_intra ? (p?CQM_4IC:CQM_4IY) : (p?CQM_4PC:CQM_4PY);
    if( h->mb.b_noise_reduction )
        h->quantf.denoise_dct( dct, h->nr_residual_sum[0+!!p*2], h->nr_offset[0+!!p*2], 16 );
    if( h->mb.b_trellis )
        return x264_quant_4x4_trellis( h, dct, i_quant_cat, i_qp, ctx_block_cat, b_intra, !!p, idx+p*16 );
    else
        return h->quantf.quant_4x4( dct, h->quant4_mf[i_quant_cat][i_qp], h->quant4_bias[i_quant_cat][i_qp] );
}

static ALWAYS_INLINE int x264_quant_8x8( x264_t *h, dctcoef dct[64], int i_qp, int ctx_block_cat, int b_intra, int p, int idx )
{
    if( h->param.i_mobiclip )
        return mobi_quant_8x8( h, dct, i_qp, b_intra );
    int i_quant_cat = b_intra ? (p?CQM_8IC:CQM_8IY) : (p?CQM_8PC:CQM_8PY);
    if( h->mb.b_noise_reduction )
        h->quantf.denoise_dct( dct, h->nr_residual_sum[1+!!p*2], h->nr_offset[1+!!p*2], 64 );
    if( h->mb.b_trellis )
        return x264_quant_8x8_trellis( h, dct, i_quant_cat, i_qp, ctx_block_cat, b_intra, !!p, idx+p*4 );
    else
        return h->quantf.quant_8x8( dct, h->quant8_mf[i_quant_cat][i_qp], h->quant8_bias[i_quant_cat][i_qp] );
}

#define STORE_8x8_NNZ( p, idx, nz )\
do\
{\
    M16( &h->mb.cache.non_zero_count[x264_scan8[p*16+idx*4]+0] ) = (nz) * 0x0101;\
    M16( &h->mb.cache.non_zero_count[x264_scan8[p*16+idx*4]+8] ) = (nz) * 0x0101;\
} while( 0 )

#define CLEAR_16x16_NNZ( p ) \
do\
{\
    M32( &h->mb.cache.non_zero_count[x264_scan8[16*p] + 0*8] ) = 0;\
    M32( &h->mb.cache.non_zero_count[x264_scan8[16*p] + 1*8] ) = 0;\
    M32( &h->mb.cache.non_zero_count[x264_scan8[16*p] + 2*8] ) = 0;\
    M32( &h->mb.cache.non_zero_count[x264_scan8[16*p] + 3*8] ) = 0;\
} while( 0 )

/* A special for loop that iterates branchlessly over each set
 * bit in a 4-bit input. */
#define FOREACH_BIT(idx,start,mask) for( int idx = start, msk = mask, skip; msk && (skip = x264_ctz_4bit(msk), idx += skip, msk >>= skip+1, 1); idx++ )

/* ------------------------------------------------------------------------
 * Mobiclip intra prediction (4x4), bit-exact with the ffmpeg mobiclip
 * decoder's predict_intra()/pick_*()/get_prediction helpers.  Reads
 * reconstructed neighbours straight out of the FDEC tile (p_dst) using the
 * decoder's pget() boundary-clamping rules so the encoder reconstructs the
 * same pixels the decoder will.  Modes: 0=V 1=H 2=plane(unused) 3=DC 4..8=dir.
 * ------------------------------------------------------------------------ */
#if !HIGH_BIT_DEPTH
/* H.264 4x4 intra mode (0..8) -> Mobiclip intra mode.  MUST stay identical to
 * the table in cavlc.c so the reconstruction prediction matches the mode that
 * gets written into the bitstream (and thus the decoder's predict_intra). */
static const int mobi_h264_to_mobiclip_mode[9] = { 0, 1, 3, 3, 7, 6, 5, 8, 4 };
/* read neighbour at block-relative (rx,ry) with decoder pget() semantics */
static ALWAYS_INLINE int mobi_pget( x264_t *h, pixel *p_dst, int idx, int rx, int ry )
{
    const int size = 4;
    if( rx == -1 && ry >= size )      { rx = -1; ry = size - 1; }
    else if( rx >= -1 && ry >= -1 )   { /* keep */ }
    else if( rx == -1 && ry == -2 )   { rx = 0; ry = -1; }
    else if( rx == -2 && ry == -1 )   { rx = -1; ry = 0; }
    int mb_x = h->mb.i_mb_x;
    int mb_y = h->mb.i_mb_y;
    int ax = mb_x * 16 + block_idx_x[idx] * 4;
    int ay = mb_y * 16 + block_idx_y[idx] * 4;

    int orig_x = ax + rx;
    int orig_y = ay + ry;

    // Decoder clips to frame bounds
    int tx = orig_x; if( tx < 0 ) tx = 0; else if( tx > h->param.i_width - 1 ) tx = h->param.i_width - 1;
    int ty = orig_y; if( ty < 0 ) ty = 0; else if( ty > h->param.i_height - 1 ) ty = h->param.i_height - 1;

    int t_mb_x = tx / 16;
    int t_mb_y = ty / 16;

    if( t_mb_y < mb_y || (t_mb_y == mb_y && t_mb_x < mb_x) )
    {
        // Past macroblock. Already reconstructed. Read from frame buffer!
        return h->fdec->plane[0][ ty * h->mb.pic.i_stride[0] + tx ];
    }
    else if( t_mb_y == mb_y && t_mb_x == mb_x )
    {
        // Current macroblock. Read the local fdec tile at the clamped position,
        // EXACTLY as the decoder reads its frame buffer in pget(). This is the
        // critical fix: the decoder fills each 4x4 block in raster order and the
        // directional modes (H, and the diagonal pick_* modes) read pixels that
        // were just written EARLIER IN THE SAME BLOCK (e.g. H mode: pixel x reads
        // pixel x-1, cascading the left edge across the row). Returning 128 for
        // in-block positions made the encoder form residuals against [L,128,128,
        // 128] while the decoder reconstructed [L,L,L,L] -> permanent desync.
        // p_dst holds: reconstructed neighbour blocks (above/left, via the tile
        // border and earlier-idx blocks) AND the progressively-filled pixels of
        // the current block, matching the decoder's frame buffer for every
        // position that a valid prediction mode actually reads.
        return p_dst[ (ty - ay) * FDEC_STRIDE + (tx - ax) ];
    }
    else
    {
        // Future macroblock to the right/below: not yet reconstructed. Valid
        // intra modes never read here (clamping keeps reads to up/left/up-right
        // neighbours), but return a deterministic value just in case.
        return 128;
    }
}
static ALWAYS_INLINE int mobi_half( int a, int b )       { return ((a + b) + 1) / 2; }
static ALWAYS_INLINE int mobi_half3( int a, int b, int c ){ return ((a + b + b + c) * 2 / 4 + 1) / 2; }
#define MPG(rx,ry) mobi_pget(h,p_dst,idx,(rx),(ry))
static ALWAYS_INLINE int mobi_half_horz( x264_t *h, pixel *p_dst, int idx, int x, int y )
{ return mobi_half3( MPG(x-1,y), MPG(x,y), MPG(x+1,y) ); }
static ALWAYS_INLINE int mobi_half_vert( x264_t *h, pixel *p_dst, int idx, int x, int y )
{ return mobi_half3( MPG(x,y-1), MPG(x,y), MPG(x,y+1) ); }

static void mobi_predict_4x4( x264_t *h, pixel *p_dst, int idx, int pmode )
{
    const int size = 4;
    int mb_x = h->mb.i_mb_x;
    int mb_y = h->mb.i_mb_y;
    int ax = mb_x * 16 + block_idx_x[idx] * 4;
    int ay = mb_y * 16 + block_idx_y[idx] * 4;
    if( pmode == 3 ) /* DC */
    {
        int fill;
        if( ax == 0 && ay == 0 )
            fill = 0x80;
        else if( ax >= 1 && ay >= 1 )
        {
            int left = 0, top = 0;
            for( int y = 0; y < size; y++ ) left += p_dst[y*FDEC_STRIDE - 1];
            for( int x = 0; x < size; x++ ) top  += p_dst[-FDEC_STRIDE + x];
            fill = ((left + top) * 2 / (2*size) + 1) / 2;
        }
        else if( ax >= 1 )
        {
            int left = 0;
            for( int y = 0; y < size; y++ ) left += p_dst[y*FDEC_STRIDE - 1];
            fill = (left * 2 / size + 1) / 2;
        }
        else /* ay >= 1 */
        {
            int top = 0;
            for( int x = 0; x < size; x++ ) top += p_dst[-FDEC_STRIDE + x];
            fill = (top * 2 / size + 1) / 2;
        }
        for( int y = 0; y < size; y++ )
            for( int x = 0; x < size; x++ )
                p_dst[y*FDEC_STRIDE + x] = fill;
        return;
    }
    for( int y = 0; y < size; y++ )
    {
        for( int x = 0; x < size; x++ )
        {
            int val = 0;
            switch( pmode )
            {
            case 0: /* V / above */ val = MPG(x, y-1); break;
            case 1: /* H / left  */ val = MPG(x-1, y); break;
            case 4: /* pick_4 */
                if( (x % 2) == 0 )
                    val = mobi_half( MPG(-1, y + x/2), MPG(-1, y + x/2 + 1) );
                else
                    val = mobi_half_vert( h, p_dst, idx, -1, y + x/2 + 1 );
                break;
            case 5: /* pick_5 */
                if( x == 0 )
                    val = mobi_half( MPG(-1, y-1), MPG(-1, y) );
                else if( y == 0 )
                    val = mobi_half_horz( h, p_dst, idx, x-2, y-1 );
                else if( x == 1 )
                    val = mobi_half_vert( h, p_dst, idx, x-2, y-1 );
                else
                    val = MPG(x-2, y-1);
                break;
            case 6: /* pick_6 */
                if( y == 0 )
                    val = mobi_half( MPG(x-1, -1), MPG(x, -1) );
                else if( x == 0 )
                    val = mobi_half_vert( h, p_dst, idx, x-1, y-2 );
                else if( y == 1 )
                    val = mobi_half_horz( h, p_dst, idx, x-1, y-2 );
                else
                    val = MPG(x-1, y-2);
                break;
            case 7: /* pick_7 */
            {
                int clr = MPG(x-1, y-1);
                if( x && y )
                    val = clr;
                else
                {
                    int acc1, acc2;
                    if( x == 0 ) acc1 = MPG(-1, y);
                    else         acc1 = MPG(x-2, -1);
                    if( y == 0 ) acc2 = MPG(x, -1);
                    else         acc2 = MPG(-1, y-2);
                    val = mobi_half3( acc1, clr, acc2 );
                }
                break;
            }
            case 8: /* pick_8 */
                if( y == 0 )
                    val = mobi_half( MPG(x, -1), MPG(x+1, -1) );
                else if( y == 1 )
                    val = mobi_half_horz( h, p_dst, idx, x+1, -1 );
                else if( x < size - 1 )
                    val = MPG(x+1, y-2);
                else if( (y % 2) == 0 )
                    val = mobi_half( MPG(y/2 + size - 1, -1), MPG(y/2 + size, -1) );
                else
                    val = mobi_half_horz( h, p_dst, idx, y/2 + size, -1 );
                break;
            default: val = 0x80; break;
            }
            p_dst[y*FDEC_STRIDE + x] = val;
        }
    }
}
#undef MPG

/* Mobiclip chroma 8x8 prediction, matching the decoder's predict_intra(size=8)
 * for the only modes the chroma-mode fix table can emit: 0=V, 1=H, 3=DC.
 * Reads reconstructed neighbours from the fdec tile borders (populated by x264
 * before chroma encode); DC handles neighbour availability exactly like the
 * decoder. p_dst is the chroma fdec tile origin. */
static void mobi_predict_chroma_8x8( x264_t *h, int ch, int mode )
{
    const int size = 8;
    int ax = h->mb.i_mb_x * 8;
    int ay = h->mb.i_mb_y * 8;
    int stride = FDEC_STRIDE;
    pixel *p_dst = h->mb.pic.p_fdec[1+ch];
    pixel *fp = NULL;
    int fstride = 0;
    if( h->fdec && h->fdec->plane[1+ch] )
    {
        fp = h->fdec->plane[1+ch];
        fstride = h->fdec->i_stride[1];
    }
    if( mode == 0 ) /* V / above */
    {
        int ref_row = X264_MAX(ay - 1, 0);
        if( fp )
        {
            for( int y = 0; y < size; y++ )
                for( int x = 0; x < size; x++ )
                    p_dst[y*stride + x] = fp[ref_row * fstride + ax + x];
        }
        else
        {
            for( int y = 0; y < size; y++ )
                for( int x = 0; x < size; x++ )
                    p_dst[y*stride + x] = p_dst[-stride + x];
        }
    }
    else if( mode == 1 ) /* H / left */
    {
        int ref_col = X264_MAX(ax - 1, 0);
        if( fp )
        {
            for( int y = 0; y < size; y++ )
            {
                pixel val = fp[(ay + y) * fstride + ref_col];
                for( int x = 0; x < size; x++ )
                    p_dst[y*stride + x] = val;
            }
        }
        else
        {
            for( int y = 0; y < size; y++ )
                for( int x = 0; x < size; x++ )
                    p_dst[y*stride + x] = p_dst[y*stride - 1];
        }
    }
    else /* DC (mode 3) - matches decoder's predict_intra case 3 */
    {
        int fill;
        if( fp )
        {
            if( ax == 0 && ay == 0 )
                fill = 0x80;
            else if( ax >= 1 && ay >= 1 )
            {
                int left = 0, top = 0;
                for( int y = 0; y < size; y++ ) left += fp[(ay + y) * fstride + ax - 1];
                for( int x = 0; x < size; x++ ) top  += fp[(ay - 1) * fstride + ax + x];
                fill = ((left + top) * 2 / (2*size) + 1) / 2;
            }
            else if( ax >= 1 )
            {
                int left = 0;
                for( int y = 0; y < size; y++ ) left += fp[(ay + y) * fstride + ax - 1];
                fill = (left * 2 / size + 1) / 2;
            }
            else /* ay >= 1 */
            {
                int top = 0;
                for( int x = 0; x < size; x++ ) top += fp[(ay - 1) * fstride + ax + x];
                fill = (top * 2 / size + 1) / 2;
            }
        }
        else
        {
            if( ax == 0 && ay == 0 )
                fill = 0x80;
            else if( ax >= 1 && ay >= 1 )
            {
                int left = 0, top = 0;
                for( int y = 0; y < size; y++ ) left += p_dst[y*stride - 1];
                for( int x = 0; x < size; x++ ) top  += p_dst[-stride + x];
                fill = ((left + top) * 2 / (2*size) + 1) / 2;
            }
            else if( ax >= 1 )
            {
                int left = 0;
                for( int y = 0; y < size; y++ ) left += p_dst[y*stride - 1];
                fill = (left * 2 / size + 1) / 2;
            }
            else /* ay >= 1 */
            {
                int top = 0;
                for( int x = 0; x < size; x++ ) top += p_dst[-stride + x];
                fill = (top * 2 / size + 1) / 2;
            }
        }
        for( int y = 0; y < size; y++ )
            for( int x = 0; x < size; x++ )
                p_dst[y*stride + x] = fill;
    }
}
#endif

static ALWAYS_INLINE void x264_mb_encode_i4x4( x264_t *h, int p, int idx, int i_qp, int i_mode, int b_predict )
{
    int nz;
    pixel *p_src = &h->mb.pic.p_fenc[p][block_idx_xy_fenc[idx]];
    pixel *p_dst = &h->mb.pic.p_fdec[p][block_idx_xy_fdec[idx]];
    ALIGNED_ARRAY_64( dctcoef, dct4x4,[16] );

    if( b_predict )
    {
#if !HIGH_BIT_DEPTH
        if( h->param.i_mobiclip && p == 0 )
        {
            /* Convert the H.264 prediction mode to the Mobiclip mode exactly as
             * cavlc.c does (fix collapses DC_LEFT/TOP/128->DC and VL->HU), so the
             * pixels we reconstruct here match the mode written to the stream. */
            /* Convert x264 internal I4x4 mode to H.264 standard mode number.
             * The global x264_mb_pred_mode4x4_fix table is BROKEN in this fork
             * because I_PRED_4x4_PLANE=2 was inserted into the enum, shifting
             * values 3-8, but the fix table was not updated for the shift. */
            static const uint8_t x264_intra_to_h264[13] =
                { 2,0,1,2,2,8,6,5,4,3,2,2,2 };
            int i_h264_mode = x264_intra_to_h264[ (unsigned)i_mode + 1 < 13 ? i_mode + 1 : 0 ];
            int mobi_mode = mobi_h264_to_mobiclip_mode[ i_h264_mode < 9 ? i_h264_mode : 8 ];
            mobi_predict_4x4( h, p_dst, idx, mobi_mode );
        }
        else
#endif
        if( h->mb.b_lossless )
            x264_predict_lossless_4x4( h, p_dst, p, idx, i_mode );
        else
            h->predict_4x4[i_mode]( p_dst );
    }

    if( h->mb.b_lossless )
    {
        nz = h->zigzagf.sub_4x4( h->dct.luma4x4[p*16+idx], p_src, p_dst );
        h->mb.cache.non_zero_count[x264_scan8[p*16+idx]] = nz;
        h->mb.i_cbp_luma |= nz<<(idx>>2);
        return;
    }

    h->dctf.sub4x4_dct( dct4x4, p_src, p_dst );

    nz = x264_quant_4x4( h, dct4x4, i_qp, ctx_cat_plane[DCT_LUMA_4x4][p], 1, p, idx );
    h->mb.cache.non_zero_count[x264_scan8[p*16+idx]] = nz;
#if !HIGH_BIT_DEPTH
    /* Always write luma4x4 in Mobiclip mode to clear stale analysis data. */
    if( h->param.i_mobiclip && p == 0 )
        for( int zz = 0; zz < 16; zz++ )
            h->dct.luma4x4[p*16+idx][zz] = dct4x4[mobi_zigzag4x4[zz]];
#endif
    if( nz )
    {
#if !HIGH_BIT_DEPTH
        if( !h->param.i_mobiclip || p != 0 )
#endif
        for( int zz = 0; zz < 16; zz++ )
            h->dct.luma4x4[p*16+idx][zz] = dct4x4[mobi_zigzag4x4[zz]];
        if( 1 ) /* mobi dequant always for matching mobiclip decoder */
        {
            int qx = i_qp % 6;
            int qtab64[16];
            /* Reconstruction uses << 2 (qy=2), matching the header QP = qx + 12.
             * Forward quant used unshifted mobi_q4[qx], so level=16R/step.
             * Here: level * (step<<2) >> 6 = 16R * 4 / 64 = R. */
            for( int zz = 0; zz < 16; zz++ )
                qtab64[mobi_zigzag4x4[zz]] = mobi_q4[qx][zz] << (2 + mobi_qyx(h));
            /* Dequantize into a 32-bit matrix: level*qtab (level up to ~hundreds,
             * qtab up to ~29<<2) overflows the int16 dct4x4 buffer, and the
             * inverse4 row/column sums overflow it further.  The decoder uses
             * int mat[]; mirror that exactly (this is the same fix as chroma
             * 8x8) or large-coefficient blocks reconstruct wrong and the error
             * drifts through P-frames as visible shearing. */
            int mat[16];
            /* Re-quantization guard.  The Mobiclip decoder reconstructs each
             * pixel as MinMaxTable[0x40 + prediction + (residual>>6)] with NO
             * bounds check.  The table is 384 entries (a 0..255 ramp with 64
             * zero/255 guard entries each side), so prediction+residual must
             * stay in [-64,319].  DCT ringing on sharp edges (e.g. the first
             * frame's high-contrast logo) can overshoot this; the real Wii then
             * reads memory outside the table and the garbage cascades through
             * intra prediction (green corruption).  Genuine streams never
             * overshoot.  Iteratively attenuate the coefficients (AC first, then
             * DC) and re-run the decoder-exact inverse transform until every
             * reconstructed pixel's index is in range; worst case the block
             * degrades to pure prediction. */
            for( int mobi_pass = 0; ; mobi_pass++ )
            {
                for( int i = 0; i < 16; i++ )
                    mat[i] = (int)dct4x4[i] * qtab64[i];
                mat[0] += 32;
                for( int i = 0; i < 4; i++ )
                {
                    unsigned a = (unsigned)mat[i*4+0] + mat[i*4+2];
                    unsigned b = (unsigned)mat[i*4+0] - mat[i*4+2];
                    unsigned c = (unsigned)mat[i*4+1] + (mat[i*4+3] >> 1);
                    unsigned d = (mat[i*4+1] >> 1) - (unsigned)mat[i*4+3];
                    mat[i*4+0] = a + c;
                    mat[i*4+1] = b + d;
                    mat[i*4+2] = b - d;
                    mat[i*4+3] = a - c;
                }
                for( int y = 0; y < 4; y++ )
                    for( int x = y+1; x < 4; x++ )
                    {
                        int tmp = mat[y*4+x];
                        mat[y*4+x] = mat[x*4+y];
                        mat[x*4+y] = tmp;
                    }
                for( int y = 0; y < 4; y++ )
                {
                    unsigned a = (unsigned)mat[y*4+0] + mat[y*4+2];
                    unsigned b = (unsigned)mat[y*4+0] - mat[y*4+2];
                    unsigned c = (unsigned)mat[y*4+1] + (mat[y*4+3] >> 1);
                    unsigned d = (mat[y*4+1] >> 1) - (unsigned)mat[y*4+3];
                    mat[y*4+0] = (int)(a + c) >> 6;
                    mat[y*4+1] = (int)(b + d) >> 6;
                    mat[y*4+2] = (int)(b - d) >> 6;
                    mat[y*4+3] = (int)(a - c) >> 6;
                }
                int oob = 0;
                {
                    pixel *md = p_dst;
                    for( int y = 0; y < 4 && !oob; y++ )
                    {
                        for( int x = 0; x < 4; x++ )
                        {
                            int s = md[x] + mat[y*4+x];
                            if( s < -64 || s > 319 ) { oob = 1; break; }
                        }
                        md += FDEC_STRIDE;
                    }
                }
                if( !oob || mobi_pass >= 16 )
                    break;
                int has_ac = 0;
                for( int i = 1; i < 16; i++ ) if( dct4x4[i] ) { has_ac = 1; break; }
                if( has_ac )
                    for( int i = 1; i < 16; i++ ) dct4x4[i] /= 2;
                else
                    dct4x4[0] /= 2;
            }
            /* The attenuation may have changed the levels; re-store the coded
             * coefficients so the bitstream matches the reconstruction. */
            if( h->param.i_mobiclip && p == 0 )
                for( int zz = 0; zz < 16; zz++ )
                    h->dct.luma4x4[p*16+idx][zz] = dct4x4[mobi_zigzag4x4[zz]];
            pixel *mobi_dst = p_dst;
            for( int y = 0; y < 4; y++ )
            {
                for( int x = 0; x < 4; x++ )
                    mobi_dst[x] = x264_clip_pixel( mobi_dst[x] + mat[y*4+x] );
                mobi_dst += FDEC_STRIDE;
            }
        }
        else
        {
            h->quantf.dequant_4x4( dct4x4, h->dequant4_mf[CQM_4IY], i_qp );
            h->dctf.add4x4_idct( p_dst, dct4x4 );
        }
    }
}

static ALWAYS_INLINE void x264_mb_encode_i8x8( x264_t *h, int p, int idx, int i_qp, int i_mode, pixel *edge, int b_predict )
{
    int x = idx&1;
    int y = idx>>1;
    int nz;
    pixel *p_src = &h->mb.pic.p_fenc[p][8*x + 8*y*FENC_STRIDE];
    pixel *p_dst = &h->mb.pic.p_fdec[p][8*x + 8*y*FDEC_STRIDE];
    ALIGNED_ARRAY_64( dctcoef, dct8x8,[64] );
    ALIGNED_ARRAY_32( pixel, edge_buf,[36] );

    if( b_predict )
    {
        if( !edge )
        {
            h->predict_8x8_filter( p_dst, edge_buf, h->mb.i_neighbour8[idx], x264_pred_i4x4_neighbors[i_mode] );
            edge = edge_buf;
        }

        if( h->mb.b_lossless )
            x264_predict_lossless_8x8( h, p_dst, p, idx, i_mode, edge );
        else
            h->predict_8x8[i_mode]( p_dst, edge );
    }

    if( h->mb.b_lossless )
    {
        nz = h->zigzagf.sub_8x8( h->dct.luma8x8[p*4+idx], p_src, p_dst );
        STORE_8x8_NNZ( p, idx, nz );
        h->mb.i_cbp_luma |= nz<<idx;
        return;
    }

    h->dctf.sub8x8_dct8( dct8x8, p_src, p_dst );

    nz = x264_quant_8x8( h, dct8x8, i_qp, ctx_cat_plane[DCT_LUMA_8x8][p], 1, p, idx );
    if( nz )
    {
        h->mb.i_cbp_luma |= 1<<idx;
		for (int i = 0; i < 64; i++)
			h->dct.luma8x8[p * 4 + idx][i] = dct8x8[ZigZagTable8x8[i]];
        //h->zigzagf.scan_8x8( h->dct.luma8x8[p*4+idx], dct8x8 );
        if( 1 )
        {
            int qx = i_qp % 6;
            /* 8x8 DCT DC = 64*R. Forward step = mobi_q8[qx] (unshifted, via
             * mobi_quant_8x8 clamp). For r6=0 (qy=2): reconstruct = 64R/step * step/64 = R.
             * r6 must match the decoder's qy=2 (header Q=(i_qp%6)+12). */
            int qtab[64];
            for( int zz = 0; zz < 64; zz++ )
                qtab[ZigZagTable8x8[zz]] = mobi_q8[qx][zz];  /* r6=0 always */
            for( int i = 0; i < 64; i++ )
                dct8x8[i] = dct8x8[i] * (unsigned)qtab[i];
        }
        else
        {
            h->quantf.dequant_8x8( dct8x8, h->dequant8_mf[CQM_8IY], i_qp );
        }
        h->dctf.add8x8_idct8( p_dst, dct8x8 );
        STORE_8x8_NNZ( p, idx, 1 );
    }
    else
        STORE_8x8_NNZ( p, idx, 0 );
}

#endif
