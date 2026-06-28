/*****************************************************************************
 * macroblock.c: macroblock encoding
 *****************************************************************************
 * Copyright (C) 2003-2025 x264 project
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Loren Merritt <lorenm@u.washington.edu>
 *          Fiona Glaser <fiona@x264.com>
 *          Henrik Gramner <henrik@gramner.com>
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

#include "common/common.h"
#include "macroblock.h"

/* These chroma DC functions don't have assembly versions and are only used here. */

#define ZIG(i,y,x) level[i] = dct[x*2+y];
static inline void zigzag_scan_2x2_dc( dctcoef level[4], dctcoef dct[4] )
{
    ZIG(0,0,0)
    ZIG(1,0,1)
    ZIG(2,1,0)
    ZIG(3,1,1)
}
#undef ZIG

static inline void zigzag_scan_2x4_dc( dctcoef level[8], dctcoef dct[8] )
{
    level[0] = dct[0];
    level[1] = dct[2];
    level[2] = dct[1];
    level[3] = dct[4];
    level[4] = dct[6];
    level[5] = dct[3];
    level[6] = dct[5];
    level[7] = dct[7];
}

#define IDCT_DEQUANT_2X2_START \
    int d0 = dct[0] + dct[1]; \
    int d1 = dct[2] + dct[3]; \
    int d2 = dct[0] - dct[1]; \
    int d3 = dct[2] - dct[3]; \
    int dmf = dequant_mf[i_qp%6][0] << i_qp/6;

static inline void idct_dequant_2x2_dc( dctcoef dct[4], dctcoef dct4x4[4][16], int dequant_mf[6][16], int i_qp )
{
    IDCT_DEQUANT_2X2_START
    dct4x4[0][0] = (d0 + d1) * dmf >> 5;
    dct4x4[1][0] = (d0 - d1) * dmf >> 5;
    dct4x4[2][0] = (d2 + d3) * dmf >> 5;
    dct4x4[3][0] = (d2 - d3) * dmf >> 5;
}

static inline void idct_dequant_2x2_dconly( dctcoef dct[4], int dequant_mf[6][16], int i_qp )
{
    IDCT_DEQUANT_2X2_START
    dct[0] = (d0 + d1) * dmf >> 5;
    dct[1] = (d0 - d1) * dmf >> 5;
    dct[2] = (d2 + d3) * dmf >> 5;
    dct[3] = (d2 - d3) * dmf >> 5;
}
#undef IDCT_2X2_DEQUANT_START

static inline void dct2x2dc( dctcoef d[4], dctcoef dct4x4[4][16] )
{
    int d0 = dct4x4[0][0] + dct4x4[1][0];
    int d1 = dct4x4[2][0] + dct4x4[3][0];
    int d2 = dct4x4[0][0] - dct4x4[1][0];
    int d3 = dct4x4[2][0] - dct4x4[3][0];
    d[0] = d0 + d1;
    d[2] = d2 + d3;
    d[1] = d0 - d1;
    d[3] = d2 - d3;
    dct4x4[0][0] = 0;
    dct4x4[1][0] = 0;
    dct4x4[2][0] = 0;
    dct4x4[3][0] = 0;
}

static ALWAYS_INLINE int array_non_zero( dctcoef *v, int i_count )
{
    if( WORD_SIZE == 8 )
    {
        for( int i = 0; i < i_count; i += 8/sizeof(dctcoef) )
            if( M64( &v[i] ) )
                return 1;
    }
    else
    {
        for( int i = 0; i < i_count; i += 4/sizeof(dctcoef) )
            if( M32( &v[i] ) )
                return 1;
    }
    return 0;
}

/* All encoding functions must output the correct CBP and NNZ values.
 * The entropy coding functions will check CBP first, then NNZ, before
 * actually reading the DCT coefficients.  NNZ still must be correct even
 * if CBP is zero because of the use of NNZ values for context selection.
 * "NNZ" need only be 0 or 1 rather than the exact coefficient count because
 * that is only needed in CAVLC, and will be calculated by CAVLC's residual
 * coding and stored as necessary. */

/* This means that decimation can be done merely by adjusting the CBP and NNZ
 * rather than memsetting the coefficients. */

static void mb_encode_i16x16( x264_t *h, int p, int i_qp )
{
    pixel *p_src = h->mb.pic.p_fenc[p];
    pixel *p_dst = h->mb.pic.p_fdec[p];

    ALIGNED_ARRAY_64( dctcoef, dct4x4,[16],[16] );
    ALIGNED_ARRAY_64( dctcoef, dct_dc4x4,[16] );

    int nz, block_cbp = 0;
    int decimate_score = h->mb.b_dct_decimate ? 0 : 9;
    int i_quant_cat = p ? CQM_4IC : CQM_4IY;
    int i_mode = h->mb.i_intra16x16_pred_mode;

    if( h->mb.b_lossless )
        x264_predict_lossless_16x16( h, p, i_mode );
    else
        h->predict_16x16[i_mode]( h->mb.pic.p_fdec[p] );

    if( h->mb.b_lossless )
    {
        for( int i = 0; i < 16; i++ )
        {
            int oe = block_idx_xy_fenc[i];
            int od = block_idx_xy_fdec[i];
            nz = h->zigzagf.sub_4x4ac( h->dct.luma4x4[16*p+i], p_src+oe, p_dst+od, &dct_dc4x4[block_idx_yx_1d[i]] );
            h->mb.cache.non_zero_count[x264_scan8[16*p+i]] = nz;
            block_cbp |= nz;
        }
        h->mb.i_cbp_luma |= block_cbp * 0xf;
        h->mb.cache.non_zero_count[x264_scan8[LUMA_DC+p]] = array_non_zero( dct_dc4x4, 16 );
        h->zigzagf.scan_4x4( h->dct.luma16x16_dc[p], dct_dc4x4 );
        return;
    }

    CLEAR_16x16_NNZ( p );

    h->dctf.sub16x16_dct( dct4x4, p_src, p_dst );

    if( h->mb.b_noise_reduction )
        for( int idx = 0; idx < 16; idx++ )
            h->quantf.denoise_dct( dct4x4[idx], h->nr_residual_sum[0], h->nr_offset[0], 16 );

    for( int idx = 0; idx < 16; idx++ )
    {
        dct_dc4x4[block_idx_xy_1d[idx]] = dct4x4[idx][0];
        dct4x4[idx][0] = 0;
    }

    if( h->mb.b_trellis && !h->param.i_mobiclip )
    {
        for( int idx = 0; idx < 16; idx++ )
            if( x264_quant_4x4_trellis( h, dct4x4[idx], i_quant_cat, i_qp, ctx_cat_plane[DCT_LUMA_AC][p], 1, !!p, idx ) )
            {
                block_cbp = 0xf;
                h->zigzagf.scan_4x4( h->dct.luma4x4[16*p+idx], dct4x4[idx] );
                h->quantf.dequant_4x4( dct4x4[idx], h->dequant4_mf[i_quant_cat], i_qp );
                if( decimate_score < 6 ) decimate_score += h->quantf.decimate_score15( h->dct.luma4x4[16*p+idx] );
                h->mb.cache.non_zero_count[x264_scan8[16*p+idx]] = 1;
            }
    }
    else
    {
        for( int i8x8 = 0; i8x8 < 4; i8x8++ )
        {
            nz = h->quantf.quant_4x4x4( &dct4x4[i8x8*4], h->quant4_mf[i_quant_cat][i_qp], h->quant4_bias[i_quant_cat][i_qp] );
            if( nz )
            {
                block_cbp = 0xf;
                FOREACH_BIT( idx, i8x8*4, nz )
                {
                    h->zigzagf.scan_4x4( h->dct.luma4x4[16*p+idx], dct4x4[idx] );
                    h->quantf.dequant_4x4( dct4x4[idx], h->dequant4_mf[i_quant_cat], i_qp );
                    if( decimate_score < 6 ) decimate_score += h->quantf.decimate_score15( h->dct.luma4x4[16*p+idx] );
                    h->mb.cache.non_zero_count[x264_scan8[16*p+idx]] = 1;
                }
            }
        }
    }

    /* Writing the 16 CBFs in an i16x16 block is quite costly, so decimation can save many bits. */
    /* More useful with CAVLC, but still useful with CABAC. */
    if( decimate_score < 6 )
    {
        CLEAR_16x16_NNZ( p );
        block_cbp = 0;
    }
    else
        h->mb.i_cbp_luma |= block_cbp;

    h->dctf.dct4x4dc( dct_dc4x4 );
    if( h->mb.b_trellis && !h->param.i_mobiclip )
        nz = x264_quant_luma_dc_trellis( h, dct_dc4x4, i_quant_cat, i_qp, ctx_cat_plane[DCT_LUMA_DC][p], 1, LUMA_DC+p );
    else
        nz = h->quantf.quant_4x4_dc( dct_dc4x4, h->quant4_mf[i_quant_cat][i_qp][0]>>1, h->quant4_bias[i_quant_cat][i_qp][0]<<1 );

    h->mb.cache.non_zero_count[x264_scan8[LUMA_DC+p]] = nz;
    if( nz )
    {
        h->zigzagf.scan_4x4( h->dct.luma16x16_dc[p], dct_dc4x4 );

        /* output samples to fdec */
        h->dctf.idct4x4dc( dct_dc4x4 );
        h->quantf.dequant_4x4_dc( dct_dc4x4, h->dequant4_mf[i_quant_cat], i_qp );  /* XXX not inversed */
        if( block_cbp )
            for( int i = 0; i < 16; i++ )
                dct4x4[i][0] = dct_dc4x4[block_idx_xy_1d[i]];
    }

    /* put pixels to fdec */
    if( block_cbp )
        h->dctf.add16x16_idct( p_dst, dct4x4 );
    else if( nz )
        h->dctf.add16x16_idct_dc( p_dst, dct_dc4x4 );
}

/* Round down coefficients losslessly in DC-only chroma blocks.
 * Unlike luma blocks, this can't be done with a lookup table or
 * other shortcut technique because of the interdependencies
 * between the coefficients due to the chroma DC transform. */
static ALWAYS_INLINE int mb_optimize_chroma_dc( x264_t *h, dctcoef *dct_dc, int dequant_mf[6][16], int i_qp, int chroma422 )
{
    int dmf = dequant_mf[i_qp%6][0] << i_qp/6;

    /* If the QP is too high, there's no benefit to rounding optimization. */
    if( dmf > 32*64 )
        return 1;

    if( chroma422 )
        return h->quantf.optimize_chroma_2x4_dc( dct_dc, dmf );
    else
        return h->quantf.optimize_chroma_2x2_dc( dct_dc, dmf );
}

/* Decoder-exact inverse transforms (mobiclip.c inverse4()/idct()).  Used for
 * chroma 8x8 reconstruction so the encoder's reference matches the decoder
 * bit-for-bit (x264's add8x8_idct8 has different normalisation/rounding, which
 * accumulates as chroma drift across P-frames).  Defined outside the
 * !HIGH_BIT_DEPTH guard because mb_encode_chroma_internal compiles for both
 * bit depths (the mobiclip path is dead code at high bit depth but must link). */
static ALWAYS_INLINE void mobi_inverse4( int *rs )
{
    unsigned a = (unsigned)rs[0] + rs[2];
    unsigned b = (unsigned)rs[0] - rs[2];
    unsigned c = (unsigned)rs[1] + (rs[3] >> 1);
    unsigned d = (rs[1] >> 1) - (unsigned)rs[3];
    rs[0] = a + c;
    rs[1] = b + d;
    rs[2] = b - d;
    rs[3] = a - c;
}
static ALWAYS_INLINE void mobi_idct8_1d( int *arr )
{
    int e, f, g, h;
    unsigned x0, x1, x2, x3;
    int tmp[4];
    tmp[0] = arr[0]; tmp[1] = arr[2]; tmp[2] = arr[4]; tmp[3] = arr[6];
    mobi_inverse4( tmp );
    e = (unsigned)arr[7] + arr[1] - arr[3] - (arr[3] >> 1);
    f = (unsigned)arr[7] - arr[1] + arr[5] + (arr[5] >> 1);
    g = (unsigned)arr[5] - arr[3] - arr[7] - (arr[7] >> 1);
    h = (unsigned)arr[5] + arr[3] + arr[1] + (arr[1] >> 1);
    x3 = (unsigned)g + (h >> 2);
    x2 = (unsigned)e + (f >> 2);
    x1 = (e >> 2) - (unsigned)f;
    x0 = (unsigned)h - (g >> 2);
    arr[0] = tmp[0] + x0; arr[1] = tmp[1] + x1; arr[2] = tmp[2] + x2; arr[3] = tmp[3] + x3;
    arr[4] = tmp[3] - x3; arr[5] = tmp[2] - x2; arr[6] = tmp[1] - x1; arr[7] = tmp[0] - x0;
}
/* mat[64] holds the dequantized coefficients in raster order; reconstruct and
 * add to p_dst (FDEC_STRIDE) exactly as the decoder's add_coefficients() does. */
static ALWAYS_INLINE void mobi_add8x8_idct8( pixel *p_dst, int *mat )
{
    mat[0] += 32;
    for( int y = 0; y < 8; y++ )
        mobi_idct8_1d( &mat[y*8] );
    for( int y = 0; y < 8; y++ )
    {
        for( int x = y+1; x < 8; x++ )
        {
            int a = mat[x*8+y], b = mat[y*8+x];
            mat[y*8+x] = a;
            mat[x*8+y] = b;
        }
        mobi_idct8_1d( &mat[y*8] );
        for( int x = 0; x < 8; x++ )
            p_dst[y*FDEC_STRIDE+x] = x264_clip_pixel( p_dst[y*FDEC_STRIDE+x] + (mat[y*8+x] >> 6) );
    }
}
#if !HIGH_BIT_DEPTH
/* Mobiclip's motion compensation uses a simple half-pel averaging filter that
 * differs from H.264's 6-tap luma / bilinear chroma interpolation.  For inter
 * MBs we must build the prediction with Mobiclip's filter so the residual the
 * encoder computes reconstructs identically in the decoder.  method bits:
 * bit0 = x half-pel, bit1 = y half-pel (matches predict_motion()). xs is the
 * element stride in x (1 for planar luma/chroma). */
static ALWAYS_INLINE void mobi_interp_block( pixel *dst, int ds, pixel *src, int ss,
                                             int xs, int w, int hgt, int method )
{
    for( int y = 0; y < hgt; y++ )
    {
        for( int x = 0; x < w; x++ )
        {
            pixel *s = src + x * xs;
            int v;
            switch( method )
            {
            default:
            case 0: v = s[0]; break;
            case 1: v = (s[0] >> 1) + (s[xs] >> 1); break;
            case 2: v = (s[0] >> 1) + (s[ss] >> 1); break;
            case 3: v = (((s[0] >> 1) + (s[xs] >> 1)) >> 1) +
                        (((s[ss] >> 1) + (s[xs + ss] >> 1)) >> 1); break;
            }
            dst[x] = v;
        }
        dst += ds;
        src += ss;
    }
}

/* Overwrite p_fdec[0..2] for the current inter MB with Mobiclip's MC. */
static void mobi_motion_compensate( x264_t *h )
{
    int sy = h->mb.pic.i_stride[0];
    int sc = h->mb.pic.i_stride[1];

    /* Clamp the MV so the motion-compensated 16x16 block stays fully inside the
     * reference frame.  The Mobiclip decoder reads raw reference data with no
     * border padding and rejects (predict_motion bounds check) any MV whose
     * block falls outside [0,W]x[0,H].  x264's own clamps allow border reads and
     * skip/predictor MVs can bypass them, so enforce the decoder's exact
     * constraint here (half-pel hmv = cache.mv>>1).  Writing the clamped MV back
     * to cache.mv (as an even value) keeps the MC, the bitstream MV write, and
     * the predictor chain all consistent and bit-exact with the decoder. */
    {
        int W = h->param.i_width, He = h->param.i_height;
        int ox = h->mb.i_mb_x*16, oy = h->mb.i_mb_y*16;
        /* The +4/-4 half-pel margin (2 luma px) keeps the half-resolution chroma
         * MC in bounds too: chroma uses hmv>>1 and can round 1px past the luma
         * limit. */
        /* Clamp the 16x16 block's integer position to stay inside the frame.
         * No safety margin on the max side: a previous +/-4 (2px) margin made
         * hxmax/hymax negative for the right/bottom edge macroblocks, which
         * forced even a zero MV to shift 2px inward there.  That mispredicted
         * the outermost rows/columns and made them jitter frame-to-frame.  The
         * decoder validates the block position (zero-MV at the edge is exactly
         * in bounds), and at the edge the clamp now only permits MVs pointing
         * back into the frame, so chroma sub-pel can't round past the border
         * either. */
        int hxmin = -2*ox,           hxmax = 2*(W  - 16 - ox);
        int hymin = -2*oy,           hymax = 2*(He - 16 - oy);
        for( int i = 0; i < 16; i++ )
        {
            int i8 = x264_scan8[0] + (i&3) + (i>>2)*8;
            int hx = h->mb.cache.mv[0][i8][0] >> 1;
            int hy = h->mb.cache.mv[0][i8][1] >> 1;
            hx = x264_clip3( hx, hxmin, hxmax );
            hy = x264_clip3( hy, hymin, hymax );
            h->mb.cache.mv[0][i8][0] = hx << 1;
            h->mb.cache.mv[0][i8][1] = hy << 1;
        }
    }

    /* Luma, per 4x4 block (cache.mv is populated per 4x4). */
    for( int by = 0; by < 16; by += 4 )
        for( int bx = 0; bx < 16; bx += 4 )
        {
            int i8   = x264_scan8[0] + (bx >> 2) + (by >> 2) * 8;
            int iref = h->mb.cache.ref[0][i8];
            if( iref < 0 ) iref = 0;
            int hmvx = h->mb.cache.mv[0][i8][0] >> 1;   /* quarter -> half pel */
            int hmvy = h->mb.cache.mv[0][i8][1] >> 1;
            int method = (hmvx & 1) | ((hmvy & 1) << 1);
            pixel *ref = h->mb.pic.p_fref[0][iref][0];
            pixel *src = ref + (by + (hmvy >> 1)) * sy + (bx + (hmvx >> 1));
            pixel *dst = h->mb.pic.p_fdec[0] + by * FDEC_STRIDE + bx;
            mobi_interp_block( dst, FDEC_STRIDE, src, sy, 1, 4, 4, method );
        }

    /* Chroma (4:2:0), per 4x4 chroma block = one luma 8x8 quadrant.  The
     * reference chroma is interleaved UV; the decoder's chroma MV is the luma
     * (half-pel) MV >> 1 and operates on half-resolution coordinates. */
    for( int q = 0; q < 4; q++ )
    {
        int cqx = (q & 1) * 4;
        int cqy = (q >> 1) * 4;
        /* Cache index of the luma 8x8 quadrant's top-left 4x4.  The cache is an
         * 8-wide grid, so the quadrant at 4x4-grid (col,row)=((q&1)*2,(q>>1)*2)
         * is scan8[0] + col + row*8.  (q*4 would read neighbouring MBs.) */
        int i8  = x264_scan8[0] + (q & 1) * 2 + (q >> 1) * 2 * 8;
        int iref = h->mb.cache.ref[0][i8];
        if( iref < 0 ) iref = 0;
        int cmvx = (h->mb.cache.mv[0][i8][0] >> 1) >> 1;  /* -> chroma half-pel */
        int cmvy = (h->mb.cache.mv[0][i8][1] >> 1) >> 1;
        int method = (cmvx & 1) | ((cmvy & 1) << 1);
        /* Mobiclip chroma is PLANAR: U = p_fref[..][4] (plane[1]),
         * V = p_fref[..][8] (plane[2]).  Read each plane with unit stride
         * (xs=1), NOT interleaved (xs=2).  The decoder does motion compensation
         * separately on data[1] (U) and data[2] (V). */
        int srcoff = (cqy + (cmvy >> 1)) * sc + (cqx + (cmvx >> 1));
        for( int ch = 0; ch < 2; ch++ )
        {
            pixel *refc = h->mb.pic.p_fref[0][iref][4 + ch*4];  /* [4]=U, [8]=V */
            pixel *src = refc + srcoff;
            pixel *dst = h->mb.pic.p_fdec[1 + ch] + cqy * FDEC_STRIDE + cqx;
            mobi_interp_block( dst, FDEC_STRIDE, src, sc, 1, 4, 4, method );
        }
    }
}
#endif

static ALWAYS_INLINE void mb_encode_chroma_internal( x264_t *h, int b_inter, int i_qp, int chroma422 )
{
    int nz, nz_dc;
    int b_decimate = b_inter && h->mb.b_dct_decimate;
    int (*dequant_mf)[16] = h->dequant4_mf[CQM_4IC + b_inter];
    ALIGNED_ARRAY_16( dctcoef, dct_dc,[8] );
    h->mb.i_cbp_chroma = 0;
    h->nr_count[2] += h->mb.b_noise_reduction * 4;

    M16( &h->mb.cache.non_zero_count[x264_scan8[16]] ) = 0;
    M16( &h->mb.cache.non_zero_count[x264_scan8[18]] ) = 0;
    M16( &h->mb.cache.non_zero_count[x264_scan8[32]] ) = 0;
    M16( &h->mb.cache.non_zero_count[x264_scan8[34]] ) = 0;
    if( chroma422 )
    {
        M16( &h->mb.cache.non_zero_count[x264_scan8[24]] ) = 0;
        M16( &h->mb.cache.non_zero_count[x264_scan8[26]] ) = 0;
        M16( &h->mb.cache.non_zero_count[x264_scan8[40]] ) = 0;
        M16( &h->mb.cache.non_zero_count[x264_scan8[42]] ) = 0;
    }

    if( h->param.i_mobiclip )
    {
        for( int ch = 0; ch < 2; ch++ )
        {
            pixel *p_src = h->mb.pic.p_fenc[1+ch];
            pixel *p_dst = h->mb.pic.p_fdec[1+ch];

            ALIGNED_ARRAY_64(dctcoef, dct8x8, [64]);

            h->dctf.sub8x8_dct8(dct8x8, p_src, p_dst);

            nz = mobi_quant_8x8(h, dct8x8, i_qp, !b_inter);
            if (nz)
            {
                h->mb.i_cbp_chroma |= 1 << ch;
                for (int i = 0; i < 64; i++)
                    h->dct.luma8x8[4 + 4 * ch][i] = dct8x8[mobi_zigzag8x8[i]];
                /* Use Mobi-specific dequant (r6 = qy-2), not x264 standard (qbits = qy-6).
                 * The standard dequant has a 16x smaller shift, causing color corruption. */
                {
                    int qx = i_qp % 6;
                    /* Use r6=0 always to match header QP=(i_qp%6)+12 → qy=2, r6=0. */
                    static const uint8_t mobi_q8_chroma[6][64] = {
                        {0x14,0x13,0x13,0x19,0x12,0x19,0x13,0x18,0x18,0x13,0x14,0x12,0x20,0x12,0x14,0x13,0x13,0x18,0x18,0x13,0x13,0x19,0x12,0x19,0x12,0x19,0x12,0x19,0x13,0x18,0x18,0x13,0x13,0x18,0x18,0x13,0x12,0x20,0x12,0x14,0x12,0x20,0x12,0x18,0x18,0x13,0x13,0x18,0x18,0x12,0x19,0x12,0x19,0x12,0x13,0x18,0x18,0x13,0x12,0x20,0x12,0x18,0x18,0x12},
                        {0x16,0x15,0x15,0x1C,0x13,0x1C,0x15,0x1A,0x1A,0x15,0x16,0x13,0x23,0x13,0x16,0x15,0x15,0x1A,0x1A,0x15,0x15,0x1C,0x13,0x1C,0x13,0x1C,0x13,0x1C,0x15,0x1A,0x1A,0x15,0x15,0x1A,0x1A,0x15,0x13,0x23,0x13,0x16,0x13,0x23,0x13,0x1A,0x1A,0x15,0x15,0x1A,0x1A,0x13,0x1C,0x13,0x1C,0x13,0x15,0x1A,0x1A,0x15,0x13,0x23,0x13,0x1A,0x1A,0x13},
                        {0x1A,0x18,0x18,0x21,0x17,0x21,0x18,0x1F,0x1F,0x18,0x1A,0x17,0x2A,0x17,0x1A,0x18,0x18,0x1F,0x1F,0x18,0x18,0x21,0x17,0x21,0x17,0x21,0x17,0x21,0x18,0x1F,0x1F,0x18,0x18,0x1F,0x1F,0x18,0x17,0x2A,0x17,0x1A,0x17,0x2A,0x17,0x1F,0x1F,0x18,0x18,0x1F,0x1F,0x17,0x21,0x17,0x21,0x17,0x18,0x1F,0x1F,0x18,0x17,0x2A,0x17,0x1F,0x1F,0x17},
                        {0x1C,0x1A,0x1A,0x23,0x19,0x23,0x1A,0x21,0x21,0x1A,0x1C,0x19,0x2D,0x19,0x1C,0x1A,0x1A,0x21,0x21,0x1A,0x1A,0x23,0x19,0x23,0x19,0x23,0x19,0x23,0x1A,0x21,0x21,0x1A,0x1A,0x21,0x21,0x1A,0x19,0x2D,0x19,0x1C,0x19,0x2D,0x19,0x21,0x21,0x1A,0x1A,0x21,0x21,0x19,0x23,0x19,0x23,0x19,0x1A,0x21,0x21,0x1A,0x19,0x2D,0x19,0x21,0x21,0x19},
                        {0x20,0x1E,0x1E,0x28,0x1C,0x28,0x1E,0x26,0x26,0x1E,0x20,0x1C,0x33,0x1C,0x20,0x1E,0x1E,0x26,0x26,0x1E,0x1E,0x28,0x1C,0x28,0x1C,0x28,0x1C,0x28,0x1E,0x26,0x26,0x1E,0x1E,0x26,0x26,0x1E,0x1C,0x33,0x1C,0x20,0x1C,0x33,0x1C,0x26,0x26,0x1E,0x1E,0x26,0x26,0x1C,0x28,0x1C,0x28,0x1C,0x1E,0x26,0x26,0x1E,0x1C,0x33,0x1C,0x26,0x26,0x1C},
                        {0x24,0x22,0x22,0x2E,0x20,0x2E,0x22,0x2B,0x2B,0x22,0x24,0x20,0x3A,0x20,0x24,0x22,0x22,0x2B,0x2B,0x22,0x22,0x2E,0x20,0x2E,0x20,0x2E,0x20,0x2E,0x22,0x2B,0x2B,0x22,0x22,0x2B,0x2B,0x22,0x20,0x3A,0x20,0x24,0x20,0x3A,0x20,0x2B,0x2B,0x22,0x22,0x2B,0x2B,0x20,0x2E,0x20,0x2E,0x20,0x22,0x2B,0x2B,0x22,0x20,0x3A,0x20,0x2B,0x2B,0x20},
                    };
                    int qtab[64];
                    for( int zz = 0; zz < 64; zz++ )
                        qtab[mobi_zigzag8x8[zz]] = mobi_q8_chroma[qx][zz] << mobi_qyx(h);  /* qy-2 = MOBI_QYX */
                    /* Dequantize into a 32-bit matrix (qval*level overflows the
                     * int16 dct8x8 buffer) and reconstruct with the decoder's
                     * exact inverse transform for a bit-exact reference. */
                    int mat[64];
                    for( int i = 0; i < 64; i++ )
                        mat[i] = (int)dct8x8[i] * qtab[i];
                    mobi_add8x8_idct8( p_dst, mat );
                }
                h->mb.cache.non_zero_count[x264_scan8[16 + ch * 16]] = 1;
            }
            else
                h->mb.cache.non_zero_count[x264_scan8[16 + ch * 16]] = 0;
            /* (The reconstructed chroma is committed to the frame buffer by
             * macroblock_store_pic / cache_save, which now stores planar
             * chroma correctly — no manual write-back needed here.) */
        }
    }
    else
    {
        /* Early termination: check variance of chroma residual before encoding.
         * Don't bother trying early termination at low QPs.
         * Values are experimentally derived. */
        if( b_decimate && i_qp >= (h->mb.b_trellis ? 12 : 18) && !h->mb.b_noise_reduction )
        {
            int thresh = chroma422 ? (x264_lambda2_tab[i_qp] + 16) >> 5 : (x264_lambda2_tab[i_qp] + 32) >> 6;
            ALIGNED_ARRAY_8( int, ssd,[2] );
            int chromapix = chroma422 ? PIXEL_8x16 : PIXEL_8x8;

            if( h->pixf.var2[chromapix]( h->mb.pic.p_fenc[1], h->mb.pic.p_fdec[1], ssd ) < thresh*4 )
            {
                h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+0]] = 0;
                h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+1]] = 0;

                for( int ch = 0; ch < 2; ch++ )
                {
                    if( ssd[ch] > thresh )
                    {
                        pixel *p_src = h->mb.pic.p_fenc[1+ch];
                        pixel *p_dst = h->mb.pic.p_fdec[1+ch];
                        if( chroma422 )
                            /* Cannot be replaced by two calls to sub8x8_dct_dc since the hadamard transform is different */
                            h->dctf.sub8x16_dct_dc( dct_dc, p_src, p_dst );
                        else
                            h->dctf.sub8x8_dct_dc( dct_dc, p_src, p_dst );

                        if( h->mb.b_trellis )
                            nz_dc = x264_quant_chroma_dc_trellis( h, dct_dc, i_qp+3*chroma422, !b_inter, CHROMA_DC+ch );
                        else
                        {
                            nz_dc = 0;
                            for( int i = 0; i <= chroma422; i++ )
                                nz_dc |= h->quantf.quant_2x2_dc( &dct_dc[4*i], h->quant4_mf[CQM_4IC+b_inter][i_qp+3*chroma422][0] >> 1,
                                                                 h->quant4_bias[CQM_4IC+b_inter][i_qp+3*chroma422][0] << 1 );
                        }

                        if( nz_dc )
                        {
                            if( !mb_optimize_chroma_dc( h, dct_dc, dequant_mf, i_qp+3*chroma422, chroma422 ) )
                                continue;
                            h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+ch]] = 1;
                            if( chroma422 )
                            {
                                zigzag_scan_2x4_dc( h->dct.chroma_dc[ch], dct_dc );
                                h->quantf.idct_dequant_2x4_dconly( dct_dc, dequant_mf, i_qp+3 );
                            }
                            else
                            {
                                zigzag_scan_2x2_dc( h->dct.chroma_dc[ch], dct_dc );
                                idct_dequant_2x2_dconly( dct_dc, dequant_mf, i_qp );
                            }

                            for( int i = 0; i <= chroma422; i++ )
                                h->dctf.add8x8_idct_dc( p_dst + 8*i*FDEC_STRIDE, &dct_dc[4*i] );
                            h->mb.i_cbp_chroma = 1;
                        }
                    }
                }
                return;
            }
        }

        for( int ch = 0; ch < 2; ch++ )
        {
            pixel *p_src = h->mb.pic.p_fenc[1+ch];
            pixel *p_dst = h->mb.pic.p_fdec[1+ch];
            int i_decimate_score = b_decimate ? 0 : 7;
            int nz_ac = 0;

            ALIGNED_ARRAY_64( dctcoef, dct4x4,[8],[16] );

            if( h->mb.b_lossless )
            {
                static const uint8_t chroma422_scan[8] = { 0, 2, 1, 5, 3, 6, 4, 7 };

                for( int i = 0; i < (chroma422?8:4); i++ )
                {
                    int oe = 4*(i&1) + 4*(i>>1)*FENC_STRIDE;
                    int od = 4*(i&1) + 4*(i>>1)*FDEC_STRIDE;
                    nz = h->zigzagf.sub_4x4ac( h->dct.luma4x4[16+i+(chroma422?i&4:0)+ch*16], p_src+oe, p_dst+od,
                                               &h->dct.chroma_dc[ch][chroma422?chroma422_scan[i]:i] );
                    h->mb.cache.non_zero_count[x264_scan8[16+i+(chroma422?i&4:0)+ch*16]] = nz;
                    h->mb.i_cbp_chroma |= nz;
                }
                h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+ch]] = array_non_zero( h->dct.chroma_dc[ch], chroma422?8:4 );
                continue;
            }

            for( int i = 0; i <= chroma422; i++ )
                h->dctf.sub8x8_dct( &dct4x4[4*i], p_src + 8*i*FENC_STRIDE, p_dst + 8*i*FDEC_STRIDE );

            if( h->mb.b_noise_reduction )
                for( int i = 0; i < (chroma422?8:4); i++ )
                    h->quantf.denoise_dct( dct4x4[i], h->nr_residual_sum[2], h->nr_offset[2], 16 );

            if( chroma422 )
                h->dctf.dct2x4dc( dct_dc, dct4x4 );
            else
                dct2x2dc( dct_dc, dct4x4 );

            /* calculate dct coeffs */
            for( int i8x8 = 0; i8x8 < (chroma422?2:1); i8x8++ )
            {
                if( h->mb.b_trellis )
                {
                    for( int i4x4 = 0; i4x4 < 4; i4x4++ )
                    {
                        if( x264_quant_4x4_trellis( h, dct4x4[i8x8*4+i4x4], CQM_4IC+b_inter, i_qp, DCT_CHROMA_AC, !b_inter, 1, 0 ) )
                        {
                            int idx = 16+ch*16+i8x8*8+i4x4;
                            h->zigzagf.scan_4x4( h->dct.luma4x4[idx], dct4x4[i8x8*4+i4x4] );
                            h->quantf.dequant_4x4( dct4x4[i8x8*4+i4x4], dequant_mf, i_qp );
                            if( i_decimate_score < 7 )
                                i_decimate_score += h->quantf.decimate_score15( h->dct.luma4x4[idx] );
                            h->mb.cache.non_zero_count[x264_scan8[idx]] = 1;
                            nz_ac = 1;
                        }
                    }
                }
                else
                {
                    nz = h->quantf.quant_4x4x4( &dct4x4[i8x8*4], h->quant4_mf[CQM_4IC+b_inter][i_qp],
                                                h->quant4_bias[CQM_4IC+b_inter][i_qp] );
                    nz_ac |= nz;

                    FOREACH_BIT( i4x4, 0, nz )
                    {
                        int idx = 16+ch*16+i8x8*8+i4x4;

                        h->zigzagf.scan_4x4( h->dct.luma4x4[idx], dct4x4[i8x8*4+i4x4] );
                        h->quantf.dequant_4x4( dct4x4[i8x8*4+i4x4], dequant_mf, i_qp );
                        if( i_decimate_score < 7 )
                            i_decimate_score += h->quantf.decimate_score15( h->dct.luma4x4[idx] );
                        h->mb.cache.non_zero_count[x264_scan8[idx]] = 1;
                    }
                }
            }

            if( h->mb.b_trellis )
                nz_dc = x264_quant_chroma_dc_trellis( h, dct_dc, i_qp+3*chroma422, !b_inter, CHROMA_DC+ch );
            else
            {
                nz_dc = 0;
                for( int i = 0; i <= chroma422; i++ )
                    nz_dc |= h->quantf.quant_2x2_dc( &dct_dc[4*i], h->quant4_mf[CQM_4IC+b_inter][i_qp+3*chroma422][0] >> 1,
                                                     h->quant4_bias[CQM_4IC+b_inter][i_qp+3*chroma422][0] << 1 );
            }

            h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+ch]] = nz_dc;

            if( i_decimate_score < 7 || !nz_ac )
            {
                /* Decimate the block */
                M16( &h->mb.cache.non_zero_count[x264_scan8[16+16*ch]] ) = 0;
                M16( &h->mb.cache.non_zero_count[x264_scan8[18+16*ch]] ) = 0;
                if( chroma422 )
                {
                    M16( &h->mb.cache.non_zero_count[x264_scan8[24+16*ch]] ) = 0;
                    M16( &h->mb.cache.non_zero_count[x264_scan8[26+16*ch]] ) = 0;
                }

                if( !nz_dc ) /* Whole block is empty */
                    continue;
                if( !mb_optimize_chroma_dc( h, dct_dc, dequant_mf, i_qp+3*chroma422, chroma422 ) )
                {
                    h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+ch]] = 0;
                    continue;
                }
                /* DC-only */
                if( chroma422 )
                {
                    zigzag_scan_2x4_dc( h->dct.chroma_dc[ch], dct_dc );
                    h->quantf.idct_dequant_2x4_dconly( dct_dc, dequant_mf, i_qp+3 );
                }
                else
                {
                    zigzag_scan_2x2_dc( h->dct.chroma_dc[ch], dct_dc );
                    idct_dequant_2x2_dconly( dct_dc, dequant_mf, i_qp );
                }

                for( int i = 0; i <= chroma422; i++ )
                    h->dctf.add8x8_idct_dc( p_dst + 8*i*FDEC_STRIDE, &dct_dc[4*i] );
            }
            else
            {
                h->mb.i_cbp_chroma = 1;

                if( nz_dc )
                {
                    if( chroma422 )
                    {
                        zigzag_scan_2x4_dc( h->dct.chroma_dc[ch], dct_dc );
                        h->quantf.idct_dequant_2x4_dc( dct_dc, dct4x4, dequant_mf, i_qp+3 );
                    }
                    else
                    {
                        zigzag_scan_2x2_dc( h->dct.chroma_dc[ch], dct_dc );
                        idct_dequant_2x2_dc( dct_dc, dct4x4, dequant_mf, i_qp );
                    }
                }

                for( int i = 0; i <= chroma422; i++ )
                    h->dctf.add8x8_idct( p_dst + 8*i*FDEC_STRIDE, &dct4x4[4*i] );
            }
        }

        /* 0 = none, 1 = DC only, 2 = DC+AC */
        h->mb.i_cbp_chroma += (h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+0]] |
                               h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+1]] | h->mb.i_cbp_chroma);
    }
}

void x264_mb_encode_chroma( x264_t *h, int b_inter, int i_qp )
{
    if( CHROMA_FORMAT == CHROMA_420 )
        mb_encode_chroma_internal( h, b_inter, i_qp, 0 );
    else
        mb_encode_chroma_internal( h, b_inter, i_qp, 1 );
}

static void macroblock_encode_skip( x264_t *h )
{
    M32( &h->mb.cache.non_zero_count[x264_scan8[ 0]] ) = 0;
    M32( &h->mb.cache.non_zero_count[x264_scan8[ 2]] ) = 0;
    M32( &h->mb.cache.non_zero_count[x264_scan8[ 8]] ) = 0;
    M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = 0;
    M32( &h->mb.cache.non_zero_count[x264_scan8[16+ 0]] ) = 0;
    M32( &h->mb.cache.non_zero_count[x264_scan8[16+ 2]] ) = 0;
    M32( &h->mb.cache.non_zero_count[x264_scan8[32+ 0]] ) = 0;
    M32( &h->mb.cache.non_zero_count[x264_scan8[32+ 2]] ) = 0;
    if( CHROMA_FORMAT >= CHROMA_422 )
    {
        M32( &h->mb.cache.non_zero_count[x264_scan8[16+ 8]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[16+10]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[32+ 8]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[32+10]] ) = 0;
    }
    h->mb.i_cbp_luma = 0;
    h->mb.i_cbp_chroma = 0;
    h->mb.cbp[h->mb.i_mb_xy] = 0;
}

/*****************************************************************************
 * Intra prediction for predictive lossless mode.
 *****************************************************************************/

void x264_predict_lossless_chroma( x264_t *h, int i_mode )
{
    int height = 16 >> CHROMA_V_SHIFT;
    if( i_mode == I_PRED_CHROMA_V )
    {
        h->mc.copy[PIXEL_8x8]( h->mb.pic.p_fdec[1], FDEC_STRIDE, h->mb.pic.p_fenc[1]-FENC_STRIDE, FENC_STRIDE, height );
        h->mc.copy[PIXEL_8x8]( h->mb.pic.p_fdec[2], FDEC_STRIDE, h->mb.pic.p_fenc[2]-FENC_STRIDE, FENC_STRIDE, height );
        memcpy( h->mb.pic.p_fdec[1], h->mb.pic.p_fdec[1]-FDEC_STRIDE, 8*SIZEOF_PIXEL );
        memcpy( h->mb.pic.p_fdec[2], h->mb.pic.p_fdec[2]-FDEC_STRIDE, 8*SIZEOF_PIXEL );
    }
    else if( i_mode == I_PRED_CHROMA_H )
    {
        h->mc.copy[PIXEL_8x8]( h->mb.pic.p_fdec[1], FDEC_STRIDE, h->mb.pic.p_fenc[1]-1, FENC_STRIDE, height );
        h->mc.copy[PIXEL_8x8]( h->mb.pic.p_fdec[2], FDEC_STRIDE, h->mb.pic.p_fenc[2]-1, FENC_STRIDE, height );
        x264_copy_column8( h->mb.pic.p_fdec[1]+4*FDEC_STRIDE, h->mb.pic.p_fdec[1]+4*FDEC_STRIDE-1 );
        x264_copy_column8( h->mb.pic.p_fdec[2]+4*FDEC_STRIDE, h->mb.pic.p_fdec[2]+4*FDEC_STRIDE-1 );
        if( CHROMA_FORMAT == CHROMA_422 )
        {
            x264_copy_column8( h->mb.pic.p_fdec[1]+12*FDEC_STRIDE, h->mb.pic.p_fdec[1]+12*FDEC_STRIDE-1 );
            x264_copy_column8( h->mb.pic.p_fdec[2]+12*FDEC_STRIDE, h->mb.pic.p_fdec[2]+12*FDEC_STRIDE-1 );
        }
    }
    else
    {
        h->predict_chroma[i_mode]( h->mb.pic.p_fdec[1] );
        h->predict_chroma[i_mode]( h->mb.pic.p_fdec[2] );
    }
}

void x264_predict_lossless_4x4( x264_t *h, pixel *p_dst, int p, int idx, int i_mode )
{
    int stride = h->fenc->i_stride[p] << MB_INTERLACED;
    pixel *p_src = h->mb.pic.p_fenc_plane[p] + block_idx_x[idx]*4 + block_idx_y[idx]*4 * stride;

    if( i_mode == I_PRED_4x4_V )
    {
        h->mc.copy[PIXEL_4x4]( p_dst, FDEC_STRIDE, p_src-stride, stride, 4 );
        memcpy( p_dst, p_dst-FDEC_STRIDE, 4*SIZEOF_PIXEL );
    }
    else if( i_mode == I_PRED_4x4_H )
    {
        h->mc.copy[PIXEL_4x4]( p_dst, FDEC_STRIDE, p_src-1, stride, 4 );
        for( int i = 0; i < 4; i++ )
            p_dst[i*FDEC_STRIDE] = p_dst[i*FDEC_STRIDE-1];
    }
    else
        h->predict_4x4[i_mode]( p_dst );
}

void x264_predict_lossless_8x8( x264_t *h, pixel *p_dst, int p, int idx, int i_mode, pixel edge[36] )
{
    int stride = h->fenc->i_stride[p] << MB_INTERLACED;
    pixel *p_src = h->mb.pic.p_fenc_plane[p] + (idx&1)*8 + (idx>>1)*8*stride;

    if( i_mode == I_PRED_8x8_V )
    {
        h->mc.copy[PIXEL_8x8]( p_dst, FDEC_STRIDE, p_src-stride, stride, 8 );
        memcpy( p_dst, &edge[16], 8*SIZEOF_PIXEL );
    }
    else if( i_mode == I_PRED_8x8_H )
    {
        h->mc.copy[PIXEL_8x8]( p_dst, FDEC_STRIDE, p_src-1, stride, 8 );
        for( int i = 0; i < 8; i++ )
            p_dst[i*FDEC_STRIDE] = edge[14-i];
    }
    else
        h->predict_8x8[i_mode]( p_dst, edge );
}

void x264_predict_lossless_16x16( x264_t *h, int p, int i_mode )
{
    int stride = h->fenc->i_stride[p] << MB_INTERLACED;
    pixel *p_dst = h->mb.pic.p_fdec[p];

    if( i_mode == I_PRED_16x16_V )
    {
        h->mc.copy[PIXEL_16x16]( p_dst, FDEC_STRIDE, h->mb.pic.p_fenc_plane[p]-stride, stride, 16 );
        memcpy( p_dst, p_dst-FDEC_STRIDE, 16*SIZEOF_PIXEL );
    }
    else if( i_mode == I_PRED_16x16_H )
    {
        h->mc.copy_16x16_unaligned( p_dst, FDEC_STRIDE, h->mb.pic.p_fenc_plane[p]-1, stride, 16 );
        for( int i = 0; i < 16; i++ )
            p_dst[i*FDEC_STRIDE] = p_dst[i*FDEC_STRIDE-1];
    }
    else
        h->predict_16x16[i_mode]( p_dst );
}

/*****************************************************************************
 * x264_macroblock_encode:
 *****************************************************************************/
static ALWAYS_INLINE void macroblock_encode_internal( x264_t *h, int plane_count, int chroma )
{
    int i_qp = h->mb.i_qp;
    int b_decimate = h->mb.b_dct_decimate;
    int b_force_no_skip = 0;
    int nz;
    h->mb.i_cbp_luma = 0;
    for( int p = 0; p < plane_count; p++ )
        h->mb.cache.non_zero_count[x264_scan8[LUMA_DC+p]] = 0;

    /*if( h->mb.i_type == I_PCM )
    {
        /* if PCM is chosen, we need to store reconstructed frame data /
        for( int p = 0; p < plane_count; p++ )
            h->mc.copy[PIXEL_16x16]( h->mb.pic.p_fdec[p], FDEC_STRIDE, h->mb.pic.p_fenc[p], FENC_STRIDE, 16 );
        if( chroma )
        {
            int height = 16 >> CHROMA_V_SHIFT;
            h->mc.copy[PIXEL_8x8]  ( h->mb.pic.p_fdec[1], FDEC_STRIDE, h->mb.pic.p_fenc[1], FENC_STRIDE, height );
            h->mc.copy[PIXEL_8x8]  ( h->mb.pic.p_fdec[2], FDEC_STRIDE, h->mb.pic.p_fenc[2], FENC_STRIDE, height );
        }
        return;
    }*/

    if(TRUE)//!h->mb.b_allow_skip )
    {
        b_force_no_skip = 1;
        if( IS_SKIP(h->mb.i_type) )
        {
            if( h->mb.i_type == P_SKIP )
                h->mb.i_type = P_L0;
            else if( h->mb.i_type == B_SKIP )
                h->mb.i_type = B_DIRECT;
        }
    }

    if( h->mb.i_type == P_SKIP )
    {
        /* don't do pskip motion compensation if it was already done in macroblock_analyse */
        if( !h->mb.b_skip_mc )
        {
            int mvx = x264_clip3( h->mb.cache.mv[0][x264_scan8[0]][0],
                                  h->mb.mv_min[0], h->mb.mv_max[0] );
            int mvy = x264_clip3( h->mb.cache.mv[0][x264_scan8[0]][1],
                                  h->mb.mv_min[1], h->mb.mv_max[1] );

            for( int p = 0; p < plane_count; p++ )
                h->mc.mc_luma( h->mb.pic.p_fdec[p], FDEC_STRIDE,
                               &h->mb.pic.p_fref[0][0][p*4], h->mb.pic.i_stride[p],
                               mvx, mvy, 16, 16, &h->sh.weight[0][p] );

            if( chroma )
            {
                int v_shift = CHROMA_V_SHIFT;
                int height = 16 >> v_shift;

                /* Special case for mv0, which is (of course) very common in P-skip mode. */
                if( mvx | mvy )
                    h->mc.mc_chroma( h->mb.pic.p_fdec[1], h->mb.pic.p_fdec[2], FDEC_STRIDE,
                                     h->mb.pic.p_fref[0][0][4], h->mb.pic.i_stride[1],
                                     mvx, 2*mvy>>v_shift, 8, height );
                else
                    h->mc.load_deinterleave_chroma_fdec( h->mb.pic.p_fdec[1], h->mb.pic.p_fref[0][0][4],
                                                         h->mb.pic.i_stride[1], height );

                if( h->sh.weight[0][1].weightfn )
                    h->sh.weight[0][1].weightfn[8>>2]( h->mb.pic.p_fdec[1], FDEC_STRIDE,
                                                       h->mb.pic.p_fdec[1], FDEC_STRIDE,
                                                       &h->sh.weight[0][1], height );
                if( h->sh.weight[0][2].weightfn )
                    h->sh.weight[0][2].weightfn[8>>2]( h->mb.pic.p_fdec[2], FDEC_STRIDE,
                                                       h->mb.pic.p_fdec[2], FDEC_STRIDE,
                                                       &h->sh.weight[0][2], height );
            }
        }

        macroblock_encode_skip( h );
        return;
    }
    /*if( h->mb.i_type == B_SKIP )
    {
        /* don't do bskip motion compensation if it was already done in macroblock_analyse /
        if( !h->mb.b_skip_mc )
            x264_mb_mc( h );
        macroblock_encode_skip( h );
        return;
    }*/

    if( h->mb.i_type == I_16x16 )
    {
        h->mb.b_transform_8x8 = 0;

        for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_qp )
            mb_encode_i16x16( h, p, i_qp );
    }
    else if( h->mb.i_type == I_8x8 )
    {
        h->mb.b_transform_8x8 = 1;
        /* If we already encoded 3 of the 4 i8x8 blocks, we don't have to do them again. */
        if( h->mb.i_skip_intra )
        {
            h->mc.copy[PIXEL_16x16]( h->mb.pic.p_fdec[0], FDEC_STRIDE, h->mb.pic.i8x8_fdec_buf, 16, 16 );
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 0]] ) = h->mb.pic.i8x8_nnz_buf[0];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 2]] ) = h->mb.pic.i8x8_nnz_buf[1];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 8]] ) = h->mb.pic.i8x8_nnz_buf[2];
            M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = h->mb.pic.i8x8_nnz_buf[3];
            h->mb.i_cbp_luma = h->mb.pic.i8x8_cbp;
            /* In RD mode, restore the now-overwritten DCT data. */
            if( h->mb.i_skip_intra == 2 )
                h->mc.memcpy_aligned( h->dct.luma8x8, h->mb.pic.i8x8_dct_buf, sizeof(h->mb.pic.i8x8_dct_buf) );
        }
        for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_qp )
        {
            for( int i = (p == 0 && h->mb.i_skip_intra) ? 3 : 0; i < 4; i++ )
            {
                int i_mode = h->mb.cache.intra4x4_pred_mode[x264_scan8[4*i]];
                x264_mb_encode_i8x8( h, p, i, i_qp, i_mode, NULL, 1 );
            }
        }
    }
    else if( h->mb.i_type == I_4x4 )
    {
        h->mb.b_transform_8x8 = 0;
#if !HIGH_BIT_DEPTH
        /* Mobiclip: at the frame's left/top edge, directional intra modes read
         * neighbour positions that the decoder's pget() clamps back into the
         * current (not-yet-reconstructed) block, reading stale frame-buffer data
         * the encoder cannot reproduce -> reconstruction desync that drifts
         * through P-frames as shearing.  Force DC (always reads only the
         * available reconstructed side) for blocks touching ax==0 or ay==0. */
        if( h->param.i_mobiclip )
            for( int i = 0; i < 16; i++ )
            {
                int ax = h->mb.i_mb_x*16 + block_idx_x[i]*4;
                int ay = h->mb.i_mb_y*16 + block_idx_y[i]*4;
                if( ax == 0 || ay == 0 )
                    h->mb.cache.intra4x4_pred_mode[x264_scan8[i]] = I_PRED_4x4_DC;
            }
#endif
        /* If we already encoded 15 of the 16 i4x4 blocks, we don't have to do them again. */
        if( h->mb.i_skip_intra )
        {
            h->mc.copy[PIXEL_16x16]( h->mb.pic.p_fdec[0], FDEC_STRIDE, h->mb.pic.i4x4_fdec_buf, 16, 16 );
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 0]] ) = h->mb.pic.i4x4_nnz_buf[0];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 2]] ) = h->mb.pic.i4x4_nnz_buf[1];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 8]] ) = h->mb.pic.i4x4_nnz_buf[2];
            M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = h->mb.pic.i4x4_nnz_buf[3];
            h->mb.i_cbp_luma = h->mb.pic.i4x4_cbp;
            /* In RD mode, restore the now-overwritten DCT data. */
            if( h->mb.i_skip_intra == 2 )
                h->mc.memcpy_aligned( h->dct.luma4x4, h->mb.pic.i4x4_dct_buf, sizeof(h->mb.pic.i4x4_dct_buf) );
        }
        for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_qp )
        {
            for( int i = (p == 0 && h->mb.i_skip_intra) ? 15 : 0; i < 16; i++ )
            {
                pixel *p_dst = &h->mb.pic.p_fdec[p][block_idx_xy_fdec[i]];
                int i_mode = h->mb.cache.intra4x4_pred_mode[x264_scan8[i]];

                if( (h->mb.i_neighbour4[i] & (MB_TOPRIGHT|MB_TOP)) == MB_TOP )
                    /* emulate missing topright samples */
                    MPIXEL_X4( &p_dst[4-FDEC_STRIDE] ) = PIXEL_SPLAT_X4( p_dst[3-FDEC_STRIDE] );

                x264_mb_encode_i4x4( h, p, i, i_qp, i_mode, 1 );
            }
        }
    }
    else    /* Inter MB */
    {
        int i_decimate_mb = 0;

        /* Don't repeat motion compensation if it was already done in non-RD transform analysis */
        if( !h->mb.b_skip_mc )
            x264_mb_mc( h );

#if !HIGH_BIT_DEPTH
        /* Mobiclip P_SKIP: if this MB is essentially identical to the SAME
         * position in the reference (zero MV), force a clean zero-MV copy with
         * no residual.  x264's ME otherwise picks spurious sub-pel MVs and codes
         * source dither as residual, which re-quantizes static areas slightly
         * differently every frame -> visible temporal flicker (and bloat).
         * Forcing the zero-MV copy keeps static content byte-identical across
         * the GOP.  Threshold is the per-MB luma SAD vs the zero-MV reference
         * (default ~2/pixel, tunable via MOBI_SKIP). */
        int b_mobi_skip = 0;
        /* Never freeze/stabilize the top or bottom macroblock row.  Motion
         * compensation there reads the frame's edge-extended border, whose
         * value isn't reliable enough to copy across frames; freezing it (skip)
         * locks in a wrong prediction that then oscillates frame-to-frame,
         * showing up as jittery horizontal lines along the top/bottom edges. */
        int b_mobi_edge_row = ( h->mb.i_mb_y == 0 || h->mb.i_mb_y == h->mb.i_mb_height - 1 );
        if( h->param.i_mobiclip && !h->mb.b_lossless && h->mb.pic.i_fref[0] > 0 && !b_mobi_edge_row )
        {
            static int thr = -1;
            if( thr < 0 ) { const char *e = getenv("MOBI_SKIP"); thr = e ? atoi(e) : 512; if( thr < 0 ) thr = 0; }
            pixel *src = h->mb.pic.p_fenc[0];
            pixel *ref = h->mb.pic.p_fref[0][0][0];   /* MB origin, MV=0 */
            pixel *dec = h->mb.pic.p_fdec[0];         /* ME-chosen prediction (x264_mb_mc) */
            int sref = h->mb.pic.i_stride[0];
            int sad = 0, mc_sad = 0;
            for( int y = 0; y < 16; y++ )
                for( int x = 0; x < 16; x++ )
                {
                    sad    += abs( src[y*FENC_STRIDE + x] - ref[y*sref + x] );
                    mc_sad += abs( src[y*FENC_STRIDE + x] - dec[y*FDEC_STRIDE + x] );
                }
            /* Two independent stabilizations for static content:
             *
             * 1. MV swimming (jitter): on near-static but noisy blocks, sub-pel
             *    ME picks a slightly different MV every frame, so the prediction
             *    shifts around -> visible jitter.  Whenever motion estimation
             *    barely beat the zero-MV copy (the block isn't really moving),
             *    snap the MV to zero so the prediction is temporally stable.  We
             *    still code the residual, so this does NOT freeze the block and
             *    costs almost no quality (sad ~= mc_sad here).  Slowly-but-truly
             *    moving blocks (where ME finds a much better match) keep their
             *    real MV and are untouched.
             *
             * 2. Dither flicker: when the block is additionally very low energy
             *    (sad <= thr), also drop the residual so static areas are
             *    byte-identical across frames.
             *
             * Forcing i_type (not just i_partition) is required: leaving a
             * sub-partitioned type while claiming D_16x16 makes the entropy
             * coder emit syntax the decoder reinterprets, desyncing MV/mode
             * prediction for following MBs -> horizontal shearing. */
            int motion_didnt_help = ( sad <= mc_sad + (mc_sad >> 2) + 256 );
            if( motion_didnt_help )
            {
                if( sad <= thr )
                    b_mobi_skip = 1;
                h->mb.i_type      = P_L0;
                h->mb.i_partition = D_16x16;
                for( int i = 0; i < 4; i++ )
                    h->mb.i_sub_partition[i] = D_L0_8x8;
                for( int i = 0; i < 16; i++ )
                {
                    int i8 = x264_scan8[0] + (i&3) + (i>>2)*8;
                    h->mb.cache.mv[0][i8][0] = 0;
                    h->mb.cache.mv[0][i8][1] = 0;
                    h->mb.cache.ref[0][i8]   = 0;
                }
            }
        }
        /* Replace x264's MC prediction with Mobiclip's interpolation so the
         * residual reconstructs correctly in the Mobiclip decoder. */
    if( h->param.i_mobiclip )
            mobi_motion_compensate( h );
#endif

        if( h->mb.b_lossless )
        {
            if( h->mb.b_transform_8x8 )
                for( int p = 0; p < plane_count; p++ )
                    for( int i8x8 = 0; i8x8 < 4; i8x8++ )
                    {
                        int x = i8x8&1;
                        int y = i8x8>>1;
                        nz = h->zigzagf.sub_8x8( h->dct.luma8x8[p*4+i8x8], h->mb.pic.p_fenc[p] + 8*x + 8*y*FENC_STRIDE,
                                                                           h->mb.pic.p_fdec[p] + 8*x + 8*y*FDEC_STRIDE );
                        STORE_8x8_NNZ( p, i8x8, nz );
                        h->mb.i_cbp_luma |= nz << i8x8;
                    }
            else
                for( int p = 0; p < plane_count; p++ )
                    for( int i4x4 = 0; i4x4 < 16; i4x4++ )
                    {
                        nz = h->zigzagf.sub_4x4( h->dct.luma4x4[p*16+i4x4],
                                                 h->mb.pic.p_fenc[p]+block_idx_xy_fenc[i4x4],
                                                 h->mb.pic.p_fdec[p]+block_idx_xy_fdec[i4x4] );
                        h->mb.cache.non_zero_count[x264_scan8[p*16+i4x4]] = nz;
                        h->mb.i_cbp_luma |= nz << (i4x4>>2);
                    }
        }
#if !HIGH_BIT_DEPTH
        else if( h->param.i_mobiclip && b_mobi_skip )
        {
            /* Static MB: no luma residual — p_fdec already holds the exact
             * zero-MV reference copy from mobi_motion_compensate. */
            for( int p = 0; p < plane_count; p++ )
                CLEAR_16x16_NNZ( p );
        }
        else if( h->param.i_mobiclip )
        {
            /* Inter luma residual.  x264's standard quant_4x4x4 + dequant_4x4 +
             * add8x8_idct use the wrong quantizer, the wrong zigzag, and a
             * non-decoder-exact IDCT, so the inter residual did NOT round-trip
             * (it only looked fine on static content where the residual was 0).
             * Route inter luma through the SAME decoder-exact 4x4 path as intra
             * (mobi_quant_4x4 + mobi_zigzag + inverse4 reconstruction); b_predict
             * =0 keeps the motion-compensated prediction already in p_fdec. */
            for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_qp )
            {
                CLEAR_16x16_NNZ( p );
                /* Save the motion-compensated prediction so we can restore it if
                 * the whole-MB residual is decimated away (coded as no-residual).
                 * x264's normal inter path drops near-zero residual to skip; we
                 * must do the same or every tiny coefficient gets coded (huge
                 * files).  The decoder is agnostic to the decision — it just
                 * decodes whatever coefficient pattern we signal — so the
                 * heuristic doesn't affect bit-exactness. */
                ALIGNED_ARRAY_16( pixel, save,[256] );
                if( b_decimate )
                    for( int y = 0; y < 16; y++ )
                        memcpy( save + y*16, h->mb.pic.p_fdec[p] + y*FDEC_STRIDE, 16*sizeof(pixel) );
                int i_decimate_p = 0;
                for( int i4x4 = 0; i4x4 < 16; i4x4++ )
                {
                    x264_mb_encode_i4x4( h, p, i4x4, i_qp, 0, 0 );
                    if( h->mb.cache.non_zero_count[x264_scan8[p*16+i4x4]] && b_decimate )
                        i_decimate_p += h->quantf.decimate_score16( h->dct.luma4x4[p*16+i4x4] );
                }
                if( b_decimate && i_decimate_p < 6 )
                {
                    /* Drop the residual: restore pure MC prediction, clear nnz. */
                    CLEAR_16x16_NNZ( p );
                    for( int y = 0; y < 16; y++ )
                        memcpy( h->mb.pic.p_fdec[p] + y*FDEC_STRIDE, save + y*16, 16*sizeof(pixel) );
                }
                else
                    for( int i4x4 = 0; i4x4 < 16; i4x4++ )
                        if( h->mb.cache.non_zero_count[x264_scan8[p*16+i4x4]] )
                            h->mb.i_cbp_luma |= 1 << (i4x4>>2);
            }
        }
#endif
        else if( h->mb.b_transform_8x8 )
        {
            ALIGNED_ARRAY_64( dctcoef, dct8x8,[4],[64] );
            b_decimate &= !h->mb.b_trellis || !h->param.b_cabac; // 8x8 trellis is inherently optimal decimation for CABAC

            for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_qp )
            {
				int quant_cat = CQM_8IY;//p ? CQM_8PC : CQM_8PY;
                CLEAR_16x16_NNZ( p );
                h->dctf.sub16x16_dct8( dct8x8, h->mb.pic.p_fenc[p], h->mb.pic.p_fdec[p] );
                h->nr_count[1+!!p*2] += h->mb.b_noise_reduction * 4;

                int plane_cbp = 0;
                for( int idx = 0; idx < 4; idx++ )
                {
                    nz = x264_quant_8x8( h, dct8x8[idx], i_qp, ctx_cat_plane[DCT_LUMA_8x8][p], 1, p, idx );

                    if( nz )
                    {
						for (int i = 0; i < 64; i++)
							h->dct.luma8x8[p * 4 + idx][i] = dct8x8[idx][ZigZagTable8x8[i]];
                        //h->zigzagf.scan_8x8( h->dct.luma8x8[p*4+idx], dct8x8[idx] );
                        if( b_decimate )
                        {
                            int i_decimate_8x8 = h->quantf.decimate_score64( h->dct.luma8x8[p*4+idx] );
                            i_decimate_mb += i_decimate_8x8;
                            if( i_decimate_8x8 >= 4 )
                                plane_cbp |= 1<<idx;
                        }
                        else
                            plane_cbp |= 1<<idx;
                    }
                }

                if( i_decimate_mb >= 6 || !b_decimate )
                {
                    h->mb.i_cbp_luma |= plane_cbp;
                    FOREACH_BIT( idx, 0, plane_cbp )
                    {
                        h->quantf.dequant_8x8( dct8x8[idx], h->dequant8_mf[quant_cat], i_qp );
                        h->dctf.add8x8_idct8( &h->mb.pic.p_fdec[p][8*(idx&1) + 8*(idx>>1)*FDEC_STRIDE], dct8x8[idx] );
                        STORE_8x8_NNZ( p, idx, 1 );
                    }
                }
            }
        }
        else
        {
            ALIGNED_ARRAY_64( dctcoef, dct4x4,[16],[16] );
            for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_qp )
            {
				int quant_cat = CQM_4IY;//p ? CQM_4PC : CQM_4PY;
                CLEAR_16x16_NNZ( p );
                h->dctf.sub16x16_dct( dct4x4, h->mb.pic.p_fenc[p], h->mb.pic.p_fdec[p] );

                if( h->mb.b_noise_reduction )
                {
                    h->nr_count[0+!!p*2] += 16;
                    for( int idx = 0; idx < 16; idx++ )
                        h->quantf.denoise_dct( dct4x4[idx], h->nr_residual_sum[0+!!p*2], h->nr_offset[0+!!p*2], 16 );
                }

                int plane_cbp = 0;
                for( int i8x8 = 0; i8x8 < 4; i8x8++ )
                {
                    int i_decimate_8x8 = b_decimate ? 0 : 6;
                    int nnz8x8 = 0;
                    if( h->mb.b_trellis && !h->param.i_mobiclip )
                    {
                        for( int i4x4 = 0; i4x4 < 4; i4x4++ )
                        {
                            int idx = i8x8*4+i4x4;
                            if( x264_quant_4x4_trellis( h, dct4x4[idx], quant_cat, i_qp, ctx_cat_plane[DCT_LUMA_4x4][p], 1, !!p, p*16+idx ) )
                            {
                                h->zigzagf.scan_4x4( h->dct.luma4x4[p*16+idx], dct4x4[idx] );
                                h->quantf.dequant_4x4( dct4x4[idx], h->dequant4_mf[quant_cat], i_qp );
                                if( i_decimate_8x8 < 6 )
                                    i_decimate_8x8 += h->quantf.decimate_score16( h->dct.luma4x4[p*16+idx] );
                                h->mb.cache.non_zero_count[x264_scan8[p*16+idx]] = 1;
                                nnz8x8 = 1;
                            }
                        }
                    }
                    else
                    {
                        nnz8x8 = nz = h->quantf.quant_4x4x4( &dct4x4[i8x8*4], h->quant4_mf[quant_cat][i_qp], h->quant4_bias[quant_cat][i_qp] );
                        if( nz )
                        {
                            FOREACH_BIT( idx, i8x8*4, nz )
                            {
                                h->zigzagf.scan_4x4( h->dct.luma4x4[p*16+idx], dct4x4[idx] );
                                h->quantf.dequant_4x4( dct4x4[idx], h->dequant4_mf[quant_cat], i_qp );
                                if( i_decimate_8x8 < 6 )
                                    i_decimate_8x8 += h->quantf.decimate_score16( h->dct.luma4x4[p*16+idx] );
                                h->mb.cache.non_zero_count[x264_scan8[p*16+idx]] = 1;
                            }
                        }
                    }
                    if( nnz8x8 )
                    {
                        i_decimate_mb += i_decimate_8x8;
                        if( i_decimate_8x8 < 4 )
                            STORE_8x8_NNZ( p, i8x8, 0 );
                        else
                            plane_cbp |= 1<<i8x8;
                    }
                }

                if( i_decimate_mb < 6 )
                {
                    plane_cbp = 0;
                    CLEAR_16x16_NNZ( p );
                }
                else
                {
                    h->mb.i_cbp_luma |= plane_cbp;
                    FOREACH_BIT( i8x8, 0, plane_cbp )
                    {
                        h->dctf.add8x8_idct( &h->mb.pic.p_fdec[p][(i8x8&1)*8 + (i8x8>>1)*8*FDEC_STRIDE], &dct4x4[i8x8*4] );
                    }
                }
            }
        }
    }

    /* encode chroma */
    if( chroma )
    {
        if( IS_INTRA( h->mb.i_type ) )
        {
            int i_mode = h->mb.i_chroma_pred_mode;
            if( h->mb.b_lossless )
                x264_predict_lossless_chroma( h, i_mode );
#if !HIGH_BIT_DEPTH
            else if( h->param.i_mobiclip )
            {
                /* Predict chroma with the Mobiclip predictor for the exact mode
                 * that gets written (mobi_chroma_pred_mode_fix -> 0/1/3, never
                 * plane mode 2) so the reconstruction matches the decoder's
                 * predict_intra.  Must use the SAME table as cavlc.c. */
                static const uint8_t mobi_chroma_pred_mode_fix[7] = { 0, 1, 3, 3, 3, 3, 3 };
                int mobi_cmode = mobi_chroma_pred_mode_fix[h->mb.i_chroma_pred_mode];
                mobi_predict_chroma_8x8( h, 0, mobi_cmode );
                mobi_predict_chroma_8x8( h, 1, mobi_cmode );
            }
#endif
            else
            {
                h->predict_chroma[i_mode]( h->mb.pic.p_fdec[1] );
                h->predict_chroma[i_mode]( h->mb.pic.p_fdec[2] );
            }
        }

        /* encode the 8x8 blocks */
        x264_mb_encode_chroma( h, !IS_INTRA( h->mb.i_type ), /*h->mb.i_chroma_qp*/h->mb.i_qp);
    }
    else
        h->mb.i_cbp_chroma = 0;

    /* store cbp */
    int cbp = h->mb.i_cbp_chroma << 4 | h->mb.i_cbp_luma;
    if( h->param.b_cabac )
        cbp |= h->mb.cache.non_zero_count[x264_scan8[LUMA_DC    ]] << 8
            |  h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+0]] << 9
            |  h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+1]] << 10;
    h->mb.cbp[h->mb.i_mb_xy] = cbp;

    /* Check for P_SKIP
     * XXX: in the me perhaps we should take x264_mb_predict_mv_pskip into account
     *      (if multiple mv give same result)*/
    if( !b_force_no_skip )
    {
        if( h->mb.i_type == P_L0 && h->mb.i_partition == D_16x16 &&
            !(h->mb.i_cbp_luma | h->mb.i_cbp_chroma) &&
            M32( h->mb.cache.mv[0][x264_scan8[0]] ) == M32( h->mb.cache.pskip_mv )
            && h->mb.cache.ref[0][x264_scan8[0]] == 0 )
        {
            h->mb.i_type = P_SKIP;
        }

        /* Check for B_SKIP */
        if( h->mb.i_type == B_DIRECT && !(h->mb.i_cbp_luma | h->mb.i_cbp_chroma) )
        {
            h->mb.i_type = B_SKIP;
        }
    }
}

void x264_macroblock_encode( x264_t *h )
{
    if( CHROMA444 )
        macroblock_encode_internal( h, 3, 0 );
    else if( CHROMA_FORMAT )
        macroblock_encode_internal( h, 1, 1 );
    else
        macroblock_encode_internal( h, 1, 0 );

    if( getenv("MOBI_DBG") && h->mb.i_mb_x == 4 && h->mb.i_mb_y == 4 )
    {
        long se = 0, sd = 0;
        for( int y = 0; y < 16; y++ )
            for( int x = 0; x < 16; x++ )
            {
                se += h->mb.pic.p_fenc[0][y*FENC_STRIDE + x];
                sd += h->mb.pic.p_fdec[0][y*FDEC_STRIDE + x];
            }
        fprintf( stderr, "[ENCREC] MB(4,4) type=%d fenc=%ld fdec=%ld m=%d,%d,%d,%d\n",
                 h->mb.i_type, se/256, sd/256,
                 x264_mb_pred_mode4x4_fix(h->mb.cache.intra4x4_pred_mode[x264_scan8[0]]),
                 x264_mb_pred_mode4x4_fix(h->mb.cache.intra4x4_pred_mode[x264_scan8[1]]),
                 x264_mb_pred_mode4x4_fix(h->mb.cache.intra4x4_pred_mode[x264_scan8[2]]),
                 x264_mb_pred_mode4x4_fix(h->mb.cache.intra4x4_pred_mode[x264_scan8[3]]) );
    }

}

/*****************************************************************************
 * x264_macroblock_probe_skip:
 *  Check if the current MB could be encoded as a [PB]_SKIP
 *****************************************************************************/
static ALWAYS_INLINE int macroblock_probe_skip_internal( x264_t *h, int b_bidir, int plane_count, int chroma )
{
	return 0;
    ALIGNED_ARRAY_64( dctcoef, dct4x4,[8],[16] );
    ALIGNED_ARRAY_64( dctcoef, dctscan,[16] );
    ALIGNED_4( int16_t mvp[2] );
    int i_qp = h->mb.i_qp;

    for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_chroma_qp )
    {
		int quant_cat = CQM_4IY;// p ? CQM_4PC : CQM_4PY;
        if( !b_bidir )
        {
            /* Get the MV */
            mvp[0] = x264_clip3( h->mb.cache.pskip_mv[0], h->mb.mv_min[0], h->mb.mv_max[0] );
            mvp[1] = x264_clip3( h->mb.cache.pskip_mv[1], h->mb.mv_min[1], h->mb.mv_max[1] );

            /* Motion compensation */
            h->mc.mc_luma( h->mb.pic.p_fdec[p],    FDEC_STRIDE,
                           &h->mb.pic.p_fref[0][0][p*4], h->mb.pic.i_stride[p],
                           mvp[0], mvp[1], 16, 16, &h->sh.weight[0][p] );
        }

        for( int i8x8 = 0, i_decimate_mb = 0; i8x8 < 4; i8x8++ )
        {
            int fenc_offset = (i8x8&1) * 8 + (i8x8>>1) * FENC_STRIDE * 8;
            int fdec_offset = (i8x8&1) * 8 + (i8x8>>1) * FDEC_STRIDE * 8;

            h->dctf.sub8x8_dct( dct4x4, h->mb.pic.p_fenc[p] + fenc_offset,
                                        h->mb.pic.p_fdec[p] + fdec_offset );

            if( h->mb.b_noise_reduction )
                for( int i4x4 = 0; i4x4 < 4; i4x4++ )
                    h->quantf.denoise_dct( dct4x4[i4x4], h->nr_residual_sum[0+!!p*2], h->nr_offset[0+!!p*2], 16 );

            int nz = h->quantf.quant_4x4x4( dct4x4, h->quant4_mf[quant_cat][i_qp], h->quant4_bias[quant_cat][i_qp] );
            FOREACH_BIT( idx, 0, nz )
            {
                h->zigzagf.scan_4x4( dctscan, dct4x4[idx] );
                i_decimate_mb += h->quantf.decimate_score16( dctscan );
                if( i_decimate_mb >= 6 )
                    return 0;
            }
        }
    }

    if( chroma == CHROMA_420 || chroma == CHROMA_422 )
    {
        i_qp = h->mb.i_qp;
        int chroma422 = chroma == CHROMA_422;
        int thresh = chroma422 ? (x264_lambda2_tab[i_qp] + 16) >> 5 : (x264_lambda2_tab[i_qp] + 32) >> 6;
        int ssd;
        ALIGNED_ARRAY_16( dctcoef, dct_dc,[8] );

        if( !b_bidir )
        {
            /* Special case for mv0, which is (of course) very common in P-skip mode. */
            if( M32( mvp ) )
                h->mc.mc_chroma( h->mb.pic.p_fdec[1], h->mb.pic.p_fdec[2], FDEC_STRIDE,
                                 h->mb.pic.p_fref[0][0][4], h->mb.pic.i_stride[1],
                                 mvp[0], mvp[1] * (1<<chroma422), 8, chroma422?16:8 );
            else
                h->mc.load_deinterleave_chroma_fdec( h->mb.pic.p_fdec[1], h->mb.pic.p_fref[0][0][4],
                                                     h->mb.pic.i_stride[1], chroma422?16:8 );
        }

        for( int ch = 0; ch < 2; ch++ )
        {
            pixel *p_src = h->mb.pic.p_fenc[1+ch];
            pixel *p_dst = h->mb.pic.p_fdec[1+ch];

            if( !b_bidir && h->sh.weight[0][1+ch].weightfn )
                h->sh.weight[0][1+ch].weightfn[8>>2]( h->mb.pic.p_fdec[1+ch], FDEC_STRIDE,
                                                      h->mb.pic.p_fdec[1+ch], FDEC_STRIDE,
                                                      &h->sh.weight[0][1+ch], chroma422?16:8 );

            /* there is almost never a termination during chroma, but we can't avoid the check entirely */
            /* so instead we check SSD and skip the actual check if the score is low enough. */
            ssd = h->pixf.ssd[chroma422?PIXEL_8x16:PIXEL_8x8]( p_dst, FDEC_STRIDE, p_src, FENC_STRIDE );
            if( ssd < thresh )
                continue;

            /* The vast majority of chroma checks will terminate during the DC check or the higher
             * threshold check, so we can save time by doing a DC-only DCT. */
            if( h->mb.b_noise_reduction )
            {
                for( int i = 0; i <= chroma422; i++ )
                    h->dctf.sub8x8_dct( &dct4x4[4*i], p_src + 8*i*FENC_STRIDE, p_dst + 8*i*FDEC_STRIDE );

                for( int i4x4 = 0; i4x4 < (chroma422?8:4); i4x4++ )
                {
                    h->quantf.denoise_dct( dct4x4[i4x4], h->nr_residual_sum[2], h->nr_offset[2], 16 );
                    dct_dc[i4x4] = dct4x4[i4x4][0];
                    dct4x4[i4x4][0] = 0;
                }
            }
            else
            {
                if( chroma422 )
                    h->dctf.sub8x16_dct_dc( dct_dc, p_src, p_dst );
                else
                    h->dctf.sub8x8_dct_dc( dct_dc, p_src, p_dst );
            }

            for( int i = 0; i <= chroma422; i++ )
                if( h->quantf.quant_2x2_dc( &dct_dc[4*i], h->quant4_mf[CQM_4PC][i_qp+3*chroma422][0] >> 1,
                                            h->quant4_bias[CQM_4PC][i_qp+3*chroma422][0] << 1 ) )
                    return 0;

            /* If there wasn't a termination in DC, we can check against a much higher threshold. */
            if( ssd < thresh*4 )
                continue;

            if( !h->mb.b_noise_reduction )
                for( int i = 0; i <= chroma422; i++ )
                {
                    h->dctf.sub8x8_dct( &dct4x4[4*i], p_src + 8*i*FENC_STRIDE, p_dst + 8*i*FDEC_STRIDE );
                    dct4x4[i*4+0][0] = 0;
                    dct4x4[i*4+1][0] = 0;
                    dct4x4[i*4+2][0] = 0;
                    dct4x4[i*4+3][0] = 0;
                }

            /* calculate dct coeffs */
            for( int i8x8 = 0, i_decimate_mb = 0; i8x8 < (chroma422?2:1); i8x8++ )
            {
                int nz = h->quantf.quant_4x4x4( &dct4x4[i8x8*4], h->quant4_mf[CQM_4PC][i_qp], h->quant4_bias[CQM_4PC][i_qp] );
                FOREACH_BIT( idx, i8x8*4, nz )
                {
                    h->zigzagf.scan_4x4( dctscan, dct4x4[idx] );
                    i_decimate_mb += h->quantf.decimate_score15( dctscan );
                    if( i_decimate_mb >= 7 )
                        return 0;
                }
            }
        }
    }

    h->mb.b_skip_mc = 1;
    return 1;
}

int x264_macroblock_probe_skip( x264_t *h, int b_bidir )
{
    if( CHROMA_FORMAT == CHROMA_420 )
        return macroblock_probe_skip_internal( h, b_bidir, 1, CHROMA_420 );
    else if( CHROMA_FORMAT == CHROMA_422 )
        return macroblock_probe_skip_internal( h, b_bidir, 1, CHROMA_422 );
    else if( CHROMA_FORMAT == CHROMA_444 )
        return macroblock_probe_skip_internal( h, b_bidir, 3, CHROMA_444 );
    else
        return macroblock_probe_skip_internal( h, b_bidir, 1, CHROMA_400 );
}

/****************************************************************************
 * DCT-domain noise reduction / adaptive deadzone
 * from libavcodec
 ****************************************************************************/

void x264_noise_reduction_update( x264_t *h )
{
    h->nr_offset = h->nr_offset_denoise;
    h->nr_residual_sum = h->nr_residual_sum_buf[0];
    h->nr_count = h->nr_count_buf[0];
    for( int cat = 0; cat < 3 + CHROMA444; cat++ )
    {
        int dct8x8 = cat&1;
        int size = dct8x8 ? 64 : 16;
        const uint32_t *weight = dct8x8 ? x264_dct8_weight2_tab : x264_dct4_weight2_tab;

        if( h->nr_count[cat] > (dct8x8 ? (1<<16) : (1<<18)) )
        {
            for( int i = 0; i < size; i++ )
                h->nr_residual_sum[cat][i] >>= 1;
            h->nr_count[cat] >>= 1;
        }

        for( int i = 0; i < size; i++ )
            h->nr_offset[cat][i] =
                ((uint64_t)h->param.analyse.i_noise_reduction * h->nr_count[cat]
                 + h->nr_residual_sum[cat][i]/2)
              / ((uint64_t)h->nr_residual_sum[cat][i] * weight[i]/256 + 1);

        /* Don't denoise DC coefficients */
        h->nr_offset[cat][0] = 0;
    }
}

/*****************************************************************************
 * RD only; 4 calls to this do not make up for one macroblock_encode.
 * doesn't transform chroma dc.
 *****************************************************************************/
static ALWAYS_INLINE void macroblock_encode_p8x8_internal( x264_t *h, int i8, int plane_count, int chroma )
{
    int b_decimate = h->mb.b_dct_decimate;
    int i_qp = h->mb.i_qp;
    int x = i8&1;
    int y = i8>>1;
    int nz;
    int chroma422 = chroma == CHROMA_422;

    h->mb.i_cbp_chroma = 0;
    h->mb.i_cbp_luma &= ~(1 << i8);

    if( !h->mb.b_skip_mc )
        x264_mb_mc_8x8( h, i8 );

    if( h->mb.b_lossless )
    {
        for( int p = 0; p < plane_count; p++ )
        {
            pixel *p_fenc = h->mb.pic.p_fenc[p] + 8*x + 8*y*FENC_STRIDE;
            pixel *p_fdec = h->mb.pic.p_fdec[p] + 8*x + 8*y*FDEC_STRIDE;
            int nnz8x8 = 0;
            if( h->mb.b_transform_8x8 )
            {
                nnz8x8 = h->zigzagf.sub_8x8( h->dct.luma8x8[4*p+i8], p_fenc, p_fdec );
                STORE_8x8_NNZ( p, i8, nnz8x8 );
            }
            else
            {
                for( int i4 = i8*4; i4 < i8*4+4; i4++ )
                {
                    nz = h->zigzagf.sub_4x4( h->dct.luma4x4[16*p+i4],
                                             h->mb.pic.p_fenc[p]+block_idx_xy_fenc[i4],
                                             h->mb.pic.p_fdec[p]+block_idx_xy_fdec[i4] );
                    h->mb.cache.non_zero_count[x264_scan8[16*p+i4]] = nz;
                    nnz8x8 |= nz;
                }
            }
            h->mb.i_cbp_luma |= nnz8x8 << i8;
        }
        if( chroma == CHROMA_420 || chroma == CHROMA_422 )
        {
            for( int ch = 0; ch < 2; ch++ )
            {
                dctcoef dc;
                pixel *p_fenc = h->mb.pic.p_fenc[1+ch] + 4*x + (chroma422?8:4)*y*FENC_STRIDE;
                pixel *p_fdec = h->mb.pic.p_fdec[1+ch] + 4*x + (chroma422?8:4)*y*FDEC_STRIDE;

                for( int i4x4 = 0; i4x4 <= chroma422; i4x4++ )
                {
                    int offset = chroma422 ? 8*y + 2*i4x4 + x : i8;
                    nz = h->zigzagf.sub_4x4ac( h->dct.luma4x4[16+offset+ch*16], p_fenc+4*i4x4*FENC_STRIDE, p_fdec+4*i4x4*FDEC_STRIDE, &dc );
                    h->mb.cache.non_zero_count[x264_scan8[16+offset+ch*16]] = nz;
                }
            }
            h->mb.i_cbp_chroma = 0x02;
        }
    }
    else
    {
        if( h->mb.b_transform_8x8 )
        {
            for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_chroma_qp )
            {
				int quant_cat = CQM_8IY;// p ? CQM_8PC : CQM_8PY;
                pixel *p_fenc = h->mb.pic.p_fenc[p] + 8*x + 8*y*FENC_STRIDE;
                pixel *p_fdec = h->mb.pic.p_fdec[p] + 8*x + 8*y*FDEC_STRIDE;
                ALIGNED_ARRAY_64( dctcoef, dct8x8,[64] );

                h->dctf.sub8x8_dct8( dct8x8, p_fenc, p_fdec );
                int nnz8x8 = x264_quant_8x8( h, dct8x8, i_qp, ctx_cat_plane[DCT_LUMA_8x8][p], 1, p, i8 );
                if( nnz8x8 )
                {
					for (int i = 0; i < 64; i++)
						h->dct.luma8x8[p * 4 + i8][i] = dct8x8[ZigZagTable8x8[i]];
                    //h->zigzagf.scan_8x8( h->dct.luma8x8[4*p+i8], dct8x8 );

                    if( b_decimate && !h->mb.b_trellis )
                        nnz8x8 = 4 <= h->quantf.decimate_score64( h->dct.luma8x8[4*p+i8] );

                    if( nnz8x8 )
                    {
                        h->quantf.dequant_8x8( dct8x8, h->dequant8_mf[quant_cat], i_qp );
                        h->dctf.add8x8_idct8( p_fdec, dct8x8 );
                        STORE_8x8_NNZ( p, i8, 1 );
                        h->mb.i_cbp_luma |= 1 << i8;
                    }
                    else
                        STORE_8x8_NNZ( p, i8, 0 );
                }
                else
                    STORE_8x8_NNZ( p, i8, 0 );
            }
        }
        else
        {
            for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_chroma_qp )
            {
				int quant_cat = CQM_4IY;//p ? CQM_4PC : CQM_4PY;
                pixel *p_fenc = h->mb.pic.p_fenc[p] + 8*x + 8*y*FENC_STRIDE;
                pixel *p_fdec = h->mb.pic.p_fdec[p] + 8*x + 8*y*FDEC_STRIDE;
                int i_decimate_8x8 = b_decimate ? 0 : 4;
                ALIGNED_ARRAY_64( dctcoef, dct4x4,[4],[16] );
                int nnz8x8 = 0;

                h->dctf.sub8x8_dct( dct4x4, p_fenc, p_fdec );
                STORE_8x8_NNZ( p, i8, 0 );

                if( h->mb.b_noise_reduction )
                    for( int idx = 0; idx < 4; idx++ )
                        h->quantf.denoise_dct( dct4x4[idx], h->nr_residual_sum[0+!!p*2], h->nr_offset[0+!!p*2], 16 );

                if( h->mb.b_trellis && !h->param.i_mobiclip )
                {
                    for( int i4x4 = 0; i4x4 < 4; i4x4++ )
                    {
                        if( x264_quant_4x4_trellis( h, dct4x4[i4x4], quant_cat, i_qp, ctx_cat_plane[DCT_LUMA_4x4][p], 0, !!p, i8*4+i4x4+p*16 ) )
                        {
                            h->zigzagf.scan_4x4( h->dct.luma4x4[p*16+i8*4+i4x4], dct4x4[i4x4] );
                            h->quantf.dequant_4x4( dct4x4[i4x4], h->dequant4_mf[quant_cat], i_qp );
                            if( i_decimate_8x8 < 4 )
                                i_decimate_8x8 += h->quantf.decimate_score16( h->dct.luma4x4[p*16+i8*4+i4x4] );
                            h->mb.cache.non_zero_count[x264_scan8[p*16+i8*4+i4x4]] = 1;
                            nnz8x8 = 1;
                        }
                    }
                }
                else
                {
                    nnz8x8 = nz = h->quantf.quant_4x4x4( dct4x4, h->quant4_mf[quant_cat][i_qp], h->quant4_bias[quant_cat][i_qp] );
                    if( nz )
                    {
                        FOREACH_BIT( i4x4, 0, nz )
                        {
                            h->zigzagf.scan_4x4( h->dct.luma4x4[p*16+i8*4+i4x4], dct4x4[i4x4] );
                            h->quantf.dequant_4x4( dct4x4[i4x4], h->dequant4_mf[quant_cat], i_qp );
                            if( i_decimate_8x8 < 4 )
                                i_decimate_8x8 += h->quantf.decimate_score16( h->dct.luma4x4[p*16+i8*4+i4x4] );
                            h->mb.cache.non_zero_count[x264_scan8[p*16+i8*4+i4x4]] = 1;
                        }
                    }
                }
                if( nnz8x8 )
                {
                    /* decimate this 8x8 block */
                    if( i_decimate_8x8 < 4 )
                        STORE_8x8_NNZ( p, i8, 0 );
                    else
                    {
                        h->dctf.add8x8_idct( p_fdec, dct4x4 );
                        h->mb.i_cbp_luma |= 1 << i8;
                    }
                }
            }
        }

        if( chroma == CHROMA_420 || chroma == CHROMA_422 )
        {
            i_qp = h->mb.i_qp;
            for( int ch = 0; ch < 2; ch++ )
            {
                ALIGNED_ARRAY_64( dctcoef, dct4x4,[2],[16] );
                pixel *p_fenc = h->mb.pic.p_fenc[1+ch] + 4*x + (chroma422?8:4)*y*FENC_STRIDE;
                pixel *p_fdec = h->mb.pic.p_fdec[1+ch] + 4*x + (chroma422?8:4)*y*FDEC_STRIDE;

                for( int i4x4 = 0; i4x4 <= chroma422; i4x4++ )
                {
                    h->dctf.sub4x4_dct( dct4x4[i4x4], p_fenc + 4*i4x4*FENC_STRIDE, p_fdec + 4*i4x4*FDEC_STRIDE );

                    if( h->mb.b_noise_reduction )
                        h->quantf.denoise_dct( dct4x4[i4x4], h->nr_residual_sum[2], h->nr_offset[2], 16 );
                    dct4x4[i4x4][0] = 0;

                    if( h->mb.b_trellis && !h->param.i_mobiclip )
                        nz = x264_quant_4x4_trellis( h, dct4x4[i4x4], CQM_4PC, i_qp, DCT_CHROMA_AC, 0, 1, 0 );
                    else
                        nz = h->quantf.quant_4x4( dct4x4[i4x4], h->quant4_mf[CQM_4PC][i_qp], h->quant4_bias[CQM_4PC][i_qp] );

                    int offset = chroma422 ? ((5*i8) & 0x09) + 2*i4x4 : i8;
                    h->mb.cache.non_zero_count[x264_scan8[16+offset+ch*16]] = nz;
                    if( nz )
                    {
                        h->zigzagf.scan_4x4( h->dct.luma4x4[16+offset+ch*16], dct4x4[i4x4] );
                        h->quantf.dequant_4x4( dct4x4[i4x4], h->dequant4_mf[CQM_4PC], i_qp );
                        h->dctf.add4x4_idct( p_fdec + 4*i4x4*FDEC_STRIDE, dct4x4[i4x4] );
                    }
                }
            }
            h->mb.i_cbp_chroma = 0x02;
        }
    }
}

void x264_macroblock_encode_p8x8( x264_t *h, int i8 )
{
    if( CHROMA_FORMAT == CHROMA_420 )
        macroblock_encode_p8x8_internal( h, i8, 1, CHROMA_420 );
    else if( CHROMA_FORMAT == CHROMA_422 )
        macroblock_encode_p8x8_internal( h, i8, 1, CHROMA_422 );
    else if( CHROMA_FORMAT == CHROMA_444 )
        macroblock_encode_p8x8_internal( h, i8, 3, CHROMA_444 );
    else
        macroblock_encode_p8x8_internal( h, i8, 1, CHROMA_400 );
}

/*****************************************************************************
 * RD only, luma only (for 4:2:0)
 *****************************************************************************/
static ALWAYS_INLINE void macroblock_encode_p4x4_internal( x264_t *h, int i4, int plane_count )
{
    int i_qp = h->mb.i_qp;

    for( int p = 0; p < plane_count; p++, i_qp = h->mb.i_chroma_qp )
    {
        int quant_cat = CQM_4IY;
        pixel *p_fenc = &h->mb.pic.p_fenc[p][block_idx_xy_fenc[i4]];
        pixel *p_fdec = &h->mb.pic.p_fdec[p][block_idx_xy_fdec[i4]];
        int nz;

        /* Don't need motion compensation as this function is only used in qpel-RD, which caches pixel data. */

        if( h->mb.b_lossless )
        {
            nz = h->zigzagf.sub_4x4( h->dct.luma4x4[p*16+i4], p_fenc, p_fdec );
            h->mb.cache.non_zero_count[x264_scan8[p*16+i4]] = nz;
        }
        else
        {
            ALIGNED_ARRAY_64( dctcoef, dct4x4,[16] );
            h->dctf.sub4x4_dct( dct4x4, p_fenc, p_fdec );
            nz = x264_quant_4x4( h, dct4x4, i_qp, ctx_cat_plane[DCT_LUMA_4x4][p], 1, p, i4 );
            h->mb.cache.non_zero_count[x264_scan8[p*16+i4]] = nz;
            if( nz )
            {
                h->zigzagf.scan_4x4( h->dct.luma4x4[p*16+i4], dct4x4 );
                h->quantf.dequant_4x4( dct4x4, h->dequant4_mf[quant_cat], i_qp );
                h->dctf.add4x4_idct( p_fdec, dct4x4 );
            }
        }
    }
}

void x264_macroblock_encode_p4x4( x264_t *h, int i8 )
{
    if( CHROMA444 )
        macroblock_encode_p4x4_internal( h, i8, 3 );
    else
        macroblock_encode_p4x4_internal( h, i8, 1 );
}
