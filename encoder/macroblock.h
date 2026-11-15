/*****************************************************************************
 * macroblock.h: macroblock encoding
 *****************************************************************************
 * Copyright (C) 2003-2024 x264 project
 *
 * Authors: Loren Merritt <lorenm@u.washington.edu>
 *          Laurent Aimar <fenrir@via.ecp.fr>
 *          quatric <quatricsoftware@gmail.com>
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
#include "mobi_neon.h"

/* ---------------------------------------------------------------------------
 * Mobiclip transform / quantization.
 *
 * Ported from Helwettpackardenterprise's Codex-assisted translation of the
 * retail Mobiclip VfW encoder (MODS_Encoder_v43_2), which establishes that
 * Mobiclip's transform and quantizer are plain H.264:
 *
 *   forward   level = sign(c) * ((|c| * MF[q%6][pos] + f) >> shift)
 *             shift = 15 + q/6 (4x4) or 16 + q/6 (8x8),  f = (1 << shift) / 3
 *   inverse   coef  = level * (DQ[q%6][pos] << (q/6))       (4x4)
 *             coef  = level * (DQ[q%6][pos] << (q/6 - 2))   (8x8)
 *
 * MF/DQ are the standard H.264 scale matrices; the single 1/3 rounding bias is
 * used for both intra and inter (the reference encoder stores exactly one
 * offset in its quant struct).  q is the quantizer carried in the bitstream,
 * not x264's internal QP -- see mobi_qp().
 * ------------------------------------------------------------------------- */
static const uint8_t mobi_zigzag4x4[16] =
    { 0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15 };
static const uint8_t mobi_zigzag8x8[64] =
    { 0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
     12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
     35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
     58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63 };

/* Position -> scale-table column, raster order. */
static const uint8_t mobi_scale_idx4[16] =
    { 0,2,0,2,  2,1,2,1,  0,2,0,2,  2,1,2,1 };
static const uint8_t mobi_scale_idx8[64] = {
    0,3,4,3,0,3,4,3,  3,1,5,1,3,1,5,1,  4,5,2,5,4,5,2,5,  3,1,5,1,3,1,5,1,
    0,3,4,3,0,3,4,3,  3,1,5,1,3,1,5,1,  4,5,2,5,4,5,2,5,  3,1,5,1,3,1,5,1 };

static const uint16_t mobi_quant4_scale[6][3] = {
    { 13107, 5243, 8066 }, { 11916, 4660, 7490 }, { 10082, 4194, 6554 },
    {  9362, 3647, 5825 }, {  8192, 3355, 5243 }, {  7282, 2893, 4559 },
};
static const uint8_t mobi_dequant4_scale[6][3] = {
    { 10, 16, 13 }, { 11, 18, 14 }, { 13, 20, 16 },
    { 14, 23, 18 }, { 16, 25, 20 }, { 18, 29, 23 },
};
static const uint16_t mobi_quant8_scale[6][6] = {
    { 13107, 11428, 20972, 12222, 16777, 15481 },
    { 11916, 10826, 19174, 11058, 14980, 14290 },
    { 10082,  8943, 15978,  9675, 12710, 11985 },
    {  9362,  8228, 14913,  8931, 11984, 11259 },
    {  8192,  7346, 13159,  7740, 10486,  9777 },
    {  7282,  6428, 11570,  6830,  9118,  8640 },
};
static const uint8_t mobi_dequant8_scale[6][6] = {
    { 20, 18, 32, 19, 25, 24 }, { 22, 19, 35, 21, 28, 26 },
    { 26, 23, 42, 24, 33, 31 }, { 28, 25, 45, 26, 35, 33 },
    { 32, 28, 51, 30, 40, 38 }, { 36, 32, 58, 34, 46, 43 },
};

/* The quantizer actually carried in the bitstream.  The I-frame header field is
 * 6 bits and the decoder rejects anything below 12 (setup_qtables), so the
 * usable range is [12,63]; it is an H.264 QP, so x264's own QP maps straight
 * through.  i_mobi_qyx survives as an additive coarseness offset in whole
 * qy tiers (it used to be the *only* way to change quantization, because the
 * header wrote (qp%6)+12 and threw away qp/6). */
static ALWAYS_INLINE int mobi_qp( x264_t *h, int i_qp )
{
    int extra = h->param.i_mobi_qyx;
    if( extra < 0 )
        extra = x264_mobi_env.i_qyx;
    return x264_clip3( i_qp + 6 * extra, 12, 63 );
}

/* The scale tables above laid out flat for one quantizer, so the block loops
 * index them by position alone.  mobi_ratecost_init() fills one entry per
 * quantizer when an encoder opens; mobi_qp() only ever yields 12..63. */
#define MOBI_QP_COUNT 64
typedef struct
{
    uint16_t mf[64];    /* forward scale, raster order */
    int16_t  iq[64];    /* inverse scale, before its shift */
    int16_t  dq[64];    /* inverse scale with its shift applied (0 where the shift is negative) */
    int16_t  dq_t[64];  /* dq, transposed */
    int      fwd_offset, fwd_shift, inv_shift;
} mobi_quant_t;
#define mobi_quant4 x264_template(mobi_quant4)
#define mobi_quant8 x264_template(mobi_quant8)
extern mobi_quant_t mobi_quant4[MOBI_QP_COUNT], mobi_quant8[MOBI_QP_COUNT];

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
#if MOBI_NEON && !HIGH_BIT_DEPTH
/* mobi_inverse4() across four vectors, one transform per lane */
static ALWAYS_INLINE void mobi_inverse4_neon( int32x4_t r[4] )
{
    const int32x4_t a = vaddq_s32( r[0], r[2] ), b = vsubq_s32( r[0], r[2] );
    const int32x4_t c = vaddq_s32( r[1], vshrq_n_s32( r[3], 1 ) );
    const int32x4_t d = vsubq_s32( vshrq_n_s32( r[1], 1 ), r[3] );
    r[0] = vaddq_s32( a, c );
    r[1] = vaddq_s32( b, d );
    r[2] = vsubq_s32( b, d );
    r[3] = vsubq_s32( a, c );
}
/* mobi_idct8_1d() across eight vectors, one transform per lane */
static ALWAYS_INLINE void mobi_idct8_neon( int32x4_t a[8] )
{
    int32x4_t t[4] = { a[0], a[2], a[4], a[6] };
    mobi_inverse4_neon( t );
    const int32x4_t e = vsubq_s32( vsubq_s32( vaddq_s32( a[7], a[1] ), a[3] ), vshrq_n_s32( a[3], 1 ) );
    const int32x4_t f = vaddq_s32( vaddq_s32( vsubq_s32( a[7], a[1] ), a[5] ), vshrq_n_s32( a[5], 1 ) );
    const int32x4_t g = vsubq_s32( vsubq_s32( vsubq_s32( a[5], a[3] ), a[7] ), vshrq_n_s32( a[7], 1 ) );
    const int32x4_t k = vaddq_s32( vaddq_s32( vaddq_s32( a[5], a[3] ), a[1] ), vshrq_n_s32( a[1], 1 ) );
    const int32x4_t x3 = vaddq_s32( g, vshrq_n_s32( k, 2 ) );
    const int32x4_t x2 = vaddq_s32( e, vshrq_n_s32( f, 2 ) );
    const int32x4_t x1 = vsubq_s32( vshrq_n_s32( e, 2 ), f );
    const int32x4_t x0 = vsubq_s32( k, vshrq_n_s32( g, 2 ) );
    a[0] = vaddq_s32( t[0], x0 );  a[1] = vaddq_s32( t[1], x1 );
    a[2] = vaddq_s32( t[2], x2 );  a[3] = vaddq_s32( t[3], x3 );
    a[4] = vsubq_s32( t[3], x3 );  a[5] = vsubq_s32( t[2], x2 );
    a[6] = vsubq_s32( t[1], x1 );  a[7] = vsubq_s32( t[0], x0 );
}
#endif

/* Dequantize an 8x8 block of levels (raster order) with the decoder's rule,
 * level * (DQ[q%6][pos] << (q/6 - 2)), and add its inverse transform to p_dst.
 * The products go into 32 bits because level*step overflows int16, and the
 * transform is the decoder's own: x264's add8x8_idct8 normalises differently,
 * which accumulates as drift across P-frames. */
static ALWAYS_INLINE void mobi_dequant_add8x8( pixel *p_dst, const dctcoef *dct, const mobi_quant_t *t )
{
#if MOBI_NEON && !HIGH_BIT_DEPTH
    /* The scalar code transforms the rows, transposes, transforms the rows
     * again and adds.  Here a vector holds one column, so each pass runs
     * across the vectors and the transposes sit around the passes instead.
     * The residual is narrowed with saturation before the add: anything that
     * saturates clips to 0 or 255 either way. */
    int16x8_t l[8];
    int32x4_t lo[8], hi[8];
    for( int i = 0; i < 8; i++ )
        l[i] = vld1q_s16( dct + 8*i );
    mobi_transpose8_neon( l );
    for( int i = 0; i < 8; i++ )
    {
        const int16x8_t dq = vld1q_s16( t->dq_t + 8*i );
        lo[i] = vmull_s16( vget_low_s16( l[i] ), vget_low_s16( dq ) );
        hi[i] = vmull_high_s16( l[i], dq );
    }
    lo[0] = vaddq_s32( lo[0], vsetq_lane_s32( 32, vdupq_n_s32( 0 ), 0 ) );
    mobi_idct8_neon( lo );
    mobi_idct8_neon( hi );
    mobi_transpose4_s32_neon( lo );
    mobi_transpose4_s32_neon( lo + 4 );
    mobi_transpose4_s32_neon( hi );
    mobi_transpose4_s32_neon( hi + 4 );
    for( int i = 0; i < 4; i++ )
    {
        const int32x4_t tmp = lo[4+i];
        lo[4+i] = hi[i];
        hi[i] = tmp;
    }
    mobi_idct8_neon( lo );
    mobi_idct8_neon( hi );
    for( int i = 0; i < 8; i++ )
        l[i] = vcombine_s16( vqmovn_s32( vshrq_n_s32( lo[i], 6 ) ), vqmovn_s32( vshrq_n_s32( hi[i], 6 ) ) );
    mobi_transpose8_neon( l );
    for( int i = 0; i < 8; i++ )
    {
        const int16x8_t px = vreinterpretq_s16_u16( vmovl_u8( vld1_u8( p_dst + i*FDEC_STRIDE ) ) );
        vst1_u8( p_dst + i*FDEC_STRIDE, vqmovun_s16( vqaddq_s16( px, l[i] ) ) );
    }
#else
    int mat[64];
    for( int i = 0; i < 64; i++ )
        mat[i] = (int)dct[i] * t->dq[i];
    mobi_add8x8_idct8( p_dst, mat );
#endif
}

/* Whether this macroblock's luma residual is coded 8x8.  The coefficient
 * writer gates on x264_mb_transform_8x8_allowed() as well as b_transform_8x8
 * -- a P_8x8 macroblock with sub-8x8 partitions is not allowed 8x8 even when
 * the transform decision picked it -- so the encode side must use exactly the
 * same predicate or it reconstructs one way and writes the other. */
static ALWAYS_INLINE int mobi_use_8x8( x264_t *h )
{
    return h->mb.b_transform_8x8 && x264_mb_transform_8x8_allowed( h );
}

/* Scan an 8x8 coefficient block into coded order.
 *
 * The Mobiclip decoder scans 8x8 blocks with ff_zigzag_direct, which is
 * mobi_zigzag8x8 -- the plain scan-index -> position table.  This fork used
 * ZigZagTable8x8, the *inverse* permutation (position -> scan index), so 8x8
 * luma coefficients landed in the wrong slots; that is why luma 8x8 was
 * force-disabled in mobiclip mode.  Chroma always used the right table, which
 * is why only luma was affected.  Non-mobiclip encoding keeps x264's own scan. */
static ALWAYS_INLINE void mobi_scan_8x8( x264_t *h, dctcoef level[64], dctcoef dct[64] )
{
    if( h->param.i_mobiclip )
        for( int i = 0; i < 64; i++ )
            level[i] = dct[mobi_zigzag8x8[i]];
    else
        h->zigzagf.scan_8x8( level, dct );
}

/* Mobiclip's own forward transforms.  Structurally these are the H.264 integer
 * DCTs, but the reference encoder keeps every intermediate in int16 and runs
 * the 8x8 column pass *before* the row pass.  The >>1 / >>2 truncations make
 * the 2D transform non-commutative, so both details change the coefficients;
 * x264's dctf.sub*_dct do it the other way with wider intermediates. */
static ALWAYS_INLINE void mobi_transform_8pt( int16_t *p, int stride )
{
    int16_t x0 = p[0*stride], x1 = p[1*stride], x2 = p[2*stride], x3 = p[3*stride];
    int16_t x4 = p[4*stride], x5 = p[5*stride], x6 = p[6*stride], x7 = p[7*stride];

    int16_t s0 = x0 + x7, s1 = x1 + x6, s2 = x2 + x5, s3 = x3 + x4;
    int16_t d0 = x0 - x7, d1 = x1 - x6, d2 = x2 - x5, d3 = x3 - x4;

    int16_t ee0 = s0 + s3, ee1 = s1 + s2;
    int16_t eo0 = s0 - s3, eo1 = s1 - s2;

    p[0*stride] = ee0 + ee1;
    p[2*stride] = eo0 + (eo1 >> 1);
    p[4*stride] = ee0 - ee1;
    p[6*stride] = (eo0 >> 1) - eo1;

    int16_t o0 = d0 + (d0 >> 1) + d1 + d2;
    int16_t o1 = d0 - d2 - (d2 >> 1) - d3;
    int16_t o2 = d0 + d3 - d1 - (d1 >> 1);
    int16_t o3 = d1 - d2 + d3 + (d3 >> 1);

    p[1*stride] = o0 + (o3 >> 2);
    p[3*stride] = o1 + (o2 >> 2);
    p[5*stride] = o2 - (o1 >> 2);
    p[7*stride] = (o0 >> 2) - o3;
}

static ALWAYS_INLINE void mobi_sub8x8_dct8( dctcoef dst[64], pixel *p_src, pixel *p_dst )
{
#if MOBI_NEON && !HIGH_BIT_DEPTH
    /* One vector per row: the column pass runs across the vectors, and after a
     * transpose so does the row pass, which leaves the result in the
     * transposed layout the scalar code ends with. */
    int16x8_t r[8];
    for( int y = 0; y < 8; y++ )
        r[y] = vreinterpretq_s16_u16( vsubl_u8( vld1_u8( p_src + y*FENC_STRIDE ), vld1_u8( p_dst + y*FDEC_STRIDE ) ) );
    mobi_transform_8pt_neon( r );
    mobi_transpose8_neon( r );
    mobi_transform_8pt_neon( r );
    for( int y = 0; y < 8; y++ )
        vst1q_s16( dst + 8*y, r[y] );
#else
    int16_t t[64];
    for( int y = 0; y < 8; y++ )
        for( int x = 0; x < 8; x++ )
            t[y*8+x] = (int16_t)(p_src[y*FENC_STRIDE+x] - p_dst[y*FDEC_STRIDE+x]);
    for( int x = 0; x < 8; x++ )
        mobi_transform_8pt( &t[x], 8 );   /* columns first */
    for( int y = 0; y < 8; y++ )
        mobi_transform_8pt( &t[y*8], 1 ); /* then rows */
    /* x264's sub8x8_dct8 writes its second pass transposed, and the rest of
     * the mobiclip path (zigzag, coefficient writer, inverse transform) is
     * built on that layout, so transpose to match. */
    for( int y = 0; y < 8; y++ )
        for( int x = 0; x < 8; x++ )
            dst[x*8+y] = t[y*8+x];
#endif
}

#if MOBI_NEON && !HIGH_BIT_DEPTH
/* Two rows of four pixels as eight 16-bit lanes. */
static ALWAYS_INLINE int16x8_t mobi_load4x2_neon( const pixel *p, int stride )
{
    const uint32x2_t v = vset_lane_u32( M32( p + stride ), vdup_n_u32( M32( p ) ), 1 );
    return vreinterpretq_s16_u16( vmovl_u8( vreinterpret_u8_u32( v ) ) );
}
#endif

static ALWAYS_INLINE void mobi_sub4x4_dct( dctcoef dst[16], pixel *p_src, pixel *p_dst )
{
#if MOBI_NEON && !HIGH_BIT_DEPTH
    const int16x8_t r01 = vsubq_s16( mobi_load4x2_neon( p_src, FENC_STRIDE ), mobi_load4x2_neon( p_dst, FDEC_STRIDE ) );
    const int16x8_t r23 = vsubq_s16( mobi_load4x2_neon( p_src + 2*FENC_STRIDE, FENC_STRIDE ),
                                     mobi_load4x2_neon( p_dst + 2*FDEC_STRIDE, FDEC_STRIDE ) );
    int16x4_t r[4] = { vget_low_s16( r01 ), vget_high_s16( r01 ), vget_low_s16( r23 ), vget_high_s16( r23 ) };
    mobi_transform_4pt_neon( r );
    mobi_transpose4_neon( r );
    mobi_transform_4pt_neon( r );
    vst1q_s16( dst,     vcombine_s16( r[0], r[1] ) );
    vst1q_s16( dst + 8, vcombine_s16( r[2], r[3] ) );
#else
    int16_t t[16];
    for( int y = 0; y < 4; y++ )
        for( int x = 0; x < 4; x++ )
            t[y*4+x] = (int16_t)(p_src[y*FENC_STRIDE+x] - p_dst[y*FDEC_STRIDE+x]);
    for( int x = 0; x < 4; x++ )   /* column pass */
    {
        int16_t a = t[0*4+x] + t[3*4+x], b = t[1*4+x] + t[2*4+x];
        int16_t c = t[1*4+x] - t[2*4+x], d = t[0*4+x] - t[3*4+x];
        t[0*4+x] = a + b;
        t[1*4+x] = c + 2*d;
        t[2*4+x] = a - b;
        t[3*4+x] = d - 2*c;
    }
    for( int y = 0; y < 4; y++ )   /* row pass, transposed out (see above) */
    {
        int16_t a = t[y*4+0] + t[y*4+3], b = t[y*4+1] + t[y*4+2];
        int16_t c = t[y*4+1] - t[y*4+2], d = t[y*4+0] - t[y*4+3];
        dst[0*4+y] = a + b;
        dst[1*4+y] = c + 2*d;
        dst[2*4+y] = a - b;
        dst[3*4+y] = d - 2*c;
    }
#endif
}

/* b_intra is accepted for call-site symmetry but unused: the reference encoder
 * uses the same (1<<shift)/3 bias for intra and inter blocks alike. */
static ALWAYS_INLINE int mobi_quant_4x4( x264_t *h, dctcoef dct[16], int i_qp, int b_intra )
{
    const mobi_quant_t *t = &mobi_quant4[mobi_qp( h, i_qp )];
    (void)b_intra;
#if MOBI_NEON && !HIGH_BIT_DEPTH
    const int32x4_t f = vdupq_n_s32( t->fwd_offset ), neg_shift = vdupq_n_s32( -t->fwd_shift );
    const int16x8_t a = mobi_quant8_neon( vld1q_s16( dct ),     vld1q_u16( t->mf ),     f, neg_shift );
    const int16x8_t b = mobi_quant8_neon( vld1q_s16( dct + 8 ), vld1q_u16( t->mf + 8 ), f, neg_shift );
    vst1q_s16( dct, a );
    vst1q_s16( dct + 8, b );
    return !!vmaxvq_u16( vreinterpretq_u16_s16( vorrq_s16( a, b ) ) );
#else
    int shift = t->fwd_shift;
    int f = t->fwd_offset;
    int nz = 0;
    for( int i = 0; i < 16; i++ )
    {
        int coef = dct[i], level;
        int m = t->mf[i];
        if( coef >= 0 )
            level =  ( ( coef * m + f) >> shift);
        else
            level = -( ((-coef) * m + f) >> shift);
        dct[i] = level;
        nz |= level;
    }
    return !!nz;
#endif
}

static ALWAYS_INLINE int mobi_quant_8x8( x264_t *h, dctcoef dct[64], int i_qp, int b_intra )
{
    const mobi_quant_t *t = &mobi_quant8[mobi_qp( h, i_qp )];
    (void)b_intra;
#if MOBI_NEON && !HIGH_BIT_DEPTH
    const int32x4_t f = vdupq_n_s32( t->fwd_offset ), neg_shift = vdupq_n_s32( -t->fwd_shift );
    int16x8_t any = vdupq_n_s16( 0 );
    for( int i = 0; i < 64; i += 8 )
    {
        const int16x8_t v = mobi_quant8_neon( vld1q_s16( dct + i ), vld1q_u16( t->mf + i ), f, neg_shift );
        vst1q_s16( dct + i, v );
        any = vorrq_s16( any, v );
    }
    return !!vmaxvq_u16( vreinterpretq_u16_s16( any ) );
#else
    int shift = t->fwd_shift;
    int f = t->fwd_offset;
    int nz = 0;
    for( int i = 0; i < 64; i++ )
    {
        int coef = dct[i], level;
        int m = t->mf[i];
        if( coef >= 0 )
            level =  ( ( coef * m + f) >> shift);
        else
            level = -( ((-coef) * m + f) >> shift);
        dct[i] = level;
        nz |= level;
    }
    return !!nz;
#endif
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
/* I_8x8 intra macroblocks for Mobiclip.  These were force-disabled for a long
 * time because the path desynced whole frames; the cause was a missing 8x8
 * Mobiclip predictor plus H.264 mode lists offering VL where the composed mode
 * map needs an above-left neighbour.  With both fixed the path is bit-exact
 * against the decoder across every bitexact.sh case, so it is on by default.
 * Set MOBI_I8X8=0 to fall back to I_4x4-only if a stream ever needs bisecting.
 * Defined outside the !HIGH_BIT_DEPTH guard because analyse.c compiles for both
 * bit depths. */
static ALWAYS_INLINE int mobi_i8x8_enabled( void )
{
    return x264_mobi_env.b_i8x8;
}

#if !HIGH_BIT_DEPTH
/* H.264 4x4 intra mode (0..8) -> Mobiclip intra mode.  MUST stay identical to
 * the table in cavlc.c so the reconstruction prediction matches the mode that
 * gets written into the bitstream (and thus the decoder's predict_intra). */
static const int mobi_h264_to_mobiclip_mode[9] = { 0, 1, 3, 3, 7, 6, 5, 8, 4 };
/* read neighbour at block-relative (rx,ry) with decoder pget() semantics */
/* Neighbour fetch for Mobiclip intra prediction, generic over block size.
 * The caller passes the block's frame-space origin (ax,ay) and its size, so the
 * same code serves 4x4 and 8x8 luma: the decoder's predict_intra() is one
 * size-parametrised routine, and every mode formula below is already written in
 * terms of `size`. */
static ALWAYS_INLINE int mobi_pget( x264_t *h, pixel *p_dst, int ax, int ay, int size, int rx, int ry )
{
    int mb_x = h->mb.i_mb_x;
    int mb_y = h->mb.i_mb_y;
    if( rx == -1 && ry >= size )      { rx = -1; ry = size - 1; }
    else if( rx >= -1 && ry >= -1 )   { /* keep */ }
    else if( rx == -1 && ry == -2 )   { rx = 0; ry = -1; }
    else if( rx == -2 && ry == -1 )   { rx = -1; ry = 0; }

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
#define MPG(rx,ry) mobi_pget(h,p_dst,ax,ay,size,(rx),(ry))
static ALWAYS_INLINE int mobi_half_horz( x264_t *h, pixel *p_dst, int ax, int ay, int size, int x, int y )
{ return mobi_half3( MPG(x-1,y), MPG(x,y), MPG(x+1,y) ); }
static ALWAYS_INLINE int mobi_half_vert( x264_t *h, pixel *p_dst, int ax, int ay, int size, int x, int y )
{ return mobi_half3( MPG(x,y-1), MPG(x,y), MPG(x,y+1) ); }

/* Mirrors the decoder's predict_intra() for a size x size luma block at frame
 * origin (ax,ay).  p_dst is the block's fdec tile position, FDEC_STRIDE. */
static void mobi_predict_block( x264_t *h, pixel *p_dst, int ax, int ay, int size, int pmode )
{
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
    /* A block that lies wholly inside the frame reads only two kinds of
     * pixel: its row above and column to the left, and pixels of its own it
     * has already written.  Fetch the former once each (still through
     * mobi_pget(), with its clamping and remapping) rather than for every
     * pixel that uses them, and read the latter straight from p_dst, which is
     * where mobi_pget() finds them when no clamping applies.  (Mode 8, which
     * the encoder never selects, is left to the general code below.) */
    if( ((unsigned)pmode < 2 || (unsigned)(pmode - 4) < 4) &&
        ax + size <= h->param.i_width && ay + size <= h->param.i_height )
    {
        /* TP(i) = MPG(i,-1), LP(j) = MPG(-1,j); the ranges each mode touches */
        static const int8_t t_first[8] = { 0, 1, 1, 1, 1, -1, -1, -1 };
        static const int8_t l_first[8] = { 1, 0, 1, 1, 0, -1, -1, -1 };
        int top[16], left[16];
#define TP(i) top[(i)+1]
#define LP(j) left[(j)+1]
        const int t_last = pmode == 5 ? size - 2 : size - 1;
        const int l_last = pmode == 6 ? size - 2 : size - 1;
        for( int i = t_first[pmode]; i <= t_last; i++ )
            TP(i) = MPG(i, -1);
        for( int j = l_first[pmode]; j <= l_last; j++ )
            LP(j) = MPG(-1, j);
        if( pmode == 4 )
            /* below the block the left column repeats its last pixel */
            for( int j = size; j <= size + (size-1)/2 + 1; j++ )
                LP(j) = LP(size-1);
#define PD(x,y) p_dst[(y)*FDEC_STRIDE + (x)]
        for( int y = 0; y < size; y++ )
            for( int x = 0; x < size; x++ )
            {
                int val;
                switch( pmode )
                {
                case 0:  val = TP(x); break;
                case 1:  val = LP(y); break;
                case 4:
                    if( (x % 2) == 0 )
                        val = mobi_half( LP(y + x/2), LP(y + x/2 + 1) );
                    else
                        val = mobi_half3( LP(y + x/2), LP(y + x/2 + 1), LP(y + x/2 + 2) );
                    break;
                case 5:
                    if( x == 0 )
                        val = mobi_half( LP(y-1), LP(y) );
                    else if( y == 0 )
                        val = mobi_half3( x == 1 ? LP(0) : TP(x-3), TP(x-2), TP(x-1) );
                    else if( x == 1 )
                        val = mobi_half3( LP(y-2), LP(y-1), LP(y) );
                    else
                        val = PD(x-2, y-1);
                    break;
                case 6:
                    if( y == 0 )
                        val = mobi_half( TP(x-1), TP(x) );
                    else if( x == 0 )
                        val = mobi_half3( y == 1 ? TP(0) : LP(y-3), LP(y-2), LP(y-1) );
                    else if( y == 1 )
                        val = mobi_half3( TP(x-2), TP(x-1), TP(x) );
                    else
                        val = PD(x-1, y-2);
                    break;
                default: /* 7 */
                    if( x && y )
                        val = PD(x-1, y-1);
                    else
                        val = mobi_half3( x == 0 ? LP(y) : TP(x-2),
                                          x == 0 ? LP(y-1) : TP(x-1),
                                          y == 0 ? TP(x) : LP(y-2) );
                    break;
                }
                PD(x, y) = val;
            }
#undef PD
#undef TP
#undef LP
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
                    val = mobi_half_vert( h, p_dst, ax, ay, size, -1, y + x/2 + 1 );
                break;
            case 5: /* pick_5 */
                if( x == 0 )
                    val = mobi_half( MPG(-1, y-1), MPG(-1, y) );
                else if( y == 0 )
                    val = mobi_half_horz( h, p_dst, ax, ay, size, x-2, y-1 );
                else if( x == 1 )
                    val = mobi_half_vert( h, p_dst, ax, ay, size, x-2, y-1 );
                else
                    val = MPG(x-2, y-1);
                break;
            case 6: /* pick_6 */
                if( y == 0 )
                    val = mobi_half( MPG(x-1, -1), MPG(x, -1) );
                else if( x == 0 )
                    val = mobi_half_vert( h, p_dst, ax, ay, size, x-1, y-2 );
                else if( y == 1 )
                    val = mobi_half_horz( h, p_dst, ax, ay, size, x-1, y-2 );
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
                    val = mobi_half_horz( h, p_dst, ax, ay, size, x+1, -1 );
                else if( x < size - 1 )
                    val = MPG(x+1, y-2);
                else if( (y % 2) == 0 )
                    val = mobi_half( MPG(y/2 + size - 1, -1), MPG(y/2 + size, -1) );
                else
                    val = mobi_half_horz( h, p_dst, ax, ay, size, y/2 + size, -1 );
                break;
            default: val = 0x80; break;
            }
            p_dst[y*FDEC_STRIDE + x] = val;
        }
    }
}
#undef MPG

/* 4x4 luma: origin from the raster block index. */
static void mobi_predict_4x4( x264_t *h, pixel *p_dst, int idx, int pmode )
{
    mobi_predict_block( h, p_dst,
                        h->mb.i_mb_x * 16 + block_idx_x[idx] * 4,
                        h->mb.i_mb_y * 16 + block_idx_y[idx] * 4, 4, pmode );
}

/* 8x8 luma: idx 0..3 in raster order within the macroblock, matching the order
 * decode_macroblock() walks its four 8x8 sub-blocks. */
static void mobi_predict_8x8( x264_t *h, pixel *p_dst, int idx, int pmode )
{
    mobi_predict_block( h, p_dst,
                        h->mb.i_mb_x * 16 + (idx & 1) * 8,
                        h->mb.i_mb_y * 16 + (idx >> 1) * 8, 8, pmode );
}

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

#if MOBI_NEON && !HIGH_BIT_DEPTH
/* dst[i] = src[mobi_zigzag4x4[i]] for a block held in two vectors */
static ALWAYS_INLINE void mobi_zigzag4x4_neon( dctcoef *dst, int16x8_t c0, int16x8_t c1 )
{
    static const uint8_t scan[32] =
    {
         0, 1,  8, 9,  2, 3,  4, 5, 10,11, 16,17, 24,25, 18,19,
        12,13,  6, 7, 14,15, 20,21, 26,27, 28,29, 22,23, 30,31
    };
    const uint8x16x2_t tab = {{ vreinterpretq_u8_s16( c0 ), vreinterpretq_u8_s16( c1 ) }};
    vst1q_u8( (uint8_t*)dst,      vqtbl2q_u8( tab, vld1q_u8( scan ) ) );
    vst1q_u8( (uint8_t*)dst + 16, vqtbl2q_u8( tab, vld1q_u8( scan + 16 ) ) );
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

    if( h->param.i_mobiclip )
        mobi_sub4x4_dct( dct4x4, p_src, p_dst );
    else
        h->dctf.sub4x4_dct( dct4x4, p_src, p_dst );

    nz = x264_quant_4x4( h, dct4x4, i_qp, ctx_cat_plane[DCT_LUMA_4x4][p], 1, p, idx );
    h->mb.cache.non_zero_count[x264_scan8[p*16+idx]] = nz;

    if( !h->param.i_mobiclip )
    {
        /* Standard H.264: x264's own scan, dequant and inverse transform.
         * i_cbp_luma MUST be set here -- the CAVLC writer emits a block's
         * coefficients only for the 8x8 groups flagged in the CBP, so leaving
         * it clear writes a "no luma residual" CBP while the nnz cache still
         * claims coefficients.  The decoder then predicts total_coeff from a
         * different nC than the encoder did and the slice desyncs a few
         * macroblocks later ("negative number of zero coeffs", "cbp too
         * large", bogus sub_mb_type/skip runs).  Mobiclip sets its own CBP
         * around this call, so it is deliberately left out of that path. */
        if( nz )
        {
            h->mb.i_cbp_luma |= 1<<(idx>>2);
            h->zigzagf.scan_4x4( h->dct.luma4x4[p*16+idx], dct4x4 );
            h->quantf.dequant_4x4( dct4x4, h->dequant4_mf[p?CQM_4IC:CQM_4IY], i_qp );
            h->dctf.add4x4_idct( p_dst, dct4x4 );
        }
        return;
    }

#if MOBI_NEON && !HIGH_BIT_DEPTH
    /* Always write luma4x4 for luma, to clear stale analysis data. */
    if( p == 0 || nz )
        mobi_zigzag4x4_neon( h->dct.luma4x4[p*16+idx], vld1q_s16( dct4x4 ), vld1q_s16( dct4x4 + 8 ) );
    if( nz )
    {
        const mobi_quant_t *t = &mobi_quant4[mobi_qp( h, i_qp )];
        const int16x8_t px01 = mobi_load4x2_neon( p_dst, FDEC_STRIDE );
        const int16x8_t px23 = mobi_load4x2_neon( p_dst + 2*FDEC_STRIDE, FDEC_STRIDE );
        int16x8_t s01, s23;
        /* Dequantize into a 32-bit matrix: level*qtab (level up to ~hundreds,
         * qtab up to ~29<<2) overflows the int16 dct4x4 buffer, and the
         * inverse4 row/column sums overflow it further.  The decoder uses
         * int mat[]; mirror that exactly (this is the same fix as chroma
         * 8x8) or large-coefficient blocks reconstruct wrong and the error
         * drifts through P-frames as visible shearing. */
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
        /* As in mobi_dequant_add8x8(), a vector holds one column of the
         * matrix and the residual is narrowed with saturation, which leaves
         * the range test and the clip with the same outcome. */
        for( int mobi_pass = 0; ; mobi_pass++ )
        {
            int16x4_t l[4] = { vld1_s16( dct4x4 ), vld1_s16( dct4x4 + 4 ), vld1_s16( dct4x4 + 8 ), vld1_s16( dct4x4 + 12 ) };
            int32x4_t m[4];
            mobi_transpose4_neon( l );
            for( int i = 0; i < 4; i++ )
                m[i] = vmull_s16( l[i], vld1_s16( t->dq_t + 4*i ) );
            m[0] = vaddq_s32( m[0], vsetq_lane_s32( 32, vdupq_n_s32( 0 ), 0 ) );
            mobi_inverse4_neon( m );
            mobi_transpose4_s32_neon( m );
            mobi_inverse4_neon( m );
            for( int i = 0; i < 4; i++ )
                l[i] = vqmovn_s32( vshrq_n_s32( m[i], 6 ) );
            mobi_transpose4_neon( l );
            s01 = vqaddq_s16( px01, vcombine_s16( l[0], l[1] ) );
            s23 = vqaddq_s16( px23, vcombine_s16( l[2], l[3] ) );
            const uint16x8_t oob = vorrq_u16( vorrq_u16( vcltq_s16( s01, vdupq_n_s16( -64 ) ), vcgtq_s16( s01, vdupq_n_s16( 319 ) ) ),
                                              vorrq_u16( vcltq_s16( s23, vdupq_n_s16( -64 ) ), vcgtq_s16( s23, vdupq_n_s16( 319 ) ) ) );
            if( !vmaxvq_u16( oob ) || mobi_pass >= 16 )
            {
                /* The attenuation changed the levels; re-store the coded
                 * coefficients so the bitstream matches the reconstruction. */
                if( mobi_pass && p == 0 )
                    mobi_zigzag4x4_neon( h->dct.luma4x4[p*16+idx], vld1q_s16( dct4x4 ), vld1q_s16( dct4x4 + 8 ) );
                break;
            }
            int has_ac = 0;
            for( int i = 1; i < 16; i++ ) if( dct4x4[i] ) { has_ac = 1; break; }
            if( has_ac )
                for( int i = 1; i < 16; i++ ) dct4x4[i] /= 2;
            else
                dct4x4[0] /= 2;
        }
        const uint32x2_t o01 = vreinterpret_u32_u8( vqmovun_s16( s01 ) );
        const uint32x2_t o23 = vreinterpret_u32_u8( vqmovun_s16( s23 ) );
        M32( p_dst + 0*FDEC_STRIDE ) = vget_lane_u32( o01, 0 );
        M32( p_dst + 1*FDEC_STRIDE ) = vget_lane_u32( o01, 1 );
        M32( p_dst + 2*FDEC_STRIDE ) = vget_lane_u32( o23, 0 );
        M32( p_dst + 3*FDEC_STRIDE ) = vget_lane_u32( o23, 1 );
    }
#else
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
            /* Decoder-exact 4x4 dequant: level * (DQ[q%6][pos] << (q/6)). */
            const int16_t *qtab64 = mobi_quant4[mobi_qp( h, i_qp )].dq;
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
#endif
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

#if !HIGH_BIT_DEPTH
        if( h->param.i_mobiclip && p == 0 )
        {
            /* Same conversion as the I_4x4 path and as cavlc.c's I_8x8 write
             * branch (identical tables in both places), so the pixels we
             * reconstruct here match the mode actually written to the stream. */
            static const uint8_t x264_intra_to_h264[13] =
                { 2,0,1,2,2,8,6,5,4,3,2,2,2 };
            int i_h264_mode = x264_intra_to_h264[ (unsigned)i_mode + 1 < 13 ? i_mode + 1 : 0 ];
            int mobi_mode = mobi_h264_to_mobiclip_mode[ i_h264_mode < 9 ? i_h264_mode : 8 ];
            mobi_predict_8x8( h, p_dst, idx, mobi_mode );
        }
        else
#endif
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

    if( h->param.i_mobiclip )
        mobi_sub8x8_dct8( dct8x8, p_src, p_dst );
    else
        h->dctf.sub8x8_dct8( dct8x8, p_src, p_dst );

    nz = x264_quant_8x8( h, dct8x8, i_qp, ctx_cat_plane[DCT_LUMA_8x8][p], 1, p, idx );
    if( nz )
    {
        h->mb.i_cbp_luma |= 1<<idx;
        mobi_scan_8x8( h, h->dct.luma8x8[p*4+idx], dct8x8 );
        if( h->param.i_mobiclip )
        {
            /* Decoder-exact 8x8 dequant: level * (DQ[q%6][pos] << (q/6 - 2)).
             * Dequantize into a 32-bit matrix and reconstruct with the
             * decoder's own inverse transform -- level*step overflows the
             * int16 dct8x8 buffer, and x264's add8x8_idct8 normalises
             * differently, which accumulates as drift across P-frames. */
            mobi_dequant_add8x8( p_dst, dct8x8, &mobi_quant8[mobi_qp( h, i_qp )] );
        }
        else
        {
            h->quantf.dequant_8x8( dct8x8, h->dequant8_mf[CQM_8IY], i_qp );
            h->dctf.add8x8_idct8( p_dst, dct8x8 );
        }
        STORE_8x8_NNZ( p, idx, 1 );
    }
    else
        STORE_8x8_NNZ( p, idx, 0 );
}

#endif
