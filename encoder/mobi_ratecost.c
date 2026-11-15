/*****************************************************************************
 * mobi_ratecost.c: retail Mobiclip's SAD-rate RD-cost primitives
 *****************************************************************************
 * Copyright (C) 2026 quatric <quatricsoftware@gmail.com>
 *
 * Ported verbatim from Helwettpackardenterprise's MODS_Encoder_v43_2
 * (mods_encoder.c:145-421 and 6694-6984). See mobi_ratecost.h for the
 * verification summary and exact quant-parameter formulas.
 *****************************************************************************/
#include "mobi_ratecost.h"
#include "mobi_ratecost_tables.h"
#include "common/common.h"
#include "macroblock.h"
#include <string.h>

/* Tables in mobi_ratecost_tables.c: mods_scan_4x4/8x8, mods_sad_thresh_4x4/8x8,
 * mods_vlc_mid_8x8, mods_vlc_last_8x8 -- copied verbatim from 43_2's
 * tests/vlc_tables.c rather than retyped, to avoid transcription risk on the
 * two 32768-entry cost tables. Declared in mobi_ratecost_tables.h. */

/* The SAD-rate cost kernel's own mf/iq tables (retail DLL addresses
 * 0x1009C940/0x1009CD00 for 4x4, 0x1009C640/0x1009CA00 for 8x8, confirmed by
 * reading the raw DLL bytes) turn out to be byte-identical to the tables
 * mobi_quant4_scale/mobi_dequant4_scale/mobi_quant8_scale/mobi_dequant8_scale
 * (macroblock.h) already use for the real forward/inverse quantize path --
 * verified numerically against the mapped DLL image before relying on this.
 * So no new embedded table is needed here; build the flat per-QP arrays the
 * same way mobi_quant_4x4()/mobi_quant_8x8() do. */
static void mobi_build_quant_4x4( int qp, uint16_t mf[16], int16_t iq[16],
                                   int *fwd_offset, int *fwd_shift, int *inv_shift )
{
    int q = qp; /* caller passes the already-mapped retail quantizer */
    *fwd_shift = 15 + q/6;
    *fwd_offset = (1 << *fwd_shift) / 3;
    *inv_shift = q/6;
    const uint16_t *m = mobi_quant4_scale[q%6];
    const uint8_t  *d = mobi_dequant4_scale[q%6];
    for( int i = 0; i < 16; i++ )
    {
        mf[i] = m[mobi_scale_idx4[i]];
        iq[i] = d[mobi_scale_idx4[i]];
    }
}
static void mobi_build_quant_8x8( int qp, uint16_t mf[64], int16_t iq[64],
                                   int *fwd_offset, int *fwd_shift, int *inv_shift )
{
    int q = qp;
    *fwd_shift = 16 + q/6;
    *fwd_offset = (1 << *fwd_shift) / 3;
    *inv_shift = q/6 - 2;
    const uint16_t *m = mobi_quant8_scale[q%6];
    const uint8_t  *d = mobi_dequant8_scale[q%6];
    for( int i = 0; i < 64; i++ )
    {
        mf[i] = m[mobi_scale_idx8[i]];
        iq[i] = d[mobi_scale_idx8[i]];
    }
}

/* ---- mods_encoder.c:145-234 (8-point transform) ---- */
static void mobi_transform_8pt_1d( int16_t *data, int stride, int count )
{
    for( int i = 0; i < count; i++ )
    {
        int16_t *p = data + i * stride;
        int16_t x0 = p[0], x1 = p[1], x2 = p[2], x3 = p[3];
        int16_t x4 = p[4], x5 = p[5], x6 = p[6], x7 = p[7];
        int16_t s0 = x0 + x7, s1 = x1 + x6, s2 = x2 + x5, s3 = x3 + x4;
        int16_t d0 = x0 - x7, d1 = x1 - x6, d2 = x2 - x5, d3 = x3 - x4;
        int16_t ee0 = s0 + s3, ee1 = s1 + s2;
        int16_t eo0 = s0 - s3, eo1 = s1 - s2;
        p[0] = ee0 + ee1;
        p[2] = eo0 + (eo1 >> 1);
        p[4] = ee0 - ee1;
        p[6] = (eo0 >> 1) - eo1;
        int16_t o0 = d0 + (d0 >> 1) + d1 + d2;
        int16_t o1 = d0 - d2 - (d2 >> 1) - d3;
        int16_t o2 = d0 + d3 - d1 - (d1 >> 1);
        int16_t o3 = d1 - d2 + d3 + (d3 >> 1);
        p[1] = o0 + (o3 >> 2);
        p[3] = o1 + (o2 >> 2);
        p[5] = o2 - (o1 >> 2);
        p[7] = (o0 >> 2) - o3;
    }
}
static void mobi_transpose_8x8( int16_t block[64] )
{
    for( int i = 0; i < 8; i++ )
        for( int j = i + 1; j < 8; j++ )
        {
            int16_t tmp = block[i * 8 + j];
            block[i * 8 + j] = block[j * 8 + i];
            block[j * 8 + i] = tmp;
        }
}
static void mobi_forward_transform_8x8( const int16_t *src, int16_t *dst, int src_stride )
{
    int16_t tmp[64];
    for( int r = 0; r < 8; r++ )
        for( int c = 0; c < 8; c++ )
            tmp[r * 8 + c] = src[r * src_stride + c];
    mobi_transpose_8x8( tmp );
    mobi_transform_8pt_1d( tmp, 8, 8 );
    mobi_transpose_8x8( tmp );
    mobi_transform_8pt_1d( tmp, 8, 8 );
    memcpy( dst, tmp, 128 );
}
static void mobi_inv_transform_8pt_1d( int16_t *Y )
{
    int16_t o0 = Y[5] - Y[7] - (Y[7] >> 1) - Y[3];
    int16_t o2 = Y[7] - Y[3] - (Y[3] >> 1) + Y[1];
    int16_t o1 = Y[5] + (Y[5] >> 1) + Y[7] - Y[1];
    int16_t o3 = Y[1] + (Y[1] >> 1) + Y[5] + Y[3];
    int16_t d0 = o0 + (o3 >> 2);
    int16_t d1 = (o1 >> 2) + o2;
    int16_t d2 = (o2 >> 2) - o1;
    int16_t d3 = o3 - (o0 >> 2);
    int16_t eo1 = (Y[2] >> 1) - Y[6];
    int16_t eo0 = (Y[6] >> 1) + Y[2];
    int16_t ee0 = Y[0] + Y[4];
    int16_t ee1 = Y[0] - Y[4];
    int16_t s0 = ee0 + eo0, s1 = ee1 + eo1;
    int16_t s3 = ee0 - eo0, s2 = ee1 - eo1;
    Y[0] = s0 + d3;  Y[1] = s1 + d2;  Y[2] = s2 + d1;  Y[3] = s3 + d0;
    Y[4] = s3 - d0;  Y[5] = s2 - d1;  Y[6] = s1 - d2;  Y[7] = s0 - d3;
}
static void mobi_inv_transform_8pt_1d_shr6( int16_t *Y )
{
    mobi_inv_transform_8pt_1d( Y );
    for( int i = 0; i < 8; i++ ) Y[i] = Y[i] >> 6;
}
static int mobi_forward_quantize( const int16_t *coeffs, int16_t *levels,
                                   const uint16_t *mf_matrix, int offset, int shift )
{
    int nonzero = 0;
    for( int i = 0; i < 64; i++ )
    {
        int16_t c = coeffs[i];
        if( c > 0 )
            levels[i] = (int16_t)(((int)c * mf_matrix[i] + offset) >> shift);
        else
            levels[i] = -(int16_t)(((int)(-c) * mf_matrix[i] + offset) >> shift);
        nonzero |= levels[i];
    }
    return nonzero;
}
static void mobi_inv_quantize( const int16_t *levels, const int16_t *dq_matrix,
                                int16_t *coeffs, int qp_div )
{
    for( int i = 0; i < 64; i++ )
    {
        int value = levels[i] * dq_matrix[i];
        coeffs[i] = (qp_div >= 0 && qp_div < 16)
            ? (int16_t)((uint32_t)(uint16_t)value << qp_div) : 0;
    }
}
static void mobi_reconstruct_block( const int16_t *quant_levels, const int16_t *dq_matrix,
                                     int qp_div, int16_t *output )
{
    int16_t tmp[64];
    mobi_inv_quantize( quant_levels, dq_matrix, tmp, qp_div );
    tmp[0] += 32;
    mobi_transpose_8x8( tmp );
    for( int r = 0; r < 8; r++ ) mobi_inv_transform_8pt_1d( &tmp[r * 8] );
    mobi_transpose_8x8( tmp );
    for( int r = 0; r < 8; r++ ) mobi_inv_transform_8pt_1d_shr6( &tmp[r * 8] );
    memcpy( output, tmp, 128 );
}

/* ---- mods_encoder.c:331-411 (4x4 transform/quant) ---- */
static void mobi_forward_transform_4x4( const int16_t *src, int16_t *dst )
{
    int16_t tmp[16];
    for( int j = 0; j < 4; j++ )
    {
        int a = src[0*4+j] + src[3*4+j];
        int b = src[1*4+j] + src[2*4+j];
        int c = src[1*4+j] - src[2*4+j];
        int d = src[0*4+j] - src[3*4+j];
        tmp[0*4+j] = (int16_t)(a + b);
        tmp[1*4+j] = (int16_t)(c + 2*d);
        tmp[2*4+j] = (int16_t)(a - b);
        tmp[3*4+j] = (int16_t)(d - 2*c);
    }
    for( int i = 0; i < 4; i++ )
    {
        int a = tmp[i*4+0] + tmp[i*4+3];
        int b = tmp[i*4+1] + tmp[i*4+2];
        int c = tmp[i*4+1] - tmp[i*4+2];
        int d = tmp[i*4+0] - tmp[i*4+3];
        dst[i*4+0] = (int16_t)(a + b);
        dst[i*4+1] = (int16_t)(c + 2*d);
        dst[i*4+2] = (int16_t)(a - b);
        dst[i*4+3] = (int16_t)(d - 2*c);
    }
}
static void mobi_inverse_transform_4x4( int16_t *data )
{
    int16_t tmp[16];
    for( int j = 0; j < 4; j++ )
    {
        int a = data[0*4+j] + data[2*4+j];
        int b = data[0*4+j] - data[2*4+j];
        int c = (data[1*4+j] >> 1) - data[3*4+j];
        int d = data[1*4+j] + (data[3*4+j] >> 1);
        tmp[0*4+j] = (int16_t)(a + d);
        tmp[1*4+j] = (int16_t)(b + c);
        tmp[2*4+j] = (int16_t)(b - c);
        tmp[3*4+j] = (int16_t)(a - d);
    }
    for( int i = 0; i < 4; i++ )
    {
        int a = tmp[i*4+0] + tmp[i*4+2];
        int b = tmp[i*4+0] - tmp[i*4+2];
        int c = (tmp[i*4+1] >> 1) - tmp[i*4+3];
        int d = tmp[i*4+1] + (tmp[i*4+3] >> 1);
        data[i*4+0] = (int16_t)((a + d + 32) >> 6);
        data[i*4+1] = (int16_t)((b + c + 32) >> 6);
        data[i*4+2] = (int16_t)((b - c + 32) >> 6);
        data[i*4+3] = (int16_t)((a - d + 32) >> 6);
    }
}
static int mobi_forward_quantize_4x4( const int16_t *coeffs, int16_t *levels,
                                       const uint16_t *mf, int offset, int shift )
{
    int any_nonzero = 0;
    for( int i = 0; i < 16; i++ )
    {
        int c = coeffs[i];
        int q;
        if( c >= 0 )
            q = (c * (int)mf[i] + offset) >> shift;
        else
            q = -(((-c) * (int)mf[i] + offset) >> shift);
        levels[i] = (int16_t)q;
        any_nonzero |= q;
    }
    return any_nonzero != 0;
}
static void mobi_inv_quantize_4x4( const int16_t *levels, const int16_t *dq_matrix,
                                    int shift, int16_t *output )
{
    for( int i = 0; i < 16; i++ )
        output[i] = (int16_t)((uint32_t)(int32_t)
            (levels[i] * dq_matrix[i]) << shift);
}

/* ---- mods_encoder.c:6694-6824 (VLC bit-count model) ---- */
static int mobi_count_vlc_bits_8x8( const int16_t *levels, int is_keyframe )
{
    int first_nz = -1, run = 0;
    for( int i = 0; i < 64; i++ )
    {
        if( levels[mods_scan_8x8[i]] != 0 ) { first_nz = i; break; }
        run++;
    }
    if( first_nz == -1 ) return 0;
    int prev_level = (int)levels[mods_scan_8x8[first_nz]];
    int prev_run = run, total_bits = 0;
    int pos = first_nz + 1; run = 0;
    while( pos < 64 )
    {
        int level = (int)levels[mods_scan_8x8[pos]]; pos++;
        if( level == 0 ) { run++; continue; }
        int combined = prev_level + 0x40;
        if( combined >= 0 && combined < 128 && prev_run < 128 )
        {
            int idx = (is_keyframe * 128 + prev_run) * 128 + combined;
            total_bits += (idx < 32768) ? mods_vlc_mid_8x8[idx] : 28;
        }
        else total_bits += 28;
        prev_level = level; prev_run = run; run = 0;
    }
    int combined = prev_level + 0x40;
    if( combined >= 0 && combined < 128 && prev_run < 128 )
    {
        int idx = (is_keyframe * 128 + prev_run) * 128 + combined;
        total_bits += (idx < 32768) ? mods_vlc_last_8x8[idx] : 28;
    }
    else total_bits += 28;
    return total_bits;
}
static int mobi_count_vlc_bits_4x4( const int16_t *levels, int is_keyframe )
{
    int first_nz = -1, run = 0;
    if( levels[0] != 0 ) first_nz = 0;
    else
    {
        run = 1;
        for( int i = 0; i < 15; i++ )
        {
            if( levels[mods_scan_4x4[i]] != 0 ) { first_nz = i + 1; break; }
            run++;
        }
    }
    if( first_nz == -1 ) return 0;
    int prev_level = (first_nz == 0) ? (int)levels[0]
                                     : (int)levels[mods_scan_4x4[first_nz - 1]];
    int prev_run = run, total_bits = 0;
    int pos = first_nz + 1; run = 0;
    while( pos < 16 )
    {
        int idx_in_block = (pos == 0) ? 0 : mods_scan_4x4[pos - 1];
        int level = (int)levels[idx_in_block]; pos++;
        if( level == 0 ) { run++; continue; }
        int combined = prev_level + 0x40;
        if( combined >= 0 && combined < 128 && prev_run < 128 )
        {
            int i = (is_keyframe * 128 + prev_run) * 128 + combined;
            total_bits += (i < 32768) ? mods_vlc_mid_8x8[i] : 28;
        }
        else total_bits += 28;
        prev_level = level; prev_run = run; run = 0;
    }
    int combined = prev_level + 0x40;
    if( combined >= 0 && combined < 128 && prev_run < 128 )
    {
        int i = (is_keyframe * 128 + prev_run) * 128 + combined;
        total_bits += (i < 32768) ? mods_vlc_last_8x8[i] : 28;
    }
    else total_bits += 28;
    return total_bits;
}

/* ---- mods_encoder.c:6836-6984 (RD-cost core) ---- */
int mobi_sad_rate_4x4_core(
    const uint8_t *src, const uint8_t *pred, int stride, uint8_t *workspace,
    int qp, int lambda, int is_keyframe, const uint16_t *mf, int fwd_offset,
    int fwd_shift, const int16_t *iq, int inv_shift, int sub_idx, int32_t *flag_ptr )
{
    int16_t *residual = (int16_t *)(workspace + 0x00);
    int16_t *levels = (int16_t *)(workspace + 0x20);
    int16_t *reconstructed = (int16_t *)(workspace + 0x40);
    int initial_sse = 0;
    for( int y = 0; y < 4; y++ )
        for( int x = 0; x < 4; x++ )
        {
            int d = (int)src[y * stride + x] - (int)pred[y * stride + x];
            residual[y * 4 + x] = (int16_t)d;
            initial_sse += d * d;
        }
    int threshold = (qp >= 0 && qp < 52) ? (int)mods_sad_thresh_4x4[qp] : 3;
    if( initial_sse < threshold ) return initial_sse;
    mobi_forward_transform_4x4( residual, levels );
    if( !mobi_forward_quantize_4x4( levels, levels, mf, fwd_offset, fwd_shift ) )
        return initial_sse;
    int vlc_bits = mobi_count_vlc_bits_4x4( levels, is_keyframe );
    mobi_inv_quantize_4x4( levels, iq, inv_shift, reconstructed );
    mobi_inverse_transform_4x4( reconstructed );
    int distortion = 0;
    for( int i = 0; i < 16; i++ )
    {
        int d = residual[i] - reconstructed[i];
        distortion += d * d;
    }
    int total_cost = distortion + lambda * vlc_bits;
    if( total_cost < initial_sse )
    {
        if( flag_ptr ) *flag_ptr |= sub_idx;
        return total_cost;
    }
    return initial_sse;
}

/* The flat tables for every quantizer the bitstream can carry (see
 * macroblock.h), built once by mobi_ratecost_init() rather than for each block
 * scored or coded. */
mobi_quant_t mobi_quant4[MOBI_QP_COUNT], mobi_quant8[MOBI_QP_COUNT];
/* mods_scan_4x4 (with the DC position in front) and mods_scan_8x8 as bytes */
static uint8_t mobi_scan4_u8[16], mobi_scan8_u8[64];

static void mobi_fill_dq( mobi_quant_t *q, int n )
{
    for( int y = 0; y < n; y++ )
        for( int x = 0; x < n; x++ )
        {
            int v = q->inv_shift >= 0 ? q->iq[y*n+x] << q->inv_shift : 0;
            q->dq[y*n+x] = v;
            q->dq_t[x*n+y] = v;
        }
}

void mobi_ratecost_init( void )
{
    for( int i = 0; i < 15; i++ )
        mobi_scan4_u8[i+1] = mods_scan_4x4[i];
    for( int i = 0; i < 64; i++ )
        mobi_scan8_u8[i] = mods_scan_8x8[i];
    for( int qp = 0; qp < MOBI_QP_COUNT; qp++ )
    {
        mobi_quant_t q4 = {{0}}, q8 = {{0}};
        mobi_build_quant_4x4( qp, q4.mf, q4.iq, &q4.fwd_offset, &q4.fwd_shift, &q4.inv_shift );
        mobi_build_quant_8x8( qp, q8.mf, q8.iq, &q8.fwd_offset, &q8.fwd_shift, &q8.inv_shift );
        mobi_fill_dq( &q4, 4 );
        mobi_fill_dq( &q8, 8 );
        /* every caller builds the same tables, so concurrent calls are harmless */
        mobi_quant4[qp] = q4;
        mobi_quant8[qp] = q8;
    }
}

#if MOBI_NEON
/* NEON forms of the two cost kernels above, for tightly packed blocks.  They
 * follow the reference step for step and keep its arithmetic: 16-bit lanes
 * wherever the reference stores into int16_t (so the wrap-around is the
 * same), 32-bit lanes for the quantizer products, the 4x4 inverse transform's
 * rounding shift and the distortion sum. */

/* mobi_inv_transform_8pt_1d(), one transform per lane */
static ALWAYS_INLINE void mobi_inv_transform_8pt_neon( int16x8_t y[8] )
{
    const int16x8_t o0 = vsubq_s16( vsubq_s16( y[5], y[7] ), vaddq_s16( vshrq_n_s16( y[7], 1 ), y[3] ) );
    const int16x8_t o2 = vaddq_s16( vsubq_s16( y[7], y[3] ), vsubq_s16( y[1], vshrq_n_s16( y[3], 1 ) ) );
    const int16x8_t o1 = vaddq_s16( vaddq_s16( y[5], vshrq_n_s16( y[5], 1 ) ), vsubq_s16( y[7], y[1] ) );
    const int16x8_t o3 = vaddq_s16( vaddq_s16( y[1], vshrq_n_s16( y[1], 1 ) ), vaddq_s16( y[5], y[3] ) );
    const int16x8_t d0 = vaddq_s16( o0, vshrq_n_s16( o3, 2 ) );
    const int16x8_t d1 = vaddq_s16( vshrq_n_s16( o1, 2 ), o2 );
    const int16x8_t d2 = vsubq_s16( vshrq_n_s16( o2, 2 ), o1 );
    const int16x8_t d3 = vsubq_s16( o3, vshrq_n_s16( o0, 2 ) );
    const int16x8_t eo1 = vsubq_s16( vshrq_n_s16( y[2], 1 ), y[6] );
    const int16x8_t eo0 = vaddq_s16( vshrq_n_s16( y[6], 1 ), y[2] );
    const int16x8_t ee0 = vaddq_s16( y[0], y[4] ), ee1 = vsubq_s16( y[0], y[4] );
    const int16x8_t s0 = vaddq_s16( ee0, eo0 ), s1 = vaddq_s16( ee1, eo1 );
    const int16x8_t s3 = vsubq_s16( ee0, eo0 ), s2 = vsubq_s16( ee1, eo1 );
    y[0] = vaddq_s16( s0, d3 );  y[1] = vaddq_s16( s1, d2 );
    y[2] = vaddq_s16( s2, d1 );  y[3] = vaddq_s16( s3, d0 );
    y[4] = vsubq_s16( s3, d0 );  y[5] = vsubq_s16( s2, d1 );
    y[6] = vsubq_s16( s1, d2 );  y[7] = vsubq_s16( s0, d3 );
}

/* What one coded coefficient costs, as mobi_count_vlc_bits_*() look it up. */
static ALWAYS_INLINE int mobi_vlc_cost( const uint8_t *tab, int level, int run, int is_keyframe )
{
    int combined = level + 0x40;
    if( combined >= 0 && combined < 128 && run < 128 )
    {
        int idx = (is_keyframe * 128 + run) * 128 + combined;
        return idx < 32768 ? tab[idx] : 28;
    }
    return 28;
}

/* mobi_count_vlc_bits_*() given where the non-zero levels are: nz[] holds one
 * nibble per coefficient in scan order (16 coefficients a word), scan[] maps
 * scan position to raster position.  Only the coded coefficients are visited;
 * the run before each is the distance from the one before it. */
static ALWAYS_INLINE int mobi_count_vlc_bits_neon( const int16_t *levels, const uint8_t *scan,
                                                   const uint64_t *nz, int words, int is_keyframe )
{
    int prev = -1, prev_level = 0, prev_run = 0, bits = 0;
    for( int w = 0; w < words; w++ )
        for( uint64_t m = nz[w]; m; )
        {
            const int n = __builtin_ctzll( m ) >> 2;
            const int pos = w*16 + n;
            m &= ~(0xfULL << (4*n));
            if( prev >= 0 )
                bits += mobi_vlc_cost( mods_vlc_mid_8x8, prev_level, prev_run, is_keyframe );
            prev_run = pos - prev - 1;
            prev_level = levels[scan[pos]];
            prev = pos;
        }
    if( prev < 0 )
        return 0;
    return bits + mobi_vlc_cost( mods_vlc_last_8x8, prev_level, prev_run, is_keyframe );
}

/* mobi_sad_rate_4x4_core() for src/pred at stride 4 */
static int mobi_sad_rate_4x4_neon( const uint8_t *src, const uint8_t *pred, int qp, int lambda,
                                   int is_keyframe, const mobi_quant_t *q )
{
    const uint8x16_t s = vld1q_u8( src ), p = vld1q_u8( pred );
    const int16x8_t res01 = vreinterpretq_s16_u16( vsubl_u8( vget_low_u8( s ), vget_low_u8( p ) ) );
    const int16x8_t res23 = vreinterpretq_s16_u16( vsubl_high_u8( s, p ) );
    int32x4_t acc = vmull_s16( vget_low_s16( res01 ), vget_low_s16( res01 ) );
    acc = vmlal_high_s16( acc, res01, res01 );
    acc = vmlal_s16( acc, vget_low_s16( res23 ), vget_low_s16( res23 ) );
    acc = vmlal_high_s16( acc, res23, res23 );
    const int initial_sse = vaddvq_s32( acc );
    const int threshold = (qp >= 0 && qp < 52) ? (int)mods_sad_thresh_4x4[qp] : 3;
    if( initial_sse < threshold ) return initial_sse;

    /* forward transform: down the columns, then along the rows */
    int16x4_t r[4] = { vget_low_s16( res01 ), vget_high_s16( res01 ), vget_low_s16( res23 ), vget_high_s16( res23 ) };
    for( int pass = 0; pass < 2; pass++ )
    {
        const int16x4_t a = vadd_s16( r[0], r[3] ), b = vadd_s16( r[1], r[2] );
        const int16x4_t c = vsub_s16( r[1], r[2] ), d = vsub_s16( r[0], r[3] );
        r[0] = vadd_s16( a, b );
        r[1] = vadd_s16( c, vshl_n_s16( d, 1 ) );
        r[2] = vsub_s16( a, b );
        r[3] = vsub_s16( d, vshl_n_s16( c, 1 ) );
        mobi_transpose4_neon( r );
    }

    const int32x4_t offset = vdupq_n_s32( q->fwd_offset ), neg_shift = vdupq_n_s32( -q->fwd_shift );
    ALIGNED_ARRAY_16( int16_t, levels, [16] );
    int32x4_t any = vdupq_n_s32( 0 );
    for( int i = 0; i < 4; i++ )
    {
        const int32x4_t v = mobi_quant_neon( r[i], vld1_u16( q->mf + 4*i ), offset, neg_shift );
        any  = vorrq_s32( any, v );
        r[i] = vmovn_s32( v );
        vst1_s16( levels + 4*i, r[i] );
    }
    if( !vmaxvq_u32( vreinterpretq_u32_s32( any ) ) )
        return initial_sse;
    const int16x8_t l01 = vcombine_s16( r[0], r[1] ), l23 = vcombine_s16( r[2], r[3] );
    const uint8x16_t coded = vcombine_u8( vmovn_u16( vtstq_s16( l01, l01 ) ), vmovn_u16( vtstq_s16( l23, l23 ) ) );
    const uint64_t nz = mobi_nibble_mask_neon( vqtbl1q_u8( coded, vld1q_u8( mobi_scan4_u8 ) ) );
    const int vlc_bits = mobi_count_vlc_bits_neon( levels, mobi_scan4_u8, &nz, 1, is_keyframe );

    /* dequantize, then the inverse transform: down the columns in 16 bits,
     * along the rows in 32 because of the rounding shift */
    const int16x4_t inv_shift = vdup_n_s16( q->inv_shift );
    for( int i = 0; i < 4; i++ )
        r[i] = vshl_s16( vmul_s16( r[i], vld1_s16( q->iq + 4*i ) ), inv_shift );
    {
        const int16x4_t a = vadd_s16( r[0], r[2] ), b = vsub_s16( r[0], r[2] );
        const int16x4_t c = vsub_s16( vshr_n_s16( r[1], 1 ), r[3] );
        const int16x4_t d = vadd_s16( r[1], vshr_n_s16( r[3], 1 ) );
        r[0] = vadd_s16( a, d );
        r[1] = vadd_s16( b, c );
        r[2] = vsub_s16( b, c );
        r[3] = vsub_s16( a, d );
        mobi_transpose4_neon( r );
    }
    {
        const int32x4_t round = vdupq_n_s32( 32 );
        const int32x4_t a = vaddq_s32( vaddl_s16( r[0], r[2] ), round );
        const int32x4_t b = vaddq_s32( vsubl_s16( r[0], r[2] ), round );
        const int32x4_t c = vsubl_s16( vshr_n_s16( r[1], 1 ), r[3] );
        const int32x4_t d = vaddl_s16( r[1], vshr_n_s16( r[3], 1 ) );
        r[0] = vmovn_s32( vshrq_n_s32( vaddq_s32( a, d ), 6 ) );
        r[1] = vmovn_s32( vshrq_n_s32( vaddq_s32( b, c ), 6 ) );
        r[2] = vmovn_s32( vshrq_n_s32( vsubq_s32( b, c ), 6 ) );
        r[3] = vmovn_s32( vshrq_n_s32( vsubq_s32( a, d ), 6 ) );
        mobi_transpose4_neon( r );
    }

    int32x4_t dist = vdupq_n_s32( 0 );
    for( int i = 0; i < 4; i++ )
    {
        const int16x4_t res = i < 2 ? (i ? vget_high_s16( res01 ) : vget_low_s16( res01 ))
                                    : (i & 1 ? vget_high_s16( res23 ) : vget_low_s16( res23 ));
        const int32x4_t d = vsubl_s16( res, r[i] );
        dist = vmlaq_s32( dist, d, d );
    }
    const int total_cost = vaddvq_s32( dist ) + lambda * vlc_bits;
    return total_cost < initial_sse ? total_cost : initial_sse;
}

/* mobi_sad_rate_8x8_core() for src/pred at stride 8; needs 0 <= inv_shift < 16 */
static int mobi_sad_rate_8x8_neon( const uint8_t *src, const uint8_t *pred, int qp, int lambda,
                                   int is_keyframe, const mobi_quant_t *q )
{
    int16x8_t res[8], r[8];
    int32x4_t acc = vdupq_n_s32( 0 );
    for( int i = 0; i < 8; i++ )
    {
        res[i] = vreinterpretq_s16_u16( vsubl_u8( vld1_u8( src + 8*i ), vld1_u8( pred + 8*i ) ) );
        acc = vmlal_s16( acc, vget_low_s16( res[i] ), vget_low_s16( res[i] ) );
        acc = vmlal_high_s16( acc, res[i], res[i] );
        r[i] = res[i];
    }
    const int initial_sse = vaddvq_s32( acc );
    const int threshold = (qp >= 0 && qp < 52) ? (int)mods_sad_thresh_8x8[qp] : 3;
    if( initial_sse < threshold ) return initial_sse;

    /* forward transform: down the columns, then along the rows */
    mobi_transform_8pt_neon( r );
    mobi_transpose8_neon( r );
    mobi_transform_8pt_neon( r );
    mobi_transpose8_neon( r );

    const int32x4_t offset = vdupq_n_s32( q->fwd_offset ), neg_shift = vdupq_n_s32( -q->fwd_shift );
    ALIGNED_ARRAY_16( int16_t, levels, [64] );
    int16x8_t any = vdupq_n_s16( 0 );
    for( int i = 0; i < 8; i++ )
    {
        r[i] = mobi_quant8_neon( r[i], vld1q_u16( q->mf + 8*i ), offset, neg_shift );
        any = vorrq_s16( any, r[i] );
        vst1q_s16( levels + 8*i, r[i] );
    }
    if( !vmaxvq_u16( vreinterpretq_u16_s16( any ) ) )
        return initial_sse;
    uint8x16x4_t coded;
    uint64_t nz[4];
    for( int i = 0; i < 4; i++ )
        coded.val[i] = vcombine_u8( vmovn_u16( vtstq_s16( r[2*i], r[2*i] ) ), vmovn_u16( vtstq_s16( r[2*i+1], r[2*i+1] ) ) );
    for( int i = 0; i < 4; i++ )
        nz[i] = mobi_nibble_mask_neon( vqtbl4q_u8( coded, vld1q_u8( mobi_scan8_u8 + 16*i ) ) );
    const int vlc_bits = mobi_count_vlc_bits_neon( levels, mobi_scan8_u8, nz, 4, is_keyframe );

    /* dequantize and reconstruct: columns, then rows with the final >> 6 */
    const int16x8_t inv_shift = vdupq_n_s16( q->inv_shift );
    for( int i = 0; i < 8; i++ )
        r[i] = vshlq_s16( vmulq_s16( r[i], vld1q_s16( q->iq + 8*i ) ), inv_shift );
    r[0] = vaddq_s16( r[0], vsetq_lane_s16( 32, vdupq_n_s16( 0 ), 0 ) );
    mobi_inv_transform_8pt_neon( r );
    mobi_transpose8_neon( r );
    mobi_inv_transform_8pt_neon( r );
    for( int i = 0; i < 8; i++ )
        r[i] = vshrq_n_s16( r[i], 6 );
    mobi_transpose8_neon( r );

    int32x4_t dist = vdupq_n_s32( 0 );
    for( int i = 0; i < 8; i++ )
    {
        const int32x4_t d0 = vsubl_s16( vget_low_s16( res[i] ), vget_low_s16( r[i] ) );
        const int32x4_t d1 = vsubl_high_s16( res[i], r[i] );
        dist = vmlaq_s32( dist, d0, d0 );
        dist = vmlaq_s32( dist, d1, d1 );
    }
    const int total_cost = vaddvq_s32( dist ) + lambda * vlc_bits;
    return total_cost < initial_sse ? total_cost : initial_sse;
}
#endif

int mobi_rescore_4x4( const uint8_t *src, const uint8_t *pred, int stride,
                       int qp, int lambda, int is_keyframe )
{
    mobi_quant_t local;
    const mobi_quant_t *q = &mobi_quant4[qp & (MOBI_QP_COUNT - 1)];
    if( qp < 0 || qp >= MOBI_QP_COUNT || !q->fwd_shift )
    {
        mobi_build_quant_4x4( qp, local.mf, local.iq, &local.fwd_offset, &local.fwd_shift, &local.inv_shift );
        q = &local;
    }
#if MOBI_NEON
    if( stride == 4 && q != &local )
        return mobi_sad_rate_4x4_neon( src, pred, qp, lambda, is_keyframe, q );
#endif
    uint8_t workspace[0x60];
    int32_t flag = 0;
    return mobi_sad_rate_4x4_core( src, pred, stride, workspace, qp, lambda,
                                    is_keyframe, q->mf, q->fwd_offset, q->fwd_shift,
                                    q->iq, q->inv_shift, 1, &flag );
}
int mobi_rescore_8x8( const uint8_t *src, const uint8_t *pred, int stride,
                       int qp, int lambda, int is_keyframe )
{
    mobi_quant_t local;
    const mobi_quant_t *q = &mobi_quant8[qp & (MOBI_QP_COUNT - 1)];
    if( qp < 0 || qp >= MOBI_QP_COUNT || !q->fwd_shift )
    {
        mobi_build_quant_8x8( qp, local.mf, local.iq, &local.fwd_offset, &local.fwd_shift, &local.inv_shift );
        q = &local;
    }
#if MOBI_NEON
    if( stride == 8 && q != &local && q->inv_shift >= 0 )
        return mobi_sad_rate_8x8_neon( src, pred, qp, lambda, is_keyframe, q );
#endif
    uint8_t workspace[0x180];
    int32_t flag = 0;
    return mobi_sad_rate_8x8_core( src, pred, stride, workspace, qp, lambda,
                                    is_keyframe, q->mf, q->fwd_offset, q->fwd_shift,
                                    q->iq, q->inv_shift, 1, &flag );
}

int mobi_sad_rate_8x8_core(
    const uint8_t *src, const uint8_t *pred, int stride, uint8_t *workspace,
    int qp, int lambda, int is_keyframe, const uint16_t *mf, int fwd_offset,
    int fwd_shift, const int16_t *iq, int inv_shift, int sub_idx, int32_t *flag_ptr )
{
    int16_t *residual = (int16_t *)(workspace + 0x00);
    int16_t *levels = (int16_t *)(workspace + 0x80);
    int16_t *reconstructed = (int16_t *)(workspace + 0x100);
    int initial_sse = 0;
    for( int y = 0; y < 8; y++ )
        for( int x = 0; x < 8; x++ )
        {
            int d = (int)src[y * stride + x] - (int)pred[y * stride + x];
            residual[y * 8 + x] = (int16_t)d;
            initial_sse += d * d;
        }
    int threshold = (qp >= 0 && qp < 52) ? (int)mods_sad_thresh_8x8[qp] : 3;
    if( initial_sse < threshold ) return initial_sse;
    mobi_forward_transform_8x8( residual, levels, 8 );
    if( !mobi_forward_quantize( levels, levels, mf, fwd_offset, fwd_shift ) )
        return initial_sse;
    int vlc_bits = mobi_count_vlc_bits_8x8( levels, is_keyframe );
    mobi_reconstruct_block( levels, iq, inv_shift, reconstructed );
    int distortion = 0;
    for( int i = 0; i < 64; i++ )
    {
        int d = residual[i] - reconstructed[i];
        distortion += d * d;
    }
    int total_cost = distortion + lambda * vlc_bits;
    if( total_cost < initial_sse )
    {
        if( flag_ptr ) *flag_ptr |= sub_idx;
        return total_cost;
    }
    return initial_sse;
}
