/*****************************************************************************
 * mobi_ratecost.c: retail Mobiclip's SAD-rate RD-cost primitives
 *****************************************************************************
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

int mobi_rescore_4x4( const uint8_t *src, const uint8_t *pred, int stride,
                       int qp, int lambda, int is_keyframe )
{
    uint16_t mf[16]; int16_t iq[16];
    int fwd_offset, fwd_shift, inv_shift;
    mobi_build_quant_4x4( qp, mf, iq, &fwd_offset, &fwd_shift, &inv_shift );
    uint8_t workspace[0x60];
    int32_t flag = 0;
    return mobi_sad_rate_4x4_core( src, pred, stride, workspace, qp, lambda,
                                    is_keyframe, mf, fwd_offset, fwd_shift,
                                    iq, inv_shift, 1, &flag );
}
int mobi_rescore_8x8( const uint8_t *src, const uint8_t *pred, int stride,
                       int qp, int lambda, int is_keyframe )
{
    uint16_t mf[64]; int16_t iq[64];
    int fwd_offset, fwd_shift, inv_shift;
    mobi_build_quant_8x8( qp, mf, iq, &fwd_offset, &fwd_shift, &inv_shift );
    uint8_t workspace[0x180];
    int32_t flag = 0;
    return mobi_sad_rate_8x8_core( src, pred, stride, workspace, qp, lambda,
                                    is_keyframe, mf, fwd_offset, fwd_shift,
                                    iq, inv_shift, 1, &flag );
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
