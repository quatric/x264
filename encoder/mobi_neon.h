/*****************************************************************************
 * mobi_neon.h: NEON building blocks shared by the Mobiclip kernels
 *****************************************************************************
 * Copyright (C) 2026 quatric <quatricsoftware@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *****************************************************************************/

#ifndef X264_ENCODER_MOBI_NEON_H
#define X264_ENCODER_MOBI_NEON_H

#if ARCH_AARCH64 && HAVE_NEON
#include <arm_neon.h>
#define MOBI_NEON 1

/* Every helper here keeps the arithmetic of the scalar code it stands in for:
 * 16-bit lanes where that code stores into int16_t (same wrap-around), 32-bit
 * lanes where it computes in int. */

static ALWAYS_INLINE void mobi_transpose4_neon( int16x4_t r[4] )
{
    const int16x4x2_t ab = vtrn_s16( r[0], r[1] );
    const int16x4x2_t cd = vtrn_s16( r[2], r[3] );
    const int32x2x2_t x = vtrn_s32( vreinterpret_s32_s16( ab.val[0] ), vreinterpret_s32_s16( cd.val[0] ) );
    const int32x2x2_t y = vtrn_s32( vreinterpret_s32_s16( ab.val[1] ), vreinterpret_s32_s16( cd.val[1] ) );
    r[0] = vreinterpret_s16_s32( x.val[0] );
    r[1] = vreinterpret_s16_s32( y.val[0] );
    r[2] = vreinterpret_s16_s32( x.val[1] );
    r[3] = vreinterpret_s16_s32( y.val[1] );
}

static ALWAYS_INLINE void mobi_transpose4_s32_neon( int32x4_t r[4] )
{
    const int64x2_t a0 = vreinterpretq_s64_s32( vtrn1q_s32( r[0], r[1] ) );
    const int64x2_t a1 = vreinterpretq_s64_s32( vtrn2q_s32( r[0], r[1] ) );
    const int64x2_t a2 = vreinterpretq_s64_s32( vtrn1q_s32( r[2], r[3] ) );
    const int64x2_t a3 = vreinterpretq_s64_s32( vtrn2q_s32( r[2], r[3] ) );
    r[0] = vreinterpretq_s32_s64( vtrn1q_s64( a0, a2 ) );
    r[1] = vreinterpretq_s32_s64( vtrn1q_s64( a1, a3 ) );
    r[2] = vreinterpretq_s32_s64( vtrn2q_s64( a0, a2 ) );
    r[3] = vreinterpretq_s32_s64( vtrn2q_s64( a1, a3 ) );
}

static ALWAYS_INLINE void mobi_transpose8_neon( int16x8_t r[8] )
{
    const int16x8_t a0 = vtrn1q_s16( r[0], r[1] ), a1 = vtrn2q_s16( r[0], r[1] );
    const int16x8_t a2 = vtrn1q_s16( r[2], r[3] ), a3 = vtrn2q_s16( r[2], r[3] );
    const int16x8_t a4 = vtrn1q_s16( r[4], r[5] ), a5 = vtrn2q_s16( r[4], r[5] );
    const int16x8_t a6 = vtrn1q_s16( r[6], r[7] ), a7 = vtrn2q_s16( r[6], r[7] );
#define TRN32( n, x, y ) vreinterpretq_s64_s32( vtrn##n##q_s32( vreinterpretq_s32_s16( x ), vreinterpretq_s32_s16( y ) ) )
    const int64x2_t b0 = TRN32( 1, a0, a2 ), b2 = TRN32( 2, a0, a2 );
    const int64x2_t b1 = TRN32( 1, a1, a3 ), b3 = TRN32( 2, a1, a3 );
    const int64x2_t b4 = TRN32( 1, a4, a6 ), b6 = TRN32( 2, a4, a6 );
    const int64x2_t b5 = TRN32( 1, a5, a7 ), b7 = TRN32( 2, a5, a7 );
#undef TRN32
    r[0] = vreinterpretq_s16_s64( vtrn1q_s64( b0, b4 ) );
    r[1] = vreinterpretq_s16_s64( vtrn1q_s64( b1, b5 ) );
    r[2] = vreinterpretq_s16_s64( vtrn1q_s64( b2, b6 ) );
    r[3] = vreinterpretq_s16_s64( vtrn1q_s64( b3, b7 ) );
    r[4] = vreinterpretq_s16_s64( vtrn2q_s64( b0, b4 ) );
    r[5] = vreinterpretq_s16_s64( vtrn2q_s64( b1, b5 ) );
    r[6] = vreinterpretq_s16_s64( vtrn2q_s64( b2, b6 ) );
    r[7] = vreinterpretq_s16_s64( vtrn2q_s64( b3, b7 ) );
}

/* The forward 8-point transform in int16, one transform per lane. */
static ALWAYS_INLINE void mobi_transform_8pt_neon( int16x8_t x[8] )
{
    const int16x8_t s0 = vaddq_s16( x[0], x[7] ), s1 = vaddq_s16( x[1], x[6] );
    const int16x8_t s2 = vaddq_s16( x[2], x[5] ), s3 = vaddq_s16( x[3], x[4] );
    const int16x8_t d0 = vsubq_s16( x[0], x[7] ), d1 = vsubq_s16( x[1], x[6] );
    const int16x8_t d2 = vsubq_s16( x[2], x[5] ), d3 = vsubq_s16( x[3], x[4] );
    const int16x8_t ee0 = vaddq_s16( s0, s3 ), ee1 = vaddq_s16( s1, s2 );
    const int16x8_t eo0 = vsubq_s16( s0, s3 ), eo1 = vsubq_s16( s1, s2 );
    const int16x8_t o0 = vaddq_s16( vaddq_s16( d0, vshrq_n_s16( d0, 1 ) ), vaddq_s16( d1, d2 ) );
    const int16x8_t o1 = vsubq_s16( vsubq_s16( d0, d2 ), vaddq_s16( vshrq_n_s16( d2, 1 ), d3 ) );
    const int16x8_t o2 = vsubq_s16( vaddq_s16( d0, d3 ), vaddq_s16( d1, vshrq_n_s16( d1, 1 ) ) );
    const int16x8_t o3 = vaddq_s16( vsubq_s16( d1, d2 ), vaddq_s16( d3, vshrq_n_s16( d3, 1 ) ) );
    x[0] = vaddq_s16( ee0, ee1 );
    x[2] = vaddq_s16( eo0, vshrq_n_s16( eo1, 1 ) );
    x[4] = vsubq_s16( ee0, ee1 );
    x[6] = vsubq_s16( vshrq_n_s16( eo0, 1 ), eo1 );
    x[1] = vaddq_s16( o0, vshrq_n_s16( o3, 2 ) );
    x[3] = vaddq_s16( o1, vshrq_n_s16( o2, 2 ) );
    x[5] = vsubq_s16( o2, vshrq_n_s16( o1, 2 ) );
    x[7] = vsubq_s16( vshrq_n_s16( o0, 2 ), o3 );
}

/* The forward 4-point transform in int16, one transform per lane. */
static ALWAYS_INLINE void mobi_transform_4pt_neon( int16x4_t r[4] )
{
    const int16x4_t a = vadd_s16( r[0], r[3] ), b = vadd_s16( r[1], r[2] );
    const int16x4_t c = vsub_s16( r[1], r[2] ), d = vsub_s16( r[0], r[3] );
    r[0] = vadd_s16( a, b );
    r[1] = vadd_s16( c, vshl_n_s16( d, 1 ) );
    r[2] = vsub_s16( a, b );
    r[3] = vsub_s16( d, vshl_n_s16( c, 1 ) );
}

/* sign(c) * ((|c| * mf + offset) >> shift) in 32 bits */
static ALWAYS_INLINE int32x4_t mobi_quant_neon( int16x4_t c, uint16x4_t mf, int32x4_t offset, int32x4_t neg_shift )
{
    const int32x4_t c32  = vmovl_s16( c );
    const int32x4_t sign = vshrq_n_s32( c32, 31 );
    const int32x4_t v    = vshlq_s32( vmlaq_s32( offset, vabsq_s32( c32 ), vreinterpretq_s32_u32( vmovl_u16( mf ) ) ), neg_shift );
    return vsubq_s32( veorq_s32( v, sign ), sign );
}

static ALWAYS_INLINE int16x8_t mobi_quant8_neon( int16x8_t c, uint16x8_t mf, int32x4_t offset, int32x4_t neg_shift )
{
    return vcombine_s16( vmovn_s32( mobi_quant_neon( vget_low_s16( c ),  vget_low_u16( mf ),  offset, neg_shift ) ),
                         vmovn_s32( mobi_quant_neon( vget_high_s16( c ), vget_high_u16( mf ), offset, neg_shift ) ) );
}

/* One nibble per byte of m, in memory order: 0xf where the byte is non-zero. */
static ALWAYS_INLINE uint64_t mobi_nibble_mask_neon( uint8x16_t m )
{
    return vget_lane_u64( vreinterpret_u64_u8( vshrn_n_u16( vreinterpretq_u16_u8( vtstq_u8( m, m ) ), 4 ) ), 0 );
}

#else
#define MOBI_NEON 0
#endif

#endif
