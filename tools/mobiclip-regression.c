/* Mobiclip parameter and input-layout regressions. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "x264.h"

static int failures;
#define CHECK(condition, message) do { if( !(condition) ) { \
    fprintf( stderr, "FAIL: %s\n", message ); failures++; } } while( 0 )

static void defaults( x264_param_t *p )
{
    x264_param_default_preset( p, "medium", "zerolatency" );
    p->i_width = p->i_height = 64;
    p->i_csp = X264_CSP_I420;
    p->i_bitdepth = 8;
    p->i_mobiclip = 1;
    p->b_moflex = 1;
    p->i_log_level = X264_LOG_NONE;
    p->rc.i_rc_method = X264_RC_CQP;
    p->rc.i_qp_constant = 24;
    p->i_keyint_max = 30;
    p->i_scenecut_threshold = 0;
}

static void test_parameters( void )
{
    x264_param_t p, actual;
    x264_t *h;
    defaults( &p );
    p.i_keyint_max = 1;
    h = x264_encoder_open( &p );
    CHECK( h, "open all-intra encoder" );
    if( h )
    {
        x264_encoder_parameters( h, &actual );
        CHECK( actual.i_keyint_max == 1, "explicit keyint=1 must be preserved" );
        x264_encoder_close( h );
    }
    defaults( &p );
    p.rc.i_qp_constant = 0;
    h = x264_encoder_open( &p );
    CHECK( h, "open encoder at minimum quantizer" );
    if( h )
    {
        x264_encoder_parameters( h, &actual );
        CHECK( actual.rc.i_qp_constant >= 12, "Mobiclip cannot use lossless QP 0" );
        CHECK( actual.rc.i_qp_min <= actual.rc.i_qp_max, "QP bounds must not be inverted" );
        x264_encoder_close( h );
    }
    defaults( &p );
    p.rc.i_rc_method = X264_RC_CRF;
    p.rc.f_rf_constant = 0;
    h = x264_encoder_open( &p );
    CHECK( h, "open encoder at minimum CRF" );
    if( h )
    {
        x264_encoder_parameters( h, &actual );
        CHECK( actual.rc.i_rc_method == X264_RC_CRF && actual.rc.f_rf_constant >= 12,
               "CRF 0 must not activate H.264 lossless mode" );
        x264_encoder_close( h );
    }
    defaults( &p );
    p.rc.i_rc_method = X264_RC_ABR;
    p.rc.i_bitrate = 100;
    p.rc.i_qp_max = 5;
    h = x264_encoder_open( &p );
    CHECK( h, "open bitrate encoder with low QP ceiling" );
    if( h )
    {
        x264_encoder_parameters( h, &actual );
        CHECK( actual.rc.i_qp_min >= 12 && actual.rc.i_qp_min <= actual.rc.i_qp_max,
               "bitrate mode must also maintain valid QP bounds" );
        x264_encoder_close( h );
    }
    const int unsupported[] = { X264_CSP_I400, X264_CSP_I422, X264_CSP_I444 };
    for( unsigned i = 0; i < sizeof(unsupported)/sizeof(*unsupported); i++ )
    {
        defaults( &p );
        p.i_csp = unsupported[i];
        h = x264_encoder_open( &p );
        CHECK( !h, "reject unsupported Mobiclip chroma formats" );
        if( h ) x264_encoder_close( h );
    }
    defaults( &p );
    p.i_bitdepth = 10;
    p.i_csp |= X264_CSP_HIGH_DEPTH;
    h = x264_encoder_open( &p );
    CHECK( !h, "reject unsupported 10-bit Mobiclip" );
    if( h ) x264_encoder_close( h );
}

/* Encode identical samples in each layout and compare the actual slice bytes. */
static int encode_layout( int csp, int qp, int keyint, uint8_t *output, int capacity )
{
    x264_param_t p;
    x264_picture_t in, out;
    x264_nal_t *nals;
    int nnal, used = 0;
    defaults( &p );
    p.i_csp = csp;
    p.rc.i_qp_constant = qp;
    p.i_keyint_max = keyint;
    x264_t *h = x264_encoder_open( &p );
    if( !h ) return -1;
    if( x264_picture_alloc( &in, csp, 64, 64 ) < 0 )
    {
        x264_encoder_close( h );
        return -1;
    }
    for( int f = 0; f < 3; f++ )
    {
        for( int y = 0; y < 64; y++ )
            for( int x = 0; x < 64; x++ )
                in.img.plane[0][y * in.img.i_stride[0] + x] = 32 + (x + y + f) % 160;
        for( int y = 0; y < 32; y++ )
            for( int x = 0; x < 32; x++ )
            {
                uint8_t u = 48 + (x + f) % 32, v = 160 + (y + f) % 32;
                if( csp == X264_CSP_NV12 || csp == X264_CSP_NV21 )
                {
                    int swap = csp == X264_CSP_NV21;
                    in.img.plane[1][y * in.img.i_stride[1] + 2*x + swap] = u;
                    in.img.plane[1][y * in.img.i_stride[1] + 2*x + !swap] = v;
                }
                else
                {
                    int swap = csp == X264_CSP_YV12;
                    in.img.plane[1+swap][y * in.img.i_stride[1+swap] + x] = u;
                    in.img.plane[2-swap][y * in.img.i_stride[2-swap] + x] = v;
                }
            }
        in.i_pts = f;
        int size = x264_encoder_encode( h, &nals, &nnal, &in, &out );
        CHECK( size > 0, "zerolatency encoding must produce a frame" );
        if( size <= 0 ) { used = -1; break; }
        if( keyint == 1 ) CHECK( out.b_keyframe, "keyint=1 must encode every frame as a keyframe" );
        for( int n = 0; n < nnal; n++ )
            if( nals[n].i_type == NAL_SLICE || nals[n].i_type == NAL_SLICE_IDR )
            {
                if( nals[n].i_payload > capacity - used ) { used = -1; goto end; }
                memcpy( output + used, nals[n].p_payload, nals[n].i_payload );
                used += nals[n].i_payload;
            }
    }
end:
    x264_picture_clean( &in );
    x264_encoder_close( h );
    return used;
}

int main( int argc, char **argv )
{
    static uint8_t planar[65536], other[65536];
    if( argc > 1 && !strcmp( argv[1], "parameters" ) )
        test_parameters();
    else
    {
        int size = encode_layout( X264_CSP_I420, 24, 30, planar, sizeof(planar) );
        CHECK( size > 0, "encode planar reference" );
        const int layouts[] = { X264_CSP_YV12, X264_CSP_NV12, X264_CSP_NV21 };
        for( unsigned i = 0; i < sizeof(layouts)/sizeof(*layouts); i++ )
        {
            int len = encode_layout( layouts[i], 24, 30, other, sizeof(other) );
            CHECK( len == size && len > 0 && !memcmp( planar, other, len ),
                   "input layouts must encode identical chroma samples" );
        }
        CHECK( encode_layout( X264_CSP_I420, 24, 1, other, sizeof(other) ) > 0,
               "encode all-intra sequence" );
        size = encode_layout( X264_CSP_I420, 12, 30, planar, sizeof(planar) );
        int len = encode_layout( X264_CSP_I420, 0, 30, other, sizeof(other) );
        CHECK( len == size && len > 0 && !memcmp( planar, other, len ),
               "QP below the format floor must encode like QP 12" );
    }
    return failures != 0;
}
