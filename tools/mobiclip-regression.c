/* Mobiclip parameter and input-layout regressions. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "x264.h"

static int failures;
static char *dump_path;
static int mobiclip = 1;
static int slice_case;
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
    const char *bad_integers[] = { "2147483648", "-2147483649", "4294967297",
                                    "99999999999999999999999999999" };
    for( unsigned i = 0; i < sizeof(bad_integers)/sizeof(*bad_integers); i++ )
    {
        defaults( &p );
        CHECK( x264_param_parse( &p, "threads", bad_integers[i] ) == X264_PARAM_BAD_VALUE,
               "integer option overflow must be rejected" );
    }
    const char *bad_floats[] = { "nan", "-nan", "inf", "-inf", "1e999", "1e100" };
    for( unsigned i = 0; i < sizeof(bad_floats)/sizeof(*bad_floats); i++ )
    {
        defaults( &p );
        CHECK( x264_param_parse( &p, "crf", bad_floats[i] ) == X264_PARAM_BAD_VALUE,
               "non-finite and overflowing floating-point options must be rejected" );
    }
    defaults( &p );
    CHECK( !x264_param_parse( &p, "ratetol", "inf" ),
           "documented infinite rate tolerance must remain supported" );
    const int bad_modes[] = { -1, 3, 2147483647 };
    for( unsigned i = 0; i < sizeof(bad_modes)/sizeof(*bad_modes); i++ )
    {
        defaults( &p );
        p.i_mobiclip = bad_modes[i];
        h = x264_encoder_open( &p );
        CHECK( !h, "reject undefined Mobiclip modes" );
        if( h ) x264_encoder_close( h );
    }
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
    for( int mode = 0; mode <= 2; mode++ )
    {
        defaults( &p );
        p.i_mobiclip = mode;
        p.rc.i_rc_method = X264_RC_CRF;
        p.rc.f_rf_constant = 60;
        p.rc.f_rf_constant_max = 63;
        h = x264_encoder_open( &p );
        CHECK( h, "open encoder at high CRF" );
        if( h )
        {
            x264_encoder_parameters( h, &actual );
            CHECK( actual.rc.f_rf_constant == (mode ? 60 : 51),
                   "CRF ceiling must match the selected bitstream format" );
            CHECK( actual.rc.f_rf_constant_max == (mode ? 63 : 51),
                   "CRF maximum must retain the Mobiclip quantizer range" );
            x264_encoder_close( h );
        }
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
    p.i_mobiclip = mobiclip;
    p.rc.i_qp_constant = qp;
    p.i_keyint_max = keyint;
    p.psz_dump_yuv = dump_path;
    if( slice_case == 1 ) p.i_slice_count = 2;
    if( slice_case == 2 ) p.i_slice_max_mbs = 1;
    if( slice_case == 3 ) p.i_slice_max_size = 100;
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
        CHECK( out.img.i_csp == (mobiclip ? X264_CSP_I420 : X264_CSP_NV12) &&
               out.img.i_plane == (mobiclip ? 3 : 2),
               "reconstruction metadata must match the plane layout" );
        if( dump_path )
        {
            FILE *dump = fopen( dump_path, "rb" );
            CHECK( dump, "open reconstruction dump" );
            if( dump )
            {
                uint8_t row[64];
                int matches = !fseek( dump, (long)f * 64 * 64 * 3 / 2, SEEK_SET );
                for( int pl = 0; pl < 3; pl++ )
                    for( int y = 0; y < (64 >> !!pl); y++ )
                    {
                        int width = 64 >> !!pl;
                        matches &= fread( row, 1, width, dump ) == width &&
                            !memcmp( row, out.img.plane[pl] + y*out.img.i_stride[pl], width );
                    }
                CHECK( matches, "dump must match reconstructed Y, U and V planes" );
                fclose( dump );
            }
        }
        if( keyint == 1 ) CHECK( out.b_keyframe, "keyint=1 must encode every frame as a keyframe" );
        int slices = 0;
        for( int n = 0; n < nnal; n++ )
            if( nals[n].i_type == NAL_SLICE || nals[n].i_type == NAL_SLICE_IDR )
            {
                slices++;
                if( nals[n].i_payload > capacity - used ) { used = -1; goto end; }
                memcpy( output + used, nals[n].p_payload, nals[n].i_payload );
                used += nals[n].i_payload;
            }
        if( mobiclip ) CHECK( slices == 1, "Mobiclip frames must not be split into H.264 slices" );
        else if( slice_case && (slice_case != 3 || f == 0) ) CHECK( slices > 1, "H.264 slice limits must still split frames" );
    }
end:
    x264_picture_clean( &in );
    x264_encoder_close( h );
    return used;
}

/* Inspect actual SEI messages, including the API caller's payload. */
static void test_standard_sei( int threads )
{
    x264_param_t p;
    x264_picture_t in, out;
    x264_nal_t *nals;
    int count, seen[256] = { 0 }, found_payload = 0;
    uint8_t payload[20] = "0123456789abcdefSEI!";
    x264_sei_payload_t extra = { sizeof(payload), 5, payload };
    defaults( &p );
    p.i_mobiclip = 0;
    p.i_threads = threads;
    p.b_sliced_threads = 0;
    p.i_bframe = 0;
    p.i_keyint_max = 5;
    p.b_intra_refresh = 1;
    p.i_frame_packing = 3;
    p.b_pic_struct = 1;
    p.i_nal_hrd = X264_NAL_HRD_CBR;
    p.rc.i_rc_method = X264_RC_ABR;
    p.rc.i_bitrate = p.rc.i_vbv_max_bitrate = 1000;
    p.rc.i_vbv_buffer_size = 1000;
    x264_t *h = x264_encoder_open( &p );
    CHECK( h, "open H.264 SEI encoder" );
    if( !h ) return;
    if( x264_picture_alloc( &in, X264_CSP_I420, 64, 64 ) < 0 )
    {
        CHECK( 0, "allocate SEI input" );
        x264_encoder_close( h );
        return;
    }
    memset( in.img.plane[0], 100, 64 * 64 );
    memset( in.img.plane[1], 128, 32 * 32 );
    memset( in.img.plane[2], 128, 32 * 32 );
    for( int frame = 0; frame < 20 || x264_encoder_delayed_frames( h ); frame++ )
    {
        in.i_pts = frame;
        in.extra_sei.num_payloads = frame == 0;
        in.extra_sei.payloads = &extra;
        int ret = x264_encoder_encode( h, &nals, &count, frame < 20 ? &in : NULL, &out );
        CHECK( ret >= 0, "encode H.264 SEI frame" );
        if( ret < 0 ) break;
        for( int n = 0; n < count; n++ )
        {
            if( nals[n].i_type != 6 ) continue;
            uint8_t rbsp[8192];
            int used = 0, zeros = 0;
            int prefix = nals[n].p_payload[2] == 1 ? 4 : 5;
            for( int j = prefix; j < nals[n].i_payload && used < sizeof(rbsp); j++ )
            {
                int byte = nals[n].p_payload[j];
                if( zeros == 2 && byte == 3 ) { zeros = 0; continue; }
                rbsp[used++] = byte;
                zeros = byte == 0 ? zeros + 1 : 0;
            }
            for( int j = 0; j + 1 < used; )
            {
                int type = 0, size = 0;
                while( j < used && rbsp[j] == 255 ) { type += 255; j++; }
                if( j >= used ) break;
                type += rbsp[j++];
                while( j < used && rbsp[j] == 255 ) { size += 255; j++; }
                if( j >= used ) break;
                size += rbsp[j++];
                if( size > used - j ) break;
                if( type < 256 ) seen[type]++;
                if( type == 5 && size == sizeof(payload) && !memcmp( rbsp + j, payload, size ) )
                    found_payload = 1;
                j += size;
            }
        }
    }
    CHECK( found_payload, "caller-supplied SEI must reach the bitstream" );
    CHECK( seen[5] >= 2, "repeated headers must include encoder identification SEI" );
    CHECK( seen[0], "HRD must include buffering-period SEI" );
    CHECK( seen[1] == 20, "HRD must include picture timing for every frame" );
    CHECK( seen[6], "intra refresh must include recovery-point SEI" );
    CHECK( seen[45], "frame-packing option must emit SEI" );
    x264_picture_clean( &in );
    x264_encoder_close( h );
}

int main( int argc, char **argv )
{
    static uint8_t planar[65536], other[65536];
    if( argc > 1 && !strcmp( argv[1], "parameters" ) )
        test_parameters();
    else if( argc > 1 && !strcmp( argv[1], "sei" ) )
    {
        test_standard_sei( 1 );
        test_standard_sei( 2 );
    }
    else if( argc > 1 && !strcmp( argv[1], "slices" ) )
    {
        for( mobiclip = 1; mobiclip <= 2; mobiclip++ )
        {
            slice_case = 0;
            int size = encode_layout( X264_CSP_I420, 24, 30, planar, sizeof(planar) );
            for( slice_case = 1; slice_case <= 3; slice_case++ )
            {
                int len = encode_layout( X264_CSP_I420, 24, 30, other, sizeof(other) );
                CHECK( len == size && len > 0 && !memcmp( planar, other, len ),
                       "Mobiclip slice limits must preserve complete frame output" );
            }
        }
        mobiclip = 0;
        for( slice_case = 1; slice_case <= 3; slice_case++ )
            CHECK( encode_layout( X264_CSP_I420, 24, 30, other, sizeof(other) ) > 0,
                   "encode H.264 with slice limits" );
    }
    else if( argc > 1 && !strcmp( argv[1], "standard" ) )
    {
        mobiclip = 0;
        CHECK( encode_layout( X264_CSP_I420, 24, 30, planar, sizeof(planar) ) > 0,
               "Mobiclip debug options must not break standard H.264 encoding" );
    }
    else if( argc > 2 && !strcmp( argv[1], "reconstruction" ) )
    {
        dump_path = argv[2];
        CHECK( encode_layout( X264_CSP_I420, 24, 30, planar, sizeof(planar) ) > 0,
               "encode sequence with reconstruction dump" );
    }
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
