/*****************************************************************************
 * mobi_ratecontrol.c: retail Mobiclip's CQ rate-control policy
 *****************************************************************************
 * Ported from mods_encoder.c's mods_cq_policy_init/mods_cq_pre_encode/
 * mods_cq_force_iframe/mods_cq_output_ready. See mobi_ratecontrol.h for the
 * verification summary.
 *****************************************************************************/
#include "mobi_ratecontrol.h"
#include <math.h>

static float mobi_cq_quantizer_to_target_qp( int32_t quantizer_100 )
{
    static const float binary32_percent = 0.01f;
    return (float)quantizer_100 * binary32_percent;
}

/* fyl2x with y=ln(2) computes ln(2)*log2(x) = ln(x) -- see header comment
 * for how this was caught (an initial log2f() port was off by ~1 QP). */
static float mobi_cq_boosted_target( float base_qp, int32_t boost_percent )
{
    static const float percent_scale = 0.01f;
    static const float one = 1.0f;
    static const float qp_scale = 8.0f;
    float x = (float)boost_percent * percent_scale + one;
    float lnx = logf( x );
    float scaled = lnx * qp_scale;
    return base_qp - scaled;
}

/* Round-half-away-from-zero on (target+adjustment), matching the x87
 * fcomp/C0-flag branch in mods_cq_round_target_adjustment. */
static int32_t mobi_cq_round_target_adjustment( float target, float adjustment )
{
    double sum = (double)adjustment + (double)target;
    double rounded = (sum >= 0.0) ? (sum + 0.5) : (sum - 0.5);
    return (int32_t)rounded;
}

void mobi_cq_policy_init( mobi_cq_policy_t *policy, int32_t quantizer_100,
                           int32_t boost_percent, int32_t threshold_percent,
                           int32_t interval_frames )
{
    policy->interval_frames = interval_frames;
    policy->last_keyframe = 0;
    policy->adjustment = 0.0f;
    policy->target_qp = mobi_cq_quantizer_to_target_qp( quantizer_100 );
    if( policy->target_qp < 12.0f ) policy->target_qp = 12.0f;
    else if( policy->target_qp > 48.0f ) policy->target_qp = 48.0f;
    policy->base_qp = policy->target_qp;
    policy->boost_percent = boost_percent;
    policy->threshold_percent = threshold_percent;
}

static int32_t mobi_cq_pre_encode( mobi_cq_policy_t *policy, int32_t total_frame_count, int *is_keyframe )
{
    int make_key = total_frame_count == 0;
    if( !make_key )
    {
        int32_t elapsed = (int32_t)((uint32_t)total_frame_count - (uint32_t)policy->last_keyframe);
        make_key = elapsed >= policy->interval_frames;
    }
    if( make_key )
    {
        policy->last_keyframe = total_frame_count;
        *is_keyframe = 1;
    }
    float target = policy->base_qp;
    if( *is_keyframe )
    {
        target = mobi_cq_boosted_target( policy->base_qp, policy->boost_percent );
        if( target < 12.0f ) target = 12.0f;
        else if( target > 48.0f ) target = 48.0f;
    }
    policy->target_qp = target;
    int32_t qp = mobi_cq_round_target_adjustment( target, policy->adjustment );
    if( qp < 12 ) qp = 12;
    else if( qp > 48 ) qp = 48;
    return qp;
}

int32_t mobi_cq_decide( mobi_cq_policy_t *policy, int32_t total_frame_count,
                         int32_t intra_cost, int32_t inter_cost, int *is_keyframe )
{
    *is_keyframe = 0;
    int32_t threshold = policy->threshold_percent;
    int32_t lhs = (int32_t)((uint32_t)intra_cost * (uint32_t)(100 - threshold));
    int32_t rhs = (int32_t)((uint32_t)inter_cost * (uint32_t)threshold);
    if( lhs >= rhs )
    {
        policy->last_keyframe = total_frame_count;
        *is_keyframe = 1;
        return mobi_cq_pre_encode( policy, total_frame_count, is_keyframe );
    }
    return mobi_cq_pre_encode( policy, total_frame_count, is_keyframe );
}

void mobi_cq_feedback( mobi_cq_policy_t *policy, int32_t qp_used )
{
    /* asm: fildl qp; fsubrs target (FSUBR reverses operand order: st0 =
     * target - qp); fadds adjustment; fstps adjustment.
     * adjustment_new = adjustment + (target_qp - qp). */
    policy->adjustment = policy->adjustment + (policy->target_qp - (float)qp_used);
}
