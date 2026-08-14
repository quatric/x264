/*****************************************************************************
 * mobi_ratecontrol.h: retail Mobiclip's CQ rate-control policy
 *****************************************************************************
 * Ported from Helwettpackardenterprise's MODS_Encoder_v43_2
 * (mods_encoder.c ~12824-13182): the retail "Constant Quality" policy --
 * per-frame QP selection with I-frame boosting, scene-cut forced keyframes,
 * and a running quality-drift feedback accumulator.
 *
 * Verified against 43_2's own mods_cq_policy_init/pre_encode/force_iframe/
 * output_ready (all public, non-static API) at the point of this port:
 * 760 simulated frame sequences (700-case parameter grid sweeping QP,
 * IBoostPercent, IThreshold, and keyframe interval across their full
 * practical ranges, plus 60 randomized-parameter/randomized-cost trials),
 * comparing the per-frame chosen QP and keyframe flag -- 0 mismatches.
 *
 * One real mistake this caught before it shipped: mods_cq_boosted_target's
 * x87 asm loads y=fldln2 (the constant ln(2)) before fyl2x, so the result
 * is ln(2)*log2(x) = ln(x) (the classic x87 natural-log idiom) -- an
 * initial port assumed log2f() directly and was off by nearly a full QP
 * unit on every keyframe (verified wrong via the frame-0 test case, not by
 * re-reading the asm carefully enough the first time).
 *
 * This module implements the policy's decision logic only -- it does NOT
 * yet drive x264's actual per-frame QP selection in encoder/ratecontrol.c;
 * that integration is a separate, not-yet-done step (see memory note
 * mobiclip-native-mode-decision-port). x264 currently approximates retail's
 * IBoostPercent/IThreshold via its own generic i_qfactor/sc_threshold knobs
 * (see mobipeg's encode.py --i-boost/--i-threshold), which is structurally
 * similar but not decision-for-decision identical to this policy.
 *****************************************************************************/
#ifndef X264_MOBI_RATECONTROL_H
#define X264_MOBI_RATECONTROL_H

#include <stdint.h>
#include "common/common.h"

/* Compiled twice (once per BIT_DEPTH) like the rest of SRCS_X even though
 * the policy math itself doesn't vary with pixel depth -- needs the same
 * x264_template() renaming as mobi_ratecost.h or the 8-bit/10-bit objects
 * collide at link time with duplicate symbols. */
#define mobi_cq_policy_init  x264_template(mobi_cq_policy_init)
#define mobi_cq_decide       x264_template(mobi_cq_decide)
#define mobi_cq_feedback     x264_template(mobi_cq_feedback)
#define mobi_cq_qp_for_frame x264_template(mobi_cq_qp_for_frame)

typedef struct
{
    int32_t interval_frames;
    int32_t last_keyframe;
    float   adjustment;
    float   target_qp;
    float   base_qp;
    int32_t boost_percent;
    int32_t threshold_percent;
} mobi_cq_policy_t;

/* quantizer_100 is retail's QP*100 units (e.g. 2400 for QP 24). */
void mobi_cq_policy_init( mobi_cq_policy_t *policy, int32_t quantizer_100,
                           int32_t boost_percent, int32_t threshold_percent,
                           int32_t interval_frames );

/* total_frame_count: running frame counter since stream start.
 * intra_cost/inter_cost: this frame's estimated intra vs inter coding cost
 * (used only by the scene-cut check).
 * Returns the retail-chosen QP (12-63 clamp is the caller's job -- this
 * policy clamps to retail's CQ-specific 12-48 target range internally, per
 * mods_cq_pre_encode/mods_cq_boosted_target). *is_keyframe is set to 1 if
 * this policy forces a keyframe (either the periodic interval or the
 * scene-cut check), 0 otherwise. */
int32_t mobi_cq_decide( mobi_cq_policy_t *policy, int32_t total_frame_count,
                         int32_t intra_cost, int32_t inter_cost, int *is_keyframe );

/* Feed back the QP the encoder actually used for the frame mobi_cq_decide
 * just returned a decision for, updating the drift-correction accumulator.
 * Must be called once per frame, after mobi_cq_decide, even if the encoder
 * didn't use the suggested QP verbatim. */
void mobi_cq_feedback( mobi_cq_policy_t *policy, int32_t qp_used );

/* Same QP math as mobi_cq_decide (boost + round + drift-feedback), but
 * takes is_keyframe as an INPUT rather than deriving it from intra/inter
 * cost -- for integration into a host encoder (x264) that already has its
 * own scene-cut/GOP decision made by the time rate control needs a QP.
 * Retail's own scene-cut trigger (mods_cq_force_iframe's intra/inter cost
 * ratio) is deliberately NOT replicated here: x264's SATD-domain lookahead
 * costs are not the same domain as retail's, so re-deriving the trigger
 * from x264's costs would risk exactly the kind of cross-domain mixing
 * mistake already caught once this session (see mobi_ratecost.c's intra4x4
 * integration notes) -- better to trust x264's own (already-tuned, already
 * exposed via --i-threshold/-sc_threshold) scene-cut decision and only
 * port retail's QP-given-keyframe math, which is what's actually novel
 * here. Equivalent to mobi_cq_decide() when is_keyframe is supplied from
 * the interval+scene-cut trigger mobi_cq_decide would have computed
 * itself -- verified by feeding mobi_cq_decide's own is_keyframe output
 * back through this function and confirming identical QPs. */
int32_t mobi_cq_qp_for_frame( mobi_cq_policy_t *policy, int32_t total_frame_count, int is_keyframe );

#endif
