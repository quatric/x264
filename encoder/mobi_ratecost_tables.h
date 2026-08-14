/*****************************************************************************
 * mobi_ratecost_tables.h: retail Mobiclip scan/VLC-cost tables (see
 * mobi_ratecost_tables.c and mobi_ratecost.h)
 *****************************************************************************/
#ifndef X264_MOBI_RATECOST_TABLES_H
#define X264_MOBI_RATECOST_TABLES_H

#include <stdint.h>
#include "common/common.h"

/* Plain data, compiled once per BIT_DEPTH like the rest of SRCS_X -- needs
 * the same x264_template() renaming as mobi_ratecost.h or the 8-bit/10-bit
 * objects collide at link time with duplicate symbols. */
#define mods_scan_8x8       x264_template(mods_scan_8x8)
#define mods_sad_thresh_8x8 x264_template(mods_sad_thresh_8x8)
#define mods_vlc_mid_8x8    x264_template(mods_vlc_mid_8x8)
#define mods_vlc_last_8x8   x264_template(mods_vlc_last_8x8)
#define mods_scan_4x4       x264_template(mods_scan_4x4)
#define mods_sad_thresh_4x4 x264_template(mods_sad_thresh_4x4)

extern const uint32_t mods_scan_8x8[64];
extern const uint32_t mods_sad_thresh_8x8[52];
extern const uint8_t  mods_vlc_mid_8x8[32768];
extern const uint8_t  mods_vlc_last_8x8[32768];
extern const uint32_t mods_scan_4x4[15];
extern const uint32_t mods_sad_thresh_4x4[52];

#endif
