#ifndef _METADATA_MAIN_PRIVATE_H
#define _METADATA_MAIN_PRIVATE_H

#include "metadata_main.h"
#include "metadata_compressor.h"

/*----------------- defines ----------------------*/
#define MAX_DRC_BANDS        (1<<4)
#define MAX_DRC_CHANNELS        (8)
#define MAX_DRC_FRAMELEN   (2*1024)

/*--------------- structure definitions --------------------*/

typedef struct AAC_METADATA
{
    /* MPEG: Dynamic Range Control */
    struct {
        UCHAR                         prog_ref_level_present;
        SCHAR                         prog_ref_level;

        UCHAR                         dyn_rng_sgn[MAX_DRC_BANDS];
        UCHAR                         dyn_rng_ctl[MAX_DRC_BANDS];

        UCHAR                         drc_bands_present;
        UCHAR                         drc_band_incr;
        UCHAR                         drc_band_top[MAX_DRC_BANDS];
        UCHAR                         drc_interpolation_scheme;
        AACENC_METADATA_DRC_PROFILE   drc_profile;
        INT                           drc_TargetRefLevel;    /* used for Limiter */

        /* excluded channels */
        UCHAR                         excluded_chns_present;
        UCHAR                         exclude_mask[2];       /* MAX_NUMBER_CHANNELS/8 */
    } mpegDrc;

    /* ETSI: addtl ancillary data */
    struct {
        /* Heavy Compression */
        UCHAR                         compression_on;        /* flag, if compression value should be written */
        UCHAR                         compression_value;     /* compression value */
        AACENC_METADATA_DRC_PROFILE   comp_profile;
        INT                           comp_TargetRefLevel;   /* used for Limiter */
        INT                           timecode_coarse_status;
        INT                           timecode_fine_status;
    } etsiAncData;

    SCHAR                         centerMixLevel;          /* center downmix level (0...7, according to table) */
    SCHAR                         surroundMixLevel;        /* surround downmix level (0...7, according to table) */
    UCHAR                         WritePCEMixDwnIdx;       /* flag */
    UCHAR                         DmxLvl_On;               /* flag */

    UCHAR                         dolbySurroundMode;

    UCHAR                         metadataMode;            /* indicate meta data mode in current frame (delay line) */

} AAC_METADATA;

struct FDK_METADATA_ENCODER
{
  INT                metadataMode;
  HDRC_COMP          hDrcComp;
  AACENC_MetaData    submittedMetaData;

  INT                nAudioDataDelay;
  INT                nMetaDataDelay;
  INT                nChannels;

  INT_PCM            audioDelayBuffer[MAX_DRC_CHANNELS*MAX_DRC_FRAMELEN];
  int                audioDelayIdx;

  AAC_METADATA       metaDataBuffer[3];
  int                metaDataDelayIdx;

  UCHAR              drcInfoPayload[12];
  UCHAR              drcDsePayload[8];

  INT                matrix_mixdown_idx;
  AACENC_EXT_PAYLOAD exPayload[2];
  INT                nExtensions;

  INT                finalizeMetaData;                   /* Delay switch off by one frame and write default configuration to
                                                            finalize the metadata setup. */
};

#endif /* _METADATA_MAIN_PRIVATE_H */