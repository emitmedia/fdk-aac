#ifndef _METADATA_COMPRESSOR_PRIVATE_H
#define _METADATA_COMPRESSOR_PRIVATE_H

#define MAX_DRC_CHANNELS        (8)          /*!< Max number of audio input channels. */

/*--------------- structure definitions --------------------*/

/**
 *  Structure holds weighting filter filter states.
 */
struct WEIGHTING_STATES {
    FIXP_DBL x1;
    FIXP_DBL x2;
    FIXP_DBL y1;
    FIXP_DBL y2;
};

/**
 *  Dynamic Range Control compressor structure.
 */
struct DRC_COMP {

    FIXP_DBL     maxBoostThr[2];             /*!< Max boost threshold. */
    FIXP_DBL     boostThr[2];                /*!< Boost threshold. */
    FIXP_DBL     earlyCutThr[2];             /*!< Early cut threshold. */
    FIXP_DBL     cutThr[2];                  /*!< Cut threshold. */
    FIXP_DBL     maxCutThr[2];               /*!< Max cut threshold. */

    FIXP_DBL     boostFac[2];                /*!< Precalculated factor for boost compression. */
    FIXP_DBL     earlyCutFac[2];             /*!< Precalculated factor for early cut compression. */
    FIXP_DBL     cutFac[2];                  /*!< Precalculated factor for cut compression. */

    FIXP_DBL     maxBoost[2];                /*!< Maximum boost. */
    FIXP_DBL     maxCut[2];                  /*!< Maximum cut. */
    FIXP_DBL     maxEarlyCut[2];             /*!< Maximum early cut. */

    FIXP_DBL     fastAttack[2];              /*!< Fast attack coefficient. */
    FIXP_DBL     fastDecay[2];               /*!< Fast release coefficient. */
    FIXP_DBL     slowAttack[2];              /*!< Slow attack coefficient. */
    FIXP_DBL     slowDecay[2];               /*!< Slow release coefficient. */
    UINT         holdOff[2];                 /*!< Hold time in blocks. */

    FIXP_DBL     attackThr[2];               /*!< Slow/fast attack threshold. */
    FIXP_DBL     decayThr[2];                /*!< Slow/fast release threshold. */

    DRC_PROFILE  profile[2];                 /*!< DRC profile. */
    INT          blockLength;                /*!< Block length in samples. */
    UINT         sampleRate;                 /*!< Sample rate. */
    CHANNEL_MODE chanConfig;                 /*!< Channel configuration. */

    UCHAR        useWeighting;               /*!< Use weighting filter. */

    UINT         channels;                   /*!< Number of channels. */
    UINT         fullChannels;               /*!< Number of full range channels. */
    INT          channelIdx[9];              /*!< Offsets of interleaved channel samples (L, R, C, LFE, Ls, Rs, S, Ls2, Rs2). */

    FIXP_DBL     smoothLevel[2];             /*!< level smoothing states */
    FIXP_DBL     smoothGain[2];              /*!< gain smoothing states */
    UINT         holdCnt[2];                 /*!< hold counter */

    FIXP_DBL     limGain[2];                 /*!< limiter gain */
    FIXP_DBL     limDecay;                   /*!< limiter decay (linear) */
    FIXP_DBL     prevPeak[2];                /*!< max peak of previous block (stereo/mono)*/

    WEIGHTING_STATES filter[MAX_DRC_CHANNELS]; /*!< array holds weighting filter states */

};

#endif _METADATA_COMPRESSOR_PRIVATE_H