#ifndef __PCMUTILS_LIB_PRIVATE_H__
#define __PCMUTILS_LIB_PRIVATE_H__

/* ------------------------ *
 *  GLOBAL SETTINGS (GFR):  *
 * ------------------------ */
#define DSE_METADATA_ENABLE          /*!< Enable this to support MPEG/ETSI DVB ancillary data for
                                          encoder assisted downmixing of MPEG-4 AAC and
                                          MPEG-1/2 layer 2 streams.                             */
#define PCE_METADATA_ENABLE          /*!< Enable this to support MPEG matrix mixdown with a
                                          coefficient carried in the PCE.                       */

#define PCM_DMX_MAX_IN_CHANNELS          ( 8 )   /* Neither the maximum number of input nor the maximum number of output channels ... */
#define PCM_DMX_MAX_OUT_CHANNELS         ( 8 )   /* ... must exceed the maximum number of channels that the framework can handle. */

/* ------------------------ *
 *    SPECIFIC SETTINGS:    *
 * ------------------------ */
#define PCM_CHANNEL_EXTENSION_ENABLE             /*!< Allow module to duplicate mono signals or add zero channels to achieve the
                                                      desired number of output channels. */

#define PCM_DMX_DFLT_MAX_OUT_CHANNELS    ( 6 )   /*!< The maximum number of output channels. If the value is greater than 0 the module
                                                      will automatically create a mixdown for all input signals with more channels
                                                      than specified. */
#define PCM_DMX_DFLT_MIN_OUT_CHANNELS    ( 0 )   /*!< The minimum number of output channels. If the value is greater than 0 the module
                                                      will do channel extension automatically for all input signals with less channels
                                                      than specified. */
#define PCM_DMX_MAX_DELAY_FRAMES         ( 1 )   /*!< The maximum delay frames to align the bitstreams payload with the PCM output. */
#define PCM_DMX_DFLT_EXPIRY_FRAME        ( 50 )  /*!< If value is greater than 0 the mixdown coefficients will expire by default after the
                                                      given number of frames. The value 50 corresponds to at least 500ms (FL 960 @ 96kHz) */

typedef struct
{
  UINT   typeFlags;
  /* From DSE */
  UCHAR  cLevIdx;
  UCHAR  sLevIdx;
  UCHAR  dmixIdxA;
  UCHAR  dmixIdxB;
  UCHAR  dmixIdxLfe;
  UCHAR  dmxGainIdx2;
  UCHAR  dmxGainIdx5;
#ifdef PCE_METADATA_ENABLE
  /* From PCE */
  UCHAR  matrixMixdownIdx;
#endif
  /* Attributes: */
  SCHAR  pseudoSurround;               /*!< If set to 1 the signal is pseudo surround compatible. The value 0 tells
                                            that it is not. If the value is -1 the information is not available.  */
  UINT   expiryCount;                  /*!< Counter to monitor the life time of a meta data set. */

} DMX_BS_META_DATA;

/* Dynamic (user) params:
     See the definition of PCMDMX_PARAM for details on the specific fields. */
typedef struct
{
  UINT   expiryFrame;                   /*!< Linked to DMX_BS_DATA_EXPIRY_FRAME       */
  DUAL_CHANNEL_MODE dualChannelMode;    /*!< Linked to DMX_DUAL_CHANNEL_MODE          */
  PSEUDO_SURROUND_MODE pseudoSurrMode;  /*!< Linked to DMX_PSEUDO_SURROUND_MODE       */
  SHORT  numOutChannelsMin;             /*!< Linked to MIN_NUMBER_OF_OUTPUT_CHANNELS  */
  SHORT  numOutChannelsMax;             /*!< Linked to MAX_NUMBER_OF_OUTPUT_CHANNELS  */
  UCHAR  frameDelay;                    /*!< Linked to DMX_BS_DATA_DELAY              */

} PCM_DMX_USER_PARAMS;

/* Modules main data structure: */
struct PCM_DMX_INSTANCE
{
  /* Metadata */
  DMX_BS_META_DATA     bsMetaData[PCM_DMX_MAX_DELAY_FRAMES+1];
  PCM_DMX_USER_PARAMS  userParams;

  UCHAR  applyProcessing;              /*!< Flag to en-/disable modules processing.
                                            The max channel limiting is done independently. */
};

#endif /* __PCMUTILS_LIB_PRIVATE_H__ */