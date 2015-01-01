#ifndef __TPDEC_LIB_PRIVATE_H__
#define __TPDEC_LIB_PRIVATE_H__

#include "tpdec_adts.h"

#include "tpdec_adif.h"

#include "tpdec_latm.h"


typedef union {
    STRUCT_ADTS adts;

    CAdifHeader adif;

    CLatmDemux latm;


} transportdec_parser_t;

struct TRANSPORTDEC
{
    TRANSPORT_TYPE transportFmt;     /*!< MPEG4 transportDec type. */

    CSTpCallBacks callbacks;         /*!< Struct holding callback and its data */

    FDK_BITSTREAM bitStream[2]; /* Bitstream reader */
    UCHAR *bsBuffer;                 /* Internal bitstreamd data buffer (unallocated in case of TT_MP4_RAWPACKETS) */

    transportdec_parser_t parser;    /* Format specific parser structs. */

    CSAudioSpecificConfig asc[(1 * 2)]; /* Audio specific config from the last config found. */
    UINT  globalFramePos;            /* Global transport frame reference bit position. */
    UINT  accessUnitAnchor[2];    /* Current access unit start bit position. */
    INT   auLength[2];            /* Length of current access unit. */
    INT   numberOfRawDataBlocks;     /* Current number of raw data blocks contained remaining from the current transport frame. */
    UINT  avgBitRate;                /* Average bit rate used for frame loss estimation. */
    UINT  lastValidBufferFullness;   /* Last valid buffer fullness value for frame loss estimation */
    INT   remainder;                 /* Reminder in division during lost access unit estimation. */
    INT   missingAccessUnits;        /* Estimated missing access units. */
    UINT  burstPeriod;               /* Data burst period in mili seconds. */
    UINT  holdOffFrames;             /* Amount of frames that were already hold off due to buffer fullness condition not being met. */
    UINT  flags;                     /* Flags. */
};

#endif __TPDEC_LIB_PRIVATE_H__