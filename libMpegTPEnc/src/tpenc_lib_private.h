#ifndef __TPENC_LIB_PRIVATE_H__
#define __TPENC_LIB_PRIVATE_H__

#include "tpenc_adts.h"

#include "tpenc_adif.h"

#include "tpenc_latm.h"

typedef struct {
  int curSubFrame;
  int nSubFrames;
  int prevBits;
} RAWPACKETS_INFO;

struct TRANSPORTENC
{
  CODER_CONFIG config;
  TRANSPORT_TYPE transportFmt;          /*!< MPEG4 transport type. */

  FDK_BITSTREAM bitStream;
  UCHAR *bsBuffer;
  INT bsBufferSize;

  INT pceFrameCounter;                  /*!< Indicates frame period when PCE must be written in raw_data_block.
                                             -1 means not to write a PCE in raw_dat_block. */
  union {
    STRUCT_ADTS adts;

    ADIF_INFO adif;

    LATM_STREAM latm;

    RAWPACKETS_INFO raw;



  } writer;

  CSTpCallBacks callbacks;
};

#endif /* #ifndef __TPENC_LIB_PRIVATE_H__ */
