
/* -----------------------------------------------------------------------------------------------------------
Software License for The Fraunhofer FDK AAC Codec Library for Android

© Copyright  1995 - 2013 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
  All rights reserved.

 1.    INTRODUCTION
The Fraunhofer FDK AAC Codec Library for Android ("FDK AAC Codec") is software that implements
the MPEG Advanced Audio Coding ("AAC") encoding and decoding scheme for digital audio.
This FDK AAC Codec software is intended to be used on a wide variety of Android devices.

AAC's HE-AAC and HE-AAC v2 versions are regarded as today's most efficient general perceptual
audio codecs. AAC-ELD is considered the best-performing full-bandwidth communications codec by
independent studies and is widely deployed. AAC has been standardized by ISO and IEC as part
of the MPEG specifications.

Patent licenses for necessary patent claims for the FDK AAC Codec (including those of Fraunhofer)
may be obtained through Via Licensing (www.vialicensing.com) or through the respective patent owners
individually for the purpose of encoding or decoding bit streams in products that are compliant with
the ISO/IEC MPEG audio standards. Please note that most manufacturers of Android devices already license
these patent claims through Via Licensing or directly from the patent owners, and therefore FDK AAC Codec
software may already be covered under those patent licenses when it is used for those licensed purposes only.

Commercially-licensed AAC software libraries, including floating-point versions with enhanced sound quality,
are also available from Fraunhofer. Users are encouraged to check the Fraunhofer website for additional
applications information and documentation.

2.    COPYRIGHT LICENSE

Redistribution and use in source and binary forms, with or without modification, are permitted without
payment of copyright license fees provided that you satisfy the following conditions:

You must retain the complete text of this software license in redistributions of the FDK AAC Codec or
your modifications thereto in source code form.

You must retain the complete text of this software license in the documentation and/or other materials
provided with redistributions of the FDK AAC Codec or your modifications thereto in binary form.
You must make available free of charge copies of the complete source code of the FDK AAC Codec and your
modifications thereto to recipients of copies in binary form.

The name of Fraunhofer may not be used to endorse or promote products derived from this library without
prior written permission.

You may not charge copyright license fees for anyone to use, copy or distribute the FDK AAC Codec
software or your modifications thereto.

Your modified versions of the FDK AAC Codec must carry prominent notices stating that you changed the software
and the date of any change. For modified versions of the FDK AAC Codec, the term
"Fraunhofer FDK AAC Codec Library for Android" must be replaced by the term
"Third-Party Modified Version of the Fraunhofer FDK AAC Codec Library for Android."

3.    NO PATENT LICENSE

NO EXPRESS OR IMPLIED LICENSES TO ANY PATENT CLAIMS, including without limitation the patents of Fraunhofer,
ARE GRANTED BY THIS SOFTWARE LICENSE. Fraunhofer provides no warranty of patent non-infringement with
respect to this software.

You may use this FDK AAC Codec software or modifications thereto only for purposes that are authorized
by appropriate patent licenses.

4.    DISCLAIMER

This FDK AAC Codec software is provided by Fraunhofer on behalf of the copyright holders and contributors
"AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties
of merchantability and fitness for a particular purpose. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages,
including but not limited to procurement of substitute goods or services; loss of use, data, or profits,
or business interruption, however caused and on any theory of liability, whether in contract, strict
liability, or tort (including negligence), arising in any way out of the use of this software, even if
advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Institute for Integrated Circuits IIS
Attention: Audio and Multimedia Departments - FDK AAC LL
Am Wolfsmantel 33
91058 Erlangen, Germany

www.iis.fraunhofer.de/amm
amm-info@iis.fraunhofer.de
----------------------------------------------------------------------------------------------------------- */

/*****************************  MPEG-4 AAC Decoder  **************************

   Author(s):   Manuel Jander
   Description:

******************************************************************************/

#include "aacdecoder_lib.h"

#include "aac_ram.h"
#include "aacdecoder.h"
#include "tpdec_lib.h"
#include "FDK_core.h" /* FDK_tools version info */


 #include "sbrdecoder.h"




#include "conceal.h"

 #include "aacdec_drc.h"



/* Decoder library info */
#define AACDECODER_LIB_VL0 2
#define AACDECODER_LIB_VL1 5
#define AACDECODER_LIB_VL2 10
#define AACDECODER_LIB_TITLE "AAC Decoder Lib"
#define AACDECODER_LIB_BUILD_DATE __DATE__
#define AACDECODER_LIB_BUILD_TIME __TIME__

static AAC_DECODER_ERROR
setConcealMethod ( const HANDLE_AACDECODER  self,
                   const INT                method );


LINKSPEC_CPP AAC_DECODER_ERROR aacDecoder_GetFreeBytes ( const HANDLE_AACDECODER  self, UINT *pFreeBytes){

  /* reset free bytes */
  *pFreeBytes = 0;

  /* check handle */
  if(!self)
    return AAC_DEC_INVALID_HANDLE;

  /* return nr of free bytes */
  HANDLE_FDK_BITSTREAM hBs = transportDec_GetBitstream(self->hInput, 0);
  *pFreeBytes = FDKgetFreeBits(hBs) >> 3;

  /* success */
  return AAC_DEC_OK;
}

/**
 * Config Decoder using a CSAudioSpecificConfig struct.
 */
static
LINKSPEC_CPP AAC_DECODER_ERROR aacDecoder_Config(HANDLE_AACDECODER self, const CSAudioSpecificConfig *pAscStruct)
{
  AAC_DECODER_ERROR err;

  /* Initialize AAC core decoder, and update self->streaminfo */
  err = CAacDecoder_Init(self, pAscStruct);

  return err;
}

LINKSPEC_CPP AAC_DECODER_ERROR aacDecoder_ConfigRaw (
        HANDLE_AACDECODER self,
        UCHAR *conf[],
        const UINT length[] )
{
  AAC_DECODER_ERROR err = AAC_DEC_OK;
  TRANSPORTDEC_ERROR   errTp;
  UINT layer, nrOfLayers = self->nrOfLayers;

  for(layer = 0; layer < nrOfLayers; layer++){
    if(length[layer] > 0){
      errTp = transportDec_OutOfBandConfig(self->hInput, conf[layer], length[layer], layer);
      if (errTp != TRANSPORTDEC_OK) {
        switch (errTp) {
        case TRANSPORTDEC_NEED_TO_RESTART:
          err = AAC_DEC_NEED_TO_RESTART;
          break;
        case TRANSPORTDEC_UNSUPPORTED_FORMAT:
          err = AAC_DEC_UNSUPPORTED_FORMAT;
          break;
        default:
          err = AAC_DEC_UNKNOWN;
          break;
        }
        /* if baselayer is OK we continue decoding */
        if(layer  >= 1){
          self->nrOfLayers = layer;
        }
        break;
      }
    }
  }

  return err;
}



static INT aacDecoder_ConfigCallback(void *handle, const CSAudioSpecificConfig *pAscStruct)
{
  HANDLE_AACDECODER self = (HANDLE_AACDECODER)handle;
  AAC_DECODER_ERROR err = AAC_DEC_OK;
  TRANSPORTDEC_ERROR errTp;

  {
    {
      err = aacDecoder_Config(self, pAscStruct);
    }
  }
  if (err == AAC_DEC_OK) {
    if ( self->flags & (AC_USAC|AC_RSVD50|AC_LD|AC_ELD)
      && CConcealment_GetDelay(&self->concealCommonData) > 0 )
    {
      /* Revert to error concealment method Noise Substitution.
         Because interpolation is not implemented for USAC/RSVD50 or
         the additional delay is unwanted for low delay codecs. */
      setConcealMethod(self, 1);
#ifdef DEBUG
      FDKprintf("  Concealment method was reverted to 1 !\n");
#endif
    }
    errTp = TRANSPORTDEC_OK;
  } else {
    if (IS_INIT_ERROR(err)) {
      errTp = TRANSPORTDEC_UNSUPPORTED_FORMAT;
    } /* Fatal errors */
    else if (err == AAC_DEC_NEED_TO_RESTART) {
      errTp = TRANSPORTDEC_NEED_TO_RESTART;
    } else {
      errTp = TRANSPORTDEC_UNKOWN_ERROR;
    }
  }

  return errTp;
}



LINKSPEC_CPP AAC_DECODER_ERROR
aacDecoder_AncDataInit ( HANDLE_AACDECODER self,
                         UCHAR *buffer,
                         int size )
{
  CAncData *ancData = &self->ancData;

  return CAacDecoder_AncDataInit(ancData, buffer, size);
}


LINKSPEC_CPP AAC_DECODER_ERROR
aacDecoder_AncDataGet ( HANDLE_AACDECODER self,
                        int     index,
                        UCHAR **ptr,
                        int    *size )
{
  CAncData *ancData = &self->ancData;

  return CAacDecoder_AncDataGet(ancData, index, ptr, size);
}


static AAC_DECODER_ERROR
setConcealMethod ( const HANDLE_AACDECODER  self,   /*!< Handle of the decoder instance */
                   const INT                method )
{
  AAC_DECODER_ERROR errorStatus = AAC_DEC_OK;
  CConcealParams  *pConcealData = NULL;
  HANDLE_SBRDECODER hSbrDec = NULL;
  HANDLE_AAC_DRC hDrcInfo = NULL;
  HANDLE_PCM_DOWNMIX hPcmDmx = NULL;
  CConcealmentMethod backupMethod = ConcealMethodNone;
  int backupDelay = 0;
  int bsDelay = 0;

  /* check decoder handle */
  if (self != NULL) {
    pConcealData = &self->concealCommonData;
    hSbrDec = self->hSbrDecoder;
    hDrcInfo = self->hDrcInfo;
    hPcmDmx = self->hPcmUtils;
  }


  /* Get current method/delay */
  backupMethod = CConcealment_GetMethod(pConcealData);
  backupDelay  = CConcealment_GetDelay(pConcealData);

  /* Be sure to set AAC and SBR concealment method simultaneously! */
  errorStatus =
    CConcealment_SetParams(
      pConcealData,
      (int)method,                         // concealMethod
      AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,  // concealFadeOutSlope
      AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,  // concealFadeInSlope
      AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,  // concealMuteRelease
      AACDEC_CONCEAL_PARAM_NOT_SPECIFIED   // concealComfNoiseLevel
    );
  if ( (errorStatus != AAC_DEC_OK)
    && (errorStatus != AAC_DEC_INVALID_HANDLE) ) {
    goto bail;
  }

  /* Get new delay */
  bsDelay = CConcealment_GetDelay(pConcealData);

  {
    SBR_ERROR sbrErr = SBRDEC_OK;

    /* set SBR bitstream delay */
    sbrErr = sbrDecoder_SetParam (
      hSbrDec,
      SBR_SYSTEM_BITSTREAM_DELAY,
      bsDelay
    );

    switch (sbrErr) {
    case SBRDEC_OK:
    case SBRDEC_NOT_INITIALIZED:
      if (self != NULL) {
        /* save the param value and set later
           (when SBR has been initialized) */
        self->sbrParams.bsDelay = bsDelay;
      }
      break;
    default:
      errorStatus = AAC_DEC_SET_PARAM_FAIL;
      goto bail;
    }
  }

  errorStatus =
    aacDecoder_drcSetParam (
      hDrcInfo,
      DRC_BS_DELAY,
      bsDelay
    );
  if ( (errorStatus != AAC_DEC_OK)
    && (errorStatus != AAC_DEC_INVALID_HANDLE) ) {
    goto bail;
  }

  if (errorStatus == AAC_DEC_OK) {
    PCMDMX_ERROR err =
      pcmDmx_SetParam (
        hPcmDmx,
        DMX_BS_DATA_DELAY,
        bsDelay
      );
    switch (err) {
    case PCMDMX_INVALID_HANDLE:
      errorStatus = AAC_DEC_INVALID_HANDLE;
    case PCMDMX_OK:
      break;
    default:
      errorStatus = AAC_DEC_SET_PARAM_FAIL;
      goto bail;
    }
  }


bail:
  if ( (errorStatus != AAC_DEC_OK)
    && (errorStatus != AAC_DEC_INVALID_HANDLE) )
  {
    /* Revert to the initial state */
    CConcealment_SetParams (
        pConcealData,
        (int)backupMethod,
        AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
        AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
        AACDEC_CONCEAL_PARAM_NOT_SPECIFIED,
        AACDEC_CONCEAL_PARAM_NOT_SPECIFIED
      );
    /* Revert SBR bitstream delay */
    sbrDecoder_SetParam (
        hSbrDec,
        SBR_SYSTEM_BITSTREAM_DELAY,
        backupDelay
      );
    /* Revert DRC bitstream delay */
    aacDecoder_drcSetParam (
        hDrcInfo,
        DRC_BS_DELAY,
        backupDelay
      );
    /* Revert PCM mixdown bitstream delay */
    pcmDmx_SetParam (
        hPcmDmx,
        DMX_BS_DATA_DELAY,
        backupDelay
      );
  }

  return errorStatus;
}


LINKSPEC_CPP AAC_DECODER_ERROR
aacDecoder_SetParam ( const HANDLE_AACDECODER  self,   /*!< Handle of the decoder instance */
                      const AACDEC_PARAM       param,  /*!< Parameter to set               */
                      const INT                value)  /*!< Parameter valued               */
{
  AAC_DECODER_ERROR errorStatus = AAC_DEC_OK;
  CConcealParams  *pConcealData = NULL;
  HANDLE_AAC_DRC hDrcInfo = NULL;
  HANDLE_PCM_DOWNMIX hPcmDmx = NULL;
  TDLimiterPtr hPcmTdl = NULL;

  /* check decoder handle */
  if (self != NULL) {
    pConcealData = &self->concealCommonData;
    hDrcInfo = self->hDrcInfo;
    hPcmDmx = self->hPcmUtils;
    hPcmTdl = self->hLimiter;
  } else {
    errorStatus = AAC_DEC_INVALID_HANDLE;
  }

  /* configure the subsystems */
  switch (param)
  {
  case AAC_PCM_OUTPUT_INTERLEAVED:
    if (value < 0 || value > 1) {
      return AAC_DEC_SET_PARAM_FAIL;
    }
    if (self == NULL) {
      return AAC_DEC_INVALID_HANDLE;
    }
    self->outputInterleaved = value;
    break;

  case AAC_PCM_MIN_OUTPUT_CHANNELS:
    if (value < -1 || value > (8)) {
      return AAC_DEC_SET_PARAM_FAIL;
    }
    {
      PCMDMX_ERROR err;

      err = pcmDmx_SetParam (
              hPcmDmx,
              MIN_NUMBER_OF_OUTPUT_CHANNELS,
              value );

      switch (err) {
      case PCMDMX_OK:
        break;
      case PCMDMX_INVALID_HANDLE:
        return AAC_DEC_INVALID_HANDLE;
      default:
        return AAC_DEC_SET_PARAM_FAIL;
      }
    }
    break;

  case AAC_PCM_MAX_OUTPUT_CHANNELS:
    if (value < -1 || value > (8)) {
      return AAC_DEC_SET_PARAM_FAIL;
    }
    {
      PCMDMX_ERROR err;

      err = pcmDmx_SetParam (
              hPcmDmx,
              MAX_NUMBER_OF_OUTPUT_CHANNELS,
              value );

      switch (err) {
      case PCMDMX_OK:
        break;
      case PCMDMX_INVALID_HANDLE:
        return AAC_DEC_INVALID_HANDLE;
      default:
        return AAC_DEC_SET_PARAM_FAIL;
      }
    }
    break;

  case AAC_PCM_DUAL_CHANNEL_OUTPUT_MODE:
    {
      PCMDMX_ERROR err;

      err = pcmDmx_SetParam (
              hPcmDmx,
              DMX_DUAL_CHANNEL_MODE,
              value );

      switch (err) {
      case PCMDMX_OK:
        break;
      case PCMDMX_INVALID_HANDLE:
        return AAC_DEC_INVALID_HANDLE;
      default:
        return AAC_DEC_SET_PARAM_FAIL;
      }
    }
    break;


  case AAC_PCM_LIMITER_ENABLE:
    if (value < -1 || value > 1) {
      return AAC_DEC_SET_PARAM_FAIL;
    }
    if (self == NULL) {
      return AAC_DEC_INVALID_HANDLE;
    }
    self->limiterEnableUser = value;
    break;

  case AAC_PCM_LIMITER_ATTACK_TIME:
    if (value <= 0) {  /* module function converts value to unsigned */
      return AAC_DEC_SET_PARAM_FAIL;
    }
    switch (setLimiterAttack(hPcmTdl, value)) {
    case TDLIMIT_OK:
      break;
    case TDLIMIT_INVALID_HANDLE:
      return AAC_DEC_INVALID_HANDLE;
    case TDLIMIT_INVALID_PARAMETER:
    default:
      return AAC_DEC_SET_PARAM_FAIL;
    }
    break;

  case AAC_PCM_LIMITER_RELEAS_TIME:
    if (value <= 0) {  /* module function converts value to unsigned */
      return AAC_DEC_SET_PARAM_FAIL;
    }
    switch (setLimiterRelease(hPcmTdl, value)) {
    case TDLIMIT_OK:
      break;
    case TDLIMIT_INVALID_HANDLE:
      return AAC_DEC_INVALID_HANDLE;
    case TDLIMIT_INVALID_PARAMETER:
    default:
      return AAC_DEC_SET_PARAM_FAIL;
    }
    break;

  case AAC_PCM_OUTPUT_CHANNEL_MAPPING:
    switch (value) {
      case 0:
        if (self != NULL) {
          self->channelOutputMapping = channelMappingTablePassthrough;
        }
        break;
      case 1:
        if (self != NULL) {
          self->channelOutputMapping = channelMappingTableWAV;
        }
        break;
      default:
        errorStatus = AAC_DEC_SET_PARAM_FAIL;
        break;
    }
    break;


  case AAC_QMF_LOWPOWER:
    if (value < -1 || value > 1) {
      return AAC_DEC_SET_PARAM_FAIL;
    }
    if (self == NULL) {
      return AAC_DEC_INVALID_HANDLE;
    }

    /**
     * Set QMF mode (might be overriden)
     *  0:HQ (complex)
     *  1:LP (partially complex)
     */
    self->qmfModeUser = (QMF_MODE)value;
    break;


  case AAC_DRC_ATTENUATION_FACTOR:
    /* DRC compression factor (where 0 is no and 127 is max compression) */
    errorStatus =
      aacDecoder_drcSetParam (
        hDrcInfo,
        DRC_CUT_SCALE,
        value
      );
    break;

  case AAC_DRC_BOOST_FACTOR:
    /* DRC boost factor (where 0 is no and 127 is max boost) */
    errorStatus =
      aacDecoder_drcSetParam (
        hDrcInfo,
        DRC_BOOST_SCALE,
        value
      );
    break;

  case AAC_DRC_REFERENCE_LEVEL:
    /* DRC reference level quantized in 0.25dB steps using values [0..127] it is '-' for analog scaling */
    errorStatus =
      aacDecoder_drcSetParam (
        hDrcInfo,
        TARGET_REF_LEVEL,
        value
      );
    break;

  case AAC_DRC_HEAVY_COMPRESSION:
    /* Don't need to overwrite cut/boost values */
    errorStatus =
      aacDecoder_drcSetParam (
        hDrcInfo,
        APPLY_HEAVY_COMPRESSION,
        value
      );
    break;


  case AAC_TPDEC_CLEAR_BUFFER:
    transportDec_SetParam(self->hInput, TPDEC_PARAM_RESET, 1);
    self->streamInfo.numLostAccessUnits = 0;
    self->streamInfo.numBadBytes = 0;
    self->streamInfo.numTotalBytes = 0;
    /* aacDecoder_SignalInterruption(self); */
    break;

  case AAC_CONCEAL_METHOD:
    /* Changing the concealment method can introduce additional bitstream delay. And
       that in turn affects sub libraries and modules which makes the whole thing quite
       complex.  So the complete changing routine is packed into a helper function which
       keeps all modules and libs in a consistent state even in the case an error occures. */
    errorStatus = setConcealMethod ( self, value );
    break;

  default:
    return AAC_DEC_SET_PARAM_FAIL;
  }  /* switch(param) */

  return (errorStatus);
}


LINKSPEC_CPP HANDLE_AACDECODER aacDecoder_Open(TRANSPORT_TYPE transportFmt, UINT nrOfLayers)
{
  AAC_DECODER_INSTANCE *aacDec = NULL;
  HANDLE_TRANSPORTDEC pIn;
  int err = 0;

  /* Allocate transport layer struct. */
  pIn = transportDec_Open(transportFmt, TP_FLAG_MPEG4);
  if (pIn == NULL) {
    return NULL;
  }

  transportDec_SetParam(pIn, TPDEC_PARAM_IGNORE_BUFFERFULLNESS, 1);

  /* Allocate AAC decoder core struct. */
  aacDec = CAacDecoder_Open(transportFmt);

  if (aacDec == NULL) {
    transportDec_Close(&pIn);
    goto bail;
  }
  aacDec->hInput = pIn;

  aacDec->nrOfLayers = nrOfLayers;

  aacDec->channelOutputMapping = channelMappingTableWAV;

  /* Register Config Update callback. */
  transportDec_RegisterAscCallback(pIn, aacDecoder_ConfigCallback, (void*)aacDec);

  /* open SBR decoder */
  if ( SBRDEC_OK != sbrDecoder_Open ( &aacDec->hSbrDecoder )) {
    err = -1;
    goto bail;
  }
  aacDec->qmfModeUser = NOT_DEFINED;
  transportDec_RegisterSbrCallback(aacDec->hInput, (cbSbr_t)sbrDecoder_Header, (void*)aacDec->hSbrDecoder);


  pcmDmx_Open( &aacDec->hPcmUtils );
  if (aacDec->hPcmUtils == NULL) {
    err = -1;
    goto bail;
  }

  aacDec->hLimiter = createLimiter(TDL_ATTACK_DEFAULT_MS, TDL_RELEASE_DEFAULT_MS, SAMPLE_MAX, (8), 96000);
  if (NULL == aacDec->hLimiter) {
    err = -1;
    goto bail;
  }
  aacDec->limiterEnableUser = (UCHAR)-1;
  aacDec->limiterEnableCurr = 0;



  /* Assure that all modules have same delay */
  if ( setConcealMethod(aacDec, CConcealment_GetMethod(&aacDec->concealCommonData)) ) {
    err = -1;
    goto bail;
  }

bail:
  if (err == -1) {
    aacDecoder_Close(aacDec);
    aacDec = NULL;
  }
  return aacDec;
}

LINKSPEC_CPP AAC_DECODER_ERROR aacDecoder_Fill(
        HANDLE_AACDECODER   self,
        UCHAR              *pBuffer[],
        const UINT          bufferSize[],
        UINT               *pBytesValid
        )
{
  TRANSPORTDEC_ERROR tpErr;
  /* loop counter for layers; if not TT_MP4_RAWPACKETS used as index for only 
     available layer                                                           */
  INT layer      = 0;
  INT nrOfLayers = self->nrOfLayers;

  {
    for (layer = 0; layer < nrOfLayers; layer++){
      {
        tpErr = transportDec_FillData( self->hInput, pBuffer[layer], bufferSize[layer], &pBytesValid[layer], layer );
        if (tpErr != TRANSPORTDEC_OK) {
          return AAC_DEC_UNKNOWN;  /* Must be an internal error */
        }
      }
    }
  }

  return AAC_DEC_OK;
}


static void aacDecoder_SignalInterruption(HANDLE_AACDECODER self)
{
  CAacDecoder_SignalInterruption(self);

  if ( self->hSbrDecoder != NULL ) {
    sbrDecoder_SetParam(self->hSbrDecoder, SBR_BS_INTERRUPTION, 0);
  }
}

static void aacDecoder_UpdateBitStreamCounters(CStreamInfo *pSi, HANDLE_FDK_BITSTREAM hBs, int nBits, AAC_DECODER_ERROR ErrorStatus)
{
  /* calculate bit difference (amount of bits moved forward) */
  nBits = nBits - FDKgetValidBits(hBs);

  /* Note: The amount of bits consumed might become negative when parsing a
     bit stream with several sub frames, and we find out at the last sub frame
     that the total frame length does not match the sum of sub frame length. 
     If this happens, the transport decoder might want to rewind to the supposed
     ending of the transport frame, and this position might be before the last
     access unit beginning. */

  /* Calc bitrate. */
  if (pSi->frameSize > 0) {
    pSi->bitRate = (nBits * pSi->sampleRate)/pSi->frameSize;
  }

  /* bit/byte counters */
  {
    int nBytes;

    nBytes = nBits>>3;
    pSi->numTotalBytes += nBytes;
    if (IS_OUTPUT_VALID(ErrorStatus)) {
      pSi->numTotalAccessUnits++;
    }
    if (IS_DECODE_ERROR(ErrorStatus)) {
      pSi->numBadBytes += nBytes;
      pSi->numBadAccessUnits++;
    }
  }
}

static INT aacDecoder_EstimateNumberOfLostFrames(HANDLE_AACDECODER self)
{
  INT n;

  transportDec_GetMissingAccessUnitCount( &n, self->hInput);

  return n;
}

LINKSPEC_CPP AAC_DECODER_ERROR aacDecoder_DecodeFrame(
        HANDLE_AACDECODER  self,
        INT_PCM           *pTimeData,
        const INT          timeDataSize,
        const UINT         flags)
{
    AAC_DECODER_ERROR ErrorStatus;
    INT layer;
    INT nBits;
    INT interleaved = self->outputInterleaved;
    HANDLE_FDK_BITSTREAM hBs;
    int fTpInterruption = 0;  /* Transport originated interruption detection. */
    int fTpConceal = 0;       /* Transport originated concealment. */


    if (self == NULL) {
      return AAC_DEC_INVALID_HANDLE;
    }

    if (flags & AACDEC_INTR) {
      self->streamInfo.numLostAccessUnits = 0;
    }

    hBs = transportDec_GetBitstream(self->hInput, 0);

    /* Get current bits position for bitrate calculation. */
    nBits = FDKgetValidBits(hBs);
    if (! (flags & (AACDEC_CONCEAL | AACDEC_FLUSH) ) )
    {
      TRANSPORTDEC_ERROR err;

      for(layer = 0; layer < self->nrOfLayers; layer++)
      {
        err = transportDec_ReadAccessUnit(self->hInput, layer);
        if (err != TRANSPORTDEC_OK) {
          switch (err) {
          case TRANSPORTDEC_NOT_ENOUGH_BITS:
            ErrorStatus = AAC_DEC_NOT_ENOUGH_BITS;
            goto bail;
          case TRANSPORTDEC_SYNC_ERROR:
            self->streamInfo.numLostAccessUnits = aacDecoder_EstimateNumberOfLostFrames(self);
            fTpInterruption = 1;
            break;
          case TRANSPORTDEC_NEED_TO_RESTART:
            ErrorStatus = AAC_DEC_NEED_TO_RESTART;
            goto bail;
          case TRANSPORTDEC_CRC_ERROR:
            fTpConceal = 1;
            break;
          default:
            ErrorStatus = AAC_DEC_UNKNOWN;
            goto bail;
          }
        }
      }
    } else {
      if (self->streamInfo.numLostAccessUnits > 0) {
        self->streamInfo.numLostAccessUnits--;
      }
    }

    /* Signal bit stream interruption to other modules if required. */
    if ( fTpInterruption || (flags & (AACDEC_INTR|AACDEC_CLRHIST)) )
    {
      sbrDecoder_SetParam(self->hSbrDecoder, SBR_CLEAR_HISTORY, (flags&AACDEC_CLRHIST));
      aacDecoder_SignalInterruption(self);
      if ( ! (flags & AACDEC_INTR) ) {
        ErrorStatus = AAC_DEC_TRANSPORT_SYNC_ERROR;
        goto bail;
      }
    }

    /* Empty bit buffer in case of flush request. */
    if (flags & AACDEC_FLUSH)
    {
      transportDec_SetParam(self->hInput, TPDEC_PARAM_RESET, 1);
      self->streamInfo.numLostAccessUnits = 0;
      self->streamInfo.numBadBytes = 0;
      self->streamInfo.numTotalBytes = 0;
    }
    /* Reset the output delay field. The modules will add their figures one after another. */
    self->streamInfo.outputDelay = 0;

    if (self->limiterEnableUser==(UCHAR)-1) {
      /* Enbale limiter for all non-lowdelay AOT's. */
      self->limiterEnableCurr = ( self->flags & (AC_LD|AC_ELD) ) ? 0 : 1;
    }
    else {
      /* Use limiter configuration as requested. */
      self->limiterEnableCurr = self->limiterEnableUser;
    }
    /* reset limiter gain on a per frame basis */
    self->extGain[0] = FL2FXCONST_DBL(1.0f/(float)(1<<TDL_GAIN_SCALING));


    ErrorStatus = CAacDecoder_DecodeFrame(self,
                                          flags | (fTpConceal ? AACDEC_CONCEAL : 0),
                                          pTimeData,
                                          timeDataSize,
                                          interleaved);

    if (!(flags & (AACDEC_CONCEAL|AACDEC_FLUSH))) {
      TRANSPORTDEC_ERROR tpErr;
      tpErr = transportDec_EndAccessUnit(self->hInput);
      if (tpErr != TRANSPORTDEC_OK) {
        self->frameOK = 0;
      }
    }

    /* If the current pTimeData does not contain a valid signal, there nothing else we can do, so bail. */
    if ( ! IS_OUTPUT_VALID(ErrorStatus) ) {
      goto bail;
    }

    {
      /* Export data into streaminfo structure */
      self->streamInfo.sampleRate = self->streamInfo.aacSampleRate;
      self->streamInfo.frameSize  = self->streamInfo.aacSamplesPerFrame;
    }
    self->streamInfo.numChannels = self->streamInfo.aacNumChannels;



    CAacDecoder_SyncQmfMode(self);

/* sbr decoder */

    if (ErrorStatus || (flags & AACDEC_CONCEAL) || self->pAacDecoderStaticChannelInfo[0]->concealmentInfo.concealState > ConcealState_FadeIn)
    {
      self->frameOK = 0;  /* if an error has occured do concealment in the SBR decoder too */
    }

    if (self->sbrEnabled)
    {
      SBR_ERROR sbrError = SBRDEC_OK;
      int chOutMapIdx = ((self->chMapIndex==0) && (self->streamInfo.numChannels<7)) ? self->streamInfo.numChannels : self->chMapIndex;

      /* set params */
      sbrDecoder_SetParam ( self->hSbrDecoder,
                            SBR_SYSTEM_BITSTREAM_DELAY,
                            self->sbrParams.bsDelay);
      sbrDecoder_SetParam ( self->hSbrDecoder,
                            SBR_FLUSH_DATA,
                            (flags & AACDEC_FLUSH) );

      if ( self->streamInfo.aot == AOT_ER_AAC_ELD ) {
        /* Configure QMF */
        sbrDecoder_SetParam ( self->hSbrDecoder,
                              SBR_LD_QMF_TIME_ALIGN,
                              (self->flags & AC_LD_MPS) ? 1 : 0 );
      }

      {
        PCMDMX_ERROR dmxErr;
        INT  maxOutCh = 0;

        dmxErr = pcmDmx_GetParam(self->hPcmUtils, MAX_NUMBER_OF_OUTPUT_CHANNELS, &maxOutCh);
        if ( (dmxErr == PCMDMX_OK) && (maxOutCh == 1) ) {
          /* Disable PS processing if we have to create a mono output signal. */
          self->psPossible = 0;
        }
      }


      /* apply SBR processing */
      sbrError = sbrDecoder_Apply ( self->hSbrDecoder,
                                    pTimeData,
                                   &self->streamInfo.numChannels,
                                   &self->streamInfo.sampleRate,
                                    self->channelOutputMapping[chOutMapIdx],
                                    interleaved,
                                    self->frameOK,
                                   &self->psPossible);


     if (sbrError == SBRDEC_OK) {
       #define UPS_SCALE  2  /* Maximum upsampling factor is 4 (CELP+SBR) */
       FIXP_DBL  upsampleFactor = FL2FXCONST_DBL(1.0f/(1<<UPS_SCALE));

       /* Update data in streaminfo structure. Assume that the SBR upsampling factor is either 1 or 2 */
       self->flags |= AC_SBR_PRESENT;
       if (self->streamInfo.aacSampleRate != self->streamInfo.sampleRate) {
         if (self->streamInfo.frameSize == 768) {
           upsampleFactor = FL2FXCONST_DBL(8.0f/(3<<UPS_SCALE));
         } else {
           upsampleFactor = FL2FXCONST_DBL(2.0f/(1<<UPS_SCALE));
         }
       }
       /* Apply upsampling factor to both the core frame length and the core delay */
       self->streamInfo.frameSize    =       (INT)fMult((FIXP_DBL)self->streamInfo.aacSamplesPerFrame<<UPS_SCALE, upsampleFactor);
       self->streamInfo.outputDelay  = (UINT)(INT)fMult((FIXP_DBL)self->streamInfo.outputDelay<<UPS_SCALE, upsampleFactor);
       self->streamInfo.outputDelay += sbrDecoder_GetDelay( self->hSbrDecoder );

       if (self->psPossible) {
         self->flags |= AC_PS_PRESENT;
         self->channelType[0] = ACT_FRONT;
         self->channelType[1] = ACT_FRONT;
         self->channelIndices[0] = 0;
         self->channelIndices[1] = 1;
       }
     }
   }


    {
    INT pcmLimiterScale = 0;
    PCMDMX_ERROR dmxErr = PCMDMX_OK;
    if ( flags & (AACDEC_INTR | AACDEC_CLRHIST) ) {
      /* delete data from the past (e.g. mixdown coeficients) */
      pcmDmx_Reset( self->hPcmUtils, PCMDMX_RESET_BS_DATA );
    }
    /* do PCM post processing */
    dmxErr = pcmDmx_ApplyFrame (
            self->hPcmUtils,
            pTimeData,
            self->streamInfo.frameSize,
           &self->streamInfo.numChannels,
            interleaved,
            self->channelType,
            self->channelIndices,
            self->channelOutputMapping,
            (self->limiterEnableCurr) ? &pcmLimiterScale : NULL
      );
    if (dmxErr == PCMDMX_INVALID_MODE) {
      /* Announce the framework that the current combination of channel configuration and downmix
       * settings are not know to produce a predictable behavior and thus maybe produce strange output. */
      ErrorStatus = AAC_DEC_DECODE_FRAME_ERROR;
    }

    if ( flags & AACDEC_CLRHIST ) {
      /* Delete the delayed signal. */
      resetLimiter(self->hLimiter);
    }
    if (self->limiterEnableCurr)
    {
      /* Set actual signal parameters */
      setLimiterNChannels(self->hLimiter, self->streamInfo.numChannels);
      setLimiterSampleRate(self->hLimiter, self->streamInfo.sampleRate);

      applyLimiter(
              self->hLimiter,
              pTimeData,
              self->extGain,
             &pcmLimiterScale,
              1,
              self->extGainDelay,
              self->streamInfo.frameSize
              );

      /* Announce the additional limiter output delay */
      self->streamInfo.outputDelay += getLimiterDelay(self->hLimiter);
    }
    }


    /* Signal interruption to take effect in next frame. */
    if ( flags & AACDEC_FLUSH ) {
      aacDecoder_SignalInterruption(self);
    }

    /* Update externally visible copy of flags */
    self->streamInfo.flags = self->flags;

bail:

    /* Update Statistics */
    aacDecoder_UpdateBitStreamCounters(&self->streamInfo, hBs, nBits, ErrorStatus);

    return ErrorStatus;
}

LINKSPEC_CPP void aacDecoder_Close ( HANDLE_AACDECODER self )
{
  if (self == NULL)
    return;


  if (self->hLimiter != NULL) {
    destroyLimiter(self->hLimiter);
  }

  if (self->hPcmUtils != NULL) {
    pcmDmx_Close( &self->hPcmUtils );
  }



  if (self->hSbrDecoder != NULL) {
    sbrDecoder_Close(&self->hSbrDecoder);
  }

  if (self->hInput != NULL) {
    transportDec_Close(&self->hInput);
  }

  CAacDecoder_Close(self);
}


LINKSPEC_CPP CStreamInfo* aacDecoder_GetStreamInfo ( HANDLE_AACDECODER self )
{
  return CAacDecoder_GetStreamInfo(self);
}

LINKSPEC_CPP INT aacDecoder_GetLibInfo ( LIB_INFO *info )
{
  int i;

  if (info == NULL) {
    return -1;
  }

  sbrDecoder_GetLibInfo( info );
  transportDec_GetLibInfo( info );
  FDK_toolsGetLibInfo( info );
  pcmDmx_GetLibInfo( info );

  /* search for next free tab */
  for (i = 0; i < FDK_MODULE_LAST; i++) {
    if (info[i].module_id == FDK_NONE) break;
  }
  if (i == FDK_MODULE_LAST) {
    return -1;
  }
  info += i;

  info->module_id = FDK_AACDEC;
  /* build own library info */
  info->version = LIB_VERSION(AACDECODER_LIB_VL0, AACDECODER_LIB_VL1, AACDECODER_LIB_VL2);
  LIB_VERSION_STRING(info);
  info->build_date = AACDECODER_LIB_BUILD_DATE;
  info->build_time = AACDECODER_LIB_BUILD_TIME;
  info->title = AACDECODER_LIB_TITLE;

  /* Set flags */
  info->flags = 0
      | CAPF_AAC_LC
      | CAPF_AAC_VCB11
      | CAPF_AAC_HCR
      | CAPF_AAC_RVLC
      | CAPF_ER_AAC_LD
      | CAPF_ER_AAC_ELD
      | CAPF_AAC_CONCEALMENT
      | CAPF_AAC_DRC

      | CAPF_AAC_MPEG4


      | CAPF_AAC_1024
      | CAPF_AAC_960

      | CAPF_AAC_512

      | CAPF_AAC_480

      ;
  /* End of flags */

  return 0;
}


#define RB_STATE_PERSISTENCE_EXTENSION
#ifdef RB_STATE_PERSISTENCE_EXTENSION

//#define RB_PRINT_TRACE

#include <cassert>
#include <cstdio>
#include <stdexcept>
#include <stdint.h> // cstdint is tr1 and C++11 only
#include <map>
#ifdef RB_PRINT_TRACE
#include <typeinfo>
#endif
#include <vector>

#include "../../libPCMutils/src/limiter_private.h"
#include "../../libPCMutils/src/pcmutils_lib_private.h"
#include "../../libMpegTPDec/src/tpdec_lib_private.h"
#include "../../libSBRdec/src/sbr_ram.h"


namespace {

    //////////////////////////////////////////////////////////////////////////////////////////////

    struct PersistenceTraversalData {
        enum TraversalType { READ, WRITE };

        TraversalType type;
        FILE *fp;

#ifdef RB_PRINT_TRACE
        std::vector<const char *> traversalStack_;
#endif /* RB_PRINT_TRACE */

        PersistenceTraversalData(TraversalType t, FILE *f)
            : type(t)
            , fp(f) {}

        std::map<intptr_t, size_t> traversedRanges_;

        bool enterTraversal(void *ptr, size_t size, const char *className)
        {
            if (!ptr)
                return false;

#ifdef RB_PRINT_TRACE
            traversalStack_.push_back(className);
#endif /* RB_PRINT_TRACE */

            // prevent the same item from being persisted more than once:

            intptr_t iptr = (intptr_t)ptr;

            if (traversedRanges_.empty()) {
#ifdef RB_PRINT_TRACE
                //fprintf(stderr, "%p %d\n", (void*)ptr, size);
#endif /* RB_PRINT_TRACE */
                return true;
            }
            else {
                std::map<intptr_t, size_t>::iterator i = traversedRanges_.upper_bound(iptr); // returns next key strictly after iptr
                if (i != traversedRanges_.begin())
                    --i; // should give key at or before iptr

                if ((iptr >= i->first) && (iptr < (i->first + (intptr_t)i->second))) {
                    // range has already been persisted
                    return false;
                }
                else {
#ifdef RB_PRINT_TRACE
                    //fprintf(stderr, "%p %d\n", (void*)ptr, size);
#endif /* RB_PRINT_TRACE */
                    return true;
                }
            }
        }

        void leaveTraversal(void *ptr, size_t size)
        {
            intptr_t iptr = (intptr_t)ptr;
            traversedRanges_.insert(std::make_pair(iptr, size));

#ifdef RB_PRINT_TRACE
            traversalStack_.pop_back();
#endif /* RB_PRINT_TRACE */
        }
};

#ifdef RB_PRINT_TRACE
#define ENTER_TRAVERSAL { if (!td.enterTraversal(ptr, sizeof(*ptr), typeid(*this).name())) return; }
#else
#define ENTER_TRAVERSAL { if (!td.enterTraversal(ptr, sizeof(*ptr), "")) return; }
#endif /* RB_PRINT_TRACE */    
    
#define LEAVE_TRAVERSAL { td.leaveTraversal(ptr, sizeof(*ptr)); }

//////////////////////////////////////////////////////////////////////////////////////////////

void writeStruct(const void* ptr, size_t size, FILE *fp)
{
#ifdef RB_PRINT_TRACE
    fprintf(stderr, "writeStruct: %p %d\n", (void*)ptr, size);
#endif /* RB_PRINT_TRACE */

    std::fwrite(&size, sizeof(size_t), 1, fp);
    std::fwrite(ptr, size, 1, fp);
}

void readStruct(void* ptr, size_t size, FILE *fp)
{
#ifdef RB_PRINT_TRACE
    fprintf(stderr, "readStruct: %p %d\n", (void*)ptr, size);
#endif /* RB_PRINT_TRACE */

    size_t persistedStorageSize = 0;
    std::fread(&persistedStorageSize, sizeof(size_t), 1, fp);
    if (persistedStorageSize != size)
        throw std::length_error("ERROR: file storage layout does not match memory format");

    std::fread(ptr, size, 1, fp);
}

void readOrWriteStruct(void *ptr, size_t size, PersistenceTraversalData& td)
{
    switch (td.type) {
    case PersistenceTraversalData::READ:
        readStruct(ptr, size, td.fp);
        break;
    case PersistenceTraversalData::WRITE:
        writeStruct(ptr, size, td.fp);
        break;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////

class SparseStructPersistInfo {
    struct Range { // half open range [begin, end)
        Range(size_t b, size_t e)
            : begin(b)
            , end(e) {}

        size_t begin;
        size_t end;    
    };

    size_t storageSize_;
    std::vector<Range> ranges_; // sparse list of byte ranges to persist

    void write_(const void *ptr, FILE *fp, const PersistenceTraversalData& td)
    {
#ifdef RB_PRINT_TRACE
        fprintf(stderr, "SparseStructPersistInfo::write_: [%s] %p %d\n", td.traversalStack_.back(), (void*)ptr, storageSize_);
#endif /* RB_PRINT_TRACE */

        const uint8_t *p = (const uint8_t*)ptr;
        std::fwrite(&storageSize_, sizeof(size_t), 1, fp);

        size_t n=0;
        for (std::vector<Range>::iterator i = ranges_.begin(); i != ranges_.end(); ++i) {
            std::fwrite(&p[i->begin], (i->end - i->begin), 1, fp);
            n += (i->end - i->begin);
        }
        assert( n == storageSize_ );
    }

    void read_(void *ptr, FILE *fp, const PersistenceTraversalData& td)
    {
#ifdef RB_PRINT_TRACE
        fprintf(stderr, "SparseStructPersistInfo::read_: [%s] %p %d\n", td.traversalStack_.back(), (void*)ptr, storageSize_);
#endif /* RB_PRINT_TRACE */

        uint8_t *p = (uint8_t*)ptr;

        size_t persistedStorageSize = 0;
        std::fread(&persistedStorageSize, sizeof(size_t), 1, fp);
        if (persistedStorageSize != storageSize_)
            throw std::length_error("ERROR: file storage layout does not match memory format");
            
        size_t n=0;
        for (std::vector<Range>::iterator i = ranges_.begin(); i != ranges_.end(); ++i) {
            std::fread(&p[i->begin], (i->end - i->begin), 1, fp);
            n += (i->end - i->begin);
        }
        assert( n == storageSize_ );
    }

public:
    SparseStructPersistInfo(size_t structSize)
        : storageSize_( structSize )
    {
        ranges_.push_back(Range(0, structSize));
    }

    void skipField(size_t fieldOffset, size_t fieldSize)
    {
        // require that splits are ordered
        // split final segment at fieldOffset...
        // keep track of storage size

        if (storageSize_ <= fieldSize)
        assert( storageSize_ > fieldSize ); // > not >= because we assume that we will always write some data
        
        if (fieldOffset < ranges_.back().begin)
        assert( fieldOffset >= ranges_.back().begin ); // fields must be skipped in order. therefore always fall in the trailing segment
        
        if (fieldOffset + fieldSize > ranges_.back().end)
        assert(fieldOffset + fieldSize <= ranges_.back().end); // field-skip overruns end of struct. impossible. 

        if (fieldOffset == ranges_.back().begin) { // skipping field at the start of final segment
        
            ranges_.back().begin += fieldSize;
            assert( ranges_.back().begin <= ranges_.back().end ); 
        
            if (ranges_.back().begin == ranges_.back().end) // skipped final and only field
                ranges_.pop_back();
        
        } else if (fieldOffset + fieldSize == ranges_.back().end) { // skipping field at end of final segment

            ranges_.back().end = fieldOffset;

            assert( ranges_.back().begin != ranges_.back().end ); // we shouldn't be in this if branch if fieldOffset==begin
        
        } else { // split final segment 
            size_t end = ranges_.back().end;
            ranges_.back().end = fieldOffset;
            ranges_.push_back(Range(fieldOffset+fieldSize, end));
        }
        
        storageSize_ -= fieldSize;
    }

protected:
    void readOrWrite_( void *ptr, PersistenceTraversalData& td )
    {
        switch (td.type) {
        case PersistenceTraversalData::READ:
            read_(ptr, td.fp, td);
            break;
        case PersistenceTraversalData::WRITE:
            write_(ptr, td.fp, td);
            break;
        }
    }
};

#define SKIP_FIELD( s, t, m ) skipField( offsetof(s,m), sizeof(t) ) // struct, memberType, member

#define SKIP_ARRAY_FIELD( s, t, m, n ) skipField( offsetof(s,m), sizeof(t)*(n) ) // struct, memberType, member, array elem count

//////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
struct ContiguousStructPersistInfo_T { // used for structs with no ptrs, where all data should be persisted
    // leafs only read or write. never traverse
    void readOrWrite(T *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWriteStruct(ptr, sizeof(T), td);

        LEAVE_TRAVERSAL
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////
struct FDK_BITBUF_PersistInfo : SparseStructPersistInfo {
    FDK_BITBUF_PersistInfo()
        : SparseStructPersistInfo(sizeof(FDK_BITBUF))
    {
        SKIP_FIELD(FDK_BITBUF, UCHAR*, Buffer);
        SKIP_FIELD(FDK_BITBUF, UINT, bufSize);
        SKIP_FIELD(FDK_BITBUF, UINT, bufBits);
    }

    void traverse(FDK_BITBUF *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        //assert(ptr->ValidBits == 0);
        //REVIEW FIXME: the above assert fails and the bitstream indicates that it has valid bits at the end of the decode

        readOrWrite_(ptr, td);
        
        // The following FDK_BITBUF fields are initialized from TRANSPORTDEC bsBuffer, which is inited from AACENCODER outBuffer
        // at present we don't save any of this.
        /*
        UCHAR *Buffer;
        UINT   bufSize;
        UINT   bufBits; // total number of bits stored in the buffer (bufSize*8)
        */

        LEAVE_TRAVERSAL
    }
};
static FDK_BITBUF_PersistInfo persist_FDK_BITBUF;


struct FDK_BITSTREAM_PersistInfo : SparseStructPersistInfo {
    FDK_BITSTREAM_PersistInfo()
        : SparseStructPersistInfo(sizeof(FDK_BITSTREAM))
    {
        SKIP_FIELD(FDK_BITSTREAM, FDK_BITBUF, hBitBuf);
    }

    void traverse(FDK_BITSTREAM *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        persist_FDK_BITBUF.traverse(&ptr->hBitBuf, td);

        LEAVE_TRAVERSAL
    }
};
static FDK_BITSTREAM_PersistInfo persist_FDK_BITSTREAM;

//////////////////////////////////////////////////////////////////////////////////////////////
// TRANSPORTDEC [ has sub-structs]

struct TRANSPORTDEC_PersistInfo : SparseStructPersistInfo {
    TRANSPORTDEC_PersistInfo()
        : SparseStructPersistInfo(sizeof(TRANSPORTDEC))
    {
        SKIP_FIELD(TRANSPORTDEC, CSTpCallBacks, callbacks);
        SKIP_ARRAY_FIELD(TRANSPORTDEC, FDK_BITSTREAM, bitStream, 2);
        SKIP_FIELD(TRANSPORTDEC, UCHAR*, bsBuffer);
        SKIP_ARRAY_FIELD(TRANSPORTDEC,CSAudioSpecificConfig, asc, 2);
    }

    void traverse(TRANSPORTDEC *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // TRANSPORTDEC:
        // don't persist CSTpCallBacks

        for (int i=0; i < 2; ++i)
            persist_FDK_BITSTREAM.traverse(&ptr->bitStream[i], td);

        // TRANSPORTDEC::parser is flat

        // TRANSPORTDEC::asc CSAudioSpecificConfig is flat. we only have special handling here to capture the file offset.

        if (td.type == PersistenceTraversalData::WRITE) {
             // update CSAudioSpecificConfigOffset stored at start of file
             long int CSAudioSpecificConfigOffset = ftell(td.fp);
             fseek(td.fp, 0, SEEK_SET);
             fwrite(&CSAudioSpecificConfigOffset, sizeof(CSAudioSpecificConfigOffset), 1, td.fp);
             fseek(td.fp, CSAudioSpecificConfigOffset, SEEK_SET);
        }

        for (int i=0; i < 2; ++i)
            readOrWriteStruct(&ptr->asc[i], sizeof(CSAudioSpecificConfig), td);

        if (ptr->bsBuffer) // bitstream buffer: REVIEW: may not need to write the whole buffer
            readOrWriteStruct(ptr->bsBuffer, TRANSPORTDEC_INBUF_SIZE, td);

        LEAVE_TRAVERSAL
    }
};
static TRANSPORTDEC_PersistInfo persist_TRANSPORTDEC;

//////////////////////////////////////////////////////////////////////////////////////////////
// CStreamInfo [has pointers]

struct CStreamInfo_PersistInfo : SparseStructPersistInfo {
    CStreamInfo_PersistInfo()
        : SparseStructPersistInfo(sizeof(CStreamInfo))
    {
        SKIP_FIELD(CStreamInfo, AUDIO_CHANNEL_TYPE*, pChannelType);
        SKIP_FIELD(CStreamInfo, UCHAR*, pChannelIndices);
    }

    void traverse(CStreamInfo *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // CStreamInfo:
        // pChannelType and pChannelIndices are set to static arrays.
        // REVIEW ASSUMPTION Shouldn't need to persist if decoder is correctly configured.

        LEAVE_TRAVERSAL
    }
};
static CStreamInfo_PersistInfo persist_CStreamInfo;

//////////////////////////////////////////////////////////////////////////////////////////////
// CAacDecoderChannelInfo [has pointers]

struct CAacDecoderChannelInfo_PersistInfo : SparseStructPersistInfo {
    CAacDecoderChannelInfo_PersistInfo()
        : SparseStructPersistInfo(sizeof(CAacDecoderChannelInfo))
    {
        SKIP_FIELD(CAacDecoderChannelInfo, SPECTRAL_PTR, pSpectralCoefficient);

        SKIP_FIELD(CAacDecoderChannelInfo, CPnsData, data);

        SKIP_FIELD(CAacDecoderChannelInfo, CAacDecoderDynamicData*, pDynData);
        SKIP_FIELD(CAacDecoderChannelInfo, CAacDecoderCommonData*, pComData);
    }

    void traverse(CAacDecoderChannelInfo *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // CAacDecoderChannelInfo:
        // pSpectralCoefficient and data "can be overwritten after time signal rendering."
        // CIcsInfo is flat
        // CAacDecoderDynamicData *pDynData; only required during decoding
        // CAacDecoderCommonData  *pComData; only required during decoding

        LEAVE_TRAVERSAL
    }
};
static CAacDecoderChannelInfo_PersistInfo persist_CAacDecoderChannelInfo;

//////////////////////////////////////////////////////////////////////////////////////////////
// CConcealmentInfo [has pointers]

static ContiguousStructPersistInfo_T<CConcealParams> persist_CConcealParams;

struct CConcealmentInfo_PersistInfo : SparseStructPersistInfo {
    CConcealmentInfo_PersistInfo()
        : SparseStructPersistInfo(sizeof(CConcealmentInfo))
    {
        SKIP_FIELD(CConcealmentInfo, CConcealParams*, pConcealParams);
    }

    void traverse(CConcealmentInfo *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // CConcealmentInfo:

        persist_CConcealParams.readOrWrite(ptr->pConcealParams, td);

        LEAVE_TRAVERSAL
    }
};
static CConcealmentInfo_PersistInfo persist_CConcealmentInfo;

//////////////////////////////////////////////////////////////////////////////////////////////
// CAacDecoderStaticChannelInfo [has pointers]

// seems easier to scan the array directly than to work out baked parameters for FDKgetWindowSlope.

struct WindowsSlopeParameters {
    WindowsSlopeParameters()
        : shape(0), rasterand(0), length(0) {}
    WindowsSlopeParameters( int s, int r, int l )
        : shape(s), rasterand(r), length(l) {}
    
    int shape;
    int rasterand;
    int length;
};

static WindowsSlopeParameters findWindowSlopeParameters(const FIXP_WTP *windowSlope)
{
    WindowsSlopeParameters result;

    for (int shape=0; shape < 2; ++shape) { // shape&1 is used in FDKgetWindowSlope so we only need to search for 
        for (int rasterand=0; rasterand < 3; ++rasterand) {
            for (int length=0; length < 9; ++length) {
                if (windowSlopes[shape][rasterand][length] == windowSlope)
                    return WindowsSlopeParameters(shape, rasterand, length);
            }
        }
    }

    assert(false); // should have found it
    return result;
}

struct mdct_t_PersistInfo : SparseStructPersistInfo {
    mdct_t_PersistInfo()
        : SparseStructPersistInfo(sizeof(mdct_t))
    {
        SKIP_FIELD(mdct_t, FIXP_DBL*, overlap);
        SKIP_FIELD(mdct_t, FIXP_DBL*, prev_wrs);
    }

    void traverse(mdct_t *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // mdct_t:
        // overlap.freq/ overlap.time point to pOverlapBuffer, which is handled in CAacDecoderStaticChannelInfo_PersistInfo
        
        // prev_wrs points to one of the static lookup tables returned by FDKgetWindowSlope

        if (td.type == PersistenceTraversalData::READ) {
            WindowsSlopeParameters p;
            readOrWriteStruct(&p, sizeof(WindowsSlopeParameters), td);
            ptr->prev_wrs = windowSlopes[p.shape][p.rasterand][p.length];
        } else {
            assert (td.type == PersistenceTraversalData::WRITE);

            WindowsSlopeParameters p = findWindowSlopeParameters(ptr->prev_wrs);
            readOrWriteStruct(&p, sizeof(WindowsSlopeParameters), td);
        }
        
        LEAVE_TRAVERSAL
    }
};
static mdct_t_PersistInfo persist_mdct_t;

struct CAacDecoderStaticChannelInfo_PersistInfo : SparseStructPersistInfo {
    CAacDecoderStaticChannelInfo_PersistInfo()
        : SparseStructPersistInfo(sizeof(CAacDecoderStaticChannelInfo))
    {
        SKIP_FIELD(CAacDecoderStaticChannelInfo, FIXP_DBL*, pOverlapBuffer);
        SKIP_FIELD(CAacDecoderStaticChannelInfo, mdct_t, IMdct);
        SKIP_FIELD(CAacDecoderStaticChannelInfo, CConcealmentInfo, concealmentInfo);
    }

    void traverse(CAacDecoderStaticChannelInfo *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // CAacDecoderStaticChannelInfo:
        readOrWriteStruct(ptr->pOverlapBuffer, sizeof(FIXP_DBL)*OverlapBufferSize, td);
        persist_mdct_t.traverse(&ptr->IMdct, td);
        // CDrcChannelData is flat
        persist_CConcealmentInfo.traverse(&ptr->concealmentInfo, td);

        LEAVE_TRAVERSAL
    }
};
static CAacDecoderStaticChannelInfo_PersistInfo persist_CAacDecoderStaticChannelInfo;

//////////////////////////////////////////////////////////////////////////////////////////////
// CAacDecoderCommonData [has pointers, possibly skip work buffer and overlay]

//"pComData->overlay memory pointed to can be overwritten after each CChannelElement_Decode() call.."

# if 0 // unused
struct CAacDecoderCommonData_PersistInfo : SparseStructPersistInfo {
    CAacDecoderCommonData_PersistInfo()
        : SparseStructPersistInfo(sizeof(CAacDecoderCommonData))
    {
        // SKIP_FIELD(CAacDecoderCommonData, X, Y);
    }

    void traverse(CAacDecoderCommonData *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // CAacDecoderCommonData:
        LEAVE_TRAVERSAL
    }
};
static CAacDecoderCommonData_PersistInfo persist_CAacDecoderCommonData;
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// SBR_DECODER_INSTANCE [ has pointers ]

struct QMF_FILTER_BANK_PersistInfo : SparseStructPersistInfo {
    QMF_FILTER_BANK_PersistInfo()
        : SparseStructPersistInfo(sizeof(QMF_FILTER_BANK))
    {
        SKIP_FIELD(QMF_FILTER_BANK, const FIXP_PFT *, p_filter);
        SKIP_FIELD(QMF_FILTER_BANK, void *, FilterStates);
        SKIP_FIELD(QMF_FILTER_BANK, const FIXP_QTW *, t_cos);
        SKIP_FIELD(QMF_FILTER_BANK, const FIXP_QTW *, t_sin);
    }

    void traverse(QMF_FILTER_BANK *ptr, size_t stateSize, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        int oldNoChannels = ptr->no_channels;

        readOrWrite_(ptr, td);

        assert( ptr->no_channels == oldNoChannels ); // if this fails then we should be saving/restoring the static lookup tables as they are selected based on no_channels

        // QMF_FILTER_BANK:
        // QMF_FILTER_BANK::const FIXP_PFT *p_filter; // static lookup table
        
        // QMF_FILTER_BANK::void *FilterStates;           /*!< Pointer to buffer of filter states
        //                             FIXP_PCM in analyse and
        //                             FIXP_DBL in synthesis filter */

        size_t filterStatesSize = (2*QMF_NO_POLY-1)*ptr->no_channels*stateSize;
        readOrWriteStruct(ptr->FilterStates, filterStatesSize, td);

        // QMF_FILTER_BANK::const FIXP_QTW *t_cos; // static lookup table
        // QMF_FILTER_BANK::const FIXP_QTW *t_sin; // static lookup table

        LEAVE_TRAVERSAL
    }
};
static QMF_FILTER_BANK_PersistInfo persist_QMF_FILTER_BANK;

///

struct SBR_LPP_TRANS_PersistInfo : SparseStructPersistInfo {
    SBR_LPP_TRANS_PersistInfo()
        : SparseStructPersistInfo(sizeof(SBR_LPP_TRANS))
    {
        SKIP_FIELD(SBR_LPP_TRANS, TRANSPOSER_SETTINGS*, pSettings);
    }

    void traverse(SBR_LPP_TRANS *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // SBR_LPP_TRANS:
        // TRANSPOSER_SETTINGS
        readOrWriteStruct(ptr->pSettings, sizeof(TRANSPOSER_SETTINGS), td);

        LEAVE_TRAVERSAL
    }
};
static SBR_LPP_TRANS_PersistInfo persist_SBR_LPP_TRANS;

struct SBR_DEC_PersistInfo : SparseStructPersistInfo {

#define QMF_BUFFER_ITEM_COUNT ((((1024)/(32))+(6)))

    SBR_DEC_PersistInfo()
        : SparseStructPersistInfo(sizeof(SBR_DEC))
    {
        SKIP_FIELD(SBR_DEC, QMF_FILTER_BANK, AnalysiscQMF);
        SKIP_FIELD(SBR_DEC, QMF_FILTER_BANK, SynthesisQMF);
        SKIP_FIELD(SBR_DEC, SBR_LPP_TRANS, LppTrans);
        SKIP_FIELD(SBR_DEC, FIXP_DBL*, pSbrOverlapBuffer);
        SKIP_FIELD(SBR_DEC, FIXP_DBL*, WorkBuffer1);
        SKIP_FIELD(SBR_DEC, FIXP_DBL*, WorkBuffer2);

        SKIP_FIELD(SBR_DEC, FIXP_QSS*, pSynQmfStates);

        SKIP_ARRAY_FIELD(SBR_DEC, FIXP_DBL*, QmfBufferReal, QMF_BUFFER_ITEM_COUNT);
        SKIP_ARRAY_FIELD(SBR_DEC, FIXP_DBL*, QmfBufferImag, QMF_BUFFER_ITEM_COUNT);
    }

// encode which buffer is used in the top bits
#define QMF_BUFFER_SOURCE_MASK 0xC0000000
#define QMF_BUFFER_SOURCE_NONE 0x00000000
#define QMF_BUFFER_SOURCE_WB1  0x40000000
#define QMF_BUFFER_SOURCE_WB2 0x80000000
#define QMF_BUFFER_SOURCE_OVLP 0xC0000000
#define QMF_BUFFER_INDEX_MASK   0x3FFFFFFF

    void computeQmfBufferOffsets(size_t *realOffsets, size_t *imagOffsets, const SBR_DEC *ptr)
    {
        const FIXP_DBL *pSbrOverlapBufferBegin = ptr->pSbrOverlapBuffer;
        const FIXP_DBL *pSbrOverlapBufferEnd = ptr->pSbrOverlapBuffer + (2 * (6) * (64));
        const FIXP_DBL *WorkBuffer1Begin = ptr->WorkBuffer1;
        const FIXP_DBL *WorkBuffer1End = ptr->WorkBuffer1 + (((1024)/(32))*(64));
        const FIXP_DBL *WorkBuffer2Begin = ptr->WorkBuffer2;
        const FIXP_DBL *WorkBuffer2End = ptr->WorkBuffer2 + (((1024)/(32))*(64));

        for (int i = 0; i < QMF_BUFFER_ITEM_COUNT; ++i) {

            // real
            if (ptr->QmfBufferReal[i] >= pSbrOverlapBufferBegin && ptr->QmfBufferReal[i] < pSbrOverlapBufferEnd) {

                realOffsets[i] = (ptr->QmfBufferReal[i] - pSbrOverlapBufferBegin) | QMF_BUFFER_SOURCE_OVLP;

            } else if (ptr->QmfBufferReal[i] >= WorkBuffer1Begin && ptr->QmfBufferReal[i] < WorkBuffer1End) {

                realOffsets[i] = (ptr->QmfBufferReal[i] - WorkBuffer1Begin) | QMF_BUFFER_SOURCE_WB1;

            } else if (ptr->QmfBufferReal[i] >= WorkBuffer2Begin && ptr->QmfBufferReal[i] < WorkBuffer2End) {

                realOffsets[i] = (ptr->QmfBufferReal[i] - WorkBuffer2Begin) | QMF_BUFFER_SOURCE_WB2;

            } else {
                realOffsets[i] = 0;
            }

            // imag
            if (ptr->QmfBufferImag[i] >= pSbrOverlapBufferBegin && ptr->QmfBufferImag[i] < pSbrOverlapBufferEnd) {

                imagOffsets[i] = (ptr->QmfBufferImag[i] - pSbrOverlapBufferBegin) | QMF_BUFFER_SOURCE_OVLP;

            }
            else if (ptr->QmfBufferImag[i] >= WorkBuffer1Begin && ptr->QmfBufferImag[i] < WorkBuffer1End) {

                imagOffsets[i] = (ptr->QmfBufferImag[i] - WorkBuffer1Begin) | QMF_BUFFER_SOURCE_WB1;

            }
            else if (ptr->QmfBufferImag[i] >= WorkBuffer2Begin && ptr->QmfBufferImag[i] < WorkBuffer2End) {

                imagOffsets[i] = (ptr->QmfBufferImag[i] - WorkBuffer2Begin) | QMF_BUFFER_SOURCE_WB2;

            }
            else {
                imagOffsets[i] = 0;
            }
        }
    }

    void restoreQmfBufferPtrs(SBR_DEC *ptr, const size_t *realOffsets, const size_t *imagOffsets)
    {
        for (int i=0; i < QMF_BUFFER_ITEM_COUNT; ++i) {
        
            size_t rei = realOffsets[i];
            FIXP_DBL *reptr = 0;
            switch (rei&QMF_BUFFER_SOURCE_MASK) {
                case QMF_BUFFER_SOURCE_WB1:
                    reptr = ptr->WorkBuffer1 + (rei&QMF_BUFFER_INDEX_MASK);
                    break;
                case QMF_BUFFER_SOURCE_WB2:
                    reptr = ptr->WorkBuffer2 + (rei&QMF_BUFFER_INDEX_MASK);
                    break;
                case QMF_BUFFER_SOURCE_OVLP:
                    reptr = ptr->pSbrOverlapBuffer + (rei&QMF_BUFFER_INDEX_MASK);
                    break;
                // else fallthrough and assign 0
            }
            // note: only real coefficients will be set if using LP/LC mode
            //if (reptr != ptr->QmfBufferReal[i]) {
            //    fprintf(stderr, "xre\n");
            //}
            ptr->QmfBufferReal[i] = reptr;


            size_t imi = imagOffsets[i];
            FIXP_DBL *imptr = 0;
            switch (imi&QMF_BUFFER_SOURCE_MASK) {
            case QMF_BUFFER_SOURCE_WB1:
                imptr = ptr->WorkBuffer1 + (imi&QMF_BUFFER_INDEX_MASK);
                break;
            case QMF_BUFFER_SOURCE_WB2:
                imptr = ptr->WorkBuffer2 + (imi&QMF_BUFFER_INDEX_MASK);
                break;
            case QMF_BUFFER_SOURCE_OVLP:
                imptr = ptr->pSbrOverlapBuffer + (imi&QMF_BUFFER_INDEX_MASK);
                break;
            // else fallthrough and assign 0
            }
            //if (imptr != ptr->QmfBufferImag[i]) {
            //    fprintf(stderr, "xim\n");
            //}
            ptr->QmfBufferImag[i] = imptr;
        }
    }

    void traverse(SBR_DEC *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // SBR_DEC:

        // Note comment in persist_QMF_FILTER_BANK code suggests state sizes differ:
        persist_QMF_FILTER_BANK.traverse(&ptr->AnalysiscQMF, sizeof(FIXP_QAS), td);
        persist_QMF_FILTER_BANK.traverse(&ptr->SynthesisQMF, sizeof(FIXP_QSS), td);
        
        // SBR_CALCULATE_ENVELOPE flat struct
        persist_SBR_LPP_TRANS.traverse(&ptr->LppTrans, td);
        // QMF_SCALE_FACTOR flat struct

        // pSbrOverlapBuffer
        size_t pSbrOverlapBufferSize = sizeof(FIXP_DBL)*(2 * (6) * (64));
        readOrWriteStruct(ptr->pSbrOverlapBuffer, pSbrOverlapBufferSize, td);

        // skip WorkBuffer1, WorkBuffer2

        // pSynQmfStates
        size_t pSynQmfStatesSize = sizeof(FIXP_QSS)*((640)-(64));
        readOrWriteStruct(ptr->pSynQmfStates, pSynQmfStatesSize, td);

        // QmfBufferReal, QmfBufferImag 
        // Not all elements of QmfBufferReal, QmfBufferImag are updated every frame. (omitting persisting them clicks).

        size_t qmfBufferRealOffsets[QMF_BUFFER_ITEM_COUNT];
        size_t qmfBufferImagOffsets[QMF_BUFFER_ITEM_COUNT];

        if (td.type == PersistenceTraversalData::READ) {
            readOrWriteStruct(&qmfBufferRealOffsets, QMF_BUFFER_ITEM_COUNT*sizeof(size_t), td);
            readOrWriteStruct(&qmfBufferImagOffsets, QMF_BUFFER_ITEM_COUNT*sizeof(size_t), td);

            restoreQmfBufferPtrs(ptr, qmfBufferRealOffsets, qmfBufferImagOffsets);
        } else {
            assert(td.type == PersistenceTraversalData::WRITE);
            
            computeQmfBufferOffsets(qmfBufferRealOffsets, qmfBufferImagOffsets, ptr);

            readOrWriteStruct(&qmfBufferRealOffsets, QMF_BUFFER_ITEM_COUNT*sizeof(size_t), td);
            readOrWriteStruct(&qmfBufferImagOffsets, QMF_BUFFER_ITEM_COUNT*sizeof(size_t), td);
        }

        // SBRDEC_DRC_CHANNEL flat struct

        LEAVE_TRAVERSAL
    }
};
static SBR_DEC_PersistInfo persist_SBR_DEC;


struct SBR_CHANNEL_PersistInfo : SparseStructPersistInfo {
    SBR_CHANNEL_PersistInfo()
        : SparseStructPersistInfo(sizeof(SBR_CHANNEL))
    {
        SKIP_FIELD(SBR_CHANNEL, SBR_DEC, SbrDec);
    }

    void traverse(SBR_CHANNEL *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // SBR_CHANNEL:
        // SBR_FRAME_DATA flat struct
        // SBR_PREV_FRAME_DATA flat struct
        // SBR_DEC
        persist_SBR_DEC.traverse(&ptr->SbrDec, td);

        LEAVE_TRAVERSAL
    }
};
static SBR_CHANNEL_PersistInfo persist_SBR_CHANNEL;


struct SBR_DECODER_ELEMENT_PersistInfo : SparseStructPersistInfo {
    SBR_DECODER_ELEMENT_PersistInfo()
        : SparseStructPersistInfo(sizeof(SBR_DECODER_ELEMENT))
    {
        SKIP_ARRAY_FIELD(SBR_DECODER_ELEMENT, SBR_CHANNEL*, pSbrChannel, SBRDEC_MAX_CH_PER_ELEMENT);
        SKIP_FIELD(SBR_DECODER_ELEMENT, HANDLE_FDK_BITSTREAM, hBs);
    }

    void traverse(SBR_DECODER_ELEMENT *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // SBR_DECODER_ELEMENT:
        // SBR_CHANNEL [SBRDEC_MAX_CH_PER_ELEMENT]
        for (int i = 0; i < SBRDEC_MAX_CH_PER_ELEMENT; ++i)
            persist_SBR_CHANNEL.traverse(ptr->pSbrChannel[i], td);

        // TRANSPOSER_SETTINGS is flat
        // HANDLE_FDK_BITSTREAM
        persist_FDK_BITSTREAM.traverse(ptr->hBs, td);

        LEAVE_TRAVERSAL
    }
};
static SBR_DECODER_ELEMENT_PersistInfo persist_SBR_DECODER_ELEMENT;


struct PS_DEC_PersistInfo : SparseStructPersistInfo {
    PS_DEC_PersistInfo()
        : SparseStructPersistInfo(sizeof(PS_DEC))
    {
        SKIP_FIELD(PS_DEC, PS_DEC_COEFFICIENTS, specificTo.mpeg.coef);
    }

    void traverse(PS_DEC *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // PS_DEC:
        // flat struct. 
        // PS_DEC_COEFFICIENTS is marked as "reusable scratch memory" so skip it.

        LEAVE_TRAVERSAL
    }
};
static PS_DEC_PersistInfo persist_PS_DEC;


struct FREQ_BAND_DATA_PersistInfo : SparseStructPersistInfo {
    FREQ_BAND_DATA_PersistInfo()
        : SparseStructPersistInfo(sizeof(FREQ_BAND_DATA))
    {
        SKIP_ARRAY_FIELD(FREQ_BAND_DATA, UCHAR*, freqBandTable, 2);
    }

    void traverse(FREQ_BAND_DATA *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // freqBandTable is just redundant pointers to fields freqBandTableLo and freqBandTableHi

        LEAVE_TRAVERSAL
    }
};
static FREQ_BAND_DATA_PersistInfo persist_FREQ_BAND_DATA;


struct SBR_HEADER_DATA_PersistInfo : SparseStructPersistInfo {
    SBR_HEADER_DATA_PersistInfo()
        : SparseStructPersistInfo(sizeof(SBR_HEADER_DATA))
    {
        SKIP_FIELD(SBR_HEADER_DATA, FREQ_BAND_DATA, freqBandData);
    }

    void traverse(SBR_HEADER_DATA *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // FREQ_BAND_DATA
        persist_FREQ_BAND_DATA.traverse(&ptr->freqBandData, td);

        LEAVE_TRAVERSAL
    }
};
static SBR_HEADER_DATA_PersistInfo persist_SBR_HEADER_DATA;


struct SBR_DECODER_INSTANCE_PersistInfo : SparseStructPersistInfo {
    SBR_DECODER_INSTANCE_PersistInfo()
        : SparseStructPersistInfo(sizeof(SBR_DECODER_INSTANCE))
    {
        SKIP_ARRAY_FIELD(SBR_DECODER_INSTANCE, SBR_DECODER_ELEMENT*, pSbrElement, 8);
        SKIP_ARRAY_FIELD(SBR_DECODER_INSTANCE, SBR_HEADER_DATA, sbrHeader, (8)*((1)+1));
        SKIP_FIELD(SBR_DECODER_INSTANCE, FIXP_DBL*, workBuffer1);
        SKIP_FIELD(SBR_DECODER_INSTANCE, FIXP_DBL*, workBuffer2);
        SKIP_FIELD(SBR_DECODER_INSTANCE, HANDLE_PS_DEC, hParametricStereoDec);
    }

    void traverse(SBR_DECODER_INSTANCE *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);
        
        // FIXME TODO: pSbrElement fails to load unless the element has already been allocated.

        // SBR_DECODER_INSTANCE:
        // SBR_DECODER_ELEMENT [8]
        for (int i=0; i < 8; ++i)
            persist_SBR_DECODER_ELEMENT.traverse(ptr->pSbrElement[i], td);

        for (int i=0; i<8; ++i)
            for (int j=0; j < 2; ++j)
                persist_SBR_HEADER_DATA.traverse(&ptr->sbrHeader[i][j], td);

        // skip workBuffer1, workBuffer2

        // HANDLE_PS_DEC
        persist_PS_DEC.traverse(ptr->hParametricStereoDec, td);

        LEAVE_TRAVERSAL
    }
};
static SBR_DECODER_INSTANCE_PersistInfo persist_SBR_DECODER_INSTANCE;

//////////////////////////////////////////////////////////////////////////////////////////////
// CAncData [contains pointer buffer]

struct CAncData_PersistInfo : SparseStructPersistInfo {
    CAncData_PersistInfo()
        : SparseStructPersistInfo(sizeof(CAncData))
    {
        SKIP_FIELD(CAncData, char*, buffer);
    }

    void traverse(CAncData *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // CAncData:
        // :: buffer
        if (ptr->buffer)
            readOrWriteStruct(ptr->buffer, ptr->bufferSize, td);

        LEAVE_TRAVERSAL
    }
};
static CAncData_PersistInfo persist_CAncData;

//////////////////////////////////////////////////////////////////////////////////////////////
// TDLimiter [contains pointers]

struct TDLimiter_PersistInfo : SparseStructPersistInfo {
    TDLimiter_PersistInfo()
        : SparseStructPersistInfo(sizeof(TDLimiter))
    {
        SKIP_FIELD(TDLimiter, FIXP_DBL*, maxBuf);
        SKIP_FIELD(TDLimiter, FIXP_DBL*, delayBuf);
    }

    void traverse(TDLimiter *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        readOrWrite_(ptr, td);

        // TDLimiter:
        size_t maxBufSize = (ptr->attack + 1) * sizeof(FIXP_DBL);
        readOrWriteStruct(ptr->maxBuf, maxBufSize, td);

        size_t delayBufSize = (ptr->attack * ptr->maxChannels) * sizeof(FIXP_DBL);
        readOrWriteStruct(ptr->delayBuf, delayBufSize, td);

        LEAVE_TRAVERSAL
    }
};
static TDLimiter_PersistInfo persist_TDLimiter;

//////////////////////////////////////////////////////////////////////////////////////////////

struct SamplingRateInfo_PersistInfo : SparseStructPersistInfo {
    SamplingRateInfo_PersistInfo()
        : SparseStructPersistInfo(sizeof(SamplingRateInfo))
    {
        // ASSUMPTION: sample rate doesn't change.
        // FIXME REVIEW: we aren't currently saving the scale factor bands
        SKIP_FIELD(SamplingRateInfo, SHORT*, ScaleFactorBands_Long);
        SKIP_FIELD(SamplingRateInfo, SHORT*, ScaleFactorBands_Short);
    }

    void traverse(SamplingRateInfo *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        UINT oldSamplingRate = ptr->samplingRate;

        readOrWrite_(ptr, td);

        assert( ptr->samplingRate == oldSamplingRate ); // FIXME REVIEW if sample rates don't match then we should have persisted the scale factor bands

        LEAVE_TRAVERSAL
    }
};
static SamplingRateInfo_PersistInfo persist_SamplingRateInfo;

static ContiguousStructPersistInfo_T<CDrcInfo> persist_CDrcInfo;
static ContiguousStructPersistInfo_T<PCM_DMX_INSTANCE> persist_PCM_DMX_INSTANCE;

struct AAC_DECODER_INSTANCE_PersistInfo : SparseStructPersistInfo {
    AAC_DECODER_INSTANCE_PersistInfo()
        : SparseStructPersistInfo(sizeof(AAC_DECODER_INSTANCE))
    {
        SKIP_FIELD(AAC_DECODER_INSTANCE, HANDLE_TRANSPORTDEC, hInput);
        SKIP_FIELD(AAC_DECODER_INSTANCE, SamplingRateInfo, samplingRateInfo);
        SKIP_ARRAY_FIELD(AAC_DECODER_INSTANCE, UCHAR*, channelOutputMapping, 8);
        SKIP_FIELD(AAC_DECODER_INSTANCE, CStreamInfo, streamInfo);
        SKIP_ARRAY_FIELD(AAC_DECODER_INSTANCE, CAacDecoderChannelInfo*, pAacDecoderChannelInfo, 8);
        SKIP_ARRAY_FIELD(AAC_DECODER_INSTANCE, CAacDecoderStaticChannelInfo*, pAacDecoderStaticChannelInfo, 8);
        SKIP_FIELD(AAC_DECODER_INSTANCE, CAacDecoderCommonData, aacCommonData);
        SKIP_FIELD(AAC_DECODER_INSTANCE, HANDLE_SBRDECODER, hSbrDecoder);
        SKIP_FIELD(AAC_DECODER_INSTANCE, HANDLE_AAC_DRC, hDrcInfo);
        SKIP_FIELD(AAC_DECODER_INSTANCE, CAncData, ancData);
        SKIP_FIELD(AAC_DECODER_INSTANCE, HANDLE_PCM_DOWNMIX, hPcmUtils);
        SKIP_FIELD(AAC_DECODER_INSTANCE, TDLimiterPtr, hLimiter);
    }
    
    void traverse(AAC_DECODER_INSTANCE *ptr, PersistenceTraversalData& td)
    {
        ENTER_TRAVERSAL

        // Special handling for configuring the decoder:
        if (td.type == PersistenceTraversalData::READ) {
            // The first thing we do when loading the decoder is reconfigure it 
            // to the correct audio settings using aacDecoder_Config. 
            // The first long int in the file is an offset to the persisted
            // CSAudioSpecificConfig structure.

            long int CSAudioSpecificConfigOffset = 0;
            fread( &CSAudioSpecificConfigOffset, sizeof(CSAudioSpecificConfigOffset), 1, td.fp );
            long int pos = ftell(td.fp);

            // seek to CSAudioSpecificConfig, load it, configure decoder, seek back to where we were.
            fseek(td.fp, CSAudioSpecificConfigOffset, SEEK_SET);

            CSAudioSpecificConfig asc;
            readStruct(&asc, sizeof(CSAudioSpecificConfig), td.fp);

            AAC_DECODER_ERROR errTp = aacDecoder_Config(ptr, &asc);
            assert(errTp == AAC_DEC_OK);

            fseek(td.fp, pos, SEEK_SET);

        } else { // write
            assert(td.type == PersistenceTraversalData::WRITE);

            // Write null CSAudioSpecificConfig offset. This is updated when CSAudioSpecificConfigOffset is persisted.
            long int CSAudioSpecificConfigOffset = 0;
            fwrite( &CSAudioSpecificConfigOffset, sizeof(CSAudioSpecificConfigOffset), 1, td.fp );
        }

        readOrWrite_(ptr, td);

        // AAC_DECODER_INSTANCE:
        // AAC_DECODER_INSTANCE::HANDLE_TRANSPORTDEC -> TRANSPORTDEC
        persist_TRANSPORTDEC.traverse(ptr->hInput, td);

        // AAC_DECODER_INSTANCE::SamplingRateInfo [has pointers]
        // REVIEW SamplingRateInfo is only updated when the stream sampling frequency changes
        // Current code partially persists it and verifies that sample rate hasn't changed.
        persist_SamplingRateInfo.traverse(&ptr->samplingRateInfo, td);

        // AAC_DECODER_INSTANCE :: UCHAR (*channelOutputMapping)[8];
        if (td.type == PersistenceTraversalData::READ) {
            char channelOutputMappingFlag;
            readOrWriteStruct(&channelOutputMappingFlag, 1, td);

            if (channelOutputMappingFlag == 0) {
                ptr->channelOutputMapping = channelMappingTablePassthrough;
            } else {
                assert( channelOutputMappingFlag == 1 );
                ptr->channelOutputMapping = channelMappingTableWAV;
            }
        } else { // write
            assert(td.type == PersistenceTraversalData::WRITE);

            char channelOutputMappingFlag;
            if (ptr->channelOutputMapping == channelMappingTablePassthrough) {
                channelOutputMappingFlag = 0;
            }  else {
                assert(ptr->channelOutputMapping = channelMappingTableWAV);
                channelOutputMappingFlag = 1;
            }

            readOrWriteStruct(&channelOutputMappingFlag, 1, td);
        }

        // AAC_DECODER_INSTANCE::CProgramConfig [flat]

        // AAC_DECODER_INSTANCE::CStreamInfo [has pointers]
        persist_CStreamInfo.traverse(&ptr->streamInfo, td);

        // AAC_DECODER_INSTANCE::CAacDecoderChannelInfo [array of pointers to struct with pointers]
        for (int i = 0; i < 8; ++i) {
            persist_CAacDecoderChannelInfo.traverse(ptr->pAacDecoderChannelInfo[i], td);
        }
        
        // AAC_DECODER_INSTANCE::CAacDecoderStaticChannelInfo [array of pointers to struct with pointers]
        for (int i = 0; i < 8; ++i) {
            persist_CAacDecoderStaticChannelInfo.traverse(ptr->pAacDecoderStaticChannelInfo[i], td);
        }
    
        //// AAC_DECODER_INSTANCE::CAacDecoderCommonData [contains pointers]
        // "Data required for one channel at a time during decode"
        // "pComData->overlay memory pointed to can be overwritten after each CChannelElement_Decode() call.."
        // persist_CAacDecoderCommonData.traverse(&ptr->aacCommonData, td);

        // AAC_DECODER_INSTANCE::CConcealParams [flat]

        // AAC_DECODER_INSTANCE::HANDLE_SBRDECODER -> SBR_DECODER_INSTANCE
        persist_SBR_DECODER_INSTANCE.traverse(ptr->hSbrDecoder, td);

        // AAC_DECODER_INSTANCE::SBR_PARAMS [flat]
        // QMF_MODE [enum]

        // HANDLE_AAC_DRC -> CDrcInfo [CDrcInfo is flat]
        persist_CDrcInfo.readOrWrite(ptr->hDrcInfo, td);

        // AAC_DECODER_INSTANCE::CAncData [contains pointer buffer]
        persist_CAncData.traverse(&ptr->ancData, td);

        // AAC_DECODER_INSTANCE::HANDLE_PCM_DOWNMIX -> PCM_DMX_INSTANCE
        persist_PCM_DMX_INSTANCE.readOrWrite(ptr->hPcmUtils, td);
        
        // TDLimiterPtr -> TDLimiter [contains pointers]
        persist_TDLimiter.traverse(ptr->hLimiter, td);

        LEAVE_TRAVERSAL
    }
};
static AAC_DECODER_INSTANCE_PersistInfo persist_AAC_DECODER_INSTANCE;

//////////////////////////////////////////////////////////////////////////////////////////////

} // end anonymous namespace

void aacDecoder_ExtSaveState(const HANDLE_AACDECODER hAacDecoder, void *fp)
{
#ifdef RB_PRINT_TRACE
    fprintf(stderr, "begin save state\n");
#endif /* RB_PRINT_TRACE */
    PersistenceTraversalData td(PersistenceTraversalData::WRITE, (std::FILE*)fp);
    try {
        persist_AAC_DECODER_INSTANCE.traverse(const_cast<HANDLE_AACDECODER>(hAacDecoder), td);
    }
    catch (std::exception &e) {
        fprintf(stderr, "saving decoder state failed: %s\n", e.what());
        exit(-1);
    }
#ifdef RB_PRINT_TRACE
    fprintf(stderr, "end save state\n");
#endif /* RB_PRINT_TRACE */
}

void aacDecoder_ExtLoadState(HANDLE_AACDECODER hAacDecoder, void *fp)
{
#ifdef RB_PRINT_TRACE
    fprintf(stderr, "begin load state\n");
#endif /* RB_PRINT_TRACE */
    PersistenceTraversalData td(PersistenceTraversalData::READ, (std::FILE*)fp);
    try {
        persist_AAC_DECODER_INSTANCE.traverse(hAacDecoder, td);
    }
    catch (std::exception &e) {
        fprintf(stderr, "loading decoder state failed: %s\n", e.what());
        exit(-1);
    }
#ifdef RB_PRINT_TRACE
    fprintf(stderr, "end load state\n");
#endif /* RB_PRINT_TRACE */
}

#endif /* RB_STATE_PERSISTENCE_EXTENSION */
