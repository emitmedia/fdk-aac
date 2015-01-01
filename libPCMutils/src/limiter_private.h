#ifndef _LIMITER_PRIVATE_H_
#define _LIMITER_PRIVATE_H_

struct TDLimiter {
    unsigned int  attack;
    FIXP_DBL      attackConst, releaseConst;
    unsigned int  attackMs, releaseMs, maxAttackMs;
    FIXP_PCM      threshold;
    unsigned int  channels, maxChannels;
    unsigned int  sampleRate, maxSampleRate;
    FIXP_DBL      cor, max;
    FIXP_DBL*     maxBuf;
    FIXP_DBL*     delayBuf;
    unsigned int  maxBufIdx, delayBufIdx;
    FIXP_DBL      smoothState0;
    FIXP_DBL      minGain;

    FIXP_DBL      additionalGainPrev;
    FIXP_DBL      additionalGainFilterState;
    FIXP_DBL      additionalGainFilterState1;
};

#endif /* _LIMITER_PRIVATE_H_ */