#ifndef TASKS3_HPP
#define TASKS3_HPP

#include "IQSignal.hpp"

namespace Task3 {
    using namespace DSP;
    struct ExperimentResult {
        Samples<UnitDSP::Seconds, double> refSignal;
        Samples<UnitDSP::Seconds, double> refI;
        Samples<UnitDSP::Seconds, double> refQ;
        Samples<UnitDSP::Seconds, double> delayedSignal;
        Samples<UnitDSP::Seconds, double> I;
        Samples<UnitDSP::Seconds, double> Q;
        Samples<UnitDSP::Seconds, double> crossCorrelation;
        UnitDSP::Seconds estimatedDelay;
    };

    struct StatResult {
        Samples<UnitDSP::dB, double> ASKprobVsSNR;
        Samples<UnitDSP::dB, double> BPSKprobVsSNR;
        Samples<UnitDSP::dB, double> MSKprobVsSNR;
    };

    struct StatResultDoppler {
        Samples<UnitDSP::Hertz, double> ASKmaxToStd;
        Samples<UnitDSP::Hertz, double> BPSKmaxToStd;
        Samples<UnitDSP::Hertz, double> MSKmaxToStd;
    };


    ExperimentResult doExperiment(IQSignal& iqSignal, UnitDSP::Hertz carrier,
        size_t bitCount, UnitDSP::Seconds duration, UnitDSP::Seconds delay,
        UnitDSP::dB SNR, UnitDSP::Hertz doppler);

    ExperimentResult singleExperimentASK(double amplitudeLow, double amplitudeHigh,
        UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
        UnitDSP::Hertz carrier, UnitDSP::Seconds delay, UnitDSP::Seconds duration,
        UnitDSP::dB SNR, UnitDSP::Hertz doppler);

    ExperimentResult singleExperimentBPSK(UnitDSP::Hertz sampleRate, size_t bitCount,
        double bitRate, UnitDSP::Hertz carrier, UnitDSP::Seconds delay,
        UnitDSP::Seconds duration, UnitDSP::dB SNR, UnitDSP::Hertz doppler);

    ExperimentResult singleExperimentMSK(UnitDSP::Hertz sampleRate, size_t bitCount,
        double bitRate, UnitDSP::Hertz carrier, UnitDSP::Seconds delay,
        UnitDSP::Seconds duration, UnitDSP::dB SNR, UnitDSP::Hertz doppler);

    StatResultDoppler statisticalExperiment(double amplitudeLow, double amplitudeHigh,
        UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
        UnitDSP::Hertz carrier, UnitDSP::Seconds delay, UnitDSP::Seconds duration,
        UnitDSP::Hertz dopplerLow, UnitDSP::Hertz dopplerHigh, int dopplerStepCount,
        int repsPerDoppler, UnitDSP::dB SNR, float* statProgress);

}

#endif
