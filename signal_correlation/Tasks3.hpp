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

    template<typename T>
    bool isWithinRange(const T& low, const T& high, const T& val) {
        return (low <= val) && (val <= high);
    }

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

    StatResult statisticalExperiment(double amplitudeLow, double amplitudeHigh,
        UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
        UnitDSP::Hertz carrier, UnitDSP::Seconds delay,
        UnitDSP::dB snrLow, UnitDSP::dB snrHigh, int snrStepCount,
        int repsPerSNR, float* statProgress);

}

#endif
