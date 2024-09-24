#ifndef TASKS_HPP
#define TASKS_HPP
#include "SignalModel.hpp"

struct ExperimentResult {
    Samples<UnitDSP::Seconds, double> refSignal;
    Samples<UnitDSP::Seconds, double> delayedSignal;
    Samples<UnitDSP::Seconds, double> crossCorrelation;
    UnitDSP::Seconds estimatedDelay;
    Samples<UnitDSP::Seconds, double> bitSamples;
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

ExperimentResult doExperiment(SineSignalBitSampler& sampler, UnitDSP::Seconds duration,
    UnitDSP::Seconds delay, UnitDSP::dB SNR);

ExperimentResult singleExperimentASK(double amplitudeLow, double amplitudeHigh,
    UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
    UnitDSP::Hertz carrier, UnitDSP::Seconds delay, UnitDSP::dB snr);

ExperimentResult singleExperimentBPSK(UnitDSP::Hertz sampleRate, size_t bitCount,
    double bitRate, UnitDSP::Hertz carrier,
    UnitDSP::Seconds delay, UnitDSP::dB SNR);

ExperimentResult singleExperimentMSK(UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
    UnitDSP::Hertz carrier, UnitDSP::Hertz frequenceDiff,
    UnitDSP::Seconds delay, UnitDSP::dB SNR);

StatResult statisticalExperiment(double amplitudeLow, double amplitudeHigh,
    UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
    UnitDSP::Hertz carrier, UnitDSP::Seconds delay,
    UnitDSP::dB snrLow, UnitDSP::dB snrHigh, int snrStepCount,
    int repsPerSNR, float* statProgress);

#endif 