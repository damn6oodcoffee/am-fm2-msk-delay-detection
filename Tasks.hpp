#ifndef TASKS_HPP
#define TASKS_HPP
#include "SignalModel.hpp"

struct ExperimentResult {
    Samples<UnitDSP::Seconds, double> refSignal;
    Samples<UnitDSP::Seconds, double> delayedSignal;
    Samples<UnitDSP::Seconds, double> crossCorrelation;
    double estimatedDelay;
};

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

#endif 