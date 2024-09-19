#ifndef TASKS_HPP
#define TASKS_HPP
#include "SignalModel.hpp"

struct ExperimentResult {
    std::vector<double> refSignalTime;
    std::vector<double> refSignalAmplitude;
    std::vector<double> delayedSignalTime;
    std::vector<double> delayedSignalAmplitude;
    std::vector<double> crosscorrDelay;
    std::vector<double> crosscorrValue;

    double estimatedDelay;
};

ExperimentResult singleExperimentAM(double amplitudeLow, double amplitudeHigh, 
                                     SignalModel::Hertz sampleRate, size_t bitCount, double bitRate,
                                     SignalModel::Hertz carrier, SignalModel::Seconds delay, SignalModel::dB snr);

ExperimentResult singleExperimentFM2(SignalModel::Hertz sampleRate, size_t bitCount,
                                      double bitRate, SignalModel::Hertz carrier,
                                      SignalModel::Seconds delay, SignalModel::dB SNR);

ExperimentResult singleExperimentMSK(SignalModel::Hertz sampleRate, size_t bitCount, double bitRate,
                                      SignalModel::Hertz carrier, SignalModel::Hertz frequenceDiff,
                                      SignalModel::Seconds delay, SignalModel::dB SNR);

#endif 