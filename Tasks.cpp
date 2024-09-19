
#include <algorithm>
#include "Tasks.hpp"


ExperimentResult singleExperimentAM(double amplitudeLow, double amplitudeHigh, 
                                     SignalModel::Hertz sampleRate, size_t bitCount, double bitRate,
                                     SignalModel::Hertz carrier, SignalModel::Seconds delay, SignalModel::dB SNR)
{
    ExperimentResult result;
    double duration = static_cast<double>(bitCount) / bitRate; 
    SineSignalASK signalASK(carrier, 0.0, sampleRate, bitRate);
    signalASK.generateDelayed(delay, duration);
    result.delayedSignalTime = signalASK.getTimeSamples();
    result.delayedSignalAmplitude = addNoise(signalASK.getAmplitudeSamples(), SNR);

    auto refSignal = signalASK.getReferenceSignal(delay, duration);
    result.refSignalTime = refSignal.getTimeSamples();
    result.refSignalAmplitude = addNoise(refSignal.getAmplitudeSamples(), 10.0);
    
    auto crosscorr = computeCrossCorrelation(result.delayedSignalAmplitude, result.refSignalAmplitude);
    double timeInterval{ 1.0 / refSignal.getSampleRate() };
    for (auto& elt : crosscorr) {
        result.crosscorrDelay.push_back(elt.x * timeInterval);
        result.crosscorrValue.push_back(elt.y);
    }
    auto maxIter = std::max_element(result.crosscorrValue.begin(), result.crosscorrValue.end());
    auto diff = std::distance(result.crosscorrValue.begin(), maxIter);
    maxIter = result.crosscorrDelay.begin();
    std::advance(maxIter, diff);
    result.estimatedDelay = *maxIter;

    return result;
}


ExperimentResult singleExperimentFM2(SignalModel::Hertz sampleRate, size_t bitCount,
                                      double bitRate, SignalModel::Hertz carrier,
                                      SignalModel::Seconds delay, SignalModel::dB SNR)
{
    return ExperimentResult{};
}

ExperimentResult singleExperimentMSK(SignalModel::Hertz sampleRate, size_t bitCount, double bitRate,
                                     SignalModel::Hertz carrier, SignalModel::Hertz frequenceDiff,
                                     SignalModel::Seconds delay, SignalModel::dB SNR)
{
    return ExperimentResult{};
}