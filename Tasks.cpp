
#include <algorithm>
#include "Tasks.hpp"


ExperimentResult singleExperimentASK(double amplitudeLow, double amplitudeHigh, 
                                    UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
                                    UnitDSP::Hertz carrier, UnitDSP::Seconds delay, UnitDSP::dB SNR)
{
    // ExperimentResult result;
    // double duration = static_cast<double>(bitCount) / bitRate; 
    // SineSignalASK signalASK(carrier, 0.0, sampleRate, bitRate);
    // signalASK.generateDelayed(delay, duration);
    // result.delayedSignalTime = signalASK.getTimeSamples();
    // result.delayedSignalAmplitude = addNoise(signalASK.getAmplitudeSamples(), SNR);

    // auto refSignal = signalASK.getReferenceSignal(delay, duration);
    // result.refSignalTime = refSignal.getTimeSamples();
    // result.refSignalAmplitude = addNoise(refSignal.getAmplitudeSamples(), 10.0);
    
    // auto crosscorr = computeCrossCorrelation(result.delayedSignalAmplitude, result.refSignalAmplitude);
    // double timeInterval{ 1.0 / refSignal.getSampleRate() };
    // for (auto& elt : crosscorr) {
    //     result.crosscorrDelay.push_back(elt.x * timeInterval);
    //     result.crosscorrValue.push_back(elt.y);
    // }
    // auto maxIter = std::max_element(result.crosscorrValue.begin(), result.crosscorrValue.end());
    // auto diff = std::distance(result.crosscorrValue.begin(), maxIter);
    // maxIter = result.crosscorrDelay.begin();
    // std::advance(maxIter, diff);
    // result.estimatedDelay = *maxIter;

    // return result;

    double duration = static_cast<double>(bitCount) / bitRate; 
    SineSignalASK signalASK(carrier, 0.0, sampleRate, bitRate);
    return doExperiment(signalASK, duration, delay, SNR);
}


ExperimentResult singleExperimentBPSK(UnitDSP::Hertz sampleRate, size_t bitCount,
                                      double bitRate, UnitDSP::Hertz carrier,
                                      UnitDSP::Seconds delay, UnitDSP::dB SNR)
{
    double duration = static_cast<double>(bitCount) / bitRate; 
    SineSignalBPSK signalBPSK(carrier, 0.0, sampleRate, bitRate);
    return doExperiment(signalBPSK, duration, delay, SNR);
}

ExperimentResult singleExperimentMSK(UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
                                     UnitDSP::Hertz carrier, UnitDSP::Hertz frequenceDiff,
                                     UnitDSP::Seconds delay, UnitDSP::dB SNR)
{
    return ExperimentResult{};
}


ExperimentResult doExperiment(SineSignalBitSampler& sampler, UnitDSP::Seconds duration,
                              UnitDSP::Seconds delay, UnitDSP::dB SNR)
{
    ExperimentResult result;
    double timeInterval{ 1.0 / sampler.getSampleRate() };
    DelayedSineSignalBitSampler delayedSampler;
    delayedSampler.generate(sampler, delay, duration);

    result.delayedSignal = delayedSampler.getDelayedSamples();
    result.refSignal = delayedSampler.getReferenceSamples();

    result.delayedSignal.valueSamples = addNoise(result.delayedSignal.valueSamples, SNR);
    result.refSignal.valueSamples = addNoise(result.refSignal.valueSamples, 10.0);
    

    auto crossCorrelation = computeCrossCorrelation(result.delayedSignal.valueSamples, result.refSignal.valueSamples);
    result.crossCorrelation.valueSamples = crossCorrelation.valueSamples;
    result.crossCorrelation.timeSamples.reserve(result.crossCorrelation.valueSamples.size());
    for (auto&& delayIndex : crossCorrelation.timeSamples) {
        result.crossCorrelation.timeSamples.push_back(delayIndex * timeInterval);
    }
   
    auto maxIter = std::max_element(result.crossCorrelation.valueSamples.begin(), 
                                    result.crossCorrelation.valueSamples.end());
    auto diff = std::distance(result.crossCorrelation.valueSamples.begin(), maxIter);
    result.estimatedDelay = result.crossCorrelation.timeSamples[diff];
    return result;
}