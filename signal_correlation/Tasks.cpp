
#include <algorithm>
#include <functional>
#include <future>
#include <mutex>
#include "Tasks.hpp"



ExperimentResult doExperiment(SineSignalBitSampler& sampler, UnitDSP::Seconds duration,
                              UnitDSP::Seconds delay, UnitDSP::dB SNR)
{
    ExperimentResult result{};
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


ExperimentResult singleExperimentASK(double amplitudeLow, double amplitudeHigh, 
                                    UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
                                    UnitDSP::Hertz carrier, UnitDSP::Seconds delay, UnitDSP::dB SNR)
{
    double duration = static_cast<double>(bitCount) / bitRate; 
    SineSignalASK signalASK(amplitudeLow, amplitudeHigh, carrier, sampleRate, 0.0, bitRate);
    return doExperiment(signalASK, duration, delay, SNR);
}


ExperimentResult singleExperimentBPSK(UnitDSP::Hertz sampleRate, size_t bitCount,
                                      double bitRate, UnitDSP::Hertz carrier,
                                      UnitDSP::Seconds delay, UnitDSP::dB SNR)
{
    double duration = static_cast<double>(bitCount) / bitRate; 
    SineSignalBPSK signalBPSK(carrier, sampleRate, 0.0, bitRate);
    return doExperiment(signalBPSK, duration, delay, SNR);
}


ExperimentResult singleExperimentMSK(UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
                                     UnitDSP::Hertz carrier, UnitDSP::Hertz frequenceDiff,
                                     UnitDSP::Seconds delay, UnitDSP::dB SNR)
{
    double duration = static_cast<double>(bitCount) / bitRate; 
    SineSignalMSK signalMSK(carrier, sampleRate, 0.0, bitRate);
    return doExperiment(signalMSK, duration, delay, SNR);
}


StatResult statisticalExperiment(double amplitudeLow, double amplitudeHigh, 
                                UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
                                UnitDSP::Hertz carrier, UnitDSP::Seconds delay, 
                                UnitDSP::dB snrLow, UnitDSP::dB snrHigh, int snrStepCount,
                                int repsPerSNR, float* statProgress)
{
#if 0
    StatResult statResult{};
    ExperimentResult expResult{};

    *statProgress = 0.0;
    float statProgressStep = 1.0 / snrStepCount;

    UnitDSP::dB snrStep{ (snrHigh - snrLow) / (snrStepCount - 1) };
    UnitDSP::dB snr{ snrLow };
    UnitDSP::Seconds bitInterval{1.0 / bitRate};

    auto processExperiment = [bitInterval, delay](const ExperimentResult& res, int& counter) {
        double delayConfidenceLow{ delay - bitInterval / 2 };
        double delayConfidenceHigh{ delay + bitInterval / 2 };
        if (isWithinRange(delayConfidenceLow, delayConfidenceHigh, res.estimatedDelay))
            counter++;
    };

    for (int i{0}; i < snrStepCount; ++i) {
        int successfulEstimatesCountASK{0};
        int successfulEstimatesCountBPSK{0};
        int successfulEstimatesCountMSK{0};
        snr = snrLow + snrStep * i;
        for (int rep{0}; rep < repsPerSNR; ++rep) {
            expResult = singleExperimentASK(amplitudeLow, amplitudeHigh, 
                                            sampleRate, bitCount, bitRate,
                                            carrier, delay, snr);
            processExperiment(expResult, successfulEstimatesCountASK);

            expResult = singleExperimentBPSK(sampleRate, bitCount, bitRate,
                                             carrier, delay, snr);
            processExperiment(expResult, successfulEstimatesCountBPSK);
                                
            expResult = singleExperimentMSK(sampleRate, bitCount, bitRate,
                                            carrier, 0.0, delay, snr);
            processExperiment(expResult, successfulEstimatesCountMSK);
        }
        statResult.ASKprobVsSNR.timeSamples.push_back(snr);
        statResult.BPSKprobVsSNR.timeSamples.push_back(snr);
        statResult.MSKprobVsSNR.timeSamples.push_back(snr);

        statResult.ASKprobVsSNR.valueSamples.push_back(
            static_cast<double>(successfulEstimatesCountASK) / repsPerSNR
        );
        statResult.BPSKprobVsSNR.valueSamples.push_back(
            static_cast<double>(successfulEstimatesCountBPSK) / repsPerSNR
        );
        statResult.MSKprobVsSNR.valueSamples.push_back(
            static_cast<double>(successfulEstimatesCountMSK) / repsPerSNR
        );

        *statProgress += statProgressStep;
    }
    return statResult;
#else
    StatResult statResult{};

    
    *statProgress = 0.0;
    float statProgressStep = 1.0 / (3*snrStepCount);
    std::mutex progressMutex; // guards statProgress

    UnitDSP::dB snrStep{ (snrHigh - snrLow) / (snrStepCount - 1) };
    UnitDSP::Seconds bitInterval{1.0 / bitRate};

    auto processExperiment = [bitInterval, delay](const ExperimentResult& res, int& counter) {
        double delayConfidenceLow{ delay - bitInterval / 2 };
        double delayConfidenceHigh{ delay + bitInterval / 2 };
        if (isWithinRange(delayConfidenceLow, delayConfidenceHigh, res.estimatedDelay))
            counter++;
    };

    auto ask = singleExperimentASK;
    auto bpsk = singleExperimentBPSK;
    auto msk = singleExperimentMSK;
    auto processSignalASK = [=, &statResult, &progressMutex](){
        ExperimentResult expResult{};
        UnitDSP::dB snr{ snrLow };
        for (int i{0}; i < snrStepCount; ++i) {
            int successfulEstimatesCount{0};
            snr = snrLow + snrStep * i;
            for (int rep{0}; rep < repsPerSNR; ++rep) {
                expResult = ask(amplitudeLow, amplitudeHigh, sampleRate, 
                                bitCount, bitRate, carrier, delay, snr);
                processExperiment(expResult, successfulEstimatesCount);
            }
            statResult.ASKprobVsSNR.timeSamples.push_back(snr);
            statResult.ASKprobVsSNR.valueSamples.push_back(
                static_cast<double>(successfulEstimatesCount) / repsPerSNR
            );
            std::lock_guard<std::mutex> lock(progressMutex);
            *statProgress += statProgressStep;
        }
    };

    auto processSignalBPSK = [=, &statResult,  &progressMutex](){
        ExperimentResult expResult{};
        UnitDSP::dB snr{ snrLow };
        for (int i{0}; i < snrStepCount; ++i) {
            int successfulEstimatesCount{0};
            snr = snrLow + snrStep * i;
            for (int rep{0}; rep < repsPerSNR; ++rep) {
                expResult = bpsk(sampleRate, bitCount, bitRate, carrier, delay, snr);
                processExperiment(expResult, successfulEstimatesCount);
            }
            statResult.BPSKprobVsSNR.timeSamples.push_back(snr);
            statResult.BPSKprobVsSNR.valueSamples.push_back(
                static_cast<double>(successfulEstimatesCount) / repsPerSNR
            );

            std::lock_guard<std::mutex> lock(progressMutex);
            *statProgress += statProgressStep;
        }
    };

    auto processSignalMSK = [=, &statResult,  &progressMutex](){
        ExperimentResult expResult{};
        UnitDSP::dB snr{ snrLow };
        for (int i{0}; i < snrStepCount; ++i) {
            int successfulEstimatesCount{0};
            snr = snrLow + snrStep * i;
            for (int rep{0}; rep < repsPerSNR; ++rep) {
                expResult = msk(sampleRate, bitCount, bitRate, carrier, 0.0, delay, snr);
                processExperiment(expResult, successfulEstimatesCount);
            }
            statResult.MSKprobVsSNR.timeSamples.push_back(snr);
            statResult.MSKprobVsSNR.valueSamples.push_back(
                static_cast<double>(successfulEstimatesCount) / repsPerSNR
            );
            std::lock_guard<std::mutex> lock(progressMutex);
            *statProgress += statProgressStep;
        }
    };

    auto askFuture = std::async(std::launch::async, processSignalASK);
    auto bpskFuture = std::async(std::launch::async, processSignalBPSK);
    auto mskFuture = std::async(std::launch::async, processSignalMSK);

    askFuture.get();
    bpskFuture.get();
    mskFuture.get();
    return statResult;

#endif
}