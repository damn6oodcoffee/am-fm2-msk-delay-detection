
#include <mutex>
#include <future>
#include "Tasks3.hpp"


namespace Task3 {

    ExperimentResult doExperiment(IQSignal& iqSignal, UnitDSP::Hertz carrier,
        size_t bitCount, UnitDSP::Seconds duration, UnitDSP::Seconds delay,
        UnitDSP::dB SNR, UnitDSP::Hertz doppler)
    {
        auto bits = generateRandomBits(bitCount);
        auto IQSamples = iqSignal.sample(bits);
        auto refIQSamples = takeIQSlice(IQSamples, delay, duration);
        auto signal = modulateCarrier(carrier, IQSamples);
        auto refSignal = modulateCarrier(carrier, refIQSamples);

  
        signal = addDopplerShift(doppler, signal);
        signal.valueSamples = addComplexNoise(signal.valueSamples, SNR);
        refSignal.valueSamples = addComplexNoise(refSignal.valueSamples, 10.0);
#if 1
        decltype(signal) sig;
        sig.timeSamples = signal.timeSamples;
        decltype(refSignal) refSig;
        refSig.timeSamples = signal.timeSamples;
        if (false) {
            for (auto iq : IQSamples.valueSamples) {
                sig.valueSamples.push_back(std::complex(iq.I, iq.Q));
            }
            for (auto iq : refIQSamples.valueSamples) {
                refSig.valueSamples.push_back(std::complex(iq.I, iq.Q));
            }
            sig = addDopplerShift(doppler, sig);
        }
        else {
            for (auto v : signal.valueSamples)
                sig.valueSamples.push_back(v.real());
            for (auto v : refSignal.valueSamples)
                refSig.valueSamples.push_back(v.real());
        }

        auto indexedCrosscorrelation = computeComplexCrossCorrelation(
            sig.valueSamples, refSig.valueSamples);
        auto crosscorrelationSize = indexedCrosscorrelation.timeSamples.size();
#else
        auto indexedCrosscorrelation = computeComplexCrossCorrelation(
            signal.valueSamples, refSignal.valueSamples);
        auto crosscorrelationSize = indexedCrosscorrelation.timeSamples.size();
#endif

        Samples<UnitDSP::Seconds, double> crosscorrelation;
        double timeInterval = 1.0 / iqSignal.getSampleRate();
        std::transform(indexedCrosscorrelation.timeSamples.begin(),
            indexedCrosscorrelation.timeSamples.end(),
            std::back_inserter(crosscorrelation.timeSamples),
            [timeInterval](int idx) { return timeInterval * idx; });
        std::transform(indexedCrosscorrelation.valueSamples.begin(),
            indexedCrosscorrelation.valueSamples.end(),
            std::back_inserter(crosscorrelation.valueSamples),
            [timeInterval](std::complex<double> val) {
                return std::abs(val);
                //return std::real(val);
            });


        ExperimentResult result;
        std::transform(refSignal.valueSamples.begin(),
            refSignal.valueSamples.end(),
            std::back_inserter(result.refSignal.valueSamples),
            [](std::complex<double> val) {
                return val.imag();
            });
        result.refSignal.timeSamples = std::move(refSignal.timeSamples);

        std::transform(signal.valueSamples.begin(),
            signal.valueSamples.end(),
            std::back_inserter(result.delayedSignal.valueSamples),
            [](std::complex<double> val) {
                return val.real();
            });
        result.delayedSignal.timeSamples = std::move(signal.timeSamples);

        for (auto& iq : IQSamples.valueSamples) {
            result.I.valueSamples.push_back(iq.I);
            result.Q.valueSamples.push_back(iq.Q);
        }
        result.I.timeSamples = IQSamples.timeSamples;
        result.Q.timeSamples = IQSamples.timeSamples;
        for (auto& iq : refIQSamples.valueSamples) {
            result.refI.valueSamples.push_back(iq.I);
            result.refQ.valueSamples.push_back(iq.Q);
        }
        result.refI.timeSamples = refIQSamples.timeSamples;
        result.refQ.timeSamples = refIQSamples.timeSamples;


        result.crossCorrelation = std::move(crosscorrelation);
        auto maxIter = std::max_element(result.crossCorrelation.valueSamples.begin(),
            result.crossCorrelation.valueSamples.end());
        auto diff = std::distance(result.crossCorrelation.valueSamples.begin(), maxIter);
        result.estimatedDelay = result.crossCorrelation.timeSamples[diff];

        return result;

    }

    ExperimentResult singleExperimentASK(double amplitudeLow, double amplitudeHigh,
        UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
        UnitDSP::Hertz carrier, UnitDSP::Seconds delay, UnitDSP::Seconds duration,
        UnitDSP::dB SNR, UnitDSP::Hertz doppler)
    {
        IQSignalASK ask(carrier, sampleRate, bitRate);
        ask.setLowAndHigh(amplitudeLow, amplitudeHigh);
        return doExperiment(ask, carrier, bitCount, duration, delay, SNR, doppler);
    }

    ExperimentResult singleExperimentBPSK(UnitDSP::Hertz sampleRate, size_t bitCount,
        double bitRate, UnitDSP::Hertz carrier, UnitDSP::Seconds delay,
        UnitDSP::Seconds duration, UnitDSP::dB SNR, UnitDSP::Hertz doppler)
    {
        IQSignalBPSK bpsk(carrier, sampleRate, bitRate);
        return doExperiment(bpsk, carrier, bitCount, duration, delay, SNR, doppler);
    }

    ExperimentResult singleExperimentMSK(UnitDSP::Hertz sampleRate, size_t bitCount,
        double bitRate, UnitDSP::Hertz carrier, UnitDSP::Seconds delay,
        UnitDSP::Seconds duration, UnitDSP::dB SNR, UnitDSP::Hertz doppler)
    {
        IQSignalMSK msk(carrier, sampleRate, bitRate);
        return doExperiment(msk, carrier, bitCount, duration, delay, SNR, doppler);
    }


    StatResultDoppler statisticalExperiment(double amplitudeLow, double amplitudeHigh,
        UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
        UnitDSP::Hertz carrier, UnitDSP::Seconds delay, UnitDSP::Seconds duration,
        UnitDSP::Hertz dopplerLow, UnitDSP::Hertz dopplerHigh, int dopplerStepCount,
        int repsPerDoppler, UnitDSP::dB SNR, float* statProgress)
    {
        StatResultDoppler statResult;

        *statProgress = 0.0;
        float statProgressStep = 1.0 / (3 * dopplerStepCount);
        std::mutex progressMutex; // guards statProgress

        UnitDSP::Hertz dopplerStep{ (dopplerHigh - dopplerLow) / (dopplerStepCount - 1) };
        /*
        UnitDSP::Seconds bitInterval{ 1.0 / bitRate };
        auto processExperiment = [bitInterval, delay](const ExperimentResult& res, int& counter) {
            double delayConfidenceLow{ delay - bitInterval / 2 };
            double delayConfidenceHigh{ delay + bitInterval / 2 };
            if (isWithinRange(delayConfidenceLow, delayConfidenceHigh, res.estimatedDelay))
                counter++;
            };
            */

        auto processExperiment = [](const ExperimentResult& res) {
            auto stddev = stdDeviation(res.crossCorrelation.valueSamples);
            auto maxIter = std::max_element(
                res.crossCorrelation.valueSamples.begin(),
                res.crossCorrelation.valueSamples.end());
            return *maxIter / stddev;
            };

        auto ask = singleExperimentASK;
        auto bpsk = singleExperimentBPSK;
        auto msk = singleExperimentMSK;
        auto processSignalASK = [=, &statResult, &progressMutex] () 
        {
            ExperimentResult expResult{};
            UnitDSP::Hertz doppler{ dopplerLow };
            for (int i{ 0 }; i < dopplerStepCount; ++i) {
                double avgMaxToStd = 0.0;
                doppler = dopplerLow + dopplerStep * i;
                for (int rep{ 0 }; rep < repsPerDoppler; ++rep) {
                    expResult = ask(amplitudeLow, amplitudeHigh, sampleRate,
                        bitCount, bitRate, carrier, delay, duration, SNR, doppler);
                    avgMaxToStd += processExperiment(expResult) / repsPerDoppler;
                }
                statResult.ASKmaxToStd.timeSamples.push_back(doppler);
                statResult.ASKmaxToStd.valueSamples.push_back(avgMaxToStd);
                std::lock_guard<std::mutex> lock(progressMutex);
                *statProgress += statProgressStep;
            }
            };

        auto processSignalBPSK = [=, &statResult, &progressMutex] () 
        {
            ExperimentResult expResult{};
            UnitDSP::Hertz doppler{ dopplerLow };
            for (int i{ 0 }; i < dopplerStepCount; ++i) {
                double avgMaxToStd = 0.0;
                doppler = dopplerLow + dopplerStep * i;
                for (int rep{ 0 }; rep < repsPerDoppler; ++rep) {
                    expResult = bpsk(sampleRate, bitCount, bitRate, carrier, 
                        delay, duration, SNR, doppler);
                    avgMaxToStd += processExperiment(expResult) / repsPerDoppler;
                }
                statResult.BPSKmaxToStd.timeSamples.push_back(doppler);
                statResult.BPSKmaxToStd.valueSamples.push_back(avgMaxToStd);
                std::lock_guard<std::mutex> lock(progressMutex);
                *statProgress += statProgressStep;
            }
            };

        auto processSignalMSK = [=, &statResult, &progressMutex] () 
        {
            ExperimentResult expResult{};
            UnitDSP::Hertz doppler{ dopplerLow };
            for (int i{ 0 }; i < dopplerStepCount; ++i) {
                double avgMaxToStd = 0.0;
                doppler = dopplerLow + dopplerStep * i;
                for (int rep{ 0 }; rep < repsPerDoppler; ++rep) {
                    expResult = msk(sampleRate, bitCount, bitRate, carrier, 
                        delay, duration, SNR, doppler);
                    avgMaxToStd += processExperiment(expResult) / repsPerDoppler;
                }
                statResult.MSKmaxToStd.timeSamples.push_back(doppler);
                statResult.MSKmaxToStd.valueSamples.push_back(avgMaxToStd);
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
    }
    
     
}
