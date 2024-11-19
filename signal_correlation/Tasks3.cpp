
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
        addDopplerShift(doppler, signal);
        signal.valueSamples = addComplexNoise(signal.valueSamples, SNR);
        refSignal.valueSamples = addComplexNoise(refSignal.valueSamples, 10.0);

        auto indexedCrosscorrelation = computeComplexCrossCorrelation(
            signal.valueSamples, refSignal.valueSamples);
        auto crosscorrelationSize = indexedCrosscorrelation.timeSamples.size();

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
            });
        
        
        ExperimentResult result;
        std::transform(refSignal.valueSamples.begin(),
            refSignal.valueSamples.end(),
            std::back_inserter(result.refSignal.valueSamples),
            [](std::complex<double> val) {
                return val.real();
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
        for (auto& iq : refIQSamples.valueSamples) {
            result.refI.valueSamples.push_back(iq.I);
            result.refQ.valueSamples.push_back(iq.Q);
        }
        result.crossCorrelation = std::move(crosscorrelation);

        auto maxIter = std::max_element(crosscorrelation.valueSamples.begin(),
            crosscorrelation.valueSamples.end());
        auto diff = std::distance(crosscorrelation.valueSamples.begin(), maxIter);
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

    StatResult statisticalExperiment(double amplitudeLow, double amplitudeHigh,
        UnitDSP::Hertz sampleRate, size_t bitCount, double bitRate,
        UnitDSP::Hertz carrier, UnitDSP::Seconds delay,
        UnitDSP::dB snrLow, UnitDSP::dB snrHigh, int snrStepCount,
        int repsPerSNR, float* statProgress)
    {
        return{};
    }
    
     
}
