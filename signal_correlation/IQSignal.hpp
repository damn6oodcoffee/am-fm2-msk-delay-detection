#ifndef IQ_SIGNAL_HPP
#define IQ_SIGNAL_HPP

#include "SignalModel.hpp"
#include <numbers>
#include <cassert>
#include <algorithm>

namespace DSP {


    class IQSignal {
    public:
        struct IQ {
            double I;
            double Q;
        };

        IQSignal(UnitDSP::Hertz carrier, UnitDSP::Hertz sampleRate, double symbolRate)
            : carrier_{ carrier }
            , sampleRate_{ sampleRate }
            , symbolRate_{ symbolRate }
        {

        }

        virtual ~IQSignal() = 0 {};
        
        virtual Samples<UnitDSP::Seconds, IQ> sample(const std::vector<int>& bits) = 0;

    
        virtual UnitDSP::Hertz getSampleRate() const { return sampleRate_; }
        virtual void setSampleRate(UnitDSP::Hertz sampleRate) { sampleRate_ = sampleRate; }

        virtual double getSymbolRate() const { return symbolRate_; }
        virtual void setSymbolRate(double symbolRate) { symbolRate_ = symbolRate; }

        virtual UnitDSP::Hertz getCarrier() const { return carrier_; }
        virtual void setCarrier(UnitDSP::Hertz carrier) { carrier_ = carrier; }
        
         
    private:
        UnitDSP::Hertz sampleRate_;
        double symbolRate_;
        UnitDSP::Hertz carrier_;

    protected:
        virtual std::vector<UnitDSP::Seconds> getIQSignalTimePoints(size_t symbolCount) {
            UnitDSP::Seconds dt{ 1 / getSampleRate() };
            UnitDSP::Seconds duration{ static_cast<double>(symbolCount) / getSymbolRate() };
            UnitDSP::Seconds t{ 0.0 };
            return linspace(t, t + duration, dt);
        }
    };


    class IQSignalASK : public IQSignal {
    public:
        IQSignalASK(UnitDSP::Hertz carrier, UnitDSP::Hertz sampleRate, double symbolRate)
            : IQSignal(carrier, sampleRate, symbolRate)
            , lowAmp_{ 0 }
            , highAmp_{ 0 }
        {}
               
        
        Samples<UnitDSP::Seconds, IQ> sample(const std::vector<int>& bits) override
        {
            auto timeSamples = getIQSignalTimePoints(bits.size());
            auto IQSamples = computeIQ(bits, timeSamples);
            return { timeSamples, IQSamples };
        }
        
        void setLowAndHigh(double low, double high) {
            lowAmp_ = low;
            highAmp_ = high;
        }

        
    private:
        double lowAmp_;
        double highAmp_;
        std::vector<IQ> computeIQ(const std::vector<int>& bits, 
            std::vector<UnitDSP::Seconds> timeSamples) 
        {
            double bitRate{ getSymbolRate() };
            std::vector<IQ> IQSamples;
            for (auto t : timeSamples) {
                int currentBit = bits[static_cast<int>(t * bitRate)];
                if (currentBit == 0)
                    IQSamples.push_back({ lowAmp_, 0 });
                else
                    IQSamples.push_back({ highAmp_, 0 });
            }
            return IQSamples;
        }
       
        
    };
 
    class IQSignalBPSK: public IQSignal {
    public:
        IQSignalBPSK(UnitDSP::Hertz carrier, UnitDSP::Hertz sampleRate, double symbolRate)
            : IQSignal(carrier, sampleRate, symbolRate)
        {}
                       
        Samples<UnitDSP::Seconds, IQ> sample(const std::vector<int>& bits) override
        {
            auto timeSamples = getIQSignalTimePoints(bits.size());
            auto IQSamples = computeIQ(bits, timeSamples);
            return { timeSamples, IQSamples };
        }

    private:
        std::vector<IQ> computeIQ(const std::vector<int>& bits, std::vector<UnitDSP::Seconds> timeSamples) {
            double bitRate{ getSymbolRate() };
            std::vector<IQ> IQSamples;
            for (auto t : timeSamples) {
                int currentBit = bits[static_cast<int>(t * bitRate)];
                if (currentBit == 0)
                    IQSamples.push_back({ -1, 0 });
                else
                    IQSamples.push_back({ 1, 0 });
            }
            return IQSamples;
        }
    };

    class IQSignalMSK : public IQSignal {
    public:
        IQSignalMSK(UnitDSP::Hertz carrier, UnitDSP::Hertz sampleRate, double symbolRate)
            : IQSignal(carrier, sampleRate, symbolRate)
        {}

        Samples<UnitDSP::Seconds, IQ> sample(const std::vector<int>& bits) override
        {
            auto timeSamples = getIQSignalTimePoints(bits.size());
            auto IQSamples = computeIQ(bits, timeSamples);
            return { timeSamples, IQSamples };
        }

    private:
        std::vector<IQ> computeIQ(const std::vector<int>& bits, std::vector<UnitDSP::Seconds> timeSamples) {
            UnitDSP::Seconds dt{ 1 / getSampleRate() };
            double angularFrequencyDifference{ getSymbolRate() * std::numbers::pi / 2 };
            double bitRate{ getSymbolRate() };
            double bi{ 0.0 };
            std::vector<IQ> IQSamples;
            for (auto t : timeSamples) {
                int currentBit = bits[static_cast<int>(t * bitRate)];
                IQSamples.push_back({
                    cos(angularFrequencyDifference * bi),
                    sin(angularFrequencyDifference * bi),
                    });
                if (currentBit == 1)
                    bi += dt;
                else
                    bi -= dt;
            }
            return IQSamples;
        }
       
    };

    inline Samples<UnitDSP::Seconds, IQSignal::IQ> takeIQSlice(
        const Samples<UnitDSP::Seconds, IQSignal::IQ>& IQSamples,
        UnitDSP::Seconds start, UnitDSP::Seconds duration)
    {
        auto timeSliceBegIt = std::find_if(IQSamples.timeSamples.begin(), IQSamples.timeSamples.end(),
            [start](double val) { return val >= start; });
        if (timeSliceBegIt == IQSamples.timeSamples.end())
            return {};
        auto timeSliceEndIt = std::find_if(IQSamples.timeSamples.begin(), IQSamples.timeSamples.end(),
            [start, duration](double val) { return val >= start + duration; });
        auto distToBeg = std::distance(IQSamples.timeSamples.begin(), timeSliceBegIt);
        auto distToEnd = std::distance(IQSamples.timeSamples.begin(), timeSliceEndIt);

        auto IQSliceBegIt = IQSamples.valueSamples.begin();
        auto IQSliceEndIt = IQSamples.valueSamples.begin();
        std::advance(IQSliceBegIt, distToBeg);
        std::advance(IQSliceEndIt, distToEnd);

        Samples<UnitDSP::Seconds, IQSignal::IQ> slice;
        slice.timeSamples.assign(timeSliceBegIt, timeSliceEndIt);
        slice.valueSamples.assign(IQSliceBegIt, IQSliceEndIt);
        
        assert(slice.timeSamples.size() == slice.valueSamples.size());
        return slice;
    }

    inline Samples<UnitDSP::Seconds, std::complex<double>> modulateCarrier(
        UnitDSP::Hertz carrier,
        const Samples<UnitDSP::Seconds, IQSignal::IQ>& IQSamples)
    {
        using namespace std::complex_literals;
        Samples<UnitDSP::Seconds, std::complex<double>> signal;
        signal.timeSamples = IQSamples.timeSamples;
        for (size_t i{ 0 }; i < signal.timeSamples.size(); ++i) {
            auto t = signal.timeSamples[i];
            auto I = IQSamples.valueSamples[0].I;
            auto Q = IQSamples.valueSamples[0].Q;
            auto value = (I + 1i * Q) * exp(1i * carrier * t);
            signal.valueSamples.push_back(value);
        }
        return signal;
    }

    inline Samples<UnitDSP::Seconds, std::complex<double>> addDopplerShift(
        UnitDSP::Hertz dopplerShift,
        const Samples<UnitDSP::Seconds, std::complex<double>>& complexSignal)
    {
        using namespace std::complex_literals;
        auto distortedSignal = complexSignal;
        std::transform(distortedSignal.timeSamples.begin(),
            distortedSignal.timeSamples.end(),
            distortedSignal.valueSamples.begin(),
            distortedSignal.valueSamples.begin(),
            [dopplerShift](UnitDSP::Seconds t, std::complex<double> val) {
                return val * exp(1i * dopplerShift * t);
            });
        return distortedSignal;
    }
}

#endif