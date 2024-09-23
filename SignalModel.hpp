#ifndef SIGNALMODEL_HPP
#define SIGNALMODEL_HPP

#include <vector>
#include <stdexcept>

constexpr double m_pi = 3.14159265358979323846;

template<typename U, typename V>
struct SamplePoint {
    SamplePoint(U x, V y)
        : x{x}
        , y{y}
    {}
    U x; 
    V y;
};

namespace UnitDSP {
    using Hertz = double; 
    using Radians = double;
    using Seconds = double;
    using dB = double;
}


template<typename TypeX, typename TypeY>
struct Samples {
    Samples() {}
    Samples(std::vector<TypeX> timeSamples, std::vector<TypeY> valueSamples)
        : timeSamples{timeSamples}
        , valueSamples{valueSamples} 
    {}
    Samples(const Samples& other) 
        : timeSamples{other.timeSamples}
        , valueSamples{other.valueSamples}
    {}
    Samples(Samples&& other) noexcept
        : timeSamples{std::move(other.timeSamples)}
        , valueSamples{std::move(other.valueSamples)}
    {}

    Samples& operator=(const Samples& other) {
        if (this == &other)
            return *this;
        timeSamples = other.timeSamples;
        valueSamples = other.valueSamples;
        return *this;
    }

    Samples& operator=(Samples&& other) {
        timeSamples = std::move(other.timeSamples);
        valueSamples = std::move(other.valueSamples);
        return *this;
    }

    std::vector<TypeX> timeSamples;
    std::vector<TypeY> valueSamples;
};

class SignalModel {
public:
    virtual Samples<double, double> sample(size_t count) = 0;
    virtual ~SignalModel() = 0;
};


class SineSignalModel : public SignalModel {
public:
    SineSignalModel(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase);
    virtual UnitDSP::Hertz getCarrierFrequency() const;
    virtual UnitDSP::Hertz getSampleRate() const;
    virtual void setCarrierFrequency(UnitDSP::Hertz carrier);
    virtual void setSampleRate(UnitDSP::Hertz sampleRate);
    virtual ~SineSignalModel() = 0;
    UnitDSP::Radians phase;
private:
    UnitDSP::Hertz carrierFreq_;
    UnitDSP::Hertz sampleRate_;
};


class SineSignal : public SineSignalModel {
public:
    SineSignal();
    SineSignal(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase);

    Samples<UnitDSP::Seconds, double> sample(size_t sampleCount);

};


class SineSignalBitSampler : public SineSignalModel {
public:
    SineSignalBitSampler(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate);
    ~SineSignalBitSampler() = 0;
    void setBitRate(double bitRate);
    double getBitRate();
    void setBits(const std::vector<int>& bits);
    Samples<UnitDSP::Seconds, double> sample(size_t bitCount);

protected:
    virtual double sampleBit(int bit, double timePoint) = 0;

private:
    std::vector<int> bits_;
    double bitRate_;
};


class SineSignalASK : public SineSignalBitSampler {
public:
    SineSignalASK(double lowAmplitude, double highAmplitude, UnitDSP::Hertz carrierFreq,
                  UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate);
    double sampleBit(int bit, UnitDSP::Seconds timePoint);
    
private:
    double lowAmp_;
    double highAmp_;
};


class SineSignalBPSK : public SineSignalBitSampler {
public:
    SineSignalBPSK(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate);
    double sampleBit(int bit, UnitDSP::Seconds timePoint);
};


class SineSignalMSK : public SineSignalBitSampler {
public:
    SineSignalMSK(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate);
    double sampleBit(int bit, UnitDSP::Seconds timePoint);
private:
    UnitDSP::Hertz frequencyDiff_;
};


class DelayedSineSignalBitSampler {
public:
    DelayedSineSignalBitSampler() {};
    ~DelayedSineSignalBitSampler() {};
    void generate(SineSignalBitSampler& sampler, UnitDSP::Seconds delay, UnitDSP::Seconds duration);
    const Samples<double, double>& getReferenceSamples() const;
    const Samples<double, double>& getDelayedSamples() const;

private:
    UnitDSP::Seconds delay_;
    UnitDSP::Seconds duration_;
    Samples<UnitDSP::Seconds, double> refSignal;
    Samples<UnitDSP::Seconds, double> delayedSignal;
    void generateDelayed(SineSignalBitSampler& sampler);
    void extractReference(SineSignalBitSampler& sampler);
};



Samples<int, double> computeCrossCorrelation(const std::vector<double>& sequenceA, 
                                             const std::vector<double>& sequenceB);

std::vector<int> generateRandomBits(size_t size);

std::vector<double> addNoise(const std::vector<double>& amplitudes, UnitDSP::dB signalToNoiseRatio);


#endif