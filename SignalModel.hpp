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
    //virtual const std::vector<double>& getTimeSamples() const = 0;
    //virtual const std::vector<double>& getAmplitudeSamples() const = 0;
    //virtual void clear() = 0;
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
    //const std::vector<double>& getTimeSamples() const;
    //const std::vector<double>& getAmplitudeSamples() const;
    //void clear();

private:
    //std::vector<double> timeSamples_;
    //std::vector<double> amplitudeSamples_;
};


class SineSignalBitSampler : public SineSignalModel {
public:
    SineSignalBitSampler(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate);
    ~SineSignalBitSampler() = 0;
    void setBitRate(double bitRate);
    double getBitRate();
    void setBits(const std::vector<int>& bits);
    Samples<UnitDSP::Seconds, double> sample(size_t bitCount);
    // const std::vector<double>& getTimeSamples() const;
    // const std::vector<double>& getAmplitudeSamples() const;
    // void clear();

protected:
    virtual double sampleBit(int bit, double timePoint) = 0;

private:
    std::vector<int> bits_;
    // std::vector<double> timeSamples_;
    // std::vector<double> amplitudeSamples_;
    double bitRate_;
};


class SineSignalASK : public SineSignalBitSampler {
public:
    SineSignalASK(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate);
    double sampleBit(int bit, UnitDSP::Seconds timePoint);
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


#if 0
class SineSignalASK : public SineSignalModel {
public:
    SineSignalASK(UnitDSP::Hertz carrierFreq, UnitDSP::Radians phase, UnitDSP::Hertz sampleRate,
                  size_t bitRate);
    void sample(size_t bitCount);
    const std::vector<double>& getTimeSamples() const;
    const std::vector<double>& getAmplitudeSamples() const;
    void clear();
    void generateDelayed(SignalModel::UnitDSP::Seconds delay, 
                         SignalModel::UnitDSP::Seconds duration);
    SineSignalASK getReferenceSignal(SignalModel::UnitDSP::Seconds delay, 
                                     SignalModel::UnitDSP::Seconds duration);
    std::vector<int> bits;
private:
    UnitDSP::Radians phase_;
    size_t bitRate_;

    std::vector<double> timeSamples_;
    std::vector<double> amplitudeSamples_;
};


#endif

#if 0
std::vector<double> getReferenceSineSignal(const SineSignalModel& srcSignal, 
                                           SignalModel::UnitDSP::Seconds srcDelay, 
                                           SignalModel::UnitDSP::Seconds duration);
#endif

Samples<int, double> computeCrossCorrelation(const std::vector<double>& sequenceA, 
                                             const std::vector<double>& sequenceB);

std::vector<int> generateRandomBits(size_t size);

std::vector<double> addNoise(const std::vector<double>& amplitudes, UnitDSP::dB signalToNoiseRatio);


#endif