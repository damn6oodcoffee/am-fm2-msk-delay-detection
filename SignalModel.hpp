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

class SignalModel {
public:
    using Hertz = double; 
    using Radians = double;
    using Seconds = double;
    using dB = double;
    virtual void sample(size_t count) = 0;
    virtual const std::vector<double>& getTimeSamples() const = 0;
    virtual const std::vector<double>& getAmplitudeSamples() const = 0;
    virtual void clear() = 0;
    virtual ~SignalModel() = 0;
};


class SineSignalModel : public SignalModel {
public:
    virtual Hertz getCarrierFrequency() const;
    virtual Hertz getSampleRate() const;
    virtual void setCarrierFrequency(Hertz carrier);
    virtual void setSampleRate(Hertz sampleRate);
    virtual ~SineSignalModel() = 0;
protected:
    Hertz carrierFreq_;
    Hertz sampleRate_;
};


class SineSignal : public SineSignalModel {
public:
    SineSignal();
    SineSignal(Hertz carrierFreq, Radians phase, Hertz sampleRate);

    void sample(size_t sampleCount);
    const std::vector<double>& getTimeSamples() const;
    const std::vector<double>& getAmplitudeSamples() const;
    void clear();

private:
    Radians phase_;
    std::vector<double> timeSamples_;
    std::vector<double> amplitudeSamples_;
};


class SineSignalASK : public SineSignalModel {
public:
    SineSignalASK(Hertz carrierFreq, Radians phase, Hertz sampleRate,
                  size_t bitRate);
    void sample(size_t bitCount);
    const std::vector<double>& getTimeSamples() const;
    const std::vector<double>& getAmplitudeSamples() const;
    void clear();
    void generateDelayed(SignalModel::Seconds delay, 
                         SignalModel::Seconds duration);
    SineSignalASK getReferenceSignal(SignalModel::Seconds delay, 
                                     SignalModel::Seconds duration);
    std::vector<int> bits;
private:
    Radians phase_;
    size_t bitRate_;

    std::vector<double> timeSamples_;
    std::vector<double> amplitudeSamples_;
};

#if 0
std::vector<double> getReferenceSineSignal(const SineSignalModel& srcSignal, 
                                           SignalModel::Seconds srcDelay, 
                                           SignalModel::Seconds duration);
#endif

std::vector<SamplePoint<int, double>> computeCrossCorrelation(const std::vector<double>& sequenceA, 
                                                              const std::vector<double>& sequenceB);

std::vector<int> generateRandomBits(int size);

std::vector<double> addNoise(const std::vector<double>& amplitudes, double signalToNoiseRatio);


#endif