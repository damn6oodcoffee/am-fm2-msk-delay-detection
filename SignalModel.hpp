#include <vector>

constexpr double m_pi = 3.14159265358979323846;

class SignalModel {
public:
    using Hertz = double; 
    using Radians = double;
    virtual void sample(size_t count) = 0;
    virtual const std::vector<double>& getTimeSamples() = 0;
    virtual const std::vector<double>& getAmplitudeSamples() = 0;
    virtual ~SignalModel() = 0;
};


class SineSignal : public SignalModel {
public:
    SineSignal();
    SineSignal(Hertz carrierFreq, Radians phase, Hertz sampleRate);

    void sample(size_t sampleCount);
    const std::vector<double>& getTimeSamples();
    const std::vector<double>& getAmplitudeSamples();
    void clear();

private:
    Hertz carrierFreq_;
    Radians phase_;
    Hertz sampleRate_;

    std::vector<double> timeSamples_;
    std::vector<double> amplitudeSamples_;
};


class SineSignalASK : public SignalModel {
public:
    SineSignalASK(Hertz carrierFreq, Radians phase, Hertz sampleRate,
                  size_t bitRate);
    void sample(size_t bitCount);
    const std::vector<double>& getTimeSamples();
    const std::vector<double>& getAmplitudeSamples();
    void clear();
    std::vector<int> bits;
private:
    Hertz carrierFreq_;
    Radians phase_;
    Hertz sampleRate_;
    size_t bitRate_;

    std::vector<double> timeSamples_;
    std::vector<double> amplitudeSamples_;
};