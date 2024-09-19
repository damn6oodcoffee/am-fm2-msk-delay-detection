#include <random>
#include <cmath>
#include <cassert>
#include "SignalModel.hpp"

SignalModel::~SignalModel() {}


SineSignalModel::~SineSignalModel() {}

SignalModel::Hertz SineSignalModel::getCarrierFrequency() const { return carrierFreq_; }

SignalModel::Hertz SineSignalModel::getSampleRate() const { return sampleRate_; }

void SineSignalModel::setCarrierFrequency(Hertz carrier) {
    if (carrier < 0.0)
        throw std::runtime_error("invalid carrier value");
    carrierFreq_ = carrier;
}

void SineSignalModel::setSampleRate(Hertz sampleRate) {
    if (sampleRate <= 0.0) 
        throw std::runtime_error("invalid sample rate value");
    sampleRate_ = sampleRate;
}


SineSignal::SineSignal()
    : SineSignal(0.0, 0.0, 0.0)
{}

SineSignal::SineSignal(Hertz carrierFreq, Radians phase, Hertz sampleRate)
    : phase_{phase}
{
    setCarrierFrequency(carrierFreq);
    setSampleRate(sampleRate);
}

void SineSignal::sample(size_t sampleCount) {
    double dt{1 / sampleRate_};
    double time{}, amplitude{};
    for (size_t i{0}; i < sampleCount; ++i) {
        time = phase_ / (2 * m_pi * carrierFreq_);
        amplitude = 1.0 * sin(phase_);
        timeSamples_.push_back(time);
        amplitudeSamples_.push_back(amplitude);
        phase_ += 2 * m_pi * carrierFreq_ * dt;
    }
}

const std::vector<double>& SineSignal::getTimeSamples() const {
    return timeSamples_;
}

const std::vector<double>& SineSignal::getAmplitudeSamples() const {
    return amplitudeSamples_;
}

void SineSignal::clear() {
    timeSamples_.clear();
    amplitudeSamples_.clear();
}


SineSignalASK::SineSignalASK(Hertz carrierFreq, Radians phase, Hertz sampleRate,
                             size_t bitRate)
    : phase_{phase} 
    , bitRate_{bitRate}
{
    setCarrierFrequency(carrierFreq);
    setSampleRate(sampleRate);
}

void SineSignalASK::sample(size_t bitCount) {
    double dt{1.0 / sampleRate_};
    double bitInterval{1.0 / bitRate_};
    double time{}, amplitude{};
    size_t bitIndex{0};
    size_t bitSequenceSize = bits.size();
    while (bitIndex < bitCount) {
        time = phase_ / (2 * m_pi * carrierFreq_);
        amplitude = bits[bitIndex % bitSequenceSize] * sin(phase_);
        timeSamples_.push_back(time);
        amplitudeSamples_.push_back(amplitude);
        phase_ += 2 * m_pi * carrierFreq_ * dt;
        bitIndex = static_cast<int>(time / bitInterval);
    }
    return;
}

const std::vector<double>& SineSignalASK::getTimeSamples() const {
    return timeSamples_;
}

const std::vector<double>& SineSignalASK::getAmplitudeSamples() const {
    return amplitudeSamples_;
}

void SineSignalASK::clear() {
    timeSamples_.clear();
    amplitudeSamples_.clear();
}

void SineSignalASK::generateDelayed(SignalModel::Seconds delay, 
                                    SignalModel::Seconds duration) 
{
    if (delay < 0.0 || duration < 0.0)
        throw std::runtime_error("invalid parameters");

    double bitInterval{ 1.0 / bitRate_ };
    size_t bitCount{ static_cast<size_t>(std::ceil((delay + duration) / bitInterval)) };
    clear();
    bits = generateRandomBits(bitCount);
    sample(bitCount);
}

SineSignalASK SineSignalASK::getReferenceSignal(SignalModel::Seconds delay,
                                                SignalModel::Seconds duration)
{
    if (delay < 0.0 || duration < 0.0)
        throw std::runtime_error("invalid parameters");

    double dt{ 1.0 / getSampleRate() };
    size_t srcDelayInSamples{ static_cast<size_t>(delay / dt) };
    size_t durationInSamples{ static_cast<size_t>(duration / dt) };
    auto begIt = amplitudeSamples_.begin();
    auto endIt = amplitudeSamples_.begin();
    std::advance(begIt, srcDelayInSamples);
    std::advance(endIt, srcDelayInSamples + durationInSamples);
    SineSignalASK refSignal(carrierFreq_, phase_, sampleRate_, bitRate_);
    refSignal.amplitudeSamples_.assign(begIt, endIt);
    begIt = timeSamples_.begin();
    endIt = timeSamples_.begin();
    std::advance(begIt, srcDelayInSamples);
    std::advance(endIt, srcDelayInSamples + durationInSamples);
    refSignal.timeSamples_.assign(begIt, endIt);

    assert(refSignal.amplitudeSamples_.size() == refSignal.timeSamples_.size());
    return refSignal;
}

#if 0
std::vector<double> getReferenceSignal(const SineSignalModel& srcSignal, 
                                           SignalModel::Seconds srcDelay,
                                           SignalModel::Seconds duration) 
{
    if (srcDelay < 0.0 || duration < 0.0)
        throw std::runtime_error("invalid parameters");

    double dt{ 1.0 / srcSignal.getSampleRate() };
    size_t srcDelayInSamples{ static_cast<size_t>(srcDelay / dt) };
    size_t durationInSamples{ static_cast<size_t>(duration / dt) };
    auto begIt = srcSignal.getAmplitudeSamples().begin();
    auto endIt = srcSignal.getAmplitudeSamples().begin();
    std::advance(begIt, srcDelayInSamples);
    std::advance(endIt, srcDelayInSamples + durationInSamples);
    std::vector<double> refSignalAmplitudeSamples(begIt, endIt);
    return refSignalAmplitudeSamples;
}
#endif


#if 0
std::vector<SamplePoint<int, double>> computeCrossCorrelation(const std::vector<double>& sequenceA, 
                                                              const std::vector<double>& sequenceB) 
{
    if (sequenceA.size() < sequenceB.size()) {
        auto result = computeCrossCorrelation(sequenceB, sequenceA);
        // TODO:
        // inverse delay points?
        return result;
    }
    int posLagPointCount{ static_cast<int>(sequenceA.size()) };
    int negLagPointCount{ static_cast<int>(sequenceB.size()) };
    std::vector<SamplePoint<int, double>> crossCorrelation;
    crossCorrelation.reserve(negLagPointCount + posLagPointCount);

    // For negative lag values
    for (int i{ -(negLagPointCount - 1) }; i < 0; ++i) {
        double sum{ 0 };
        for (int j{ 0 }; j <= negLagPointCount - abs(i) - 1; ++j) {
            sum += sequenceA[j] * sequenceB[j + abs(i)];
        }
        crossCorrelation.emplace_back(i, sum);
    }
    // For positive lag values
    for (int i{ 0 }; i <= posLagPointCount - 1; ++i) {
        double sum{ 0 };
        for (int j{ 0 }; j < negLagPointCount && j <= posLagPointCount - i - 1; ++j) {
            sum += sequenceA[j + i] * sequenceB[j];
        }
        crossCorrelation.emplace_back(i, sum);
    }
    return crossCorrelation;
}
#else
std::vector<SamplePoint<int, double>> computeCrossCorrelation(const std::vector<double>& sequenceA, 
                                                              const std::vector<double>& sequenceB) 
{
    if (sequenceA.size() < sequenceB.size()) {
        auto result = computeCrossCorrelation(sequenceB, sequenceA);
        // TODO:
        // inverse delay points?
        return result;
    }
    int sizeA{ static_cast<int>(sequenceA.size()) };
    int sizeB{ static_cast<int>(sequenceB.size()) };
    std::vector<SamplePoint<int, double>> crossCorrelation;
    crossCorrelation.reserve(sizeA - sizeB);

    // For positive lag values
    for (int i{ 0 }; i <= sizeA - sizeB; ++i) {
        double sum{ 0 };
        for (int j{ 0 }; j < sizeB; ++j) {
            sum += sequenceA[j + i] * sequenceB[j];
        }
        crossCorrelation.emplace_back(i, sum);
    }
    return crossCorrelation;
}
#endif

std::vector<int> generateRandomBits(int size) {
    std::mt19937 mt{ std::random_device{}() };
    std::uniform_int_distribution uniform_dist(0, 1);
    std::vector<int> bits;
    bits.reserve(size);
    for (int i{0}; i < size; ++i) {
        bits.push_back(uniform_dist(mt));
    }
    return bits;
}

std::vector<double> addNoise(const std::vector<double>& amplitudes, double signalToNoiseRatio) {
    std::mt19937 mt{ std::random_device{}() };
    std::normal_distribution nd(0.0, 1.0);
    
    auto size = amplitudes.size();
    std::vector<double> noiseSamples;
    noiseSamples.reserve(size);
    double signalEnergy{0.0};   
    double noiseEnergy{0.0};

    for (size_t i{0}; i < size; ++i) {
        noiseSamples.push_back(nd(mt));
        noiseEnergy += noiseSamples[i] * noiseSamples[i];
        signalEnergy += amplitudes[i] * amplitudes[i];
    }

    double noiseSampleScaleFactor{ std::sqrt(signalEnergy / noiseEnergy * std::pow(10.0, -signalToNoiseRatio / 10.0)) };
    
    std::vector<double> noisyAmplitudes = amplitudes;
    for (size_t i{0}; i < size; ++i) {
        noisyAmplitudes[i] += noiseSampleScaleFactor * noiseSamples[i];
    }
    return noisyAmplitudes;
}









