#include <random>
#include <cmath>
#include <cassert>
#include "SignalModel.hpp"


SineSignalModel::SineSignalModel(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase)
    : phase{ phase }
{
    setCarrierFrequency(carrierFreq);
    setSampleRate(sampleRate);
}

SineSignalModel::~SineSignalModel() {}

UnitDSP::Hertz SineSignalModel::getCarrierFrequency() const { return carrierFreq_; }

UnitDSP::Hertz SineSignalModel::getSampleRate() const { return sampleRate_; }

void SineSignalModel::setCarrierFrequency(UnitDSP::Hertz carrier) {
    if (carrier < 0.0)
        throw std::runtime_error("invalid carrier value");
    carrierFreq_ = carrier;
}

void SineSignalModel::setSampleRate(UnitDSP::Hertz sampleRate) {
    if (sampleRate <= 0.0)
        throw std::runtime_error("invalid sample rate value");
    sampleRate_ = sampleRate;
}


SineSignal::SineSignal()
    : SineSignal(0.0, 0.0, 0.0)
{}

SineSignal::SineSignal(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase)
    : SineSignalModel(carrierFreq, sampleRate, phase)
{}


Samples<UnitDSP::Seconds, double> SineSignal::sample(size_t sampleCount) {
    Samples<double, double> samples;
    double dt{ 1 / getSampleRate() };
    double carrier{ getCarrierFrequency() };
    double time{}, amplitude{};
    for (size_t i{ 0 }; i < sampleCount; ++i) {
        time = phase / (2 * m_pi * carrier);
        amplitude = 1.0 * sin(phase);
        samples.timeSamples.push_back(time);
        samples.valueSamples.push_back(amplitude);
        phase += 2 * m_pi * carrier * dt;
    }
    return samples;
}


// const std::vector<double>& SineSignal::getTimeSamples() const {
//     return timeSamples_;
// }

// const std::vector<double>& SineSignal::getAmplitudeSamples() const {
//     return amplitudeSamples_;
// }

// void SineSignal::clear() {
//     timeSamples_.clear();
//     amplitudeSamples_.clear();
// }




SineSignalBitSampler::SineSignalBitSampler(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate)
    : SineSignalModel(carrierFreq, sampleRate, phase)
{
    setBitRate(bitRate);
}

SineSignalBitSampler::~SineSignalBitSampler() {}

void SineSignalBitSampler::setBitRate(double bitRate) {
    if (bitRate <= 0.0)
        throw std::runtime_error("invalid bit rate value");
    bitRate_ = bitRate;
}

double SineSignalBitSampler::getBitRate() {
    return bitRate_;
}

void SineSignalBitSampler::setBits(const std::vector<int>& bits) {
    for (auto&& b : bits) {
        if (b != 0 && b != 1)
            throw std::runtime_error("invalid bit values");
    }
    bits_ = bits;
}

const std::vector<int>& SineSignalBitSampler::getBits() {
    return bits_;
}

Samples<UnitDSP::Seconds, double> SineSignalBitSampler::sample(size_t bitCount) {
    Samples<UnitDSP::Seconds, double> samples;
    UnitDSP::Seconds dt{ 1.0 / getSampleRate() };
    UnitDSP::Seconds bitInterval{ 1.0 / bitRate_ };
    UnitDSP::Seconds time{};
    double amplitude{};
    size_t bitIndex{ 0 };
    size_t bitSequenceSize = bits_.size();
    while (bitIndex < bitCount) {
 
        samples.timeSamples.push_back(time);
        samples.valueSamples.push_back(sampleBit(bits_[bitIndex % bitSequenceSize], time));

        time += dt;
        bitIndex = static_cast<int>(time / bitInterval);
    }
    return samples;
}




SineSignalASK::SineSignalASK(double lowAmplitude, double highAmplitude, UnitDSP::Hertz carrierFreq,
    UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate)
    : SineSignalBitSampler(carrierFreq, sampleRate, phase, bitRate)
    , lowAmp_{ lowAmplitude }
    , highAmp_{ highAmplitude }
{}

double SineSignalASK::sampleBit(int bit, UnitDSP::Seconds timePoint) {
    double maxAmplitude{ bit == 1 ? highAmp_ : lowAmp_ };
    double amplitude{ maxAmplitude * sin(phase) };
    phase = 2 * m_pi * getCarrierFrequency() * timePoint;
    return amplitude;
}


SineSignalBPSK::SineSignalBPSK(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate)
    : SineSignalBitSampler(carrierFreq, sampleRate, phase, bitRate)
{}

double SineSignalBPSK::sampleBit(int bit, UnitDSP::Seconds timePoint) {
    double amplitude{ sin(phase + bit * m_pi) };
    phase = 2 * m_pi * getCarrierFrequency() * timePoint;
    return amplitude;
}


SineSignalMSK::SineSignalMSK(UnitDSP::Hertz carrierFreq, UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate)
    : SineSignalBitSampler(carrierFreq, sampleRate, phase, bitRate)
    , frequencyDiff_{ bitRate / 2.0 }
{}

double SineSignalMSK::sampleBit(int bit, UnitDSP::Seconds timePoint) {
    UnitDSP::Seconds dt{ 1.0 / getSampleRate() };
    double amplitude{ sin(phase) };
    if (bit == 0) {
        phase += 2 * m_pi * getCarrierFrequency() * dt;
    }
    else {
        phase += 2 * m_pi * (getCarrierFrequency() + frequencyDiff_) * dt;
    }
    return amplitude;
}



void DelayedSineSignalBitSampler::generate(SineSignalBitSampler& sampler,
    UnitDSP::Seconds delay,
    UnitDSP::Seconds duration)
{
    if (delay < 0.0 || duration < 0.0)
        throw std::runtime_error("invalid parameters");
    delay_ = delay;
    duration_ = duration;
    generateDelayed(sampler);
    extractReference(sampler);
}

void DelayedSineSignalBitSampler::generateDelayed(SineSignalBitSampler& sampler) {
    UnitDSP::Seconds bitInterval{ 1.0 / sampler.getBitRate() };
    size_t bitCount{ static_cast<size_t>(std::ceil(2 * (delay_ + duration_) / bitInterval)) };
    sampler.setBits(generateRandomBits(bitCount));
    delayedSignal = sampler.sample(bitCount);
}

void DelayedSineSignalBitSampler::extractReference(SineSignalBitSampler& sampler) {
    UnitDSP::Seconds dt{ 1.0 / sampler.getSampleRate() };
    size_t srcDelayInSamples{ static_cast<size_t>(delay_ / dt) };
    size_t durationInSamples{ static_cast<size_t>(duration_ / dt) };

    auto begItVal = delayedSignal.valueSamples.begin();
    auto endItVal = delayedSignal.valueSamples.begin();
    std::advance(begItVal, srcDelayInSamples);
    std::advance(endItVal, srcDelayInSamples + durationInSamples);

    auto begItTime = delayedSignal.timeSamples.begin();
    auto endItTime = delayedSignal.timeSamples.begin();
    std::advance(begItTime, srcDelayInSamples);
    std::advance(endItTime, srcDelayInSamples + durationInSamples);

    refSignal = Samples<UnitDSP::Seconds, double>(std::vector<UnitDSP::Seconds>(begItTime, endItTime),
        std::vector<double>(begItVal, endItVal));

    assert(refSignal.valueSamples.size() == refSignal.timeSamples.size());
}

const Samples<UnitDSP::Seconds, double>& DelayedSineSignalBitSampler::getReferenceSamples() const {
    return refSignal;
}

const Samples<UnitDSP::Seconds, double>& DelayedSineSignalBitSampler::getDelayedSamples() const {
    return delayedSignal;
}


Samples<UnitDSP::Seconds, double> getBitSamples(SineSignalBitSampler& sampler) {
    auto bits = sampler.getBits();
    auto bitInterval = 1.0 / sampler.getBitRate();
    Samples<double, double> bitSamples{};
    for (size_t i{ 0 }; i < bits.size(); ++i) {
        auto amp = bits[i] == 1 ? bits[i] : -1;
        bitSamples.timeSamples.push_back(i * bitInterval);
        bitSamples.valueSamples.push_back(0);
        bitSamples.timeSamples.push_back(i * bitInterval);
        bitSamples.valueSamples.push_back(amp);
        bitSamples.timeSamples.push_back((i + 1) * bitInterval);
        bitSamples.valueSamples.push_back(amp);
        bitSamples.timeSamples.push_back((i + 1) * bitInterval);
        bitSamples.valueSamples.push_back(0);
    }
    return bitSamples;
}

Samples<int, double> computeCrossCorrelation(const std::vector<double>& sequenceA,
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
    Samples<int, double> crossCorrelation;
    crossCorrelation.timeSamples.reserve(sizeA - sizeB);
    crossCorrelation.valueSamples.reserve(sizeA - sizeB);

    for (int i{ 0 }; i <= sizeA - sizeB; ++i) {
        double sum{ 0 };
        for (int j{ 0 }; j < sizeB; ++j) {
            sum += sequenceA[j + i] * sequenceB[j];
        }
        crossCorrelation.timeSamples.push_back(i);
        crossCorrelation.valueSamples.push_back(sum);
    }
    return crossCorrelation;
}



std::vector<int> generateRandomBits(size_t size) {
    std::mt19937 mt{ std::random_device{}() };
    std::uniform_int_distribution uniform_dist(0, 1);
    std::vector<int> bits;
    bits.reserve(size);
    for (size_t i{ 0 }; i < size; ++i) {
        bits.push_back(uniform_dist(mt));
    }
    return bits;
}


std::vector<double> addNoise(const std::vector<double>& amplitudes, UnitDSP::dB signalToNoiseRatio) {
    std::mt19937 mt{ std::random_device{}() };
    std::normal_distribution nd(0.0, 1.0);

    auto size = amplitudes.size();
    std::vector<double> noiseSamples;
    noiseSamples.reserve(size);
    double signalEnergy{ 0.0 };
    double noiseEnergy{ 0.0 };

    for (size_t i{ 0 }; i < size; ++i) {
        noiseSamples.push_back(nd(mt));
        noiseEnergy += noiseSamples[i] * noiseSamples[i];
        signalEnergy += amplitudes[i] * amplitudes[i];
    }

    double noiseSampleScaleFactor{ std::sqrt(signalEnergy / noiseEnergy * std::pow(10.0, -signalToNoiseRatio / 10.0)) };

    std::vector<double> noisyAmplitudes = amplitudes;
    for (size_t i{ 0 }; i < size; ++i) {
        noisyAmplitudes[i] += noiseSampleScaleFactor * noiseSamples[i];
    }
    return noisyAmplitudes;
}









