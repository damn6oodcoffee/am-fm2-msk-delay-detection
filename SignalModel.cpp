#include "SignalModel.hpp"

SignalModel::~SignalModel() {}

SineSignal::SineSignal()
    : SineSignal(0.0, 0.0, 0.0)
{}

SineSignal::SineSignal(Hertz carrierFreq, Radians phase, Hertz sampleRate)
    : carrierFreq_{carrierFreq}
    , phase_{phase}
    , sampleRate_{sampleRate}
{}

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

const std::vector<double>& SineSignal::getTimeSamples() {
    return timeSamples_;
}

const std::vector<double>& SineSignal::getAmplitudeSamples() {
    return amplitudeSamples_;
}

void SineSignal::clear() {
    timeSamples_.clear();
    amplitudeSamples_.clear();
}


SineSignalASK::SineSignalASK(Hertz carrierFreq, Radians phase, Hertz sampleRate,
                             size_t bitRate)
    : carrierFreq_{carrierFreq}
    , phase_{phase}
    , sampleRate_{sampleRate}
    , bitRate_{bitRate}
{}

void SineSignalASK::sample(size_t bitCount) {
    const double dt{1.0 / sampleRate_};
    const double bitInterval{1.0 / bitRate_};
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

const std::vector<double>& SineSignalASK::getTimeSamples() {
    return timeSamples_;
}

const std::vector<double>& SineSignalASK::getAmplitudeSamples() {
    return amplitudeSamples_;
}

void SineSignalASK::clear() {
    timeSamples_.clear();
    amplitudeSamples_.clear();
}