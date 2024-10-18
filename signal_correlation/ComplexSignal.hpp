#ifndef COMPLEXSIGNAL_HPP
#define COMPLEXSIGNAL_HPP

#include <algorithm>
#include <complex>
#include <random>
#include "SignalModel.hpp"

class ComplexSignal : public SignalModel<double, std::complex<double>> {
public:
	ComplexSignal(UnitDSP::Hertz sampleRate, UnitDSP::Radians phase)
		: sampleRate{ sampleRate }
		, phase{ phase }
	{}
	
	UnitDSP::Hertz getSampleRate() { return sampleRate;  }

	UnitDSP::Radians phase;
private:
	UnitDSP::Hertz sampleRate;
};

class ComplexSignalQPSKSampler : public ComplexSignal {
public:
	using IQ = std::complex<double>;	// I component - real, Q component - imaginary
	using IQSamples = Samples<UnitDSP::Seconds, IQ>;
	using SymbolBits = std::pair<int, int>;
	ComplexSignalQPSKSampler(UnitDSP::Hertz sampleRate, UnitDSP::Radians phase, double bitRate)
		: ComplexSignal(sampleRate, phase)
		, bitRate_{ bitRate }
	{}
	
	IQSamples sample(size_t symbolCount) {
		IQSamples samples;
		
		UnitDSP::Seconds dt{ 1.0 / getSampleRate() };
		UnitDSP::Seconds symbolInterval{ 2.0 * 1.0 / bitRate_ };
		UnitDSP::Seconds time{};
		size_t symbolIndex{ 0 };
		size_t bitSequenceSize = bits_.size();

		while (symbolIndex < symbolCount) {
			size_t bitIndex{ 2 * symbolIndex };
			SymbolBits sBits{ bits_[bitIndex % bitSequenceSize], bits_[(bitIndex + 1) % bitSequenceSize] };
			samples.timeSamples.push_back(time);
			samples.valueSamples.push_back(sampleSymbol(sBits));
			time += dt;
			symbolIndex = static_cast<size_t>(time / symbolInterval);
		}
		return samples;
	}
	
	IQSamples sample() { return sample(bits_.size() / 2); }

	IQ sampleSymbol(const SymbolBits& bits) {
		if (bits.first == 0 && bits.second == 0)
			return { -1.0, -1.0 };
		else if (bits.first == 1 && bits.second == 0)
			return { 1.0, -1.0 };
		else if (bits.first == 0 && bits.second == 1)
			return { -1.0, 1.0 };
		else
			return { 1.0, 1.0 };
	}

	void setBits(const std::vector<int>& bits) { bits_ = bits; }
	std::vector<int> getBits() { return bits_; }
private:
	double bitRate_;
	std::vector<int> bits_;
};


Samples<int, std::complex<double>> computeComplexCrossCorrelation(const std::vector<std::complex<double>>& sequenceA,
																  const std::vector<std::complex<double>>& sequenceB)
{
	if (sequenceA.size() < sequenceB.size()) {
		auto result = computeComplexCrossCorrelation(sequenceB, sequenceA);
		// TODO:
		// inverse delay points?
		return result;
	}
	int sizeA{ static_cast<int>(sequenceA.size()) };
	int sizeB{ static_cast<int>(sequenceB.size()) };
	Samples<int, std::complex<double>> crossCorrelation;
	crossCorrelation.timeSamples.reserve(sizeA - sizeB);
	crossCorrelation.valueSamples.reserve(sizeA - sizeB);

	for (int i{ 0 }; i <= sizeA - sizeB; ++i) {
		std::complex<double> sum{ 0.0, 0.0 };
		for (int j{ 0 }; j < sizeB; ++j) {
			sum += sequenceA[j + i] * std::conj(sequenceB[j]);
		}
		crossCorrelation.timeSamples.push_back(i);
		crossCorrelation.valueSamples.push_back(sum);
	}
	return crossCorrelation;
}

std::vector<std::complex<double>> getMatchedFilter(const std::vector<std::complex<double>>& samples) {
	std::vector<std::complex<double>> filterSamples{};
	std::transform(samples.rbegin(), samples.rend(), std::back_inserter(filterSamples),
		[](const std::complex<double>& elt) {
			return std::conj(elt);
		}
	);
	return filterSamples;
}

std::vector<std::complex<double>> addComplexNoise(const std::vector<std::complex<double>>& amplitudes, UnitDSP::dB signalToNoiseRatio) {
    std::mt19937 mt{ std::random_device{}() };
    std::normal_distribution nd(0.0, 1.0);

    auto size = amplitudes.size();
    std::vector<std::complex<double>> noiseSamples;
    noiseSamples.reserve(size);
    double signalEnergy{ 0.0 };
    double noiseEnergy{ 0.0 };
	
	auto squareComplex = [](const std::complex<double>& val) {
		return val.real() * val.real() + val.imag() * val.imag();
	};

	for (size_t i{ 0 }; i < size; ++i) {
		noiseSamples.push_back({ nd(mt), nd(mt) });
		noiseEnergy += squareComplex(noiseSamples[i]);
        signalEnergy += squareComplex(amplitudes[i]);
    }

    double noiseSampleScaleFactor{ std::sqrt(signalEnergy / noiseEnergy * std::pow(10.0, -signalToNoiseRatio / 10.0)) };

    std::vector<std::complex<double>> noisyAmplitudes = amplitudes;
    for (size_t i{ 0 }; i < size; ++i) {
        noisyAmplitudes[i] += noiseSampleScaleFactor * noiseSamples[i];
    }
    return noisyAmplitudes;
}

#endif