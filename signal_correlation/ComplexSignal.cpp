
#include "ComplexSignal.hpp"

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

Samples<int, std::complex<double>> computeComplexConvolution(const std::vector<std::complex<double>>& sequenceA,
															 const std::vector<std::complex<double>>& sequenceB)
{
	Samples<int, std::complex<double>> output;
	auto outputSize = sequenceA.size() + 2 * sequenceB.size();
	output.timeSamples.reserve(outputSize);
	output.valueSamples.reserve(outputSize);
	for (size_t i{ 0 }; i < outputSize; ++i) {
		std::complex<double> sum{ 0.0, 0.0 };
		for (size_t j{ 0 }; j < sequenceB.size(); ++j) {
			long long indexA = static_cast<long long>(i) - static_cast<long long>(j);
			if (indexA < 0 || indexA >= static_cast<long long>(sequenceA.size()))
				sum += 0.0;
			else
				sum += sequenceB[j] * sequenceA[i - j];
		}
		output.timeSamples.push_back(i);
		output.valueSamples.push_back(sum);
	}
	return output;
}

