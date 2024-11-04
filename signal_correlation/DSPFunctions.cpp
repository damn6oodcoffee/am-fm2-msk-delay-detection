#include <numbers>
#include <random>
#include "DSPFunctions.hpp"


namespace DSP {
	using std::numbers::pi;
	void SlowDFT(std::vector<std::complex<double>>& data, int is) {
		int size{ static_cast<int>(data.size()) };
		auto dataBuf = data;
		for (int k{ 0 }; k < size; k++) {
			data[k] = 0;
			for (int n{ 0 }; n < size; n++) {
				data[k] += dataBuf[n] * exp(std::complex<double>(0, is * 2 * pi * k * n / size));
			}
		}
	}

	void fft(std::vector<std::complex<double>>& data, int is) {
		int size{ static_cast<int>(data.size()) };
		if (size == 1) return;
		if (size % 2 == 1) {
			SlowDFT(data, is);
			return;
		}

		std::vector<std::complex<double>> dataEven(size / 2);
		std::vector<std::complex<double>> dataOdd(size / 2);
		for (int i{ 0 }; i < size / 2; i++) {
			dataEven[i] = data[2 * i];
			dataOdd[i] = data[2 * i + 1];
		}

		fft(dataEven, is);
		fft(dataOdd, is);

		for (int k{ 0 }; k < size / 2; k++) {
			std::complex<double> oddTermExp = exp(std::complex<double>(0, is * 2 * pi * k / size));
			data[k] = dataEven[k] + oddTermExp * dataOdd[k];
			data[k + size / 2] = dataEven[k] - oddTermExp * dataOdd[k];
		}
	}

	void fft2D(std::vector<std::vector<std::complex<double>>>& data, int is) {
		int sizeDim1{ static_cast<int>(data.size()) };
		int sizeDim2{ static_cast<int>(data[0].size()) };
		std::vector<std::complex<double>> dataRow;
		std::vector<std::complex<double>> dataCol;

		for (int i{ 0 }; i < sizeDim1; i++) {
			dataRow.clear();
			dataRow = data[i];
			fft(dataRow, is);
			data[i] = dataRow;
		}

		for (int j{ 0 }; j < sizeDim2; j++) {
			dataCol.resize(sizeDim1);
			for (int k{ 0 }; k < sizeDim1; k++) {
				dataCol[k] = data[k][j];
			}
			fft(dataCol, is);

			for (int k{ 0 }; k < sizeDim1; k++) {
				data[k][j] = dataCol[k];
			}
		}
	}

	void ComputeSpectrogram(const std::vector<std::complex<double>>& data, std::vector<std::vector<double>>& spectrogram, int windowSize, int windowOverlap) {
		if (windowSize <= windowOverlap)
			return;
		std::vector<std::complex<double>> window(windowSize, { 0,0 });
		int size{ static_cast<int>(data.size()) };
		int windowStartPos{ 0 };
		while (windowStartPos < size) {
			for (int i{ 0 }; i < windowSize; i++) {
				if (windowStartPos + i < size)
					window[i] = data[windowStartPos + i];
				else
					window[i] = { 0,0 };
			}
			fft(window, -1);
			spectrogram.push_back(std::vector<double>(0));
			for (auto& val : window)
				spectrogram.back().push_back(sqrt(val.real() * val.real() + val.imag() * val.imag()));
			windowStartPos += windowSize - windowOverlap;
		}

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


}