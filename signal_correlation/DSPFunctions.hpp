#ifndef DSPFUNCTIONS_HPP
#define DSPFUNCTIONS_HPP

#include <complex>
#include <vector>

namespace DSP {

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
            : timeSamples{ timeSamples }
            , valueSamples{ valueSamples }
        {}
        Samples(const Samples& other)
            : timeSamples{ other.timeSamples }
            , valueSamples{ other.valueSamples }
        {}
        Samples(Samples&& other) noexcept
            : timeSamples{ std::move(other.timeSamples) }
            , valueSamples{ std::move(other.valueSamples) }
        {}

        Samples& operator=(const Samples& other) {
            if (this == &other)
                return *this;
            timeSamples = other.timeSamples;
            valueSamples = other.valueSamples;
            return *this;
        }

        Samples& operator=(Samples&& other) noexcept {
            timeSamples = std::move(other.timeSamples);
            valueSamples = std::move(other.valueSamples);
            return *this;
        }

        std::vector<TypeX> timeSamples;
        std::vector<TypeY> valueSamples;
    };

	/**
	 * Slow DFT implementation for arbitrary data size.
	 *
	 * @data In/out parameter. Should contain data points to transform. Out goes transformed data.
	 * @is Direction of transform. Should be -1/1 (forward/inverse).
	 */
	void SlowDFT(std::vector<std::complex<double>>& data, int is);

	/**
	 * FFT implementation for data of size 2^x.
	 *
	 * @data In/out parameter. Should contain data points to transform. Out goes transformed data.
	 * @is Direction of transform. Should be -1/1 (forward/inverse).
	 */
	void fft(std::vector<std::complex<double>>& data, int is);

	/**
	 * 2D FFT implementation for data of size 2^x.
	 *
	 * @data In/out parameter. Should contain data points to transform. Out goes transformed data.
	 * @is Direction of transform. Should be -1/1 (forward/inverse).
	 */
	void fft2D(std::vector<std::vector<std::complex<double>>>& data, int is);

	/**
	 * Compute spectrogram for given data.
	 *
	 * @data In parameter. Should contain data points to transform.
	 * @spectrogram Out parameter for resulting spectrogram.
	 * @windowSize DFT window size.
	 * @windowOver Window overlap.
	 */
	void ComputeSpectrogram(const std::vector<std::complex<double>>& data,
		std::vector<std::vector<double>>& spectrogram,
		int windowSize,
		int windowOverlap);



    Samples<int, double> computeCrossCorrelation(const std::vector<double>& sequenceA,
        const std::vector<double>& sequenceB);

    std::vector<int> generateRandomBits(size_t size);

    std::vector<double> addNoise(const std::vector<double>& amplitudes, UnitDSP::dB signalToNoiseRatio);

	Samples<int, std::complex<double>> computeComplexCrossCorrelation(
		const std::vector<std::complex<double>>& sequenceA,
		const std::vector<std::complex<double>>& sequenceB);

	std::vector<std::complex<double>> getMatchedFilter(
		const std::vector<std::complex<double>>& samples);

	std::vector<std::complex<double>> addComplexNoise(
		const std::vector<std::complex<double>>& amplitudes,
		UnitDSP::dB signalToNoiseRatio);

	Samples<int, std::complex<double>> computeComplexConvolution(
		const std::vector<std::complex<double>>& sequenceA,
		const std::vector<std::complex<double>>& sequenceB);
}

#endif