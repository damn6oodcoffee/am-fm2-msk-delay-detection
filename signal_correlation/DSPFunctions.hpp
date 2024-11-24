#ifndef DSPFUNCTIONS_HPP
#define DSPFUNCTIONS_HPP

#include <complex>
#include <vector>
#include <numeric> 

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
        Samples(const std::vector<TypeX>& timeSamples, 
                const std::vector<TypeY>& valueSamples)
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
        
        void append(TypeX x, TypeY y) {
            timeSamples.push_back(x);
            valueSamples.push_back(y);
        }

        void pop() {
            timeSamples.pop_back();
            valueSamples.pop_back();
        }

        void clear() {
            timeSamples.clear();
            valueSamples.clear();
        }
        
        std::vector<TypeX> timeSamples;
        std::vector<TypeY> valueSamples;
    };

	
	using ComplexVec = std::vector<std::complex<double>>;
    using RealVec = std::vector<double>;
	ComplexVec fft(const ComplexVec& data);
	ComplexVec ifft(const ComplexVec& data);
   
    using ComplexMat2D = std::vector<std::vector<std::complex<double>>>;
    using RealMat2D = std::vector<std::vector<double>>;
    ComplexMat2D fft2D(const ComplexMat2D& data);
    ComplexMat2D ifft2D(const ComplexMat2D& data);

    ComplexVec fftshift(const ComplexVec& data);
    ComplexVec ifftshift(const ComplexVec & data);

    Samples<int, double> computeCrossCorrelation(const RealVec& sequenceA,
        const RealVec& sequenceB);

    std::vector<int> generateRandomBits(size_t size);

    RealVec addNoise(const RealVec& amplitudes, UnitDSP::dB SNR);

	Samples<int, std::complex<double>> computeComplexCrossCorrelation(
		const ComplexVec& sequenceA,
		const ComplexVec& sequenceB);

	ComplexVec getMatchedFilter(
		const ComplexVec& samples);

	ComplexVec addComplexNoise(
		const ComplexVec& amplitudes,
		UnitDSP::dB SNR);

	Samples<int, std::complex<double>> computeComplexConvolution(
		const ComplexVec& sequenceA,
		const ComplexVec& sequenceB);

    ComplexMat2D computeAmbiguityFunction(
        const ComplexVec& sequenceA,
        const ComplexVec& sequenceB);


    template<typename T>
    std::vector<T> linspace(T start, T stop, T step) {
        if (start < stop && step == static_cast<T>(0))
            throw std::runtime_error("");
        std::vector<T> result;
        while (start < stop) {
            result.push_back(start);
            start += step;
        }
        return result;
    }
    
    RealVec abs(const ComplexVec& data);
    RealMat2D abs(const ComplexMat2D& data);

    double maxValue(const RealMat2D& data);

    template <typename T>
    T meanValue(const std::vector<T>& data) {
        if (data.empty())
            return T{};
        return std::reduce(data.begin(), data.end()) / data.size();
    }

    template <typename T>
    T stdDeviation(const std::vector<T>& data) {
        auto mean = meanValue(data);
        auto size = data.size();
        auto variance = std::accumulate(data.begin(), data.end(), T{}, [mean, size](T sum, T next) {
            return sum + (next - mean) * (next - mean) / size;
            });
        return std::sqrt(variance);
    }

}

#endif