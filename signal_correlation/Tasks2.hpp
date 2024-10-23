#ifndef TASKS2_HPP
#define TASKS2_HPP

#include "ComplexSignal.hpp"
#include "GoldCode.hpp"


class QPSKGoldCodeExperiment {
public:
	static constexpr size_t filterCount{ 4 };
	struct ExperimentResult {
		std::vector<char> sentBits;
		Samples<UnitDSP::Seconds, double> I;
		Samples<UnitDSP::Seconds, double> Q;
		Samples<UnitDSP::Seconds, double> filterConv[filterCount];
		Samples<UnitDSP::Seconds, double> clampedFilterConv[filterCount];
		Samples<UnitDSP::Seconds, double> derivative[filterCount];
		std::vector<char> receivedBits;
		double errorProb;
	};

	QPSKGoldCodeExperiment(UnitDSP::Hertz sampleRate, double bitRate)
		: goldCodeTransformer_(symbolBitLength_)
		, qpskSampler_(sampleRate, 0.0, bitRate)
	{}

	ExperimentResult doExperiment(size_t bitCount, UnitDSP::dB SNR)
	{
		if (bitCount % symbolBitLength_ != 0)
			throw std::runtime_error("only even number of bits is allowed");
		auto bits = generateRandomBits(bitCount);
		for (auto c : bits)
			result_.sentBits.push_back(c == 0 ? '0' : '1');
		auto goldCodeBits = transformBitsToGoldCode(goldCodeTransformer_, bits);
		qpskSampler_.setBits(goldCodeBits);
		samplesIQ_ = qpskSampler_.sample();
		samplesIQ_.valueSamples = addComplexNoise(samplesIQ_.valueSamples, SNR);
		populateIQComponents();
		computeGoldCodeMatchedFilters();
		computeMatchedFilterConvolutions();
		movingAverage__(10);
		clampFilterConvolutions();
		populateDerivatives__();
		extractReceivedBits();
		//extractReceivedBitsWithTimeSync();
		computeErrorProbability();
		return result_;
	}

	
private:
	static constexpr size_t symbolBitLength_{ 2 };
	GoldCodeBitStreamTransformer goldCodeTransformer_;
	ComplexSignalQPSKSampler qpskSampler_;
	ComplexSignalQPSKSampler::IQSamples samplesIQ_;
	std::vector<ComplexSignalQPSKSampler::IQ> filterIQ_[filterCount];
	const static std::pair<int, int> filterBitPair[filterCount];
	ExperimentResult result_;

	void populateIQComponents() {
		result_.I.timeSamples = samplesIQ_.timeSamples;
		std::transform(
			samplesIQ_.valueSamples.begin(),
			samplesIQ_.valueSamples.end(),
			std::back_inserter(result_.I.valueSamples),
			[](auto& elt) {
				return elt.real();
			}
		);
		result_.Q.timeSamples = samplesIQ_.timeSamples;
		std::transform(
			samplesIQ_.valueSamples.begin(),
			samplesIQ_.valueSamples.end(),
			std::back_inserter(result_.Q.valueSamples),
			[](auto& elt) {
				return elt.imag();
			}
		);
	}

	void computeGoldCodeMatchedFilters() {
		for (int i{ 0 }; i < filterCount; ++i) {
			qpskSampler_.setBits(
				goldCodeTransformer_.getGoldCode({ filterBitPair[i].first, filterBitPair[i].second }));
			filterIQ_[i] = getMatchedFilter(qpskSampler_.sample().valueSamples);
		}
	}
	
	void computeMatchedFilterConvolutions() {
		double dt = 1.0 / qpskSampler_.getSampleRate();
		for (size_t i{ 0 }; i < filterCount; ++i) {
			auto filterCorrTemp = computeComplexConvolution(samplesIQ_.valueSamples, filterIQ_[i]);
			for (size_t j{ 0 }; j < filterCorrTemp.timeSamples.size(); ++j) {
				result_.filterConv[i].timeSamples.push_back(static_cast<double>(filterCorrTemp.timeSamples[j]) * dt);
				result_.filterConv[i].valueSamples.push_back(std::abs(filterCorrTemp.valueSamples[j]));
			}
		}
	}

	void clampFilterConvolutions() {
		constexpr double threshold{ 0.75 };
		for (size_t i{ 0 }; i < filterCount; ++i) {
			auto maxVal = std::max_element(result_.filterConv[i].valueSamples.begin(),
										   result_.filterConv[i].valueSamples.end());
			auto valThreshold = threshold * (*maxVal);
			Samples<UnitDSP::Seconds, double> clampedConv;
			clampedConv.timeSamples = result_.filterConv[i].timeSamples;
			clampedConv.valueSamples.reserve(clampedConv.timeSamples.size());
			for (auto elt : result_.filterConv[i].valueSamples)
				clampedConv.valueSamples.push_back(std::clamp(elt, valThreshold, *maxVal));
			result_.clampedFilterConv[i] = std::move(clampedConv);
		}
	}
	

	void extractReceivedBits() {
#if 0
		constexpr double threshold{ 0.2 };
		std::vector<std::tuple<double, double, int>> peaks;
		for (size_t i{ 0 }; i < filterCount; ++i) {

			auto maxVal = std::max_element(result_.derivative[i].valueSamples.begin(),
										   result_.derivative[i].valueSamples.end());
			auto valThreshold = threshold * (*maxVal);
			for (size_t j{ 0 }; j < result_.derivative[i].timeSamples.size(); ++j) {
				if (result_.derivative[i].valueSamples[j] > valThreshold)
					peaks.push_back({
						result_.derivative[i].timeSamples[j],
						result_.derivative[i].valueSamples[j],
						i });
			}
		}
#else
		std::vector<std::tuple<double, int>> peaks;
		for (size_t i{ 0 }; i < filterCount; ++i) {
			long long size = result_.clampedFilterConv[i].timeSamples.size() - 1;
			for (size_t j{ 1 }; j < size; ++j) {
				if (result_.clampedFilterConv[i].valueSamples[j] > result_.clampedFilterConv[i].valueSamples[j - 1]
					&& result_.clampedFilterConv[i].valueSamples[j] > result_.clampedFilterConv[i].valueSamples[j + 1])
				{
					peaks.push_back({ result_.clampedFilterConv[i].timeSamples[j], i });
				}
			}
		}
#endif
		std::sort(peaks.begin(), peaks.end(), [](auto& a, auto& b) {
			return std::get<0>(a) < std::get<0>(b);
		});
		for (auto& elt : peaks) {
			auto bitPair = filterBitPair[std::get<1>(elt)];
			result_.receivedBits.push_back(bitPair.first == 0 ? '0' : '1');
			result_.receivedBits.push_back(bitPair.second == 0 ? '0' : '1');
		}

	}

	void extractReceivedBitsWithTimeSync() {
		std::vector<std::tuple<double,int>> peaks;
		size_t windowSize{ filterIQ_[0].size() };
		size_t windowStart{ windowSize / 2 };
		size_t windowEnd{ 
			result_.filterConv[0].timeSamples.size() - 3*windowSize/2
		};
		for (size_t windowPos{ windowStart }; windowPos < windowEnd; windowPos += windowSize) {
			std::vector<std::tuple<double, int>> filterMaxOutputs; // tuple of value and filter index
			for (size_t i{ 0 }; i < filterCount; ++i) {
				auto begIt = result_.filterConv[i].valueSamples.begin();
				auto endIt = result_.filterConv[i].valueSamples.begin();
				std::advance(begIt, windowPos);
				std::advance(endIt, windowPos + windowSize);
				filterMaxOutputs.emplace_back(*std::max_element(begIt, endIt), i);
			}
			peaks.push_back(*std::max_element(filterMaxOutputs.begin(), filterMaxOutputs.end()));
		}
		for (auto& elt : peaks) {
			auto bitPair = filterBitPair[std::get<1>(elt)];
			result_.receivedBits.push_back(bitPair.first == 0 ? '0' : '1');
			result_.receivedBits.push_back(bitPair.second == 0 ? '0' : '1');
		}
	}

	void movingAverage__(int windowSize) {
		for (int i{ 0 }; i < filterCount; ++i) {
			std::vector<double> newVals;
			std::vector<UnitDSP::Seconds> newTime;
			for (int j{ 0 }; j < result_.filterConv[i].timeSamples.size() - windowSize; ++j) {
				double avg{ 0.0 };
				for (int k{ j }; k < j + windowSize; ++k) {
					avg += result_.filterConv[i].valueSamples[k];
				}
				avg /= windowSize;
				newVals.push_back(avg);
				newTime.push_back(result_.filterConv[i].timeSamples[j]);
			}
			result_.filterConv[i].timeSamples = std::move(newTime);
			result_.filterConv[i].valueSamples = std::move(newVals);
		}
	}

	Samples<UnitDSP::Seconds, double> findPeaks(const Samples<UnitDSP::Seconds, double>& data) {
		Samples<UnitDSP::Seconds, double> derivative;
		if (data.timeSamples.size() < 5)
			return {};
		double h{ data.timeSamples[1] - data.timeSamples[0] };
		for (size_t i{ 2 }; i < data.timeSamples.size() - 2; ++i) {
#if 1
			double derivativeValue{
				data.valueSamples[i + 1] - data.valueSamples[i - 1]
			};
#else
			double derivativeValue{
				(data.valueSamples[i - 1]
				 - 2 * data.valueSamples[i]
				 + data.valueSamples[i + 1])
			};
#endif
			derivative.timeSamples.push_back(data.timeSamples[i]);
			derivative.valueSamples.push_back(derivativeValue);
		}
		
		Samples<UnitDSP::Seconds, double> derivative2;
		for (size_t i{ 0 }; i < derivative.timeSamples.size() - 1; ++i) {
			if (derivative.valueSamples[i] > 0 && derivative.valueSamples[i + 1] < 1)
				derivative2.valueSamples.push_back(-derivative.valueSamples[i + 1] + derivative.valueSamples[i]);
			else
				derivative2.valueSamples.push_back(0.0);
		}
		derivative2.timeSamples = derivative.timeSamples;
		derivative2.timeSamples.pop_back();
		return derivative2;
	}

	void populateDerivatives__() {
		for (size_t i{ 0 }; i < filterCount; ++i) {
			result_.derivative[i] = findPeaks(result_.clampedFilterConv[i]);
		}
	}

	void computeErrorProbability() {
		size_t wrongBitCount{ 0 };
		size_t totalBits{ result_.sentBits.size() };
		for (size_t i{ 0 }; i < totalBits; ++i) {
			if (i >= result_.receivedBits.size() || result_.receivedBits[i] != result_.sentBits[i])
				wrongBitCount += 1;
		}
		result_.errorProb = static_cast<double>(wrongBitCount) / static_cast<double>(totalBits);
	}
};


Samples<UnitDSP::dB, double> doQPSKGoldCodeStatExperiment(UnitDSP::Hertz sampleRate,
														  double bitRate,
														  size_t bitCount,
														  UnitDSP::dB lowSNR,
														  UnitDSP::dB highSNR,
														  size_t stepCountSNR,
														  size_t repsPerSNR, 
														  float* statProgress);


#endif