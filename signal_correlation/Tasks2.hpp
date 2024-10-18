#ifndef TASKS2_HPP
#define TASKS2_HPP

#include "ComplexSignal.hpp"
#include "GoldCode.hpp"


class QPSKGoldCodeExperiment {
public:
	static constexpr size_t filterCount{ 4 };
	struct ExperimentResult {
		Samples<UnitDSP::Seconds, double> I;
		Samples<UnitDSP::Seconds, double> Q;
		Samples<UnitDSP::Seconds, double> filterConv[filterCount];
	};

	struct StatResult {};

	QPSKGoldCodeExperiment(UnitDSP::Hertz sampleRate, double bitRate)
		: goldCodeTransformer_(2)
		, qpskSampler_(sampleRate, 0.0, bitRate)
	{}

	ExperimentResult doExperiment(size_t bitCount, UnitDSP::dB SNR)
	{
		auto bits = generateRandomBits(bitCount);
		auto goldCodeBits = transformBitsToGoldCode(goldCodeTransformer_, bits);
		qpskSampler_.setBits(goldCodeBits);
		samplesIQ_ = qpskSampler_.sample();
		samplesIQ_.valueSamples = addComplexNoise(samplesIQ_.valueSamples, SNR);
		computeGoldCodeMatchedFilters();
		computeMatchedFilterConvolutions();
		populateIQComponents();
		return result_;
	}

	
private:
	GoldCodeBitStreamTransformer goldCodeTransformer_;
	ComplexSignalQPSKSampler qpskSampler_;
	ComplexSignalQPSKSampler::IQSamples samplesIQ_;
	std::vector<ComplexSignalQPSKSampler::IQ> filterIQ_[filterCount];
	ExperimentResult result_;

	void computeGoldCodeMatchedFilters() {
		for (int i{ 0 }; i < filterCount; ++i) {
			std::vector<int> bitPair{ i & 1, i & (1 << 1) };
			qpskSampler_.setBits(goldCodeTransformer_.getGoldCode(bitPair));
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
};

#if 0
ExperimentResult singleGoldCodeExperiment(UnitDSP::Hertz sampleRate, 
										  size_t bitCount, 
									      double bitRate, 
										  UnitDSP::dB SNR)
{
	auto bits = generateRandomBits(bitCount);
	GoldCodeBitStreamTransformer goldCodeTransformer(2); // QPSK - 2 bits per symbol

	// bits = { 0, 0, 0, 0, 0 ,0 ,0 ,0 ,0 , 0, 0, 0 };
	// bits = std::vector<int>(bitCount, 0);

	auto goldCodeBits = transformBitsToGoldCode(goldCodeTransformer, bits);
	ComplexSignalQPSKSampler qpskSampler(sampleRate, 0.0, bitRate);
	qpskSampler.setBits(goldCodeBits);
	auto samplesIQ = qpskSampler.sample();
	qpskSampler.setBits(goldCodeTransformer.getGoldCode({ 0, 0 }));
	auto filterIQSamples1 = getMatchedFilter(qpskSampler.sample().valueSamples);
	qpskSampler.setBits(goldCodeTransformer.getGoldCode({ 0, 1 }));
	auto filterIQSamples2 = getMatchedFilter(qpskSampler.sample().valueSamples);
	qpskSampler.setBits(goldCodeTransformer.getGoldCode({ 1, 0 }));
	auto filterIQSamples3 = getMatchedFilter(qpskSampler.sample().valueSamples);
	qpskSampler.setBits(goldCodeTransformer.getGoldCode({ 1, 1 }));
	auto filterIQSamples4 = getMatchedFilter(qpskSampler.sample().valueSamples);
	ExperimentResult result;
	result.I.timeSamples = samplesIQ.timeSamples;
	std::transform(
		samplesIQ.valueSamples.begin(),
		samplesIQ.valueSamples.end(),
		std::back_inserter(result.I.valueSamples),
		[](auto& elt) {
			return elt.real();
		}
	);
	result.Q.timeSamples = samplesIQ.timeSamples;
	std::transform(
		samplesIQ.valueSamples.begin(),
		samplesIQ.valueSamples.end(),
		std::back_inserter(result.Q.valueSamples),
		[](auto& elt) {
			return elt.imag();
		}
	);
	double dt = 1.0 / sampleRate;
	auto filterCorrTemp = computeComplexConvolution(samplesIQ.valueSamples, filterIQSamples1);
	for (int i{ 0 }; i < filterCorrTemp.timeSamples.size(); ++i) {
		result.filterConv1.timeSamples.push_back(static_cast<double>(filterCorrTemp.timeSamples[i]) * dt);
		result.filterConv1.valueSamples.push_back(std::abs(filterCorrTemp.valueSamples[i]));
	}
	filterCorrTemp = computeComplexConvolution(samplesIQ.valueSamples, filterIQSamples2);
	for (int i{ 0 }; i < filterCorrTemp.timeSamples.size(); ++i) {
		result.filterConv2.timeSamples.push_back(static_cast<double>(filterCorrTemp.timeSamples[i]) * dt);
		result.filterConv2.valueSamples.push_back(std::abs(filterCorrTemp.valueSamples[i]));
	}
	filterCorrTemp = computeComplexConvolution(samplesIQ.valueSamples, filterIQSamples3);
	for (int i{ 0 }; i < filterCorrTemp.timeSamples.size(); ++i) {
		result.filterConv3.timeSamples.push_back(static_cast<double>(filterCorrTemp.timeSamples[i]) * dt);
		result.filterConv3.valueSamples.push_back(std::abs(filterCorrTemp.valueSamples[i]));
	}
	filterCorrTemp = computeComplexConvolution(samplesIQ.valueSamples, filterIQSamples4);
	for (int i{ 0 }; i < filterCorrTemp.timeSamples.size(); ++i) {
		result.filterConv4.timeSamples.push_back(static_cast<double>(filterCorrTemp.timeSamples[i]) * dt);
		result.filterConv4.valueSamples.push_back(std::abs(filterCorrTemp.valueSamples[i]));
	}
	return result;
}
#endif

#endif