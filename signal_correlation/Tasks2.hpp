#ifndef TASKS2_HPP
#define TASKS2_HPP

#include "ComplexSignal.hpp"
#include "GoldCode.hpp"

struct ExperimentResult {
	Samples<UnitDSP::Seconds, double> I;
	Samples<UnitDSP::Seconds, double> Q;
	Samples<UnitDSP::Seconds, double> filterConv1;
	Samples<UnitDSP::Seconds, double> filterConv2;
	Samples<UnitDSP::Seconds, double> filterConv3;
	Samples<UnitDSP::Seconds, double> filterConv4;
};

struct StatResult {};

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