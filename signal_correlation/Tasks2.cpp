
#include <future>
#include <algorithm>
#include "Tasks2.hpp"

const std::pair<int, int> QPSKGoldCodeExperiment::filterBitPair[filterCount] = { {0,0}, {1,0}, {0,1}, {1,1} };

Samples<UnitDSP::dB, double> doQPSKGoldCodeStatExperiment(UnitDSP::Hertz sampleRate,
														  double bitRate,
														  size_t bitCount,
														  UnitDSP::dB lowSNR,
														  UnitDSP::dB highSNR,
														  size_t stepCountSNR,
														  size_t repsPerSNR,
														  float* statProgress)
{
	if (sampleRate < 0.0 || bitRate < 0.0 || bitCount % 2 == 1 
		|| highSNR <= lowSNR || stepCountSNR < 2 || repsPerSNR < 1)
		throw std::runtime_error("invalid parameters");

    *statProgress = 0.0;
    float statProgressStep = 1.0 / stepCountSNR;
    std::mutex progressMutex; // guards statProgress

	double snrStep{ (highSNR - lowSNR) / (stepCountSNR - 1) };
	Samples<UnitDSP::dB, double> result;
	for (size_t i{ 0 }; i < stepCountSNR; ++i) {
		result.timeSamples.push_back(lowSNR + i * snrStep);
		result.valueSamples.push_back(1.0);
	}
	auto subExperiment = [=, &result, &progressMutex](size_t start, size_t end) {
		for (size_t i{ start }; i < end; ++i) {
			
			double accumError{ 0.0 };
			for (size_t j{ 0 }; j < repsPerSNR; ++j) {

                QPSKGoldCodeExperiment gcExperiment(sampleRate, bitRate);
				accumError += gcExperiment.doExperiment(
					bitCount, result.timeSamples[i]).errorProb;
			}
			result.valueSamples[i] = accumError / repsPerSNR;
			if (statProgress) {
				std::lock_guard lock(progressMutex);
				*statProgress += statProgressStep;
			}
		}
	};
	size_t taskCount{ std::min(size_t{4}, result.valueSamples.size()) };
	size_t snrStepCountPerTask{ result.valueSamples.size() / taskCount };
	std::vector<std::future<void>> futures;
	for (size_t i{ 0 }; i < taskCount; ++i) {
		auto start = i * snrStepCountPerTask;
		auto end = (i + 1) * snrStepCountPerTask;
		if (i == (taskCount - 1))
			end = result.valueSamples.size();
		futures.push_back(std::async(std::launch::async, subExperiment, start, end));
	}
	for (auto& f : futures)
		f.get();
	return result;
}
