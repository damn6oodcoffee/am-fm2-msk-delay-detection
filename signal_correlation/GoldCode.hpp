#ifndef GOLDCODE_HPP
#define GOLDCODE_HPP

#include <vector>
#include <cassert>

std::vector<int> generateGaloisMaxLengthSequence(unsigned long polynomial) {
	constexpr unsigned long startState{ 0x1 };
	unsigned long shiftRegister{ startState };
	unsigned period = 0;
	std::vector<int> bits;
	do {
		unsigned bit{ shiftRegister & 1 };
		bits.push_back(bit);
		shiftRegister >>= 1;
		if (bit)
			shiftRegister ^= polynomial;
		++period;
	} while (shiftRegister != startState);
	
	return bits;
}

class GoldCodeBitStreamTransformer {
public:
	GoldCodeBitStreamTransformer(size_t symbolBitLength)
		: symbolBitLength_{symbolBitLength}
		, goldCodeCount_{static_cast<size_t>(1) << symbolBitLength}
	{
		// x^5 + x^4 + x^3 + x^2 + 1
		auto maxLenSeq1 = generateGaloisMaxLengthSequence(0x1Eu);
		// x^5 + x^3 + 1
		auto maxLenSeq2 = generateGaloisMaxLengthSequence(0x14u);
		assert(maxLenSeq1.size() == maxLenSeq2.size());
		auto mlsLength = maxLenSeq1.size();
		for (size_t i{ 0 }; i < goldCodeCount_; ++i) {
			goldCodes_.push_back({});
			goldCodes_[i].reserve(mlsLength);
			for (size_t j{ 0 }; j < maxLenSeq1.size(); ++j) {
				goldCodes_[i].push_back((maxLenSeq1[j] + maxLenSeq2[(j + i) % mlsLength]) % 2);
			}
#if 1	// 31 bit -> 30 bit
			goldCodes_[i].pop_back();
#endif
		}
	}

	std::vector<int> getGoldCode(const std::vector<int>& symbolBitSequence) const {
		size_t index{};
		for (size_t i{ 0 }; i < symbolBitSequence.size(); ++i) {
			index ^= symbolBitSequence[i] ? 1 << i : 0;
		}
		return goldCodes_[index];
	}
	
	size_t getSymbolBitLength() const { return symbolBitLength_; }
	size_t getGoldCodeCount() const { return goldCodeCount_; }

private:
	size_t symbolBitLength_;
	size_t goldCodeCount_;
	std::vector<std::vector<int>> goldCodes_;
};

std::vector<int> transformBitsToGoldCode(const GoldCodeBitStreamTransformer& transformer,
									     const std::vector<int>& bits)
{
	auto symbolBitLength = transformer.getSymbolBitLength();
	if (bits.size() % symbolBitLength != 0)
		throw std::runtime_error("bits length and transformer are not compatible");
	std::vector<int> transformedBits;
	transformedBits.reserve((bits.size() / symbolBitLength) * 31);
	for (auto begIt = bits.begin(), endIt = bits.end(); begIt != endIt; begIt += symbolBitLength) {
		auto codeBits = transformer.getGoldCode(std::vector<int>(begIt, begIt + symbolBitLength));
		transformedBits.insert(transformedBits.end(), codeBits.begin(), codeBits.end());
	}
	return transformedBits;
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


#endif