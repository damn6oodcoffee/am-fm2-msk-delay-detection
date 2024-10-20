#ifndef GOLDCODE_HPP
#define GOLDCODE_HPP

#include <vector>
#include <cassert>

class GoldCodeBitStreamTransformer;

std::vector<int> generateGaloisMaxLengthSequence(unsigned long polynomial);

std::vector<int> transformBitsToGoldCode(const GoldCodeBitStreamTransformer& transformer,
									     const std::vector<int>& bits);

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



#endif