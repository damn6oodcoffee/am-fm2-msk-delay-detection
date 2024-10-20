
#include <stdexcept>
#include "GoldCode.hpp"


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
