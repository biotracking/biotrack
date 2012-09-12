#pragma once

#include <map>

class ModeAccumulator
{
public:
	ModeAccumulator();

	void pushValue(unsigned char value);

private:
	unsigned short maxCount;
	std::map<unsigned char, unsigned short> valueCounts;
};

