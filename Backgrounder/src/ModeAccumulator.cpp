#include "ModeAccumulator.h"


ModeAccumulator::ModeAccumulator()
{
	maxCount = 0;
}

void ModeAccumulator::pushValue(unsigned char value)
{
	if(valueCounts.find(value) == valueCounts.end())
		valueCounts[value] = 1;
	else
		valueCounts[value]++;
}