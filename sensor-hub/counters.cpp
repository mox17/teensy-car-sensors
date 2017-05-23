#include "counters.h"

static char const * CounterNames[] = {
    NAMED_COUNTERS(DO_NAME)
};

uint32_t Counters::counts[CounterCount];
bool Counters::flags[CounterCount];

    void Counters::inc(counters name)
    {
        counts[name]++;
        flags[name] = true;
    }

    void Counters::printCounter(counters name)
    {
        Serial.print(CounterNames[name]);
        Serial.println(counts[name]);
    }

    void Counters::printNZ()
    {
        for (unsigned i=0; i<CounterCount; i++)
        {
            if (counts[i])
            {
                printCounter((counters)i);
            }
        }
    }

    void Counters::printChanged()
    {
        for (unsigned i=0; i<CounterCount; i++)
        {
            if (flags[i])
            {
                flags[i] = false;
                printCounter((counters)i);
            }
        }
    }

    uint32_t Counters::count(counters name)
    {
        return counts[name];
    }

    char const * Counters::name(counters name)
    {
        return CounterNames[name];
    }

Counters cnt;
