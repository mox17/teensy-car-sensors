#include "counters.h"
#include "telemetry.h"

extern Telemetry messageHandling;

static char const * CounterNames[] = {
    NAMED_COUNTERS(DO_NAME)
};

uint32_t Counters::counts[CounterCount];
bool Counters::flags[CounterCount];

    /**
     * @brief Increment a counter
     *
     * A flag is maintained. This makes it possible to find
     * counters changed after a certain time.
     */
    void Counters::inc(counters name)
    {
        counts[name]++;
        flags[name] = true;
    }

    /**
     * @brief Print counter with specific enumeration
     */
    void Counters::printCounter(counters name)
    {
        const char spaces[] = "                      : ";
        unsigned l = strlen(CounterNames[name]);
        Serial.print(CounterNames[name]);
        Serial.print(spaces+l);
        Serial.println(counts[name]);
    }

    /**
     * @brief Print non-zero counters
     */
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

    /**
     * @brief Send non-zero counters over UART
     */
    void Counters::sendNZ()
    {
        for (unsigned i=0; i<CounterCount; i++)
        {
            if (counts[i])
            {
                messageHandling.errorCounter(counts[i],CounterNames[i]);
            }
        }
    }

    /**
     * @brief Print changed counters
     *
     * This function also resets the change flag.
     */
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

    /**
     * @brief Get a specific counter value
     */
    uint32_t Counters::count(counters name)
    {
        return counts[name];
    }

    /**
     * @brief Get a specific counter name as text
     */
    char const * Counters::name(counters name)
    {
        return CounterNames[name];
    }

Counters cnt; //!< This is the single object used t count
