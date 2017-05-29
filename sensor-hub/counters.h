/**
 * @brief Error counter encapsulation
 *
 * Define counters in the MACRO below.
 * Through a bit of macro magic, a single line is all it takes to declare
 * a new counter.
 * The name and value can be sent to remote side.
 */
#ifndef COUNTERS_H
#define COUNTERS_H
#include <Arduino.h>

#define NAMED_COUNTERS(NAMC) \
NAMC(rxErrorChecksum, "Number of packets with checksum errors"            ) \
NAMC(rxErrorTooShort, "Number of too short packets (not even a header)"   ) \
NAMC(rxErrorTooLong,  "Number of packets exceeding max length"            ) \
NAMC(rxErrorBuffer,   "No receive buffer available"                       ) \
NAMC(rxErrorDropped,  "Number of bytes thrown away while looking for sync") \
NAMC(rxErrorUnknown,  "Unknown command in buffer"                         ) \
NAMC(txErrorNoBuf,    "No free tx buffers"                                ) \
NAMC(txInfoSonarDrop, "Sonar message dropped (old msg still queued)"      ) \
NAMC(txInfoWheelDrop, "Wheel message dropped (old msg still queued)"      ) \
NAMC(txBytes,         "Bytes sent over telemetry link"                    ) \
NAMC(txPackets,       "Packets sent over telemetry link"                  ) \
NAMC(rxBytes,         "Bytes received over telemetry link"                ) \
NAMC(rxPackets,       "Packets received over telemetry link"              ) \
NAMC(pingFail,        "Sonar device failed"                               ) \
NAMC(badSeqCommand,   "Bad sonar sequence"                                ) \
NAMC(badSonarId,      "Sonar ID is out of range"                          ) \
NAMC(unknownCommand,  "Unknown CMD field in main message loop"            ) \
NAMC(badSeqLen,       "Wrong length of sonar sequence"                    ) \
NAMC(badSonarIdx,     "Wrong sonar ID in sequence"                        ) \
NAMC(rotBufFull,      "Rotation interrupt buffer full"                    ) \
NAMC(rotUpdSkip,      "Less than 5ms between updates"                     ) \
NAMC(errPing0,        "Error sending ping sonar 0"                        ) \
NAMC(errPing1,        "Error sending ping sonar 1"                        ) \
NAMC(errPing2,        "Error sending ping sonar 2"                        ) \
NAMC(errPing3,        "Error sending ping sonar 3"                        ) \
NAMC(errPing4,        "Error sending ping sonar 4"                        ) \
NAMC(errPing5,        "Error sending ping sonar 5"                        ) \
NAMC(CounterCount,    ""                                                  )

#define DO_NAME(e, t)  #e,
#define DO_ENUM(e, t)  e,

enum counters {
    NAMED_COUNTERS(DO_ENUM)
};

class Counters
{
public:
    static void inc(counters name);
    static void printCounter(counters name);
    static void printNZ();
    static void sendNZ();
    static void printChanged();
    static uint32_t count(counters name);
    static char const * name(counters name);

private:
    static uint32_t counts[CounterCount];
    static bool flags[CounterCount];
};

extern Counters cnt;

#endif // COUNTERS_H
