/**
 * AuriolDecoder - 433MHz Auriol sensor decoder library implementation
 */

#include "AuriolDecoder.h"
#include <Arduino.h>

////////////////////////////////////////////////////////////
// CONFIGURATION
////////////////////////////////////////////////////////////

// Pulse timing in microseconds
#define TYPE1_SHORT_US 2400
#define TYPE1_LONG_US 4400
#define TYPE2_SHORT_US 1400
#define TYPE2_LONG_US 2400

#define GLITCH_MIN_US 600
#define TRAIN_RESET_US 15000
#define TRAIN_RESET_HOLD_COUNT 3
#define PROFILE_LOCK_MIN_SAMPLES 12
#define PROFILE_LOCK_FORCE_SAMPLES 24
#define PACKET_MIN_BITS 26
#define PACKET_MAX_BITS 52
#define DECODE_DUP_SUPPRESS_MS 1500

// ISR queue: timing deltas only, decode in loop() to keep ISR tiny.
#define ISR_QUEUE_SIZE 128
#define ISR_QUEUE_MASK (ISR_QUEUE_SIZE - 1)

////////////////////////////////////////////////////////////
// INTERNAL STATE
////////////////////////////////////////////////////////////

volatile uint16_t isrDurations[ISR_QUEUE_SIZE];
volatile uint8_t isrHead = 0;
volatile uint8_t isrTail = 0;

enum DecoderProfile : uint8_t
{
    PROFILE_UNKNOWN = 0,
    PROFILE_TYPE1 = 1,
    PROFILE_TYPE2 = 2,
};

static DecoderProfile activeProfile = PROFILE_UNKNOWN;
static bool waitForPacketStart = true;

static uint64_t workBits = 0;
static uint8_t workBitCount = 0;

static uint64_t lastCandidateBits = 0;
static uint8_t lastCandidateBitsCount = 0;
static DecoderProfile lastCandidateProfile = PROFILE_UNKNOWN;
static uint8_t lastCandidateRepeatCount = 0;
static bool lastCandidateConsensusSent = false;

static uint32_t lockScoreType1 = 0;
static uint32_t lockScoreType2 = 0;
static uint8_t lockSamples = 0;
static uint8_t trainResetStreak = 0;

static DecoderProfile lastPublishedProfile = PROFILE_UNKNOWN;
static uint64_t lastPublishedRawBits = 0;
static uint8_t lastPublishedBitCount = 0;
static unsigned long lastPublishedMs = 0;

static uint8_t rf_data_pin_global = 0;
static AuriolDecoderCallback decoder_callback = nullptr;

////////////////////////////////////////////////////////////
// ISR HANDLER
////////////////////////////////////////////////////////////

IRAM_ATTR void auriolDecoderISR()
{
    static unsigned long lastRisingTime = 0;

    unsigned long now = micros();
    if (lastRisingTime == 0)
    {
        lastRisingTime = now;
        return;
    }

    unsigned long rawDuration = now - lastRisingTime;
    lastRisingTime = now;

    if (rawDuration < GLITCH_MIN_US)
    {
        return;
    }

    uint16_t duration = (rawDuration > 65535UL) ? 65535 : (uint16_t)rawDuration;
    uint8_t nextHead = (isrHead + 1) & ISR_QUEUE_MASK;
    if (nextHead == isrTail)
    {
        return;
    }

    isrDurations[isrHead] = duration;
    isrHead = nextHead;
}

////////////////////////////////////////////////////////////
// INTERNAL HELPER FUNCTIONS
////////////////////////////////////////////////////////////

static inline bool shouldSuppressDecodedPublish(DecoderProfile profile, uint64_t rawBits, uint8_t bitCount)
{
    unsigned long now = millis();
    bool samePayload = (lastPublishedProfile == profile &&
                        lastPublishedRawBits == rawBits &&
                        lastPublishedBitCount == bitCount);

    if (samePayload && (now - lastPublishedMs) <= DECODE_DUP_SUPPRESS_MS)
    {
        return true;
    }

    lastPublishedProfile = profile;
    lastPublishedRawBits = rawBits;
    lastPublishedBitCount = bitCount;
    lastPublishedMs = now;
    return false;
}

static inline uint16_t absDiffU16(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static inline uint16_t timingTolUs(uint16_t nominal)
{
    uint16_t pct = nominal / 5; // 20%
    return (pct < 250) ? 250 : pct;
}

static inline bool inWindow(uint16_t value, uint16_t nominal)
{
    uint16_t tol = timingTolUs(nominal);
    return (value >= (nominal - tol) && value <= (nominal + tol));
}

static inline uint8_t classifySymbol(uint16_t dt, DecoderProfile profile)
{
    if (profile == PROFILE_TYPE1)
    {
        if (inWindow(dt, TYPE1_SHORT_US))
            return 0;
        if (inWindow(dt, TYPE1_LONG_US))
            return 1;
        return 0xFF;
    }
    if (profile == PROFILE_TYPE2)
    {
        if (inWindow(dt, TYPE2_SHORT_US))
            return 0;
        if (inWindow(dt, TYPE2_LONG_US))
            return 1;
        return 0xFF;
    }
    return 0xFF;
}

static inline uint16_t profileLongUs(DecoderProfile profile)
{
    return (profile == PROFILE_TYPE2) ? TYPE2_LONG_US : TYPE1_LONG_US;
}

static inline void resetPacketAssembler()
{
    workBits = 0;
    workBitCount = 0;
}

static inline void resetCandidateHistory()
{
    lastCandidateBits = 0;
    lastCandidateBitsCount = 0;
    lastCandidateProfile = PROFILE_UNKNOWN;
    lastCandidateRepeatCount = 0;
    lastCandidateConsensusSent = false;
}

static inline void resetProfileLock()
{
    activeProfile = PROFILE_UNKNOWN;
    lockScoreType1 = 0;
    lockScoreType2 = 0;
    lockSamples = 0;
    trainResetStreak = 0;
    waitForPacketStart = true;
    resetPacketAssembler();
    resetCandidateHistory();
}

static inline bool popIsrDuration(uint16_t *outDt)
{
    bool hasItem = false;
    noInterrupts();
    if (isrTail != isrHead)
    {
        *outDt = isrDurations[isrTail];
        isrTail = (isrTail + 1) & ISR_QUEUE_MASK;
        hasItem = true;
    }
    interrupts();
    return hasItem;
}

////////////////////////////////////////////////////////////
// DECODE FUNCTIONS
////////////////////////////////////////////////////////////

static inline void emitType1DecodeEvent(uint64_t rawBits, uint8_t bitCount)
{
    if (bitCount != 32 || !decoder_callback)
        return;

    if (shouldSuppressDecodedPublish(PROFILE_TYPE1, rawBits, bitCount))
        return;

    uint32_t raw32 = (uint32_t)(rawBits & 0xFFFFFFFFUL);
    uint8_t b0 = (raw32 >> 24) & 0xFF;
    uint8_t b1 = (raw32 >> 16) & 0xFF;
    uint8_t b2 = (raw32 >> 8) & 0xFF;

    uint16_t sensor_id = (uint16_t)b0;

    // Type 1: b1 low nibble + b2 forms signed 12-bit temp in 0.1C steps.
    int16_t rawTemp = ((int16_t)(b1 & 0x0F) << 8) | b2;
    if (rawTemp >= 2048)
        rawTemp -= 4096;

    float finalTemp = rawTemp / 10.0f;
    bool batteryOk = (b0 & 0x80) == 0; // bit 7: 1=Low, 0=OK

    // Type 1 has no humidity; pass 0
    (*decoder_callback)(sensor_id, 0, finalTemp, 0.0f, batteryOk);
}

static inline void emitType2DecodeEvent(uint64_t rawBits, uint8_t bitCount)
{
    if (bitCount != 36 || !decoder_callback)
        return;

    if (shouldSuppressDecodedPublish(PROFILE_TYPE2, rawBits, bitCount))
        return;

    // Treat the 36-bit payload as 9 nibbles: n0..n8 (MSB to LSB).
    uint64_t raw36 = rawBits & 0xFFFFFFFFFULL;
    uint8_t n0 = (raw36 >> 32) & 0x0F;
    uint8_t n1 = (raw36 >> 28) & 0x0F;
    uint8_t n2 = (raw36 >> 24) & 0x0F;
    uint8_t n3 = (raw36 >> 20) & 0x0F;
    uint8_t n4 = (raw36 >> 16) & 0x0F;
    uint8_t n5 = (raw36 >> 12) & 0x0F;
    uint8_t n7 = (raw36 >> 4) & 0x0F;
    uint8_t n8 = raw36 & 0x0F;

    uint16_t sensor_id = ((uint16_t)n0 << 4) | n1;
    uint8_t channel = (n2 & 0x03) + 1;
    uint16_t rawTemp = ((uint16_t)n3 << 8) | ((uint16_t)n4 << 4) | n5;
    float finalTemp = rawTemp / 10.0f;

    uint8_t humidity = (uint8_t)((n7 << 4) | n8);
    bool batteryOk = (n2 & 0x08) != 0;  // bit 3 of n2

    (*decoder_callback)(sensor_id, channel, finalTemp, humidity, batteryOk);
}

static inline void finalizeCandidate(bool allowEmit)
{
    if (workBitCount == 0)
        return;

    if (workBitCount < PACKET_MIN_BITS || workBitCount > PACKET_MAX_BITS)
    {
        resetPacketAssembler();
        return;
    }

    if (lastCandidateProfile == activeProfile &&
        lastCandidateBitsCount == workBitCount &&
        lastCandidateBits == workBits)
    {
        if (lastCandidateRepeatCount < 255)
            lastCandidateRepeatCount++;
    }
    else
    {
        lastCandidateProfile = activeProfile;
        lastCandidateBitsCount = workBitCount;
        lastCandidateBits = workBits;
        lastCandidateRepeatCount = 1;
        lastCandidateConsensusSent = false;
    }

    if (allowEmit && lastCandidateRepeatCount >= 2 && !lastCandidateConsensusSent)
    {
        // Process payloads only on first consensus event.
        if (activeProfile == PROFILE_TYPE1)
            emitType1DecodeEvent(workBits, workBitCount);
        else if (activeProfile == PROFILE_TYPE2)
            emitType2DecodeEvent(workBits, workBitCount);

        lastCandidateConsensusSent = true;
    }

    resetPacketAssembler();
}

////////////////////////////////////////////////////////////
// PUBLIC API
////////////////////////////////////////////////////////////

void auriolDecoderSetup(uint8_t rf_data_pin, AuriolDecoderCallback callback)
{
    rf_data_pin_global = rf_data_pin;
    decoder_callback = callback;

    // Attach the RF data ISR
    pinMode(rf_data_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(rf_data_pin), auriolDecoderISR, RISING);
}

void auriolDecoderProcess(void)
{
    uint16_t dt = 0;
    while (popIsrDuration(&dt))
    {
        if (dt >= TRAIN_RESET_US)
        {
            // Train reset boundaries are state transitions, not packet events.
            finalizeCandidate(false);
            resetPacketAssembler();
            waitForPacketStart = true;
            resetCandidateHistory();

            if (trainResetStreak < 255)
                trainResetStreak++;

            // Hold lock for a few long silences to reduce profile flapping.
            if (trainResetStreak >= TRAIN_RESET_HOLD_COUNT)
                resetProfileLock();
            continue;
        }

        trainResetStreak = 0;

        if (activeProfile == PROFILE_UNKNOWN)
        {
            uint16_t d1 = absDiffU16(dt, TYPE1_SHORT_US);
            uint16_t d2 = absDiffU16(dt, TYPE1_LONG_US);
            lockScoreType1 += (d1 < d2) ? d1 : d2;

            uint16_t d3 = absDiffU16(dt, TYPE2_SHORT_US);
            uint16_t d4 = absDiffU16(dt, TYPE2_LONG_US);
            lockScoreType2 += (d3 < d4) ? d3 : d4;

            if (lockSamples < 255)
                lockSamples++;

            bool canLock = false;
            if (lockSamples >= PROFILE_LOCK_MIN_SAMPLES)
            {
                uint32_t scoreDiff = (lockScoreType1 > lockScoreType2) ? (lockScoreType1 - lockScoreType2) : (lockScoreType2 - lockScoreType1);
                if (scoreDiff > ((uint32_t)lockSamples * 120))
                    canLock = true;
            }

            if (!canLock && lockSamples >= PROFILE_LOCK_FORCE_SAMPLES)
                canLock = true;

            if (canLock)
            {
                activeProfile = (lockScoreType1 <= lockScoreType2) ? PROFILE_TYPE1 : PROFILE_TYPE2;
                waitForPacketStart = true;
                resetCandidateHistory();
            }
            continue;
        }

        uint8_t bitVal = classifySymbol(dt, activeProfile);
        if (bitVal != 0xFF)
        {
            if (!waitForPacketStart)
            {
                workBits = (workBits << 1) | bitVal;
                if (workBitCount < 255)
                    workBitCount++;

                if (workBitCount >= 64)
                {
                    resetPacketAssembler();
                    waitForPacketStart = true;
                }
            }
            continue;
        }

        uint16_t gapFloor = (uint16_t)(profileLongUs(activeProfile) + (profileLongUs(activeProfile) / 2));
        if (dt >= gapFloor)
        {
            if (waitForPacketStart)
            {
                // First gap after lock marks a packet boundary; start collecting from next symbol.
                waitForPacketStart = false;
                resetPacketAssembler();
            }
            else
            {
                finalizeCandidate(true);
            }
            continue;
        }
    }
}

void auriolDecoderReset(void)
{
    resetProfileLock();
    isrHead = 0;
    isrTail = 0;
}
