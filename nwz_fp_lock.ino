/*
 * nwz_fp_lock — управление замком по отпечатку на RP2040.
 *
 * Распиновка (Raspberry Pi Pico / RP2040):
 *   GPIO0  (Serial1 TX) -> сенсор RX
 *   GPIO1  (Serial1 RX) <- сенсор TX
 *   GPIO2                -> DIN кольца WS2812 (8 светодиодов)
 *   GPIO3                -> IN модуля реле
 *   GPIO4                -> buzzer (+ через транзистор при необходимости)
 *   GPIO5                <- кнопка ENROLL (другой контакт на GND)
 *   GPIO6                <- кнопка DELETE  (другой контакт на GND)
 *   Сенсор VCC/GND, реле VCC/GND, кольцо VCC/GND — по питанию.
 *
 * Конфигурация (ниже):
 *   BUZZER_ACTIVE     0 = пассивный buzzer (tone), 1 = активный (on/off)
 *   RELAY_ACTIVE_LOW  0 = реле вкл при HIGH, 1 = вкл при LOW (многие 5V модули)
 *
 * Зависимости (Менеджер библиотек Arduino):
 *   - HLK_ZW_Fingerprint_Universal (из этого репозитория)
 *   - Adafruit NeoPixel
 * Плата: ядро Earle F. Philhower III (arduino-pico), модель "Raspberry Pi Pico".
 */

#include <Arduino.h>
#include <HLK_fingerprint.h>
#include <Adafruit_NeoPixel.h>

// ─── Конфигурация ─────────────────────────────────────────────────────────────
#define BUZZER_ACTIVE      0
#define RELAY_ACTIVE_LOW   0

#define FP_RX_PIN          1
#define FP_TX_PIN          0
#define RING_PIN           2
#define RELAY_PIN          3
#define BUZZER_PIN         4
#define BTN_ENROLL_PIN     5
#define BTN_DELETE_PIN     6

#define RING_LED_COUNT     8
#define RING_BRIGHTNESS    60

#define DEBOUNCE_MS        20
#define DELETE_LONG_MS     3000
#define RELAY_ON_MS        3000
#define FP_POLL_MS         100
#define DUR_MATCH          1500
#define DUR_NOMATCH        1500
#define DUR_SUCCESS        2000
#define DUR_FAIL           2000
#define ENROLL_TIMEOUT_MS  20000
#define FP_BAUD            57600

// ─── Объекты ──────────────────────────────────────────────────────────────────
Adafruit_NeoPixel ring(RING_LED_COUNT, RING_PIN, NEO_GRB + NEO_KHZ800);
FingerprintModule fp(Serial1);

// ─── Состояние системы ────────────────────────────────────────────────────────
enum SysState : uint8_t { ST_IDLE = 0, ST_BUSY };
SysState sysState = ST_IDLE;
bool needLift = false;

// ─── Кольцо: режимы ───────────────────────────────────────────────────────────
enum RingMode : uint8_t {
    RM_OFF = 0, RM_READY, RM_SCANNING, RM_MATCH, RM_NOMATCH,
    RM_ENROLL_PLACE, RM_ENROLL_LIFT, RM_SUCCESS, RM_FAIL,
    RM_DELETE_WARN, RM_ERROR
};
RingMode ringMode = RM_OFF;
uint32_t ringModeStart = 0;
uint16_t ringModeDur = 0;
uint32_t lastRingShow = 0;
const uint16_t RING_SHOW_MS = 16;

// ─── Реле ─────────────────────────────────────────────────────────────────────
bool relayState = false;
uint32_t relayOffAt = 0;

// ─── Buzzer ───────────────────────────────────────────────────────────────────
struct Note { uint16_t freq; uint16_t ms; };
Note melodyBuf[8];
uint8_t melodyLen = 0, melodyIdx = 0;
uint32_t noteEnd = 0;
uint32_t beepEnd = 0;

// ─── Кнопки ───────────────────────────────────────────────────────────────────
struct Button {
    uint8_t pin;
    bool reading;
    bool stable;
    uint32_t lastChange;
};
Button btnEnroll = { BTN_ENROLL_PIN, false, false, 0 };
Button btnDelete = { BTN_DELETE_PIN, false, false, 0 };
bool enrollPrev = false;
bool deletePrev = false;
uint32_t deletePressStart = 0;
bool deleteLongTriggered = false;

uint32_t lastFpPoll = 0;

// ══════ LOW-LEVEL HELPERS ════════════════════════════════════════════════════

inline void relayWrite(bool on) {
    digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? !on : on);
}

inline void buzzerWrite(bool on, uint16_t freq = 2000) {
#if BUZZER_ACTIVE
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#else
    if (on) tone(BUZZER_PIN, freq);
    else    noTone(BUZZER_PIN);
#endif
}

// ══════ КОЛЬЦО ════════════════════════════════════════════════════════════════

void ringFill(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t c = ring.Color(r, g, b);
    for (uint8_t i = 0; i < RING_LED_COUNT; i++) ring.setPixelColor(i, c);
}

void ringSetMode(RingMode m, uint16_t durMs = 0) {
    ringMode = m;
    ringModeStart = millis();
    ringModeDur = durMs;
}

static uint8_t triBreathe(uint32_t t, uint16_t periodMs, uint8_t lo, uint8_t hi) {
    uint32_t ph = t % periodMs;
    uint16_t half = periodMs / 2;
    uint16_t v = (ph < half) ? (uint16_t)ph : (uint16_t)(periodMs - ph);
    return (uint8_t)map((long)v, 0L, (long)half, (long)lo, (long)hi);
}

void ringUpdate() {
    uint32_t now = millis();
    if (ringModeDur && (now - ringModeStart) >= ringModeDur) {
        ringSetMode(RM_READY);
    }

    switch (ringMode) {
        case RM_READY: {
            uint8_t lv = triBreathe(now, 2200, 20, 200);
            ringFill(0, (lv * 3) / 4, lv);
            break;
        }
        case RM_SCANNING: {
            ring.clear();
            uint8_t phase = (now / 70) % RING_LED_COUNT;
            for (uint8_t i = 0; i < RING_LED_COUNT; i++) {
                uint8_t d = (i + RING_LED_COUNT - phase) % RING_LED_COUNT;
                uint8_t lv = (d == 0) ? 255 : (uint8_t)(255 >> d);
                ring.setPixelColor(i, ring.Color(0, (lv * 2) / 3, lv));
            }
            break;
        }
        case RM_MATCH:
            ringFill(0, 255, 0);
            break;
        case RM_NOMATCH: {
            bool on = ((now / 120) % 2) == 0;
            if (on) ringFill(255, 0, 0); else ring.clear();
            break;
        }
        case RM_ENROLL_PLACE: {
            uint8_t lv = triBreathe(now, 900, 25, 255);
            ringFill(lv, lv, 0);
            break;
        }
        case RM_ENROLL_LIFT: {
            bool on = ((now / 150) % 2) == 0;
            if (on) ringFill(255, 180, 0); else ring.clear();
            break;
        }
        case RM_SUCCESS: {
            uint8_t phase = (now / 60) % RING_LED_COUNT;
            for (uint8_t i = 0; i < RING_LED_COUNT; i++) {
                ring.setPixelColor(i, (i == phase) ? ring.Color(0, 255, 0)
                                                   : ring.Color(0, 70, 0));
            }
            break;
        }
        case RM_FAIL: {
            bool on = ((now / 110) % 2) == 0;
            if (on) ringFill(255, 0, 0); else ring.clear();
            break;
        }
        case RM_DELETE_WARN: {
            bool on = ((now / 100) % 2) == 0;
            if (on) ringFill(255, 0, 0); else ringFill(120, 0, 160);
            break;
        }
        case RM_ERROR: {
            bool on = ((now / 300) % 2) == 0;
            if (on) ringFill(255, 0, 0); else ring.clear();
            break;
        }
        case RM_OFF:
        default:
            ring.clear();
            break;
    }

    if (now - lastRingShow >= RING_SHOW_MS) {
        lastRingShow = now;
        ring.show();
    }
}

// ══════ BUZZER ════════════════════════════════════════════════════════════════

void buzzerBeep(uint16_t freq, uint16_t ms) {
    melodyLen = 0;
    buzzerWrite(true, freq);
    beepEnd = millis() + ms;
}

void buzzerMelody(const Note *n, uint8_t len) {
    if (len > 8) len = 8;
    memcpy(melodyBuf, n, sizeof(Note) * len);
    melodyLen = len;
    melodyIdx = 0;
    noteEnd = 0;
    beepEnd = 0;
    buzzerWrite(false);
}

void buzzerUpdate() {
    uint32_t now = millis();
    if (melodyLen) {
        if (noteEnd == 0) {
            if (melodyIdx < melodyLen) {
                buzzerWrite(true, melodyBuf[melodyIdx].freq);
                noteEnd = now + melodyBuf[melodyIdx].ms;
            } else {
                buzzerWrite(false);
                melodyLen = 0;
            }
        } else if (now >= noteEnd) {
            buzzerWrite(false);
            melodyIdx++;
            noteEnd = 0;
        }
    } else if (beepEnd && now >= beepEnd) {
        buzzerWrite(false);
        beepEnd = 0;
    }
}

// ══════ РЕЛЕ ══════════════════════════════════════════════════════════════════

void relayOn() {
    relayWrite(true);
    relayState = true;
    relayOffAt = millis() + RELAY_ON_MS;
}

void relayOff() {
    relayWrite(false);
    relayState = false;
    relayOffAt = 0;
}

void relayUpdate() {
    if (relayState && millis() >= relayOffAt) relayOff();
}

// ══════ КНОПКИ ════════════════════════════════════════════════════════════════

bool debounceRead(Button &b) {
    bool raw = (digitalRead(b.pin) == LOW);
    uint32_t now = millis();
    if (raw != b.reading) {
        b.reading = raw;
        b.lastChange = now;
    }
    if (now - b.lastChange >= DEBOUNCE_MS) b.stable = b.reading;
    return b.stable;
}

// ══════ АНИМИРОВАННАЯ ЗАДЕРЖКА (держит кольцо и buzzer живыми) ════════════════

void animDelay(uint32_t ms) {
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < ms) {
        ringUpdate();
        buzzerUpdate();
        relayUpdate();
    }
}

// ══════ СЛОТЫ ОТПЕЧАТКОВ ══════════════════════════════════════════════════════

uint16_t firstFreeSlot() {
    const uint16_t MAXS = 256;
    bool states[MAXS];
    uint16_t cap = min((uint16_t)MAXS, fp.capacity());
    if (fp.getStorageMap(states, cap)) {
        for (uint16_t i = 0; i < cap; i++) if (!states[i]) return i;
        return 0xFFFF;
    }
    for (uint16_t i = 0; i < cap; i++) if (!fp.templateExists(i)) return i;
    return 0xFFFF;
}

uint16_t lastOccupiedSlot() {
    const uint16_t MAXS = 256;
    bool states[MAXS];
    uint16_t cap = min((uint16_t)MAXS, fp.capacity());
    if (fp.getStorageMap(states, cap)) {
        for (int16_t i = (int16_t)cap - 1; i >= 0; i--) if (states[i]) return (uint16_t)i;
        return 0xFFFF;
    }
    for (int16_t i = (int16_t)cap - 1; i >= 0; i--) if (fp.templateExists((uint16_t)i)) return (uint16_t)i;
    return 0xFFFF;
}

// ══════ РЕЗУЛЬТАТЫ СОПОВОСТАВЛЕНИЯ ════════════════════════════════════════════

void onMatch(int16_t id, uint16_t score) {
    (void)id; (void)score;
    needLift = true;
    relayOn();
    fp.ledFlash(FP_LED_GREEN, 2);
    ringSetMode(RM_MATCH, DUR_MATCH);
    static const Note ok[] = { {2400, 80}, {3000, 120} };
    buzzerMelody(ok, 2);
}

void onNoMatch() {
    needLift = true;
    fp.ledFlash(FP_LED_RED, 3);
    ringSetMode(RM_NOMATCH, DUR_NOMATCH);
    buzzerBeep(300, 350);
}

void doMatch() {
    sysState = ST_BUSY;
    ringSetMode(RM_SCANNING);
    uint16_t score = 0;
    int16_t id = fp.matchFingerprint(score, 3000);
    if (id >= 0)        onMatch(id, score);
    else if (id == -1)  onNoMatch();
    else { ringSetMode(RM_READY); buzzerBeep(400, 80); }
    sysState = ST_IDLE;
}

// ══════ ДОБАВЛЕНИЕ ОТПЕЧАТКА (пошагово, с анимацией) ═════════════════════════

int16_t doEnroll() {
    uint32_t t0 = millis();

    fp.ledSteady(FP_LED_YELLOW);
    ringSetMode(RM_ENROLL_PLACE);
    buzzerBeep(2000, 60);

    while (!fp.getImage()) {
        if (fp.lastCC != 0x02 && fp.lastCC != 0xFF && fp.lastCC != 0xFE) return -1;
        if (millis() - t0 > ENROLL_TIMEOUT_MS) return -1;
        animDelay(20);
    }
    if (!fp.image2Tz(1)) return -1;
    buzzerBeep(2200, 60);

    ringSetMode(RM_ENROLL_LIFT);
    fp.ledFlash(FP_LED_YELLOW, 3);
    animDelay(500);
    uint32_t tl = millis();
    do {
        animDelay(100);
        fp.getImage();
        if (millis() - t0 > ENROLL_TIMEOUT_MS) return -1;
    } while (fp.lastCC != 0x02 && (millis() - tl) < 4000);

    ringSetMode(RM_ENROLL_PLACE);
    fp.ledSteady(FP_LED_YELLOW);
    buzzerBeep(2000, 60);

    while (!fp.getImage()) {
        if (fp.lastCC != 0x02 && fp.lastCC != 0xFF && fp.lastCC != 0xFE) return -1;
        if (millis() - t0 > ENROLL_TIMEOUT_MS) return -1;
        animDelay(20);
    }
    if (!fp.image2Tz(2)) return -1;

    if (!fp.createModel()) return -1;

    uint16_t id = firstFreeSlot();
    if (id == 0xFFFF) return -1;
    if (!fp.storeTemplate(1, id)) return -1;
    return (int16_t)id;
}

void startEnroll() {
    if (fp.getTemplateCount() >= fp.capacity()) {
        ringSetMode(RM_FAIL, DUR_FAIL);
        buzzerBeep(300, 400);
        animDelay(DUR_FAIL);
        return;
    }

    sysState = ST_BUSY;
    int16_t id = doEnroll();

    if (id >= 0) {
        fp.ledFlash(FP_LED_GREEN, 2);
        ringSetMode(RM_SUCCESS, DUR_SUCCESS);
        static const Note ok[] = { {2000, 80}, {2500, 80}, {3000, 140} };
        buzzerMelody(ok, 3);
        animDelay(DUR_SUCCESS);
    } else {
        fp.ledFlash(FP_LED_RED, 3);
        ringSetMode(RM_FAIL, DUR_FAIL);
        buzzerBeep(300, 400);
        animDelay(DUR_FAIL);
    }

    sysState = ST_IDLE;
    fp.ledBreathing(FP_LED_BLUE);
    ringSetMode(RM_READY);
}

// ══════ УДАЛЕНИЕ ОТПЕЧАТКОВ ═══════════════════════════════════════════════════

void deleteLastFinger() {
    sysState = ST_BUSY;
    uint16_t id = lastOccupiedSlot();
    if (id == 0xFFFF) {
        fp.ledFlash(FP_LED_RED, 2);
        ringSetMode(RM_NOMATCH, DUR_NOMATCH);
        buzzerBeep(300, 300);
        animDelay(DUR_NOMATCH);
    } else if (fp.deleteFingerprint(id)) {
        fp.ledFlash(FP_LED_PURPLE, 2);
        ringSetMode(RM_SUCCESS, DUR_SUCCESS);
        buzzerBeep(2200, 120);
        animDelay(DUR_SUCCESS);
    } else {
        ringSetMode(RM_FAIL, DUR_FAIL);
        buzzerBeep(300, 400);
        animDelay(DUR_FAIL);
    }
    sysState = ST_IDLE;
    fp.ledBreathing(FP_LED_BLUE);
    ringSetMode(RM_READY);
}

void deleteAllFingers() {
    sysState = ST_BUSY;
    bool ok = fp.deleteAllFingerprints();
    if (ok) {
        fp.ledFlash(FP_LED_RED, 3);
        ringSetMode(RM_SUCCESS, DUR_SUCCESS);
        static const Note gone[] = { {2000, 110}, {1500, 110}, {1000, 160} };
        buzzerMelody(gone, 3);
        animDelay(DUR_SUCCESS);
    } else {
        ringSetMode(RM_FAIL, DUR_FAIL);
        buzzerBeep(300, 400);
        animDelay(DUR_FAIL);
    }
    sysState = ST_IDLE;
    fp.ledBreathing(FP_LED_BLUE);
    ringSetMode(RM_READY);
}

// ══════ SETUP / LOOP ══════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);

    ring.begin();
    ring.setBrightness(RING_BRIGHTNESS);
    ring.show();

    pinMode(BUZZER_PIN, OUTPUT);
    buzzerWrite(false);
    pinMode(RELAY_PIN, OUTPUT);
    relayOff();
    pinMode(BTN_ENROLL_PIN, INPUT_PULLUP);
    pinMode(BTN_DELETE_PIN, INPUT_PULLUP);

    Serial1.setRX(FP_RX_PIN);
    Serial1.setTX(FP_TX_PIN);
    Serial1.begin(FP_BAUD);

    ringSetMode(RM_ERROR);
    ringUpdate();

    Serial.println(F("FP init..."));
    if (!fp.begin()) {
        Serial.println(F("ERROR: fingerprint module not found"));
        buzzerBeep(200, 1000);
        while (true) {
            ringUpdate();
            buzzerUpdate();
        }
    }

    Serial.print(F("Ready. Slots used: "));
    Serial.print(fp.getTemplateCount());
    Serial.print(F(" / "));
    Serial.println(fp.capacity());

    fp.ledBreathing(FP_LED_BLUE);
    ringSetMode(RM_READY);
    buzzerBeep(2500, 80);
}

void loop() {
    uint32_t now = millis();
    ringUpdate();
    buzzerUpdate();
    relayUpdate();

    bool e = debounceRead(btnEnroll);
    bool dn = debounceRead(btnDelete);

    if (e && !enrollPrev && sysState == ST_IDLE) {
        startEnroll();
    }
    enrollPrev = e;

    if (dn) {
        if (!deletePrev) {
            deletePressStart = now;
            deleteLongTriggered = false;
            if (sysState == ST_IDLE) {
                ringSetMode(RM_DELETE_WARN);
                buzzerBeep(1500, 30);
            }
        } else if (sysState == ST_IDLE && !deleteLongTriggered) {
            if ((now - deletePressStart) >= DELETE_LONG_MS) {
                deleteLongTriggered = true;
                buzzerBeep(2600, 160);
            }
        }
    } else {
        if (deletePrev && sysState == ST_IDLE) {
            if (deleteLongTriggered) deleteAllFingers();
            else                      deleteLastFinger();
        }
    }
    deletePrev = dn;

    if (sysState == ST_IDLE && (now - lastFpPoll) >= FP_POLL_MS) {
        lastFpPoll = now;
        bool present = fp.isFingerPresent();
        if (!present) {
            needLift = false;
        } else if (!needLift) {
            doMatch();
        }
    }
}
