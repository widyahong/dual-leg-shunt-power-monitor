/*
 * Dual-leg shunt power monitor
 * Target : ESP32 (Arduino-ESP32 core). No external libraries: only Arduino.h
 *          and standard C headers shipped with the ESP32 toolchain.
 *
 * Measures voltage, current and power of a single-phase, non-polarized load.
 * Each load terminal (leg A and leg B) has a shunt with a source-side tap and
 * a load-side tap, giving four ADC channels (see README.md).
 * On each leg, positive current flows from the source side to the load side.
 *
 * Every setting below is a plain variable that can be overridden at run time
 * over the serial port (type "help"). Overrides are not stored in flash: after
 * a reset the values below apply again.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// ---------------- Configuration (defaults, overridable via Serial) ----------------
enum Node { NODE_SRC_A, NODE_LOAD_A, NODE_SRC_B, NODE_LOAD_B, NODE_COUNT };
int            NODE_PINS[NODE_COUNT] = { 34, 35, 32, 33 };  // ADC1 pins, same order as Node

float          R_SERIES_OHM      = 150000.0f; // Rs, tap to ADC node
float          R_UP_OHM          = 10000.0f;  // Rup, ADC node to 3V3
float          R_DOWN_OHM        = 10000.0f;  // Rdn, ADC node to GND
float          R_SHUNT_A_OHM     = 0.05f;     // Shunt on leg A
float          R_SHUNT_B_OHM     = 0.05f;     // Shunt on leg B
float          VREF_MV           = 3300.0f;   // Actual 3V3 rail, in mV

uint32_t       WINDOW_MS         = 200;       // Integration window (50/60 Hz friendly)
uint32_t       ZERO_SAMPLES      = 1000;      // Samples used for the zero offsets
const bool     AUTO_ZERO_AT_BOOT = true;      // Needs a de-energized source at boot (compile-time)
float          CM_TRIM_A_PER_V   = 0.0f;      // Common-mode trim, leg A (A per V)
float          CM_TRIM_B_PER_V   = 0.0f;      // Common-mode trim, leg B (A per V)
float          MIN_CAL_VRMS_V    = 1.0f;      // Minimum tap voltage for a trim estimate

enum Mode { MODE_MEASURE, MODE_CALIBRATE };
Mode           RUN_MODE          = MODE_MEASURE;
bool           STREAM_OUTPUT     = true;      // Print one line per window; off = only on "read"

const uint32_t SERIAL_BAUD       = 115200;
const size_t   SERIAL_LINE_MAX   = 80;

// ---------------- Derived values ----------------
float gainK = 0.0f;                    // Tap-to-node attenuation
float nodeZeroMv[NODE_COUNT];          // Node level at 0 V tap voltage (mV)

struct Sample {
  float vA, vB;                        // Load-side tap voltages (V)
  float iA, iB;                        // Leg currents (A)
  float vLoad, iLoad, leak;            // Load voltage, load current, leg imbalance
};

struct Stats {
  uint32_t n;
  float sumV, sumI, sumVV, sumII, sumVI, sumLL;
  float sumVAVA, sumVAIA, sumVBVB, sumVBIB;
};

struct Result {
  float vRms, iRms, vMean, iMean, p, s, q, pf, leakRms;
};

// ---------------- Setup helpers ----------------
void initScaling() {
  float gSeries = 1.0f / R_SERIES_OHM;
  float gTotal  = gSeries + 1.0f / R_UP_OHM + 1.0f / R_DOWN_OHM;
  gainK = gSeries / gTotal;
  float biasMv = VREF_MV * (1.0f / R_UP_OHM) / gTotal;   // Calculated bias
  for (int i = 0; i < NODE_COUNT; i++) nodeZeroMv[i] = biasMv;
}

void initAdc() {
  analogReadResolution(12);
  for (int i = 0; i < NODE_COUNT; i++) {
    analogSetPinAttenuation(NODE_PINS[i], ADC_11db);
  }
}

// ---------------- Sampling ----------------
float readMv(Node node) {
  return (float)analogReadMilliVolts(NODE_PINS[node]);
}

// Reads SA, LA, SB, LB, SB, LA, SA and averages the repeated channels so that
// every channel is estimated at the instant of the single LB reading.
void readNodes(float mv[NODE_COUNT]) {
  float sa1 = readMv(NODE_SRC_A);
  float la1 = readMv(NODE_LOAD_A);
  float sb1 = readMv(NODE_SRC_B);
  float lb  = readMv(NODE_LOAD_B);
  float sb2 = readMv(NODE_SRC_B);
  float la2 = readMv(NODE_LOAD_A);
  float sa2 = readMv(NODE_SRC_A);
  mv[NODE_SRC_A]  = 0.5f * (sa1 + sa2);
  mv[NODE_LOAD_A] = 0.5f * (la1 + la2);
  mv[NODE_SRC_B]  = 0.5f * (sb1 + sb2);
  mv[NODE_LOAD_B] = lb;
}

Sample takeSample() {
  float mv[NODE_COUNT];
  readNodes(mv);
  float scale = gainK * 1000.0f;                       // Node mV per tap volt
  float vSA = (mv[NODE_SRC_A]  - nodeZeroMv[NODE_SRC_A])  / scale;
  float vLA = (mv[NODE_LOAD_A] - nodeZeroMv[NODE_LOAD_A]) / scale;
  float vSB = (mv[NODE_SRC_B]  - nodeZeroMv[NODE_SRC_B])  / scale;
  float vLB = (mv[NODE_LOAD_B] - nodeZeroMv[NODE_LOAD_B]) / scale;

  Sample s;
  s.vA    = vLA;
  s.vB    = vLB;
  s.iA    = (vSA - vLA) / R_SHUNT_A_OHM - CM_TRIM_A_PER_V * vLA;
  s.iB    = (vSB - vLB) / R_SHUNT_B_OHM - CM_TRIM_B_PER_V * vLB;
  s.vLoad = vLA - vLB;
  s.iLoad = 0.5f * (s.iA - s.iB);
  s.leak  = s.iA + s.iB;
  return s;
}

// Averages every node and stores it as that node's zero level.
// The source must be de-energized (all taps at 0 V).
void measureZero() {
  if (ZERO_SAMPLES == 0) return;
  float sum[NODE_COUNT] = {0, 0, 0, 0};
  for (uint32_t k = 0; k < ZERO_SAMPLES; k++) {
    float mv[NODE_COUNT];
    readNodes(mv);
    for (int i = 0; i < NODE_COUNT; i++) sum[i] += mv[i];
  }
  for (int i = 0; i < NODE_COUNT; i++) nodeZeroMv[i] = sum[i] / ZERO_SAMPLES;
}

void printZero() {
  Serial.printf("zero(mV): SA=%.1f LA=%.1f SB=%.1f LB=%.1f\n",
                nodeZeroMv[NODE_SRC_A], nodeZeroMv[NODE_LOAD_A],
                nodeZeroMv[NODE_SRC_B], nodeZeroMv[NODE_LOAD_B]);
}

// ---------------- Window statistics ----------------
Stats collectWindow() {
  Stats st = {};
  uint32_t start = millis();
  while (millis() - start < WINDOW_MS) {
    Sample s = takeSample();
    st.n++;
    st.sumV    += s.vLoad;
    st.sumI    += s.iLoad;
    st.sumVV   += s.vLoad * s.vLoad;
    st.sumII   += s.iLoad * s.iLoad;
    st.sumVI   += s.vLoad * s.iLoad;
    st.sumLL   += s.leak * s.leak;
    st.sumVAVA += s.vA * s.vA;
    st.sumVAIA += s.vA * s.iA;
    st.sumVBVB += s.vB * s.vB;
    st.sumVBIB += s.vB * s.iB;
  }
  return st;
}

Result computeResult(const Stats &st) {
  Result r = {};
  if (st.n == 0) return r;
  float n = (float)st.n;
  r.vMean   = st.sumV / n;
  r.iMean   = st.sumI / n;
  r.vRms    = sqrtf(st.sumVV / n);
  r.iRms    = sqrtf(st.sumII / n);
  r.leakRms = sqrtf(st.sumLL / n);
  r.p       = st.sumVI / n;
  r.s       = r.vRms * r.iRms;
  float q2  = r.s * r.s - r.p * r.p;
  r.q       = (q2 > 0.0f) ? sqrtf(q2) : 0.0f;
  r.pf      = (r.s > 1e-6f) ? (r.p / r.s) : 0.0f;
  return r;
}

// ---------------- Reporting ----------------
void printResult(const Result &r, uint32_t samples) {
  Serial.printf("Vrms=%.2f V  Irms=%.3f A  P=%.2f W  S=%.2f VA  Q=%.2f var  PF=%.3f"
                "  Ileak=%.3f A  Vdc=%.2f V  Idc=%.3f A  n=%lu\n",
                r.vRms, r.iRms, r.p, r.s, r.q, r.pf,
                r.leakRms, r.vMean, r.iMean, (unsigned long)samples);
}

void printLegHint(const char *name, float sumVI, float sumVV, float trim, uint32_t n) {
  float vRms = sqrtf(sumVV / (float)n);
  if (vRms < MIN_CAL_VRMS_V) {
    Serial.printf("Leg %s: tap voltage too low (%.2f V), no trim needed\n", name, vRms);
    return;
  }
  float residual = sumVI / sumVV;                       // Residual current per volt
  Serial.printf("Leg %s: residual=%.5f A/V  suggested trim_%c=%.5f\n",
                name, residual, (name[0] == 'A') ? 'a' : 'b', trim + residual);
}

void printCalibrationHint(const Stats &st) {
  if (st.n == 0) return;
  printLegHint("A", st.sumVAIA, st.sumVAVA, CM_TRIM_A_PER_V, st.n);
  printLegHint("B", st.sumVBIB, st.sumVBVB, CM_TRIM_B_PER_V, st.n);
}

// ---------------- Modes ----------------
void runMeasure() {
  Stats st = collectWindow();
  printResult(computeResult(st), st.n);
}

void runCalibrate() {
  Stats st = collectWindow();
  printCalibrationHint(st);
}

void runMode() {
  switch (RUN_MODE) {
    case MODE_MEASURE:   runMeasure();   break;
    case MODE_CALIBRATE: runCalibrate(); break;
  }
}

// ---------------- Serial parameter table ----------------
enum ParamType   { P_FLOAT, P_U32, P_PIN, P_BOOL, P_MODE };
enum ParamEffect { FX_NONE, FX_SCALING, FX_PINS };

struct Param {
  const char *name;
  ParamType   type;
  void       *ptr;
  float       minV, maxV;      // Accepted range (numeric types)
  ParamEffect fx;              // What must be recomputed after a change
  const char *help;
};

const Param PARAMS[] = {
  {"rs",         P_FLOAT, &R_SERIES_OHM,       1.0f,     1.0e7f,  FX_SCALING, "Rs, tap to ADC node (ohm)"},
  {"rup",        P_FLOAT, &R_UP_OHM,           1.0f,     1.0e7f,  FX_SCALING, "Rup, node to 3V3 (ohm)"},
  {"rdn",        P_FLOAT, &R_DOWN_OHM,         1.0f,     1.0e7f,  FX_SCALING, "Rdn, node to GND (ohm)"},
  {"vref",       P_FLOAT, &VREF_MV,            2500.0f,  3600.0f, FX_SCALING, "actual 3V3 rail (mV)"},
  {"rshunt_a",   P_FLOAT, &R_SHUNT_A_OHM,      0.0001f,  100.0f,  FX_NONE,    "shunt, leg A (ohm)"},
  {"rshunt_b",   P_FLOAT, &R_SHUNT_B_OHM,      0.0001f,  100.0f,  FX_NONE,    "shunt, leg B (ohm)"},
  {"trim_a",     P_FLOAT, &CM_TRIM_A_PER_V,    -10.0f,   10.0f,   FX_NONE,    "common-mode trim, leg A (A/V)"},
  {"trim_b",     P_FLOAT, &CM_TRIM_B_PER_V,    -10.0f,   10.0f,   FX_NONE,    "common-mode trim, leg B (A/V)"},
  {"min_cal_v",  P_FLOAT, &MIN_CAL_VRMS_V,     0.0f,     1000.0f, FX_NONE,    "min tap Vrms for a trim estimate (V)"},
  {"window_ms",  P_U32,   &WINDOW_MS,          10.0f,    10000.0f,FX_NONE,    "integration window (ms)"},
  {"zero_samples",P_U32,  &ZERO_SAMPLES,       1.0f,     100000.0f,FX_NONE,   "samples used by 'zero'"},
  {"pin_sa",     P_PIN,   &NODE_PINS[NODE_SRC_A],  32.0f, 39.0f,   FX_PINS,    "ADC1 GPIO, source tap A"},
  {"pin_la",     P_PIN,   &NODE_PINS[NODE_LOAD_A], 32.0f, 39.0f,   FX_PINS,    "ADC1 GPIO, load tap A"},
  {"pin_sb",     P_PIN,   &NODE_PINS[NODE_SRC_B],  32.0f, 39.0f,   FX_PINS,    "ADC1 GPIO, source tap B"},
  {"pin_lb",     P_PIN,   &NODE_PINS[NODE_LOAD_B], 32.0f, 39.0f,   FX_PINS,    "ADC1 GPIO, load tap B"},
  {"mode",       P_MODE,  &RUN_MODE,           0.0f,     0.0f,    FX_NONE,    "measure | calibrate"},
  {"stream",     P_BOOL,  &STREAM_OUTPUT,      0.0f,     0.0f,    FX_NONE,    "on/off: print every window"},
};
const size_t PARAM_COUNT = sizeof(PARAMS) / sizeof(PARAMS[0]);

const Param *findParam(const char *name) {
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    if (strcasecmp(name, PARAMS[i].name) == 0) return &PARAMS[i];
  }
  return nullptr;
}

void printParam(const Param &p) {
  Serial.printf("%-13s = ", p.name);
  switch (p.type) {
    case P_FLOAT: Serial.printf("%.6g", (double)*(float *)p.ptr);             break;
    case P_U32:   Serial.printf("%lu", (unsigned long)*(uint32_t *)p.ptr);    break;
    case P_PIN:   Serial.printf("%d", *(int *)p.ptr);                         break;
    case P_BOOL:  Serial.print(*(bool *)p.ptr ? "on" : "off");                break;
    case P_MODE:  Serial.print(*(Mode *)p.ptr == MODE_MEASURE ? "measure" : "calibrate"); break;
  }
  Serial.printf("   (%s)\n", p.help);
}

void printAllParams() {
  for (size_t i = 0; i < PARAM_COUNT; i++) printParam(PARAMS[i]);
  Serial.printf("k=%.5f  ", gainK);
  printZero();
}

// ---------------- Serial value parsing ----------------
bool parseFloat(const char *s, float &out) {
  char *end;
  double v = strtod(s, &end);
  if (end == s || *end != '\0' || !isfinite(v)) return false;
  out = (float)v;
  return true;
}

bool parseLong(const char *s, long &out) {
  char *end;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;
  out = v;
  return true;
}

bool parseBool(const char *s, bool &out) {
  if (!strcasecmp(s, "1") || !strcasecmp(s, "on")  || !strcasecmp(s, "true"))  { out = true;  return true; }
  if (!strcasecmp(s, "0") || !strcasecmp(s, "off") || !strcasecmp(s, "false")) { out = false; return true; }
  return false;
}

void rangeError(const Param &p) {
  Serial.printf("ERR: %s must be between %.6g and %.6g\n", p.name, (double)p.minV, (double)p.maxV);
}

bool setParam(const Param &p, const char *txt) {
  switch (p.type) {
    case P_FLOAT: {
      float v;
      if (!parseFloat(txt, v))          { Serial.println("ERR: not a number"); return false; }
      if (v < p.minV || v > p.maxV)     { rangeError(p); return false; }
      *(float *)p.ptr = v;
      break;
    }
    case P_U32: {
      long v;
      if (!parseLong(txt, v))           { Serial.println("ERR: not an integer"); return false; }
      if (v < (long)p.minV || v > (long)p.maxV) { rangeError(p); return false; }
      *(uint32_t *)p.ptr = (uint32_t)v;
      break;
    }
    case P_PIN: {
      long v;
      if (!parseLong(txt, v))           { Serial.println("ERR: not an integer"); return false; }
      if (v < (long)p.minV || v > (long)p.maxV) { rangeError(p); return false; }
      for (int i = 0; i < NODE_COUNT; i++) {
        if (&NODE_PINS[i] != (int *)p.ptr && NODE_PINS[i] == (int)v) {
          Serial.printf("ERR: GPIO%ld is already used by another tap\n", v);
          return false;
        }
      }
      *(int *)p.ptr = (int)v;
      break;
    }
    case P_BOOL: {
      bool v;
      if (!parseBool(txt, v))           { Serial.println("ERR: use on/off, 1/0 or true/false"); return false; }
      *(bool *)p.ptr = v;
      break;
    }
    case P_MODE: {
      if (!strcasecmp(txt, "measure") || !strcmp(txt, "0"))        *(Mode *)p.ptr = MODE_MEASURE;
      else if (!strcasecmp(txt, "calibrate") || !strcmp(txt, "1")) *(Mode *)p.ptr = MODE_CALIBRATE;
      else { Serial.println("ERR: use measure or calibrate"); return false; }
      break;
    }
  }

  printParam(p);
  if (p.fx == FX_SCALING) {
    initScaling();
    Serial.println("NOTE: zero offsets reset to the calculated bias. Run 'zero' with the source de-energized to re-measure.");
  } else if (p.fx == FX_PINS) {
    initAdc();
  }
  return true;
}

// ---------------- Serial commands ----------------
void printHelp() {
  Serial.println("Commands (end with newline; names are case-insensitive):");
  Serial.println("  help                  this list");
  Serial.println("  get [name]            show all variables, or one");
  Serial.println("  set <name> <value>    override a variable (also: <name> <value>, <name>=<value>)");
  Serial.println("  zero                  measure zero offsets now (source must be de-energized)");
  Serial.println("  zero calc             use the calculated bias instead of a measured zero");
  Serial.println("  read                  run and print one window now");
  Serial.println("  reboot                restart the ESP32 (overrides are lost)");
  Serial.println("Variables:");
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    Serial.printf("  %-13s %s\n", PARAMS[i].name, PARAMS[i].help);
  }
}

void processLine(char *line) {
  for (char *c = line; *c; c++) if (*c == '=') *c = ' ';

  char *tok[3];
  int n = 0;
  char *save = nullptr;
  for (char *t = strtok_r(line, " \t", &save); t && n < 3; t = strtok_r(nullptr, " \t", &save)) {
    tok[n++] = t;
  }
  if (n == 0) return;

  const char *cmd = tok[0];

  if (!strcasecmp(cmd, "help") || !strcmp(cmd, "?")) { printHelp(); return; }

  if (!strcasecmp(cmd, "get") || !strcasecmp(cmd, "show")) {
    if (n < 2) { printAllParams(); return; }
    const Param *p = findParam(tok[1]);
    if (p) printParam(*p); else Serial.println("ERR: unknown variable");
    return;
  }

  if (!strcasecmp(cmd, "zero")) {
    if (n >= 2 && !strcasecmp(tok[1], "calc")) {
      initScaling();
    } else {
      measureZero();
    }
    printZero();
    return;
  }

  if (!strcasecmp(cmd, "read"))   { runMode(); return; }
  if (!strcasecmp(cmd, "reboot")) { Serial.println("Restarting..."); Serial.flush(); ESP.restart(); return; }

  // "set <name> <value>", "<name> <value>" or bare "<name>"
  int first = !strcasecmp(cmd, "set") ? 1 : 0;
  if (first >= n) { Serial.println("ERR: usage: set <name> <value>"); return; }
  const Param *p = findParam(tok[first]);
  if (!p) { Serial.println("ERR: unknown command or variable (type 'help')"); return; }
  if (first + 1 >= n) { printParam(*p); return; }   // No value: just show it
  setParam(*p, tok[first + 1]);
}

// Collects characters until a newline; processes complete lines. Called between windows.
void handleSerial() {
  static char   line[SERIAL_LINE_MAX];
  static size_t len = 0;
  static bool   overflow = false;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (overflow) {
        Serial.println("ERR: line too long");
      } else if (len > 0) {
        line[len] = '\0';
        processLine(line);
      }
      len = 0;
      overflow = false;
    } else if (len < SERIAL_LINE_MAX - 1) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
}

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  initScaling();
  initAdc();
  if (AUTO_ZERO_AT_BOOT) measureZero();
  Serial.printf("Power monitor ready. k=%.4f  ", gainK);
  printZero();
  Serial.println("Type 'help' for serial commands.");
}

void loop() {
  if (STREAM_OUTPUT) runMode(); else delay(5);
  handleSerial();
}
