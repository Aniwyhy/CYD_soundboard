// CYD 4.0" Bluetooth Soundboard
// Board: ESP32-4832S040C (ST7796 480x320 + GT911 touch), ESP32 classic
// Libs: LovyanGFX, ESP32-A2DP (pschatzmann)

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <vector>
#include <algorithm>
#include "freertos/ringbuf.h"
#include <LovyanGFX.hpp>
#include "BluetoothA2DPSource.h"

// ---------------- Pin config (adjust if your board revision differs) ----------------
#define TFT_SCLK 14
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_CS   15
#define TFT_DC    2
#define TFT_BL   27
#define TOUCH_SDA 33
#define TOUCH_SCL 32
#define TOUCH_INT 21
#define TOUCH_RST 25
#define SD_CS     5
#define SD_SCK   18
#define SD_MISO  19
#define SD_MOSI  23

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
  lgfx::Touch_GT911  _touch;
public:
  LGFX() {
    { auto c = _bus.config();
      c.spi_host = SPI2_HOST; c.spi_mode = 0;
      c.freq_write = 40000000; c.freq_read = 16000000;
      c.pin_sclk = TFT_SCLK; c.pin_mosi = TFT_MOSI; c.pin_miso = TFT_MISO; c.pin_dc = TFT_DC;
      _bus.config(c); _panel.setBus(&_bus); }
    { auto c = _panel.config();
      c.pin_cs = TFT_CS; c.pin_rst = -1;
      c.panel_width = 320; c.panel_height = 480;
      c.invert = false;      // set true if colors look inverted
      c.rgb_order = false;   // set true if red/blue are swapped
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = TFT_BL; c.invert = false; c.freq = 44100; c.pwm_channel = 7;
      _light.config(c); _panel.setLight(&_light); }
    { auto c = _touch.config();
      c.x_min = 0; c.x_max = 319; c.y_min = 0; c.y_max = 479;
      c.pin_int = TOUCH_INT; c.pin_rst = TOUCH_RST; c.bus_shared = false;
      c.offset_rotation = 0;  // if touch is rotated/mirrored vs display, try 0..7
      c.i2c_port = 1; c.i2c_addr = 0x5D;  // some boards use 0x14
      c.pin_sda = TOUCH_SDA; c.pin_scl = TOUCH_SCL; c.freq = 400000;
      _touch.config(c); _panel.setTouch(&_touch); }
    setPanel(&_panel);
  }
};

LGFX tft;
BluetoothA2DPSource a2dp;
Preferences prefs;

// ---------------- Shared state ----------------
struct Sound { String path; String label; };
std::vector<Sound> sounds;

SemaphoreHandle_t sdMutex;
RingbufHandle_t   rb;

volatile int     playReq    = -1;
volatile bool    stopReq    = false;
volatile int     playingIdx = -1;
volatile uint8_t volume     = 70;   // 0..100
uint8_t          bright     = 200;  // 0..255
bool             sdOk       = false;

// Bluetooth
#define MAXDEV 12
struct BtDev { char name[33]; };
BtDev devs[MAXDEV];
volatile int devCount = 0;
char btTarget[33] = "";

// ---------------- WAV reader + resampler (runs in audio task) ----------------
struct WavReader {
  File f;
  uint16_t ch = 2, bits = 16, bps = 2;
  uint32_t rate = 44100;
  uint32_t remaining = 0;
  uint8_t  buf[1536];
  size_t   bufLen = 0, bufPos = 0;

  void close() {
    if (f) { xSemaphoreTake(sdMutex, portMAX_DELAY); f.close(); xSemaphoreGive(sdMutex); }
  }

  bool open(const String& path) {
    bool ok = false;
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    f = SD.open(path, FILE_READ);
    if (f) {
      uint8_t hdr[12];
      if (f.read(hdr, 12) == 12 && !memcmp(hdr, "RIFF", 4) && !memcmp(hdr + 8, "WAVE", 4)) {
        bool gotFmt = false;
        while (f.available() >= 8) {
          uint8_t ck[8];
          f.read(ck, 8);
          uint32_t sz = ck[4] | (ck[5] << 8) | (ck[6] << 16) | ((uint32_t)ck[7] << 24);
          if (!memcmp(ck, "fmt ", 4) && sz >= 16) {
            uint8_t fm[16];
            f.read(fm, 16);
            uint16_t tag = fm[0] | (fm[1] << 8);
            ch   = fm[2] | (fm[3] << 8);
            rate = fm[4] | (fm[5] << 8) | (fm[6] << 16) | ((uint32_t)fm[7] << 24);
            bits = fm[14] | (fm[15] << 8);
            bps  = bits / 8;
            gotFmt = (tag == 1 || tag == 0xFFFE) && ch >= 1 && (bits == 8 || bits == 16 || bits == 24) && rate > 0;
            f.seek(f.position() + (sz - 16) + (sz & 1));
          } else if (!memcmp(ck, "data", 4)) {
            uint32_t avail = f.size() - f.position();
            remaining = min(sz, avail);
            ok = gotFmt;
            break;
          } else {
            f.seek(f.position() + sz + (sz & 1));
          }
        }
      }
      if (!ok) f.close();
    }
    xSemaphoreGive(sdMutex);
    bufLen = bufPos = 0;
    return ok;
  }

  bool readFrame(int16_t& l, int16_t& r) {
    size_t need = ch * bps;
    if (bufLen - bufPos < need) {
      size_t left = bufLen - bufPos;
      memmove(buf, buf + bufPos, left);
      size_t toRead = min((size_t)(sizeof(buf) - left), (size_t)remaining);
      size_t n = 0;
      if (toRead) {
        xSemaphoreTake(sdMutex, portMAX_DELAY);
        n = f.read(buf + left, toRead);
        xSemaphoreGive(sdMutex);
      }
      remaining -= n;
      bufLen = left + n; bufPos = 0;
      if (bufLen < need) return false;
    }
    const uint8_t* p = buf + bufPos;
    l = sampleAt(p);
    r = (ch >= 2) ? sampleAt(p + bps) : l;
    bufPos += need;
    return true;
  }

  inline int16_t sampleAt(const uint8_t* p) {
    if (bits == 8)  return (int16_t)(((int)p[0] - 128) << 8);
    if (bits == 16) return (int16_t)(p[0] | (p[1] << 8));
    return (int16_t)(p[1] | (p[2] << 8));  // 24-bit: top 16 bits
  }
};

WavReader wr;

void drainRing() {
  size_t n; void* p;
  while ((p = xRingbufferReceiveUpTo(rb, &n, 0, 4096)) != NULL) vRingbufferReturnItem(rb, p);
}

void audioTask(void*) {
  bool active = false, done = false;
  float step = 1, frac = 0;
  int16_t cl = 0, cr = 0, nl = 0, nr = 0;
  static int16_t out[256 * 2];

  for (;;) {
    if (stopReq || playReq >= 0) {
      int idx = playReq; playReq = -1;
      stopReq = false;
      active = false; wr.close(); drainRing(); playingIdx = -1;
      if (idx >= 0 && idx < (int)sounds.size() && wr.open(sounds[idx].path)) {
        step = (float)wr.rate / 44100.0f; frac = 0; done = false;
        if (wr.readFrame(cl, cr)) {
          if (!wr.readFrame(nl, nr)) { nl = cl; nr = cr; done = true; }
          active = true; playingIdx = idx;
        } else wr.close();
      }
    }

    if (!active) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    float g = (volume / 100.0f); g *= g;
    int n = 0;
    while (n < 256 && !done) {
      float l = cl + (nl - cl) * frac;
      float r = cr + (nr - cr) * frac;
      out[n * 2]     = (int16_t)(l * g);
      out[n * 2 + 1] = (int16_t)(r * g);
      n++;
      frac += step;
      while (frac >= 1.0f) {
        frac -= 1.0f; cl = nl; cr = nr;
        if (!wr.readFrame(nl, nr)) { done = true; break; }
      }
    }

    bool sent = (n == 0);
    while (!sent) {
      if (stopReq || playReq >= 0) break;
      sent = xRingbufferSend(rb, out, n * 4, pdMS_TO_TICKS(20)) == pdTRUE;
    }
    if (!sent) continue;  // interrupted by new request

    if (done) { active = false; wr.close(); playingIdx = -1; }
  }
}

// ---------------- Bluetooth callbacks ----------------
int32_t getFrames(Frame* frame, int32_t count) {
  uint8_t* dst = (uint8_t*)frame;
  size_t need = count * 4, got = 0;
  while (got < need) {
    size_t n; void* p = xRingbufferReceiveUpTo(rb, &n, 0, need - got);
    if (!p) break;
    memcpy(dst + got, p, n);
    vRingbufferReturnItem(rb, p);
    got += n;
  }
  if (got < need) memset(dst + got, 0, need - got);
  return count;
}

bool onSsid(const char* ssid, esp_bd_addr_t address, int rssi) {
  if (!ssid || !*ssid) return false;
  bool dup = false;
  int c = devCount;
  for (int i = 0; i < c; i++) if (!strcmp(devs[i].name, ssid)) { dup = true; break; }
  if (!dup && c < MAXDEV) { strlcpy(devs[c].name, ssid, 33); devCount = c + 1; }
  return btTarget[0] && !strcmp(btTarget, ssid);  // true = connect to this one
}

// ---------------- SD scan ----------------
void scanSD() {
  stopReq = true; delay(120);
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  sounds.clear();
  File root = SD.open("/");
  if (root) {
    File e;
    while ((e = root.openNextFile())) {
      String name = e.name();
      bool isDir = e.isDirectory();
      e.close();
      if (isDir || name.startsWith(".")) continue;
      String low = name; low.toLowerCase();
      if (!low.endsWith(".wav")) continue;
      Sound s;
      s.path = "/" + name;
      s.label = name.substring(0, name.length() - 4);
      s.label.replace('_', ' ');
      sounds.push_back(s);
      if (sounds.size() >= 96) break;
    }
    root.close();
  }
  xSemaphoreGive(sdMutex);
  std::sort(sounds.begin(), sounds.end(), [](const Sound& a, const Sound& b) {
    String x = a.label, y = b.label; x.toLowerCase(); y.toLowerCase(); return x < y;
  });
}

// ---------------- UI ----------------
enum Screen { SCR_MAIN, SCR_SETTINGS, SCR_BT };
Screen screen = SCR_MAIN;

#define W 480
#define H 320
#define TOP 36
#define BOT 36
#define COLS 4
#define ROWS 3
#define PER (COLS * ROWS)
#define GAP 4
#define CELLW ((W - (COLS + 1) * GAP) / COLS)
#define CELLH ((H - TOP - BOT - (ROWS + 1) * GAP) / ROWS)

int page = 0;
int shownPlaying = -2;
bool shownConn = false;

uint16_t padColor(int i) {
  static const uint8_t pal[][3] = {
    {220, 60, 60}, {230, 140, 30}, {200, 180, 30}, {60, 170, 80},
    {40, 160, 170}, {60, 110, 220}, {130, 80, 210}, {200, 70, 150}};
  const uint8_t* c = pal[i % 8];
  return tft.color565(c[0], c[1], c[2]);
}

bool inRect(int x, int y, int rx, int ry, int rw, int rh) {
  return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

void drawBtn(int x, int y, int w, int h, const char* label, uint16_t col = 0x3186) {
  tft.fillRoundRect(x, y, w, h, 6, col);
  tft.drawRoundRect(x, y, w, h, 6, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, col);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.setFont(&fonts::DejaVu12);
  tft.drawString(label, x + w / 2, y + h / 2);
}

std::vector<String> wrapText(const String& s, int maxw) {
  std::vector<String> out; String cur;
  for (size_t i = 0; i < s.length(); i++) {
    cur += s[i];
    if (tft.textWidth(cur) > maxw) {
      int sp = cur.lastIndexOf(' ');
      if (sp > 0) { out.push_back(cur.substring(0, sp)); cur = cur.substring(sp + 1); }
      else { String last = cur.substring(cur.length() - 1); cur.remove(cur.length() - 1); out.push_back(cur); cur = last; }
    }
  }
  if (cur.length()) out.push_back(cur);
  return out;
}

void padRect(int slot, int& x, int& y) {
  x = GAP + (slot % COLS) * (CELLW + GAP);
  y = TOP + GAP + (slot / COLS) * (CELLH + GAP);
}

void drawPad(int slot) {
  int x, y; padRect(slot, x, y);
  int idx = page * PER + slot;
  if (idx >= (int)sounds.size()) { tft.fillRect(x, y, CELLW, CELLH, TFT_BLACK); return; }
  bool on = (idx == playingIdx);
  uint16_t col = padColor(idx);
  tft.fillRoundRect(x, y, CELLW, CELLH, 8, col);
  if (on) { tft.drawRoundRect(x, y, CELLW, CELLH, 8, TFT_WHITE); tft.drawRoundRect(x + 1, y + 1, CELLW - 2, CELLH - 2, 7, TFT_WHITE); tft.drawRoundRect(x + 2, y + 2, CELLW - 4, CELLH - 4, 6, TFT_WHITE); }
  tft.setFont(&fonts::DejaVu12);
  tft.setTextColor(TFT_WHITE, col);
  tft.setTextDatum(textdatum_t::middle_center);
  auto lines = wrapText(sounds[idx].label, CELLW - 12);
  int n = min((int)lines.size(), 3);
  int lh = 15, y0 = y + CELLH / 2 - (n * lh) / 2 + lh / 2;
  for (int i = 0; i < n; i++) {
    String t = lines[i];
    if (i == 2 && (int)lines.size() > 3) t += "..";
    tft.drawString(t, x + CELLW / 2, y0 + i * lh);
  }
}

int pageCount() { return max(1, ((int)sounds.size() + PER - 1) / PER); }

bool btConnected() { return a2dp.is_connected(); }

void drawTop() {
  tft.fillRect(0, 0, W, TOP, 0x2104);
  tft.setFont(&fonts::DejaVu18);
  tft.setTextColor(TFT_WHITE, 0x2104);
  tft.setTextDatum(textdatum_t::middle_left);
  tft.drawString("SOUNDBOARD", 8, TOP / 2);
  bool c = btConnected();
  uint16_t dot = c ? TFT_GREEN : (btTarget[0] ? TFT_ORANGE : TFT_DARKGREY);
  tft.fillCircle(190, TOP / 2, 6, dot);
  tft.setFont(&fonts::DejaVu12);
  tft.drawString("BT", 202, TOP / 2);
  if (playingIdx >= 0) drawBtn(280, 4, 90, 28, "STOP", tft.color565(180, 40, 40));
  drawBtn(382, 4, 94, 28, "Settings");
  shownConn = c;
}

void drawBottom() {
  tft.fillRect(0, H - BOT, W, BOT, 0x2104);
  drawBtn(4, H - BOT + 4, 90, 28, "<");
  drawBtn(W - 94, H - BOT + 4, 90, 28, ">");
  char b[32]; snprintf(b, sizeof(b), "Page %d/%d  (%d sounds)", page + 1, pageCount(), (int)sounds.size());
  tft.setFont(&fonts::DejaVu12);
  tft.setTextColor(TFT_WHITE, 0x2104);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.drawString(b, W / 2, H - BOT / 2);
}

void drawMain() {
  screen = SCR_MAIN;
  tft.fillScreen(TFT_BLACK);
  drawTop(); drawBottom();
  if (sounds.empty()) {
    tft.setFont(&fonts::DejaVu18);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(textdatum_t::middle_center);
    tft.drawString(sdOk ? "No .wav files in SD root" : "SD card not found", W / 2, H / 2);
  } else {
    for (int s = 0; s < PER; s++) drawPad(s);
  }
  shownPlaying = playingIdx;
}

void drawSlider(int y, const char* label, int value, int maxv) {
  tft.fillRect(0, y, W, 48, TFT_BLACK);
  tft.setFont(&fonts::DejaVu18);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(textdatum_t::middle_left);
  tft.drawString(label, 20, y + 22);
  drawBtn(190, y, 56, 44, "-");
  tft.drawRect(252, y, 124, 44, TFT_DARKGREY);
  tft.fillRect(253, y + 1, (122 * value) / maxv, 42, tft.color565(60, 110, 220));
  drawBtn(380, y, 56, 44, "+");
}

void drawSettings() {
  screen = SCR_SETTINGS;
  tft.fillScreen(TFT_BLACK);
  drawBtn(4, 4, 70, 32, "< Back");
  tft.setFont(&fonts::DejaVu18);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.drawString("Settings", W / 2, 20);
  drawSlider(60, "Volume", volume, 100);
  drawSlider(116, "Brightness", bright - 20, 235);
  String bt = String("Bluetooth:  ") + (btConnected() ? String("connected to ") + btTarget : (btTarget[0] ? String("searching...") : String("not set")));
  drawBtn(20, 172, 440, 44, bt.c_str());
  drawBtn(20, 228, 440, 44, "Rescan SD card");
}

uint32_t btSig() {
  uint32_t s = devCount * 7 + (btConnected() ? 1000 : 0);
  for (const char* p = btTarget; *p; p++) s = s * 31 + *p;
  return s;
}
uint32_t shownBtSig = 0, lastBtCheck = 0;

void drawBT() {
  screen = SCR_BT;
  tft.fillScreen(TFT_BLACK);
  drawBtn(4, 4, 70, 32, "< Back");
  tft.setFont(&fonts::DejaVu18);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.drawString("Bluetooth speaker", W / 2, 20);

  tft.setFont(&fonts::DejaVu12);
  tft.setTextDatum(textdatum_t::middle_left);
  String st;
  uint16_t sc = TFT_WHITE;
  if (btConnected())      { st = String("Connected: ") + btTarget; sc = TFT_GREEN; }
  else if (btTarget[0])   { st = String("Searching for: ") + btTarget + " (turn speaker on / pairing mode)"; sc = TFT_ORANGE; }
  else                    { st = "Put your speaker in pairing mode, then tap it below"; }
  tft.setTextColor(sc, TFT_BLACK);
  tft.drawString(st, 12, 56);

  int c = devCount;
  for (int i = 0; i < 5; i++) {
    int y = 80 + i * 36;
    if (i < c) {
      bool sel = !strcmp(devs[i].name, btTarget);
      drawBtn(20, y, 440, 32, devs[i].name, sel ? tft.color565(40, 120, 60) : 0x3186);
    } else tft.fillRect(20, y, 440, 32, TFT_BLACK);
  }
  if (c == 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextDatum(textdatum_t::middle_center);
    tft.drawString("Scanning... no devices found yet", W / 2, 150);
  }
  drawBtn(20, 270, 200, 40, "Forget device", tft.color565(150, 40, 40));
  shownBtSig = btSig();
}

// ---------------- Touch handling ----------------
bool wasTouch = false;
uint32_t lastRepeat = 0;

void playIndex(int idx) { playReq = idx; }

void handleMain(int x, int y) {
  if (inRect(x, y, 382, 4, 94, 28)) { drawSettings(); return; }
  if (playingIdx >= 0 && inRect(x, y, 280, 4, 90, 28)) { stopReq = true; return; }
  if (inRect(x, y, 4, H - BOT + 4, 90, 28)) { page = (page + pageCount() - 1) % pageCount(); drawMain(); return; }
  if (inRect(x, y, W - 94, H - BOT + 4, 90, 28)) { page = (page + 1) % pageCount(); drawMain(); return; }
  for (int s = 0; s < PER; s++) {
    int px, py; padRect(s, px, py);
    int idx = page * PER + s;
    if (idx < (int)sounds.size() && inRect(x, y, px, py, CELLW, CELLH)) { playIndex(idx); return; }
  }
}

void handleSettings(bool down, int x, int y) {
  bool rep = down || (millis() - lastRepeat > 140);
  if (down && inRect(x, y, 4, 4, 70, 32)) {
    prefs.putUChar("vol", volume); prefs.putUChar("bri", bright);
    drawMain(); return;
  }
  if (rep && inRect(x, y, 190, 60, 56, 44)) { volume = max(0, (int)volume - 5); lastRepeat = millis(); drawSlider(60, "Volume", volume, 100); }
  if (rep && inRect(x, y, 380, 60, 56, 44)) { volume = min(100, (int)volume + 5); lastRepeat = millis(); drawSlider(60, "Volume", volume, 100); }
  if (rep && inRect(x, y, 190, 116, 56, 44)) { bright = max(20, (int)bright - 15); lastRepeat = millis(); tft.setBrightness(bright); drawSlider(116, "Brightness", bright - 20, 235); }
  if (rep && inRect(x, y, 380, 116, 56, 44)) { bright = min(255, (int)bright + 15); lastRepeat = millis(); tft.setBrightness(bright); drawSlider(116, "Brightness", bright - 20, 235); }
  if (down && inRect(x, y, 20, 172, 440, 44)) { drawBT(); return; }
  if (down && inRect(x, y, 20, 228, 440, 44)) {
    drawBtn(20, 228, 440, 44, "Scanning...");
    scanSD(); page = 0;
    drawBtn(20, 228, 440, 44, "Done!");
  }
}

void handleBT(bool down, int x, int y) {
  if (!down) return;
  if (inRect(x, y, 4, 4, 70, 32)) { drawSettings(); return; }
  int c = devCount;
  for (int i = 0; i < 5 && i < c; i++) {
    if (inRect(x, y, 20, 80 + i * 36, 440, 32)) {
      bool wasConn = btConnected();
      prefs.putString("bt", devs[i].name);
      strlcpy(btTarget, devs[i].name, 33);
      if (wasConn) { delay(100); ESP.restart(); }  // cleanest way to drop the old link
      drawBT();
      return;
    }
  }
  if (inRect(x, y, 20, 270, 200, 40)) {
    bool wasConn = btConnected();
    prefs.remove("bt");
    btTarget[0] = 0;
    if (wasConn) { delay(100); ESP.restart(); }
    drawBT();
  }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  prefs.begin("sb", false);
  volume = prefs.getUChar("vol", 70);
  bright = prefs.getUChar("bri", 200);
  strlcpy(btTarget, prefs.getString("bt", "").c_str(), 33);

  tft.init();
  tft.setRotation(1);
  tft.setBrightness(bright);
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::DejaVu18);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(textdatum_t::middle_center);
  tft.drawString("Starting...", W / 2, H / 2);

  sdMutex = xSemaphoreCreateMutex();
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk = SD.begin(SD_CS, SPI, 20000000);
  if (sdOk) scanSD();

  rb = xRingbufferCreate(32768, RINGBUF_TYPE_BYTEBUF);
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 3, NULL, 1);

  a2dp.set_auto_reconnect(false);
  a2dp.set_ssid_callback(onSsid);
  a2dp.set_data_callback_in_frames(getFrames);
  a2dp.start();   // discovery runs continuously; connects when onSsid() returns true

  drawMain();
}

void loop() {
  int32_t tx, ty;
  bool touching = tft.getTouch(&tx, &ty);
  bool down = touching && !wasTouch;
  wasTouch = touching;

  switch (screen) {
    case SCR_MAIN:
      if (down) handleMain(tx, ty);
      if (playingIdx != shownPlaying || btConnected() != shownConn) {
        int old = shownPlaying; shownPlaying = playingIdx;
        for (int i : {old, (int)playingIdx}) {
          if (i >= page * PER && i < (page + 1) * PER) drawPad(i - page * PER);
        }
        drawTop();
      }
      break;
    case SCR_SETTINGS:
      if (touching) handleSettings(down, tx, ty);
      break;
    case SCR_BT:
      handleBT(down, tx, ty);
      if (millis() - lastBtCheck > 700) {
        lastBtCheck = millis();
        if (btSig() != shownBtSig) drawBT();
      }
      break;
  }
  delay(10);
}
