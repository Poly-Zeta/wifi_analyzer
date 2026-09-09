#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <vector>
#include <math.h>

// ILI9341接続ピン (ESP32-C5)
#define TFT_RST 25
#define TFT_DC  26
#define TFT_CS  10
#define TFT_MOSI 7
#define TFT_CLK  6
#define TFT_MISO 2

#define DRAW_SWITCH_PIN 9
enum Mode { MONITOR, SERIALOUT };

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

// 画面レイアウト
static const int SCREEN_W = 320;
static const int SCREEN_H = 240;
static const int BAND_MARGIN_L = 14;
static const int BAND_MARGIN_R = 6;
static const int BAND_MARGIN_T = 6;
static const int BAND_MARGIN_B = 6;
static const int MID_GAP = 4;

struct BandRect { int x0,y0,x1,y1; } band2g, band5g;

// AP情報（表示用）
struct APInfo {
  uint8_t bssid[6];
  String ssid;
  bool is5g;
  int primaryCh;
  int centerCh;
  int centerFreqMHz;
  int bandwidthMHz;
  float rssiEMA;
  uint32_t lastSeenMs;
  uint16_t color;
};

std::vector<APInfo> scanBuffer;
std::vector<APInfo> displaySet;

// チャネルリスト
int channels2g[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};
int channels5g[] = {36,40,44,48,52,56,60,64,100,104,108,112,116,120,124,128,149,153,157,161};

// スイープ制御
enum SweepBand { BAND_2G, BAND_5G };
SweepBand currentBand = BAND_2G;
int hopIndex = 0;
uint32_t dwellStart = 0;
const uint32_t DWELL_MS = 200;

// FTM設定
const uint8_t FTM_FRAME_COUNT = 8;
const uint16_t FTM_BURST_PERIOD = 2;

SemaphoreHandle_t ftmSemaphore;
bool ftmSuccess = true;

struct accessPoint_t {
  uint8_t  mac[6];
  uint16_t channel; // FTMはprimaryチャネルを使用
};
std::vector<accessPoint_t> ftmCandidates;

// 帯域幅判定用情報
struct BwInfo {
  int primary = 0;      // 1..14 or 36..
  int bandwidth = 20;   // 20/40/80/160
  int center = 0;       // center channel index (5GHz) or primary (2.4GHz)
  int ht_sec_off = 0;   // 0: none/unknown, 1: above, 3: below
  bool is5g = false;
  bool valid = false;
};

// ISR→メイン転送（固定長）
struct BeaconMini {
  uint8_t  bssid[6];
  uint16_t primary;   // プライマリチャネル
  uint16_t center;    // センタチャネル（2.4Gはprimary、5GはVHT/HT規則）
  int8_t   rssi;
  uint8_t  ftmCap;    // Extended Cap bit70
  uint8_t  ssidLen;   // 0..32
  char     ssid[32];  // SSID原文（最大32）
  uint16_t len;       // パケット長（デバッグ用）
  uint8_t  width;     // 帯域幅 MHz
};

QueueHandle_t beaconQueue;
static const int BEACON_QUEUE_SIZE = 64;
static const size_t SCAN_MAX_APS = 128;
static const size_t FTM_MAX_CAND = 64;

static std::vector<uint64_t> ftmSeenMacs;

uint32_t nowMs() { return millis(); }

// 周波数計算
int centerFreqMHz_2g(int ch) { return (ch == 14) ? 2484 : (2412 + 5 * (ch - 1)); }
int centerFreqMHz_5g(int ch) { return 5000 + 5 * ch; }

// 線形マップ
int mapLinear(int v, int vmin, int vmax, int omin, int omax) {
  if (v <= vmin) return omin;
  if (v >= vmax) return omax;
  return omin + (int)((float)(v - vmin) * (float)(omax - omin) / (float)(vmax - vmin));
}

// RSSI→Y座標
int mapRSSIToY(float rssi, const BandRect& br) {
  const int top = br.y0 + 12;
  const int bottom = br.y1 - 6;
  if (rssi > 0) rssi = 0;
  if (rssi < -90) rssi = -90;
  float t = (0 - rssi) / 90.0f;
  return top + (int)(t * (bottom - top));
}

// 周波数→X座標
int mapFreqToX(int fMHz, bool is5g) {
  int left = BAND_MARGIN_L;
  int right = SCREEN_W - BAND_MARGIN_R;
  return is5g ? mapLinear(fMHz, 5180, 5825, left, right)
              : mapLinear(fMHz, 2412, 2484, left, right);
}

// ランダム色（RGB565）
uint16_t randomColor() {
  uint8_t r = random(8, 31);
  uint8_t g = random(16, 63);
  uint8_t b = random(8, 31);
  return (r << 11) | (g << 5) | b;
}

// グリッド・目盛り描画
void drawGridForBand(const BandRect& br, bool is5g) {
  tft.drawRect(br.x0, br.y0, SCREEN_W, br.y1 - br.y0 + 1, ILI9341_DARKGREY);
  int marks[] = {0, -20, -40, -60, -80};
  for (int i = 0; i < 5; ++i) {
    int y = mapRSSIToY(marks[i], br);
    tft.drawFastHLine(BAND_MARGIN_L, y, SCREEN_W - BAND_MARGIN_L - BAND_MARGIN_R, ILI9341_DARKGREY);
    tft.setCursor(2, y - 6);
    tft.setTextColor(ILI9341_DARKGREY);
    tft.setTextSize(1);
    tft.print(marks[i]);
  }
  if (!is5g) {
    int chans[] = {1, 6, 11, 13, 14};
    for (int i = 0; i < 5; ++i) {
      int f = centerFreqMHz_2g(chans[i]);
      int x = mapFreqToX(f, false);
      tft.drawFastVLine(x, br.y0, br.y1 - br.y0 + 1, ILI9341_DARKGREY);
      tft.setCursor(x - 6, br.y0 + 2);
      tft.print(chans[i]);
    }
  } else {
    int chans5[] = {36, 52, 100, 149, 161};
    for (int i = 0; i < 5; ++i) {
      int f = centerFreqMHz_5g(chans5[i]);
      int x = mapFreqToX(f, true);
      tft.drawFastVLine(x, br.y0, br.y1 - br.y0 + 1, ILI9341_DARKGREY);
      tft.setCursor(x - 10, br.y0 + 2);
      tft.print(chans5[i]);
    }
  }
}

// 画面初期化
void setupLayout(int mode) {
  tft.fillScreen(ILI9341_BLACK);
  if(mode==MONITOR){
    int halfH = (SCREEN_H - BAND_MARGIN_T - BAND_MARGIN_B - MID_GAP) / 2;
    band2g = {0, BAND_MARGIN_T, SCREEN_W - 1, BAND_MARGIN_T + halfH - 1};
    band5g = {0, band2g.y1 + 1 + MID_GAP, SCREEN_W - 1, SCREEN_H - 1 - BAND_MARGIN_B};
    drawGridForBand(band2g, false);
    drawGridForBand(band5g, true);
  }else if(mode==SERIALOUT){
    tft.setCursor(20, 20);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.println("STREAM MODE");
    tft.setTextSize(1);
    tft.setCursor(20, 60);
    tft.println("Output: Serial Console");
    tft.setCursor(20, 80);
    tft.println("Format: isFTM(0),SSID,BSSID,CH,WIDTH,CENTER,RSSI");
    tft.setCursor(20, 100);
    tft.println("Format: isFTM(1),BSSID,range[m],rtt_raw[ns],rtt_est[ns],RSSI");
    tft.setCursor(20, 120);
    tft.println("Capturing WiFi Beacons...");
  }
}

void drawAPCurve(const APInfo& ap, uint16_t color) {
  const BandRect& br = ap.is5g ? band5g : band2g;
  int x0 = mapFreqToX(ap.centerFreqMHz, ap.is5g);
  int yPeak = mapRSSIToY(ap.rssiEMA, br);
  int yBottom = br.y1;

  int halfBW = ap.bandwidthMHz / 2;
  int w = mapFreqToX(ap.centerFreqMHz + halfBW, ap.is5g)
        - mapFreqToX(ap.centerFreqMHz, ap.is5g);
  float a = (float)(yBottom - yPeak) / (float)(w * w);

  int xmin = x0 - w;
  int xmax = x0 + w;

  int prevX = xmin;
  int prevY = yBottom;

  for (int x = xmin+1; x <= xmax; ++x) {
    int y = yPeak + (int)round(a * (x - x0) * (x - x0));
    if (y > yBottom) y = yBottom;
    tft.drawLine(prevX, prevY, x, y, color);
    prevX = x; prevY = y;
  }

  tft.setTextColor(color);
  tft.setTextSize(1);
  int labelY = max(br.y0 + 2, yPeak - 10);
  int labelX = max(BAND_MARGIN_L, x0 - 20);
  tft.setCursor(labelX, labelY);
  tft.print(ap.ssid);
}

void eraseAPCurve(const APInfo& ap) {
  drawAPCurve(ap, ILI9341_BLACK);
  if (ap.is5g) drawGridForBand(band5g, true);
  else         drawGridForBand(band2g, false);
}

inline bool isZeroMac(const uint8_t mac[6]) {
  uint8_t z[6] = {0};
  return memcmp(mac, z, 6) == 0;
}

// SSID抽出（ISRで使える最小限）
void extractSsid_min(const uint8_t* payload, int len, char* out, uint8_t& outLen) {
  int ieStart = 36;
  outLen = 0;
  for (int i = ieStart; i + 2 < len; ) {
    uint8_t id = payload[i];
    uint8_t l  = payload[i+1];
    if (i + 2 + l > len) break;
    if (id == 0) {
      if (l == 0) { outLen = 0; return; } // hidden
      uint8_t n = (l > 32) ? 32 : l;
      for (uint8_t j = 0; j < n; ++j) out[j] = (char)payload[i+2+j];
      outLen = n;
      return;
    }
    i += (2 + l);
  }
  outLen = 0;
}

bool isBeaconOrProbeResp(const uint8_t* payload, int len) {
  if (len < 24) return false;
  uint16_t fc = payload[0] | (payload[1] << 8);
  uint8_t subtype = (fc >> 4) & 0xF;
  return (subtype == 8 || subtype == 5);
}

extern "C" int ets_printf(const char *fmt, ...);

// 5GHzの中心チャネル妥当性（地域依存だが代表値でガード）
static inline IRAM_ATTR bool is_valid_80_center(int c) {
  // 42, 58, 106, 122, 138, 155 あたりが代表
  return (c==42 || c==58 || c==106 || c==122 || c==138 || c==155);
}
static inline IRAM_ATTR bool is_valid_160_center(int c) {
  // 50, 114 が代表（地域により異なる）
  return (c==50 || c==114);
}

// HT/VHT/HE 統合パース（VHT優先、HEはクロスチェックのみ）
static inline IRAM_ATTR BwInfo parse_bw(const uint8_t* p, int len) {
  BwInfo out{};
  int i = 36;
  bool have_vht = false;
  bool have_ht  = false;

  while (i + 2 < len) {
    uint8_t id = p[i], l = p[i+1];
    if (i + 2 + l > len) break;

    // HT Operation (ID=61):
    // data[0] = primary channel
    // data[1] bits0-1 = secondary channel offset (1=above, 3=below)
    if (id == 61 && l >= 2) {
      const uint8_t* d = &p[i+2];
      out.primary = d[0];
      uint8_t sec_off = d[1] & 0x3;
      if (sec_off == 1 || sec_off == 3) {
        out.bandwidth = 40;
        out.ht_sec_off = sec_off;
      } else {
        out.bandwidth = 20; // HTがあってもsec_off未設定なら20
      }
      have_ht = true;
    }

    // DS Parameter Set (ID=3): primary（2.4で有効、5GHzでは無いことも）
    if (id == 3 && l == 1 && out.primary == 0) {
      out.primary = p[i+2];
    }

    // VHT Operation (ID=192): width + center segments
    if (id == 192 && l >= 3) {
      const uint8_t* d = &p[i+2];
      uint8_t ch_width = d[0];
      uint8_t seg0     = (l >= 4) ? d[1] : 0;
      uint8_t seg1     = (l >= 5) ? d[2] : 0;

      switch (ch_width) {
        case 0: // 20
          out.bandwidth = 20;
          break;
        case 1: // 40
          out.bandwidth = 40;
          break;
        case 2: // 80
          out.bandwidth = 80;
          out.center = seg0;
          break;
        case 3: // 160
          out.bandwidth = 160;
          out.center = seg1 ? seg1 : seg0;
          break;
        default:
          break;
      }
      have_vht = true;
    }

    // HE Operation (Extension IE, ID=255, ExtID=36)
    // 幅は採用しない。VHTが無いときのcenter補助やprimaryクロスチェックに限定。
    if (id == 255 && l >= 6) {
      const uint8_t* d = &p[i+2];
      if (d[0] == 36) {  // Ext ID = HE Operation
        uint8_t he_primary = d[1];     // 実装差あり。存在すればprimaryのクロスチェックに使用。
        uint8_t he_seg0    = d[4];
        uint8_t he_seg1    = d[5];
        if (out.primary == 0 && he_primary != 0) out.primary = he_primary;
        // VHTが無い場合に限り、seg0/seg1を参考にする（80/160は強制採用しない）
        if (!have_vht) {
          // 80/160は誤検出を避けるため採用しない。40の上下はHTに依存。
          // ただし中心が未設定で、he_seg0が合法っぽい場合は「候補」として使うことは可能。
          if (out.center == 0 && is_valid_80_center(he_seg0)) {
            // 参考値として保持（最終決定は下の整合チェックで）
            out.center = he_seg0;
          }
        }
      }
    }

    i += 2 + l;
  }

  // 5GHz判定はチャネル>=36
  out.is5g = (out.primary >= 36);

  // 40MHz中心（HTの上下で確定）。上下不明なら20扱い。
  if (out.bandwidth == 40) {
    if (out.ht_sec_off == 1)      out.center = out.primary + 2;
    else if (out.ht_sec_off == 3) out.center = out.primary - 2;
    else {
      out.bandwidth = 20;
      out.center = out.primary;
    }
  }

  // 80/160の妥当性（VHTが示したcenterを検証）
  if (out.bandwidth == 80) {
    if (!is_valid_80_center(out.center) || !out.is5g) {
      out.bandwidth = 20;
      out.center = out.primary;
    }
  } else if (out.bandwidth == 160) {
    if (!is_valid_160_center(out.center) || !out.is5g) {
      // 160誤検出を強く抑制：80/20にダウングレード。seg0を持っていれば80に落とす選択肢もあるが保守的に20へ
      out.bandwidth = 20;
      out.center = out.primary;
    }
  }

  // 最終フォールバック
  if (out.bandwidth == 20 || out.center == 0) {
    out.bandwidth = 20;
    out.center = out.primary;
  }

  out.valid = (out.primary > 0) && (out.center > 0);
  return out;
}

static inline IRAM_ATTR bool valid_channel(int c) {
  return (c >= 1 && c <= 14) || (c >= 36 && c <= 165);
}

// ISR最小化：BeaconMiniのみQueueへ＋短い行のUART出力
static void IRAM_ATTR sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;

  wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* payload = ppkt->payload;
  int len = ppkt->rx_ctrl.sig_len;
  if (len < 36) return;

  if (!isBeaconOrProbeResp(payload, len)) return;

  const uint8_t* bssid = payload + 16;
  uint8_t zero[6] = {0};
  if (memcmp(bssid, zero, 6) == 0) return;

  BwInfo bwinfo = parse_bw(payload, len);

  // SSID抽出（最大32）
  BeaconMini m{};
  m.ssidLen = 0;
  extractSsid_min(payload, len, m.ssid, m.ssidLen);

  memcpy(m.bssid, bssid, 6);
  m.primary = (uint16_t)(valid_channel(bwinfo.primary) ? bwinfo.primary : 0);
  m.center  = (uint16_t)(valid_channel(bwinfo.center)  ? bwinfo.center  : m.primary);
  m.rssi    = ppkt->rx_ctrl.rssi;
  m.ftmCap  = hasFtmResponderExtCap_fromIE(payload, len) ? 1 : 0;
  m.len     = (uint16_t)len;
  m.width   = (uint8_t)bwinfo.bandwidth;

  // UART出力（短く・一定長）
  char ssidBuf[33];
  uint8_t n = m.ssidLen;
  for (uint8_t i = 0; i < n; ++i) ssidBuf[i] = m.ssid[i];
  ssidBuf[n] = '\0';
  ets_printf("0,%s,%02X:%02X:%02X:%02X:%02X:%02X,%d,%d,%d,%d\n",
             (n==0) ? ((len>36) ? "<hidden>" : "<no-ssid>") : ssidBuf,
             bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
             (int)m.primary, (int)m.width, (int)m.center, (int)ppkt->rx_ctrl.rssi);

  // Queueへ（満杯ならドロップ）
  xQueueSendFromISR(beaconQueue, &m, nullptr);
}

// Queue→バッファ反映（メイン側のみヒープ操作 & SSID保持）
void pumpQueueToBuffers() {
  BeaconMini m;
  while (xQueueReceive(beaconQueue, &m, 0) == pdPASS) {
    if (scanBuffer.size() < SCAN_MAX_APS) {
      APInfo ap{};
      memcpy(ap.bssid, m.bssid, 6);
      ap.ssid = (m.ssidLen == 0) ? "<hidden>" : String(m.ssid).substring(0, m.ssidLen);
      ap.is5g = (m.primary >= 36);
      ap.primaryCh = (int)m.primary;
      ap.centerCh  = (int)m.center;
      ap.centerFreqMHz = ap.is5g ? centerFreqMHz_5g(ap.centerCh)
                                 : centerFreqMHz_2g(ap.centerCh);
      ap.bandwidthMHz = (int)m.width;
      ap.rssiEMA = (float)m.rssi;
      ap.lastSeenMs = nowMs();
      ap.color = randomColor();
      scanBuffer.push_back(ap);
    }

    // FTM候補（1スイープ中は重複MACを除外、FTMはprimaryチャネル使用）
    if (m.ftmCap && m.primary != 0 && ftmCandidates.size() < FTM_MAX_CAND) {
      uint64_t mac64 = 0;
      for (int i = 0; i < 6; ++i) mac64 = (mac64 << 8) | m.bssid[i];
      bool seen = false;
      for (auto &v : ftmSeenMacs) { if (v == mac64) { seen = true; break; } }
      if (!seen) {
        accessPoint_t cand{};
        memcpy(cand.mac, m.bssid, 6);
        cand.channel = m.primary;
        ftmCandidates.push_back(cand);
        ftmSeenMacs.push_back(mac64);
      }
    }
  }
}

// 表示セットの消去→再描画
void renderNewScan() {
  for (auto &ap : displaySet) drawAPCurve(ap, ILI9341_BLACK);
  drawGridForBand(band2g, false);
  drawGridForBand(band5g, true);
  for (auto &ap : scanBuffer) drawAPCurve(ap, ap.color);
  displaySet = scanBuffer;
}

void pauseSniffer() {
  esp_wifi_set_promiscuous_rx_cb(nullptr);
  esp_wifi_set_promiscuous(false);
}
void resumeSniffer() {
  wifi_promiscuous_filter_t filter{};
  filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(sniffer_cb);
}

// FTMレポートハンドラ（freeは使わず公式APIでコピー）
void onFtmReport(arduino_event_t *event) {
  wifi_event_ftm_report_t *report = &event->event_info.wifi_ftm_report;
  bool ok = (report->status == FTM_STATUS_SUCCESS);

  char mac[18];
  sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X",
          report->peer_mac[0], report->peer_mac[1], report->peer_mac[2],
          report->peer_mac[3], report->peer_mac[4], report->peer_mac[5]);

  float dist_m = ok ? ((float)report->dist_est / 100.0f) : -1.0f;
  unsigned long rtt_raw_ns = ok ? (unsigned long)report->rtt_raw : 0UL;
  unsigned long rtt_est_ns = ok ? (unsigned long)report->rtt_est : 0UL;

  wifi_ftm_report_entry_t entries[8];
  int num = 0;
  int rssi_out = -128;

  if (ok) {
    num = esp_wifi_ftm_get_report(entries, 8);
    if (num > 0) rssi_out = entries[0].rssi;
  }

  Serial.printf("1,%s,%.2f,%lu,%lu,%d\n",
                mac, dist_m, rtt_raw_ns, rtt_est_ns, rssi_out);

  xSemaphoreGive(ftmSemaphore);
}

// 初期化
void setup() {
  Serial.begin(115200);
  pinMode(DRAW_SWITCH_PIN, INPUT);

  tft.begin(40000000);
  tft.setRotation(1);
  setupLayout(digitalRead(DRAW_SWITCH_PIN));

  WiFi.mode(WIFI_STA);

  beaconQueue = xQueueCreate(BEACON_QUEUE_SIZE, sizeof(BeaconMini));

  wifi_promiscuous_filter_t filter{};
  filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filter);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(sniffer_cb);

  randomSeed(esp_timer_get_time());

  ftmSemaphore = xSemaphoreCreateBinary();
  WiFi.onEvent(onFtmReport, ARDUINO_EVENT_WIFI_FTM_REPORT);
}

void startDwell(int ch, SweepBand band) {
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  dwellStart = nowMs();
}

void startNewSweep() {
  scanBuffer.clear();
  ftmCandidates.clear();
  ftmSeenMacs.clear();
  currentBand = BAND_2G;
  hopIndex = 0;
  startDwell(channels2g[hopIndex], currentBand);
}

enum FtmState { FTM_IDLE, FTM_RUNNING, FTM_WAIT };
static FtmState ftmState = FTM_IDLE;
static size_t ftmIndex = 0;
static uint32_t ftmStartMs = 0;

bool startFtm(const accessPoint_t& t) {
  esp_wifi_set_channel(t.channel, WIFI_SECOND_CHAN_NONE);
  return WiFi.initiateFTM(FTM_FRAME_COUNT, FTM_BURST_PERIOD, (uint8_t)t.channel, t.mac);
}

void ftmTickNonBlocking() {
  switch (ftmState) {
    case FTM_IDLE:
      if (!ftmCandidates.empty()) { ftmIndex = 0; ftmState = FTM_RUNNING; }
      break;
    case FTM_RUNNING:
      if (ftmIndex < ftmCandidates.size()) {
        auto &t = ftmCandidates[ftmIndex];
        uint16_t ch = t.channel;
        bool chValid = valid_channel(ch);
        if (chValid && !isZeroMac(t.mac)) {
          bool ok = startFtm(t);
          if (ok) { ftmStartMs = millis(); ftmState = FTM_WAIT; break; }
        }
        ftmIndex++;
      } else {
        ftmCandidates.clear();
        ftmState = FTM_IDLE;
      }
      break;
    case FTM_WAIT:
      if (xSemaphoreTake(ftmSemaphore, 0) == pdPASS || (millis() - ftmStartMs) > 3000) {
        ftmIndex++; ftmState = FTM_RUNNING;
      }
      break;
  }
}

void loop() {
  uint32_t now = nowMs();

  static bool started = false;
  static int modeSwitch_now = 0;
  static int modeSwitch_old = 0;

  pumpQueueToBuffers();
  ftmTickNonBlocking();

  modeSwitch_now = digitalRead(DRAW_SWITCH_PIN);
  if (modeSwitch_now != modeSwitch_old) { started = false; setupLayout(modeSwitch_now); }

  if (!started) { startNewSweep(); started = true; }

  if (now - dwellStart >= DWELL_MS) {
    hopIndex++;
    bool bandDone = false;

    if (currentBand == BAND_2G) {
      if (hopIndex < (int)(sizeof(channels2g) / sizeof(channels2g[0]))) startDwell(channels2g[hopIndex], currentBand);
      else bandDone = true;
    } else {
      if (hopIndex < (int)(sizeof(channels5g) / sizeof(channels5g[0]))) startDwell(channels5g[hopIndex], currentBand);
      else bandDone = true;
    }

    if (bandDone) {
      if (currentBand == BAND_2G) {
        currentBand = BAND_5G; hopIndex = 0; startDwell(channels5g[hopIndex], currentBand);
      } else {
        if (modeSwitch_now == MONITOR) {
          renderNewScan();
        } else if (modeSwitch_now == SERIALOUT) {
          for (auto &ap : scanBuffer) {
            char bssidStr[18];
            sprintf(bssidStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                    ap.bssid[0], ap.bssid[1], ap.bssid[2],
                    ap.bssid[3], ap.bssid[4], ap.bssid[5]);
            Serial.printf("0,%s,%s,%d,%d,%d,%d\n",
                          ap.ssid.c_str(),
                          bssidStr,
                          ap.primaryCh,         // CH (primary)
                          ap.bandwidthMHz,      // WIDTH
                          ap.centerCh,          // CENTER
                          (int)ap.rssiEMA);     // RSSI
            vTaskDelay(1);
          }
          Serial.flush();
        }
        startNewSweep();
      }
    }
  }

  modeSwitch_old = modeSwitch_now;
}