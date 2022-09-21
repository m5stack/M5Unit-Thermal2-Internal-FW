#include <Arduino.h>
#include <Wire.h>

#include <M5Unified.h>

static M5Canvas canvas_graph;
static M5Canvas canvas_hist;

struct rect_t
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  void set(int16_t x_, int16_t y_, int16_t w_, int16_t h_)
  {
    x = x_;
    y = y_;
    w = w_;
    h = h_;
  }
};

static rect_t camera_area;
static rect_t text_area;
static rect_t graph_area;
static rect_t histogram_area;

static constexpr size_t hist_count = 128;
static uint16_t histgram[hist_count] = {0};
static int16_t prev_x[hist_count] = { 0 };
static bool prev_line[hist_count] = { true };
static uint32_t frame_counter = 0;
static uint32_t fps = 0;

static uint8_t register_prev[0x80] = {0xFF};
static uint8_t register_data[0x80] = {0};

enum check_list_t
{
  button_click,
  refresh_64Hz,
  refresh_32Hz,
  refresh_16Hz,
  buzzer_led,
  register_write,
  check_list_max,
};

enum check_state_t
{
  unknown,
  progress,
  success,
  failed,
};

static constexpr char* test_name[] = {
  "Button Clk",
  " Rate 64Hz",
  " Rate 32Hz",
  " Rate 16Hz",
  "Buzzer&Led",
  " Reg Write",
};

static uint8_t check_list[check_list_max] = {0};
static uint8_t check_prev[check_list_max] = {0};

static constexpr int block_width = 31; // グラフ目盛り線の間隔
// static constexpr int graph_width = 64;
// static constexpr int graph_height = 168;
// static constexpr size_t wave_count = graph_width + block_width;
static constexpr size_t wave_count = 320 + block_width;
static float wave_datas[4][wave_count]; // 温度4種類のグラフ表示用履歴データ
static size_t wave_index; // グラフデータ配列の代入先インデクス
static float min_value;       // グラフ表示範囲の最小値
static float max_value;       // グラフ表示範囲の最大値
// static int drawarea_y_offset = 63; // グラフ描画先のY座標オフセット
static int drawarea_x_offset; // グラフ描画先のX座標オフセット 

static bool flgInfo = false;

static constexpr int i2c_addr = 0x32;
static constexpr int pin_sda = 21;
static constexpr int pin_scl = 22;

#pragma pack(push)
#pragma pack(1)
struct ctrl_reg_t
{
  uint16_t device_id;
  uint8_t mode;
  uint8_t refresh_rate;
  uint8_t buzzer_volume;
  uint8_t fahrenheit;
  uint8_t enable_function;
  uint8_t enable_wireless;
  uint32_t reserve0x08_0x0B;
  uint32_t reserve0x0C_0x0F;
  uint16_t alert_threshold_temp;
  uint16_t alert_threshold_diff;
};

static constexpr const int step_table[] = { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, INT_MAX };

//*  // color
static constexpr const uint32_t color_table[] = {
0x0000FFu,0x0003FFu,0x0006FFu,0x0009FFu,0x000CFFu,0x000FFFu,0x0012FFu,0x0016FFu,
0x0019FEu,0x001CFEu,0x001FFEu,0x0022FDu,0x0025FDu,0x0028FCu,0x002BFCu,0x002FFBu,
0x0032FBu,0x0035FAu,0x0038F9u,0x003BF9u,0x003EF8u,0x0041F7u,0x0044F6u,0x0047F6u,
0x004AF5u,0x004DF4u,0x0050F3u,0x0053F2u,0x0056F1u,0x0059F0u,0x005CEFu,0x005FEEu,
0x0062ECu,0x0065EBu,0x0068EAu,0x006AE9u,0x006DE7u,0x0070E6u,0x0073E5u,0x0076E3u,
0x0079E2u,0x007BE0u,0x007EDFu,0x0081DDu,0x0084DCu,0x0086DAu,0x0089D8u,0x008CD7u,
0x008ED5u,0x0091D3u,0x0093D1u,0x0096CFu,0x0098CEu,0x009BCCu,0x009DCAu,0x00A0C8u,
0x00A2C6u,0x00A5C4u,0x00A7C2u,0x00AAC0u,0x00ACBEu,0x00AEBCu,0x00B1B9u,0x00B3B7u,
0x00B5B5u,0x00B7B3u,0x00B9B1u,0x00BCAEu,0x00BEACu,0x00C0AAu,0x00C2A7u,0x00C4A5u,
0x00C6A2u,0x00C8A0u,0x00CA9Du,0x00CC9Bu,0x00CE98u,0x00CF96u,0x00D193u,0x00D391u,
0x00D58Eu,0x00D78Cu,0x00D889u,0x00DA86u,0x00DC84u,0x00DD81u,0x00DF7Eu,0x00E07Bu,
0x00E279u,0x00E376u,0x00E573u,0x00E670u,0x00E76Du,0x00E96Au,0x00EA68u,0x00EB65u,
0x00EC62u,0x00EE5Fu,0x00EF5Cu,0x00F059u,0x00F156u,0x00F253u,0x00F350u,0x00F44Du,
0x00F54Au,0x00F647u,0x00F644u,0x00F741u,0x00F83Eu,0x00F93Bu,0x00F938u,0x00FA35u,
0x00FB32u,0x00FB2Fu,0x00FC2Bu,0x00FC28u,0x00FD25u,0x00FD22u,0x00FE1Fu,0x00FE1Cu,
0x00FE19u,0x00FF16u,0x00FF12u,0x00FF0Fu,0x00FF0Cu,0x00FF09u,0x00FF06u,0x00FF03u,
0x03FF00u,0x06FF00u,0x09FF00u,0x0CFF00u,0x0FFF00u,0x12FF00u,0x16FF00u,0x19FE00u,
0x1CFE00u,0x1FFE00u,0x22FD00u,0x25FD00u,0x28FC00u,0x2BFC00u,0x2FFB00u,0x32FB00u,
0x35FA00u,0x38F900u,0x3BF900u,0x3EF800u,0x41F700u,0x44F600u,0x47F600u,0x4AF500u,
0x4DF400u,0x50F300u,0x53F200u,0x56F100u,0x59F000u,0x5CEF00u,0x5FEE00u,0x62EC00u,
0x65EB00u,0x68EA00u,0x6AE900u,0x6DE700u,0x70E600u,0x73E500u,0x76E300u,0x79E200u,
0x7BE000u,0x7EDF00u,0x81DD00u,0x84DC00u,0x86DA00u,0x89D800u,0x8CD700u,0x8ED500u,
0x91D300u,0x93D100u,0x96CF00u,0x98CE00u,0x9BCC00u,0x9DCA00u,0xA0C800u,0xA2C600u,
0xA5C400u,0xA7C200u,0xAAC000u,0xACBE00u,0xAEBC00u,0xB1B900u,0xB3B700u,0xB5B500u,
0xB7B300u,0xB9B100u,0xBCAE00u,0xBEAC00u,0xC0AA00u,0xC2A700u,0xC4A500u,0xC6A200u,
0xC8A000u,0xCA9D00u,0xCC9B00u,0xCE9800u,0xCF9600u,0xD19300u,0xD39100u,0xD58E00u,
0xD78C00u,0xD88900u,0xDA8600u,0xDC8400u,0xDD8100u,0xDF7E00u,0xE07B00u,0xE27900u,
0xE37600u,0xE57300u,0xE67000u,0xE76D00u,0xE96A00u,0xEA6800u,0xEB6500u,0xEC6200u,
0xEE5F00u,0xEF5C00u,0xF05900u,0xF15600u,0xF25300u,0xF35000u,0xF44D00u,0xF54A00u,
0xF64700u,0xF64400u,0xF74100u,0xF83E00u,0xF93B00u,0xF93800u,0xFA3500u,0xFB3200u,
0xFB2F00u,0xFC2B00u,0xFC2800u,0xFD2500u,0xFD2200u,0xFE1F00u,0xFE1C00u,0xFE1900u,
0xFF1600u,0xFF1200u,0xFF0F00u,0xFF0C00u,0xFF0900u,0xFF0600u,0xFF0300u,0xFF0000u,
};
/*/  // monochrome
static constexpr const uint32_t color_table[] = {
0x020000u, 0x030100u, 0x040200u, 0x050301u, 0x060402u, 0x070503u, 0x080604u, 0x090705u, 
0x0A0806u, 0x0B0907u, 0x0C0A08u, 0x0D0B09u, 0x0E0C0Au, 0x0F0D0Bu, 0x100E0Cu, 0x110F0Du, 
0x12100Eu, 0x13110Fu, 0x141210u, 0x151311u, 0x161412u, 0x171513u, 0x181614u, 0x191715u, 
0x1A1816u, 0x1B1917u, 0x1C1A18u, 0x1D1B19u, 0x1E1C1Au, 0x1F1D1Bu, 0x201E1Cu, 0x211F1Du, 
0x22201Eu, 0x23211Fu, 0x242220u, 0x252321u, 0x262422u, 0x272523u, 0x282624u, 0x292725u, 
0x2A2826u, 0x2B2927u, 0x2C2A28u, 0x2D2B29u, 0x2E2C2Au, 0x2F2D2Bu, 0x302E2Cu, 0x312F2Du, 
0x32302Eu, 0x33312Fu, 0x343230u, 0x353331u, 0x363432u, 0x373533u, 0x383634u, 0x393735u, 
0x3A3836u, 0x3B3937u, 0x3C3A38u, 0x3D3B39u, 0x3E3C3Au, 0x3F3D3Bu, 0x403E3Cu, 0x413F3Du, 
0x42403Eu, 0x43413Fu, 0x444240u, 0x454341u, 0x464442u, 0x474543u, 0x484644u, 0x494745u, 
0x4A4846u, 0x4B4947u, 0x4C4A48u, 0x4D4B49u, 0x4E4C4Au, 0x4F4D4Bu, 0x504E4Cu, 0x514F4Du, 
0x52504Eu, 0x53514Fu, 0x545250u, 0x555351u, 0x565452u, 0x575553u, 0x585654u, 0x595755u, 
0x5A5856u, 0x5B5957u, 0x5C5A58u, 0x5D5B59u, 0x5E5C5Au, 0x5F5D5Bu, 0x605E5Cu, 0x615F5Du, 
0x62605Eu, 0x63615Fu, 0x646260u, 0x656361u, 0x666462u, 0x676563u, 0x686664u, 0x696765u, 
0x6A6866u, 0x6B6967u, 0x6C6A68u, 0x6D6B69u, 0x6E6C6Au, 0x6F6D6Bu, 0x706E6Cu, 0x716F6Du, 
0x72706Eu, 0x73716Fu, 0x747270u, 0x757371u, 0x767472u, 0x777573u, 0x787674u, 0x797775u, 
0x7A7876u, 0x7B7977u, 0x7C7A78u, 0x7D7B79u, 0x7E7C7Au, 0x7F7D7Bu, 0x807E7Cu, 0x817F7Du, 
0x82807Eu, 0x83817Fu, 0x848280u, 0x858381u, 0x868482u, 0x878583u, 0x888684u, 0x898785u, 
0x8A8886u, 0x8B8987u, 0x8C8A88u, 0x8D8B89u, 0x8E8C8Au, 0x8F8D8Bu, 0x908E8Cu, 0x918F8Du, 
0x92908Eu, 0x93918Fu, 0x949290u, 0x959391u, 0x969492u, 0x979593u, 0x989694u, 0x999795u, 
0x9A9896u, 0x9B9997u, 0x9C9A98u, 0x9D9B99u, 0x9E9C9Au, 0x9F9D9Bu, 0xA09E9Cu, 0xA19F9Du, 
0xA2A09Eu, 0xA3A19Fu, 0xA4A2A0u, 0xA5A3A1u, 0xA6A4A2u, 0xA7A5A3u, 0xA8A6A4u, 0xA9A7A5u, 
0xAAA8A6u, 0xABA9A7u, 0xACAAA8u, 0xADABA9u, 0xAEACAAu, 0xAFADABu, 0xB0AEACu, 0xB1AFADu, 
0xB2B0AEu, 0xB3B1AFu, 0xB4B2B0u, 0xB5B3B1u, 0xB6B4B2u, 0xB7B5B3u, 0xB8B6B4u, 0xB9B7B5u, 
0xBAB8B6u, 0xBBB9B7u, 0xBCBAB8u, 0xBDBBB9u, 0xBEBCBAu, 0xBFBDBBu, 0xC0BEBCu, 0xC1BFBDu, 
0xC2C0BEu, 0xC3C1BFu, 0xC4C2C0u, 0xC5C3C1u, 0xC6C4C2u, 0xC7C5C3u, 0xC8C6C4u, 0xC9C7C5u, 
0xCAC8C6u, 0xCBC9C7u, 0xCCCAC8u, 0xCDCBC9u, 0xCECCCAu, 0xCFCDCBu, 0xD0CECCu, 0xD1CFCDu, 
0xD2D0CEu, 0xD3D1CFu, 0xD4D2D0u, 0xD5D3D1u, 0xD6D4D2u, 0xD7D5D3u, 0xD8D6D4u, 0xD9D7D5u, 
0xDAD8D6u, 0xDBD9D7u, 0xDCDAD8u, 0xDDDBD9u, 0xDEDCDAu, 0xDFDDDBu, 0xE0DEDCu, 0xE1DFDDu, 
0xE2E0DEu, 0xE3E1DFu, 0xE4E2E0u, 0xE5E3E1u, 0xE6E4E2u, 0xE7E5E3u, 0xE8E6E4u, 0xE9E7E5u, 
0xEAE8E6u, 0xEBE9E7u, 0xECEAE8u, 0xEDEBE9u, 0xEEECEAu, 0xEFEDEBu, 0xF0EEECu, 0xF1EFEDu, 
0xF2F0EEu, 0xF3F1EFu, 0xF4F2F0u, 0xF5F3F1u, 0xF6F4F2u, 0xF7F5F3u, 0xF8F6F4u, 0xF9F7F5u, 
0xFAF8F6u, 0xFBF9F7u, 0xFCFAF8u, 0xFDFBF9u, 0xFEFCFAu, 0xFFFDFBu, 0xFFFEFCu, 0xFFFFFDu, 
};
//*/
struct temperature_info_t
{
  uint16_t temp;
  uint8_t x;
  uint8_t y;
};
struct temp_data_t
{
  int8_t refresh_control;
  uint8_t subpage;
  uint16_t med_temp;
  uint16_t avg_temp;
  temperature_info_t diff_info;
  temperature_info_t min_info;
  temperature_info_t max_info;
  uint16_t data[16 * 24];
};
#pragma pack(pop)

temp_data_t tempdatas[2];
int idx_recv = 0;
int idx_draw = 0;

struct value_avg_t {
  size_t exec(size_t src, size_t margin) {
    target = (target * 3 + src) / 4.0f;
    auto temp = current;
    int diff = roundf(target - temp);
    if (add || abs(diff) > margin)
    {
      add = (add + diff) / 4 + ((temp < target) ? 1 : -1);
      temp += add;
      current = temp;
    }
    return roundf(temp);
  }
  value_avg_t(float default_value) : target { default_value }, current { default_value } {}

private:
  int add = 0;
  float target = (24 + 64) << 7;
  float current = (24 + 64) << 7;
};

//*
void drawRegister(void)
{
  int fh = 10;
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextDatum(textdatum_t::top_left);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

  for (int i = 0; i < 0x80; ++i)
  {
    if (register_prev[i] == register_data[i]) { continue; }
    uint_fast8_t val = register_data[i];
    register_prev[i] = val;

    int x = i & 15;
    int y = (i >> 4);
    uint8_t bc, fc;
    bc = val;
    uint32_t rgb = lgfx::color_convert<lgfx::rgb888_t, lgfx::rgb332_t>(val);
    uint32_t v = ((rgb >> 16) & 0xFF) + ((rgb>>8) & 0xFF)*2 + (rgb & 0xFF);
    if (v < 512)
    {
      fc = v ? 0xFFu : 0x92u;
    }
    else
    {
      fc = 0u;
    }
    M5.Display.setTextColor(fc, bc);
    M5.Display.setCursor( (x + 1) * 13
                        , (y + 1) * fh + 128);
    M5.Display.printf("%02x", val);
  }
}

void drawCheck(void)
{
  int fh = 14;
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextDatum(textdatum_t::top_left);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

  for (int i = 0; i < check_list_max; ++i)
  {
    if (check_prev[i] == check_list[i]) { continue; }
    uint_fast8_t val = check_list[i];
    check_prev[i] = val;
    int y = i;
    M5.Display.setCursor( 240 , y * fh + 140);

    switch (val) {
    default:
      if (val == check_state_t::progress) {
        M5.Display.setTextColor(0xFFFFFFu, 0x181818u);
      } else {
        M5.Display.setTextColor(0xAAAAAAu, 0x181818u);
      }
      M5.Display.print(test_name[i]);
      break;

    case check_state_t::failed:
      M5.Display.setTextColor(0xFFFF00u, 0xFF0000u);
      M5.Display.print("  FAILED  ");
      break;

    case check_state_t::success:
      M5.Display.setTextColor(0x000000u, 0x00C000u);
      M5.Display.print(" COMPLETE ");
      break;
    }
  }
}

void drawToLCD(const temp_data_t* tempdata)
{
  static int16_t values[32*24];
  static value_avg_t min_avg = { (16 + 64) << 8 };
  static value_avg_t max_avg = { (48 + 64) << 8 };

  /// 2フレーム分の値の合計
  static uint32_t min_sum = 0;
  static uint32_t max_sum = 0;
  static uint32_t avg_sum = 0;
  static uint32_t med_sum = 0;

  static uint32_t min_subpage = 0;
  static uint32_t max_subpage = 0;
  static uint32_t avg_subpage = 0;
  static uint32_t med_subpage = 0;

  /// ヒストグラム用の下限と上限
  static int min_temp;
  static int max_temp;

  if (tempdata->min_info.temp >= tempdata->max_info.temp) {
    return;
  }

  bool subpage = tempdata->subpage;

  if (subpage)
  {
    min_subpage = tempdata->min_info.temp;
    max_subpage = tempdata->max_info.temp;
    avg_subpage = tempdata->avg_temp;
    med_subpage = tempdata->med_temp;
  }
  else
  {
    min_sum = min_subpage + tempdata->min_info.temp;
    max_sum = max_subpage + tempdata->max_info.temp;
    avg_sum = avg_subpage + tempdata->avg_temp;
    med_sum = med_subpage + tempdata->med_temp;

    int margin = (max_sum - min_sum + 31) >> 3;

    min_temp = min_avg.exec(min_sum - (margin >> 2), margin) >> 1;
    max_temp = max_avg.exec(max_sum + (margin >> 2), margin) >> 1;

    if (min_temp >= max_temp) {
      std::swap(min_temp, max_temp);
      --min_temp;
      ++max_temp;
    }
  }


  int temp_diff = max_temp - min_temp;
  if (temp_diff == 0) return;
  int div = (1<<24) / temp_diff;

  for (int y = 0; y < 24; ++y)
  {
    bool shift = (subpage ^ y) & 1;
    auto src = &tempdata->data[y * 16];

    auto value = &values[y*32];
    for (int x = 0; x < 16; ++x)
    {
      value[(x<<1) + shift] = src[x];
    }
  }

  for (int y = 0; y < 24; ++y)
  {
    auto vy0 = camera_area.y + (camera_area.h *  y    / 24);
    auto vy1 = camera_area.y + (camera_area.h * (y+1) / 24);
    bool shift = ((subpage ^ y) & 1);
    for (int x = 0; x < 16; ++x)
    {
      int xpos = (x<<1)+(shift);
      auto vx0 = camera_area.x + (camera_area.w *  xpos    >> 5);
      auto vx1 = camera_area.x + (camera_area.w * (xpos+1) >> 5);
      int pn = xpos + y*32;
      int temp = values[pn];
      bool lowtemp = (tempdata->min_info.temp >= temp);
      bool hightemp = (tempdata->max_info.temp <= temp);

      temp = (temp - min_temp) * div >> 16;
      if (temp < 0) temp = 0;
      else if (temp > 255) temp = 255;
      histgram[temp >> 1] ++;
      M5.Display.fillRect(vx0, vy0, vx1 - vx0, vy1 - vy0, color_table[temp]);
      // M5.Display.fillRect(vx0, vy0, vx1 - vx0, vy1 - vy0, temp * 0x10101u);

      if (tempdata->diff_info.x == xpos && tempdata->diff_info.y == y)
      {
        M5.Display.drawRect(vx0 + 1, vy0 + 1, vx1 - vx0 - 2, vy1 - vy0 - 2, 0xFF00FFu);
      }
      if (tempdata->min_info.x == xpos && tempdata->min_info.y == y)
      {
        M5.Display.drawRect(vx0, vy0, vx1 - vx0, vy1 - vy0, 0x00FFFFu);
      }
      else
      if (tempdata->max_info.x == xpos && tempdata->max_info.y == y)
      {
        M5.Display.drawRect(vx0, vy0, vx1 - vx0, vy1 - vy0, 0xFFFF00u);
      }
    }
  }
  // {
  //   auto vx0 = viewarea_w *  8 >> 5;
  //   auto vx1 = viewarea_w * 24 >> 5;
  //   auto vy0 = viewarea_h *  4 / 24;
  //   auto vy1 = viewarea_h * 20 / 24;
  //   M5.Display.drawRect(vx0 - 1, vy0 - 1, vx1 - vx0 + 2, vy1 - vy0 + 2, 0xFFFFFFu);
  // }

  // static int plot_x = viewarea_w;
  // int plot_y = 0;

  ++frame_counter;
  static uint32_t prev_sec;
  uint32_t sec = millis() / 1000;
  M5.Display.setTextDatum(textdatum_t::top_left);
  if (prev_sec != sec)
  {
    prev_sec = sec;
    fps = frame_counter;
    frame_counter = 0;

    int_fast16_t top = text_area.y + 10;
    int_fast16_t height = text_area.h - 10;

    M5.Display.setCursor(text_area.x + text_area.w - 36, text_area.y);  M5.Display.setTextColor(TFT_WHITE, 0);
    M5.Display.printf("% 3dfps", fps);

    M5.Display.setCursor(text_area.x, top);
    M5.Display.setTextColor(TFT_DARKGREEN, 0);
    M5.Display.print("Med");

    M5.Display.setCursor(text_area.x, top + (height * 1 / 5));
    M5.Display.setTextColor(TFT_DARKGRAY, 0);
    M5.Display.print("Avg");

    M5.Display.setCursor(text_area.x, top + (height * 2 / 5));
    M5.Display.setTextColor(0xFF00FFu, 0);
    M5.Display.print("Diff");

    M5.Display.setCursor(text_area.x,  top + (height * 3 / 5));
    M5.Display.setTextColor(0xFFCF00u, 0);
    M5.Display.print("High");

    M5.Display.setCursor(text_area.x, top + (height * 4 / 5));
    M5.Display.setTextColor(0x00CFFFu, 0u);
    M5.Display.print("Low");
  }

  if (subpage)
  {
    // if (0)
    {
      int prev_y_line = 0;
      int step_index = 0;
      while ((div * step_table[step_index] >> 8) < 32) { ++step_index; }
      int step = step_table[step_index];

      canvas_hist.setTextDatum(textdatum_t::bottom_right);
      canvas_hist.setTextPadding(0);
      // M5.Display.setTextDatum(textdatum_t::bottom_right);
      // M5.Display.setTextPadding(0);
      int prev_temp = ((float)((((hist_count)<<17) / div) + min_temp) / 128 - 64);
      // draw histgram
      // int hist_y = M5.Display.height() - 1;
      int hist_y = histogram_area.y + histogram_area.h - 1;
      int hist_x = histogram_area.x;

      M5.Display.setClipRect(histogram_area.x, histogram_area.y, histogram_area.w, histogram_area.h);
      for (int i = hist_count-1; i >= -10; --i)
      {
        int temp = ((float)((((i+1)<<17) / div) + min_temp) / 128 - 64);
        bool line = prev_temp/step != temp/step;
        uint32_t color = color_table[0];
        bool linedraw = line;
        if (i >= 0)
        {
          linedraw = prev_line[i] != line;
          prev_line[i] = line;
          int px = prev_x[i];
          int x = histgram[i];
          histgram[i] = 0;
          color = color_table[i << 1];
          uint32_t bgcolor = line ? ((color >> 1) & 0x7F7F7Fu) : ((color>>2) & 0x3F3F3Fu);
          if (linedraw)
          {
            M5.Display.fillRect(hist_x + x, hist_y - i, histogram_area.w - x, 1, bgcolor);
          }
          canvas_hist.fillRect(x, hist_count - i - 1, histogram_area.w, 1, bgcolor);
          if (px != x)
          {
            prev_x[i] = x;
            M5.Display.fillRect(hist_x + x, hist_y - i, px - x, 1, x > px ? color : bgcolor);
            canvas_hist.fillRect(x, hist_count - i - 1, px - x, 1, x > px ? color : bgcolor);
          }
        }
        if (line)
        {
          int y = hist_count - i - 1;
          {
            uint32_t bgcolor = (color>>2) & 0x3F3F3Fu;
            canvas_hist.setTextColor(color);
            // M5.Display.fillRect(histogram_area.x + histogram_area.w - 18, prev_y_line, 18, (y - prev_y_line) - 8, bgcolor);
            canvas_hist.drawNumber(prev_temp, histogram_area.w, y);
          }
          prev_y_line = y + 1;
        }
        prev_temp = temp;
      }
      // M5.Display.setClipRect(histogram_area.x + histogram_area.w - (canvas_hist.fontWidth() * 3), histogram_area.y, (canvas_hist.fontWidth() * 3), histogram_area.h);
      canvas_hist.pushSprite(&M5.Display, histogram_area.x, histogram_area.y);
      M5.Display.clearClipRect();
    }
  }
  else
  {
    if ((frame_counter & 6) == 0)
    {
      M5.Display.setTextDatum(textdatum_t::bottom_right);
      M5.Display.setTextPadding(36);

      int_fast16_t right = text_area.x + text_area.w;
      int_fast16_t top = text_area.y + 10;
      int_fast16_t height = text_area.h - 10;
      M5.Display.setTextColor(TFT_DARKGREEN, 0);
      M5.Display.drawFloat(((float)med_sum) / 256 - 64 , 2, right, top + (height * 1 / 5));

      M5.Display.setTextColor(TFT_DARKGRAY, 0);
      M5.Display.drawFloat(((float)avg_sum) / 256 - 64 , 2, right, top + (height * 2 / 5));

      M5.Display.setTextColor(0xFF00FFu, 0);
      M5.Display.drawFloat((float)(tempdata->diff_info.temp) / 128 , 2, right, top + (height * 3 / 5));

      M5.Display.setTextColor(0xFFCF00u, 0);
      M5.Display.drawFloat(((float)max_sum) / 256 - 64 , 2, right, top + (height * 4 / 5));

      M5.Display.setTextColor(0x00CFFFu, 0u);
      M5.Display.drawFloat(((float)min_sum) / 256 - 64 , 2, right, top + height);
    }

    // if (0)
    {
      if (++wave_index >= wave_count) { wave_index = 0; }
    // センサ値を取得しグラフデータに蓄積する
      float min_tmp = (float)(min_sum) / 256 - 64;
      float max_tmp = (float)(max_sum) / 256 - 64;
      float avg_tmp = (float)(avg_sum) / 256 - 64;
      float med_tmp = (float)(med_sum) / 256 - 64;
      wave_datas[0][wave_index] = min_tmp;
      wave_datas[1][wave_index] = max_tmp;
      wave_datas[2][wave_index] = avg_tmp;
      wave_datas[3][wave_index] = med_tmp;

      {// 画面に表示される範囲のデータから最大値と最小値を取得する
        int index = wave_index;
        for (int i = 0; i < graph_area.w; ++i)
        {
          float t = wave_datas[0][index];
          if (min_tmp > t) { min_tmp = t; }
          t = wave_datas[1][index];
          if (max_tmp < t) { max_tmp = t; }
          if (--index < 0) { index = wave_count - 1; }
        }
      }

    /// 最小値・最大値に基づき，グラフに表示する範囲を更新する
      float diff = (max_value - min_value) + 1.0f;
      max_tmp += diff / 16.0f; // 表示範囲の上下の余白分を加算
      min_tmp -= diff / 16.0f;
    // 縦方向の表示範囲を最大値・最小値に緩やかに追従させる
      max_value = (max_value * 7 + max_tmp) / 8;
      min_value = (min_value * 7 + min_tmp) / 8;

      float magnify = (float)graph_area.h / (max_value - min_value);

    // 縦方向の表示範囲に応じて，目盛り線の間隔を算出する
      int step_index = 0;
      while (magnify * step_table[step_index] < 10) { ++step_index; }
      int step = step_table[step_index];

    // グラフの表示開始位置を1ピクセルずつ画面外に移動する．
    // マス目一つ分が画面外にはみ出したら表示位置をリセットする
      if (--drawarea_x_offset <= - block_width) { drawarea_x_offset = 0; }

    // グラフ右端の文字描画用の色と座標基準の設定を行う
      canvas_graph.setTextDatum(textdatum_t::bottom_right);
//    M5.Display.setTextDatum(textdatum_t::bottom_right);

      int graph_ymin[4][block_width]; // グラフ描画用のY座標保持配列
      int graph_ymax[4][block_width];

      int gy = 0;
      int grapharea_x = 0;//graph_area.x;

      for (int bx = drawarea_x_offset; bx < graph_area.w; bx += block_width)
      { // 横方向のループ(マス目を左から右にループ)
        int block_x = bx + grapharea_x;
        for (int gidx = 0; gidx < 4; ++gidx)
        {
          // マス目内に表示するグラフ線の縦座標を準備する
          int index = (bx + wave_index + wave_count - graph_area.w) % wave_count;
          int y0 = gy + (int)((max_value - wave_datas[gidx][index]) * magnify);
          for (int x = 0; x < block_width; ++x)
          {
            int y1 = y0;
            if (++index >= wave_count) { index -= wave_count; }
            y0 = gy + (int)((max_value - wave_datas[gidx][index]) * magnify);
            graph_ymin[gidx][x] = (y0 < y1) ? y0 : y1;
            graph_ymax[gidx][x] = (y0 > y1) ? y0 : y1;
          }
        }

        /// 縦方向ループで使用する，マス目のY座標の初期値を算出する
        int block_bottom = gy - 1;
        int value = ((int)(max_value / step) + (max_value >= 0.0f ? 1 : 0)) * step;

        do
        { // 縦方向のループ(マス目を上から下にループ)
    // 前回と今回のマス目のY座標の差からマス目の高さを算出する
          int block_top = block_bottom + 1;
          value -= step;
          block_bottom = gy + (max_value - value) * magnify;
          int h = block_bottom - block_top + 1;

          // 描画範囲をマス目内に限定する
          canvas_graph.setClipRect(0, block_top, graph_area.w, std::min(h, gy + graph_area.h - block_top));
          // M5.Display.setClipRect(grapharea_x, block_top, graph_area.w, std::min(h, gy + graph_area.h - block_top));

          int temp = (((value+64) * 128 - min_temp)*div)>>16;
          if (temp < 0) temp = 0;
          else if (temp > 255) temp = 255;
          uint32_t color = (color_table[temp]>>1) & 0x7F7F7Fu;

          // マス目の下側と左側の線を描画
          canvas_graph.writeFastHLine(block_x, block_bottom, block_width, color);
          temp = (((value+64) * 128 + step*64 - min_temp)*div)>>16;
          if (temp < 0) temp = 0;
          else if (temp > 255) temp = 255;
          color = color_table[temp];
          canvas_graph.setTextColor(color);
          canvas_graph.writeFastVLine(block_x, block_top, h - 1, (color>>1) & 0x7F7F7Fu);
          // マス目の背景を塗り潰し
          canvas_graph.writeFillRect(block_x + 1, block_top, block_width - 1, h - 1, (color>>2)&0x3F3F3Fu);

          if (graph_area.w - block_x < 30 + block_width)
          { // グラフ右端の数字を描画
            canvas_graph.drawNumber(value, graph_area.w, block_bottom);
          }

          for (int gidx = 0; gidx < 4; ++gidx)
          {
            static constexpr uint32_t color_tbl[] = { 0x00FFFFu, 0xFFFF00u, 0xAAAAAAu, 0x00CF00u };
            // グラフ線の色を設定
            canvas_graph.setColor(color_tbl[gidx]);

            for (int x = (block_x < 0) ? - block_x : 0; x < block_width; ++x)
            { // マス目内のグラフを描画する
              int y_min = graph_ymin[gidx][x];
              if (y_min > block_bottom) { continue; }
              int y_max = graph_ymax[gidx][x];
              if (y_max < block_top) { continue; }

              canvas_graph.writeFastVLine(block_x + x, y_min, y_max - y_min + 1);
            }
          }
          canvas_graph.clearClipRect();
        } while (block_bottom < (gy + graph_area.h));
      }
      canvas_graph.pushSprite(&M5.Display, graph_area.x, graph_area.y);
    }
//*/
  }

}

/*/
void drawToLCD(const temp_data_t* tempdata)
{
  static uint32_t min_temp = 0;
  static uint32_t max_temp = 128 << 6;
  min_temp = ( min_temp * 7 + tempdata->min_info.temp ) >> 3;
  max_temp = ( max_temp * 7 + tempdata->max_info.temp ) >> 3;
// min_temp = tempdata.min_info.temp;
// max_temp = tempdata.max_info.temp;
  int temp_diff = max_temp - min_temp;
  if (temp_diff == 0) return;
  int div = 65536 / temp_diff;
  bool subpage = tempdata->subpage;

  for (int y = 0; y < 24; ++y)
  {
    for (int x = 0; x < 16; ++x)
    {
      int xpos = (x<<1) + ((subpage ^ y)&1);
      int temp = tempdata->data[x + y * 16];
      int color = 0;
      if (tempdata->min_info.temp > temp)
      {
        color = TFT_BLUE;
      }
      else if (tempdata->max_info.temp < temp)
      {
        color = TFT_YELLOW;
      }
      else
      {
        temp = (temp - min_temp) * div >> 11;
        if (temp < 0) color = 0;
        else if (temp > 31) color = 0xFFFF;
        else color = (temp << 11) + (temp << 6) + temp;
      }
      display.fillRect(xpos*10, y*10, 10, 10, color);
    }
  }
}
//*/

void drawTask(void*)
{
  int idx_prev = -1;
  for (;;)
  {
    drawRegister();
    drawCheck();
    if (idx_prev != idx_draw)
    {
      idx_prev = idx_draw;
      drawToLCD(&tempdatas[idx_prev]);
    }
    else
    {
      vTaskDelay(1);
    }
  }
}

void recvTask(void*)
{
  static constexpr uint32_t threshold_low = (0.0f + 64) * 128;
  static constexpr uint32_t threshold_high = (100.0f + 64) * 128;

  enum test_step_t
  {
    test_button_click,
    test_refresh_rate_64Hz,
    test_refresh_rate_32Hz,
    test_refresh_rate_16Hz,
    test_buzzer_led,
    test_register_write,
    test_max,
  };

  test_step_t test_step = test_button_click;
  uint32_t step_counter = 0;

  static constexpr uint8_t cmd[] =
  {
    0x32, (uint8_t)~0x32, // I2C_Addr
    0x04,        // function control
    0x07,        // refresh rate
    0x04,        // Filter level
    0,
    0,
    0,

    0xFF,                     // monitor size
    0b00000000,               // Temp Alert enabled
    (uint8_t)4800, 4800 >> 8, // buzzer freq
    128,                      // buzzer duty
    0x00, 0x04, 0x00,         // color
    0,0,0,0,0,0,0,0,

    (uint8_t)threshold_low, (uint8_t)(threshold_low >> 8),
    (uint8_t)4800, 4800 >> 8,
    5,
    0x00, 0x00, 0x08,
    0,0,0,0,0,0,0,0,

    (uint8_t)threshold_high, (uint8_t)(threshold_high >> 8),
    (uint8_t)4800, 4800 >> 8,
    5,
    0x08, 0x00, 0x00,
    0,0,0,0,0,0,0,0,
  };

  static constexpr uint32_t default_threshold_low = (0.0f + 64) * 128;
  static constexpr uint32_t default_threshold_high = (100.0f + 64) * 128;
  static constexpr uint32_t default_buzzer_freq = 4800;
  static constexpr uint8_t reg_default[] =
  {
    0, 0, 0, 0,
    0x90, 0x64, 0x00, 0x07,
    0x32, (uint8_t)~0x32, // I2C_Addr
    0x04,        // function control
    0x05,        // refresh rate
    0x04,        // Filter level
    0,
    0,
    0,

    0xFF,                     // monitor size
    0b00000000,               // Temp Alert enabled
    (uint8_t)default_buzzer_freq, default_buzzer_freq >> 8, // buzzer freq
    128,                      // buzzer duty
    0x00, 0x04, 0x00,         // color
    0,0,0,0,0,0,0,0,

    (uint8_t)default_threshold_low, (uint8_t)(default_threshold_low >> 8),
    (uint8_t)default_buzzer_freq, default_buzzer_freq >> 8,
    5,                        // interval
    0x00, 0x00, 0x08,         // color
    0,0,0,0,0,0,0,0,

    (uint8_t)default_threshold_high, (uint8_t)(default_threshold_high >> 8),
    (uint8_t)default_buzzer_freq, default_buzzer_freq >> 8,
    5,                        // interval
    0x08, 0x00, 0x00,         // color
    0,0,0,0,0,0,0,0,
  };

  static constexpr uint8_t THERMAL2_REG_FUNCTION_CTRL = 0x0A;
  static constexpr uint8_t THERMAL2_REG_REFRESH_RATE = 0x0B;
  static constexpr uint8_t THERMAL2_REG_NOISE_FILTER = 0x0C;
  static constexpr uint8_t THERMAL2_REG_REFRESH_CTRL = 0x6E;

  uint32_t freq_i2c_default = 400000;
  uint32_t freq_i2c_read = 921600;

  size_t i2c_error_count = 0;
  for (;;)
  {
    M5.update();

    if (!M5.Ex_I2C.readRegister(i2c_addr, 0x00, register_data, sizeof(register_data), 400000) || M5.BtnA.wasClicked())
    {
      memset(check_list, check_state_t::unknown, sizeof(check_list));
      check_list[check_list_t::button_click] = check_state_t::progress;
      test_step = test_button_click;
      step_counter = 0;
      M5.Ex_I2C.writeRegister(i2c_addr, 0, reg_default, sizeof(reg_default), freq_i2c_default);
    } else
    if (register_data[0x00]) {
      M5.Ex_I2C.writeRegister8(i2c_addr, 0, register_data[0x00], freq_i2c_default);
    }

// if (register_data[0x06]) {
//   delay(2500); // Unit register bug occured.
// }

    switch (test_step)
    {
    case test_button_click:
      if (register_data[0x00]) {
        auto rd = register_data[0x00];
        if (rd & 8) {
          check_list[check_list_t::button_click] = check_state_t::success;

          check_list[check_list_t::refresh_64Hz] = check_state_t::progress;
          test_step = test_refresh_rate_64Hz;
          step_counter = 0;

          M5.Ex_I2C.writeRegister(i2c_addr, 0x08, cmd, sizeof(cmd), 400000);
          freq_i2c_read = 921600;
        }
      }
      break;

    case test_refresh_rate_64Hz:
      if (++step_counter > 8192)
      {
        check_list[check_list_t::refresh_64Hz] = check_state_t::failed;
        test_step = test_button_click;
        break;
      }
      else if (fps >= 55 && fps <= 64) {
        check_list[check_list_t::refresh_64Hz] = check_state_t::success;

        check_list[check_list_t::refresh_32Hz] = check_state_t::progress;
        test_step = test_refresh_rate_32Hz;
        M5.Ex_I2C.writeRegister8(i2c_addr, THERMAL2_REG_REFRESH_RATE, 6, freq_i2c_default);
        step_counter = 0;
        freq_i2c_read = 400000;
      }
      break;

    case test_refresh_rate_32Hz:
      if (++step_counter > 8192)
      {
        check_list[check_list_t::refresh_32Hz] = check_state_t::failed;
        test_step = test_button_click;
        break;
      }
      else if (fps >= 31 && fps <= 32) {
        check_list[check_list_t::refresh_32Hz] = check_state_t::success;

        check_list[check_list_t::refresh_16Hz] = check_state_t::progress;
        test_step = test_refresh_rate_16Hz;
        M5.Ex_I2C.writeRegister8(i2c_addr, THERMAL2_REG_REFRESH_RATE, 5, freq_i2c_default);
        step_counter = 0;
      }
      break;

    case test_refresh_rate_16Hz:
      if (++step_counter > 8192)
      {
        check_list[check_list_t::refresh_16Hz] = check_state_t::failed;
        test_step = test_button_click;
        break;
      }
      else if (fps >= 15 && fps <= 16) {
        check_list[check_list_t::refresh_16Hz] = check_state_t::success;

        check_list[check_list_t::buzzer_led] = check_state_t::progress;
        test_step = test_buzzer_led;
        step_counter = 0;
        M5.Ex_I2C.writeRegister8(i2c_addr, THERMAL2_REG_FUNCTION_CTRL, 7, freq_i2c_default);
      }
      break;

    case test_buzzer_led:
      if ((step_counter++ & 63) == 0)
      {
        uint8_t reg_0x12[6] = { 0 };
        auto s = step_counter >> 6;
        uint_fast16_t freq = 4000 + (s * 200);
        reg_0x12[0] = (uint8_t)freq;
        reg_0x12[1] = (uint8_t)(freq >> 8);
        reg_0x12[2] = 128;
        if (s == 10) {
          step_counter = 0;
          check_list[check_list_t::buzzer_led] = check_state_t::success;

          check_list[check_list_t::register_write] = check_state_t::progress;
          test_step = test_register_write;
        } else {
          switch (s % 5) {
          default:
          case 0:
            reg_0x12[3] = 7;
            break;

          case 1:
            reg_0x12[4] = 7;
            break;

          case 2:
            reg_0x12[5] = 7;
            break;

          case 3:
            reg_0x12[3] = 4;
            reg_0x12[4] = 4;
            reg_0x12[5] = 4;
            break;

          case 4:
            break;
          }
        }
        M5.Ex_I2C.writeRegister(i2c_addr, 0x12, reg_0x12, sizeof(reg_0x12), freq_i2c_default);
      }
      break;

    case test_register_write:
      if (step_counter++) {
        if (memcmp(register_data, reg_default, sizeof(reg_default))) {
          check_list[check_list_t::register_write] = check_state_t::failed;
          test_step = test_button_click;
          break;
        } else if (step_counter > 10) {
          check_list[check_list_t::register_write] = check_state_t::success;
          test_step = test_button_click;
          break;
        }
      }
      M5.Ex_I2C.writeRegister(i2c_addr, 0, reg_default, sizeof(reg_default), freq_i2c_default);
      break;

    default:
      break;
    }

    uint8_t send_data[2] = { THERMAL2_REG_REFRESH_CTRL, 2 };
    if (M5.Ex_I2C.start(i2c_addr, false, freq_i2c_default)
     && M5.Ex_I2C.write(send_data, 1)
     && M5.Ex_I2C.restart(i2c_addr, true, freq_i2c_read)
     && M5.Ex_I2C.read((uint8_t*)&tempdatas[idx_recv], 1)
     && tempdatas[idx_recv].refresh_control & 1) // has_new data none
    {
      if (M5.Ex_I2C.read(&((uint8_t*)&tempdatas[idx_recv])[1], sizeof(temp_data_t)-1))
      {
        idx_draw = idx_recv;
        idx_recv = !idx_recv;
      }
    }
    else
    {
      vTaskDelay(1);
    }
    M5.Ex_I2C.stop();
  }
}

void setup(void)
{
  M5.begin();
  M5.Power.setExtPower(true);
  M5.Ex_I2C.begin();

  M5.Display.setColorDepth(16);
  M5.Display.startWrite();
  M5.Display.print("Unit Thermal2 demo.");

  int_fast16_t width = M5.Display.width();
  int_fast16_t height = M5.Display.height();

  camera_area.set(0, 0, width - 128, 128);
  histogram_area.set(width - 128, 0, 64, 128);
  text_area.set(width - 64, 0, 64, 55);
  graph_area.set(width - 64, 55, 64, 128 - 55);

  canvas_graph.createSprite(graph_area.w, graph_area.h);
  canvas_hist.createSprite(histogram_area.w, histogram_area.h);

  delay(100);

  for (int i = 0; i < hist_count; ++i)
  {
    prev_line[i] = true;
    prev_x[i] = 0;//M5.Display.width();
  }
  for (int i = 0; i < wave_count; ++i)
  {
    wave_datas[0][i] = 32;
    wave_datas[1][i] = 32;
    wave_datas[2][i] = 32;
    wave_datas[3][i] = 32;
  }

  xTaskCreatePinnedToCore(recvTask, "recvTask", 8192, NULL, 5, NULL, PRO_CPU_NUM);

  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  for (int x = 0; x < 16; ++x) {
    M5.Display.setCursor((x+1) * 13, 128);
    M5.Display.printf("_%x", x);
  }
  for (int y = 0; y < 8; ++y) {
    M5.Display.setCursor(0, 128+(y+1) * 10);
    M5.Display.printf("%x_", y);
  }
  memset(check_list, 0xFF, sizeof(check_prev));
}

void loop(void)
{
  drawTask(nullptr);
}
