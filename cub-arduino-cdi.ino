#include <EEPROM.h>

#define OUTPIN 5 // PD5
#define INPIN 2  // PD2
#define ENPIN 3  // PD3
#define RPMINTVL 500 // LCDの回転数更新間隔(mS)
#define MAXDEG 50  // 最大進角
#define MINDEG 10  // 最小進角
#define TIMINTVL 25 // Timing割込み最小間隔(4uS単位：25だと100uS)

// α-βフィルタの追従性チューニング用定数
// シフト量が小さいほど追従性が高く(過敏)、大きいほど平滑性が高い(鈍感)
#define ALPHA_SHIFT 1  // αゲインのビットシフト量 (1=1/2, 2=1/4)
#define BETA_SHIFT  2  // βゲインのビットシフト量 (3=1/8, 4=1/16)

// マップサイズ5要素 (1000, 3000, 5000, 7000, 9000rpm)
#define MAPSIZE 5   
#define MAPSELECT  0  
#define MAPAOFFSET 8  
#define MAPBOFFSET 16 
#define LCDRPM 0
#define LCDINI 1
#define LCDMOD 2
#define VERSION 70  // バージョン番号 (70=Ver7.0 フィルタ定数化・リファクタリング)

volatile bool pon = false;
volatile bool ovf = false; 
volatile bool Bon=false;   
volatile bool Aen=false;   
volatile bool Ben=false;   
volatile unsigned widthA=0;
volatile unsigned widthB=0;
volatile unsigned int intervaltime=0;

// デフォルト配列 (5要素)
uint8_t tmap[]={ 0, 0, 0, 0, 0}; 
uint8_t dmapA[]={17,22,30,32,32}; 
uint8_t dmapB[]={17,27,27,27,27}; 

uint16_t rpm=0;
unsigned int averagetime=0;
int mapselect=0;  
int mapselcd=0;   
bool edit=false;
int  select=0;
bool modify=false;
uint8_t *mappt=dmapA;
int offset=MAPAOFFSET;

// 高解像度 α-βフィルタ用の状態変数
int32_t ab_interval_fp = 15000 << 8;
int32_t ab_velocity_fp = 0;

// 4ストローク脈動キャンセル用
int32_t history[2] = {15000, 15000};

ISR(INT0_vect, ISR_NAKED) 
{
  asm volatile  (
    "cli \n"              
    "push r16 \n"         
    "push r17 \n"
    "push r18 \n"
    "in  r17, (__SREG__) \n" 

    "cbi  0x1d, 0 \n"     
    
    "ldi  r16, 0x00 \n"   
    "sts  0x0081, r16 \n"

    "lds r16, 0x0084 \n"
    "sts (intervaltime), r16 \n"
    "lds r16, 0x0085 \n"
    "sts (intervaltime+1), r16 \n"

    "lds  r16, (Aen) \n"
    "lds  r18, (Bon) \n"
    "or   r16, r18 \n"
    "ldi  r18, 0x01 \n"     
    "tst  r16 \n"
    "brbs 1,ASKIP \n"
    "lds  r16, (widthA+1)\n"
    "sts  0x0089, r16 \n"   
    "lds  r16, (widthA)\n"
    "sts  0x0088, r16 \n"   
    "sbr  r18, 0x02 \n"     
    "ASKIP:\n"

    "lds  r16, (Ben) \n"
    "tst  r16 \n"
    "brbs 1,BSKIP \n"
    "lds  r16, (widthB+1)\n"
    "sts  0x008b, r16 \n"   
    "lds  r16, (widthB)\n"
    "sts  0x008a, r16 \n"   
    "sbr  r18, 0x04 \n"     
    "ldi  r16, 0x01 \n"     
    "sts  (Bon), r16 \n"
    "BSKIP:\n"
    
    "sts  0x006f, r18 \n"   
    "ldi  r16,0x27 \n"      
    "out  0x16, r16 \n"     
    
    "ldi   r16, 0x01 \n"  
    "sts  (pon), r16 \n"

    "ldi   r16, 0x00 \n"  
    "sts  0x0085, r16 \n" 
    "sts  0x0084, r16 \n" 

    "ldi   r16, 0x03 \n"  
    "sts  0x0081, r16 \n"  

    "out (__SREG__), r17  \n" 
    "pop r18 \n"
    "pop r17 \n"
    "pop r16 \n"          
    "reti"                
    :::"r16","r17","r18"
    );
}

ISR(TIMER1_COMPA_vect, ISR_NAKED) 
{
  asm volatile  (
    "cli \n" "sbi 0x0b, 5 \n" 
    "push r16 \n" "push r17 \n" 
    "in  r17, (__SREG__) \n" 
    "ldi r16,32 \n" "LP1: \n" 
    "dec r16 \n" "brne LP1\n" 
    "out (__SREG__), r17  \n" 
    "pop r17 \n" "pop r16 \n" 
    "cbi 0x0b, 5 \n" 
    "reti" :::"r16","r17"
  );
}

ISR(TIMER1_COMPB_vect, ISR_NAKED) 
{
  asm volatile  (
    "cli \n" 
    "sbi 0x0b, 5 \n" 
    "push r16 \n" 
    "push r17 \n" 
    "in  r17, (__SREG__) \n" 
    "ldi   r16, 0x00 \n" 
    "sts  (Bon), r16 \n" 
    "ldi r16,32 \n" 
    "LP2: \n" 
    "dec r16 \n" 
    "brne LP2\n" 
    "out (__SREG__), r17  \n" 
    "pop r17 \n" "pop r16 \n" 
    "cbi 0x0b, 5 \n" 
    "reti" :::"r16","r17"
  );
}

ISR(TIMER1_OVF_vect, ISR_NAKED) 
{
  asm volatile  (
    "cli \n" 
    "push r16 \n" 
    "ldi  r16, 0x00 \n" 
    "sts  0x0081, r16 \n" 
    "ldi r16, 0x98 \n" 
    "sts (intervaltime), r16 \n" 
    "ldi r16, 0x3a \n" 
    "sts (intervaltime+1), r16 \n" 
    "ldi   r16, 0x01 \n" 
    "sts  (ovf), r16 \n" 
    "pop r16 \n" "reti" :::"r16"
  );
}

void defmap() {
  int i, m, offset;
  bool flag = false;
  mapselcd = EEPROM.read(MAPSELECT);
  for (i = 0; i < MAPSIZE; i++)
  {
    m = EEPROM.read(i + MAPAOFFSET);
    if (m < 1 || 100 < m)
      flag = true;
  }
  if (flag)
  {
    for (i = 0; i < MAPSIZE; i++)
    {
      EEPROM.write(i + MAPAOFFSET, dmapA[i]);
    }
  }
  else
  {
    for (i = 0; i < MAPSIZE; i++)
    {
      dmapA[i] = EEPROM.read(i + MAPAOFFSET);
    }
  }
  flag = false;
  for (i = 0; i < MAPSIZE; i++)
  {
    m = EEPROM.read(i + MAPBOFFSET);
    if (m < 1 || 100 < m)
      flag = true;
  }
  if (flag)
  {
    for (i = 0; i < MAPSIZE; i++)
    {
      EEPROM.write(i + MAPBOFFSET, dmapB[i]);
    }
  }
  else
  {
    for (i = 0; i < MAPSIZE; i++)
    {
      dmapB[i] = EEPROM.read(i + MAPBOFFSET);
    }
  }
  mapselect = EEPROM.read(MAPSELECT);
  switch (mapselect)
  {
  case 0:
    offset = MAPAOFFSET;
    break;
  case 1:
    offset = MAPBOFFSET;
    break;
  default:
    offset = MAPAOFFSET;
    mapselect = 0;
    EEPROM.write(MAPSELECT, mapselect);
  }
  for (i = 0; i < MAPSIZE; i++)
  {
    tmap[i] = EEPROM.read(i + offset);
  }
}

void lcddisp(int mode, bool ed, int sel, bool mod)
{
  char buf[12];
  char sbuf[48];
  static int mpos[] = {0, 4, 7, 10, 13, 16};
  static char selserial[2][2][2] = {{{'a', 'B'}, {'A', 'b'}}, {{'B', 'a'}, {'b', 'A'}}};

  switch (mode)
  {
  case LCDRPM:
    sprintf(sbuf, "R:%3d\n", rpm / 100);
    Serial.print(sbuf);
    break;
  case LCDINI:
    sprintf(sbuf, "O: %c%3d%3d%3d%3d%3d %c%3d%3d%3d%3d%3d\n",
            selserial[mapselect][mapselcd][0],
            (mapselect == 0) ? dmapA[0] : dmapB[0], (mapselect == 0) ? dmapA[1] : dmapB[1], (mapselect == 0) ? dmapA[2] : dmapB[2], (mapselect == 0) ? dmapA[3] : dmapB[3], (mapselect == 0) ? dmapA[4] : dmapB[4],
            selserial[mapselect][mapselcd][1],
            (mapselect == 0) ? dmapB[0] : dmapA[0], (mapselect == 0) ? dmapB[1] : dmapA[1], (mapselect == 0) ? dmapB[2] : dmapA[2], (mapselect == 0) ? dmapB[3] : dmapA[3], (mapselect == 0) ? dmapB[4] : dmapA[4]);
    Serial.print(sbuf);
    break;
  case LCDMOD:
    sprintf(sbuf, "E: WR %3d%3d%3d%3d%3d\n", tmap[0], tmap[1], tmap[2], tmap[3], tmap[4]);
    buf[0] = (mod) ? 0xff : '*';
    sbuf[2 + mpos[sel]] = buf[0];
    Serial.print(sbuf);
    break;
  default:
    break;
  }
}

void writeeeprom(int map)
{
  int i, offset;
  uint8_t *mappt;
  if (mapselect == 1)
  {
    offset = MAPBOFFSET;
    mappt = dmapB;
  }
  else
  {
    offset = MAPAOFFSET;
    mappt = dmapA;
  }
  for (i = 0; i < MAPSIZE; i++)
  {
    if (mappt[i] != tmap[i])
    {
      mappt[i] = tmap[i];
      EEPROM.write(i + offset, mappt[i]);
    }
  }
  if (mapselect != EEPROM.read(MAPSELECT))
    EEPROM.write(MAPSELECT, (char)map);
}

void switchmap(int map)
{
  int i, offset;
  if (mapselect != map)
  {
    mapselect = map;
    switch (mapselect)
    {
    case 0:
      offset = MAPAOFFSET;
      break;
    case 1:
      offset = MAPBOFFSET;
      break;
    default:
      break;
    }
    for (i = 0; i < MAPSIZE; i++)
    {
      tmap[i] = EEPROM.read(i + offset);
    }
  }
}

void keyenter()
{
  if (edit)
  {
    if (0 == select)
    {
      writeeeprom(mapselect);
      edit = false;
    }
    else
    {
      modify = !modify;
    }
  }
  else
  {
    if (mapselect == mapselcd)
    {
      edit = true;
      select = 1;
    }
    else
    {
      switchmap(mapselcd);
    }
  }
}
void keycancel()
{
  if (edit)
  {
    if (modify)
    {
      modify = false;
    }
    else
    {
      if (1 == mapselcd)
      {
        mappt = dmapB;
        offset = MAPBOFFSET;
      }
      else
      {
        mappt = dmapA;
        offset = MAPAOFFSET;
      }
      for (int i = 0; i < MAPSIZE; i++)
      {
        if (mappt[i] != tmap[i])
        {
          tmap[i] = mappt[i];
        }
      }
      edit = false;
    }
  }
}
void keyup()
{
  if (edit)
  {
    if (modify && select != 0)
    {
      if (select < MAPSIZE)
      {
        if (tmap[select - 1] < tmap[select])
          tmap[select - 1]++;
      }
      else
      {
        if (tmap[select - 1] < MAXDEG)
          tmap[select - 1]++;
      }
    }
    else
    {
      if (MAPSIZE < ++select)
        select = 0;
    }
  }
  else
  {
    if (++mapselcd == 2)
      mapselcd = 0;
  }
}
void keydown()
{
  if (edit)
  {
    if (modify && select != 0)
    {
      if (1 < select)
      {
        if (tmap[select - 2] < tmap[select - 1])
          tmap[select - 1]--;
      }
      else
      {
        if (MINDEG < tmap[select - 1])
          tmap[select - 1]--;
      }
    }
    else
    {
      if (--select < 0)
        select = MAPSIZE;
    }
  }
  else
  {
    if (--mapselcd == -1)
      mapselcd = 1;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.print("CDI-AC6 Ver");
  Serial.print(VERSION / 10);
  Serial.print(".");
  Serial.println(VERSION % 10);
  pinMode(OUTPIN, OUTPUT);
  digitalWrite(OUTPIN, LOW);
  pinMode(INPIN, INPUT_PULLUP);
  pinMode(ENPIN, INPUT_PULLUP);
  defmap();
  lcddisp(LCDINI, false, mapselcd, false);
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  TCCR1C = 0x00;
  OCR1A = 0x0000;
  OCR1B = 0x0000;
  TCNT1 = 0x0000;
  TIMSK1 = 0x07;

  unsigned char rtmp = EICRA;
  rtmp = (rtmp & 0xfc) | 0x02;
  EICRA = rtmp;
  rtmp = EIMSK;
  rtmp = (rtmp & 0xfe) | 0x01;
  EIMSK = rtmp;
}

void loop()
{
  int i;
  int16_t fdelay;
  static unsigned long nowtime = 0;
  static unsigned long prevtime = 0;
  bool enable = false;
  enable = (digitalRead(ENPIN) == HIGH) ? true : false;

  if (enable)
  {
    if (pon)
    {
      int32_t measurement = (int32_t)intervaltime;
      int32_t current_ab_interval = ab_interval_fp >> 8;
      bool is_rescued = false; // ダブルファイア防止フラグ

      // 1. ノイズ足切り（デッドタイム）
      if (measurement < (current_ab_interval >> 2))
      {
        pon = false;
        return;
      }

      // 2. 4ストローク脈動のキャンセルと真の減速(delta)抽出
      int32_t delta = measurement - history[0];
      history[0] = history[1];
      history[1] = measurement;

      // レスキュー点火（加速による異常スキップ防止）
      if (measurement < (current_ab_interval - (current_ab_interval >> 3)))
      {
        bitSet(PORTD, 5);
        delayMicroseconds(30);
        bitClear(PORTD, 5);
        is_rescued = true; // レスキュー点火済みをマーク
      }

      // 3. 高解像度 α-β フィルタ (256倍精度・固定小数点)
      int32_t measurement_fp = measurement << 8;
      int32_t error_fp = measurement_fp - ab_interval_fp;
      int32_t abs_error_fp = (error_fp < 0) ? -error_fp : error_fp;

      if (abs_error_fp > (ab_interval_fp >> 1))
      {
        ab_interval_fp = measurement_fp;
        ab_velocity_fp = 0;
      }
      else
      {
        int32_t predicted_measurement_fp = ab_interval_fp + ab_velocity_fp;
        int32_t residual_fp = measurement_fp - predicted_measurement_fp;

        // 定数化されたシフト量による追従性チューニング
        ab_interval_fp = predicted_measurement_fp + (residual_fp >> ALPHA_SHIFT);
        ab_velocity_fp = ab_velocity_fp + (residual_fp >> BETA_SHIFT);
      }

      // 4. 変曲点ジャンプキャンセラー
      if (delta > 0 && ab_velocity_fp < 0)
      {
        ab_interval_fp = measurement << 8;
        ab_velocity_fp = 0;
      }

      // 5. 2手先予測による位相遅れの完全相殺
      int32_t predicted_interval = (ab_interval_fp + 2 * ab_velocity_fp) >> 8;

      if (predicted_interval < 1)
        predicted_interval = 1;
      averagetime = (unsigned int)predicted_interval;

      // 16ビット高速除算によるRPM計算
      rpm = (uint16_t)(15000000UL / (unsigned long)averagetime);

      // サブディグリー（1/16度）高精度直線補間 5ポイント
      int32_t fdeg_fp;

      if (rpm < 1000)
      {
        averagetime = 15000;
        fdeg_fp = (int32_t)tmap[0] << 4;
      }
      else if (rpm < 3000)
      {
        fdeg_fp = (int32_t)((((long)(tmap[1] - tmap[0]) * (long)(rpm - 1000)) * 33) >> 12) + ((int32_t)tmap[0] << 4);
      }
      else if (rpm < 5000)
      {
        fdeg_fp = (int32_t)((((long)(tmap[2] - tmap[1]) * (long)(rpm - 3000)) * 33) >> 12) + ((int32_t)tmap[1] << 4);
      }
      else if (rpm < 7000)
      {
        fdeg_fp = (int32_t)((((long)(tmap[3] - tmap[2]) * (long)(rpm - 5000)) * 33) >> 12) + ((int32_t)tmap[2] << 4);
      }
      else if (rpm < 9000)
      {
        fdeg_fp = (int32_t)((((long)(tmap[4] - tmap[3]) * (long)(rpm - 7000)) * 33) >> 12) + ((int32_t)tmap[3] << 4);
      }
      else
      {
        fdeg_fp = (int32_t)tmap[4] << 4;
      }

      // 遅角量を1/16度単位で計算し、ディレイ値へ変換 (>> 20 で吸収)
      int32_t sdeg_fp = (27L << 4) - fdeg_fp;
      fdelay = (int16_t)(((long)sdeg_fp * (long)averagetime * 182L) >> 20);

      // 6. モード切り替え同期・セーフティ制御
      if (is_rescued)
      {
        // レスキュー発火済みの場合は、タイマーによる二重発火を完全にキルし、タコメーター倍表示を防ぐ
        widthA = 0;
        Aen = false;
        Ben = false;
        widthB = 0;
      }
      else if (fdelay == -1 || fdelay == 0)
      {
        widthA = 1;
        Aen = true;
        Bon = false;
        Ben = false;
      }
      else if (fdelay < -1)
      {
        // 急減速時の安全弁
        if ((measurement - current_ab_interval) > (measurement >> 3))
        {
          widthA = 1;
          Aen = true;
          Ben = false;
          widthB = 0;
        }
        else if (Ben == false)
        {
          TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
          widthA = 1;
          Aen = true;
          widthB = fdelay + averagetime;
          Ben = true;
        }
        else
        {
          widthA = 1;
          Aen = false;
          widthB = fdelay + averagetime;
          Ben = true;
        }
      }
      else
      {
        if (Ben == true)
        {
          TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
        }
        widthA = fdelay;
        Aen = true;
        Ben = false;
        widthB = 0;
      }
      pon = false;
    }
  }
  else
  {
    Aen = false;
    Ben = false;
  }

  if (ovf)
  {
    ovf = false;
  }

  // シリアルコマンド受信・マップ編集
  bool skf = false;
  if (Serial.available())
  {
    switch (char rd = Serial.read())
    {
    case 'E':
      keyenter();
      skf = true;
      break;
    case 'C':
      keycancel();
      skf = true;
      break;
    case 'U':
      keyup();
      skf = true;
      break;
    case 'D':
      keydown();
      skf = true;
      break;
    }
    if (skf)
    {
      if (edit)
        lcddisp(LCDMOD, edit, select, modify);
      else
        lcddisp(LCDINI, edit, select, modify);
    }
  }

  nowtime = millis();
  // LCD(シリアル)の回転数更新
  if (RPMINTVL < (nowtime - prevtime))
  {
    lcddisp(LCDRPM, edit, select, modify);
    prevtime = nowtime;
  }

  if ((bitRead(EIMSK, 0) == 0) && (TIMINTVL < TCNT1))
  {
    bitSet(EIFR, 0);
    bitSet(EIMSK, 0);
  }
}
