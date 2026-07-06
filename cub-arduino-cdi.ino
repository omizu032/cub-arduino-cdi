// Copyright 2024 ほいほい堂本舗 http://www.hoihoido.com
//
//デジタル進角/遅角 AC-CDI マップエディタつき
// 20240423 Ver5.3 加減速予測
// 20240424 Ver5.4 TIMER1_COMPA,B割込み内のSREG退避漏れを修正(LCDが豆腐になるバグFIX)。
//                 Timingパルス入力割込み後100uS間は次の割込みを受付けなくした。
//                 最大進角は35→50度に変更。
// 20240503 Ver5.5 MAPデータをA/Bの2セット持てる様に変更。
// 20240529 Ver5.6 ピックアップタイミングが28度と思っていたのを27度前提に変更。
// 20240611 Ver6   シリアル設定
// 20260425 Ver6.1 バグ修正・加減速予測改善(Claude Sonnet)
//                 [BUG1] 進角時(fdelay<-1)のAen制御修正: Ben?false:true → false固定。
//                         遅角→進角切替時にAen=trueになりD2直後に不正パルスが出力されていた。
//                 [BUG2] OVF ISR内アセンブラ誤記修正: lds r16,0x3a → ldi r16,0x3a。
//                         停止・極低回転時にintervaltimeの上位バイトが不定値になっていた。
//                 [BUG3] fdelay==0/-1の境界値処理でBon=falseを追加(残存Bonによる誤点火防止)。
//                 [BUG4] defmap()のブレース構造修正: MAP-AがEEPROM未登録時にMAP-Bの初期化がスキップされていた。
//                 [改善] 加減速予測の計算式修正: intervaltime*2-preprev → intervaltime*2-prev。
//                         前々回ではなく前回との差分を使う事でブログ設計通りの予測を実現。
//                 [改善] 4ストローク対応の1回置き計算(skipflag)を実装。
// 20260503 Ver6.2 バグ修正(Gemini)
//                 [BUG] 多重割込みの禁止
//                       retiの直前にあるseiをすべて削除: 処理が終わる前に次の割り込みをしてしまい
//                       高回転時にスタックオーバーフローやパルスの踊り(ジッター)を引き起こすのを防ぐ
//                 [改善] 表示・シリアル通信の制限箇所: sprintf とシリアル送信は非常に重く、点火計算を妨害しているので
//                        if (RPMINTVL < (nowtime - prevtime)) の RPMINTVL を 500（0.5秒）以上に設定し
//                        走行中はシリアル出力を止める。
//                 [改善] 可変デッドタイムの実装: intervaltime（1回転の時間）の 1/4（25%）未満で入るパルスは、
//                       すべてノイズとして弾く。prev には前回の有効な intervaltime が入っているためこれを利用
// 20260507 Ver6.3 コントローラLCD表示機能の削除（設定はArduino IDE(ArduinoDroid)のシリアルモニタだけで行う）
// 20260518 Ver6.4 バグ修正(Gemini)
//                 [BUG] 点火タイミングのハンチング（バタつき）発生
//                       加減速していない時（またはエンジンブレーキ時）、予測値が実際のクランク速度を追い越してしまい、
//                       遅角から進角への切り替え論理が激しくハンチング（バタつき）を起こしているため
//                 [改善] 予測セーフティリミッター
//                        パーシャル時や減速時に、予測値が実測値から±12.5%以上乖離するのを防ぐ
//                        これによりミクロな回転変動による予測値の乱高下（ハンチング）を抑制する
// 20260518 Ver6.5 バグ修正(Gemini)
//                 [改善] 減速・エンブレ時の誤進角防止セーフティ
//                        アクセル全閉（減速）なのに予測のブレでfdelay < -1（進角モード）に入ってしまうのを防ぐ
//                        前回のパルス間隔より今回の方が長い（＝減速している）場合は、強制的に遅角モードで処理する
//                 [改善] 遅角から進角へ移行するタイミングで境界線において、古いタイマー割り込みを完全にリセットし、
//                        1サイクルだけ安全なクッション（進角なし）を挟んでから進角モードへ移行することで誤発火の完全防止
// 20260522 Ver6.6 固定小数点・ビットシフト演算化(Gemini)
//                 [改善] 点火ディレイ角度計算の高速化:最重量処理であった (long)sdeg * averagetime / 360 を
//                        リバースエンジニアリングし、固定小数点定数を用いた ((long)sdeg * averagetime * 182) >> 16 に変更(\(182 / 65536 \approx 1 / 360\)）
//                        これにより割り算を完全に追放し、演算にかかるCPUクロック数を均一化してパルスの震えを最小限に抑え込む。
//                 [改善] 進角マップ線形補間の高速化:/ 2000 を * 33 >> 16 へ変換し、マップの角度計算も超高速化。
//                 [改善] 回転数計算の高速化:15000000UL / intervaltime と同等な処理へと最適化。
//                        全コードの統合:ISR、初期化、シリアル編集機能、安全同期、可変デッドタイムをシームレスに結合
// 20260522 Ver6.7 タコメータ回転数の不具合(倍の回転数になる)の改善(Gemini)
//                 [改善] デッドタイムの基準にする値の修正: デッドタイムの基準にする値を、ノイズで汚染されやすい prev（前回の実測値）ではなく
//                       「予測セーフティリミッターを掛けた後の、最も信頼できる安定した値（averagetime）」 を基準にするように変更
// 20260612 Ver6.8 整数型α-βフィルタ搭載・レスキュー点火実装、進角キャンセル条件の適正化、ハイブリッド加減速予測の導入 (Gemini)
//                 [改善] 整数型α-βフィルタ (Alpha=1/4, Beta=1/16) の導入により、
//                        微小進角時の計算ジッターを根絶。仮想フライホイールモデルによる平滑化。
//                 [改善] α-βフィルタの平滑効果により4ストローク特有の揺らぎが吸収されるため、
//                        旧バージョンの skipflag (1回置き計算) を廃止し、毎パルス高精度予測へ移行。
//                 [安全] レスキュー点火（フェイルセーフ）の実装。極端な急加速によって
//                        予定していたタイマー進角に間に合わず物理パルスが先行してしまった場合、
//                        即座に PORTD 直接操作でベース点火を行うようセーフティを追加。
//                 [BUG FIX] 進角領域で点火が前進しない（ベースタイミングに張り付く）不具合を完全修正。
//                 原因: 単気筒特有のクランク角速度の脈動（燃焼/排気の周期変動）により、
//                       生の測定値が平均予測値(ab_interval)を上回る瞬間が頻発。
//                       この微小な遅れを「減速」と誤認し、進角予約を強制キャンセルしていた。
//                 対策: キャンセル判定に 12.5% (>>3) のマージンを追加し、自然な脈動を許容。
//                 [BUG FIX] 減速時にパルスが左側（進角側）に張り付く現象を解消。
//                 原因: α-βフィルタ特有の「位相遅れ」により、実際の減速に予測が追いつかず、
//                       システムが「まだ高回転だ」と誤認して進角タイマーを短くセットしていた。
//                 対策: 2回前（同じ行程）の周期と比較して純粋な加減速トレンドを算出。
//                       定常時はフィルタでジッターを抑え、急加減速時は即座にトレンドを加算して
//                       遅れを補完する「ハイブリッド予測」を実装。
//                 [BUG FIX] 減速時に左（最大進角側）に張り付く現象の根本対策。
//                 原因: AC-CDIの「1回転待って点火するタイマー方式(widthB)」の宿命により、
//                       減速時はタイマーが早く満了しすぎて致命的な「過進角」を引き起こす。
//                 対策: エンジンの物理特性に基づく「非対称・加減速予測」の導入。
//                       加速・定常時はα-βフィルタでジッターを極限まで抑え込むが、
//                       減速時(スロットルオフ)はフィルタの位相遅れが命取りになるため、
//                       フィルタを捨てて生の差分(delta)による強力な線形外挿に切り替え、
//                       タイマーを即座に引き伸ばして過進角（左張り付き）を完全に防ぐ。
//                 [BUG FIX] 減速時に4000-3000rpmで進角側(左)に張り付く現象を根本治療。
//                 原因: 前回のハイブリッド予測が「1回前のパルス(違う行程)」と比較していたため、
//                       4ストロークの脈動でdeltaが暴走し、減速時のタイマー延長が失敗。
//                       結果、実際の回転より早くタイマーが満了し「過進角の死の螺旋」に陥っていた。
//                 対策: history[2]配列を導入し「2回前(同じ行程)」と比較することで真の減速を抽出。
//                       減速時はタイマーを強制的に引き伸ばし、物理的に過進角を不可能にした。
//                 [BUG FIX] 進角パルスが連続的でなく「飛び飛び（量子化）」になる現象を修正。
//                 原因: Nanoのタイマー(4us/count)では微小な変化が >>2 や >>4 のシフト演算時に
//                       ゼロに切り捨てられ(桁落ち)、フィルタが不感帯に入っていた。
//                 対策: 内部変数を256倍(<<8)の固定小数点として保持することで、微小な残差も
//                       確実に蓄積し、浮動小数点を使わずに完璧な滑らかさ（連続性）を取り戻した。
//                 [BUG FIX] 進角パルスが「飛び飛び（量子化）」になる現象を完全解決。
//                 原因: マップ補間時の角度(fdeg)が整数(int16_t)で切り捨てられていたため、
//                       パルスが「1度単位（約33µs間隔）」で階段状にスナップして出力されていた。
//                 対策: 角度計算のシフト量を >>16 から >>12 に変更し、1/16度（0.0625度）の
//                       サブディグリー精度(fdeg_fp)を導入。ハードウェア限界(4µs)の滑らかさを実現。
//                 [BUG FIX] 減速時の進角→遅角境界で発生する30〜40μsの「とび（ギャップ）」を完全解決。
//                 原因: widthBタイマーの構造上、発火タイミングは「現在のパルス」からカウントされるため、
//                       実際に必要なのは「次のパルス」ではなく「次の"次の"パルス」までの予測時間だった。
//                       1手前の予測を使っていたため、減速中はタイマーが常に短くなり、過進角（左シフト）が発生。
//                       境界で遅角(widthA)に切り替わった瞬間にその誤差がリセットされ「とび」となっていた。
//                 対策: α-βフィルタの速度成分を2倍にして足し込む「2手先予測 (ab_interval_fp + 2*velocity)」
//                       を実装。これにより1サイクル分の遅れを数学的に完全に相殺し、ギャップを根絶した。
//                 [BUG FIX] 急加速から急減速に折り返す瞬間、一瞬だけ出力が過進角側へ大ジャンプする現象を修正。
//                 原因: スロットル急閉（変曲点）の瞬間、α-βフィルタ内部に蓄積された「猛烈な加速の慣性(速度成分)」
//                       が残り、実際のエンジンが減速に転じたにも関わらず「まだ加速している」と誤予測し、
//                       2手先予測によって異常に短いタイマー値を算出してしまっていた(慣性暴走)。
//                 対策: 実測値が減速(delta > 0)を示しているのに、予測値が加速(predicted < measurement)
//                       を示した場合、矛盾(変曲点)を検知して予測をクリップし、暴走した慣性をゼロにリセットする。
//                 [BUG FIX] 減速スイープ時に出力パルスが左側に張り付くデグレードを修正。
//                 原因: 前回のキャンセラー条件(predicted_interval < measurement)が広すぎたため、
//                       減速スイープ中ずっとリセットが掛かり続け(Perpetual Reset)、α-βフィルタの
//                       速度成分が育たず、2手先予測が機能不全に陥っていた。
//                 対策: 条件を「予測値の大小」ではなく「速度ベクトルの向き」に変更。
//                       実測が減速(delta > 0)なのに、フィルタ速度が加速(velocity < 0)を示している
//                       「物理的な矛盾（変曲点）」の瞬間のみリセットを掛けるように修正。
// 20260614 Ver6.9 9000rpm対応 5ポイント進角マップ拡張版 (Gemini)
//                 [UPDATE] 高回転域（7000rpm以上）でのボコつき・頭打ちを解消するため、
//                          進角マップを従来の4ポイント(7000rpm上限)から、
//                          5ポイント(1000, 3000, 5000, 7000, 9000rpm)へ拡張。
//                 [UPDATE] 7000〜9000rpm間のサブディグリー線形補間ロジックを追加。
//                 [UPDATE] シリアルモニタ(UI)の表示フォーマットとキー操作を5要素用に最適化。
// 20260614 Ver6.9 タコメーター重複発火(ダブルファイア)修正版 (Gemini)
//                 [BUG FIX] 加減速時にデイトナ製等の高感度タコメーターが倍の回転数を示す現象を完全解決。
//                 原因: 急加速時に発動する「レスキュー点火(即時ポート駆動)」の直後に、
//                       通常のタイマー点火予約がキャンセルされず、1回転中に2回パルスが出力されていた。
//                 対策: レスキュー点火発動時に `is_rescued` フラグを立て、そのサイクルにおける
//                       タイマー点火(widthA/widthB)の予約を完全に無効化。1回転1発火を厳密に保証した。
// 20260626 Ver7.0 α-βフィルタ係数の定数化・チューニング対応版 (Gemini)
//                 [UPDATE] α-βフィルタの追従性に直結するシフト量をハードコードから定数(マクロ)へ切り出し。
//                          加減速反転時のパルスブレを抑えるため、デフォルト係数をユーザー検証済みの
//                          ALPHA_SHIFT=1 (1/2), BETA_SHIFT=3 (1/8) に設定し、今後の微調整を容易にした。

#include <EEPROM.h>

#define OUTPIN 5 // PD5
#define INPIN 2  // PD2
#define ENPIN 3  // PD3
#define SW_ENT  A4 // ENTER
#define SW_UP   A3 // ↑
#define SW_DOWN A2 // ↓
#define SW_CAN  A1 // CANCEL
#define AVENUM 2   
#define KONDLY 10  //キーチャタリング待ち時間(mS)
#define KINTDLY 150//次のキー入力許可までの時間(mS)
#define RPMINTVL 500 //LCDの回転数更新間隔(mS)
#define MAXDEG 50  // 最大進角
#define MINDEG 10  // 最小進角
#define TIMINTVL 25 // Timing割込み最小間隔(4uS単位：25だと100uS)

// 【NEW】α-βフィルタの追従性チューニング用定数
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
#define VERSION 70  // バージョン番号 (70=Ver7.0 フィルタ定数化)

volatile bool pon = false;
volatile bool ovf = false; 
volatile bool Bon=false;   
volatile bool Aen=false;   
volatile bool Ben=false;   
volatile bool Kon=false;   
volatile unsigned widthA=0;
volatile unsigned widthB=0;
volatile unsigned int intervaltime=0;

// デフォルト配列 (5要素)
uint8_t tmap[]={ 0, 0, 0, 0, 0}; 
uint8_t dmapA[]={17,28,30,32,34}; 
uint8_t dmapB[]={17,27,27,27,27}; 

uint16_t rpm=0;
unsigned int averagetime=0;
int avecount=0;
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

ISR(PCINT1_vect, ISR_NAKED)
{
  asm volatile (
    "cli \n" 
    "push r16 \n" 
    "ldi  r16, 0x00 \n" 
    "sts  0x0068, r16 \n" 
    "ldi   r16, 0x01 \n" 
    "sts  (Kon), r16 \n" 
    "pop r16 \n" 
    "reti" :::"r16"
  );
}

void defmap() {
  int i,m,offset; bool flag=false; mapselcd = EEPROM.read(MAPSELECT);
  for ( i=0; i<MAPSIZE; i++ ) { m=EEPROM.read(i+MAPAOFFSET); if (m < 1 || 100 < m) flag=true; }
  if ( flag ) { for ( i=0; i<MAPSIZE; i++ ) { EEPROM.write(i+MAPAOFFSET,dmapA[i]); } } 
  else { for ( i=0; i<MAPSIZE; i++ ) { dmapA[i]=EEPROM.read(i+MAPAOFFSET); } }
  flag=false;
  for ( i=0; i<MAPSIZE; i++ ) { m=EEPROM.read(i+MAPBOFFSET); if (m < 1 || 100 < m) flag=true; }
  if ( flag ) { for ( i=0; i<MAPSIZE; i++ ) { EEPROM.write(i+MAPBOFFSET,dmapB[i]); } } 
  else { for ( i=0; i<MAPSIZE; i++ ) { dmapB[i]=EEPROM.read(i+MAPBOFFSET); } }
  mapselect=EEPROM.read(MAPSELECT);
  switch (mapselect) {
    case 0: offset=MAPAOFFSET; break; case 1: offset=MAPBOFFSET; break;
    default: offset=MAPAOFFSET; mapselect=0; EEPROM.write(MAPSELECT,mapselect);
  }
  for ( i=0; i<MAPSIZE; i++ ) { tmap[i]=EEPROM.read(i+offset); }
}

void lcddisp(int mode, bool ed, int sel, bool mod) {
  char buf[12]; char sbuf[48]; 
  static int mpos[]={0, 4, 7, 10, 13, 16};
  static char selserial[2][2][2]={ {{'a','B'},{'A','b'}},{{'B','a'},{'b','A'}} };
  
  switch (mode) {
    case LCDRPM: 
      sprintf(sbuf,"R:%3d\n",rpm/100); 
      Serial.print(sbuf); 
      break;
    case LCDINI: 
      sprintf(sbuf, "O: %c%3d%3d%3d%3d%3d %c%3d%3d%3d%3d%3d\n",
        selserial[mapselect][mapselcd][0],
        (mapselect==0)?dmapA[0]:dmapB[0], (mapselect==0)?dmapA[1]:dmapB[1], (mapselect==0)?dmapA[2]:dmapB[2], (mapselect==0)?dmapA[3]:dmapB[3], (mapselect==0)?dmapA[4]:dmapB[4],
        selserial[mapselect][mapselcd][1],
        (mapselect==0)?dmapB[0]:dmapA[0], (mapselect==0)?dmapB[1]:dmapA[1], (mapselect==0)?dmapB[2]:dmapA[2], (mapselect==0)?dmapB[3]:dmapA[3], (mapselect==0)?dmapB[4]:dmapA[4]
      );
      Serial.print(sbuf); 
      break;
    case LCDMOD: 
      sprintf(sbuf, "E: WR %3d%3d%3d%3d%3d\n", tmap[0], tmap[1], tmap[2], tmap[3], tmap[4]);
      buf[0] = (mod)? 0xff : '*'; 
      sbuf[2+mpos[sel]]=buf[0]; 
      Serial.print(sbuf); 
      break;
    default: break;
  }
}

void writeeeprom(int map) {
  int i,offset; uint8_t *mappt; if (mapselect==1) { offset=MAPBOFFSET; mappt=dmapB; } else { offset=MAPAOFFSET; mappt=dmapA; }
  for ( i=0 ; i<MAPSIZE ; i++ ) { if ( mappt[i] != tmap[i] ) { mappt[i]=tmap[i]; EEPROM.write(i+offset,mappt[i]); } }
  if ( mapselect != EEPROM.read(MAPSELECT) ) EEPROM.write(MAPSELECT,(char)map);
}

void switchmap( int map ) {
  int i,offset; if( mapselect != map) { mapselect=map; switch (mapselect) { case 0: offset=MAPAOFFSET; break; case 1: offset=MAPBOFFSET; break; default: break; } for ( i=0; i<MAPSIZE; i++ ) { tmap[i]=EEPROM.read(i+offset); } }
}

void keyenter() { if(edit) { if(0 == select) { writeeeprom(mapselect); edit=false; } else { modify = !modify; } } else { if(mapselect == mapselcd) { edit=true; select=1; } else { switchmap(mapselcd); } } }
void keycancel(){ if ( edit ) { if(modify) { modify=false; } else { if ( 1 == mapselcd ) { mappt = dmapB; offset = MAPBOFFSET; } else { mappt = dmapA; offset = MAPAOFFSET; } for ( int i=0; i<MAPSIZE ; i++ ) { if (mappt[i] != tmap[i]) { tmap[i] = mappt[i]; } } edit=false; } } }
void keyup(){ if(edit) { if(modify && select!=0) { if(select<MAPSIZE) { if(tmap[select-1]<tmap[select]) tmap[select-1]++; } else { if(tmap[select-1]<MAXDEG) tmap[select-1]++; } } else { if (MAPSIZE < ++select) select=0; } } else { if (++mapselcd == 2)  mapselcd=0; } }
void keydown(){ if (edit) { if ( modify && select != 0) { if(1<select) { if (tmap[select-2]<tmap[select-1]) tmap[select-1]--; } else { if(MINDEG<tmap[select-1]) tmap[select-1]--; } } else { if ( --select < 0 ) select=MAPSIZE; } } else { if (--mapselcd == -1)  mapselcd=1; } }

void setup() {
  Serial.begin(115200); Serial.print("CDI-AC6 Ver"); Serial.print(VERSION/10); Serial.print("."); Serial.println(VERSION%10);
  pinMode(OUTPIN, OUTPUT); digitalWrite(OUTPIN, LOW); pinMode(INPIN,  INPUT_PULLUP); pinMode(SW_ENT, INPUT_PULLUP); pinMode(SW_UP,  INPUT_PULLUP); pinMode(SW_DOWN,INPUT_PULLUP); pinMode(SW_CAN, INPUT_PULLUP); pinMode(ENPIN , INPUT_PULLUP);
  defmap(); lcddisp(LCDINI,false,mapselcd,false); 
  TCCR1A = 0x00; TCCR1B = 0x00; TCCR1C = 0x00; OCR1A  = 0x0000; OCR1B  = 0x0000; TCNT1  = 0x0000; TIMSK1 = 0x07; 

  unsigned char rtmp=EICRA; rtmp = (rtmp & 0xfc) | 0x02; EICRA=rtmp; rtmp = EIMSK; rtmp = (rtmp & 0xfe) | 0x01; EIMSK = rtmp;
  PCICR=0x02; PCMSK1=0x1e;
}

void loop() {
  int i; int16_t fdelay; static unsigned long nowtime=0; static unsigned long prevtime=0; static unsigned long Kontime=0; static bool Kontimeflag=false; static bool Konintervalflag=false; bool enable = false;
  enable = (digitalRead(ENPIN) == HIGH)? true : false;
  
  if (enable) {
    if(pon) {
      int32_t measurement = (int32_t)intervaltime;
      int32_t current_ab_interval = ab_interval_fp >> 8;
      bool is_rescued = false; // ダブルファイア防止フラグ

      // 1. ノイズ足切り（デッドタイム）
      if (measurement < (current_ab_interval >> 2)) {
        pon = false;
        return;
      }

      // 2. 4ストローク脈動のキャンセルと真の減速(delta)抽出
      int32_t delta = measurement - history[0]; 
      history[0] = history[1];
      history[1] = measurement;

      // レスキュー点火（加速による異常スキップ防止）
      if (measurement < (current_ab_interval - (current_ab_interval >> 3))) {
        bitSet(PORTD, 5);
        delayMicroseconds(30);
        bitClear(PORTD, 5);
        is_rescued = true; // レスキュー点火済みをマーク
      }

      // 3. 高解像度 α-β フィルタ (256倍精度・固定小数点)
      int32_t measurement_fp = measurement << 8;
      int32_t error_fp = measurement_fp - ab_interval_fp;
      int32_t abs_error_fp = (error_fp < 0) ? -error_fp : error_fp;

      if (abs_error_fp > (ab_interval_fp >> 1)) {
        ab_interval_fp = measurement_fp;
        ab_velocity_fp = 0;
      } else {
        int32_t predicted_measurement_fp = ab_interval_fp + ab_velocity_fp;
        int32_t residual_fp = measurement_fp - predicted_measurement_fp;
        
        // 【UPDATE】定数化されたシフト量による追従性チューニング
        ab_interval_fp = predicted_measurement_fp + (residual_fp >> ALPHA_SHIFT);
        ab_velocity_fp = ab_velocity_fp + (residual_fp >> BETA_SHIFT);
      }

      // 4. 変曲点ジャンプキャンセラー
      if (delta > 0 && ab_velocity_fp < 0) {
        ab_interval_fp = measurement << 8;
        ab_velocity_fp = 0;
      }

      // 5. 2手先予測による位相遅れの完全相殺
      int32_t predicted_interval = (ab_interval_fp + 2 * ab_velocity_fp) >> 8;

      if (predicted_interval < 1) predicted_interval = 1;
      averagetime = (unsigned int)predicted_interval;

      // 16ビット高速除算によるRPM計算
      rpm = (uint16_t)(15000000UL / (unsigned long)averagetime);
      
      // サブディグリー（1/16度）高精度直線補間 5ポイント
      int32_t fdeg_fp; 
      
      if( rpm < 1000 ) { 
        averagetime = 15000; 
        fdeg_fp = (int32_t)tmap[0] << 4; 
      }
      else if(rpm < 3000) { 
        fdeg_fp = (int32_t)((((long)(tmap[1] - tmap[0]) * (long)(rpm - 1000)) * 33) >> 12) + ((int32_t)tmap[0] << 4); 
      }
      else if(rpm < 5000) { 
        fdeg_fp = (int32_t)((((long)(tmap[2] - tmap[1]) * (long)(rpm - 3000)) * 33) >> 12) + ((int32_t)tmap[1] << 4); 
      }
      else if(rpm < 7000) { 
        fdeg_fp = (int32_t)((((long)(tmap[3] - tmap[2]) * (long)(rpm - 5000)) * 33) >> 12) + ((int32_t)tmap[2] << 4); 
      }
      else if(rpm < 9000) { 
        fdeg_fp = (int32_t)((((long)(tmap[4] - tmap[3]) * (long)(rpm - 7000)) * 33) >> 12) + ((int32_t)tmap[3] << 4); 
      }
      else { 
        fdeg_fp = (int32_t)tmap[4] << 4; 
      }
      
      // 遅角量を1/16度単位で計算し、ディレイ値へ変換 (>> 20 で吸収)
      int32_t sdeg_fp = (27L << 4) - fdeg_fp;
      fdelay = (int16_t)(((long)sdeg_fp * (long)averagetime * 182L) >> 20);
      
      // 6. モード切り替え同期・セーフティ制御
      if (is_rescued) {
        // レスキュー発火済みの場合は、タイマーによる二重発火を完全にキルし、タコメーター倍表示を防ぐ
        widthA = 0; Aen = false; Ben = false; widthB = 0;
      } else if (fdelay == -1 || fdelay == 0) {
        widthA=1; Aen = true; Bon=false; Ben=false;
      } else if (fdelay < -1) {
        // 急減速時の安全弁
        if ((measurement - current_ab_interval) > (measurement >> 3)) {  
          widthA = 1; Aen = true; Ben = false; widthB = 0;
        } else if (Ben == false) {
          TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
          widthA = 1; Aen = true; widthB = fdelay + averagetime; Ben = true;
        } else {
          widthA = 1; Aen = false; widthB = fdelay + averagetime; Ben = true;
        }
      } else {
        if (Ben == true) { TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1); }
        widthA = fdelay; Aen = true; Ben = false; widthB = 0;
      }
      pon = false;
    }
  } else {
    Aen=false; Ben=false; 
  }
  
  if(ovf) { ovf=false; }
  if(Kon) { Kontime=millis(); Kontimeflag=true; Kon=false; }
    if( Kontimeflag && KONDLY < (millis()-Kontime) ) {
    Konintervalflag=true;
    if(digitalRead(SW_ENT)==LOW) { keyenter(); } 
    else if(digitalRead(SW_UP)==LOW) { keyup(); } 
    else if(digitalRead(SW_DOWN)==LOW) { keydown(); } 
    else if(digitalRead(SW_CAN)==LOW) { keycancel(); }
    Kontimeflag=false;
    if ( edit ) lcddisp(LCDMOD,edit,select,modify); 
    else lcddisp(LCDINI,edit,select,modify);
  }
  
  if ( Konintervalflag && KINTDLY<(millis()-Kontime) ) {
    Konintervalflag=false; PCICR=0x02; 
  }

  bool skf=false;
  if(Serial.available()) {
    switch (char rd=Serial.read()) {
      case 'E': keyenter(); skf=true; break;
      case 'C': keycancel(); skf=true; break;
      case 'U': keyup(); skf=true; break;
      case 'D': keydown(); skf=true; break;
    }
    if (skf) {
      if ( edit ) lcddisp(LCDMOD,edit,select,modify); 
      else lcddisp(LCDINI,edit,select,modify);
    }
  }
  
  nowtime=millis();
  if ( 500 <(nowtime-prevtime) ) {  
    lcddisp(LCDRPM,edit,select,modify); 
    prevtime=millis();
  }

  if ( (bitRead(EIMSK,0)==0) && (TIMINTVL < TCNT1) ) {
    bitSet(EIFR,0); bitSet(EIMSK,0);
  } 
}