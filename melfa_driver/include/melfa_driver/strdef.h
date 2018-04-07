//***************************************************************
// リアルタイム制御サンプルプログラム
// 通信パケットデータ構造体定義ヘッダーファイル
//***************************************************************
// strdef.h
#define VER_H7
/********************************************************/
/* 関節座標系 (未使用軸は0にします) */
/* 各成分の詳細は、各ロボット付属の取扱説明書を */
/* 参照してください */
/********************************************************/
typedef struct{
  float j1; // J1軸角度 (ラジアン)
  float j2; // J2軸角度 (ラジアン)
  float j3; // J3軸角度 (ラジアン)
  float j4; // J4軸角度 (ラジアン)
  float j5; // J5軸角度 (ラジアン)
  float j6; // J6軸角度 (ラジアン)
  float j7; // 付加軸1(J7軸角度) (ラジアン)
  float j8; // 付加軸2(J8軸角度) (ラジアン)
} JOINT;
/********************************************************/
/* 直交座標系 (未使用軸は0にします) */
/* 各成分の詳細は、各ロボット付属の取扱説明書を */
/* 参照してください */
/********************************************************/
typedef struct{
  float x; // X軸座標値(mm)
  float y; // Y軸座標値(mm)
  float z; // Z軸座標値(mm)
  float a; // A軸座標値(ラジアン)
  float b; // B軸座標値(ラジアン)
  float c; // C軸座標値(ラジアン)
  float l1; // 付加軸1(mm またはラジアン)
  float l2; // 付加軸2(mm またはラジアン)
} WORLD;
typedef struct{
  WORLD w;
  unsigned int sflg1; // 構造フラグ1
  unsigned int sflg2; // 構造フラグ2
} POSE;

/********************************************************/
/* パルス座標系 (未使用軸は0にします) */
/* 各関節をモータパルス値で表した座標です */
/********************************************************/
typedef struct{
  long p1; // モータ1軸
  long p2; // モータ2軸
  long p3; // モータ3軸
  long p4; // モータ4軸
  long p5; // モータ5軸
  long p6; // モータ6軸
  long p7; // 付加軸1(モータ7軸)
  long p8; // 付加軸2(モータ8軸)
} PULSE;
/****************************************/
/* リアルタイム機能通信データパケット */
/****************************************/
typedef struct enet_rtcmd_str {
  unsigned short Command; // コマンド
#define MXT_CMD_NULL 0 // リアルタイム外部指令なし
#define MXT_CMD_MOVE 1 // リアルタイム外部指令あり
#define MXT_CMD_END 255 // リアルタイム外部指令終了
  unsigned short SendType; // 指令データタイプ指定
  unsigned short RecvType; // モニタデータタイプ指定

  //////////// 指令またはモニタデータタイプ //

#define MXT_TYP_NULL 0 // データなし

  // 指令用およびモニタ用 ////////////////////

#define MXT_TYP_POSE 1 // 直交データ(指令値)
#define MXT_TYP_JOINT 2 // 関節データ(指令値)
#define MXT_TYP_PULSE 3 // パルスデータ(指令値)

  // 位置関連モニタ用 ////////////////////////
#define MXT_TYP_FPOSE 4 // 直交データ(指令値フィルタ処理後)
#define MXT_TYP_FJOINT 5 // 関節データ(指令値フィルタ処理後)
#define MXT_TYP_FPULSE 6 // パルスデータ(指令値フィルタ処理後)
#define MXT_TYP_FB_POSE 7 // 直交データ(エンコーダフィードバック値)
#define MXT_TYP_FB_JOINT 8 // 関節データ(エンコーダフィードバック値)
#define MXT_TYP_FB_PULSE 9 // パルスデータ(エンコーダフィードバック値)
  // 電流関連モニタ用 ///////////////////////

#define MXT_TYP_CMDCUR 10 // 電流指令
#define MXT_TYP_FBKCUR 11 // 電流フィードバック
  union rtdata { // 指令データ
    POSE pos; // 直交タイプ[mm/rad]
    JOINT jnt; // 関節タイプ[rad]
    PULSE pls; // パルスタイプ[pls]
    long lng[8]; // 整数タイプ[%/無単位など]
  } dat;
  unsigned short SendIOType; // 送信入出力信号データ指定
  unsigned short RecvIOType; // 返信入出力信号データ指定
#define MXT_IO_NULL 0 // データなし
#define MXT_IO_OUT 1 // 出力信号
#define MXT_IO_IN 2 // 入力信号
  unsigned short BitTop; // 先頭ビット番号
  unsigned short BitMask; // 送信用ビットマスクパターン指定(0x0001-0xffff)
  unsigned short IoData; // 入出力信号データ(0x0000-0xffff)
  unsigned short TCount; // タイムアウト時間のカウンタ値
  unsigned long CCount; // 通信データ用のカウンタ値
  unsigned short RecvType1; // 返信データタイプ指定1
  union rtdata1 { // モニタデータ1
    POSE pos1; // 直交タイプ[mm/rad]
    JOINT jnt1; // 関節タイプ[rad]
    PULSE pls1; // パルスタイプ[pls]
    long lng1[8]; // 整数タイプ[%/無単位など]
  } dat1;
  unsigned short RecvType2; // 返信データタイプ指定2
  union rtdata2 { // モニタデータ2
    POSE pos2; // 直交タイプ[mm/rad]
    JOINT jnt2; // 関節タイプ[rad]
    PULSE pls2; // パルスタイプ[pls]または整数タイプ[%/無単位など]
    long lng2[8]; // 整数タイプ[%/無単位など]
  } dat2;
  unsigned short RecvType3; // 返信データタイプ指定3
  union rtdata3 { // モニタデータ3
    POSE pos3; // 直交タイプ[mm/rad]
    JOINT jnt3; // 関節タイプ[rad]
    PULSE pls3; // パルスタイプ[pls]または整数タイプ[%/無単位など]
    long lng3[8]; // 整数タイプ[%/無単位など]
  } dat3;
} MXTCMD;
