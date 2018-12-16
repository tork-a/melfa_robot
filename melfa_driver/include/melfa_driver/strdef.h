// Copyright 2018 Tokyo Opensource Robotics Kyokai Association (TORK)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//***************************************************************
// リアルタイム制御サンプルプログラム
// 通信パケットデータ構造体定義ヘッダーファイル
//***************************************************************
// strdef.h
#include <stdint.h>
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
  uint32_t sflg1; // 構造フラグ1
  uint32_t sflg2; // 構造フラグ2
} POSE;

/********************************************************/
/* パルス座標系 (未使用軸は0にします) */
/* 各関節をモータパルス値で表した座標です */
/********************************************************/
typedef struct{
  int32_t p1; // モータ1軸
  int32_t p2; // モータ2軸
  int32_t p3; // モータ3軸
  int32_t p4; // モータ4軸
  int32_t p5; // モータ5軸
  int32_t p6; // モータ6軸
  int32_t p7; // 付加軸1(モータ7軸)
  int32_t p8; // 付加軸2(モータ8軸)
} PULSE;
/****************************************/
/* リアルタイム機能通信データパケット */
/****************************************/
typedef struct enet_rtcmd_str {
  uint16_t Command; // コマンド
#define MXT_CMD_NULL 0 // リアルタイム外部指令なし
#define MXT_CMD_MOVE 1 // リアルタイム外部指令あり
#define MXT_CMD_END 255 // リアルタイム外部指令終了
  uint16_t SendType; // 指令データタイプ指定
  uint16_t RecvType; // モニタデータタイプ指定

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
    int32_t lng[8]; // 整数タイプ[%/無単位など]
  } dat;
  uint16_t SendIOType; // 送信入出力信号データ指定
  uint16_t RecvIOType; // 返信入出力信号データ指定
#define MXT_IO_NULL 0 // データなし
#define MXT_IO_OUT 1 // 出力信号
#define MXT_IO_IN 2 // 入力信号
  uint16_t BitTop; // 先頭ビット番号
  uint16_t BitMask; // 送信用ビットマスクパターン指定(0x0001-0xffff)
  uint16_t IoData; // 入出力信号データ(0x0000-0xffff)
  uint16_t TCount; // タイムアウト時間のカウンタ値
  uint32_t CCount; // 通信データ用のカウンタ値
  uint16_t RecvType1; // 返信データタイプ指定1
  union rtdata1 { // モニタデータ1
    POSE pos1; // 直交タイプ[mm/rad]
    JOINT jnt1; // 関節タイプ[rad]
    PULSE pls1; // パルスタイプ[pls]
    int32_t lng1[8]; // 整数タイプ[%/無単位など]
  } dat1;
  uint16_t RecvType2; // 返信データタイプ指定2
  union rtdata2 { // モニタデータ2
    POSE pos2; // 直交タイプ[mm/rad]
    JOINT jnt2; // 関節タイプ[rad]
    PULSE pls2; // パルスタイプ[pls]または整数タイプ[%/無単位など]
    int32_t lng2[8]; // 整数タイプ[%/無単位など]
  } dat2;
  uint16_t RecvType3; // 返信データタイプ指定3
  union rtdata3 { // モニタデータ3
    POSE pos3; // 直交タイプ[mm/rad]
    JOINT jnt3; // 関節タイプ[rad]
    PULSE pls3; // パルスタイプ[pls]または整数タイプ[%/無単位など]
    int32_t lng3[8]; // 整数タイプ[%/無単位など]
  } dat3;
} MXTCMD;
