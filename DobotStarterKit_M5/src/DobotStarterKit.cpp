#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#include "DobotStarterKit_M5.h"

#include "Packet.h"
#include "Protocol.h"
#include "ProtocolID.h"
#include "ESP32TimerInterrupt.h"
#include "command.h"

// シリアル通信のTX/RXバッファーサイズを設定
#undef SERIAL_TX_BUFFER_SIZE
#undef SERIAL_RX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

// プロトタイプ宣言
void Serialread(void);
void printf_begin(void);


void IRAM_ATTR TimerHandler0(void)
{
  Serialread();
}

ESP32Timer ITimer0(0);//改造：使用esp时钟

void SetupDobot(void) {
    Serial.begin(115200);
    printf_begin();

    // 周期的に実行する処理の設定
    //FlexiTimer2::set(100, Serialread);
    //FlexiTimer2::start();

 if (ITimer0.attachInterruptInterval( 100 * 1000, TimerHandler0)){;}

//用这个试试看

    // プロトコルのバッファーを初期化
    ProtocolInit();
}




void InitDobot(void) {
    // エンドエフェクタのパラメータを設定
    EndEffectorParams endEffectorParams;
    endEffectorParams.xBias = 59.7f;
    endEffectorParams.yBias = 0;
    endEffectorParams.zBias = 0;
    SetEndEffectorParamsSync(&endEffectorParams);

    // JOGジョイントパラメータを設定
    JOGJointParams jogJointParams;
    jogJointParams.velocity[0] = 50;
    jogJointParams.acceleration[0] = 50;
    jogJointParams.velocity[1] = 50;
    jogJointParams.acceleration[1] = 50;
    jogJointParams.velocity[2] = 50;
    jogJointParams.acceleration[2] = 50;
    jogJointParams.velocity[3] = 50;
    jogJointParams.acceleration[3] = 50;
    SetJOGJointParamsSync(&jogJointParams);

    // JOG軸パラメーターを設定
    JOGCoordinateParams jogCoordinateParams;
    jogCoordinateParams.velocity[0] = 50;
    jogCoordinateParams.acceleration[0] = 50;
    jogCoordinateParams.velocity[1] = 50;
    jogCoordinateParams.acceleration[1] = 50;
    jogCoordinateParams.velocity[2] = 50;
    jogCoordinateParams.acceleration[2] = 50;
    jogCoordinateParams.velocity[3] = 50;
    jogCoordinateParams.acceleration[3] = 50;
    SetJOGCoordinateParamsSync(&jogCoordinateParams);

    // JOG共通パラメーターを設定
    JOGCommonParams jogCommonParams;
    jogCommonParams.velocityRatio = 100;
    jogCommonParams.accelerationRatio = 100;
    SetJOGCommonParamsSync(&jogCommonParams);

    // PTPジョイントパラメータを設定
    PTPJointParams ptpJointParams;
    ptpJointParams.velocity[0] = 100;
    ptpJointParams.acceleration[0] = 100;
    ptpJointParams.velocity[1] = 100;
    ptpJointParams.acceleration[1] = 100;
    ptpJointParams.velocity[2] = 100;
    ptpJointParams.acceleration[2] = 100;
    ptpJointParams.velocity[3] = 100;
    ptpJointParams.acceleration[3] = 100;
    SetPTPJointParamsSync(&ptpJointParams);

    // PTP軸パラメータを設定
    PTPCoordinateParams ptpCoordinateParams;
    ptpCoordinateParams.xyzVelocity = 100;
    ptpCoordinateParams.xyzAcceleration = 100;
    ptpCoordinateParams.rVelocity = 100;
    ptpCoordinateParams.rAcceleration = 100;
    SetPTPCoordinateParamsSync(&ptpCoordinateParams);

    // PTP JUMPモードのパラーメータを設定
    PTPJumpParams ptpJumpParams;
    ptpJumpParams.jumpHeight = 20;
    ptpJumpParams.maxJumpHeight = 100;
    SetPTPJumpParamsSync(&ptpJumpParams);

    // PTP共通パラメータを設定
    PTPCommonParams ptpCommonParams;
    ptpCommonParams.velocityRatio = 100;
    ptpCommonParams.accelerationRatio = 100;
    SetPTPCommonParamsSync(&ptpCommonParams);
}

/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:
** Returned value:
*********************************************************************************************************/
int Serial_putc(char c, struct __file *)
{
    Serial.write(c);
    return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:
** Output parameters:
** Returned value:
*********************************************************************************************************/
void printf_begin(void) {
    //fdevopen(&Serial_putc, 0);
}

/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:
** Returned value:
*********************************************************************************************************/
void Serialread() {
  while(Serial.available()) {
        uint8_t data = Serial.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
            RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
        }
  }
}
