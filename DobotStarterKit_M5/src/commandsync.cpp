#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#include "commandsync.h"
#include "Protocol.h"
#include "ProtocolDef.h"
#include "ProtocolID.h"
#include "Packet.h"

void SerialWrite();
bool ReadCmdResponse(uint8_t messageId, Message *message);
void WaitCmdExecution(uint64_t queuedCmdIndex);
int SetQueuedCmdClear(void);

int SetHOMECmd(HOMECmd *homeCmd, bool isQueued, uint64_t *queuedCmdIndex) {
    // 送信するメッセージを作成
    Message tempMessage;
    memset(&tempMessage, 0, sizeof(Message));
    tempMessage.id = ProtocolHOMECmd;
    tempMessage.rw = true;
    tempMessage.isQueued = isQueued;
    tempMessage.paramsLen = sizeof(HOMECmd);
    memcpy(tempMessage.params, (uint8_t *)homeCmd, tempMessage.paramsLen);
    // メッセージを送信
    MessageWrite(&gSerialProtocolHandler, &tempMessage);
    return true;
}

int SetWAITCmd(WAITCmd *waitCmd, bool isQueued, uint64_t *queuedCmdIndex) {
    // 送信するメッセージを作成
    Message tempMessage;
    memset(&tempMessage, 0, sizeof(Message));
    tempMessage.id = ProtocolWAITCmd;
    tempMessage.rw = true;
    tempMessage.isQueued = isQueued;
    tempMessage.paramsLen = sizeof(WAITCmd);
    memcpy(tempMessage.params, (uint8_t *)waitCmd, tempMessage.paramsLen);
    // メッセージを送信
    MessageWrite(&gSerialProtocolHandler, &tempMessage);
    return true;
}

int GetQueuedCmdCurrentIndex(uint64_t *queuedCmdIndex) {
    // 送信するメッセージを作成
    Message tempMessage;
    memset(&tempMessage, 0, sizeof(Message));
    tempMessage.id = ProtocolQueuedCmdCurrentIndex;
    tempMessage.rw = true;
    tempMessage.isQueued = false;
    tempMessage.paramsLen = 0;
    // メッセージを送信
    MessageWrite(&gSerialProtocolHandler, &tempMessage);
    PacketProcess(&gSerialProtocolHandler);
    SerialWrite();
    Message message;
    // 応答メッセージを受信
    if (!ReadCmdResponse(ProtocolQueuedCmdCurrentIndex, &message)) {
      return false;
    }
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
    return true;
}

int SetHOMECmd2(HOMECmd *homeCmd, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetHOMECmd(homeCmd, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolHOMECmd, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}

int SetWAITCmd2(WAITCmd *waitCmd, bool isQueued, uint64_t *queuedCmdIndex) {
    if (!SetWAITCmd(waitCmd, isQueued, queuedCmdIndex)) {
      return false;
    }
    // メッセージを送信
    PacketProcess(&gSerialProtocolHandler);
    SerialWrite();
    // 応答メッセージを受信
    Message message;
    if (!ReadCmdResponse(ProtocolWAITCmd, &message)) {
      return false;
    }
    if (isQueued) {
      *queuedCmdIndex = *((uint64_t*)&message.params[0]);
    }
    return true;
}

int SetEndEffectorParams2(EndEffectorParams *endEffectorParams, bool isQueued, uint64_t *queuedCmdIndex) {
    if (!SetEndEffectorParams(endEffectorParams, isQueued, queuedCmdIndex)) {
      return false;
    }
    // メッセージを送信
    PacketProcess(&gSerialProtocolHandler);
    SerialWrite();
    // 応答メッセージを受信
    Message message;
    if (!ReadCmdResponse(ProtocolEndEffectorParams, &message)) {
      return false;
    }
    if (isQueued) {
      *queuedCmdIndex = *((uint64_t*)&message.params[0]);
    }
    return true;
}

int SetEndEffectorLaser2(bool enableCtrl, bool on, bool isQueued, uint64_t *queuedCmdIndex) {
    if (!SetEndEffectorLaser(enableCtrl, on, isQueued, queuedCmdIndex)) {
      return false;
    }
    // メッセージを送信
    PacketProcess(&gSerialProtocolHandler);
    SerialWrite();
    // 応答メッセージを受信
    Message message;
    if (!ReadCmdResponse(ProtocolEndEffectorLaser, &message)) {
      return false;
    }
    if (isQueued) {
      *queuedCmdIndex = *((uint64_t*)&message.params[0]);
    }
    return true;
}

int SetEndEffectorSuctionCup2(bool enableCtrl, bool suck, bool isQueued, uint64_t *queuedCmdIndex) {
    if (!SetEndEffectorSuctionCup(enableCtrl, suck, isQueued, queuedCmdIndex)) {
      return false;
    }
    // メッセージを送信
    PacketProcess(&gSerialProtocolHandler);
    SerialWrite();
    // 応答メッセージを受信
    Message message;
    if (!ReadCmdResponse(ProtocolEndEffectorSuctionCup, &message)) {
      return false;
    }
    if (isQueued) {
      *queuedCmdIndex = *((uint64_t*)&message.params[0]);
    }
    return true;
}

int SetEndEffectorGripper2(bool enableCtrl, bool grip, bool isQueued, uint64_t *queuedCmdIndex) {
    if (!SetEndEffectorGripper(enableCtrl, grip, isQueued, queuedCmdIndex)) {
      return false;
    }
    // メッセージを送信
    PacketProcess(&gSerialProtocolHandler);
    SerialWrite();
    // 応答メッセージを受信
    Message message;
    if (!ReadCmdResponse(ProtocolEndEffectorGripper, &message)) {
      return false;
    }
    if (isQueued) {
      *queuedCmdIndex = *((uint64_t*)&message.params[0]);
    }
    return true;
}

int SetJOGJointParams2(JOGJointParams *jogJointParams, bool isQueued, uint64_t *queuedCmdIndex) {
    if (!SetJOGJointParams(jogJointParams, isQueued, queuedCmdIndex)) {
      return false;
    }
    // メッセージを送信
    PacketProcess(&gSerialProtocolHandler);
    SerialWrite();
    // 応答メッセージを受信
    Message message;
    if (!ReadCmdResponse(ProtocolJOGJointParams, &message)) {
      return false;
    }
    if (isQueued) {
      *queuedCmdIndex = *((uint64_t*)&message.params[0]);
    }
    return true;
}

int SetJOGCoordinateParams2(JOGCoordinateParams *jogCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetJOGCoordinateParams(jogCoordinateParams, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolJOGCoordinateParams, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}

int SetJOGCommonParams2(JOGCommonParams *jogCommonParams, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetJOGCommonParams(jogCommonParams, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolJOGCommonParams, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}

int SetJOGCmd2(JOGCmd *jogCmd, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetJOGCmd(jogCmd, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolJOGCmd, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}

int SetPTPJointParams2(PTPJointParams *ptpJointParams, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetPTPJointParams(ptpJointParams, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolPTPJointParams, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}

int SetPTPCoordinateParams2(PTPCoordinateParams *ptpCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetPTPCoordinateParams(ptpCoordinateParams, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolPTPCoordinateParams, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}

int SetPTPJumpParams2(PTPJumpParams *ptpJumpParams, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetPTPJumpParams(ptpJumpParams, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolPTPJumpParams, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}

int SetPTPCommonParams2(PTPCommonParams *ptpCommonParams, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetPTPCommonParams(ptpCommonParams, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolPTPCommonParams, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}

int SetPTPCmd2(PTPCmd *ptpCmd, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetPTPCmd(ptpCmd, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolPTPCmd, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}






































int SetHOMECmdSync(HOMECmd *homeCmd) {
  uint64_t queuedCmdIndex;
  SetHOMECmd2(homeCmd, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetEndEffectorParamsSync(EndEffectorParams *endEffectorParams) {
  uint64_t queuedCmdIndex;
  SetEndEffectorParams2(endEffectorParams, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetEndEffectorLaserSync(bool enableCtrl, bool on) {
  uint64_t queuedCmdIndex;
  SetEndEffectorLaser2(enableCtrl, on, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetEndEffectorSuctionCupSync(bool enableCtrl, bool suck) {
  uint64_t queuedCmdIndex;
  SetEndEffectorSuctionCup2(enableCtrl, suck, true, &queuedCmdIndex);
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetEndEffectorGripperSync(bool enableCtrl, bool grip) {
  uint64_t queuedCmdIndex;
  SetEndEffectorGripper2(enableCtrl, grip, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetJOGJointParamsSync(JOGJointParams *jogJointParams) {
  uint64_t queuedCmdIndex;
  SetJOGJointParams2(jogJointParams, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetJOGCoordinateParamsSync(JOGCoordinateParams *jogCoordinateParams) {
  uint64_t queuedCmdIndex;
  SetJOGCoordinateParams2(jogCoordinateParams, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetJOGCommonParamsSync(JOGCommonParams *jogCommonParams) {
  uint64_t queuedCmdIndex;
  SetJOGCommonParams2(jogCommonParams, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetJOGCmdSync(JOGCmd *jogCmd) {
  uint64_t queuedCmdIndex;
  SetJOGCmd2(jogCmd, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetPTPJointParamsSync(PTPJointParams *ptpJointParams) {
  uint64_t queuedCmdIndex;
  SetPTPJointParams2(ptpJointParams, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetPTPCoordinateParamsSync(PTPCoordinateParams *ptpCoordinateParams) {
  uint64_t queuedCmdIndex;
  SetPTPCoordinateParams2(ptpCoordinateParams, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetPTPJumpParamsSync(PTPJumpParams *ptpJumpParams) {
  uint64_t queuedCmdIndex;
  SetPTPJumpParams2(ptpJumpParams, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetPTPCommonParamsSync(PTPCommonParams *ptpCommonParams) {
  uint64_t queuedCmdIndex;
  SetPTPCommonParams2(ptpCommonParams, false, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  //WaitCmdExecution(queuedCmdIndex);
  return true;
}



int SetPTPCmdSync(PTPCmd *ptpCmd) {
  uint64_t queuedCmdIndex;
  SetPTPCmd2(ptpCmd, true, &queuedCmdIndex);
  // コマンドが実行し終わるまで待機
  WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetQueuedCmdForceStopExec(void){
//强制停止
    Message tempMessage;

    memset(&tempMessage, 0, sizeof(Message));
    tempMessage.id = 242;
    tempMessage.rw = true;
    tempMessage.isQueued = false;
    tempMessage.paramsLen = 0;

    MessageWrite(&gSerialProtocolHandler, &tempMessage);

    return true;
}

int SetPTPCmd3(PTPCmd *ptpCmd) {
  uint64_t queuedCmdIndex;
  SetPTPCmd2(ptpCmd, false, &queuedCmdIndex);
  // 改个不待机的,把队列去掉改成立即
  // WaitCmdExecution(queuedCmdIndex);
  return true;
}

int SetPTPCmd3(uint8_t ptpMode, float x, float y, float z, float r) {
//不待机的
  //SetQueuedCmdForceStopExec();
  PTPCmd ptpCmd;
  ptpCmd.ptpMode = ptpMode;
  ptpCmd.x = x;
  ptpCmd.y = y;
  ptpCmd.z = z;
  ptpCmd.r = r;
  return SetPTPCmd3(&ptpCmd);
}




int SetHOMECmdSync() {
    HOMECmd homeCmd;
    homeCmd.reserved = 0;
    return SetHOMECmdSync(&homeCmd);
}

int SetPTPCmdSync(uint8_t ptpMode, float x, float y, float z, float r) {
  PTPCmd ptpCmd;
  ptpCmd.ptpMode = ptpMode;
  ptpCmd.x = x;
  ptpCmd.y = y;
  ptpCmd.z = z;
  ptpCmd.r = r;
  return SetPTPCmdSync(&ptpCmd);
}

int SetJOGCmdSync(uint8_t isJoint, uint8_t cmd) {
  JOGCmd jogCmd;
  jogCmd.isJoint = isJoint;
  jogCmd.cmd = cmd;
  return SetJOGCmdSync(&jogCmd);
}

void SerialWrite() {
  uint8_t data;
  while (RingBufferIsEmpty(&gSerialProtocolHandler.txRawByteQueue) == false) {
      RingBufferDequeue(&gSerialProtocolHandler.txRawByteQueue, &data);
      Serial.write(data);
  }
}

bool ReadCmdResponse(uint8_t messageId, Message *message) {
  ProtocolResult result;
  while (true) {
    PacketProcess(&gSerialProtocolHandler);
    result = MessageRead(&gSerialProtocolHandler, message);
    if(result == ProtocolNoError && message->id == messageId){
      return true;
    }
    delay(50);
  }
}

void WaitCmdExecution(uint64_t queuedCmdIndex) {
  uint64_t queuedCmdCurrentIndex;
  while (true) {
    if(GetQueuedCmdCurrentIndex(&queuedCmdCurrentIndex)){
      if (queuedCmdCurrentIndex == queuedCmdIndex) {
        return;
      }
    }
    delay(100);
  }
}

//新做的执行ARC指令函数
int SetARCCmd2(ARCCmd *arcCmd, bool isQueued, uint64_t *queuedCmdIndex) {
  if (!SetARCCmd(arcCmd, isQueued, queuedCmdIndex)) {
    return false;
  }
  // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  Message message;
  if (!ReadCmdResponse(ProtocolARCCmd, &message)) {
    return false;
  }
  if (isQueued) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  return true;
}


int SetARCCmd3(float x1, float y1, float z1, float r1,float x2, float y2, float z2, float r2) {
//不待机的
  SetQueuedCmdForceStopExec();
  ARCCmd arcCmd;
  arcCmd.cirPoint.x=x1;
  arcCmd.cirPoint.y=y1;
  arcCmd.cirPoint.z=z1;
  arcCmd.cirPoint.r=r1;
  arcCmd.toPoint.x=x2;
  arcCmd.toPoint.y=y2;
  arcCmd.toPoint.z=z2;
  arcCmd.toPoint.r=r2;

  uint64_t queuedCmdIndex;

  SetARCCmd2(&arcCmd, false, &queuedCmdIndex);
    // 改个不待机的,把队列去掉改成立即
  // WaitCmdExecution(queuedCmdIndex);
  return true;
}

int emStop(void) {
//紧急暂停，不知怎么运行完就不动了
  uint64_t queuedCmdIndex;
  SetQueuedCmdForceStopExec();
  //WaitCmdExecution(queuedCmdIndex);//应该是这条的问题，或者是queuedCmdIndex的问题。
    // メッセージを送信
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  // 応答メッセージを受信
  /*Message message;
  if (!ReadCmdResponse(ProtocolPTPCmd, &message)) {
    return false;
  }
  if (false) {
    *queuedCmdIndex = *((uint64_t*)&message.params[0]);
  }
  */
  return true;
}

int SetPTPCommonParams3(float s, float a) {
  uint64_t queuedCmdIndex;
  PTPCommonParams ptpCommonParams;
  ptpCommonParams.velocityRatio = s;
  ptpCommonParams.accelerationRatio = a;
  SetPTPCommonParamsSync(&ptpCommonParams);
  // コマンドが実行し終わるまで待機
  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  //WaitCmdExecution(queuedCmdIndex);
  return true;
}

//读角度信息
float MessageRead_pose_r(ProtocolHandler *protocolHandler, Message *message)
{
    Pose pose;
    float r;
    RingBuffer *rxPacketQueue = &protocolHandler->rxPacketQueue;
    Packet *packet = &protocolHandler->rxAppPacket;

    if (RingBufferIsEmpty(rxPacketQueue)) {
        return -999;
    }
    // Read a packet 吧消息读进packet里面
    RingBufferDequeue(rxPacketQueue, packet);
    // Transform to message
    message->id = packet->payload.id;
    message->rw = packet->payload.ctrl & 0x01;
    message->isQueued = (packet->payload.ctrl >> 1) & 0x01;
    message->paramsLen = packet->header.payloadLen - 2;
    if (message->paramsLen) {
        memcpy(&message->params[0], &packet->payload.params[0], message->paramsLen);
    }

    memcpy(&pose,&message->params , message->paramsLen);  //把message->params的地址拷给pose
    return pose.r;
}
//接收实时角度信息
float ReadPoseResponse_r(uint8_t messageId, Message *message) {
  float r;
  while (true) {
    PacketProcess(&gSerialProtocolHandler);
    r = MessageRead_pose_r(&gSerialProtocolHandler, message);
    if(r != -999 && message->id == messageId){
      //有返回值


      return r;
    }
    delay(50);
  }
}

float GetPose_r(void){
  //取得实时角度函数
  //Pose pose;
  

  //组装一个message
  Message tempMessage;
  memset(&tempMessage, 0, sizeof(Message));
  tempMessage.id = ProtocolGetPose;
  tempMessage.rw = false;
  tempMessage.isQueued = false;
  tempMessage.paramsLen = 0;
  // メッセージを送信
  MessageWrite(&gSerialProtocolHandler, &tempMessage);

  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  Message message;
  // 応答メッセージを受信
  //返回pose
  //暂时就先返回一个r
  return ReadPoseResponse_r(ProtocolGetPose, &message);

}
/*
//机械臂实时位置Pose定义
typedef struct tagPose {
 float x; //机械臂坐标系 x
 float y; //机械臂坐标系 y
 float z; //机械臂坐标系 z
 float r; //机械臂坐标系 r
 float jointAngle[4]; //机械臂关节轴(底座、大臂、小臂、末端)角度
}Pose;
*/


Pose MessageRead_pose(ProtocolHandler *protocolHandler, Message *message)
{
    Pose pose;
    float r;
    RingBuffer *rxPacketQueue = &protocolHandler->rxPacketQueue;
    Packet *packet = &protocolHandler->rxAppPacket;

    if (RingBufferIsEmpty(rxPacketQueue)) {
        pose.r=-999;
        return pose;
    }
    // Read a packet 吧消息读进packet里面
    RingBufferDequeue(rxPacketQueue, packet);
    // Transform to message
    message->id = packet->payload.id;
    message->rw = packet->payload.ctrl & 0x01;
    message->isQueued = (packet->payload.ctrl >> 1) & 0x01;
    message->paramsLen = packet->header.payloadLen - 2;
    if (message->paramsLen) {
        memcpy(&message->params[0], &packet->payload.params[0], message->paramsLen);
    }

    memcpy(&pose,&message->params , message->paramsLen);  //把message->params的地址拷给pose
    return pose;
}
//接收实时角度信息
Pose ReadPoseResponse(uint8_t messageId, Message *message) {
  Pose pose;
  while (true) {
    PacketProcess(&gSerialProtocolHandler);
    pose = MessageRead_pose(&gSerialProtocolHandler, message);
    if(pose.r != -999 && message->id == messageId){
      //有返回值
      return pose;
    }
    delay(50);
  }
}

Pose GetPose(void){
  //取得实时角度函数
  //Pose pose;
  

  //组装一个message
  Message tempMessage;
  memset(&tempMessage, 0, sizeof(Message));
  tempMessage.id = ProtocolGetPose;
  tempMessage.rw = false;
  tempMessage.isQueued = false;
  tempMessage.paramsLen = 0;
  // メッセージを送信
  MessageWrite(&gSerialProtocolHandler, &tempMessage);

  PacketProcess(&gSerialProtocolHandler);
  SerialWrite();
  Message message;
  // 応答メッセージを受信
  //返回pose
  //暂时就先返回一个r
  return ReadPoseResponse(ProtocolGetPose, &message);

}