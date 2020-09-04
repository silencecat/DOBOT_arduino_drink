#include "DobotStarterKit_M5.h"
#include "ESP32TimerInterrupt.h"
#include "WiredController_asukiaaa.h"
#include "math.h"
#include <M5Stack.h>

//控制杆
WiredController_asukiaaa controller(&Wire);
WiredController_asukiaaa_WriteInfo wInfo;
WiredController_asukiaaa_ReadInfo rInfo;

float a = 83; //杯子口径
float b = 38; //把手到杯口的高度
float Rh, c, Xs, Zs, Rs, TRd, StartRd, rd, Rn, Xp, Zp, Rp, Xm, Zm, Rm;
Pose nowPose;
bool isStop = false;
bool needStop = false;
bool readyToStart = false;
float Trigger = 135; //触发信号的r坐标。超过这个坐标后杯子会倒到底，并这个时间段会输出数字信号。

int TriggerPinNo = 2;//触发信号针脚

 //做个显示UI的
void printUI(){
  M5.Lcd.setTextSize(2);

  M5.Lcd.setCursor(0, 220);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.printf(" Start     Stop   Change");
  M5.Lcd.setTextSize(4);

  }

//防止传入相同参数，ARC方式出现相同坐标会出错。
float x1p = 0, y1p = 0, z1p = 0, r1p = 0, x2p = 0, y2p = 0, z2p = 0, r2p = 0;
void SetARCCmd(float x1, float y1, float z1, float r1, float x2, float y2, float z2, float r2) {
  //nowPose=GetPose();
  if ( ( (x2 == x1p && y2 == y1p && z2 == z1p && r2 == r1p) || (x2 == x2p && y2 == y2p && z2 == z2p && r2 == r2p) ) && isStop == false ) {
    return; //(x2==nowPose.x && y2==nowPose.y && z2==nowPose.z && r2==nowPose.r)
  }
  isStop = false;
  SetARCCmd3(x1, y1, z1, r1, x2, y2, z2, r2);
  //不执行完就发命令可能还是有问题
  x1p = x1;
  y1p = y1;
  z1p = z1;
  r1p = r1;
  x2p = x2;
  y2p = y2;
  z2p = z2;
  r2p = r2;

  return;
}

void GotoChangePose() { //返回交换水杯位置
  
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(TFT_YELLOW);
  M5.Lcd.printf("\n GoTo \n change \n Pose...　\n");
  
  needStop = false;
  readyToStart = false;
  SetPTPCommonParams3(100, 40); //更改速度
  SetPTPCmd3(MOVL_XYZ, 0, -300, 0 , -25);
  delay(6000);//这里应该用结束后暂停的命令的，但暂时出问题了。
  
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.printf("\n Ready.\n Please \n change \n the cup.\n");
  printUI();
  return;
}

void GotoStartPose() { //返回实验准备开始位置
  
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(TFT_YELLOW);
  M5.Lcd.printf("\n　GoTo \n Start \n Pose...　\n");
  
  SetPTPCommonParams3(100, 40); //更改速度

  //算出最终点
  Xp = Xs + a - c * cos((Rh - (StartRd)) * PI / 180);
  Zp = Zs + b - c * sin((Rh - (StartRd)) * PI / 180);
  Rp = Rs + StartRd;
  needStop = false;
  SetPTPCmd3(MOVL_XYZ, Xp, 0, Zp, Rp);
  readyToStart = true;
  delay(6000);//这里应该用结束后暂停的命令的，但暂时出问题了。
  
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.printf("\n Ready　\n");
  printUI();
  
  return;
}

void startDrink() {
  
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(BLUE);
  M5.Lcd.printf("\n Drink　\n");
  
  nowPose = GetPose();
  if (readyToStart == false) { //如果没有进入实验开始姿态则不开始。但架不住按太快
    return;
  }
  //算出中间点,不需要中间点
  SetPTPCommonParams3(100, 10); //更改速度

  Rp = Rs + (StartRd + Rn); //r的终点角度，因为 Rn = TRd - StartRd，这样可以把起始位置的设置抵消掉

  rd = StartRd + Rn - (Rp - nowPose.r) / 2; //需要运动的量减去实际还剩下的量除以二就是中间点
  Xm = Xs + a - c * cos((Rh - rd ) * PI / 180);
  Zm = Zs + b - c * sin((Rh - rd ) * PI / 180);
  Rm = Rs + rd;

  //算出最终点
  Xp = Xs + a - c * cos((Rh - (StartRd + Rn)) * PI / 180);
  Zp = Zs + b - c * sin((Rh - (StartRd + Rn)) * PI / 180);

  if (Rp - nowPose.r < 10  ||  nowPose.r > Trigger ) { //保护在最后一点点角度的时候能一次运行到底
    needStop = false;
    
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.printf("\n Trigger!　\n");
    printUI();
    
  } else {
    needStop = true;
  }


  if ( Rp - nowPose.r < 5  ) { //保护在最后一丁点的话不运行。
    return;
  }

  SetARCCmd(Xm, 0, Zm, Rm, Xp, 0, Zp, Rp);
  return;

}

void undoDrink() {
  
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(BLUE);
  M5.Lcd.printf("\n UndoDrink　\n");
  
  nowPose = GetPose();
  if (readyToStart == false) { //如果没有进入实验开始姿态则不开始。但架不住按太快
    return;
  }
  //算出最终点
  SetPTPCommonParams3(100, 10); //更改速度
  Rp = Rs + StartRd;

  Xp = Xs + a - c * cos((Rh - (StartRd)) * PI / 180);
  Zp = Zs + b - c * sin((Rh - (StartRd)) * PI / 180);

  //算出中间点 // Rp = Rs + (StartRd + Rn); //r的终点角度，因为 Rn = TRd - StartRd，这样可以把起始位置的设置抵消掉
  rd = StartRd + ( nowPose.r - Rp) / 2;
  Xm = Xs + a - c * cos((Rh - rd ) * PI / 180);
  Zm = Zs + b - c * sin((Rh - rd ) * PI / 180);
  Rm = Rs + rd;
  if ( nowPose.r - Rp < 10  ) { //保护在最后一点点角度的时候能一次运行到底
    needStop = false;
    
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.printf("\n Ready　\n");
    printUI();
    
  } else {
    needStop = true;
  }

  if ( nowPose.r - Rp < 5  ) { //保护在最后一丁点的话不运行。
    return;
  }

  SetARCCmd(Xm, 0, Zm, Rm, Xp, 0, Zp, Rp);
  return;
}

void setup() {
  // put your setup code here, to run once:
  

  M5.begin(true, false, true);
  M5.Power.begin();

  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.printf("\n　Preparing...\n Please turn on\n the Robot \n and restart ME");
  
  dacWrite(25, 0); //ノイズ対策
  


  Serial.begin(115200);
  Wire.begin();
  pinMode(TriggerPinNo, OUTPUT); //设置触发针脚为输出模式
  SetupDobot();//初始化机器人
  InitDobot();

  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextSize(4);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.printf("\n　Warming...　\n");

  SetHOMECmdSync();//校正


  //  SetPTPCmdSync(MOVL_XYZ, 250, 100, 0, 0);
  //  SetPTPCmdSync(MOVL_XYZ, 250, -100, 0, 0);
  //  SetPTPCmdSync(MOVL_XYZ, 250, 0, 0, 0);
  Rh = atan( b / a ) * 180 / PI;
  c = a * ( 1 / cos( Rh * PI / 180 ) );
  Xs = 240.0; //运动起始点坐标
  Zs = 10.0;
  Rs = 60.0;//水平点

  TRd = 89; //需要转的角度
  StartRd = 0; //起始位置，如果想要一开始就有斜度的话改这里
  rd = StartRd; //这条可能没用
  Rn = TRd - StartRd; //如果一开始没有斜度，那Rn就是TRd，即需要转的角度。

  GotoChangePose();//进入起始位置
}

String getBooleanResultStr(bool target) {
  return target ? "true" : "false";
}

void loop() {
  // put your main code here, to run repeatedly:
  wInfo.led1 = false;
  wInfo.led2 = false;
  wInfo.led3 = false;
  wInfo.led4 = false;


  //控制
  if (controller.read(&rInfo) == 0) {
    //SetPTPCmd3(MOVL_XYZ,250+(rInfo.joystickVertical-500)/10,(rInfo.joystickHorizontal-500)/10,0,0);
    //SetPTPCmd3(MOVJ_XYZ,250+(rInfo.joystickVertical-500)/10,(rInfo.joystickHorizontal-500)/10,0,0);
    //SetPTPCmd3(MOVJ_ANGLE,(rInfo.joystickVertical-500)/20,(rInfo.joystickHorizontal-500)/20,0,0);  //危险

    //通过遥感控制
    if (rInfo.joystickVertical - 500 > 100 ) {
      needStop = true;
      undoDrink();

    } else if (rInfo.joystickVertical - 500 < -100 ) {
      needStop = true;
      startDrink();

    } else if (needStop == true) {
      isStop = true;
      needStop == false;
      emStop(); //停车
    }

    nowPose = GetPose();
    Serial.println("pose: " + String(nowPose.r));
    //GetPose().x y z r 都可以用



    //按键控制
    if (rInfo.btnTop) {
      GotoChangePose();//进入起始位置
    }
    if (rInfo.btnBottom) {
      GotoStartPose();//进入实验开始位置
    }
    if (rInfo.btnRight) {
      //紧急停车
      isStop = true;
      needStop == false;
      emStop(); //停车
    }
    if (rInfo.btnLeft) {
      //SetPTPCommonParams3(100, 1);
    }

    //根据推杆强度显示跑马灯
    if (abs(rInfo.joystickVertical - 500) > 50 ) {
      wInfo.led1 = true;
    }
    if (abs(rInfo.joystickVertical - 500) > 100 ) {
      wInfo.led2 = true;
    }
    if (abs(rInfo.joystickVertical - 500) > 150 ) {
      wInfo.led3 = true;
    }
    if (abs(rInfo.joystickVertical - 500) > 200 ) {
      wInfo.led4 = true;
    }

  }//获取输入结束

  if ( nowPose.r > Trigger ) {
    //触发信号
    Serial.println("Trigger siggnal!!!");
    digitalWrite(TriggerPinNo, HIGH);
  } else {
    digitalWrite(TriggerPinNo, LOW);
  }

  
  M5.update();
  if (M5.BtnA.wasReleased()) {
    
    GotoStartPose();//进入实验开始位置
  } else if (M5.BtnB.wasReleased()) {
    //紧急停车
    isStop = true;
    needStop == false;
    emStop(); //停车
  } else if (M5.BtnC.wasReleased()) {
    GotoChangePose();//进入起始位置
  }
  



  //写入
  /*
    if (controller.write(wInfo) == 0) {
    Serial.println("Wrote info to turn on led ");
    } else {
    Serial.println("Cannot write info to controller.");
    }

    //输出按钮状态
    if (controller.read(&rInfo) == 0) {
    Serial.println("JoystickHorizontal: " + String(rInfo.joystickHorizontal));
    Serial.println("JoystickVertical: " + String(rInfo.joystickVertical));
    Serial.println("btnTop: " + getBooleanResultStr(rInfo.btnTop));
    Serial.println("btnLeft: " + getBooleanResultStr(rInfo.btnLeft));
    Serial.println("btnRight: " + getBooleanResultStr(rInfo.btnRight));
    Serial.println("btnBottom: " + getBooleanResultStr(rInfo.btnBottom));
    Serial.println("btnJoy: " + getBooleanResultStr(rInfo.btnJoy));
    } else {
    Serial.println("Cannot read info from controller.");
    }
  */

  delay(100);
}
