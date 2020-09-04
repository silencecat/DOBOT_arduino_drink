/**
 * @file commandsync.h
 * 同期APIを定義します。
 * @version 1.0.0
 * @author Daiki Churei
 * @copyright 2018 Afrel Co.,Ltd.All Rights Reserved.
 * @date 2017/03/12
 */

#ifndef COMMANDSYNC_H
#define COMMANDSYNC_H

#include <stdint.h>

#include "command.h"

/**
 * @defgroup group1 追加されたコマンド
 * @brief 公式デモに含まれていないコマンドを定義します。
 * @{
 */

/**
 * @brief HOMEコマンドを定義します。
 */
typedef struct tagHOMECmd {
    uint32_t reserved; ///< 将来使用するために予約されています。0を指定してください。
} HOMECmd;

/**
* @brief WAITコマンドを定義します。
*/
typedef struct tagWAITCmd {
   uint32_t timeout; ///< タイムアウト時間[ms]
} WAITCmd;

/**
 * @brief HOMEコマンドを実行します。
 * @param [in]   homeCmd         HOMEコマンドを指すポインタ。
 * @param [in]   isQueued        この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex  キューコマンドのインデックス(コントローラーから返される)。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetHOMECmd(HOMECmd *homeCmd, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief WAITコマンドを実行します。
 * @param [in]   waitCmd         WAITコマンドを指すポインタ。
 * @param [in]   isQueued        この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex  キューコマンドのインデックス(コントローラーから返される)。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetWAITCmd(WAITCmd *waitCmd, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @}
 * @defgroup group2 キュー番号受信機能をサポートするコマンド
 * @brief 既存のAPIにキュー番号受信機能を付加します。
 * @{
 */

extern int SetHOMECmd2(HOMECmd *homeCmd, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetWAITCmd2(WAITCmd *waitCmd, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetEndEffectorParams2(EndEffectorParams *endEffectorParams, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetEndEffectorLaser2(bool enableCtrl, bool on, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetEndEffectorSuctionCup2(bool enableCtrl, bool suck, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetEndEffectorGripper2(bool enableCtrl, bool grip, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetJOGJointParams2(JOGJointParams *jogJointParams, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetJOGCoordinateParams2(JOGCoordinateParams *jogCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetJOGCommonParams2(JOGCommonParams *jogCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetJOGCmd2(JOGCmd *jogCmd, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetPTPJointParams2(PTPJointParams *ptpJointParams, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetPTPCoordinateParams2(PTPCoordinateParams *ptpCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetPTPJumpParams2(PTPJumpParams *ptpJumpParams, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetPTPCommonParams2(PTPCommonParams *ptpCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetPTPCmd2(PTPCmd *ptpCmd, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetARCCmd2(ARCCmd *arcCmd, bool isQueued, uint64_t *queuedCmdIndex);
extern int SetPTPCommonParams3(float s, float a);
/**
 * @}
 * @defgroup group3 同期コマンド
 * @brief 命令が実行し終わるまで待機する同期コマンドを定義します。
 * @{
 */

/**
 * @brief HOME機能を実行します。
 *
 * この命令は実行し終わるまで待機します。
 * @param [in]   homeCmd         HOMEコマンドを指すポインタ。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetHOMECmdSync(HOMECmd *homeCmd);

/**
 * @brief エンドエフェクタのパラメータを設定します。
 *
 * この命令は実行し終わるまで待機します。
 * @param [in]   endEffectorParams  エンドエフェクタのパラメータを指すポインタ。
 * @return                          成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetEndEffectorParamsSync(EndEffectorParams *endEffectorParams);

/**
 * @brief レーザーをオン、またはオフにします。
 *
 * この命令は実行し終わるまで待機します。
 * @param [in]   enableCtrl          コントロールを有効にする場合はtrue、無効にする場合はfalse。
 * @param [in]   on                  レーザーをオンにする場合はtrue、オフにする場合はfalse。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetEndEffectorLaserSync(bool enableCtrl, bool on);

/**
 * @brief 吸引カップを吸引、または解放させます。
 *
 * この命令は実行し終わるまで待機します。
 * @param [in]   enableCtrl          コントロールを有効にする場合はtrue、無効にする場合はfalse。
 * @param [in]   suck                吸引させる場合はtrue、解放させる場合はfalse。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetEndEffectorSuctionCupSync(bool enableCtrl, bool suck);

/**
 * @brief グリッパーを握らせる、もしくは離させます。
 *
 * この命令は実行し終わるまで待機します。
 * @param [in]   enableCtrl          コントロールを有効にする場合はtrue、無効にする場合はfalse。
 * @param [in]   grip                握らせる場合はtrue、離させる場合はfalse。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetEndEffectorGripperSync(bool enableCtrl, bool grip);

/**
 * @brief JOGコマンドを実行する時の、ジョイントパラメータを設定します。
 *
 * この命令は実行し終わるまで待機します。
 *
 * \parジョイントパラメータとは
 * ジョイントパラメータは以下の8つです。
 * - ジョイント1の角速度
 * - ジョイント2の角速度
 * - ジョイント3の角速度
 * - ジョイント4の角速度
 * - ジョイント1の角加速度
 * - ジョイント2の角加速度
 * - ジョイント3の角加速度
 * - ジョイント4の角加速度
 * @param [in]   jogJointParams JOGジョイントパラメータを指すポインタ。
 * @return                      成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGJointParamsSync(JOGJointParams *jogJointParams);

/**
 * @brief JOGコマンドを実行する時の、座標パラメータを設定します。
 *
 * この命令は実行し終わるまで待機します。
 *
 * \par座標パラメータとは
 * JOGコマンド実行時における座標パラメータは以下の8つです。
 * - X速度
 * - Y速度
 * - Z速度
 * - R速度
 * - X加速度
 * - Y加速度
 * - Z加速度
 * - R加速度
 * @param [in]   jogCoordinateParams JOG座標パラメータを指すポインタ。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGCoordinateParamsSync(JOGCoordinateParams *jogCoordinateParams);

/**
 * @brief JOGコマンドを実行する時の、共通パラメータを設定します。
 *
 * この命令は実行し終わるまで待機します。
 *
 * \par共通パラメータとは
 * 共通パラメータは以下の2つです。
 * - 速度比率
 * - 加速度比率
 * @param [in]   jogCommonParams JOG共通パラメータを指すポインタ。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGCommonParamsSync(JOGCommonParams *jogCommonParams);

/**
 * @brief JOGコマンドを実行します。
 *
 * この命令は実行し終わるまで待機します。
 * @param [in]   jogCmd          JOGコマンドを指すポインタ。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGCmdSync(JOGCmd *jogCmd);

/**
 * @brief PTPコマンドを実行する時の、ジョイントパラメータを設定します。
 *
 * この命令は実行し終わるまで待機します。
 *
 * \parジョイントパラメータとは
 * ジョイントパラメータは以下の8つです。
 * - ジョイント1の角速度
 * - ジョイント2の角速度
 * - ジョイント3の角速度
 * - ジョイント4の角速度
 * - ジョイント1の角加速度
 * - ジョイント2の角加速度
 * - ジョイント3の角加速度
 * - ジョイント4の角加速度
 * @param [in]   ptpJointParams PTPジョイントパラメータを指すポインタ。
 * @return                      成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPJointParamsSync(PTPJointParams *ptpJointParams);

/**
 * @brief PTPコマンドを実行する時の、座標パラメータを設定します。
 *
 * この命令は実行し終わるまで待機します。
 *
 * \par座標パラメータとは
 * PTPコマンド実行時における座標パラメータは以下の4つです。
 * - XYZ速度
 * - R速度
 * - XYZ加速度
 * - R加速度
 * @param [in]   ptpCoordinateParams PTP座標パラメータを指すポインタ。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPCoordinateParamsSync(PTPCoordinateParams *ptpCoordinateParams);

/**
 * @brief PTPコマンドを実行する時の、ジャンプパラメータを設定します。
 *
 * この命令は実行し終わるまで待機します。
 *
 * \parジャンプパラメータとは
 * ジャンプパラメータは以下の2つです。
 * - ジャンプ高
 * - Z制限(最大ジャンプ高)。
 * @param [in]   ptpJumpParams  PTPジャンプパラメータを指すポインタ。
 * @return                      成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPJumpParamsSync(PTPJumpParams *ptpJumpParams);

/**
 * @brief PTPコマンドを実行する時の、共通パラメータを設定します。
 *
 * この命令は実行し終わるまで待機します。
 *
 * \par共通パラメータとは
 * 共通パラメータは以下の2つです。
 * - 速度比率
 * - 加速度比率
 * @param [in]   ptpCommonParams PTP座標パラメータを指すポインタ。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPCommonParamsSync(PTPCommonParams *ptpCommonParams);

/**
 * @brief PTPコマンドを実行します。
 *
 * この命令は実行し終わるまで待機します。
 * @param [in]   ptpCmd          PTPコマンドを指すポインタ。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPCmdSync(PTPCmd *ptpCmd);

/**
 * @}
 * @defgroup group4 簡易コマンド
 * @brief 呼び出すのが容易なコマンドを定義します。
 * @{
 */

 /**
  * @brief HOME機能を実行します。
  *
  * この命令は実行し終わるまで待機します。
  * @return  成功した場合はtrue、そうでない場合はfalse。
  */
extern int SetHOMECmdSync();

/**
 * @brief PTPコマンドを実行します。
 *
 * この命令は実行し終わるまで待機します。
 * @param  ptpMode モード。
 * - @ref ::JUMP_XYZ
 * - @ref ::MOVJ_XYZ
 * - @ref ::MOVL_XYZ
 * - @ref ::JUMP_ANGLE
 * - @ref ::MOVJ_ANGLE
 * - @ref ::MOVL_ANGLE
 * - @ref ::MOVJ_INC
 * - @ref ::MOVL_INC
 * @param  x       X座標、またはジョイント1の角位置
 * @param  y       Y座標、またはジョイント2の角位置
 * @param  z       Z座標、またはジョイント3の角位置
 * @param  r       R座標、またはジョイント4の角位置
 * @return         成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPCmdSync(uint8_t ptpMode, float x, float y, float z, float r);


extern int SetPTPCmd3(PTPCmd *ptpCmd);
extern int SetPTPCmd3(uint8_t ptpMode, float x, float y, float z, float r);

extern int SetARCCmd3(float x1, float y1, float z1, float r1,float x2, float y2, float z2, float r2);
extern int SetQueuedCmdForceStopExec(void);
extern int emStop(void);

/**
 * @brief JOGコマンドを実行します。
 *
 * この命令は実行し終わるまで待機します。
 * @param  isJoint モード。
 * - @ref ::COORDINATE_MODEL
 * - @ref ::JOINT_MODEL
 * @param  cmd   操作指示。
 * - @ref ::IDEL
 * - @ref ::AP_DOWN
 * - @ref ::AN_DOWN
 * - @ref ::BP_DOWN
 * - @ref ::BN_DOWN
 * - @ref ::CP_DOWN
 * - @ref ::CN_DOWN
 * - @ref ::DP_DOWN
 * - @ref ::DN_DOWN
 * @return         成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGCmdSync(uint8_t isJoint, uint8_t cmd);

/**
 * @}
 */

#endif
