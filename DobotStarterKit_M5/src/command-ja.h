/**
 * @file command.h
 * コマンドを定義します。
 * @version 1.0.0
 * @author LiYi (Japanse translator: Daiki Churei)
 * @copyright Shenzhen Yuejiang Technology Co., LTD.
 * @date 2016/06/01
 */

#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

#pragma pack(push)
#pragma pack(1)

/**
 * @defgroup group0 コマンド
 * @brief 公式デモに含まれているコマンドです。
 * @{
 */

/**
 * @brief PTPコマンドを実行する時の、モードを定義します。
 */
enum
{
    JUMP_XYZ,   ///< 移動先を絶対座標(X, Y, Z)で指定するモード。モーションはJUMP。
    MOVJ_XYZ,   ///< 移動先を絶対座標(X, Y, Z)で指定するモード。モーションはMOVJ。
    MOVL_XYZ,   ///< 移動先を絶対座標(X, Y, Z)で指定するモード。モーションはMOVL。
    JUMP_ANGLE, ///< 移動先をジョイント角位置で指定するモード。モーションはJUMP。
    MOVJ_ANGLE, ///< 移動先をジョイント角位置で指定するモード。モーションはMOVJ。
    MOVL_ANGLE, ///< 移動先をジョイント角位置で指定するモード。モーションはMOVL。
    MOVJ_INC,   ///< 移動先を現在の位置からの相対座標(X, Y, Z)で指定するモード。モーションはMOVJ。
    MOVL_INC,   ///< 移動先を現在の位置からの相対座標(X, Y, Z)で指定するモード。モーションはMOVL。
};

/**
 * @brief JOGコマンドを実行する時の、操作指示を定義します。
 */
enum {
    IDEL,    ///< 無効なステータス
    AP_DOWN, ///< 座標JOGモードの場合はXを増加させる。ジョイントJOGモードの場合はジョイント1を正の方向に回転させる。
    AN_DOWN, ///< 座標JOGモードの場合はXを減少させる。ジョイントJOGモードの場合はジョイント1を負の方向に回転させる。
    BP_DOWN, ///< 座標JOGモードの場合はYを増加させる。ジョイントJOGモードの場合はジョイント2を正の方向に回転させる。
    BN_DOWN, ///< 座標JOGモードの場合はYを減少させる。ジョイントJOGモードの場合はジョイント2を負の方向に回転させる。
    CP_DOWN, ///< 座標JOGモードの場合はZを増加させる。ジョイントJOGモードの場合はジョイント3を正の方向に回転させる。
    CN_DOWN, ///< 座標JOGモードの場合はZを減少させる。ジョイントJOGモードの場合はジョイント3を負の方向に回転させる。
    DP_DOWN, ///< 座標JOGモードの場合はRを増加させる。ジョイントJOGモードの場合はジョイント4を正の方向に回転させる。
    DN_DOWN  ///< 座標JOGモードの場合はRを減少させる。ジョイントJOGモードの場合はジョイント4を負の方向に回転させる。
};

/**
 * @brief JOGコマンドを実行する時の、モードを定義します。
 */
enum{
    COORDINATE_MODEL, ///< 座標JOGモード。
    JOINT_MODEL       ///< ジョイントJOGモード。
};

/**
 * @brief エンドエフェクタのパラメータを定義します。
 *
 * エンドエフェクタの種類によって適切なバイアスを設定してください。
 * - ペン: (59.7, 0, 0)
 * - レーザー: (70.0, 0, 0)
 * - 吸引カップ: (59.7, 0, 0)
 * - グリッパー: (59.7, 0, 0)
 */
typedef struct tagEndEffectorParams {
    float xBias; ///< Xバイアス。
    float yBias; ///< Yバイアス。
    float zBias; ///< Zバイアス。
}EndEffectorParams;

/**
 * @brief JOGコマンドを実行する時の、ジョイントパラメータを定義します。
 */
typedef struct tagJOGJointParams {
    float velocity[4];     ///< 速度。
    float acceleration[4]; ///< 加速度。
}JOGJointParams;

/**
 * @brief JOGコマンドを実行する時の、座標パラメータを定義します。
 */
typedef struct tagJOGCoordinateParams {
    float velocity[4];     ///< 速度。
    float acceleration[4]; ///< 加速度。
}JOGCoordinateParams;

/**
 * @brief JOGコマンドを実行する時の、共通パラメータを定義します。
 */
typedef struct tagJOGCommonParams {
    float velocityRatio;     ///< 速度比率。
    float accelerationRatio; ///< 加速度比率。
}JOGCommonParams;

/**
 * @brief JOGコマンドを定義します。
 */
typedef struct tagJOGCmd {
  /**
   * @brief モード。
   * - @ref ::COORDINATE_MODEL
   * - @ref ::JOINT_MODEL
   */
    uint8_t isJoint;
    /**
     * @brief 操作指示。
     * - @ref ::IDEL
     * - @ref ::AP_DOWN
     * - @ref ::AN_DOWN
     * - @ref ::BP_DOWN
     * - @ref ::BN_DOWN
     * - @ref ::CP_DOWN
     * - @ref ::CN_DOWN
     * - @ref ::DP_DOWN
     * - @ref ::DN_DOWN
     */
    uint8_t cmd;
}JOGCmd;

/**
 * @brief PTPコマンドを実行する時の、ジョイントパラメータを定義します。
 */
typedef struct tagPTPJointParams {
    float velocity[4];     ///< 角速度。
    float acceleration[4]; ///< 角加速度。
}PTPJointParams;

/**
 * @brief PTPコマンドを実行する時の、座標パラメータを定義します。
 */
typedef struct tagPTPCoordinateParams {
    float xyzVelocity;     ///< XYZ速度。
    float rVelocity;       ///< R速度。
    float xyzAcceleration; ///< XYZ加速度。
    float rAcceleration;   ///< R加速度。
}PTPCoordinateParams;

/**
 * @brief PTPコマンドを実行する時の、ジャンプパラメータを定義します。
 */
typedef struct tagPTPJumpParams {
    float jumpHeight;    ///< ジャンプ高。
    float maxJumpHeight; ///< Z制限(最大ジャンプ高)
}PTPJumpParams;

/**
 * @brief PTPコマンドを実行する時の、共通パラメータを定義します。
 */
typedef struct tagPTPCommonParams {
    float velocityRatio;     ///< 速度比率。
    float accelerationRatio; ///< 加速度比率。
}PTPCommonParams;

/**
 * @brief PTPコマンドを定義します。
 */
typedef struct tagPTPCmd {
    /**
     * @brief モード。
     * - @ref ::JUMP_XYZ
     * - @ref ::MOVJ_XYZ
     * - @ref ::MOVL_XYZ
     * - @ref ::JUMP_ANGLE
     * - @ref ::MOVJ_ANGLE
     * - @ref ::MOVL_ANGLE
     * - @ref ::MOVJ_INC
     * - @ref ::MOVL_INC
     */
    uint8_t ptpMode;
    float x;     ///< X座標、またはジョイント1の角位置。
    float y;     ///< Y座標、またはジョイント2の角位置。
    float z;     ///< Z座標、またはジョイント3の角位置。
    float r;     ///< R座標、またはジョイント4の角位置。
}PTPCmd;

#pragma pack(pop)

/**
 * @brief エンドエフェクタのパラメータを設定します。
 * @param [in]   endEffectorParams  エンドエフェクタのパラメータを指すポインタ。
 * @param [in]   isQueued           この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex     キューコマンドのインデックス(コントローラーから返される)。
 * @return                          成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetEndEffectorParams(EndEffectorParams *endEffectorParams, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief レーザーを照射します。もしくは照射を停止します。
 * @param [in]   enableCtrl          コントロールを有効にする場合はtrue、無効にする場合はfalse。
 * @param [in]   on                  照射する場合はtrue、照射を停止する場合はfalse。
 * @param [in]   isQueued            この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex      キューコマンドのインデックス(コントローラーから返される)。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetEndEffectorLaser(bool enableCtrl, bool on, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief 吸引カップを使って吸引します。または吸引を停止します。
 * @param [in]   enableCtrl          コントロールを有効にする場合はtrue、無効にする場合はfalse。
 * @param [in]   suck                吸引する場合はtrue、吸引を停止する場合はfalse。
 * @param [in]   isQueued            この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex      キューコマンドのインデックス(コントローラーから返される)。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetEndEffectorSuctionCup(bool enableCtrl, bool suck, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief グリッパーを使って掴みます。もしくは離します。
 * @param [in]   enableCtrl          コントロールを有効にする場合はtrue、無効にする場合はfalse。
 * @param [in]   grip                握る場合はtrue、離す場合はfalse。
 * @param [in]   isQueued            この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex      キューコマンドのインデックス(コントローラーから返される)。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetEndEffectorGripper(bool enableCtrl, bool grip, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief JOGコマンドを実行する時の、ジョイントパラメータを設定します。
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
 * @param [in]   isQueued       この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex キューコマンドのインデックス(コントローラーから返される)。
 * @return                      成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGJointParams(JOGJointParams *jogJointParams, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief JOGコマンドを実行する時の、座標パラメータを設定します。
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
 * @param [in]   isQueued            この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex      キューコマンドのインデックス(コントローラーから返される)。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGCoordinateParams(JOGCoordinateParams *jogCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief JOGコマンドを実行する時の、共通パラメータを設定します。
 * \par共通パラメータとは
 * 共通パラメータは以下の2つです。
 * - 速度比率
 * - 加速度比率
 * @param [in]   jogCommonParams JOG共通パラメータを指すポインタ。
 * @param [in]   isQueued        この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex  キューコマンドのインデックス(コントローラーから返される)。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGCommonParams(JOGCommonParams *jogCommonParams, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief JOGコマンドを実行します。
 * @param [in]   jogCmd          JOGコマンドを指すポインタ。
 * @param [in]   isQueued        必ずtrueを指定してください。
 * @param [out]  queuedCmdIndex  キューコマンドのインデックス(コントローラーから返される)。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetJOGCmd(JOGCmd *jogCmd, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief PTPコマンドを実行する時の、ジョイントパラメータを設定します。
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
 * @param [in]   isQueued       この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex キューコマンドのインデックス(コントローラーから返される)。
 * @return                      成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPJointParams(PTPJointParams *ptpJointParams, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief PTPコマンドを実行する時の、座標パラメータを設定します。
 * \par座標パラメータとは
 * PTPコマンド実行時における座標パラメータは以下の4つです。
 * - XYZ速度
 * - R速度
 * - XYZ加速度
 * - R加速度
 * @param [in]   ptpCoordinateParams PTP座標パラメータを指すポインタ。
 * @param [in]   isQueued            この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex      キューコマンドのインデックス(コントローラーから返される)。
 * @return                           成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief PTPコマンドを実行する時の、ジャンプパラメータを設定します。
 * \parジャンプパラメータとは
 * ジャンプパラメータは以下の2つです。
 * - ジャンプ高
 * - Z制限(最大ジャンプ高)。
 * @param [in]   ptpJumpParams  PTPジャンプパラメータを指すポインタ。
 * @param [in]   isQueued       この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex キューコマンドのインデックス(コントローラーから返される)。
 * @return                      成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPJumpParams(PTPJumpParams *ptpJumpParams, bool isQueued, uint64_t *queuedCmdIndex);
/**
 * @brief PTPコマンドを実行する時の、共通パラメータを設定します。
 * \par共通パラメータとは
 * 共通パラメータは以下の2つです。
 * - 速度比率
 * - 加速度比率
 * @param [in]   ptpCommonParams PTP座標パラメータを指すポインタ。
 * @param [in]   isQueued        この命令をキューコマンドとして指定する場合はtrue、そうでない場合はfalse。
 * @param [out]  queuedCmdIndex  キューコマンドのインデックス(コントローラーから返される)。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPCommonParams(PTPCommonParams *ptpCommonParams, bool isQueued, uint64_t *queuedCmdIndex);

/**
 * @brief PTPコマンドを実行します。
 * @param [in]   ptpCmd          PTPコマンドを指すポインタ。
 * @param [in]   isQueued        必ずtrueを指定してください。
 * @param [out]  queuedCmdIndex  キューコマンドのインデックス(コントローラーから返される)。
 * @return                       成功した場合はtrue、そうでない場合はfalse。
 */
extern int SetPTPCmd(PTPCmd *ptpCmd, bool isQueued, uint64_t *queuedCmdIndex);

#endif
