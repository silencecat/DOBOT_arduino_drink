/**
 * @file DobotStarterKit.h
 * Dobot入門用の Arduino ライブラリです。
 * @version 1.0.0
 * @author Daiki Churei
 * @copyright 2018 Afrel Co.,Ltd.All Rights Reserved.
 * @date 2017/03/12
 */
#ifndef DOBOT_STARTER_KIT_H
#define DOBOT_STARTER_KIT_H

#include "commandsync.h"

/**
 * @brief Dobotに接続し、使用できるように準備します。
 */
extern void SetupDobot(void);

/**
 *
 */
extern void InitDobot(void);

#endif
