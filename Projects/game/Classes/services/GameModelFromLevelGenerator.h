#ifndef __GAME_MODEL_FROM_LEVEL_GENERATOR_H__
#define __GAME_MODEL_FROM_LEVEL_GENERATOR_H__

#include "configs/models/LevelConfig.h"
#include "models/GameModel.h"

/**
 * @brief 运行时模型生成器。
 *        负责把静态关卡配置转换成真正运行时使用的 GameModel。
 */
class GameModelFromLevelGenerator
{
public:
    /**
     * @brief 根据关卡配置生成运行时模型。
     * @param levelConfig 已加载好的静态关卡配置。
     * @return 填充完成的运行时模型。
     */
    GameModel generateGameModel(const LevelConfig& levelConfig) const;
};

#endif // __GAME_MODEL_FROM_LEVEL_GENERATOR_H__
