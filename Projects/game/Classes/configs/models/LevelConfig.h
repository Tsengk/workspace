#ifndef __LEVEL_CONFIG_H__
#define __LEVEL_CONFIG_H__

#include <vector>

#include "cocos2d.h"
#include "models/CardTypes.h"

/**
 * @brief 关卡文件中的单张卡牌静态配置。
 *        描述关卡启动时这张牌的点数、花色和初始位置。
 */
struct LevelCardConfig
{
    /** 配置中的点数。 */
    CardFaceType face;
    /** 配置中的花色。 */
    CardSuitType suit;
    /** 牌在场景中的初始摆放坐标。 */
    cocos2d::Vec2 position;

    LevelCardConfig()
        : face(CFT_NONE)
        , suit(CST_NONE)
        , position(cocos2d::Vec2::ZERO)
    {
    }
};

/**
 * @brief 关卡静态配置。
 *        读取 JSON 后先落到这个结构，再由生成器转换成运行时模型。
 */
struct LevelConfig
{
    /** 开局时放在桌牌区的卡牌。 */
    std::vector<LevelCardConfig> playfieldCards;
    /** 开局时放在手牌区的卡牌，数组第一张会作为最上层牌。 */
    std::vector<LevelCardConfig> stackCards;
};

#endif // __LEVEL_CONFIG_H__
