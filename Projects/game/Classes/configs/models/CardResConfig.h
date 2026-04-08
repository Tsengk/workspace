#ifndef __CARD_RES_CONFIG_H__
#define __CARD_RES_CONFIG_H__

#include <string>

#include "cocos2d.h"

/**
 * @brief 卡牌静态资源配置。
 *        统一描述一张牌上各个元素的资源路径和摆放位置。
 */
struct CardResConfig
{
    /** 所有卡牌共用的背景图资源。 */
    std::string cardBackgroundPath;
    /** 左上角小号数字的显示位置。 */
    cocos2d::Vec2 smallNumberPosition;
    /** 左上角小号花色图标的显示位置。 */
    cocos2d::Vec2 smallSuitPosition;
    /** 牌面中央大号数字的显示位置。 */
    cocos2d::Vec2 bigNumberPosition;
    /** 牌面中央花色图标的显示位置。 */
    cocos2d::Vec2 centerSuitPosition;
    /** 中央花色图标使用的缩放值。 */
    float centerSuitScale;

    /**
     * @brief 构建默认卡牌资源配置。
     * @return 可直接用于界面创建的默认配置。
     */
    static CardResConfig createDefault()
    {
        CardResConfig config;
        config.cardBackgroundPath = "res/card_general.png";
        config.smallNumberPosition = cocos2d::Vec2(42.0f, 236.0f);
        config.smallSuitPosition = cocos2d::Vec2(42.0f, 198.0f);
        config.bigNumberPosition = cocos2d::Vec2(98.0f, 160.0f);
        config.centerSuitPosition = cocos2d::Vec2(100.0f, 88.0f);
        config.centerSuitScale = 1.8f;
        return config;
    }
};

#endif // __CARD_RES_CONFIG_H__
