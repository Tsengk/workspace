#ifndef __CARD_MODEL_H__
#define __CARD_MODEL_H__

#include "cocos2d.h"

#include "models/CardTypes.h"

/**
 * @brief 单张卡牌的运行时模型。
 *        控制器、撤销管理器和视图都会围绕这个结构读写当前卡牌状态。
 */
struct CardModel
{
    /** 运行时唯一 id，用于点击命中和撤销查找。 */
    int cardId;
    /** 当前显示的点数。 */
    CardFaceType face;
    /** 当前显示的花色。 */
    CardSuitType suit;
    /** 当前所属的逻辑区域。 */
    CardAreaType area;
    /** 卡牌的原始落点，回退时会移动回这里。 */
    cocos2d::Vec2 homePosition;
    /** 当前逻辑位置，视图层会据此计算显示位置。 */
    cocos2d::Vec2 currentPosition;
    /** 当前是否正面朝上显示。 */
    bool isFaceUp;
    /** 当前是否允许直接点击。 */
    bool isInteractive;

    CardModel()
        : cardId(kInvalidCardId)
        , face(CFT_NONE)
        , suit(CST_NONE)
        , area(CAT_NONE)
        , homePosition(cocos2d::Vec2::ZERO)
        , currentPosition(cocos2d::Vec2::ZERO)
        , isFaceUp(true)
        , isInteractive(true)
    {
    }
};

#endif // __CARD_MODEL_H__
