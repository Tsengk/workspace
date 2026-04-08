#ifndef __CARD_TYPES_H__
#define __CARD_TYPES_H__

/**
 * @brief 花色枚举。
 *        供配置层、模型层和视图层共用，确保同一张牌在各层使用统一取值。
 */
enum CardSuitType
{
    CST_NONE = -1,
    CST_CLUBS = 0,
    CST_DIAMONDS,
    CST_HEARTS,
    CST_SPADES,
    CST_NUM_CARD_SUIT_TYPES
};

/**
 * @brief 点数枚举。
 *        匹配规则、资源命名和模型数据都依赖这一组统一的点数定义。
 */
enum CardFaceType
{
    CFT_NONE = -1,
    CFT_ACE = 0,
    CFT_TWO,
    CFT_THREE,
    CFT_FOUR,
    CFT_FIVE,
    CFT_SIX,
    CFT_SEVEN,
    CFT_EIGHT,
    CFT_NINE,
    CFT_TEN,
    CFT_JACK,
    CFT_QUEEN,
    CFT_KING,
    CFT_NUM_CARD_FACE_TYPES
};

/**
 * @brief 卡牌所在区域枚举。
 *        用来标记一张牌当前处于桌牌区、手牌区、顶部牌位还是顶部历史区。
 */
enum CardAreaType
{
    CAT_NONE = -1,
    CAT_PLAYFIELD = 0,
    CAT_STACK_RESERVE,
    CAT_TOP,
    CAT_TOP_HISTORY
};

/** 无效卡牌 id，占位或查找失败时使用。 */
static const int kInvalidCardId = -1;

#endif // __CARD_TYPES_H__
