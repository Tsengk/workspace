#include "utils/CardUtils.h"

bool CardUtils::isRedSuit(CardSuitType suit)
{
    // 红桃和方块使用红色数字资源，其余使用黑色数字资源。
    return suit == CST_DIAMONDS || suit == CST_HEARTS;
}

std::string CardUtils::getFaceToken(CardFaceType face)
{
    // 将内部枚举值转换成资源文件命名使用的简写。
    switch (face)
    {
    case CFT_ACE:
        return "A";
    case CFT_TWO:
        return "2";
    case CFT_THREE:
        return "3";
    case CFT_FOUR:
        return "4";
    case CFT_FIVE:
        return "5";
    case CFT_SIX:
        return "6";
    case CFT_SEVEN:
        return "7";
    case CFT_EIGHT:
        return "8";
    case CFT_NINE:
        return "9";
    case CFT_TEN:
        return "10";
    case CFT_JACK:
        return "J";
    case CFT_QUEEN:
        return "Q";
    case CFT_KING:
        return "K";
    default:
        return "A";
    }
}

std::string CardUtils::getBigNumberRes(CardFaceType face, CardSuitType suit)
{
    // 大号数字放在牌面中央，用来强调当前牌点。
    return std::string("res/number/big_") + (isRedSuit(suit) ? "red_" : "black_") + getFaceToken(face) + ".png";
}

std::string CardUtils::getSmallNumberRes(CardFaceType face, CardSuitType suit)
{
    // 小号数字放在左上角，配合花色角标组成经典牌角样式。
    return std::string("res/number/small_") + (isRedSuit(suit) ? "red_" : "black_") + getFaceToken(face) + ".png";
}

std::string CardUtils::getSuitRes(CardSuitType suit)
{
    // 花色资源命名与枚举一一对应。
    switch (suit)
    {
    case CST_CLUBS:
        return "res/suits/club.png";
    case CST_DIAMONDS:
        return "res/suits/diamond.png";
    case CST_HEARTS:
        return "res/suits/heart.png";
    case CST_SPADES:
        return "res/suits/spade.png";
    default:
        return "res/suits/club.png";
    }
}
