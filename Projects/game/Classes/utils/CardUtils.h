#ifndef __CARD_UTILS_H__
#define __CARD_UTILS_H__

#include <string>

#include "models/CardTypes.h"

/**
 * @brief 卡牌工具类。
 *        负责把点数/花色枚举转换成界面资源路径。
 */
class CardUtils
{
public:
    /**
     * @brief 判断当前花色是否应该使用红色资源。
     * @param suit 花色枚举。
     * @return 红桃或方块返回 true。
     */
    static bool isRedSuit(CardSuitType suit);

    /**
     * @brief 把点数枚举转成资源命名使用的简写。
     * @param face 点数枚举。
     * @return 例如 A、2、10、J、Q、K。
     */
    static std::string getFaceToken(CardFaceType face);

    /**
     * @brief 生成大号数字资源路径。
     * @param face 点数枚举。
     * @param suit 花色枚举。
     * @return Resources/ 下的相对路径。
     */
    static std::string getBigNumberRes(CardFaceType face, CardSuitType suit);

    /**
     * @brief 生成小号数字资源路径。
     * @param face 点数枚举。
     * @param suit 花色枚举。
     * @return Resources/ 下的相对路径。
     */
    static std::string getSmallNumberRes(CardFaceType face, CardSuitType suit);

    /**
     * @brief 生成花色图标资源路径。
     * @param suit 花色枚举。
     * @return Resources/ 下的相对路径。
     */
    static std::string getSuitRes(CardSuitType suit);
};

#endif // __CARD_UTILS_H__
