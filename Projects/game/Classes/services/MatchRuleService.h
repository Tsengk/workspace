#ifndef __MATCH_RULE_SERVICE_H__
#define __MATCH_RULE_SERVICE_H__

#include "models/CardTypes.h"

/**
 * @brief 匹配规则服务。
 *        负责判断桌牌是否满足与顶部牌匹配的条件。
 */
class MatchRuleService
{
public:
    /**
     * @brief 判断两张牌点数是否刚好相差 1。
     * @param currentTopFace 当前顶部牌点数。
     * @param targetFace 被点击桌牌点数。
     * @return 是否满足匹配规则。
     */
    static bool canMatch(CardFaceType currentTopFace, CardFaceType targetFace);
};

#endif // __MATCH_RULE_SERVICE_H__
