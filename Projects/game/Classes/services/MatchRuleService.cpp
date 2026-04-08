#include "services/MatchRuleService.h"

#include <cstdlib>

bool MatchRuleService::canMatch(CardFaceType currentTopFace, CardFaceType targetFace)
{
    // 任意一方为空都不允许匹配。
    if (currentTopFace == CFT_NONE || targetFace == CFT_NONE)
    {
        return false;
    }

    // 需求规定：点数与顶部牌相差 1 即可匹配，花色不参与判断。
    const int faceDelta = std::abs(static_cast<int>(currentTopFace) - static_cast<int>(targetFace));
    return faceDelta == 1;
}
