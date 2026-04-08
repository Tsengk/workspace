#include "configs/loaders/LevelConfigLoader.h"

#include "cocos2d.h"
#include "json/document.h"
#include "json/error/en.h"

namespace
{
// 校验点数是否落在枚举定义范围内，避免 JSON 配错导致非法值进入运行时模型。
bool isValidFaceValue(int faceValue)
{
    return faceValue >= static_cast<int>(CFT_ACE) && faceValue < static_cast<int>(CFT_NUM_CARD_FACE_TYPES);
}

// 校验花色是否落在枚举定义范围内。
bool isValidSuitValue(int suitValue)
{
    return suitValue >= static_cast<int>(CST_CLUBS) && suitValue < static_cast<int>(CST_NUM_CARD_SUIT_TYPES);
}

// 将单张牌的 JSON 描述转换成静态配置结构。
bool parseLevelCard(const rapidjson::Value& value, LevelCardConfig& outCardConfig, std::string& outError)
{
    if (!value.IsObject())
    {
        outError = "Each card entry must be a JSON object.";
        return false;
    }

    if (!value.HasMember("CardFace") || !value["CardFace"].IsInt() ||
        !value.HasMember("CardSuit") || !value["CardSuit"].IsInt() ||
        !value.HasMember("Position") || !value["Position"].IsObject())
    {
        outError = "Each card entry needs CardFace, CardSuit and Position fields.";
        return false;
    }

    const int faceValue = value["CardFace"].GetInt();
    const int suitValue = value["CardSuit"].GetInt();
    if (!isValidFaceValue(faceValue) || !isValidSuitValue(suitValue))
    {
        outError = "CardFace or CardSuit is outside the supported enum range.";
        return false;
    }

    // 位置字段直接保存为场景坐标，后续视图层按这个坐标摆放。
    const rapidjson::Value& positionValue = value["Position"];
    if (!positionValue.HasMember("x") || !positionValue["x"].IsNumber() ||
        !positionValue.HasMember("y") || !positionValue["y"].IsNumber())
    {
        outError = "Position must provide numeric x and y fields.";
        return false;
    }

    outCardConfig.face = static_cast<CardFaceType>(faceValue);
    outCardConfig.suit = static_cast<CardSuitType>(suitValue);
    outCardConfig.position = cocos2d::Vec2(positionValue["x"].GetFloat(), positionValue["y"].GetFloat());
    return true;
}

// 读取指定数组字段，例如 Playfield 或 Stack。
bool parseCardArray(const rapidjson::Document& document,
                    const char* key,
                    std::vector<LevelCardConfig>& outCards,
                    std::string& outError)
{
    if (!document.HasMember(key) || !document[key].IsArray())
    {
        outError = std::string("Missing array: ") + key;
        return false;
    }

    const rapidjson::Value& arrayValue = document[key];
    for (rapidjson::SizeType index = 0; index < arrayValue.Size(); ++index)
    {
        LevelCardConfig cardConfig;
        if (!parseLevelCard(arrayValue[index], cardConfig, outError))
        {
            return false;
        }

        outCards.push_back(cardConfig);
    }

    return true;
}
} // namespace

bool LevelConfigLoader::loadLevelConfig(const std::string& filePath, LevelConfig& outConfig, std::string& outError)
{
    // 每次重新加载前先清空输出，避免调用方拿到旧数据。
    outConfig.playfieldCards.clear();
    outConfig.stackCards.clear();
    outError.clear();

    // 1. 从 Resources 目录中读取关卡 JSON 文本。
    const std::string fileContent = cocos2d::FileUtils::getInstance()->getStringFromFile(filePath);
    if (fileContent.empty())
    {
        outError = std::string("Unable to read level config file: ") + filePath;
        return false;
    }

    // 2. 解析 JSON 文本，失败时把底层错误转成可读信息。
    rapidjson::Document document;
    document.Parse<0>(fileContent.c_str());
    if (document.HasParseError())
    {
        outError = std::string("JSON parse error: ") + rapidjson::GetParseError_En(document.GetParseError());
        return false;
    }

    // 3. 依次读取桌牌区和手/顶部牌区配置。
    if (!parseCardArray(document, "Playfield", outConfig.playfieldCards, outError))
    {
        return false;
    }

    if (!parseCardArray(document, "Stack", outConfig.stackCards, outError))
    {
        return false;
    }

    // 至少要有一张顶部牌，否则游戏规则无法开始。
    if (outConfig.stackCards.empty())
    {
        outError = "Stack array must contain at least one top card.";
        return false;
    }

    return true;
}
