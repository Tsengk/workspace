#ifndef __LEVEL_CONFIG_LOADER_H__
#define __LEVEL_CONFIG_LOADER_H__

#include <string>

#include "configs/models/LevelConfig.h"

/**
 * @brief 关卡配置加载器。
 *        负责把 JSON 文件解析成强类型的 LevelConfig 结构。
 */
class LevelConfigLoader
{
public:
    /**
     * @brief 从 Resources/ 目录加载关卡配置文件。
     * @param filePath Resources/ 下的相对路径。
     * @param outConfig 解析后的关卡配置输出。
     * @param outError 失败时的错误信息。
     * @return 是否解析成功。
     */
    static bool loadLevelConfig(const std::string& filePath, LevelConfig& outConfig, std::string& outError);
};

#endif // __LEVEL_CONFIG_LOADER_H__
