#ifndef __GAME_SCENE_H__
#define __GAME_SCENE_H__

#include <memory>

#include "cocos2d.h"

#include "controllers/GameController.h"

/**
 * @brief 游戏主场景。
 *        这里只负责创建顶层控制器，并把场景节点本身作为视图挂载根节点。
 */
class GameScene : public cocos2d::Scene
{
public:
    /**
     * @brief 创建主场景实例。
     * @return 可直接运行的场景对象。
     */
    static cocos2d::Scene* createScene();

    /**
     * @brief 初始化主场景，并启动默认关卡。
     * @return 初始化是否成功。
     */
    virtual bool init();

    CREATE_FUNC(GameScene);

private:
    /** 顶层游戏控制器，负责驱动模型与视图协作。 */
    std::unique_ptr<GameController> _gameController;
};

#endif // __GAME_SCENE_H__
