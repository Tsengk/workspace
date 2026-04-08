#ifndef __UNDO_MANAGER_H__
#define __UNDO_MANAGER_H__

#include <vector>

#include "models/UndoModel.h"

/**
 * @brief 撤销管理器。
 *        负责保存移动历史，并按后进先出的顺序弹出回退记录。
 */
class UndoManager
{
public:
    /**
     * @brief 清空当前撤销栈。
     */
    void init();

    /**
     * @brief 压入一条新的撤销记录。
     * @param undoModel 可用于回滚的移动数据。
     */
    void pushUndo(const UndoModel& undoModel);

    /**
     * @brief 当前是否还能继续回退。
     * @return 只要还有记录就返回 true。
     */
    bool canUndo() const;

    /**
     * @brief 弹出最近一次撤销记录。
     * @return 最近一次记录；如果为空则返回默认值。
     */
    UndoModel popUndo();

private:
    /** 使用后进先出的栈结构实现连续回退。 */
    std::vector<UndoModel> _undoStack;
};

#endif // __UNDO_MANAGER_H__
