#include <iostream>
#include <vector>
#include <unordered_set>
#include <algorithm>

class Solution {
public:
    int longestConsecutive(std::vector<int>& nums) {
        // std::unordered_set 仅存储键（key），且键必须唯一（去重），常用成员：insert()、find()、erase()、count()
        // .begin() 返回指向容器中第一个元素的迭代器,
        // .end() 返回指向容器中最后一个元素之后位置的迭代器,表示结束
        std::unordered_set<int> num_set(nums.begin(), nums.end());
        int longest_streak = 0;

        for (int num : num_set) {
            if (num_set.find(num - 1) == num_set.end()) {
                int current_num = num;
                int current_streak = 1;

                while (num_set.find(current_num + 1) != num_set.end()) {
                    current_num += 1;
                    current_streak += 1;
                }
                longest_streak = std::max(longest_streak, current_streak);
            }
        }
        return longest_streak;
    }
};

int main() {
    Solution solution;

    std::vector<int> nums = {100, 4, 200, 1, 3, 2};
    std::cout << "输出：" << solution.longestConsecutive(nums) << std::endl;
}