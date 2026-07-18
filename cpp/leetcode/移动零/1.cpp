#include <iostream>
#include <vector>

class Solution {
public:
    void moveZeroes(std::vector<int>& nums) {
        int left = 0;
        for (int right = 0; right < nums.size(); right++) {
            if (nums[right] != 0) {
                // 使用标准库函数交换
                // nums = {0, 1, 0, 3, 12}
                // left = 0, right = 1, nums[left] = 0, nums[right] = 1
                // -> 交换后 nums = {1, 0, 0, 3, 12}
                std::swap(nums[right], nums[left]);
                left++;
            }
        }
    }
};

int main() {
    Solution solution;
    std::vector<int> nums = {0, 1, 0, 3, 12};
    solution.moveZeroes(nums);
    for (int num : nums) {
        std::cout << num << " ";
    }
    std::cout << std::endl;
    return 0;
}