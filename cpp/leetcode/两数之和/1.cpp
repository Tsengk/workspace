#include <iostream>
#include <vector>
#include <unordered_map>
using namespace std;

class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        vector<int> result;
        // 处理空数组情况
        if (nums.empty()) return result;
        unordered_map<int, int> map;

        for (int i = 0; i < nums.size(); i++) {
            // .count(key) 统计哈希表中是否存在指定的键，存在返回1，不存在返回0
            if (map.count(target - nums[i]) > 0) {
                result.push_back(i);
                result.push_back(map[target - nums[i]]);
                break;
            }
            // 当map遍历了前面的元素后，map中存储了这些元素的值和对应的下标
            // nums[i]是当前遍历的元素(key)，i是当前元素的下标(value)
            map[nums[i]] = i;
        }
        return result;
    }
};

int main() {
    vector<int> nums = {2, 11, 7, 15};
    int target = 9;

    Solution solution;
    vector<int> result = solution.twoSum(nums, target);
    
    if (result.size() == 2) {
        cout << "找到匹配的下标: [" << result[0] << ", " << result[1] << "]" << endl;
        cout << "对应的数字是: [" << nums[result[0]] << ", " << nums[result[1]] << "]" << endl;
    } else {
        cout << "未找到符合条件的两个数" << endl;
    }
}