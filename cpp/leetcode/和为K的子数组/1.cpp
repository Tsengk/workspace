#include <iostream>
#include <vector>
#include <unordered_map>

using namespace std;

class Solution {
public:
    int subarraySum(vector<int>& nums, int k) {
        int count = 0;
        int pre_sum = 0;
        unordered_map<int, int> pre_map;
        pre_map[0] = 1;

        for (int num : nums) {
            pre_sum += num;
            int target = pre_sum - k;
            if (pre_map.count(target)) {//如果target不存在在哈希表中，则返回false；如果存在，则返回true
                count += pre_map[target];
            }
            pre_map[pre_sum]++;//如果pre_sum不存在在哈希表中.自动在哈希表中插入这个不存在的键,对应的值赋上默认值（int类型的默认值为0）
        }
        return count;
    }
};

int main() {
    Solution solution;
    vector<int> nums = {1, 2, 3, 0, };
    int k = 3;
    int result = solution.subarraySum(nums, k);
    cout << "Number of subarrays with sum " << k << ": " << result << endl;
    return 0;
}