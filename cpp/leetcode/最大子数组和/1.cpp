#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

class Solution {
public:
    int MaxSubArray(vector<int>& nums) {
        int current_sum = nums[0];
        int max_sum = nums[0];

        for (size_t i = 1; i < nums.size(); ++i) {
            current_sum = max(nums[i], current_sum + nums[i]);
            max_sum = max(max_sum, current_sum);
        }
        return max_sum;
    }
};

int main() {
    Solution solution;
    vector<int> nums = {-2, 1, -3, 4, -1, 2, 1, -5, 4};
    int result = solution.MaxSubArray(nums);
    cout << "Maximum subarray sum: " << result << endl;

    return 0;
}