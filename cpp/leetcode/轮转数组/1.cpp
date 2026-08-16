#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class Solution {
public:
    vector<int> rotate(vector<int>& nums, int k) {
        int n = nums.size();
        if (n == 0) return {};

        k %= n;
        if (k == 0) return {};

        reverse(nums.begin(), nums.end());
        reverse(nums.begin(), nums.begin() + k);
        reverse(nums.begin() + k, nums.end());

        return nums;
    }
};

int main() {
    Solution solution;
    vector<int> nums = {1, 2, 3, 4, 5, 6, 7};
    vector<int> result = solution.rotate(nums,3);
    cout << "[";
    for (const auto& res : result) {
        cout << res;
        if (&res != &result.back()) {
            cout << ", ";
        }
    }
    cout << "]" << endl;
}