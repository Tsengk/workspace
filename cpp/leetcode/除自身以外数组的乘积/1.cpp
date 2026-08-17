#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
    vector<int> productExceptSelf(vector<int>& nums) {
        int n = nums.size();
        vector<int> answer(n, 1);

        int left = 1;
        for (int i = 0; i < n; ++i) {
            answer[i] = left;
            left *= nums[i];
        }

        int right = 1;
        for (int i = n - 1; i >= 0; --i) {
            answer[i] *= right;
            right *= nums[i];
        }

        return answer;
    }
};

int main() {
    Solution solution;
    vector<int> nums = {1, 2, 3, 4};
    vector<int> result = solution.productExceptSelf(nums);
    cout << "[";
    for (const auto& res : result) {
        cout << res;
        if (&res != &result.back()) {
            cout << ", ";
        }
    }
    cout << "]" << endl;
}