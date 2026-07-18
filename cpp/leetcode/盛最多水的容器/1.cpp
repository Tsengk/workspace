#include <iostream>
#include <vector>
#include <algorithm>

class Solution {
public:
    int maxArea(std::vector<int>& height) {
        int left = 0;
        int right = height.size() - 1;
        int max_area = 0;

        while(left < right) {
            int current_area = std::min(height[left], height[right]) * (right - left);
            max_area = std::max(max_area, current_area);
            if(height[left] < height[right]) {
                left++;
            } else {
                right--;
            }
        }
        return max_area;
    }
};

int main() {
    Solution solution;
    std::vector<int> height = {1, 8, 6, 2, 5, 4, 8, 3, 7};
    int result = solution.maxArea(height);
    std::cout << "Max area: " << result << std::endl;
    return 0;
}