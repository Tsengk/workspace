#include <iostream>
#include <vector>

using namespace std;

class Solution {
public:
    vector<int> spiralOrder(vector<vector<int>>& matrix) {
        if (matrix.empty() || matrix[0].empty()) return {};

        int top = 0, bottom = matrix.size() - 1;
        int left = 0, right = matrix[0].size() - 1;
        vector<int> result;

        while (top <= bottom && left <= right) {
            for (int j = left; j <= right; ++j) {
                result.push_back(matrix[top][j]);
            }
            ++top;
            for (int i = top; i <= bottom; ++i) {
                result.push_back(matrix[i][right]);
            }
            --right;
            if (top <= bottom) {
                for (int j = right; j >= left; --j) {
                    result.push_back(matrix[bottom][j]);
                }
                --bottom;
            }
            if (left <= right) {
                for (int i = bottom; i >= top; --i) {
                    result.push_back(matrix[i][left]);
                }
                ++left;
            }
        }
        return result;
    }
};

int main() {
    Solution solution;
    vector<vector<int>> matrix = {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}};
    vector<int> result = solution.spiralOrder(matrix);
    cout << "[";
    for (const int& n : result) {
        cout << n;
        if (&n != &result.back()) {
            cout << ", ";
        }
    }
    cout << "]" << endl;
}