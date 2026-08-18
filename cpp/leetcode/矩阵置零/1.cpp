#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class Solution {
public:
    void setZeroes(vector<vector<int>>& matrix) {
        int m = matrix.size();
        int n = matrix[0].size();

        bool row0 = false;
        bool col0 = false;

        for (int i = 0; i < m; ++i) {
            if (matrix[i][0] == 0) {
                col0 = true;
                break;
            }
        }
        for (int j = 0; j < n; ++j) {
            if (matrix[0][j] == 0) {
                row0 = true;
                break;
            }
        }

        for (int i = 1; i < m; ++i) {
            for (int j = 1; j < n; ++j) {
                if (matrix[i][j] == 0) {
                    matrix[i][0] = 0;
                    matrix[0][j] = 0;
                }
            }
        }
        for (int i = 1; i < m; ++i) {
            for (int j = 1; j < n; ++j) {
                if (matrix[i][0] == 0 || matrix[0][j] == 0) {
                    matrix[i][j] = 0;
                }
            }
        }
        if (row0) {
            for (int j = 0; j < n; ++j) {
                matrix[0][j] = 0;
            }
        }
        if (col0) {
            for (int i = 0; i < m; ++i) {
                matrix[i][0] = 0;
            }
        }
    }
};

int main() {
    Solution solution;
    vector<vector<int>> matrix = {{0, 1, 2, 0}, {3, 4, 5, 2}, {1, 3, 1, 5}};
    solution.setZeroes(matrix);
    cout << "[";
    for (const vector<int>& nums : matrix) {
        cout << "[";
        for (int i = 0; i < nums.size(); ++i) {
            cout << nums[i];
            if (i < nums.size() - 1) {
                cout << ", ";
            }
        }
        cout << "]";
        if (nums != matrix.back()) {
            cout << ", ";
        }
    }
    cout << "]" << endl;
}