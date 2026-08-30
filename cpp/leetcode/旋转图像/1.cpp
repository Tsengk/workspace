#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

class Soltion {
public:
    void rotate(vector<vector<int>>& matrix) {
        int n = matrix.size();

        for (int i = 0; i < n; ++i){
            for (int j = i + 1; j < n; ++j){
                swap(matrix[i][j], matrix[j][i]);
            }
        }

        for (int i = 0; i < n; ++i){
            reverse(matrix[i].begin(), matrix[i].end());
        }
    }
};

int main() {
    Soltion soluton;
    vector<vector<int>> matrix = {{5, 1, 9, 11}, {2, 4, 8, 10}, {13, 3, 6, 7}, {15, 14, 12, 16}};
    soluton.rotate(matrix);
    cout << "[";
    for (const auto& nums : matrix) {
        cout << "[";
        for (const auto& num : nums) {
            cout << num;
            if (&num != &nums.back()) {
                cout << ", ";
            }
        }
        cout << "]";
        if (&nums != &matrix.back()) {
            cout << ", ";
        }
    }
    cout << "]" << endl;
}