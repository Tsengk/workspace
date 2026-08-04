#include <iostream>
#include <string>
#include <vector>
using namespace std;

class Solution {
public:
    string minWindow(string s, string t) {
        int m = s.length(), n = t.length();
        if (m < n) return "";

        vector<int> need(128, 0);
        vector<int> window(128, 0);

        int need_type_count = 0;
        for (char c : t) {
            if (need[c] == 0) need_type_count++;
            need[c]++;
        }

        int left = 0, right = 0;
        int valid = 0;
        int start = 0, min_len = m + 1;

        while (right < m) {
            char c = s[right];
            right++;
            if (need[c] > 0) {
                window[c]++;
                if (window[c] == need[c]) {
                    valid++;
                }
            }

            while (valid == need_type_count) {
                if (right - left < min_len) {
                    start = left;
                    min_len = right - left;
                }
                char d = s[left];
                left++;
                if (need[d] > 0) {
                    if (window[d] == need[d]) {
                        valid--;
                    }
                    window[d]--;
                }
            }
        }
        return min_len == m + 1 ? "" : s.substr(start, min_len);
    }
};

int main() {
    Solution solution;
    string s = "ADOBECODEBANC";
    string t = "AABC";
    string result = solution.minWindow(s, t);
    cout << "Minimum window substring: " << result << endl;
    return 0;
}