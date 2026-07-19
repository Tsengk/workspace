#include <iostream>
#include <string>
#include <unordered_set>
#include <algorithm>

using namespace std;

class Solution {
public:
    int lengthOfLongestSubstring(string& s) {
        unordered_set<char> charSet;
        int left = 0;
        int max_length = 0;

        for (int right = 0; right < s.length(); ++right) {
            while (charSet.find(s[right]) != charSet.end()) {
                charSet.erase(s[left]);
                ++left;
            }
            charSet.insert(s[right]);
            max_length = max(max_length, right - left + 1);
        }
        return max_length;
    }
};

int main() {
    Solution solution;
    string s = "pwwkew";
    cout << solution.lengthOfLongestSubstring(s) << endl;
}