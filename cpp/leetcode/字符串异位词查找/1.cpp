#include <iostream>
#include <vector>
#include <string>

using namespace std;

class Solution {
public:
    vector<int> findAnagrams(string s, string p) {
        int sLen = s.length(), pLen = p.length();
        if (sLen < pLen) {
            return {};
        }

        vector<int> sCount(26, 0);
        vector<int> pCount(26, 0);
        vector<int> ans;

        for (int i = 0; i < pLen; ++i) {
            pCount[p[i] - 'a']++;
            sCount[s[i] - 'a']++;
        }

        if (pCount == sCount) {
            ans.push_back(0);
        }

        for (int i = 0; i < sLen - pLen; ++i) {
            sCount[s[i] - 'a']--;
            sCount[s[i + pLen] - 'a']++;

            if (pCount == sCount) {
                ans.push_back(i + 1);
            }
        }
        return ans;
    }
};

int main() {
    string s = "cbaebabacd";
    string p = "abc";
    Solution solution;
    vector<int> result = solution.findAnagrams(s, p);
    cout << "[";
    for (const auto& i : result) {
        cout << i ;
        if (i < result.size() - 1) {
            cout << ", ";
        }
    }
    cout << "]" << endl;
}