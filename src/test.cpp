#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>

using namespace std;

// --- 1. 核心結構實作 ---

struct Road {
    int to;            
    float dist;        
    float avg_angle;   // 轉向角度 (度)
    float frequency;   // 每公里轉彎次數

    // 計算北宜基準下的 Curvature
    float getCurvature() const {
        const float BEI_YI_BENCHMARK = 1000.0f;
        float val = (avg_angle * frequency) / BEI_YI_BENCHMARK;
        return (val > 1.0f) ? 1.0f : val;
    }

    // 感官權重公式：實作非線性累積感
    float getWeight(float sensitivity) const {
        const float beta = 1.0f;
        float curv = getCurvature();
        float base_burden = dist * curv;
        // 累積性懲罰 (1.5次方)
        float penalty = pow(base_burden, 1.5f) * sensitivity * beta;
        return dist + penalty;
    }
};

struct Station {
    int id;
    string name;
};

struct Node {
    int id;
    float current_weight;
    bool operator>(const Node& other) const {
        return current_weight > other.current_weight;
    }
};

// --- 2. 路網類別整合 ---

class MountainMap {
public:
    vector<Station> stations;
    vector<vector<Road>> adj;

    void addStation(string name) {
        int id = (int)stations.size();
        stations.push_back({id, name});
        adj.push_back(vector<Road>());
    }

    // 修改 addRoad 以符合新的曲率定義
    void addRoad(int from, int to, float dist, float angle, float freq) {
        adj[from].push_back({to, dist, angle, freq});
    }

    // 路徑回溯
    vector<int> reconstructPath(const vector<int>& parent, int end) {
        vector<int> path;
        for (int at = end; at != -1; at = parent[at]) {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        return path;
    }

    // Dijkstra 演算法實作
    void findBestPath(int start, int end, float sensitivity) {
        int n = adj.size();
        if (start >= n || end >= n) {
            cout << "錯誤：站點 ID 不存在。" << endl;
            return;
        }

        vector<float> min_weight(n, numeric_limits<float>::infinity());
        vector<int> parent(n, -1);
        priority_queue<Node, vector<Node>, greater<Node>> pq;

        min_weight[start] = 0;
        pq.push({start, 0});

        while (!pq.empty()) {
            Node top = pq.top();
            pq.pop();

            int u = top.id;
            if (top.current_weight > min_weight[u]) continue;
            if (u == end) break;

            for (const auto& edge : adj[u]) {
                float weight = edge.getWeight(sensitivity);
                if (min_weight[u] + weight < min_weight[edge.to]) {
                    min_weight[edge.to] = min_weight[u] + weight;
                    parent[edge.to] = u;
                    pq.push({edge.to, min_weight[edge.to]});
                }
            }
        }

        // 輸出結果
        if (min_weight[end] == numeric_limits<float>::infinity()) {
            cout << "抱歉，無法找到可到達的路徑。" << endl;
        } else {
            vector<int> path = reconstructPath(parent, end);
            cout << "\n規劃完成！建議路徑總體感負擔分數: " << min_weight[end] << endl;
            cout << "最佳路徑: ";
            for (int i = 0; i < path.size(); i++) {
                cout << stations[path[i]].name << (i == path.size() - 1 ? "" : " -> ");
            }
            cout << endl;
        }
    }

    void printMap() {
        cout << "\n--- 現有路網預覽 ---" << endl;
        for (int i = 0; i < (int)stations.size(); i++) {
            cout << "[" << i << "] " << stations[i].name << " -> ";
            if (adj[i].empty()) cout << "無接駁路徑";
            else {
                for (auto &r : adj[i]) {
                    cout << stations[r.to].name << "(D:" << r.dist << "km/C:" << r.getCurvature() << ") ";
                }
            }
            cout << endl;
        }
    }
};

// --- 3. UI 與主程式 ---

void showUI(MountainMap& myMap) {
    cout << "====================================" << endl;
    cout << "   山區小巴導航系統 - 減暈優化版      " << endl;
    cout << "====================================" << endl;

    myMap.printMap();
    cout << "------------------------------------" << endl;

    int p_count;
    cout << "請輸入乘客人數: "; cin >> p_count;

    float max_s = 0, sum_s = 0;
    for (int i = 0; i < p_count; i++) {
        float s;
        cout << "乘客 " << i + 1 << " 的敏感度 (0.1-1.0): "; cin >> s;
        if (s > max_s) max_s = s;
        sum_s += s;
    }

    // 群體決策邏輯：70% 照顧最暈的人，30% 參考平均值
    float group_sensitivity = 0.7f * max_s + 0.3f * (sum_s / p_count);
    cout << "\n系統判定群體綜合敏感度指標為: " << group_sensitivity << endl;

    int start, end;
    cout << "請輸入起點站 ID: "; cin >> start;
    cout << "請輸入終點站 ID: "; cin >> end;

    myMap.findBestPath(start, end, group_sensitivity);
}

int main() {
    MountainMap myMap;

    // 1. 建立站點
    myMap.addStation("轉運站A");
    myMap.addStation("熱門景點B");
    myMap.addStation("山區秘境C");

    // 2. 建立路段 (起點, 終點, 距離, 平均轉向角, 轉彎頻率)
    // A -> B: 15km, 很平 (20度, 每km 1次)
    myMap.addRoad(0, 1, 15.0, 20.0, 1.0); 
    // A -> C: 5km, 極彎 (150度, 每km 6次)
    myMap.addRoad(0, 2, 5.0, 150.0, 6.0);  
    // C -> B: 4km, 頗彎 (100度, 每km 5次)
    myMap.addRoad(2, 1, 4.0, 100.0, 5.0);  

    showUI(myMap);

    cout << "\n--------------------" << endl;
    system("pause");
    return 0;
}
