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
    // 解決中文亂碼問題
    // system("chcp 65001");

    MountainMap myMap;

    // 1. 建立具備規模感的站點 (0-6)
    myMap.addStation("A0 城市轉運站");      // 起點
    myMap.addStation("B1 觀光老街");        // 中間站
    myMap.addStation("B2 雲海咖啡廳");      // 中間站
    myMap.addStation("C1 森林遊樂區");      // 靠近終點
    myMap.addStation("C2 陡坡觀景台");      // 極陡路徑點
    myMap.addStation("D0 終點秘境湖泊");    // 終點
    myMap.addStation("X1 繞遠路大橋");      // 用於展示平緩但遙遠的路徑

    // 2. 建立具備決策衝突的路段 (from, to, dist, avg_angle, freq)
    
    // --- 路徑 A: 經典捷徑 (距離短但極度彎曲，模擬九彎十八拐) ---
    myMap.addRoad(0, 2, 8.0, 140.0, 7.5);  // A0 -> B2: 近但極彎
    myMap.addRoad(2, 4, 4.0, 155.0, 8.0);  // B2 -> C2: 持續極彎
    myMap.addRoad(4, 5, 3.0, 130.0, 6.0);  // C2 -> D0: 最後衝刺

    // --- 路徑 B: 中庸之道 (距離適中，彎度一般) ---
    myMap.addRoad(0, 1, 10.0, 45.0, 2.0);  // A0 -> B1: 稍遠但平緩
    myMap.addRoad(1, 3, 7.0, 60.0, 3.0);   // B1 -> C1: 中等彎度
    myMap.addRoad(3, 5, 6.0, 55.0, 2.5);   // C1 -> D0: 穩定抵達

    // --- 路徑 C: 極致平緩繞遠路 (給超級敏感乘客的備案) ---
    myMap.addRoad(0, 6, 18.0, 15.0, 0.5);  // A0 -> X1: 非常遠但幾乎是直線
    myMap.addRoad(6, 3, 12.0, 20.0, 0.8);  // X1 -> C1: 保持平穩
    
    // 補足一些連接支線增加搜尋深度
    myMap.addRoad(1, 2, 5.0, 90.0, 4.0);   // B1 -> B2
    myMap.addRoad(3, 4, 3.0, 100.0, 5.0);  // C1 -> C2

    // 3. 執行 UI 介面
    showUI(myMap);

    cout << "\n--------------------" << endl;
    cout << "專案展示完成，請存檔後提交 GitHub。" << endl;
    system("pause");
    return 0;
}
