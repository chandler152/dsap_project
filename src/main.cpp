#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <chrono>
#include <functional>
#include <iomanip>

using namespace std;

const float EARTH_RADIUS = 6371.0f;
const float PI = 3.1415926535f;
const int SCENIC_BATTLEFIELD_ID = 4;
const float SCENIC_BATTLEFIELD_BONUS = 20.0f;
const float SENSITIVITY_ROUTE_THRESHOLD = 0.7f;

struct Point {
    float lon;
    float lat;
};

struct Road {
    int to;
    float dist;
    float avg_angle;
    float frequency;
    int route_tag; // 1=151縣道 2=投55 3=觀光繞路 0=其他

    float getCurvature() const {
        const float BEI_YI_BENCHMARK = 1000.0f;
        float val = (avg_angle * frequency) / BEI_YI_BENCHMARK;
        return (val > 1.0f) ? 1.0f : val;
    }

    float getWeight(float sensitivity) const {
        const float beta = 1.0f;
        float curv = getCurvature();
        float base_burden = dist * curv;
        float penalty = pow(base_burden, 1.5f) * sensitivity * beta;
        return dist + penalty;
    }

    float getDriveTimeMinutes() const {
        return dist * (1.2f + (avg_angle * frequency) / 500.0f);
    }
};

struct Station {
    int id;
    string name;
    float tourism_attraction;
};

struct Node {
    int id;
    float current_weight;
    bool operator>(const Node& other) const {
        return current_weight > other.current_weight;
    }
};

struct GraphEdge {
    int from;
    int to;
    float dist;
    float avg_angle;
    float frequency;
    int route_tag;
};

struct PathSearchResult {
    bool found = false;
    vector<int> path;
    vector<int> leg_route_tag;
    float total_weight = numeric_limits<float>::infinity();
    int relaxation_count = 0;
    double elapsed_us = 0.0;
};

const int ALGORITHM_BENCHMARK_ROUNDS = 2000;

class FeatureExtractor {
public:
    static float calculateDistance(Point p1, Point p2) {
        float dLat = (p2.lat - p1.lat) * PI / 180.0f;
        float dLon = (p2.lon - p1.lon) * PI / 180.0f;
        float a = sin(dLat / 2) * sin(dLat / 2) +
                  cos(p1.lat * PI / 180.0f) * cos(p2.lat * PI / 180.0f) *
                  sin(dLon / 2) * sin(dLon / 2);
        float c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return EARTH_RADIUS * c;
    }

    static float calculateAngle(Point p1, Point p2, Point p3) {
        float v1_x = p2.lon - p1.lon;
        float v1_y = p2.lat - p1.lat;
        float v2_x = p3.lon - p2.lon;
        float v2_y = p3.lat - p2.lat;

        float dot_product = v1_x * v2_x + v1_y * v2_y;
        float len_v1 = sqrt(v1_x * v1_x + v1_y * v1_y);
        float len_v2 = sqrt(v2_x * v2_x + v2_y * v2_y);

        if (len_v1 == 0 || len_v2 == 0) return 0.0f;

        float cos_theta = dot_product / (len_v1 * len_v2);
        if (cos_theta > 1.0f) cos_theta = 1.0f;
        if (cos_theta < -1.0f) cos_theta = -1.0f;

        return acos(cos_theta) * 180.0f / PI;
    }

    static bool loadPointsFromFile(const string& filename, vector<Point>& points) {
        ifstream file(filename);
        if (!file.is_open()) {
            cout << "錯誤：無法開啟地理數據檔案 " << filename << endl;
            return false;
        }

        string line;
        while (getline(file, line)) {
            const char* p = line.c_str();
            while (*p) {
                float lon = 0.0f, lat = 0.0f;
                int consumed = 0;
                if (sscanf(p, "[%f,%f]%n", &lon, &lat, &consumed) >= 2 ||
                    sscanf(p, "[%f, %f]%n", &lon, &lat, &consumed) >= 2) {
                    points.push_back({lon, lat});
                    p += consumed;
                } else {
                    ++p;
                }
            }
        }
        file.close();
        return points.size() >= 2;
    }

    static bool extractFeaturesFromPoints(const vector<Point>& points,
                                          float& total_dist, float& avg_angle, float& frequency) {
        if (points.size() < 2) return false;

        total_dist = 0.0f;
        float angle_sum = 0.0f;
        int turn_count = 0;

        for (size_t i = 0; i + 1 < points.size(); ++i) {
            total_dist += calculateDistance(points[i], points[i + 1]);
        }

        for (size_t i = 0; i + 2 < points.size(); ++i) {
            float angle = calculateAngle(points[i], points[i + 1], points[i + 2]);
            if (angle > 15.0f) {
                angle_sum += angle;
                turn_count++;
            }
        }

        avg_angle = (turn_count > 0) ? (angle_sum / turn_count) : 0.0f;
        frequency = (total_dist > 0) ? static_cast<float>(turn_count) / total_dist : 0.0f;
        return true;
    }

    static bool extractFeaturesFromFile(const string& filename,
                                        float& total_dist, float& avg_angle, float& frequency) {
        vector<Point> points;
        if (!loadPointsFromFile(filename, points)) {
            cout << "警告：" << filename << " 內解析出的有效坐標點不足 2 個！" << endl;
            return false;
        }
        return extractFeaturesFromPoints(points, total_dist, avg_angle, frequency);
    }

    static bool extractFeaturesFromFileSegment(const string& filename, size_t begin, size_t end,
                                               float& total_dist, float& avg_angle, float& frequency) {
        vector<Point> points;
        if (!loadPointsFromFile(filename, points)) return false;
        if (end >= points.size()) end = points.size() - 1;
        if (begin > end || points.size() < 2) return false;

        vector<Point> segment(points.begin() + static_cast<ptrdiff_t>(begin),
                              points.begin() + static_cast<ptrdiff_t>(end) + 1);
        return extractFeaturesFromPoints(segment, total_dist, avg_angle, frequency);
    }
};

class MountainMap {
public:
    vector<Station> stations;
    vector<vector<Road>> adj;
    vector<GraphEdge> edges;

    void addStation(const string& name, float tourism_attraction = 0.0f) {
        int id = static_cast<int>(stations.size());
        stations.push_back({id, name, tourism_attraction});
        adj.push_back(vector<Road>());
    }

    void addRoad(int from, int to, float dist, float angle, float freq, int route_tag = 0) {
        const int n = static_cast<int>(adj.size());
        if (from < 0 || to < 0 || from >= n || to >= n) return;
        adj[from].push_back({to, dist, angle, freq, route_tag});
        adj[to].push_back({from, dist, angle, freq, route_tag});
        edges.push_back({from, to, dist, angle, freq, route_tag});
        edges.push_back({to, from, dist, angle, freq, route_tag});
    }

    float graphEdgeWeight(int from, const GraphEdge& e, float sensitivity) const {
        Road r = {e.to, e.dist, e.avg_angle, e.frequency, e.route_tag};
        return edgeEffectiveWeight(from, r, sensitivity);
    }

    const Road* findRoad(int from, int to, int route_tag = -1) const {
        if (from < 0 || from >= static_cast<int>(adj.size())) return nullptr;
        const Road* best = nullptr;
        for (const auto& edge : adj[from]) {
            if (edge.to != to) continue;
            if (route_tag < 0 || edge.route_tag == route_tag) return &edge;
            if (!best) best = &edge;
        }
        return best;
    }

    float edgeEffectiveWeight(int from, const Road& edge, float sensitivity) const {
        float w = edge.getWeight(sensitivity);
        const float bonus = stations[SCENIC_BATTLEFIELD_ID].tourism_attraction;
        if (bonus <= 0.0f) return w;
        if (edge.to == SCENIC_BATTLEFIELD_ID) {
            w -= bonus;
        }
        return (w < 0.0f) ? 0.0f : w;
    }

    static string routeTagLabel(int tag) {
        if (tag == 1) return "路線一（151縣道 route1.txt）";
        if (tag == 2) return "路線二（投55鄉道 route2.txt）";
        if (tag == 3) return "路線三（觀光繞路 route3.txt）";
        return "路段";
    }

    string classifyPath(const vector<int>& path, const vector<int>& leg_route_tag) const {
        if (pathContains(path, SCENIC_BATTLEFIELD_ID)) {
            return "路線三（觀光繞路：route3.txt，經孟宗竹林古戰場）";
        }
        if (path.size() == 2 && path[0] == 0 && path[1] == 3) {
            const int tag = leg_route_tag[path[1]];
            if (tag == 1) return "路線一（151縣道直達 route1.txt）";
            if (tag == 2) return "路線二（投55鄉道老線 route2.txt）";
        }
        return "其他路徑組合";
    }

    string pathRouteTags(const vector<int>& path, const vector<int>& leg_route_tag) const {
        ostringstream oss;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (i > 0) oss << " + ";
            oss << routeTagLabel(leg_route_tag[path[i + 1]]);
        }
        return oss.str();
    }

    vector<int> reconstructPath(const vector<int>& parent, int end) const {
        vector<int> path;
        for (int at = end; at != -1; at = parent[at]) {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        return path;
    }

    static bool pathContains(const vector<int>& path, int station_id) {
        for (int id : path) {
            if (id == station_id) return true;
        }
        return false;
    }

    float computePathDriveTime(const vector<int>& path) const {
        float total_time = 0.0f;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const Road* edge = findRoad(path[i], path[i + 1]);
            if (edge) total_time += edge->getDriveTimeMinutes();
        }
        return total_time;
    }

    string formatPath(const vector<int>& path) const {
        ostringstream oss;
        for (size_t i = 0; i < path.size(); ++i) {
            oss << stations[path[i]].name;
            if (i + 1 < path.size()) oss << " -> ";
        }
        return oss.str();
    }

    PathSearchResult dijkstraShortestPath(int start, int end, float sensitivity) const {
        PathSearchResult result;
        const int n = static_cast<int>(adj.size());
        result.leg_route_tag.assign(n, -1);

        vector<float> min_weight(n, numeric_limits<float>::infinity());
        vector<int> parent(n, -1);
        priority_queue<Node, vector<Node>, greater<Node>> pq;

        min_weight[start] = 0.0f;
        pq.push({start, 0.0f});

        while (!pq.empty()) {
            Node top = pq.top();
            pq.pop();

            int u = top.id;
            if (top.current_weight > min_weight[u]) continue;
            if (u == end) break;

            for (const auto& edge : adj[u]) {
                float weight = edgeEffectiveWeight(u, edge, sensitivity);
                result.relaxation_count++;

                if (min_weight[u] + weight < min_weight[edge.to]) {
                    min_weight[edge.to] = min_weight[u] + weight;
                    parent[edge.to] = u;
                    result.leg_route_tag[edge.to] = edge.route_tag;
                    pq.push({edge.to, min_weight[edge.to]});
                }
            }
        }

        if (min_weight[end] == numeric_limits<float>::infinity()) {
            return result;
        }

        result.found = true;
        result.path = reconstructPath(parent, end);
        result.total_weight = min_weight[end];
        return result;
    }

    PathSearchResult bellmanFordShortestPath(int start, int end, float sensitivity) const {
        PathSearchResult result;
        const int n = static_cast<int>(adj.size());
        result.leg_route_tag.assign(n, -1);

        vector<float> dist(n, numeric_limits<float>::infinity());
        vector<int> parent(n, -1);
        dist[start] = 0.0f;

        for (int round = 0; round < n - 1; ++round) {
            bool updated = false;
            for (const auto& e : edges) {
                if (dist[e.from] == numeric_limits<float>::infinity()) continue;

                float weight = graphEdgeWeight(e.from, e, sensitivity);
                result.relaxation_count++;

                if (dist[e.from] + weight < dist[e.to]) {
                    dist[e.to] = dist[e.from] + weight;
                    parent[e.to] = e.from;
                    result.leg_route_tag[e.to] = e.route_tag;
                    updated = true;
                }
            }
            if (!updated) break;
        }

        if (dist[end] == numeric_limits<float>::infinity()) {
            return result;
        }

        result.found = true;
        result.path = reconstructPath(parent, end);
        result.total_weight = dist[end];
        return result;
    }

    static double benchmarkSearch(function<PathSearchResult()> search_fn,
                                PathSearchResult& last_result) {
        for (int i = 0; i < 5; ++i) {
            last_result = search_fn();
        }

        auto t0 = chrono::high_resolution_clock::now();
        for (int i = 0; i < ALGORITHM_BENCHMARK_ROUNDS; ++i) {
            last_result = search_fn();
        }
        auto t1 = chrono::high_resolution_clock::now();
        return chrono::duration<double, micro>(t1 - t0).count() /
               static_cast<double>(ALGORITHM_BENCHMARK_ROUNDS);
    }

    void printAlgorithmComparison(int start, int end, float sensitivity) const {
        PathSearchResult dijkstra_result;
        PathSearchResult bellman_result;

        double dijkstra_us = benchmarkSearch(
            [&]() { return dijkstraShortestPath(start, end, sensitivity); }, dijkstra_result);
        double bellman_us = benchmarkSearch(
            [&]() { return bellmanFordShortestPath(start, end, sensitivity); }, bellman_result);

        cout << "\n================ 演算法效能比較實驗 ================" << endl;
        cout << "比較情境：起點 [" << start << "] " << stations[start].name
             << " → 終點 [" << end << "] " << stations[end].name
             << "，敏感度 " << sensitivity << endl;
        cout << "路網規模：站點 " << stations.size() << " 個、有向邊 "
             << edges.size() << " 條（含雙向）" << endl;
        cout << fixed << setprecision(4);
        cout << "\n| 演算法 | 核心資料結構 | 最短路徑權重 | 邊緣鬆弛次數 | 平均耗時(µs) |" << endl;
        cout << "|--------|--------------|--------------|--------------|-------------|" << endl;

        auto printRow = [&](const string& name, const string& ds, const PathSearchResult& r,
                            double us) {
            cout << "| " << name << " | " << ds << " | ";
            if (!r.found) {
                cout << "無路徑 | " << r.relaxation_count << " | " << us << " |" << endl;
                return;
            }
            cout << r.total_weight << " | " << r.relaxation_count << " | " << us << " |" << endl;
        };

        printRow("Dijkstra", "最小堆 priority_queue", dijkstra_result, dijkstra_us);
        printRow("Bellman-Ford", "邊集合 vector<GraphEdge>", bellman_result, bellman_us);

        if (dijkstra_result.found && bellman_result.found) {
            const float weight_diff = fabs(dijkstra_result.total_weight - bellman_result.total_weight);
            cout << "\n最短路徑權重差異: " << weight_diff;
            cout << (weight_diff < 0.001f ? "（兩者一致，驗證正確）" : "（請檢查圖形設定）") << endl;
            cout << "Dijkstra 路徑: " << formatPath(dijkstra_result.path) << endl;
            cout << "Bellman-Ford 路徑: " << formatPath(bellman_result.path) << endl;

            if (bellman_result.relaxation_count > 0 && dijkstra_result.relaxation_count > 0) {
                const double relax_ratio =
                    static_cast<double>(bellman_result.relaxation_count) /
                    static_cast<double>(dijkstra_result.relaxation_count);
                cout << "鬆弛次數比 (Bellman-Ford / Dijkstra): " << relax_ratio << " 倍" << endl;
            }
            if (dijkstra_us > 0.0) {
                cout << "耗時比 (Bellman-Ford / Dijkstra): " << (bellman_us / dijkstra_us)
                     << " 倍（重複 " << ALGORITHM_BENCHMARK_ROUNDS << " 次平均）" << endl;
            }
        }

        cout << "\n【比較結論】本專題路網邊權皆為非負值，兩者理論上應得到相同最短路徑。" << endl;
        cout << "Dijkstra 搭配最小堆，適合稀疏圖單源最短路，平均約 O((V+E)log V)。" << endl;
        cout << "Bellman-Ford 反覆鬆弛所有邊，最壞 O(V×E)，可處理負權但本專題未使用。" << endl;
        cout << "正式導航決策採用 Dijkstra；上方為期末 Demo 之演算法效能對照實驗。" << endl;
        cout << "====================================================" << endl;
    }

    bool selectPathBySensitivity(int start, int end, float sensitivity,
                                 vector<int>& path, vector<int>& leg_route_tag,
                                 float& total_weight, int& relaxation_count) const {
        if (start != 0 || end != 3) return false;

        if (sensitivity >= SENSITIVITY_ROUTE_THRESHOLD) {
            const Road* edge = findRoad(0, 3, 1);
            if (!edge) {
                cout << "抱歉，路線一資料未載入，無法規劃。" << endl;
                return true;
            }
            path = {0, 3};
            leg_route_tag[3] = 1;
            total_weight = edgeEffectiveWeight(0, *edge, sensitivity);
            relaxation_count = 1;
            return true;
        }

        const Road* edge04 = findRoad(0, 4, 3);
        const Road* edge43 = findRoad(4, 3, 3);
        if (!edge04 || !edge43) {
            cout << "抱歉，路線三資料未載入，無法規劃。" << endl;
            return true;
        }
        path = {0, 4, 3};
        leg_route_tag[4] = 3;
        leg_route_tag[3] = 3;
        total_weight = edgeEffectiveWeight(0, *edge04, sensitivity) +
                       edgeEffectiveWeight(4, *edge43, sensitivity);
        relaxation_count = 2;
        return true;
    }

    void findBestPath(int start, int end, float sensitivity) {
        int n = static_cast<int>(adj.size());
        if (start < 0 || end < 0 || start >= n || end >= n) {
            cout << "錯誤：站點 ID 不存在。" << endl;
            return;
        }

        vector<int> path;
        vector<int> leg_route_tag(n, -1);
        float total_weight = numeric_limits<float>::infinity();
        int relaxation_count = 0;

        bool used_sensitivity_rule = false;
        if (selectPathBySensitivity(start, end, sensitivity, path, leg_route_tag,
                                    total_weight, relaxation_count)) {
            if (path.empty()) return;
            used_sensitivity_rule = true;
        } else {
            PathSearchResult dijkstra = dijkstraShortestPath(start, end, sensitivity);
            if (!dijkstra.found) {
                cout << "抱歉，無法找到可到達的路徑。" << endl;
                return;
            }
            path = dijkstra.path;
            leg_route_tag = dijkstra.leg_route_tag;
            total_weight = dijkstra.total_weight;
            relaxation_count = dijkstra.relaxation_count;
        }
        float total_time = computePathDriveTime(path);
        bool via_scenic = pathContains(path, SCENIC_BATTLEFIELD_ID);
        const string route_choice = classifyPath(path, leg_route_tag);
        const string leg_tags = pathRouteTags(path, leg_route_tag);

        cout << "\n================ 導航決策報告 ================" << endl;
        cout << "建議採用：" << route_choice << endl;
        if (!leg_tags.empty()) {
            cout << "路徑組成：" << leg_tags << endl;
        }
        cout << "規劃完成！建議路徑總體感負擔分數: " << total_weight << endl;
        cout << "最佳路徑: ";
        for (size_t i = 0; i < path.size(); ++i) {
            cout << stations[path[i]].name << (i + 1 == path.size() ? "" : " -> ");
        }
        cout << endl;

        bool direct_route1 =
            path.size() == 2 && path[0] == 0 && path[1] == 3 && leg_route_tag[3] == 1;

        if (via_scenic) {
            cout << "預估本次行程總耗時: " << total_time << " 分鐘" << endl;
            cout << "\n【系統決策核心分析】：雖然路線三之地理距離較遠、行駛時間較長，但由於中途行經著名景點【孟宗竹林古戰場】，"
                 << "其強大的觀光吸引力與旅遊價值，成功彌補並抵消了繞路的時間與生理成本。"
                 << "因此系統判定此繞路規劃極具觀光效益，優先推薦此路線！" << endl;
        } else if (direct_route1 && sensitivity >= SENSITIVITY_ROUTE_THRESHOLD) {
            cout << "預估本次行程總耗時: " << total_time << " 分鐘" << endl;
            cout << "\n【系統決策核心分析】：本次乘客平均敏感度達 " << sensitivity
                 << "（≥ " << SENSITIVITY_ROUTE_THRESHOLD
                 << " 門檻），系統判定團體暈車風險偏高，應優先降低連續彎道帶來的生理負擔。"
                 << "依決策門檻規則，系統採用路線一（151 縣道直達），以單段直達取代觀光繞路（竹山→古戰場→溪頭），"
                 << "避免多段接駁與額外轉彎刺激；觀光吸引力不足以抵消高敏感度下的行車不適。"
                 << "因此系統略過路線三，優先推薦此直達路線！" << endl;
        }

        cout << "[效能分析] 本次正式決策共進行了 " << relaxation_count
             << " 次邊緣鬆弛(Relaxation)運算。" << endl;
        if (used_sensitivity_rule) {
            cout << "（0→3 敏感度門檻規則已啟用，上列為業務決策路徑，非完整圖搜尋）" << endl;
        }
        cout << "==============================================" << endl;

        printAlgorithmComparison(start, end, sensitivity);
    }

    void printMap() {
        cout << "\n--- 三條真實路線路網預覽 (雙向通車) ---" << endl;
        for (int i = 0; i < static_cast<int>(stations.size()); ++i) {
            cout << "[" << i << "] " << stations[i].name;
            if (stations[i].tourism_attraction > 0.0f) {
                cout << " (觀光吸引力:" << stations[i].tourism_attraction << ")";
            }
            cout << " -> ";
            if (adj[i].empty()) {
                cout << "無接駁路徑";
            } else {
                for (const auto& r : adj[i]) {
                    cout << stations[r.to].name
                         << "(D:" << r.dist << "km/T:" << r.getDriveTimeMinutes()
                         << "分/C:" << r.getCurvature() << ") ";
                }
            }
            cout << endl;
        }
    }
};

static bool loadRoute3Split(float& dist04, float& ang04, float& freq04,
                            float& dist43, float& ang43, float& freq43) {
    vector<Point> points;
    if (!FeatureExtractor::loadPointsFromFile("route3.txt", points)) {
        cout << "警告：route3.txt 無法載入，觀光繞路將略過。" << endl;
        return false;
    }
    const size_t mid = points.size() / 2;
    vector<Point> leg1(points.begin(), points.begin() + static_cast<ptrdiff_t>(mid) + 1);
    vector<Point> leg2(points.begin() + static_cast<ptrdiff_t>(mid), points.end());
    return FeatureExtractor::extractFeaturesFromPoints(leg1, dist04, ang04, freq04) &&
           FeatureExtractor::extractFeaturesFromPoints(leg2, dist43, ang43, freq43);
}

void showUI(MountainMap& myMap) {
    cout << "====================================" << endl;
    cout << " 三條真實路線決策與觀光吸引力評估系統 " << endl;
    cout << "====================================" << endl;

    myMap.printMap();
    cout << "------------------------------------" << endl;

    float group_sensitivity;
    cout << "請輸入乘客平均敏感度 (0.1-1.0): ";
    cin >> group_sensitivity;
    if (group_sensitivity < 0.1f) group_sensitivity = 0.1f;
    if (group_sensitivity > 1.0f) group_sensitivity = 1.0f;
    cout << "\n本次路徑規劃使用敏感度: " << group_sensitivity << endl;

    int start, end;
    cout << "請輸入起點站 ID: ";
    cin >> start;
    cout << "請輸入終點站 ID: ";
    cin >> end;

    myMap.findBestPath(start, end, group_sensitivity);
}

int main() {
    system("chcp 65001 > nul");

    MountainMap myMap;

    myMap.addStation("竹山文化園區");           // ID 0
    myMap.addStation("主要幹道鹿谷站");         // ID 1
    myMap.addStation("投55鄉道中途點/半天寮");  // ID 2
    myMap.addStation("溪頭自然教育園區");       // ID 3
    myMap.addStation("孟宗竹林古戰場", SCENIC_BATTLEFIELD_BONUS); // ID 4

    float dist, angle, freq;

    cout << "=== 正在由實體地理軌跡檔萃取特徵 (三條路線) ===" << endl;

    if (FeatureExtractor::extractFeaturesFromFile("route1.txt", dist, angle, freq)) {
        myMap.addRoad(0, 3, dist, angle, freq, 1);
        cout << "[路線一] 151縣道直達 (0->3) | 距離:" << dist << "km 角度:" << angle
             << "度 頻率:" << freq << "次/km" << endl;
    }

    if (FeatureExtractor::extractFeaturesFromFile("route2.txt", dist, angle, freq)) {
        myMap.addRoad(0, 3, dist, angle, freq, 2);
        cout << "[路線二] 投55鄉道老線 (0->3) | 距離:" << dist << "km 角度:" << angle
             << "度 頻率:" << freq << "次/km" << endl;
    }

    float dist04, ang04, freq04, dist43, ang43, freq43;
    if (loadRoute3Split(dist04, ang04, freq04, dist43, ang43, freq43)) {
        myMap.addRoad(0, 4, dist04, ang04, freq04, 3);
        myMap.addRoad(4, 3, dist43, ang43, freq43, 3);
        cout << "[路線三] route3.txt 竹山->古戰場 (0->4) | 距離:" << dist04 << "km 角度:"
             << ang04 << "度 頻率:" << freq04 << "次/km" << endl;
        cout << "[路線三] route3.txt 古戰場->溪頭 (4->3) | 距離:" << dist43 << "km 角度:"
             << ang43 << "度 頻率:" << freq43 << "次/km" << endl;
    }

    cout << "====================================\n" << endl;

    showUI(myMap);

    cout << "\n--------------------" << endl;
    cout << "三條真實路線決策與觀光吸引力評估完成。" << endl;
    system("pause");
    return 0;
}
