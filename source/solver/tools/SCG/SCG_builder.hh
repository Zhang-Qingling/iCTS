#pragma once
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Inst.hh"
#include "Pin.hh"

namespace icts {

// 一条 FF→FF 数据边的参数
struct FFArc
{
  Pin* src_ff_clk = nullptr;  // 起始FF的引脚
  Pin* dst_ff_clk = nullptr;  // 终点FF的引脚
  double t_logic = 0.0;       // FF之间逻辑电路时延
  double t_CQ_src = 0.0;      // 起始FF的内部时延
  double setup_dst = 0.0;     // 终点FF的setup时延
  double hold_dst = 0.0;      // 终点FF的hold时延
};

// 差分约束求解器
class SCG
{
 public:
  SCG() = delete;
  ~SCG() = default;
  void setSinks(const std::vector<Pin*>& sinks)  // 图的顶点数据
  {
    _sinks = sinks;
    _idx.clear();
    for (size_t i = 0; i < sinks.size(); i++)
      _idx[sinks[i]] = i;
    _adj.assign(sinks.size(), std::vector<double>(sinks.size(), INF));  // 默认将为无穷大
    for (int i = 0; i < sinks.size(); i++)                              // 对角线设置为0
      _adj[i][i] = 0.0;
  }
  // 加入边
  void addArc(double T, const FFArc& a)  // 时钟周期，弧a
  {
    auto is = _idx.find(a.src_ff_clk), id = _idx.find(a.dst_ff_clk);
    if (is == _idx.end() || id == _idx.end())  // 起止pin是否在_sinks内
      return;
    int i = is->second, j = id->second;
    double L = a.hold_dst - a.t_CQ_src - a.t_logic;       // skew下界
    double U = T - a.setup_dst - a.t_CQ_src - a.t_logic;  // skew上界
    if (i == j)
      return;
    _adj[i][j] = std::min(_adj[i][j], -L);  // 边 i→j 权 -L
    _adj[j][i] = std::min(_adj[j][i], U);   // 边 j→i 权  U
  }
  // 计算最短路矩阵，检测是否可行（无负环）
  bool solve()
  {
    _dis = _adj;
    int n = (int) _dis.size();
    for (int k = 0; k < n; ++k)
      for (int i = 0; i < n; ++i)
        if (_dis[i][k] < INF / 2)
          for (int j = 0; j < n; ++j)
            if (_dis[k][j] < INF / 2)
              if (_dis[i][k] + _dis[k][j] < _dis[i][j])
                _dis[i][j] = _dis[i][k] + _dis[k][j];
    for (int v = 0; v < n; ++v)
      if (_dis[v][v] < -1e-12)
        return false;  // 负环
    return true;
  }
  // 查询 ti - tj 的FSR [L,U]
  std::pair<double, double> fsr(Pin* i_clk, Pin* j_clk)
  {
    int i = _idx.at(i_clk), j = _idx.at(j_clk);
    return {-_dis[i][j], _dis[j][i]};
  }

 private:
  static constexpr double INF = 1e100;
  std::vector<Pin*> _sinks;                     // idx映射pin
  std::unordered_map<Pin*, int> _idx;           // pin映射idx
  std::vector<std::vector<double>> _adj, _dis;  // 邻接矩阵，最短路矩阵
};

}  // namespace icts
