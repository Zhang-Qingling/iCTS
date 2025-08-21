#pragma once
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "Inst.hh"
#include "Pin.hh"

namespace icts {

// 一条 FF→FF 数据边的参数
struct FFArc
{
  Pin* src_ff_clk = nullptr;  // 起始FF的引脚
  Pin* dst_ff_clk = nullptr;  // 终点FF的引脚
  double dmin = 0.0;          // 组合路径的最小延迟，包含t_cq和tlogic
  double dmax = 0.0;          // 组合路径的最大延迟
  double setup_dst = 0.0;     // 终点FF的setup时延
  double hold_dst = 0.0;      // 终点FF的hold时延
};

// SCG求解器
class SCG
{
 public:
  SCG() = default;
  ~SCG() = default;
  void setSinks(const std::vector<Pin*>& sinks);          // 图的顶点数据
  void addArc(double T, const FFArc& a);                  // 加入边
  bool solve();                                           // 计算最短路矩阵，检测是否可行（无负环）
  std::pair<double, double> fsr(Pin* i_clk, Pin* j_clk);  // 查询 ti - tj 的FSR [L,U]
 private:
  static constexpr double INF = 1e100;
  std::vector<Pin*> _sinks;                     // idx映射pin
  std::unordered_map<Pin*, int> _idx;           // pin映射idx
  std::vector<std::vector<double>> _adj, _dis;  // 邻接矩阵，最短路矩阵
};

}  // namespace icts
