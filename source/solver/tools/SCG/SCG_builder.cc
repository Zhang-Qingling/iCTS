#include "SCG_builder.hh"

namespace icts {
void SCG::setSinks(const std::vector<Pin*>& sinks)  // 图的顶点数据
{
  _sinks = sinks;
  _idx.clear();
  for (size_t i = 0; i < sinks.size(); i++)
    _idx[sinks[i]] = i;
  _adj.assign(sinks.size(), std::vector<double>(sinks.size(), INF));  // 默认将为无穷大
  for (size_t i = 0; i < sinks.size(); i++)                           // 对角线设置为0
    _adj[i][i] = 0.0;
}

void SCG::addArc(double T, const FFArc& a)  // 时钟周期，弧a
{
  auto is = _idx.find(a.src_ff_clk), id = _idx.find(a.dst_ff_clk);
  if (is == _idx.end() || id == _idx.end())  // 起止pin是否在_sinks内
    return;
  int i = is->second, j = id->second;
  double L = a.hold_dst - a.dmin;       // skew下界
  double U = T - a.setup_dst - a.dmax;  // skew上界
  if (i == j)
    return;
  _adj[i][j] = std::min(_adj[i][j], -L);  // 边 i→j 权 -L
  _adj[j][i] = std::min(_adj[j][i], U);   // 边 j→i 权  U
}

bool SCG::solve()
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

std::pair<double, double> SCG::fsr(Pin* i_clk, Pin* j_clk)
{
  int i = _idx.at(i_clk), j = _idx.at(j_clk);
  return {-_dis[i][j], _dis[j][i]};
}
}  // namespace icts