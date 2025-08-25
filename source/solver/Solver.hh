// ***************************************************************************************
// Copyright (c) 2023-2025 Peng Cheng Laboratory
// Copyright (c) 2023-2025 Institute of Computing Technology, Chinese Academy of Sciences
// Copyright (c) 2023-2025 Beijing Institute of Open Source Chip
//
// iEDA is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
// http://license.coscl.org.cn/MulanPSL2
//
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
//
// See the Mulan PSL v2 for more details.
// ***************************************************************************************
/**
 * @file Solver.cc
 * @author Dawn Li (dawnli619215645@gmail.com)
 */
#pragma once  // 防止头文件被重复包含

#include <string>
#include <vector>

#include "CtsInstance.hh"  // CTS 的实例定义（时钟树里的单元/触发器）
#include "CtsNet.hh"       // CTS 的网络定义（时钟网络）
#include "CtsPin.hh"       // CTS 的引脚定义
#include "Inst.hh"         // 内部 Solver 使用的实例类（比 CtsInstance 更轻量）
#include "Node.hh"         // 基础的树节点类（Inst/Pin 都继承自 Node）

namespace icts {

/**
 * @brief Solver 类：时钟树求解器
 *
 * 负责从一个时钟源和一组 sink，引入缓冲器，构建时钟树。
 * 里面封装了完整的流程：init -> resolveSinks -> breakLongWire -> report。
 */
class Solver
{
 public:
  Solver() = delete;  // 禁止默认构造（必须传 net_name / driver / pins）

  // 构造函数：需要一个时钟网络名字、driver 引脚和所有 sink 引脚
  Solver(const std::string& net_name, CtsPin* cts_driver, const std::vector<CtsPin*>& cts_pins)
      : _net_name(net_name), _cts_driver(cts_driver), _cts_pins(cts_pins)
  {
    // 从全局配置（CTSAPIInst）里读取算法参数，保存到 Solver 的成员变量
    auto* config = CTSAPIInst.get_config();
    _root_buffer_required = config->is_root_buffer_required();           // 是否强制插 root buffer
    _inherit_root = config->is_inherit_root();                           // root 属性是否继承给子树
    _break_long_wire = config->is_break_long_wire();                     // 是否要断长线
    _shift_level = config->get_shift_level();                            // 允许 shift 的层级
    _latency_opt_level = config->get_latency_opt_level();                // 延迟优化开始的层级
    _global_latency_opt_ratio = config->get_global_latency_opt_ratio();  // 全局延迟优化比例
    _local_latency_opt_ratio = config->get_local_latency_opt_ratio();    // 局部延迟优化比例
  }

  ~Solver() = default;

  // ============= 外部接口 =============
  void set_max_thread(const uint8_t& max_thread) { _max_thread = max_thread; }  // 设置线程数
  void run();     // 主要执行函数（init + resolveSinks + breakLongWire + report）
  void runbak();  // 备用的旧版本

  std::vector<Net*> get_solver_nets() const { return _nets; }  // 获取生成的所有 Net（结果）

 private:
  // ============= 内部流程函数 =============
  void init();          // 初始化：把 CtsPin/CtsInstance 转换为内部 Inst/Pin
  void resolveSinks();  // 聚类、逐层合并 sinks，生成时钟树骨架
  std::vector<Inst*> levelProcess(const std::vector<std::vector<Inst*>>& clusters, const std::vector<Point> guide_locs,
                                  const Assign& assign);                                  // 处理某一层的聚类结果
  void breakLongWire();                                                                   // 断长线：如果 driver->sink 线太长，中间插 buffer
  Assign get_level_assign(const int& level) const;                                        // 查询该层的限制条件（fanout/cap/skew）
  std::vector<Inst*> assignApply(const std::vector<Inst*>& insts, const Assign& assign);  // 应用分配规则，聚类 + 生成下一层 buffer
  std::vector<Inst*> topGuide(const std::vector<Inst*>& insts, const Assign& assign);     // 顶层 guide 调整
  Inst* netAssign(const std::vector<Inst*>& insts, const Assign& assign, const Point& guide_center,
                  const bool& shift = true);                            // 在一组 inst 上生成 buffer + net
  Net* saltOpt(const std::vector<Inst*>& insts, const Assign& assign);  // Salt 优化（额外 skew 修正）
  void higherDelayOpt(std::vector<std::vector<Inst*>>& clusters, std::vector<Point>& guide_centers,
                      std::vector<Inst*>& level_insts) const;  // 更高延迟的分组优化

  // ============= 报表和调试输出 =============
  void writeNetPy(Pin* root, const std::string& save_name = "net") const;  // 把树写成 Python 脚本，可绘图
  void levelReport() const;                                                // 按层统计（fanout / cap / slew / delay / skew）
  void pinCapDistReport(const std::vector<Inst*>& insts) const;            // 打印某层所有 pin 的电容分布

  // ============= 成员变量 =============
  std::string _net_name;           // 时钟网络名字
  CtsPin* _cts_driver;             // 输入的 driver（源头）
  std::vector<CtsPin*> _cts_pins;  // 输入的所有 sink 引脚

  std::vector<std::vector<Inst*>> _level_insts;  // 分层存储：每层的 inst 集合（用于报表）
  std::vector<Pin*> _sink_pins;                  // 所有 sink 对应的内部 Pin
  std::vector<Pin*> _top_pins;                   // 聚类完成后仍然需要处理的顶层 Pin
  Pin* _driver = nullptr;                        // 时钟源 driver（内部 Inst）
  std::vector<Net*> _nets;                       // 生成的所有 Net（树结构结果）

  int _level = 1;           // 当前层级编号
  uint8_t _max_thread = 1;  // 线程数

  // ============= 从配置读取的参数 =============
  bool _root_buffer_required = true;       // 是否强制插 root buffer
  bool _inherit_root = true;               // root buffer 属性是否继承
  bool _break_long_wire = true;            // 是否要断长线
  int _shift_level = 1;                    // shift 从哪层开始
  int _latency_opt_level = 1;              // 延迟优化从哪层开始
  double _global_latency_opt_ratio = 0.3;  // 全局延迟优化比例
  double _local_latency_opt_ratio = 0.4;   // 局部延迟优化比例
};

}  // namespace icts
