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
 * @file Solver.hh
 * @author Dawn Li (dawnli619215645@gmail.com)
 */
#pragma once
#include <string>
#include <vector>

#include "CtsInstance.hh"
#include "CtsNet.hh"
#include "CtsPin.hh"
#include "Inst.hh"
#include "Node.hh"
namespace icts {
/**
 * @brief Global Optimal Constraint Assignment
 *
 */
class Solver
{
 public:
  Solver() = delete;
  Solver(const std::string& net_name, CtsPin* cts_driver, const std::vector<CtsPin*>& cts_pins)
      : _net_name(net_name), _cts_driver(cts_driver), _cts_pins(cts_pins)
  {
    auto* config = CTSAPIInst.get_config();
    _root_buffer_required = config->is_root_buffer_required();
    _inherit_root = config->is_inherit_root();
    _break_long_wire = config->is_break_long_wire();
    _shift_level = config->get_shift_level();
    _latency_opt_level = config->get_latency_opt_level();
    _global_latency_opt_ratio = config->get_global_latency_opt_ratio();
    _local_latency_opt_ratio = config->get_local_latency_opt_ratio();
  }

  ~Solver() = default;
  // run
  void set_max_thread(const uint8_t& max_thread) { _max_thread = max_thread; }
  void run();
  // get
  std::vector<Net*> get_solver_nets() const { return _nets; }

 private:
  // flow
  void init();
  void resolveSinks();
  std::vector<Inst*> levelProcess(const std::vector<std::vector<Inst*>>& clusters, const std::vector<Point> guide_locs,
                                  const Assign& assign);
  void breakLongWire();
  Assign get_level_assign(const int& level) const;
  std::vector<Inst*> assignApply(const std::vector<Inst*>& insts, const Assign& assign);
  std::vector<Inst*> topGuide(const std::vector<Inst*>& insts, const Assign& assign);
  Inst* netAssign(const std::vector<Inst*>& insts, const Assign& assign, const Point& guide_center, const bool& shift = true);
  Net* saltOpt(const std::vector<Inst*>& insts, const Assign& assign);
  void higherDelayOpt(std::vector<std::vector<Inst*>>& clusters, std::vector<Point>& guide_centers, std::vector<Inst*>& level_insts) const;
  // report
  void writeNetPy(Pin* root, const std::string& save_name = "net") const;
  void levelReport() const;
  void pinCapDistReport(const std::vector<Inst*>& insts) const;
  // member
  std::string _net_name;
  CtsPin* _cts_driver;
  std::vector<CtsPin*> _cts_pins; // one net's all load pins
  std::vector<std::vector<Inst*>> _level_insts;
  std::vector<Pin*> _sink_pins;
  std::vector<Pin*> _top_pins;
  Pin* _driver = nullptr;
  std::vector<Net*> _nets;
  int _level = 1;
  uint8_t _max_thread = 1;
  // config
  bool _root_buffer_required = true;
  bool _inherit_root = true;
  bool _break_long_wire = true;
  int _shift_level = 1;
  int _latency_opt_level = 1;
  double _global_latency_opt_ratio = 0.3;
  double _local_latency_opt_ratio = 0.4;
};
}  // namespace icts