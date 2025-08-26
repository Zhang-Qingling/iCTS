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

#include "Solver.hh"  // 本类的头文件，声明了 Solver 的接口和成员

#include <filesystem>  // 用于创建目录、判断文件是否存在等文件系统操作
#include <numeric>     // 提供 std::accumulate 等数值算法
#include <ranges>      // C++20 ranges，用于简洁地遍历/处理容器

#include "BalanceClustering.hh"     // 负载聚类与分组算法（HPWL、聚类、引导中心等）
#include "CtsDesign.hh"             // CTS 设计相关数据结构（实例/管脚/网络等）
#include "SCG_builder.hh"           // 你的 SCG（差分约束→时间窗口）构建器接口
#include "ThreadPool/ThreadPool.h"  // 线程池并行工具
#include "TimingPropagator.hh"      // 时序传播（线长/电容/延迟/到达时间等）
#include "TreeBuilder.hh"           // 构树与缓冲器实例化、连线的工具
#include "log/Log.hh"               // 日志接口 LOG_INFO/LOG_WARNING 等
#include "report/CtsReport.hh"      // 报表生成接口（CtsReportTable 等）
#include "time/Time.hh"             // 时间工具（格式化当前时间等）
namespace icts {                    // 命名空间 iCTS 所有符号

void Solver::run()
{
  LOG_INFO << "[DEBUG] >>>>>> My Custom iCTS Build Running <<<<<<";  // 调试打印：开始运行自定义构建
  init();  // 初始化：把 CtsPin/CtsInstance 转换为内部 Inst/Pin，并设置 driver/_sink_pins
  buildUSTDME();
  levelReport();  // 生成各层统计报表（fanout/netlen/cap/slew/delay/skew 等）
}

void Solver::runbak()  // 旧的run
{
  init();           // 初始化：把 CtsPin/CtsInstance 转换为内部 Inst/Pin，并设置 driver/_sink_pins
  resolveSinks();   // 自底向上层次化聚类/合并，生成顶层负载集合 _top_pins（或根）
  breakLongWire();  // 对过长的 driver→sink 连接进行分段插缓冲，最后生成顶层网络
  // report
  levelReport();  // 生成各层统计报表（fanout/netlen/cap/slew/delay/skew 等）
}

void Solver::init()
{
  auto* driver_inst = _cts_driver->get_instance();                             // 从 CtsPin 获取其所属实例（源驱动）
  auto* inst = new Inst(driver_inst->get_name(), driver_inst->get_location(),  // 新建内部 Inst，类型设为缓冲器（kBuffer）
                        InstType::kBuffer);
  _driver = inst->get_driver_pin();                 // 取该实例的 driver pin（作为根 driver）
  _driver->set_name(_cts_driver->get_full_name());  // 用 CtsPin 的全名覆盖以保持一致

  std::ranges::for_each(_cts_pins, [&](CtsPin* pin) {  // 遍历所有 sink 侧 CtsPin
    // LOG_INFO << "[init] visiting pin: " << (pin ? pin->get_full_name() : "<null>");
    auto* cts_inst = pin->get_instance();                                          // 所属的 CtsInstance
    auto type = cts_inst->get_type() == CtsInstanceType::kSink  ? InstType::kSink  // 将 CtsInstanceType 映射到内部 InstType
                : cts_inst->get_type() == CtsInstanceType::kMux ? InstType::kBuffer
                                                                : InstType::kBuffer;
    auto* inst = new Inst(cts_inst->get_name(), cts_inst->get_location(), type);  // 构造内部 Inst，复制名字与坐标
    auto* load_pin = inst->get_load_pin();                                        // 取该实例的负载 pin（被驱动端）
    load_pin->set_name(pin->get_full_name());                                     // 设置 pin 的全名
    // update load pin cap
    if (inst->isSink()) {                        // 如果是终端 sink
      TimingPropagator::updatePinCap(load_pin);  // 用 API/Liberty 更新负载电容
      _sink_pins.push_back(load_pin);            // 加入 sink 列表
    } else {
      // inst->set_cell_master(TimingPropagator::getMinSizeCell());              // 旧注释：可设为最小单元
      // _top_pins.push_back(load_pin);
      auto cell_exist = CTSAPIInst.cellLibExist(cts_inst->get_cell_master());  // 检查该实例的 cell master 是否存在于库
      inst->set_cell_master(cell_exist ? cts_inst->get_cell_master()           // 若存在用原型；否则退回最小单元
                                       : TimingPropagator::getMinSizeCell());
      _sink_pins.push_back(load_pin);  // TBD for mux pin                             // MUX 也临时当成 sink 处理
    }
  });
  TreeBuilder::localPlace(_sink_pins);  // 简单本地合法化放置（避免非法位置）
}

void Solver::resolveSinks()
{
  // LOG_INFO << "Sink pins count = " << _sink_pins.size();
  if (_sink_pins.empty()) {  // 若没有 sink，直接返回
    return;
  }
  if (_sink_pins.size() == 1) {  // 只有一个 sink，直接作为顶层 pin
    _top_pins.push_back(_sink_pins.front());
    return;
  }
  // convert to inst
  std::vector<Inst*> insts;  // 将 sink 的 Pin 转回其 Inst，便于聚类
  std::ranges::for_each(_sink_pins, [&](Pin* pin) { insts.push_back(pin->get_inst()); });
  _level_insts.push_back(insts);  // 记录第 0 层的实例集合
                                  // LOG_INFO << "Level " << _level_insts.size() - 1 << " inst count = " << insts.size();
  // clustering
  while (insts.size() > 1) {                       // 多于 1 个则继续层次聚类/合并
    auto assign = get_level_assign(_level);        // 从配置取本层的聚类/约束参数
    insts = assignApply(insts, assign);            // 应用本层策略：聚类+布线/缓冲，产出上层 insts
    _level_insts.push_back(insts);                 // 记录该层产物
    std::ranges::for_each(insts, [](Inst* inst) {  // 新层的每个实例，更新其负载 pin 的电容
      auto* load_pin = inst->get_load_pin();
      // update load pin cap
      TimingPropagator::updatePinCap(load_pin);
    });
    ++_level;  // 进入下一层
  }
  auto* root = insts.front();                                  // 最终只剩 1 个实例，视为“根”
  root->set_cell_master(TimingPropagator::getRootSizeCell());  // 根用 Root 尺寸的 buffer
  auto* root_driver_pin = root->get_driver_pin();              // 根的 driver pin
  auto* root_net = root_driver_pin->get_net();                 // 根 driver 所在的 net（可能为局部 net）
  if (_root_buffer_required) {                                 // 若需要保留根缓冲器
    TimingPropagator::update(root_net);                        // 先做一次时序传播
    TimingPropagator::initLoadPinDelay(root->get_load_pin());  // 初始化负载 pin 的 min/max delay
    _top_pins.push_back(root->get_load_pin());                 // 将根的 load pin 作为顶层 pin
    return;                                                    // 结束
  }
  // if (TimingPropagator::calcLen(_driver, root_driver_pin) + root_driver_pin->get_sub_len() <= TimingPropagator::getMaxLength()) {
  auto load_pins = root_net->get_load_pins();     // 取根网络的全部负载 pin
  if (!_root_buffer_required && _inherit_root) {  // 不插根缓冲，且允许继承根尺寸
    std::ranges::for_each(load_pins, [](Pin* pin) {
      auto* inst = pin->get_inst();
      inst->set_cell_master(TimingPropagator::getRootSizeCell());  // 每个子实例都设置为根尺寸（增驱动）
    });
  }
  auto net_name = root_net->get_name();  // 记录旧根网名（未直接使用）
  _nets.erase(std::remove_if(_nets.begin(), _nets.end(),
                             [&](Net* net) {  // 从 _nets 容器里移除 root_net
                               return net == root_net;
                             }),
              _nets.end());
  TimingPropagator::resetNet(root_net);                                                     // 释放根网络结构，断开/回收 Steiner/Buffer
  std::ranges::for_each(load_pins, [&](Pin* load_pin) { _top_pins.push_back(load_pin); });  // 将所有子负载 pin 作为顶层 pin
  _level_insts.erase(_level_insts.end() - 1);                                               // 移除上一条记录的根层
  // } else {
  //   _top_pins.push_back(root->get_load_pin());
  // }
}

std::vector<Inst*> Solver::levelProcess(const std::vector<std::vector<Inst*>>& clusters, const std::vector<Point> guide_locs,
                                        const Assign& assign)
{
  auto skew_bound = assign.skew_bound;              // 本层允许的 skew 界
  std::vector<Inst*> level_insts;                   // 本层输出的上层实例集合
  if (_max_thread > 1) {                            // 若开启多线程
    ThreadPool pool(_max_thread);                   // 创建线程池
    std::vector<std::future<Inst*>> results;        // 存放每个任务的 future
    for (size_t i = 0; i < clusters.size(); ++i) {  // 遍历每个簇
      auto cluster = clusters[i];
      auto guide_center = guide_locs[i];
      results.emplace_back(pool.enqueue([&] {  // 在线程池中并行处理每个簇
        if (_level > _latency_opt_level) {     // 高层级时做延迟优化（局部微调）
          BalanceClustering::latencyOpt(cluster, skew_bound, _local_latency_opt_ratio);
        }
        auto* buf = netAssign(cluster, assign, guide_center, _level > _shift_level);  // 为该簇布线/插缓冲并返回上层实例
        return buf;
      }));
    }
    for (auto&& result : results) {  // 收集并行任务的输出
      level_insts.push_back(result.get());
    }
    return level_insts;  // 返回本层输出
  }

  for (size_t i = 0; i < clusters.size(); ++i) {  // 单线程路径：逐簇处理
    auto cluster = clusters[i];
    auto guide_center = guide_locs[i];
    if (_level > _latency_opt_level) {
      BalanceClustering::latencyOpt(cluster, skew_bound, _local_latency_opt_ratio);  // 低层不做，高层微调
    }
    auto* buf = netAssign(cluster, assign, guide_center, _level > _shift_level);  // 为该簇生成上层“汇聚”实例
    level_insts.push_back(buf);
  }
  return level_insts;  // 返回本层输出
}

void Solver::breakLongWire()
{
  if (_top_pins.empty()) {  // 若没有顶层 pin，直接返回
    return;
  }
  std::vector<Pin*> final_load_pins;  // 存放最终与 driver 直接相连的 load pin（经过断线/插缓冲后）
  // auto max_len = 0.8 * TimingPropagator::getMaxLength();
  auto max_len = _break_long_wire ? TimingPropagator::getMaxLength()                     // 若允许断线，则用最大线长阈值
                                  : std::numeric_limits<double>::max();                  // 否则不限制
  std::ranges::for_each(_top_pins, [&](Pin* pin) {                                       // 遍历每个顶层 pin
    auto len = TimingPropagator::calcLen(_driver->get_location(), pin->get_location());  // 估算 driver→该 pin 的曼哈顿长度
    if (len < max_len) {                                                                 // 未超过阈值，直接收下
      final_load_pins.push_back(pin);
      return;
    }
    // break long wire
    int insert_num = std::floor(len / max_len);                                                     // 需要插入的缓冲器数量
    auto delta_loc = (_driver->get_location() - pin->get_location()) / (insert_num + 1);            // 相邻插入点的间距（向量）
    auto* load_pin = pin;                                                                           // 当前段的负载 pin（从原 pin 开始）
    auto driver_loc = pin->get_location();                                                          // 从末端往回插（或从端点递推）
    while (insert_num--) {                                                                          // 逐个插入缓冲
      driver_loc += delta_loc;                                                                      // 下一个缓冲器的位置
      auto buf_name = CTSAPIInst.toString(_net_name, "_break_", pin->get_name(), "_", insert_num);  // 生成唯一名
      auto* buf = TreeBuilder::genBufInst(buf_name, driver_loc);                                    // 实例化一个 buffer
      buf->set_cell_master(TimingPropagator::getMinSizeCell());                                     // 先用最小单元
      TreeBuilder::directConnectTree(buf->get_driver_pin(), load_pin);                              // 将该 buffer 与当前段负载 pin 直连
      auto* net = TimingPropagator::genNet(buf_name, buf->get_driver_pin(), {load_pin});            // 生成该段 Net
      TimingPropagator::update(net);                                                                // 传播，得到延迟/到达时间等
      _nets.push_back(net);                                                                         // 记录该段 net
      load_pin = buf->get_load_pin();  // 下一段的负载点改为该 buffer 的负载 pin
    }
    final_load_pins.push_back(load_pin);  // 最后一个段的负载 pin 作为最终负载
  });
  TreeBuilder::shallowLightTree("Salt", _driver, final_load_pins);            // 用浅轻树（Salt）快速连线 driver→最终负载们
  auto* net = TimingPropagator::genNet(_net_name, _driver, final_load_pins);  // 生成顶层 Net
  TimingPropagator::update(net);                                              // 传播更新
  _nets.push_back(net);                                                       // 记录顶层 Net
}

Assign Solver::get_level_assign(const int& level) const
{
  auto* config = CTSAPIInst.get_config();     // 访问 CTS 全局配置
  auto assign = config->query_assign(level);  // 查询本层级的聚类/布线参数（扇出、cap、长度等）
#ifdef DEBUG_ICTS_SOLVER
  LOG_INFO << "Level " << level << " Assign: " << std::endl;  // 调试打印该层参数
  LOG_INFO << "max_net_len: " << assign.max_net_len << std::endl;
  LOG_INFO << "max_fanout: " << assign.max_fanout << std::endl;
  LOG_INFO << "max_cap: " << assign.max_cap << std::endl;
  LOG_INFO << "ratio: " << assign.ratio << std::endl;
  LOG_INFO << "skew_bound: " << assign.skew_bound << std::endl;
#endif
  return assign;  // 返回本层级参数
}

std::vector<Inst*> Solver::assignApply(const std::vector<Inst*>& insts, const Assign& assign)
{
  LOG_INFO << "| Level: " << _level << " | Bounding HPWL: "  // 打印层级、HPWL、单元数等概览
           << BalanceClustering::calcHPWL(insts) << " um | Inst Num: " << insts.size() << " |";
  pinCapDistReport(insts);  // 打印负载电容分布（最小/最大/平均/中位）
  // pre-processing
  auto max_net_len = assign.max_net_len;  // 读取参数：最大线长
  auto max_fanout = assign.max_fanout;    // 最大扇出
  auto max_cap = assign.max_cap;          // 最大电容
  auto cluster_ratio = assign.ratio;      // 聚类比例（控制簇大小/数量）
  auto skew_bound = assign.skew_bound;    // 允许偏斜

  auto target_insts = insts;          // 基于输入 insts 做一份可修改副本
  if (_level > _latency_opt_level) {  // 高层时做全局延迟优化（排序/微调）
    BalanceClustering::latencyOpt(insts, skew_bound, _global_latency_opt_ratio);
  }

  auto clusters = BalanceClustering::iterClustering(target_insts, max_fanout, 5, 5, cluster_ratio);  // 迭代聚类得到若干簇
  // auto enhanced_clusters = clusters;
  auto enhanced_clusters = BalanceClustering::slackClustering(clusters, max_net_len, max_fanout);  // 依据长度/扇出做 slack 聚类
  if (enhanced_clusters.size() < insts.size()) {                                                   // 若簇数减少（成功聚合），进一步增强
    enhanced_clusters
        = BalanceClustering::clusteringEnhancement(enhanced_clusters, max_fanout, max_cap, max_net_len, skew_bound, 200, 0.95, 10000);
  }
  // top guide
  auto guide_centers = BalanceClustering::guideCenter(enhanced_clusters, std::nullopt,  // 计算每簇的引导中心（指导放置）
                                                      TimingPropagator::getMinLength(), 1);
  std::vector<icts::Inst*> level_insts;  // 本层输出容器
  if (_level > 1) {
    // higherDelayOpt(enhanced_clusters, guide_centers, level_insts);             // 预留：高延迟簇直接上推
  }
  // 若增强后簇数 = 上推数的补集，可做特殊处理（已注释）
  // BalanceClustering::writeClusterPy(enhanced_clusters, "cluster_level_" + std::to_string(_level)); // 输出 Python 可视化（可选）

  auto bufs = levelProcess(enhanced_clusters, guide_centers, assign);  // 对每个簇布线/插缓冲，得到上层 buffer 集
  level_insts.insert(level_insts.end(), bufs.begin(), bufs.end());     // 合并到输出
  return level_insts;                                                  // 返回本层实例
}

std::vector<Inst*> Solver::topGuide(const std::vector<Inst*>& insts, const Assign& assign)
{
  auto max_net_len = assign.max_net_len;                             // 读取最大线长约束
  auto sorted_insts = insts;                                         // 拷贝一份以便排序/修改
  int max_dist = max_net_len * 1.0 / TimingPropagator::getDbUnit();  // 以 DBU 表示的最大距离
  int est_net_dist = BalanceClustering::estimateNetLength(insts) *   // 估计该组的网络总距离（DBU）
                     TimingPropagator::getDbUnit();
  while (est_net_dist > max_dist) {                                 // 若估计大于限制，逐步插入 buffer 缩短
    std::ranges::sort(sorted_insts, [](Inst* inst1, Inst* inst2) {  // 按最大到达时间对实例排序
      return inst1->get_driver_pin()->get_max_delay() < inst2->get_driver_pin()->get_max_delay();
    });
    auto min_delay_inst = sorted_insts.front();                        // 取到达最早的实例
    auto loc = min_delay_inst->get_location();                         // 其坐标
    sorted_insts.erase(sorted_insts.begin());                          // 暂时从集合剔除
    auto center = BalanceClustering::calcBoundCentroid(sorted_insts);  // 余下实例的边界质心

    auto center_dist = TimingPropagator::calcDist(loc, center);               // 两者距离
    auto shift_dist = std::min(max_dist / 2, center_dist);                    // 允许的移动距离（不超过一半阈值）
    auto new_loc = (center - loc) * (1.0 * shift_dist / center_dist) + loc;   // 计算新的插入位置
    auto net_name = CTSAPIInst.toString(_net_name, "_", CTSAPIInst.genId());  // 生成 net/实例名
    auto* buffer = TreeBuilder::genBufInst(net_name, new_loc);                // 新建 buffer
    buffer->set_cell_master(TimingPropagator::getMinSizeCell());              // 最小单元
    auto* load_pin = min_delay_inst->get_load_pin();                          // 连接目标 load pin
    auto* driver_pin = buffer->get_driver_pin();                              // buffer 的 driver pin
    TreeBuilder::directConnectTree(driver_pin, load_pin);                     // 直连

    auto* net = TimingPropagator::genNet(net_name, driver_pin, {load_pin});  // 生成该段 net
    TimingPropagator::update(net);                                           // 更新时序
    _nets.push_back(net);                                                    // 记录
    est_net_dist = BalanceClustering::estimateNetLength(sorted_insts) *      // 重新估计总距离
                   TimingPropagator::getDbUnit();
    sorted_insts.push_back(buffer);  // 将新 buffer 也计入集合
  }
  return sorted_insts;  // 返回调整后的实例集合
}

Inst* Solver::netAssign(const std::vector<Inst*>& insts, const Assign& assign, const Point& guide_center, const bool& shift)
{
  auto max_net_len = assign.max_net_len;  // 读取最大线长
  auto skew_bound = assign.skew_bound;    // 偏斜上限

  auto guide_loc = guide_center;  // 默认引导点
  // center shift
  int max_dist = max_net_len * TimingPropagator::getDbUnit();   // DBU 下最大长度
  int net_dist = BalanceClustering::estimateNetLength(insts) *  // 估算当前簇的网络长度
                 TimingPropagator::getDbUnit();
  if (shift && net_dist <= max_dist) {                                                                           // 若允许 shift 且尚未超限
    auto center = BalanceClustering::calcBoundCentroid(insts);                                                   // 当前簇的边界质心
    int center_dist = std::ceil(TimingPropagator::calcDist(center, guide_center));                               // 与引导点距离
    auto ratio = 0.1 + (_level - 1) * 0.25;                                                                      // 允许移动比例，层越高越大
    ratio = ratio > 1 ? 1 : ratio;                                                                               // 上限 1
    int allow_center_dist = ratio * center_dist;                                                                 // 允许的移动距离
    auto shift_dist = std::min(max_dist - net_dist, allow_center_dist);                                          // 真正移动距离受两者限制
    guide_loc = center_dist > 0 ? (guide_center - center) * (1.0 * shift_dist / center_dist) + center : center;  // 计算新的引导位置
  }
  auto net_name = CTSAPIInst.toString(_net_name, "_", CTSAPIInst.genId());  // 生成该簇的 net/实例名
  std::vector<Pin*> cluster_load_pins;                                      // 收集该簇的所有 load pins
  std::ranges::for_each(insts, [&cluster_load_pins](Inst* inst) {
    auto load_pin = inst->get_load_pin();
    cluster_load_pins.push_back(load_pin);
  });
  // if (!shift) {
  if (cluster_load_pins.size() == 1) {  // 只有一个负载，直接插一个 buffer 汇聚
    auto* buffer = TreeBuilder::genBufInst(net_name, guide_loc);
    // set min size cell master
    buffer->set_cell_master(TimingPropagator::getMinSizeCell());  // 先用最小单元，后续可优化
    // location legitimization
    TreeBuilder::localPlace(buffer, cluster_load_pins);  // 本地合法化位置
    auto* driver_pin = buffer->get_driver_pin();
    auto* load_pin = cluster_load_pins.front();
    TreeBuilder::directConnectTree(driver_pin, load_pin);                           // 直连
    auto* net = TimingPropagator::genNet(net_name, driver_pin, cluster_load_pins);  // 生成 net
    TimingPropagator::update(net);                                                  // 更新时序
    _nets.push_back(net);                                                           // 记录
    return buffer;                                                                  // 返回该 buffer 作为上层实例
  }

  // build CBS
  auto* buffer = TreeBuilder::shiftCBSTree(net_name, cluster_load_pins, skew_bound,  // 构建 CBS（带 shift 的二分拓扑）
                                           guide_loc, TopoType::kBiPartition, _level > _shift_level, max_net_len);

  auto* driver_pin = buffer->get_driver_pin();                                        // 取 driver
  auto* cbs_net = TimingPropagator::genNet(net_name, driver_pin, cluster_load_pins);  // 生成 net
  buffer->set_cell_master(TimingPropagator::getMinSizeCell());                        // 先用最小单元
  // TreeBuilder::iterativeFixSkew(cbs_net, skew_bound, guide_loc); // TBD for testing
  // TreeBuilder::iterativeFixSkew(cbs_net, skew_bound, guide_loc);
  TimingPropagator::update(cbs_net);  // 传播更新
  _nets.push_back(cbs_net);           // 记录
  return buffer;                      // 返回 buffer 作为上层实例
  // }

  // 下面是另一套被注释掉的路径：先 Salt 再 DME 的尝试策略（保留供实验用）
  // ...（省略，与你原文件一致，这里不再重复注释）

  return buffer;  // 理论上不会走到这，但保留返回
}

Net* Solver::saltOpt(const std::vector<Inst*>& insts, const Assign& assign)
{
  struct Buffering
  {
    Point loc;       // 插入位置
    size_t cell_id;  // 选择的缓冲单元索引（不同驱动能力）
    double skew;     // 该方案下的 skew 值
  };
  auto skew_bound = assign.skew_bound;  // 偏斜约束

  int min_x = std::numeric_limits<int>::max();  // 计算簇的边界框
  int min_y = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int max_y = std::numeric_limits<int>::min();
  std::ranges::for_each(insts, [&](Inst* inst) {
    auto loc = inst->get_location();
    min_x = std::min(min_x, loc.x());
    min_y = std::min(min_y, loc.y());
    max_x = std::max(max_x, loc.x());
    max_y = std::max(max_y, loc.y());
  });
  Point lb = Point(min_x, min_y);                                    // 左下
  Point rb = Point(max_x, min_y);                                    // 右下
  Point lt = Point(min_x, max_y);                                    // 左上
  Point rt = Point(max_x, max_y);                                    // 右上
  Point center = BalanceClustering::calcCentroid(insts);             // 质心
  Point bound_center = BalanceClustering::calcBoundCentroid(insts);  // 边界质心

  std::vector<Point> loc_list = {lb, rb, lt, rt, center, bound_center};  // 候选插入位置集合
  auto lib_list = TimingPropagator::getDelayLibs();                      // 可用缓冲器的延迟库（不同尺寸）

  std::vector<Buffering> feasible_assign;  // 可行方案列表

  std::vector<Pin*> cluster_load_pins;  // 收集簇的负载 pin
  std::ranges::for_each(insts, [&cluster_load_pins](Inst* inst) {
    auto load_pin = inst->get_load_pin();
    cluster_load_pins.push_back(load_pin);
  });
  auto net_name = CTSAPIInst.toString(_net_name, "_", CTSAPIInst.genId());  // 名称生成
  std::ranges::for_each(loc_list, [&](const Point& loc) {                   // 枚举位置
    for (size_t i = 0; i < lib_list.size(); ++i) {                          // 枚举不同缓冲尺寸
      auto* lib = lib_list[i];
      auto cell_master = lib->get_cell_master();
      auto* buffer = TreeBuilder::genBufInst(net_name, loc);  // 生成备选 buffer
      auto* driver_pin = buffer->get_driver_pin();
      buffer->set_cell_master(cell_master);                                           // 设置该尺寸
      TreeBuilder::localPlace(buffer, cluster_load_pins);                             // 本地合法化
      TreeBuilder::shallowLightTree(net_name, driver_pin, cluster_load_pins);         // 以浅轻树连接
      auto* net = TimingPropagator::genNet(net_name, driver_pin, cluster_load_pins);  // 生成 net
      TimingPropagator::update(net);                                                  // 更新
      if (TimingPropagator::skewFeasible(driver_pin, skew_bound)) {                   // 若满足 skew 约束
        auto skew = TimingPropagator::calcSkew(driver_pin);                           // 计算当前 skew
        feasible_assign.push_back({buffer->get_location(), i, skew});                 // 记录可行方案
      }
      TimingPropagator::resetNet(net);  // 回收临时网络
    }
  });

  if (feasible_assign.empty()) {  // 若无可行方案，返回空
    return nullptr;
  }
  std::ranges::sort(feasible_assign, [&](const Buffering& assign1, const Buffering& assign2) {
    if (assign1.cell_id == assign2.cell_id) {  // 同尺寸时按 skew 升序
      return assign1.skew < assign2.skew;      // sort by skew
    }
    return assign1.cell_id < assign2.cell_id;  // 不同尺寸时偏向更小的单元
  });
  // assign
  auto best_assign = feasible_assign.front();                         // 取排序后的最优方案
  auto* buffer = TreeBuilder::genBufInst(net_name, best_assign.loc);  // 生成最终 buffer
  auto* driver_pin = buffer->get_driver_pin();
  auto cell_master = lib_list[best_assign.cell_id]->get_cell_master();  // 采用对应 cell
  buffer->set_cell_master(cell_master);
  TreeBuilder::localPlace(buffer, cluster_load_pins);
  TreeBuilder::shallowLightTree(net_name, driver_pin, cluster_load_pins);
  auto* net = TimingPropagator::genNet(net_name, driver_pin, cluster_load_pins);  // 生成最终 net
  TimingPropagator::update(net);
  return net;  // 返回该 net（供上层接收）
}

void Solver::higherDelayOpt(std::vector<std::vector<Inst*>>& clusters, std::vector<Point>& guide_centers,
                            std::vector<Inst*>& level_insts) const
{
  auto calc_max_delay = [](const std::vector<Inst*> cluster) {  // lambda：计算簇内最大到达时间
    auto max_delay = std::numeric_limits<double>::min();
    std::ranges::for_each(cluster, [&](const Inst* inst) {
      if (!inst->isBuffer()) {  // 非 buffer 跳过
        return;
      }
      auto* driver_pin = inst->get_driver_pin();
      max_delay = std::max(driver_pin->get_max_delay(), max_delay);
    });
    return max_delay;
  };
  std::vector<double> max_delay_list(clusters.size());  // 每个簇的最大到达时间
  for (size_t i = 0; i < clusters.size(); ++i) {
    max_delay_list[i] = calc_max_delay(clusters[i]);
  }
  auto min_val = std::ranges::min(max_delay_list);                  // 全局最小
  auto max_val = std::ranges::max(max_delay_list);                  // 全局最大
  if (max_val - min_val < TimingPropagator::getMinInsertDelay()) {  // 差异很小则不处理
    return;
  }
  auto bound = min_val + TimingPropagator::getMinInsertDelay();  // 低延迟阈值
  std::vector<std::vector<Inst*>> lower_clusters;                // 低延迟簇集合
  std::vector<Point> lower_guide_centers;                        // 对应引导点集合
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (max_delay_list[i] <= bound) {  // 低延迟簇：保留为下一轮处理
      lower_clusters.push_back(clusters[i]);
      lower_guide_centers.push_back(guide_centers[i]);
    } else {
      level_insts.insert(level_insts.end(), clusters[i].begin(), clusters[i].end());  // 高延迟簇：直接上推
    }
  }
  clusters = lower_clusters;            // 更新为低延迟簇
  guide_centers = lower_guide_centers;  // 更新为对应引导点
}

void Solver::writeManhattanNetPy(Pin* root, const std::string& save_name) const
{
  LOG_INFO << "Writing rectilinear (Manhattan) net to python file...";
  auto* config = CTSAPIInst.get_config();
  auto path = config->get_work_dir();
  std::ofstream ofs(path + "/" + save_name + ".py");

  ofs << "import matplotlib.pyplot as plt\n";
  ofs << "fig = plt.figure(figsize=(8,6), dpi=300)\n";
  ofs << "ax = plt.gca()\n";
  ofs << "ax.set_aspect('equal', adjustable='box')\n";  // 等比例显示
  ofs << "plt.axis('off')\n";                           // 隐藏坐标轴

  // 若 r_h*c_v < r_v*c_h，则倾向 HV，否则 VH
  bool use_hv = true;
  {
    double r_h = TimingPropagator::getUnitRes(LayerPattern::kH);
    double r_v = TimingPropagator::getUnitRes(LayerPattern::kV);
    double c_h = TimingPropagator::getUnitCap(LayerPattern::kH);
    double c_v = TimingPropagator::getUnitCap(LayerPattern::kV);
    if (r_h * c_v > r_v * c_h) {
      use_hv = false;  // 画成 VH
    }
  }

  auto write_node = [&](Node* node) {
    int x = node->get_location().x();
    int y = node->get_location().y();
    ofs << "plt.scatter([" << x << "], [" << y << "], s=6, zorder=10)\n";
    ofs << "plt.text(" << x << ", " << y << ", '" << node->get_name() << "', fontsize=6)\n";

    auto parent = node->get_parent();
    if (!parent)
      return;

    int px = parent->get_location().x();
    int py = parent->get_location().y();

    // 如果已经共线，只画一段
    if (x == px || y == py) {
      ofs << "plt.plot([" << x << ", " << px << "],[" << y << ", " << py << "], linewidth=0.6, zorder=5)\n";
      return;
    }

    if (use_hv) {
      // 先水平到 (px, y)，再垂直到 (px, py)
      ofs << "plt.plot([" << x << ", " << px << "],[" << y << ", " << y << "], linewidth=0.6, zorder=5)\n";
      ofs << "plt.plot([" << px << ", " << px << "],[" << y << ", " << py << "], linewidth=0.6, zorder=5)\n";
    } else {
      // 先垂直到 (x, py)，再水平到 (px, py)
      ofs << "plt.plot([" << x << ", " << x << "],[" << y << ", " << py << "], linewidth=0.6, zorder=5)\n";
      ofs << "plt.plot([" << x << ", " << px << "],[" << py << ", " << py << "], linewidth=0.6, zorder=5)\n";
    }
  };
  root->preOrder(write_node);

  // 只保存文件，不显示窗口
  ofs << "plt.savefig('" << save_name << ".png', dpi=300, bbox_inches='tight')\n";
  ofs.close();

  LOG_INFO << "Wrote " << (path + "/" + save_name + ".py") << " (run it in the container: `python3 " << save_name << ".py`) "
           << "→ outputs " << save_name << ".png";
}

void Solver::writeNetPy(Pin* root, const std::string& save_name) const
{
  LOG_INFO << "Writing net to python file...";  // 提示开始写出可视化脚本
  // write the cluster to python file
  auto* config = CTSAPIInst.get_config();  // 取工作目录
  auto path = config->get_work_dir();
  std::ofstream ofs(path + "/" + save_name + ".py");               // 打开输出文件
  ofs << "import matplotlib.pyplot as plt" << std::endl;           // 写入 Python header
  ofs << "fig = plt.figure(figsize=(8,6), dpi=300)" << std::endl;  // 画布设置
  auto write_node = [&ofs](Node* node) {                           // lambda：写出单个节点与父边
    ofs << "x = [";
    ofs << node->get_location().x();
    ofs << "]" << std::endl;
    ofs << "y = [";
    ofs << node->get_location().y();
    ofs << "]" << std::endl;
    ofs << "plt.scatter(x, y)" << std::endl;                                   // 画点
    ofs << "plt.text(x[0], y[0], '" << node->get_name() << "')" << std::endl;  // 标注名称
    auto parent = node->get_parent();
    if (parent) {
      // add fly line
      ofs << "plt.plot([";
      ofs << node->get_location().x();
      ofs << ", ";
      ofs << parent->get_location().x();
      ofs << "], [";
      ofs << node->get_location().y();
      ofs << ", ";
      ofs << parent->get_location().y();
      ofs << "], color='black', linestyle='-', linewidth=0.5)" << std::endl;  // 画父子连线
    }
  };
  root->preOrder(write_node);                                  // 前序遍历整棵树并写出脚本
  ofs << "plt.show()" << std::endl;                            // 显示
  ofs << "plt.savefig('" + save_name + ".png')" << std::endl;  // 保存图片
  ofs.close();                                                 // 关闭文件
}

void Solver::levelReport() const
{
  using Inst_Func = std::function<double(const Inst*)>;  // 取数函数类型别名
  auto gen_level_rpt = [&](const CtsReportType& rpt_type, const std::string& rpt_tittle, const std::string& file_name,
                           Inst_Func get_val_func, Inst_Func vio_func = nullptr) {
    auto dir = CTSAPIInst.get_config()->get_work_dir() + "/level_log";  // 报表目录
    if (!std::filesystem::exists(dir)) {                                // 若不存在则创建
      std::filesystem::create_directories(dir);
    }

    auto rpt = CtsReportTable::createReportTable("Level " + rpt_tittle + " Log", rpt_type);  // 新建报表（带标题与类型）
    for (size_t level = 1; level < _level_insts.size(); ++level) {                           // 从第 1 层起（第 0 层是原始 sinks）
      double min_val = std::numeric_limits<double>::max();                                   // 统计：最小值
      double max_val = std::numeric_limits<double>::min();                                   // 统计：最大值
      double avg_val = 0;                                                                    // 统计：均值（累计）
      double vio_num = 0;                                                                    // 统计：违规数量（可选）
      auto insts = _level_insts[level];                                                      // 本层实例集合
      std::ranges::for_each(insts, [&min_val, &max_val, &avg_val, &vio_num, &get_val_func, &vio_func](const Inst* inst) {
        auto val = get_val_func(inst);  // 取值
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
        avg_val += val;
        if (vio_func != nullptr) {  // 若有违规判定函数
          vio_num += vio_func(inst);
        }
      });
      avg_val /= insts.size();    // 均值
      if (vio_func == nullptr) {  // 无违规项时输出 "None"
        (*rpt) << level << insts.size() << min_val << max_val << avg_val << "None" << TABLE_ENDLINE;
      } else {
        (*rpt) << level << insts.size() << min_val << max_val << avg_val << vio_num << TABLE_ENDLINE;
      }
    }

    auto save_file_name = _net_name + "_" + file_name + ".rpt";  // 报表文件名
    auto save_path = dir + "/" + save_file_name;                 // 报表路径
    std::ofstream outfile(save_path);
    outfile << "Generate the report at " << Time::getNowWallTime() << std::endl;  // 写入生成时间
    outfile << rpt->c_str();                                                      // 写入表格内容
    outfile.close();
  };
  // fanout rpt
  gen_level_rpt(
      CtsReportType::kLevelFanout, "Fanout", "fanout",
      [](const Inst* inst) {
        auto* driver_pin = inst->get_driver_pin();
        auto* net = driver_pin->get_net();
        return net->getFanout();  // 每层每实例的扇出
      },
      [](const Inst* inst) {
        auto* driver_pin = inst->get_driver_pin();
        auto* net = driver_pin->get_net();
        return net->getFanout() > TimingPropagator::getMaxFanout();  // 是否违规（超过最大扇出）
      });
  // net len rpt
  gen_level_rpt(
      CtsReportType::kLevelNetLen, "Net Length", "net_len",
      [](const Inst* inst) {
        auto* driver_pin = inst->get_driver_pin();
        return driver_pin->get_sub_len();  // 子树线长
      },
      [](const Inst* inst) {
        auto* driver_pin = inst->get_driver_pin();
        return driver_pin->get_sub_len() > TimingPropagator::getMaxLength();  // 是否超过最大线长
      });
  // cap rpt
  gen_level_rpt(
      CtsReportType::kLevelCap, "Cap", "cap",
      [](const Inst* inst) {
        auto* driver_pin = inst->get_driver_pin();
        return driver_pin->get_cap_load();  // 驱动端看到的负载电容
      },
      [](const Inst* inst) {
        auto* driver_pin = inst->get_driver_pin();
        return driver_pin->get_cap_load() > TimingPropagator::getMaxCap();  // 是否超过最大电容
      });
  // slew rpt
  gen_level_rpt(
      CtsReportType::kLevelSlew, "Slew", "slew",
      [](const Inst* inst) {
        auto* load_pin = inst->get_load_pin();
        return load_pin->get_slew_in();  // 负载端输入边沿（slew）
      },
      [](const Inst* inst) {
        auto* load_pin = inst->get_load_pin();
        return load_pin->get_slew_in() > TimingPropagator::getMaxBufTran();  // 是否超过最大缓冲输出过渡时间
      });
  // min delay rpt
  gen_level_rpt(CtsReportType::kLevelDelay, "Min Delay", "min_delay", [](const Inst* inst) {
    auto* driver_pin = inst->get_driver_pin();
    return driver_pin->get_min_delay();  // 最早到达时间
  });
  // max delay rpt
  gen_level_rpt(CtsReportType::kLevelDelay, "Max Delay", "max_delay", [](const Inst* inst) {
    auto* driver_pin = inst->get_driver_pin();
    return driver_pin->get_max_delay();  // 最晚到达时间
  });
  // insert delay rpt
  gen_level_rpt(CtsReportType::kLevelInsertDelay, "Insert Delay", "insert_delay",
                [](const Inst* inst) { return inst->get_insert_delay(); });  // 单元插入延迟
  // skew rpt
  gen_level_rpt(
      CtsReportType::kLevelSkew, "Skew", "skew",
      [](const Inst* inst) {
        auto* driver_pin = inst->get_driver_pin();
        return driver_pin->get_max_delay() - driver_pin->get_min_delay();  // skew = max - min
      },
      [](const Inst* inst) {
        auto* driver_pin = inst->get_driver_pin();
        return !TimingPropagator::skewFeasible(driver_pin);  // 是否违反 skew 约束
      });
}

void Solver::pinCapDistReport(const std::vector<Inst*>& insts) const
{
  std::vector<double> cap_list;  // 收集所有负载电容
  std::ranges::for_each(insts, [&cap_list](const Inst* inst) {
    auto* load_pin = inst->get_load_pin();
    cap_list.push_back(load_pin->get_cap_load());
  });
  std::ranges::sort(cap_list);  // 排序便于取中位数
  // min, max, avg, median
  auto min_val = cap_list.front();                                                          // 最小
  auto max_val = cap_list.back();                                                           // 最大
  auto avg_val = std::accumulate(cap_list.begin(), cap_list.end(), 0.0) / cap_list.size();  // 平均
  auto median_val = cap_list[cap_list.size() / 2];                                          // 中位
  LOG_INFO << ">>> Pin Cap Dist Report: " << std::endl;                                     // 打印标题
  LOG_INFO << ">>> Min: " << min_val << " fF" << std::endl;                                 // 单位 fF（视接口而定）
  LOG_INFO << ">>> Max: " << max_val << " fF" << std::endl;
  LOG_INFO << ">>> Avg: " << avg_val << " fF" << std::endl;
  LOG_INFO << ">>> Median: " << median_val << " fF" << std::endl;
}
void Solver::buildUSTDME()
{
  LOG_INFO << "[UST/DME] Build a trivial STAR: driver -> all sinks for net " << _net_name;

  if (_driver == nullptr) {
    LOG_WARNING << "Driver is null, abort trivial build.";
    return;
  }
  if (_sink_pins.empty()) {
    LOG_WARNING << "No sinks, nothing to connect.";
    return;
  }

  // 把第0层记录为所有 sink 的实例，方便 levelReport 统计

  std::vector<Inst*> level0;
  level0.reserve(_sink_pins.size());
  for (auto* p : _sink_pins) {
    level0.push_back(p->get_inst());
  }
  _level_insts.clear();
  _level_insts.push_back(std::move(level0));

  // 确保根实例有 cell

  Inst* root_inst = _driver->get_inst();
  if (root_inst->get_cell_master().empty()) {
    // 用 root buffer 类型；没有就退回最小单元
    auto root_cell = TimingPropagator::getRootSizeCell();
    if (!root_cell.empty()) {
      root_inst->set_cell_master(root_cell);
    } else {
      root_inst->set_cell_master(TimingPropagator::getMinSizeCell());
    }
  }

  // 建立“星形”拓扑：driver 直连每个 sink
  for (Pin* sink_pin : _sink_pins) {
    TreeBuilder::directConnectTree(_driver, sink_pin);
  }

  // 生成 Net，更新 RC/时序，登记输出
  Net* net = TimingPropagator::genNet(_net_name, _driver, _sink_pins);
  TimingPropagator::update(net);
  _nets.push_back(net);

  _level_insts.push_back({_driver->get_inst()});
  _level = static_cast<int>(_level_insts.size()) - 1;
  writeManhattanNetPy(_driver, "star_" + _net_name);
}

}  // namespace icts
