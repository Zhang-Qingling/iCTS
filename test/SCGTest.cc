#include <gtest/gtest.h>

#include "Inst.hh"
#include "Node.hh"         // for Point
#include "SCG_builder.hh"  // SCG 头
using namespace icts;
static icts::Pin* make_ff_clk_pin(const std::string& ff_name, const Point& loc)
{
  // 造一个触发器实例，类型用 kSink
  auto* inst = new Inst(ff_name, loc, InstType::kSink);
  auto* clk = inst->get_load_pin();
  clk->set_name(ff_name + "/CK");
  return clk;
}
// 添加弧
TEST(SCG, SimpleTwoFF)
{
  // 构造两个FF时钟脚
  icts::Pin* i_clk = make_ff_clk_pin("FF_I", Point(0, 0));
  icts::Pin* j_clk = make_ff_clk_pin("FF_J", Point(10, 0));
  // 配置 SCG
  SCG scg;
  scg.setSinks({i_clk, j_clk});
  const double T = 1.0;         // ns
  const double tcq = 0.05;      // ns
  const double t_logic = 0.20;  // ns
  const double setup_j = 0.05;  // ns
  const double hold_j = 0.05;   // ns

  FFArc a;
  a.src_ff_clk = i_clk;
  a.dst_ff_clk = j_clk;
  a.t_cq_src = tcq;
  a.t_logic = t_logic;
  a.setup_dst = setup_j;
  a.hold_dst = hold_j;

  scg.addArc(T, a);
  ASSERT_TRUE(scg.solve());

  // skew = s_i - s_j
  // L = hold - tcq - t_logic
  // U = T - setup - tcq - t_logic
  const double L = hold_j - tcq - t_logic;       // = 0.05 - 0.05 - 0.20 = -0.20
  const double U = T - setup_j - tcq - t_logic;  // = 1.00 - 0.05 - 0.05 - 0.20 = 0.70

  auto [l_ij, u_ij] = scg.fsr(i_clk, j_clk);  // ti - tj 的范围
  EXPECT_NEAR(l_ij, L, 1e-12);
  EXPECT_NEAR(u_ij, U, 1e-12);

  // 反向 j,i：应为 [-U, -L]
  auto [l_ji, u_ji] = scg.fsr(j_clk, i_clk);
  EXPECT_NEAR(l_ji, -U, 1e-12);
  EXPECT_NEAR(u_ji, -L, 1e-12);
}

TEST(SCG, ThreeFF_Closure)
{
  // 三个点，测试 Floyd 的传递闭包是否正常（间接路径收紧上界/下界）
  icts::Pin* A = make_ff_clk_pin("FF_A", Point(0, 0));
  icts::Pin* B = make_ff_clk_pin("FF_B", Point(10, 0));
  icts::Pin* C = make_ff_clk_pin("FF_C", Point(20, 0));

  SCG scg;
  scg.setSinks({A, B, C});

  const double T = 1.0, tcq = 0.05, setup = 0.05, hold = 0.05;

  // 选不同 “t_logic” 来形成“间接更紧”的约束
  // A->B
  FFArc ab{A, B, 0.20, tcq, setup, hold};
  // B->C
  FFArc bc{B, C, 0.25, tcq, setup, hold};

  scg.addArc(T, ab);
  scg.addArc(T, bc);
  ASSERT_TRUE(scg.solve());

  // 对于当前公式（skew = s_i - s_j，且 dmin=dmax=t_logic）：
  auto bound = [&](double tlog) {
    double L = hold - tcq - tlog;
    double U = T - setup - tcq - tlog;
    return std::pair<double, double>{L, U};
  };

  auto [L_ab, U_ab] = bound(0.20);
  auto [L_bc, U_bc] = bound(0.25);

  // 传递闭包：s_A - s_C 的上界 ≤ U_AB + U_BC；下界 ≥ L_AB + L_BC
  auto [L_AC, U_AC] = scg.fsr(A, C);
  EXPECT_NEAR(U_AC, U_ab + U_bc, 1e-12);
  EXPECT_NEAR(L_AC, L_ab + L_bc, 1e-12);
}
