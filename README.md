# 使用方法
将该iCTS替换/workspace/benchmark/AiEDA/third_party/iEDA/src/operation/iCTS，
在iEDA目录下的build文件夹里重新make：
```bash
cd build
make -j$(nproc)
```
# 调试SCGtest
进入build目录重新编译生成test模块
```bash
cd iEDA/build
cmake --build . --target icts_scg_test -j$(nproc)
```
在bin目录可列出所有子测试
```bash
cd iEDA/bin
./icts_scg_test --gtest_list_tests
```
跑某一个自测试：
```bash
./icts_scg_test --gtest_filter=SCG.CreateArc
```