# 本仓库为第十届集创赛——龙芯中科杯赛题“矩阵乘法器设计与优化”任务的存档。
## CI 工作流程

推送到非受保护分支后，`.gitlab-ci.yml` 会触发 `bitstream` 作业，主要流程如下：

1. 从服务器环境复制区域赛决赛发布包 `ciciec2026_loongson_regional`。
2. 将本仓库中的 `rtl/` 覆盖到发布包的 `rtl/`。
3. 将本仓库中的 `sdk/` 覆盖到发布包的 `sdk/`。
4. 编译 `ciciec2026_loongson_regional/sdk/software/examples/asm`，命令中会显式传入：

   ```text
   MATMUL_GROUP_NUM=5000 COPY_OUTPUT=0
   ```

   因此正式 CI 产物中的 `user-sample.bin` 按 5000 组矩阵数据编译。若需要额外 C 编译参数，请写入对应 Makefile 或源码宏定义中。

5. CI 会计算本仓库 `rtl/` 的哈希值。
6. 若当前分支最近一次成功作业的 `rtl/` 哈希一致，CI 会复用上一次 Vivado 产物，并保留本次重新编译得到的 `user-sample.bin`。
7. 若 `rtl/` 发生变化，或没有可复用产物，CI 会执行 Vivado 2019.2 流程：工程创建、HDL lint、综合实现、DSP 资源检查、WNS 时序检查和 bitstream 生成。

注意：只修改 C 程序时，CI 可能不会重新执行完整 Vivado 实现流程；这是预期行为。只要本次 `user-sample.bin` 与复用的 bitstream 匹配，即可继续在 FPGA 在线实验平台选择产物进行评测。

## 最小提交流程

1. 基于发布包完成硬件和软件设计。
2. 将相对发布包新增或修改的 RTL 文件放入本仓库 `rtl/` 对应位置。
3. 将 C 程序替换为：

   ```text
   sdk/software/examples/asm/user-sample.c
   ```

4. 如需新增 IP，将其放入：

   ```text
   rtl/ip/<ip_name>/
   ```

5. 使用非 `main` / `master` 分支推送提交，等待 CI 完成。
6. 在 FPGA 在线实验平台绑定访问令牌（Access token），选择 CI 产物进行评测，并按平台要求进行有效标记。

## 提交文件要求

本仓库只需要提交相对发布包新增或修改的文件，不要提交临时生成文件、Vivado 工程缓存、无关材料或展示材料。

允许提交的工程文件类型包括：

- Verilog/VHDL 等硬件源代码。
- Netlist 形式提供的模块。
- Vivado IP 的 `.xci` 配置文件。
- C 语言程序及必要的软件适配文件。

文件位置必须严格遵从发布包中的工程结构。例如：

```text
rtl/soc_top.v
rtl/ip/matmul/matmul_axi_slave.v
sdk/software/examples/asm/user-sample.c
```

## CI 产物

流水线会保留以下主要产物，供调试和平台评测使用：

- `ciciec2026_loongson_regional/sdk/software/examples/asm/obj/user-sample.bin`
- `ciciec2026_loongson_regional/fpga/project/Loongson_Soc.runs/impl_1/*.bit`
- `ciciec2026_loongson_regional/fpga/project/Loongson_Soc.runs/impl_1/timing_summary.rpt`
- `ciciec2026_loongson_regional/fpga/project/dsp_utilization.rpt`
- `ciciec2026_loongson_regional/fpga/project/linter.log`
- `ciciec2026_loongson_regional/fpga/project/linter.args`
- `ciciec2026_loongson_regional/fpga/project/Loongson_Soc.runs/*/runme.log`
- `ciciec2026_loongson_regional/fpga/*.log`
- `ciciec2026_loongson_regional/fpga/*.jou`
- `rtl.sha256`

其中 `timing_summary.rpt` 用于查看 WNS，`dsp_utilization.rpt` 用于查看 DSP 使用情况，`linter.log` 用于定位 HDL lint 问题，`rtl.sha256` 用于判断本次作业是否复用了上一轮 Vivado 产物。

