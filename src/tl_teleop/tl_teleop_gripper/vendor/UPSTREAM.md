# 夹爪 SDK 来源

复制自用户提供的 /home/robot/tcb_sdk_python/claw_sdk.py。
仅保留类与导入，移除了直接运行时自动找行程和无限开合的示例入口。
原文件 SHA256: 9d368c4692d4cd5671a4198577b36f87525cf05865fc86eef303cb9b9611c240

本项目通过受限适配器调用，机械臂 SDK 不由此模块导入。

本副本于 2026-11-21 从 `vr_tcb610/third_party/lebai_claw_sdk/claw_sdk.py`
迁入 `tl_teleop`，SDK 文件保持原样。`tl_teleop_gripper/claw.py` 同时迁移了旧项目
`hardware/claw.py` 的寄存器白名单、CRC/应答核验，并增加串口独占。
运行时仅使用包内相对导入，不依赖 `vr_tcb610` 的目录或 Python 环境。
