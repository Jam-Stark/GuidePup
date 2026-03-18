#!/usr/bin/env python3

"""
简化的环境测试脚本 - 用于诊断环境初始化问题
"""

import torch
import argparse

# Isaac Sim启动器必须在所有Isaac相关导入之前
from isaaclab.app import AppLauncher

from _extension_source import append_acc_source_dir

# 添加命令行参数
parser = argparse.ArgumentParser(description="测试环境初始化")
parser.add_argument("--num_envs", type=int, default=1, help="并行环境数量")

# 添加AppLauncher的参数 (这会自动添加--headless等参数)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# 启动Isaac Sim
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 现在可以安全地导入Isaac Lab模块
append_acc_source_dir()
try:
    from acc.tasks.manager_based.acc.acc_env_cfg import AccEnvCfg, AccSceneCfg
    from isaaclab.envs import ManagerBasedRLEnv
    print("✓ 成功导入模块")
except Exception as e:
    print(f"✗ 导入模块失败: {e}")
    simulation_app.close()
    exit(1)


def test_environment():
    """测试环境初始化"""
    
    print("=" * 60)
    print("开始环境初始化测试...")
    print("=" * 60)
    
    try:
        # 1. 创建场景配置
        print("1. 创建场景配置...")
        scene_cfg = AccSceneCfg(num_envs=args.num_envs, env_spacing=4.0)
        print(f"   ✓ 场景配置创建成功 (环境数: {args.num_envs})")
        
        # 2. 创建环境配置
        print("2. 创建环境配置...")
        env_cfg = AccEnvCfg(scene=scene_cfg)
        print("   ✓ 环境配置创建成功")
        
        # 3. 初始化环境
        print("3. 初始化环境...")
        env = ManagerBasedRLEnv(cfg=env_cfg)
        print("   ✓ 环境初始化成功")
        
        # 4. 检查动作和观测空间
        print("4. 检查动作和观测空间...")
        print(f"   动作空间: {env.action_space}")
        print(f"   观测空间: {env.observation_space}")
        
        # 5. 重置环境
        print("5. 重置环境...")
        obs, _ = env.reset()
        print(f"   ✓ 环境重置成功")
        
        # 检查观测维度
        if isinstance(obs, dict):
            policy_obs = obs["policy"]
            print(f"   观测维度: {policy_obs.shape}")
        else:
            print(f"   观测维度: {obs.shape}")
        
        # 6. 执行一步随机动作
        print("6. 执行随机动作...")
        action = env.action_space.sample()
        print(f"   动作维度: {action.shape}")
        
        obs, rewards, terminated, truncated, info = env.step(action)
        print(f"   ✓ 动作执行成功")
        print(f"   奖励: {rewards}")
        
        # 7. 清理
        print("7. 清理环境...")
        env.close()
        print("   ✓ 环境清理完成")
        
        print("\n" + "=" * 60)
        print("环境测试成功完成!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n✗ 环境测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True


def main():
    """主函数"""
    
    try:
        success = test_environment()
        if success:
            print("\n环境配置正常，可以运行数据采集脚本。")
        else:
            print("\n环境配置存在问题，请检查错误信息。")
            
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保仿真正确关闭
        simulation_app.close()


if __name__ == "__main__":
    main()
