import os
import hydra
import rclpy
import torch
import time
import math
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Go2 Robot Simulation with Gibson Environment Support")

# add custom arguments
parser.add_argument("--env_name", type=str, default=None, help="Environment name (e.g., gibson-Allensville)")
parser.add_argument("--config-name", type=str, default="sim", help="Hydra config name")
FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse known args to handle Hydra config separately
args_cli, unknown_args = parser.parse_known_args()

# handle Hydra config
if hasattr(args_cli, 'config_name') and args_cli.config_name != "sim":
    # Override sys.argv for Hydra
    import sys
    sys.argv = [sys.argv[0], f"--config-name={args_cli.config_name}"] + unknown_args
elif args_cli.env_name:
    # Override sys.argv for direct env_name
    import sys
    sys.argv = [sys.argv[0], f"env_name={args_cli.env_name}"] + unknown_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb.settings
carb.settings.get_settings().set("/app/extensions/omni.sensors.nv.lidar.enabled", False)

"""Rest everything follows."""

import torch

from go2.go2_env import Go2RSLEnvCfg, camera_follow
import env.sim_env as sim_env
import go2.go2_sensors as go2_sensors
import omni
import carb
import go2.go2_ctrl as go2_ctrl
import ros2.go2_ros2_bridge as go2_ros2_bridge


def disable_direct_gpu_api():
    try:
        import carb
        settings = carb.settings.get_settings()
        # 兼容大小写写法
        settings.set("/renderer/enable_direct_GPU_API", False)
        settings.set("/renderer/enable_direct_gpu_api", False)
        print("✅ 已禁用 /renderer/enable_direct_GPU_API")
    except Exception as e:
        print(f"⚠️ 无法禁用 enable_direct_GPU_API: {e}")

def check_and_print_up_axis():
    """检查并打印当前 Stage 的 upAxis。"""
    try:
        from pxr import UsdGeom
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        up_axis = UsdGeom.GetStageUpAxis(stage)
        print(f"🌍 Stage upAxis: {up_axis}")
    except Exception as e:
        print(f"⚠️ 检查 upAxis 失败: {e}")

def set_robot_root_pose(env, x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
    """将机器人根位姿设置到给定位置与航向（仅根，不改关节）。
    """
    try:
        robot = env.unwrapped.scene["unitree_go2"]
        rs = robot.data.root_state_w.clone()
        rs[0, 0:3] = torch.tensor([float(x), float(y), float(z)], device=rs.device)
        # 仅绕Z轴的四元数 (w,x,y,z)
        yaw_rad = math.radians(float(yaw_deg))
        w = math.cos(yaw_rad/2.0)
        zq = math.sin(yaw_rad/2.0)
        rs[0, 3:7] = torch.tensor([w, 0.0, 0.0, zq], device=rs.device)
        rs[0, 7:13] = 0.0
        if hasattr(robot, "write_root_state_to_sim"):
            robot.write_root_state_to_sim(rs)
        elif hasattr(robot, "set_world_poses"):
            pos = rs[0, 0:3].unsqueeze(0).cpu().numpy()
            quat = rs[0, 3:7].unsqueeze(0).cpu().numpy()
            robot.set_world_poses(pos, quat)
        if hasattr(env.unwrapped, 'scene') and hasattr(env.unwrapped.scene, 'write_data_to_sim'):
            env.unwrapped.scene.write_data_to_sim()
        return True
    except Exception as e:
        print(f"⚠️ 设置根位姿失败: {e}")
        return False

def update_height_scanner_for_gibson(go2_env_cfg):
    """为Gibson环境更新高度扫描器配置"""
    try:
        # 检查Gibson环境是否存在
        from isaacsim.core.utils.prims import get_prim_at_path
        gibson_prim = get_prim_at_path("/World/Gibson")
        if gibson_prim and gibson_prim.IsValid():
            # 更新高度扫描器以使用Gibson网格
            go2_env_cfg.scene.height_scanner.mesh_prim_paths = ["/World/Gibson"]
            print("✅ 已更新高度扫描器以使用Gibson环境")
        else:
            print("⚠️ Gibson环境未找到，使用默认地面")
    except Exception as e:
        print(f"⚠️ 更新高度扫描器配置失败: {e}")

@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    disable_direct_gpu_api()
    # Go2 Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    go2_env_cfg.scene.num_envs = cfg.num_envs
    go2_env_cfg.decimation = math.ceil(1./go2_env_cfg.sim.dt/cfg.freq)
    go2_env_cfg.sim.render_interval = go2_env_cfg.decimation
    go2_ctrl.init_base_vel_cmd(cfg.num_envs)
    
    # 预创建动态障碍物Prim - 必须在环境创建前
    if cfg.env_name == "obstacle-dynamic":
        import isaacsim.core.utils.prims as prim_utils
        
        num_envs = go2_env_cfg.scene.num_envs
        for env_id in range(num_envs):
            parent_path = f"/World/envs/env_{env_id}/HumanObstacles"
            prim_utils.create_prim(parent_path, "Xform")
            
            # for human_id in range(3):
            #     human_path = f"{parent_path}/Human_{human_id+1:02d}"
            #     prim_utils.create_prim(human_path, "Capsule")
        
        print("✅ 预创建人形障碍物Prim完成")
    
    # env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)

    # Simulation environment
    if (cfg.env_name == "obstacle-dense"):
        sim_env.create_obstacle_dense_env() # obstacles dense
    elif (cfg.env_name == "obstacle-medium"):
        sim_env.create_obstacle_medium_env() # obstacles medium
    elif (cfg.env_name == "obstacle-sparse"):
        sim_env.create_obstacle_sparse_env() # obstacles sparse
    elif (cfg.env_name == "obstacle-dynamic"):
        sim_env.create_obstacle_dynamic_env() # professional dynamic obstacles
    elif (cfg.env_name == "warehouse"):
        sim_env.create_warehouse_env() # warehouse
    elif (cfg.env_name == "warehouse-forklifts"):
        sim_env.create_warehouse_forklifts_env() # warehouse forklifts
    elif (cfg.env_name == "warehouse-shelves"):
        sim_env.create_warehouse_shelves_env() # warehouse shelves
    elif (cfg.env_name == "full-warehouse"):
        sim_env.create_full_warehouse_env() # full warehouse
    elif (cfg.env_name.startswith("gibson-")):
        # Gibson environment: gibson-{env_name}
        env_name = cfg.env_name.replace("gibson-", "")
        sim_env.create_gibson_env(env_name, keep_physics=True)
        # 为Gibson环境更新高度扫描器配置
        update_height_scanner_for_gibson(go2_env_cfg)  # 暂时禁用，避免路径错误
    elif (cfg.env_name.startswith("igibson-")):
        # iGibson environment: igibson-{env_name}
        env_name = cfg.env_name.replace("igibson-", "")
        sim_env.create_igibson_env(env_name, keep_physics=True)
    
    # 检查并打印当前 Stage 的 upAxis
    check_and_print_up_axis()
    
    # Sensor setup
    sm = go2_sensors.SensorManager(cfg.num_envs)
    lidar_annotators = sm.add_rtx_lidar()
    cameras = sm.add_camera(cfg.freq)

    # Keyboard control for robot and camera
    system_input = carb.input.acquire_input_interface()
    
    # 设置全局变量用于摄像头控制
    global camera_distance_far, camera_distance_near, use_near_camera, manual_camera_control, camera_pitch_deg, pending_reset, reset_freeze_steps, reset_warmup_steps, reset_warmup_total
    camera_distance_far = getattr(cfg.get('camera', {}), 'initial_distance', 8.0)
    camera_distance_near = getattr(cfg.get('camera', {}), 'close_distance', 1.25)
    camera_pitch_deg = getattr(cfg.get('camera', {}), 'pitch_deg', 25.0)
    use_near_camera = False
    manual_camera_control = not cfg.camera_follow
    pending_reset = False
    reset_freeze_steps = 0
    reset_warmup_steps = 0
    reset_warmup_total = 0
    
    # 注册键盘事件处理器
    def keyboard_handler(event):
        go2_ctrl.sub_keyboard_event(event)  # 原有的机器人控制
        
        # 摄像头控制键盘快捷键
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            global camera_distance_far, camera_distance_near, use_near_camera, manual_camera_control, camera_pitch_deg, pending_reset
            
            if event.input == carb.input.KeyboardInput.G:
                # G键切换摄像头跟随模式
                manual_camera_control = not manual_camera_control
                print(f"摄像头跟随模式: {'关闭' if manual_camera_control else '开启'}")
            elif event.input == carb.input.KeyboardInput.H:
                # H键切换近景摄像头
                use_near_camera = not use_near_camera
                mode = '近景' if use_near_camera else '远景'
                dist = camera_distance_near if use_near_camera else camera_distance_far
                print(f"摄像头模式: {mode}（距离: {dist:.1f}）")
                
            elif event.input == carb.input.KeyboardInput.R:
                # R键：请求重置机器人到初始状态
                pending_reset = True
                print("请求重置：将在下一帧重置环境与机器人状态")
            

    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), keyboard_handler)
    
    # ROS2 Bridge
    rclpy.init()
    dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, sm, cfg)

    # Run simulation
    sim_step_dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    obs, _ = env.reset()
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():            
            # 若收到重置请求，则先重置环境与观测
            if pending_reset:
                obs, _ = env.reset()
                # 手动瞬移根位姿
                set_robot_root_pose(env, x=0.0, y=0.0, z=0.2, yaw_deg=0.0)
                
                # 重置动态障碍物位置
                if cfg.env_name == "obstacle-dynamic":
                    try:
                        sim_env.reset_human_obstacles(env)
                    except Exception as e:
                        print(f"⚠️ 重置动态障碍物失败: {e}")
                pending_reset = False
                reset_freeze_steps = 15
                reset_warmup_total = 45
                reset_warmup_steps = reset_warmup_total
                try:
                    if hasattr(go2_ctrl, 'base_vel_cmd_input') and go2_ctrl.base_vel_cmd_input is not None:
                        go2_ctrl.base_vel_cmd_input.zero_()
                except Exception:
                    pass
                print("✅ 已重置（冻结15帧 + 渐入45帧），避免瞬时大扭矩")
            
            # control joints
            actions = policy(obs)
            if 'reset_freeze_steps' in globals() and reset_freeze_steps > 0:
                actions = torch.zeros_like(actions)
                reset_freeze_steps -= 1
            elif 'reset_warmup_steps' in globals() and reset_warmup_steps > 0:
                # 线性放大：从很小逐步到1.0
                factor = float(reset_warmup_total - reset_warmup_steps + 1) / float(max(1, reset_warmup_total))
                actions = actions * factor
                reset_warmup_steps -= 1

            # step the environment
            obs, _, _, _ = env.step(actions)

            # 更新专业级动态障碍物 - 使用RigidObjectCfg标准架构
            if cfg.env_name == "obstacle-dynamic":
                try:
                    sim_env.update_professional_dynamic_obstacles(sim_step_dt, env)
                except Exception as e:
                    print(f"⚠️ 更新动态障碍物失败: {e}")  # 显示错误信息以便调试

            # # ROS2 data
            dm.pub_ros2_data()
            rclpy.spin_once(dm)

            # Camera follow - 根据模式决定是否跟随
            if not manual_camera_control:
                follow_distance = camera_distance_near if use_near_camera else camera_distance_far
                camera_follow(env, distance=follow_distance, pitch_deg=camera_pitch_deg)

            # limit loop time
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, sim_step_dt/elapsed_time)
        # 读取并打印机器人位置（第一个环境）
        try:
            pos = env.unwrapped.scene["unitree_go2"].data.root_state_w[0, :3].cpu().numpy()
            pos_str = f"XYZ=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
        except Exception:
            pos_str = "XYZ=(nan, nan, nan)"
        print(f"\rStep time: {actual_loop_time*1000:.2f}ms, RTF: {rtf:.2f}, {pos_str}", end='', flush=True)
    
    dm.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()
    