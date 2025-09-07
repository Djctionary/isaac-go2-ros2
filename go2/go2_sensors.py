import omni
import numpy as np
from pxr import Gf
import omni.replicator.core as rep
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

class SensorManager:
    def __init__(self, num_envs):
        self.num_envs = num_envs

    def add_rtx_lidar(self):
        lidar_annotators = []
        for env_idx in range(self.num_envs):
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="/lidar",
                parent=f"/World/envs/env_{env_idx}/Go2/base",
                config="Hesai_XT32_SD10",
                # config="Velodyne_VLS128",
                translation=(0.2, 0, 0.2),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
            )

            annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
            hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
            annotator.attach(hydra_texture.path)
            lidar_annotators.append(annotator)
        return lidar_annotators

    def add_camera(self, freq):
        cameras = []
        for env_idx in range(self.num_envs):
            camera = Camera(
                prim_path=f"/World/envs/env_{env_idx}/Go2/base/front_cam",
                translation=np.array([0.2, 0.0, 0.2]),
                frequency=freq,
                resolution=(640, 480),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            )
            camera.initialize()
            # camera.set_focal_length(1.5)
            
            # 通过 USD 属性设置相机参数
            try:
                import omni.usd
                from pxr import UsdGeom
                stage = omni.usd.get_context().get_stage()
                camera_prim = stage.GetPrimAtPath(camera.prim_path)
                if camera_prim and camera_prim.IsValid():
                    # 设置近平面
                    camera_prim.GetAttribute("clippingRange").Set((0.01, 100.0)) 
                    # 设置视野角度
                    # camera_prim.GetAttribute("horizontalAperture").Set(1.5)
                    horiz_aperture = camera_prim.GetAttribute("horizontalAperture").Get()
                    print(f"✅ 已设置相机 {env_idx} 的视野角度: {horiz_aperture}")
            except Exception as e:
                print(f"⚠️ 设置相机参数失败: {e}")
            
            cameras.append(camera)
        return cameras

    def compute_imu_from_physics(self, env, add_noise, accel_noise_std, gyro_noise_std):
        """基于物理量计算IMU读数。

        返回列表，长度为 num_envs，每项为 dict：
        {
            "orientation": np.ndarray shape(4,), wxyz 四元数（与现有里程计保持一致）,
            "angular_velocity": np.ndarray shape(3,), 机体系 rad/s,
            "linear_acceleration": np.ndarray shape(3,), 机体系 m/s^2
        }
        """
        imu_list = []
        for i in range(self.num_envs):
            imu_data = self._compute_imu_from_physics_single(env, i, add_noise, accel_noise_std, gyro_noise_std)
            imu_list.append(imu_data)
        return imu_list

    def _compute_imu_from_physics_single(self, env, env_idx, add_noise, accel_noise_std, gyro_noise_std):
        """为单个环境计算IMU数据（物理方法）"""
        robot_data = env.unwrapped.scene["unitree_go2"].data
        gravity_world = np.array([0.0, 0.0, -9.81], dtype=np.float32)
        
        # 姿态（wxyz）与角速度（机体系）
        quat_wxyz = robot_data.root_state_w[env_idx, 3:7].detach().cpu().numpy()
        ang_vel_b = robot_data.root_ang_vel_b[env_idx].detach().cpu().numpy()

        # 获取世界系线加速度
        if hasattr(robot_data, "root_lin_acc_w"):
            lin_acc_w = robot_data.root_lin_acc_w[env_idx].detach().cpu().numpy()
        else:
            # 如果没有直接的加速度数据，使用速度差分近似
            if hasattr(robot_data, "root_lin_vel_w"):
                lin_vel_w = robot_data.root_lin_vel_w[env_idx].detach().cpu().numpy()
                
                # 实现速度差分计算
                if not hasattr(self, '_prev_velocities'):
                    self._prev_velocities = {}
                if not hasattr(self, '_step_count'):
                    self._step_count = {}
                
                key = f"env_{env_idx}"
                
                # 从环境配置中获取时间步长
                dt = env.unwrapped.cfg.sim.dt * env.unwrapped.cfg.decimation
                
                if key in self._prev_velocities:
                    # 计算速度差分
                    lin_acc_w = (lin_vel_w - self._prev_velocities[key]) / dt
                else:
                    # 第一次运行，加速度为零
                    lin_acc_w = np.zeros(3, dtype=np.float32)
                
                # 保存当前速度
                self._prev_velocities[key] = lin_vel_w.copy()
            else:
                lin_acc_w = np.zeros(3, dtype=np.float32)

        # 正确的重力补偿方法
        qw, qx, qy, qz = quat_wxyz
        
        # 计算旋转矩阵（世界系到机体系）
        R_bw = np.array([
            [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float32)

        # 关键修复：正确的重力补偿
        # 方法：先将世界系加速度转换到机体系，然后减去机体系下的重力
        lin_acc_b = R_bw @ lin_acc_w  # 世界系加速度转换到机体系
        # gravity_b = R_bw @ gravity_world  # 世界系重力转换到机体系
        # lin_acc_b = lin_acc_b - gravity_b  # 减去机体系下的重力
        
        if add_noise:
            lin_acc_b = lin_acc_b + np.random.normal(0.0, accel_noise_std, 3)
            ang_vel_b = ang_vel_b + np.random.normal(0.0, gyro_noise_std, 3)

        return {
            "orientation": quat_wxyz,
            "angular_velocity": ang_vel_b,
            "linear_acceleration": lin_acc_b,
        }