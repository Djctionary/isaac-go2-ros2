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