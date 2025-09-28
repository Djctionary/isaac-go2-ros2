from isaaclab.scene import InteractiveSceneCfg
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

from isaaclab.sensors import RayCasterCfg, patterns, ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.noise import UniformNoiseCfg
from isaacsim.core.utils.viewports import set_camera_view
import isaacsim.core.utils.prims as prim_utils
import numpy as np
from scipy.spatial.transform import Rotation as R
import go2.go2_ctrl as go2_ctrl
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass
import torch

# Human obstacle movement controller for RigidObjectCfg-based system
class HumanMovementController:
    """Controls movement of human obstacles using IsaacLab RigidObjectCfg system"""
    
    def __init__(self, num_humans=3, movement_pattern="sinusoidal", amplitude=3.0, frequency=0.3):
        self.num_humans = num_humans
        self.movement_pattern = movement_pattern
        self.amplitude = amplitude
        self.frequency = frequency
        self.simulation_time = 0.0
        
        # Define initial positions for humans
        self.initial_positions = [
            (5.0, 8.0, 0.9), 
            (-6.0, 5.0, 0.9), 
            (8.0, -5.0, 0.9)
        ][:num_humans]  # Take only the required number
    
    def update_positions(self, env, dt: float):
        """Update human positions using IsaacLab's standard asset management"""
        self.simulation_time += dt
        
        try:
            # Get the three independent human obstacles from the scene
            human_1 = env.unwrapped.scene["human_obstacle_1"]
            human_2 = env.unwrapped.scene["human_obstacle_2"]
            human_3 = env.unwrapped.scene["human_obstacle_3"]
            human_objects = [human_1, human_2, human_3]
            
            # Update each human obstacle independently
            for i, human_obj in enumerate(human_objects[:self.num_humans]):
                pos = self._calculate_movement_position(i)
                orientation = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion (w,x,y,z)
                
                # Convert to tensor for single object
                position_tensor = torch.tensor([pos], device=env.device, dtype=torch.float32)
                orientation_tensor = torch.tensor([orientation], device=env.device, dtype=torch.float32)
                
                # Update position using IsaacLab's standard method
                human_obj.write_root_pose_to_sim(
                    root_pose=torch.cat([position_tensor, orientation_tensor], dim=-1)
                )
            
        except Exception as e:
            print(f"⚠️ 更新人形障碍物位置失败: {e}")
    
    def _calculate_movement_position(self, human_id: int) -> Tuple[float, float, float]:
        """Calculate new position for a human obstacle"""
        if human_id >= len(self.initial_positions):
            return (0.0, 0.0, 0.9)
            
        original_pos = self.initial_positions[human_id]
        time = self.simulation_time
        
        if self.movement_pattern == "sinusoidal":
            # Different movement patterns for each human
            if human_id == 0:
                x = original_pos[0] + self.amplitude * math.sin(time * self.frequency)
                y = original_pos[1]
            elif human_id == 1:
                x = original_pos[0]
                y = original_pos[1] + self.amplitude * math.sin(time * self.frequency * 1.3)
            else:
                x = original_pos[0] + self.amplitude * math.sin(time * self.frequency)
                y = original_pos[1] + self.amplitude * math.cos(time * self.frequency)
                
        elif self.movement_pattern == "circular":
            # Circular movement
            radius = self.amplitude
            x = original_pos[0] + radius * math.cos(time * self.frequency)
            y = original_pos[1] + radius * math.sin(time * self.frequency)
            
        elif self.movement_pattern == "linear":
            # Linear movement
            x = original_pos[0] + self.amplitude * math.sin(time * self.frequency)
            y = original_pos[1]
            
        else:
            # Static position
            x, y = original_pos[0], original_pos[1]
        
        z = original_pos[2]  # Keep height constant
        return (x, y, z)
    
    def reset(self, env):
        """Reset human obstacles to initial positions"""
        self.simulation_time = 0.0
        
        try:
            # Get the three independent human obstacles from the scene
            human_1 = env.unwrapped.scene["human_obstacle_1"]
            human_2 = env.unwrapped.scene["human_obstacle_2"]
            human_3 = env.unwrapped.scene["human_obstacle_3"]
            human_objects = [human_1, human_2, human_3]
            
            # Reset each human obstacle independently
            for i, human_obj in enumerate(human_objects[:self.num_humans]):
                initial_pos = self.initial_positions[i]
                orientation = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion (w,x,y,z)
                
                # Convert to tensor for single object
                position_tensor = torch.tensor([initial_pos], device=env.device, dtype=torch.float32)
                orientation_tensor = torch.tensor([orientation], device=env.device, dtype=torch.float32)
                
                # Reset position using IsaacLab's standard method
                human_obj.write_root_pose_to_sim(
                    root_pose=torch.cat([position_tensor, orientation_tensor], dim=-1)
                )
            
            print("✅ 已重置人形障碍物到初始位置")
            
        except Exception as e:
            print(f"⚠️ 重置人形障碍物位置失败: {e}")

# Global movement controller instance
_movement_controller = None

def get_human_movement_controller():
    """Get the global human movement controller"""
    global _movement_controller
    return _movement_controller

def create_human_obstacle_system(cfg=None):
    """Create human obstacle system using RigidObjectCfg"""
    global _movement_controller
    
    try:
        # Create movement controller with default or custom parameters
        if cfg is not None and hasattr(cfg, 'movement_pattern'):
            _movement_controller = HumanMovementController(
                num_humans=3,
                movement_pattern=cfg.movement_pattern,
                amplitude=getattr(cfg, 'movement_amplitude', 3.0),
                frequency=getattr(cfg, 'movement_frequency', 0.3)
            )
        else:
            _movement_controller = HumanMovementController()
        
        print("✅ 人形障碍物运动控制器创建成功")
        return True
        
    except Exception as e:
        print(f"❌ 创建人形障碍物系统失败: {e}")
        _movement_controller = None
        return False

def update_professional_dynamic_obstacles(dt):
    """Update dynamic obstacles using the movement controller"""
    global _movement_controller
    if _movement_controller is not None:
        # This will be called from the main simulation loop with env parameter
        pass  # Actual update happens in the main loop

def reset_human_obstacles():
    """Reset human obstacles to initial positions"""
    global _movement_controller
    if _movement_controller is not None:
        # This will be called from the main simulation loop with env parameter
        pass  # Actual reset happens in the main loop


@configclass
class Go2SimCfg(InteractiveSceneCfg):
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(color=(0.1, 0.1, 0.1), size=(300.0, 300.0)),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0, 0, 1e-4)
        )
    )
    
    # lights
    # Lights
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    # dome_light = AssetBaseCfg(
    #     prim_path="/World/DomeLight",
    #     spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    # )

    # Go2 Robot
    unitree_go2: ArticulationCfg = UNITREE_GO2_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Go2",
        # spawn=UNITREE_GO2_CFG.spawn.replace(
        #     scale=(0.2, 0.2, 0.2), 
        # ),
        init_state=UNITREE_GO2_CFG.init_state.replace(
            pos=(0.0, 0.0, 0.2),
        )
    )
    # Go2 foot contact sensor
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Go2/.*_foot", history_length=3, track_air_time=True)

    # Go2 height scanner - 使用默认地面，避免Gibson路径错误
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Go2/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20)), 
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]), 
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],  # 只使用默认地面，确保路径有效
    )

    human_obstacle_1: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/HumanObstacles/Human_01",
        spawn=sim_utils.CapsuleCfg(
            radius=0.2, height=1.6, axis="Z",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,            # 动态刚体
                kinematic_enabled=False,            # 由物理驱动（别设成kinematic）
                solver_position_iteration_count=16, # 稳定性
                solver_velocity_iteration_count=2,
                max_depenetration_velocity=5.0
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.9, 0.2, 0.2)  # 红色
            )
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(5.0, 8.0, 0.9))
    )

    human_obstacle_2: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/HumanObstacles/Human_02", 
        spawn=sim_utils.CapsuleCfg(
            radius=0.2, height=1.6, axis="Z",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,            # 动态刚体
                kinematic_enabled=False,            # 由物理驱动（别设成kinematic）
                solver_position_iteration_count=16, # 稳定性
                solver_velocity_iteration_count=2,
                max_depenetration_velocity=5.0
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.2, 0.9, 0.2)  # 绿色
            )
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-6.0, 5.0, 0.9))
    )

    human_obstacle_3: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/HumanObstacles/Human_03",
        spawn=sim_utils.CapsuleCfg(
            radius=0.2, height=1.6, axis="Z", 
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,            # 动态刚体
                kinematic_enabled=False,            # 由物理驱动（别设成kinematic）
                solver_position_iteration_count=16, # 稳定性
                solver_velocity_iteration_count=2,
                max_depenetration_velocity=5.0
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
                contact_offset=0.01,
                rest_offset=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.2, 0.2, 0.9)  # 蓝色
            )
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(8.0, -5.0, 0.9))
    )
    
@configclass
class ActionsCfg:
    """Action specifications for the environment."""
    joint_pos = mdp.JointPositionActionCfg(asset_name="unitree_go2", joint_names=[".*"])

@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel,
                               params={"asset_cfg": SceneEntityCfg(name="unitree_go2")})
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel,
                               params={"asset_cfg": SceneEntityCfg(name="unitree_go2")})
        projected_gravity = ObsTerm(func=mdp.projected_gravity,
                                    params={"asset_cfg": SceneEntityCfg(name="unitree_go2")},
                                    noise=UniformNoiseCfg(n_min=-0.05, n_max=0.05))
        # velocity command
        base_vel_cmd = ObsTerm(func=go2_ctrl.base_vel_cmd)

        joint_pos = ObsTerm(func=mdp.joint_pos_rel,
                            params={"asset_cfg": SceneEntityCfg(name="unitree_go2")})
        joint_vel = ObsTerm(func=mdp.joint_vel_rel,
                            params={"asset_cfg": SceneEntityCfg(name="unitree_go2")})
        actions = ObsTerm(func=mdp.last_action)
        
        # Height scan
        height_scan = ObsTerm(func=mdp.height_scan,
                              params={"sensor_cfg": SceneEntityCfg("height_scanner")},
                              clip=(-1.0, 1.0))

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()

@configclass
class CommandsCfg:
    """Command specifications for the MDP."""
    base_vel_cmd = mdp.UniformVelocityCommandCfg(
        asset_name="unitree_go2",
        resampling_time_range=(0.0, 0.0),
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0), heading=(0, 0)
        ),
    )

@configclass
class EventCfg:
    """Configuration for events."""
    pass

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    pass


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    pass

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    pass



@configclass
class Go2RSLEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the Go2 environment."""
    # scene settings
    scene = Go2SimCfg(num_envs=2, env_spacing=2.0)

    # basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()
    
    # dummy settings
    commands = CommandsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    events = EventCfg()
    curriculum = CurriculumCfg()

    def __post_init__(self):
        # viewer settings
        self.viewer.eye = [-4.0, 0.0, 5.0]
        self.viewer.lookat = [0.0, 0.0, 0.0]

        # step settings
        self.decimation = 8  # step

        # simulation settings
        self.sim.dt = 0.005  # sim step every 
        self.sim.render_interval = self.decimation  
        self.sim.disable_contact_processing = True
        self.sim.render.antialiasing_mode = None
        # self.sim.physics_material = self.scene.terrain.physics_material

        # settings for rsl env control
        self.episode_length_s = 20.0 # can be ignored
        self.is_finite_horizon = False
        self.actions.joint_pos.scale = 0.25

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt

def camera_follow(env, distance=8.0, pitch_deg=45.0):
    """摄像头跟随机器人
    Args:
        env: 环境对象
        distance: 摄像头距离机器人的距离（可调节）
        pitch_deg: 摄像头俯仰角（度），相对水平线向上为正。
    """
    if (env.unwrapped.scene.num_envs == 1):
        robot_position = env.unwrapped.scene["unitree_go2"].data.root_state_w[0, :3].cpu().numpy()
        robot_orientation = env.unwrapped.scene["unitree_go2"].data.root_state_w[0, 3:7].cpu().numpy()
        rotation = R.from_quat([robot_orientation[1], robot_orientation[2], 
                                robot_orientation[3], robot_orientation[0]])
        yaw = rotation.as_euler('zyx')[0]
        yaw_rotation = R.from_euler('z', yaw).as_matrix()
        
        # 计算摄像头位置，使用可调节的距离与俯仰角
        elev_rad = np.deg2rad(pitch_deg)
        camera_offset = np.asarray([-distance * np.cos(elev_rad), 0.0, distance * np.sin(elev_rad)])
        camera_position = yaw_rotation.dot(camera_offset) + robot_position
        
        set_camera_view(camera_position, robot_position)