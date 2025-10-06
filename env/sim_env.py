import os
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
try:
    import isaacsim.storage.native as nucleus_utils
except ModuleNotFoundError:
    import isaacsim.core.utils.nucleus as nucleus_utils
from isaaclab.terrains import TerrainImporterCfg, TerrainImporter
from isaaclab.terrains import TerrainGeneratorCfg
from env.terrain_cfg import HfUniformDiscreteObstaclesTerrainCfg
from go2.self_go2_env import create_human_obstacle_system as create_controller
import omni.replicator.core as rep
try:
    import omni.usd
    import omni.kit.commands
    from pxr import UsdGeom
    from pxr import UsdPhysics
    from pxr import Usd
    from pxr import Sdf
    from pxr import PhysxSchema
    from pxr import UsdShade
except ImportError:
    print("Warning: Omniverse USD modules not available")
    UsdGeom = None
    UsdPhysics = None
    Usd = None
    Sdf = None
    PhysxSchema = None
    UsdShade = None

def add_semantic_label():
    ground_plane = rep.get.prims("/World/GroundPlane")
    with ground_plane:
    # Add a semantic label
        rep.modify.semantics([("class", "floor")])

def create_obstacle_sparse_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=100 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 

def create_obstacle_medium_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=200 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 


def create_obstacle_dense_env():
    add_semantic_label()
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=400,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain)


def create_obstacle_dynamic_env():
    """
    创建专业级动态障碍物环境
    参考无人机导航环境，使用RigidObject创建真正的动态障碍物
    """
    add_semantic_label()
    
    # 基础静态地形 - 使用稳定的配置避免维度错误
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),  # 使用标准尺寸避免维度问题
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=200,  
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain)
    
    # 创建人员障碍物系统 - 使用RigidObjectCfg标准架构
    create_human_obstacle_system()
    
    print("🏗️ 专业级动态障碍物环境已创建")
    print("🎯 包含人员动态障碍物系统")

def create_human_obstacle_system(cfg=None):
    """创建人员障碍物系统 - 使用RigidObjectCfg标准架构"""
    return create_controller(cfg)

def update_professional_dynamic_obstacles(dt, env=None):
    """更新专业级动态障碍物 - 使用RigidObjectCfg标准架构"""
    from go2.self_go2_env import get_human_movement_controller
    controller = get_human_movement_controller()
    if controller is not None and env is not None:
        controller.update_positions(env, dt)

def reset_human_obstacles(env=None):
    """重置人员障碍物到初始位置 - 使用RigidObjectCfg标准架构"""
    from go2.self_go2_env import get_human_movement_controller
    controller = get_human_movement_controller()
    if controller is not None and env is not None:
        controller.reset(env)


def create_warehouse_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_forklifts_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_shelves_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    prim.GetReferences().AddReference(asset_path)

def create_full_warehouse_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_hospital_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Hospital")
    prim = define_prim("/World/Hospital", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Hospital/hospital.usd"
    prim.GetReferences().AddReference(asset_path)

def create_office_env():
    add_semantic_label()
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Office")
    prim = define_prim("/World/Office", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Office/office.usd"
    prim.GetReferences().AddReference(asset_path)

def create_igibson_env(env_name="RS", keep_physics=False):
    """
    导入 iGibson 环境 - 加载 USD 网格
    Args:
        env_name: iGibson 环境名称，如 'RS'
    """
    add_semantic_label()
    
    try:
        print(f"Loading Gibson environment: {env_name}")
        
        # 首先检查是否有转换后的 USD 文件
        usd_file = f"/home/vergil/dataset/igibson/{env_name}/{env_name}.usd"
        gibson_loaded = False
        
        if os.path.exists(usd_file):
            print(f"找到 USD 文件，使用预转换版本: {usd_file}")
            gibson_loaded = load_gibson_usd_with_assembly(env_name, usd_file, keep_physics=keep_physics)
        else:
            print(f"未找到 USD 文件，回退到基础环境")
            return
        
        if gibson_loaded:
            if keep_physics:
                print("✅ Gibson USD 环境加载成功（保留物理）")
            else:
                print("✅ Gibson USD 环境加载成功（房间仅渲染，碰撞靠 ground）")
        else:
            print("⚠️ Gibson USD 环境加载失败，保留默认地面与基础环境")
        
    except Exception as e:
        print(f"Failed to load Gibson environment {env_name}: {e}")
        print("Falling back to basic geometry...")

def create_gibson_env(env_name="Allensville", keep_physics=False):
    """
    导入 Gibson 环境 - 加载 USD 网格
    Args:
        env_name: Gibson 环境名称，如 'Allensville', 'Beechwood', 'Benevolence' 等
    """
    add_semantic_label()
    
    try:
        print(f"Loading Gibson environment: {env_name}")
        
        # 首先检查是否有转换后的 USD 文件
        usd_file = f"gibson_usd/{env_name}/{env_name}.usd"
        gibson_loaded = False
        
        if os.path.exists(usd_file):
            print(f"找到 USD 文件，使用预转换版本: {usd_file}")
            gibson_loaded = load_gibson_usd_with_assembly(env_name, usd_file, keep_physics=keep_physics)
        else:
            print(f"未找到 USD 文件，回退到基础环境")
            return
        
        if gibson_loaded:
            if keep_physics:
                print("✅ Gibson USD 环境加载成功（保留物理）")
            else:
                print("✅ Gibson USD 环境加载成功（房间仅渲染，碰撞靠 ground）")
        else:
            print("⚠️ Gibson USD 环境加载失败，保留默认地面与基础环境")
        
    except Exception as e:
        print(f"Failed to load Gibson environment {env_name}: {e}")
        print("Falling back to basic geometry...")

def load_gibson_usd(env_name, usd_file, keep_physics=True):
    """
    加载转换后的 Gibson USD 文件
    keep_physics: 为 True 时保留 USD 内的物理属性；为 False 时移除物理，使房间仅用于渲染
    Returns:
        bool: 加载是否成功
    """
    try:
        print(f"🎯 加载 Gibson USD 文件: {env_name}")
        
        # 校验路径与文件大小
        if not os.path.isabs(usd_file):
            usd_file = os.path.abspath(usd_file)
        if not os.path.exists(usd_file):
            print(f"❌ USD 文件不存在: {usd_file}")
            return False
        # 检查 USD 文件大小
        file_size = os.path.getsize(usd_file)
        print(f"📏 USD 文件大小: {file_size} 字节")
        
        if file_size < 5000:  # 小于 5KB 说明可能只是引用文件
            print("⚠️ USD 文件太小，可能没有包含实际网格数据；回退到基础环境")
            return False
        
        # 获取 stage
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            print("❌ 无法获取 USD Stage")
            return False
        # 创建/获取 Gibson 环境根节点
        prim = define_prim("/World/Gibson", "Xform")
        if prim is None or not prim.IsValid():
            print("❌ 创建 /World/Gibson Prim 失败")
            return False
        
        # 引用 USD 文件
        abs_usd_file = os.path.abspath(usd_file)
        try:
            prim.GetReferences().AddReference(abs_usd_file)
        except Exception as e:
            print(f"❌ 引用 USD 失败: {e}")
            return False
        
        # 无论是否保留物理，统一设置 Mesh 的 approximation 为 triangleMesh
        #（该操作不会强制开启碰撞，仅设置属性；若后续关闭物理也不会重新启用碰撞）
        set_triangle_mesh_approx_for_subtree(stage, root_path="/World/Gibson")

        # 引入并重绑定渲染材质：
        # 许多 Gibson USD 在文件根下有绝对路径的 /material/... 材质。当以引用方式加载 defaultPrim
        # 到 /World/Gibson 下时，这些根级材质不会被带入，导致原有 material:binding 指向的 /material/... 丢失。
        # 这里显式将源文件的 /material 子树引用到 /World/Gibson/material 下，并将绑定目标重写到新路径。
        try:
            import_gibson_materials(stage, abs_usd_file,
                                    src_material_prim="/material",
                                    dst_material_root="/World/Gibson/material")
            # 覆盖常见材质根的绝对路径前缀
            for old_prefix in ("/material/", "/materials/", "/Looks/"):
                rebind_materials_in_subtree(stage,
                                            root_path="/World/Gibson",
                                            old_prefix=old_prefix,
                                            new_prefix="/World/Gibson/material/")
        except Exception as e:
            print(f"⚠️ 材质引用/重绑定过程出错（跳过）：{e}")
        
        # 根据 keep_physics 控制 Gibson 的物理：
        # False -> 移除物理，仅渲染；True -> 可选地绑定物理材质
        if not keep_physics:
            disable_room_physics(stage, root_path="/World/Gibson")
        else:
            # 保持默认物理材质，仅统一 Gibson 子树的碰撞近似为 triangleMesh
            ensure_triangle_mesh_collision_for_subtree(stage, root_path="/World/Gibson")

        # 简单校验：/World/Gibson 下应至少有一个子 Prim
        gibson_root = stage.GetPrimAtPath("/World/Gibson")
        has_child = False
        if gibson_root and gibson_root.IsValid():
            for child in gibson_root.GetChildren():
                has_child = True
                break
        if not has_child:
            print("⚠️ /World/Gibson 为空，可能引用的是轻量壳层或路径错误")
        
        print(f"✅ Gibson USD 环境加载成功: {env_name}")
        print(f"📍 文件路径: {abs_usd_file}")
        return True  # 成功加载
        
    except Exception as e:
        print(f"❌ USD 加载失败: {e}")
        # 回退到基础环境
        return False  # 加载失败


def load_gibson_usd_with_assembly(env_name, abs_usd_file, keep_physics=True):
    import omni.usd
    from pxr import Usd
    stage = omni.usd.get_context().get_stage()
    # root_layer = stage.GetRootLayer()
    # # 1) 把 Gibson 文件作为 sublayer 叠到根层（让 /Materials 出现在当前 Stage）
    # if abs_usd_file not in root_layer.subLayerPaths:
    #     root_layer.subLayerPaths.append(abs_usd_file)

    # # 2) 在 /World/Gibson 下再引用一次（摆放几何/默认 prim）
    # x = define_prim("/World/Gibson", "Xform")
    # x.GetReferences().ClearReferences()
    # x.GetReferences().AddReference(abs_usd_file)  # 让 defaultPrim 进来


    prim = stage.DefinePrim("/World/Gibson", "Xform")
    prim.GetReferences().ClearReferences()
    prim.GetReferences().AddReference(Sdf.Reference(abs_usd_file))  # 引用 defaultPrim

    # 2) 在根级创建 /Materials，并把源文件的 /Materials 子树引用进来
    mroot = stage.DefinePrim("/Materials", "Scope")  # 或 "Xform"/"MaterialLibrary"，类型无所谓
    mroot.GetReferences().ClearReferences()
    mroot.GetReferences().AddReference(Sdf.Reference(abs_usd_file, "/Materials"))


    # 3) 关闭实例化以便后续可改属性（否则会报“children of an instanced prim cannot be modified”）
    root = stage.GetPrimAtPath("/World/Gibson")
    for p in Usd.PrimRange(root):
        if p.IsInstanceable():
            p.SetInstanceable(False)
    enable_trimesh_collision_for_subtree(root_path="/World/Gibson")
    if not keep_physics:
        disable_room_physics(stage, "/World/Gibson")

    print("✅ 通过 sublayer + reference 加载完成（保留 /Materials，避免重绑）")
    return True

# ------------------------------
# Utilities: 禁用/移除房间物理属性
# ------------------------------
def disable_room_physics(stage, root_path="/World/Gibson"):
    """将指定根下所有Prim设置为仅渲染：
    - 移除/禁用 UsdPhysics.CollisionAPI
    - 移除/禁用 UsdPhysics.RigidBodyAPI
    - 关闭可见的 physics:collisionEnabled 属性（若存在）
    这样房间仅用于渲染，物理碰撞仅由 /World/ground 承担。
    """
    if stage is None or UsdPhysics is None or Usd is None:
        return
    try:
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            return
        
        def _process_prim(prim):
            # 移除刚体/碰撞API
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            # 关闭可能存在的碰撞开关属性
            attr = prim.GetAttribute("physics:collisionEnabled")
            if attr and attr.IsValid():
                attr.Set(False)
        
        # 遍历根及其所有子节点（仅限 root_path 子树）
        for prim in Usd.PrimRange(root_prim):
            _process_prim(prim)
        
        print("✅ 已将房间设置为仅渲染（禁用物理）")
    except Exception as e:
        print(f"⚠️ 禁用房间物理失败: {e}")


def ensure_physical_material(stage, material_path="/World/physical/physicalMaterial",
                             static_friction=0.9, dynamic_friction=0.8, restitution=0.0):
    """在给定路径创建/更新一个 UsdPhysics 材质。
    返回材质 PrimPath 字符串。
    """
    if stage is None or UsdPhysics is None:
        return material_path
    try:
        mat_api = UsdPhysics.MaterialAPI.Define(stage, material_path)
        mat_api.CreateStaticFrictionAttr(static_friction)
        mat_api.CreateDynamicFrictionAttr(dynamic_friction)
        mat_api.CreateRestitutionAttr(restitution)
        return material_path
    except Exception as e:
        print(f"⚠️ 创建物理材质失败: {e}")
        return material_path


def bind_physical_material_to_subtree(stage, root_path="/World/Gibson", material_path="/World/physical/physicalMaterial"):
    """将物理材质绑定到指定子树下所有带碰撞的 Prim。
    绑定关系键：physics:material:binding
    """
    if stage is None:
        return
    try:
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            return
        for prim in Usd.PrimRange(root_prim):
            try:
                # 仅对具有碰撞API或是 Mesh 的几何体进行绑定
                if (UsdPhysics is not None and prim.HasAPI(UsdPhysics.CollisionAPI)) or prim.GetTypeName() == "Mesh":
                    rel = prim.CreateRelationship("physics:material:binding")
                    rel.ClearTargets(removeSpec=False)
                    rel.AddTarget(material_path)
            except Exception:
                continue
        print("✅ 已为 Gibson 子树绑定物理材质")
    except Exception as e:
        print(f"⚠️ 绑定物理材质失败: {e}")


def import_gibson_materials(stage, usd_file, src_material_prim="/material", dst_material_root="/World/Gibson/material"):
    """将源 USD 文件中的材质树引用到当前 Stage 的指定位置。
    会检测常见根：/material, /materials, /Looks，并选择第一个存在的。
    若源文件无这些 prim 或引用失败则静默返回。
    """
    if stage is None or Usd is None:
        return
    try:
        # 打开源 USD 以探测可用的材质根
        src_stage = None
        try:
            src_stage = Usd.Stage.Open(usd_file)
        except Exception:
            src_stage = None
        candidate_roots = [src_material_prim, "/materials", "/Looks"]
        chosen_root = None
        if src_stage is not None:
            for root in candidate_roots:
                p = src_stage.GetPrimAtPath(root)
                if p and p.IsValid() and p.GetChildren():
                    chosen_root = root
                    break
        # 若无法探测到，就退回到默认 /material 直接尝试
        if chosen_root is None:
            chosen_root = src_material_prim

        # 创建/获取目标根并添加引用
        define_prim(dst_material_root, "Xform")
        prim = stage.GetPrimAtPath(dst_material_root)
        if not prim or not prim.IsValid():
            return
        try:
            prim.GetReferences().ClearReferences()
            prim.GetReferences().AddReference(usd_file, chosen_root)
            print(f"✅ 已将材质从 {chosen_root} 引用到 {dst_material_root}")
        except Exception as e:
            print(f"⚠️ 引用源材质失败: {e}")
    except Exception as e:
        print(f"⚠️ 准备材质引用失败: {e}")


def rebind_materials_in_subtree(stage, root_path="/World/Gibson", old_prefix="/material/", new_prefix="/World/Gibson/material/"):
    """将指定子树下所有渲染材质绑定从 old_prefix 改写为 new_prefix。
    - 覆盖 Mesh 与 GeomSubset 以及任意 prim 上的 material:binding* 关系
    - 优先使用关系重写，若有 UsdShade 则尝试二次绑定修复
    """
    if stage is None or Usd is None:
        return
    try:
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            return
        for prim in Usd.PrimRange(root_prim):
            # 方式一：直接改写 material:binding* 关系的目标（适用于 Mesh/GeomSubset/其他带绑定关系的 prim）
            any_changed = False
            for rel in prim.GetRelationships():
                name = rel.GetName()
                if not name.startswith("material:binding"):
                    continue
                try:
                    targets = rel.GetTargets()
                    if not targets:
                        continue
                    new_targets = []
                    changed = False
                    for t in targets:
                        t_path = t.pathString
                        if t_path.startswith(old_prefix):
                            t_path = new_prefix + t_path[len(old_prefix):]
                            changed = True
                        new_targets.append(Sdf.Path(t_path))
                    if changed:
                        rel.ClearTargets(removeSpec=False)
                        for nt in new_targets:
                            rel.AddTarget(nt)
                        any_changed = True
                except Exception:
                    continue

            # 方式二（可选）：若有 UsdShade，可在找不到材质时尝试重新绑定（仅 Mesh 适用）
            if any_changed and 'UsdShade' in globals() and UsdShade is not None and prim.GetTypeName() == "Mesh":
                try:
                    bind_api = UsdShade.MaterialBindingAPI(prim)
                    rel = bind_api.GetDirectBindingRel()
                    if rel and rel.IsValid():
                        targets = rel.GetTargets()
                        if targets:
                            t = targets[0].pathString
                            if t.startswith(old_prefix):
                                new_path = new_prefix + t[len(old_prefix):]
                                mat_prim = stage.GetPrimAtPath(new_path)
                                if mat_prim and mat_prim.IsValid():
                                    material = UsdShade.Material(mat_prim)
                                    if material:
                                        bind_api.Bind(material)
                except Exception:
                    pass
        print("✅ 已重写 Gibson 子树的渲染材质绑定到 /World/Gibson/material")
    except Exception as e:
        print(f"⚠️ 重写渲染材质绑定失败: {e}")

def enable_trimesh_collision_for_subtree(root_path="/World/Gibson", make_static=True):
    """
    Enable PhysX triangle-mesh collision on all Mesh prims under `root_path`.
    - Keeps geometry static by default (no RigidBody); good for rooms/walls.
    - Clears/normalizes USD approximation so there are no warnings.
    Returns: number of Mesh prims updated.
    """
    import omni.usd
    from pxr import Usd, Sdf, UsdPhysics
    try:
        from pxr import PhysxSchema
    except Exception:
        PhysxSchema = None

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"❌ Root prim not found: {root_path}")
        return 0

    # 0) allow edits even if referenced as instance
    for p in Usd.PrimRange(root):
        if p.IsInstanceable():
            p.SetInstanceable(False)

    updated = 0
    valid_approx = {"none","convexHull","convexDecomposition","box","sphere","capsule","cylinder"}

    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "Mesh":
            continue

        # 1) ensure CollisionAPI (no need to add RigidBody for static scene)
        try:
            UsdPhysics.CollisionAPI.Apply(prim)
        except Exception:
            pass

        # 2) optionally force static by removing RigidBodyAPI if present
        if make_static:
            try:
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            except Exception:
                pass

        # 3) normalize USD approximation to avoid warnings (triangleMesh is NOT valid here)
        try:
            a = prim.GetAttribute("physics:approximation")
            if not a or not a.IsValid():
                a = prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token)
            # set to "none" so PhysX will use its own mesh setting without conflicts
            if a.Get() not in valid_approx:
                a.Set("none")
            else:
                a.Set("none")
        except Exception:
            pass

        # also make sure collisionEnabled is true
        try:
            ce = prim.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool)
            ce.Set(True)
        except Exception:
            pass

        # 4) PhysX triangle mesh collision
        if PhysxSchema is not None:
            try:
                physx_col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                physx_col.CreateMeshCollisionAttr().Set("triangleMesh")
            except Exception:
                # if PhysX schema isn't available on this build, we just skip
                pass

        updated += 1

    print(f"✅ Applied triangleMesh collision to {updated} mesh(es) under {root_path}")
    return updated


def set_triangle_mesh_approx_for_subtree(stage, root_path="/World/Gibson"):
    """仅设置子树下所有 Mesh 的 physics:approximation = triangleMesh。
    不启用碰撞 API，也不添加刚体，适用于仅渲染或延后开启碰撞的场景。
    """
    if stage is None or Sdf is None or Usd is None:
        return
    try:
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            return
        for prim in Usd.PrimRange(root_prim):
            if prim.GetTypeName() != "Mesh":
                continue
            try:
                approx_attr = prim.GetAttribute("physics:approximation")
                if not approx_attr or not approx_attr.IsValid():
                    approx_attr = prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token)
                approx_attr.Set("triangleMesh")
            except Exception:
                continue
        print("✅ 已为 Gibson 子树设置 physics:approximation = triangleMesh")
    except Exception as e:
        print(f"⚠️ 设置 triangleMesh approximation 失败: {e}")


def ensure_triangle_mesh_collision_for_subtree(stage, root_path="/World/Gibson"):
    """确保子树下所有 Mesh 的碰撞近似设为 triangleMesh，并应用 PhysX Mesh 碰撞（若可用）。
    不会强制添加 RigidBody，仅设置 CollisionAPI/近似与 PhysX 扩展。
    """
    if stage is None:
        return
    try:
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            return
        for prim in Usd.PrimRange(root_prim):
            if prim.GetTypeName() != "Mesh":
                continue
            # 应用 CollisionAPI
            try:
                UsdPhysics.CollisionAPI.Apply(prim)
            except Exception:
                pass
            # 设置 physics:approximation = triangleMesh
            try:
                approx_attr = prim.GetAttribute("physics:approximation")
                if not approx_attr or not approx_attr.IsValid():
                    # 创建属性，类型为 token
                    approx_attr = prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token)
                approx_attr.Set("triangleMesh")
            except Exception:
                pass
            # PhysX 扩展
            if PhysxSchema is not None:
                try:
                    physx_col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                    attr = physx_col.CreateMeshCollisionAttr()
                    attr.Set("triangleMesh")
                except Exception:
                    pass
        print("✅ 已将 Gibson 子树 Mesh 碰撞近似统一为 triangleMesh")
    except Exception as e:
        print(f"⚠️ 统一 triangleMesh 碰撞失败: {e}")
