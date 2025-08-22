#!/usr/bin/env python3
"""
Gibson 场景转换工具 - 直接在 Isaac Sim 中运行
将 Gibson OBJ 文件转换为 USD 格式
"""
from isaaclab.app import AppLauncher

# launch Kit (use headless=True if you don't need a window)
app_launcher = AppLauncher(headless=True)

import os
import sys

def convert_gibson_obj_to_usd(scene_name):
    """
    将 Gibson OBJ 场景转换为 USD 格式
    直接在 Isaac Sim Python 环境中运行
    """
    # 导入 Isaac Lab 模块
    try:
        from isaaclab.sim.converters.mesh_converter import MeshConverter, MeshConverterCfg
        from isaaclab.sim.schemas import schemas_cfg  # 可选：用于碰撞配置
        from pxr import Usd, UsdGeom, UsdPhysics
        print("✅ Isaac Lab MeshConverter 模块导入成功")
    except ImportError as e:
        print(f"❌ Isaac Lab 模块导入失败: {e}")
        print("请确保在 Isaac Lab 环境中运行此脚本")
        return False
    
    # 检查 Gibson 场景路径
    gibson_path = f"/home/vergil/GibsonEnv/gibson/assets/dataset/{scene_name}"
    
    if not os.path.exists(gibson_path):
        print(f"❌ Gibson 场景不存在: {scene_name}")
        return False
    
    # 检查 OBJ 文件
    obj_files = [
        os.path.join(gibson_path, "mesh_z_up.obj"),
        os.path.join(gibson_path, "mesh.obj")
    ]
    
    obj_file = None
    for f in obj_files:
        if os.path.exists(f):
            obj_file = f
            break
    
    if not obj_file:
        print(f"❌ 未找到 OBJ 文件: {scene_name}")
        return False
    
    # 创建输出目录
    output_dir = f"gibson_usd/{scene_name}"
    os.makedirs(output_dir, exist_ok=True)
    
    output_usd = os.path.join(output_dir, f"{scene_name}.usd")
    output_usd = os.path.abspath(output_usd)
    
    print(f"🔄 转换 {scene_name}")
    print(f"   输入: {obj_file}")
    print(f"   输出: {output_usd}")
    
    try:
        print("📥 使用 Isaac Lab MeshConverter 转换 OBJ 文件...")
        
        # 配置 MeshConverter
        cfg = MeshConverterCfg(
            asset_path=obj_file,
            usd_dir=os.path.dirname(output_usd),
            usd_file_name=os.path.basename(output_usd),  # 注意用 usd_file_name（带或不带 .usd 都行）
            make_instanceable=False,
            # 可选：让转换时就写入碰撞信息（否则你后面再遍历 Mesh 添加也行）
            collision_props=schemas_cfg.CollisionPropertiesCfg(), 
            collision_approximation="convexDecomposition",
        )
                
        print(f"🔧 转换配置:")
        print(f"   资源路径: {cfg.asset_path}")
        print(f"   输出目录: {cfg.usd_dir}")
        print(f"   文件名: {cfg.usd_file_name}")

        # 执行转换
        conv = MeshConverter(cfg)

        print(f"✅ MeshConverter 转换完成: {conv.usd_path}")
        
        # 检查生成的文件
        if os.path.exists(conv.usd_path):
            file_size = os.path.getsize(conv.usd_path)
            print(f"📏 生成的 USD 文件大小: {file_size:,} 字节")
            
            if file_size > 10000:  # 大于 10KB 说明包含了实际数据
                print("✅ USD 文件包含实际网格数据")
            else:
                print("⚠️ USD 文件较小，可能只包含引用")
        else:
            print(f"❌ 转换后的文件不存在: {conv.usd_path}")
            return False
        
        # 可选：添加物理属性到生成的 USD 文件
        try:
            stage = Usd.Stage.Open(conv.usd_path)
            if stage:
                # 查找网格 prim 并添加物理属性
                for prim in stage.Traverse():
                    if prim.GetTypeName() == "Mesh":
                        UsdPhysics.CollisionAPI.Apply(prim)
                        print(f"✅ 为 {prim.GetPath()} 添加碰撞属性")
                
                stage.Save()
                print("✅ 物理属性添加完成")
        except Exception as e:
            print(f"⚠️ 添加物理属性失败: {e}")
        
        return True
        
    except Exception as e:
        print(f"❌ 转换过程中出错: {e}")
        return False


def batch_convert_scenes(scene_list=None):
    """
    批量转换 Gibson 场景
    """
    if scene_list is None:
        # 默认转换一些推荐场景
        scene_list = ["Allensville", "Coffeen", "space7", "test"]
    
    print(f"=== 批量转换 {len(scene_list)} 个 Gibson 场景 ===")
    
    success_count = 0
    failed_scenes = []
    
    for scene_name in scene_list:
        print(f"\n{'='*50}")
        print(f"转换场景: {scene_name}")
        print(f"{'='*50}")
        
        success = convert_gibson_obj_to_usd(scene_name)
        
        if success:
            success_count += 1
            print(f"✅ {scene_name} 转换成功")
        else:
            failed_scenes.append(scene_name)
            print(f"❌ {scene_name} 转换失败")
    
    # 总结
    print(f"\n{'='*50}")
    print(f"转换完成: {success_count}/{len(scene_list)} 个场景成功")
    
    if success_count > 0:
        print(f"✅ 成功场景数: {success_count}")
        print("现在可以在 Isaac Sim 中使用这些场景!")
        
        # 创建使用说明
        create_usage_instructions()
        
    if failed_scenes:
        print(f"❌ 失败场景: {', '.join(failed_scenes)}")
    
    return success_count > 0

def create_usage_instructions():
    """
    创建使用说明文件
    """
    instructions = """# Gibson USD 场景使用说明

## 已转换的场景

转换后的 USD 文件位于 `gibson_usd/` 目录中。

## 使用方法

### 1. 在项目中使用
```bash
# 使用转换后的 Gibson 场景
~/IsaacLab/isaaclab.sh -p isaac_go2_ros2.py --env_name gibson-Allensville
~/IsaacLab/isaaclab.sh -p isaac_go2_ros2.py --env_name gibson-Coffeen
```

### 2. 检查转换结果
```bash
# 查看已转换的场景
ls gibson_usd/
```

### 3. 重新转换场景
如果需要重新转换，请运行:
```bash
~/IsaacLab/isaaclab.sh -p tools/convert_gibson_scene.py
```

## 注意事项

1. 转换后的 USD 文件包含完整的 3D 网格
2. 已添加物理碰撞属性
3. 支持 Isaac Sim 的实时渲染
4. 与 ROS2 完全兼容
"""
    
    with open("GIBSON_USD_USAGE.md", 'w') as f:
        f.write(instructions)
    
    print("📝 使用说明已创建: GIBSON_USD_USAGE.md")

def main():
    if len(sys.argv) < 2:
        print("=== Gibson 场景转换工具 ===")
        print("用法选项:")
        print("1. 转换单个场景:")
        print("   ~/IsaacLab/isaaclab.sh -p tools/convert_gibson_scene.py <scene_name>")
        print("   例如: ~/IsaacLab/isaaclab.sh -p tools/convert_gibson_scene.py Allensville")
        print()
        print("2. 批量转换推荐场景:")
        print("   ~/IsaacLab/isaaclab.sh -p tools/convert_gibson_scene.py batch")
        print()
        print("可用场景列表:")
        gibson_dir = "/home/vergil/GibsonEnv/gibson/assets/dataset"
        if os.path.exists(gibson_dir):
            scenes = [d for d in os.listdir(gibson_dir) 
                     if os.path.isdir(os.path.join(gibson_dir, d))][:10]
            for i, scene in enumerate(scenes, 1):
                print(f"   {i:2d}. {scene}")
            if len(scenes) == 10:
                print("   ... 还有更多场景")
        
        return
    
    if sys.argv[1] == "batch":
        # 批量转换
        batch_convert_scenes()
    else:
        # 单个场景转换
        scene_name = sys.argv[1]
        
        print(f"=== Gibson 场景转换: {scene_name} ===")
        
        success = convert_gibson_obj_to_usd(scene_name)
        
        if success:
            print("🎉 转换完成！")
            print(f"现在可以使用: --env_name gibson-{scene_name}")
            create_usage_instructions()
        else:
            print("💡 转换失败，将使用基础环境")

if __name__ == "__main__":
    main()
