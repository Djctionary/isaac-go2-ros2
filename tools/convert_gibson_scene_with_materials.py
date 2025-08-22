#!/usr/bin/env python3
"""
Gibson 场景转换工具 - 支持材质转换
将 Gibson OBJ 文件转换为 USD 格式，并添加材质
"""
from isaaclab.app import AppLauncher

# launch Kit (use headless=True if you don't need a window)
app_launcher = AppLauncher(headless=True)

import os
import sys

def create_default_materials(stage):
    """
    创建默认材质
    """
    try:
        from pxr import Usd, UsdShade, Sdf, Gf
        
        # 创建材质
        material = UsdShade.Material.Define(stage, '/World/Materials/DefaultMaterial')
        
        # 创建着色器
        shader = UsdShade.Shader.Define(stage, '/World/Materials/DefaultMaterial/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        
        # 设置着色器参数
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.8, 0.8, 0.8))
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.5)
        shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(1.0)
        
        # 连接着色器到材质
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        
        return material
        
    except Exception as e:
        print(f"⚠️ 创建默认材质失败: {e}")
        return None

def add_materials_to_usd(usd_file):
    """
    为USD文件添加材质
    """
    try:
        from pxr import Usd, UsdShade, UsdGeom
        
        print(f"🎨 为USD文件添加材质: {usd_file}")
        
        # 打开USD文件
        stage = Usd.Stage.Open(usd_file)
        if not stage:
            print(f"❌ 无法打开USD文件: {usd_file}")
            return False
        
        # 创建默认材质
        material = create_default_materials(stage)
        if not material:
            return False
        
        # 为所有网格添加材质绑定
        mesh_count = 0
        for prim in stage.Traverse():
            if prim.GetTypeName() == "Mesh":
                # 应用材质绑定API
                material_binding = UsdShade.MaterialBindingAPI.Apply(prim)
                # 绑定材质
                material_binding.Bind(material)
                mesh_count += 1
        
        # 保存文件
        stage.Save()
        print(f"✅ 为 {mesh_count} 个网格添加了材质绑定")
        
        return True
        
    except Exception as e:
        print(f"❌ 添加材质失败: {e}")
        return False

def convert_gibson_obj_to_usd_with_materials(scene_name):
    """
    将 Gibson OBJ 场景转换为 USD 格式，并添加材质
    """
    # 导入 Isaac Lab 模块
    try:
        from isaaclab.sim.converters.mesh_converter import MeshConverter, MeshConverterCfg
        from isaaclab.sim.schemas import schemas_cfg
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
    
    print(f"🔄 转换 {scene_name} (带材质)")
    print(f"   输入: {obj_file}")
    print(f"   输出: {output_usd}")
    
    try:
        print("📥 使用 Isaac Lab MeshConverter 转换 OBJ 文件...")
        
        # 配置 MeshConverter
        cfg = MeshConverterCfg(
            asset_path=obj_file,
            usd_dir=os.path.dirname(output_usd),
            usd_file_name=os.path.basename(output_usd),
            make_instanceable=False,
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
            
            if file_size > 10000:
                print("✅ USD 文件包含实际网格数据")
            else:
                print("⚠️ USD 文件较小，可能只包含引用")
        else:
            print(f"❌ 转换后的文件不存在: {conv.usd_path}")
            return False
        
        # 添加材质
        print("🎨 添加材质到USD文件...")
        if add_materials_to_usd(conv.usd_path):
            print("✅ 材质添加成功")
        else:
            print("⚠️ 材质添加失败，但转换仍然成功")
        
        # 添加物理属性
        try:
            stage = Usd.Stage.Open(conv.usd_path)
            if stage:
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

def main():
    if len(sys.argv) < 2:
        print("=== Gibson 场景转换工具 (带材质) ===")
        print("用法:")
        print("   ~/IsaacLab/isaaclab.sh -p tools/convert_gibson_scene_with_materials.py <scene_name>")
        print("   例如: ~/IsaacLab/isaaclab.sh -p tools/convert_gibson_scene_with_materials.py Allensville")
        return
    
    scene_name = sys.argv[1]
    
    print(f"=== Gibson 场景转换 (带材质): {scene_name} ===")
    
    success = convert_gibson_obj_to_usd_with_materials(scene_name)
    
    if success:
        print("🎉 转换完成！")
        print(f"现在可以使用: --env_name gibson-{scene_name}")
        print("✅ USD文件现在包含材质信息")
    else:
        print("💡 转换失败")

if __name__ == "__main__":
    main()
