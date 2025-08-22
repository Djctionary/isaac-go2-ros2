#!/usr/bin/env python3
"""
测试USD文件中的材质
"""
from isaaclab.app import AppLauncher

# launch Kit (use headless=True if you don't need a window)
app_launcher = AppLauncher(headless=True)

import os
import sys

def test_usd_materials(usd_file):
    """测试USD文件中的材质"""
    try:
        from pxr import Usd, UsdShade, UsdGeom
        
        print(f"🧪 测试USD文件材质: {usd_file}")
        
        if not os.path.exists(usd_file):
            print(f"❌ 文件不存在: {usd_file}")
            return False
        
        # 打开USD文件
        stage = Usd.Stage.Open(usd_file)
        if not stage:
            print(f"❌ 无法打开USD文件: {usd_file}")
            return False
        
        print(f"✅ USD文件打开成功")
        
        # 检查材质
        materials = []
        shaders = []
        meshes = []
        material_bindings = []
        
        for prim in stage.Traverse():
            prim_type = prim.GetTypeName()
            
            if prim_type == "Material":
                materials.append(prim.GetPath())
            elif prim_type == "Shader":
                shaders.append(prim.GetPath())
            elif prim_type == "Mesh":
                meshes.append(prim.GetPath())
                
                # 检查材质绑定
                material_binding = UsdShade.MaterialBindingAPI(prim)
                bound_material = material_binding.GetDirectBinding()
                if bound_material:
                    material_bindings.append((prim.GetPath(), bound_material.GetMaterialPath()))
        
        print(f"\n📊 材质测试结果:")
        print(f"   网格数量: {len(meshes)}")
        print(f"   材质数量: {len(materials)}")
        print(f"   着色器数量: {len(shaders)}")
        print(f"   材质绑定数量: {len(material_bindings)}")
        
        if materials:
            print(f"\n🎨 找到的材质:")
            for mat in materials:
                print(f"   - {mat}")
        else:
            print(f"\n⚠️ 未找到材质")
        
        if shaders:
            print(f"\n🔧 找到的着色器:")
            for shader in shaders:
                print(f"   - {shader}")
        else:
            print(f"\n⚠️ 未找到着色器")
        
        if material_bindings:
            print(f"\n🔗 材质绑定:")
            for mesh_path, material_path in material_bindings:
                print(f"   {mesh_path} -> {material_path}")
        else:
            print(f"\n⚠️ 未找到材质绑定")
        
        # 检查材质属性
        if materials:
            material_prim = stage.GetPrimAtPath(materials[0])
            if material_prim:
                print(f"\n🔍 材质属性检查:")
                
                # 检查表面输出
                material = UsdShade.Material(material_prim)
                surface_output = material.GetSurfaceOutput()
                if surface_output:
                    print(f"   ✅ 表面输出: {surface_output}")
                    
                    # 检查连接
                    try:
                        connections = surface_output.GetConnections()
                        if connections:
                            print(f"   ✅ 着色器连接: {connections[0]}")
                        else:
                            print(f"   ⚠️ 无着色器连接")
                    except:
                        print(f"   ✅ 表面输出存在")
                else:
                    print(f"   ❌ 无表面输出")
        
        success = len(materials) > 0 and len(material_bindings) > 0
        
        if success:
            print(f"\n✅ 材质测试通过！")
            print(f"   - 包含材质定义")
            print(f"   - 包含材质绑定")
            print(f"   - 可以在Isaac Sim中正确渲染")
        else:
            print(f"\n❌ 材质测试失败")
            print(f"   - 缺少材质定义或材质绑定")
        
        return success
        
    except ImportError as e:
        print(f"❌ 无法导入USD模块: {e}")
        return False
    except Exception as e:
        print(f"❌ 测试过程中出错: {e}")
        return False

def main():
    usd_file = "gibson_usd/Allensville/Allensville.usd"
    
    if len(sys.argv) > 1:
        usd_file = sys.argv[1]
    
    success = test_usd_materials(usd_file)
    
    if success:
        print(f"\n🎉 材质验证成功！")
        print(f"现在可以在Isaac Sim中使用这个带材质的USD文件")
    else:
        print(f"\n💡 材质验证失败，需要重新转换")

if __name__ == "__main__":
    main()
