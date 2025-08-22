#!/usr/bin/env python3
"""
Gibson 场景转换工具 - 保留原始材质
将 Gibson OBJ 文件转换为 USD 格式，保留原始材质，只在需要时添加默认材质
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

def check_existing_materials(stage):
    """
    检查USD文件中是否已存在材质
    """
    materials = []
    material_bindings = []
    
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Material":
            materials.append(prim.GetPath())
        elif prim.GetTypeName() == "Mesh":
            # 检查材质绑定
            try:
                from pxr import UsdShade
                material_binding = UsdShade.MaterialBindingAPI(prim)
                bound_material = material_binding.GetDirectBinding()
                if bound_material:
                    material_bindings.append((prim.GetPath(), bound_material.GetMaterialPath()))
            except:
                pass
    
    return materials, material_bindings

def _find_mtl_for_obj(obj_file: str):
    """在 OBJ 文件中解析 mtllib 指令或在同目录查找 .mtl 文件。"""
    if not obj_file or not os.path.exists(obj_file):
        return None
    obj_dir = os.path.dirname(obj_file)
    # 1) 从 obj 内容解析 mtllib
    try:
        with open(obj_file, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                if line.strip().lower().startswith('mtllib'):
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        mtl_candidate = os.path.join(obj_dir, parts[1])
                        if os.path.exists(mtl_candidate):
                            return mtl_candidate
    except Exception:
        pass
    # 2) 通用候选名
    candidates = [
        os.path.splitext(obj_file)[0] + '.mtl',
        os.path.join(obj_dir, 'mesh.mtl'),
        os.path.join(obj_dir, 'mesh_z_up.mtl'),
        os.path.join(obj_dir, 'materials.mtl'),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return None


def _parse_mtl(mtl_path: str):
    """简单解析 .mtl，返回材质条目列表：[{name, kd, ks, map_kd}]."""
    materials = []
    if not mtl_path or not os.path.exists(mtl_path):
        return materials
    current = None
    try:
        with open(mtl_path, 'r', encoding='utf-8', errors='ignore') as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                tokens = line.split()
                key = tokens[0].lower()
                if key == 'newmtl' and len(tokens) >= 2:
                    if current:
                        materials.append(current)
                    current = { 'name': tokens[1], 'kd': None, 'ks': None, 'map_kd': None }
                elif key == 'kd' and len(tokens) >= 4 and current is not None:
                    try:
                        current['kd'] = (float(tokens[1]), float(tokens[2]), float(tokens[3]))
                    except Exception:
                        pass
                elif key == 'ks' and len(tokens) >= 4 and current is not None:
                    try:
                        current['ks'] = (float(tokens[1]), float(tokens[2]), float(tokens[3]))
                    except Exception:
                        pass
                elif key == 'map_kd' and len(tokens) >= 2 and current is not None:
                    current['map_kd'] = ' '.join(tokens[1:])
        if current:
            materials.append(current)
    except Exception:
        return materials
    return materials


def _create_usd_materials_from_mtl(stage, mtl_path: str, materials: list, root='/World/Materials/MTL'):
    """基于 MTL 条目在 USD 中创建 UsdShade 材质网络，返回创建的材质 prim 路径列表。"""
    try:
        from pxr import Usd, UsdShade, Sdf, Gf
    except Exception as e:
        print(f"⚠️ 无法导入 Usd/UsdShade 以创建材质: {e}")
        return []

    created_paths = []
    try:
        # 确保根作用域存在
        if not stage.GetPrimAtPath(root):
            Usd.PrimDefine(stage, Sdf.Path(root))
    except Exception:
        pass

    tex_dir = os.path.dirname(mtl_path)
    for mat in materials:
        name = mat.get('name') or 'MTL_Material'
        safe_name = ''.join(c if c.isalnum() or c in ('_', '-') else '_' for c in name)
        mat_path = f"{root}/{safe_name}"
        shader_path = f"{mat_path}/Shader"
        tex_path = f"{mat_path}/Tex"
        st_reader_path = f"{mat_path}/PrimvarSt"
        try:
            material = UsdShade.Material.Define(stage, mat_path)
            shader = UsdShade.Shader.Define(stage, shader_path)
            shader.CreateIdAttr('UsdPreviewSurface')

            kd = mat.get('kd') or (0.8, 0.8, 0.8)
            # 直接颜色输入备用
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*kd))
            shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
            shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.5)
            shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(1.0)

            # 纹理贴图（map_Kd）
            map_kd = mat.get('map_kd')
            if map_kd:
                tex_file = os.path.join(tex_dir, map_kd)
                tex = UsdShade.Shader.Define(stage, tex_path)
                tex.CreateIdAttr('UsdUVTexture')
                tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(tex_file)
                tex.CreateInput('fallback', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*kd))
                tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
                tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')

                # st primvar reader
                st_reader = UsdShade.Shader.Define(stage, st_reader_path)
                st_reader.CreateIdAttr('UsdPrimvarReader_float2')
                st_reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')

                # 连接 st -> tex.uv
                tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader, 'result')
                # 连接 tex.rgb -> shader.diffuseColor
                shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex, 'rgb')

            # 输出连接
            material.CreateSurfaceOutput().ConnectToSource(shader, 'surface')
            created_paths.append(mat_path)
            print(f"✅ 基于 MTL 创建材质: {mat_path}")
        except Exception as e:
            print(f"⚠️ 创建材质失败（{name}）: {e}")
            continue
    return created_paths


def _bind_material_to_all_meshes(stage, material_prim_path: str):
    """将给定材质绑定到舞台内所有 Mesh（作为兜底行为）。"""
    try:
        from pxr import Usd, UsdShade
    except Exception:
        return 0
    try:
        mat_prim = stage.GetPrimAtPath(material_prim_path)
        if not mat_prim or not mat_prim.IsValid():
            return 0
        material = UsdShade.Material(mat_prim)
        count = 0
        for prim in stage.Traverse():
            if prim.GetTypeName() == 'Mesh':
                try:
                    binding = UsdShade.MaterialBindingAPI.Apply(prim)
                    binding.Bind(material)
                    count += 1
                except Exception:
                    continue
        return count
    except Exception:
        return 0


def add_materials_if_needed(usd_file, obj_file: str = None):
    """
    只在需要时为USD文件添加材质
    """
    try:
        from pxr import Usd, UsdShade, UsdGeom
        
        print(f"🎨 检查USD文件材质: {usd_file}")
        
        # 打开USD文件
        stage = Usd.Stage.Open(usd_file)
        if not stage:
            print(f"❌ 无法打开USD文件: {usd_file}")
            return False

        # 优先：若存在 MTL，则基于 MTL 在 USD 中创建材质（若尚未存在）
        mtl_path = _find_mtl_for_obj(obj_file) if obj_file else None
        created_from_mtl = []
        if mtl_path and os.path.exists(mtl_path):
            print(f"🎨 检测到 MTL 文件，优先使用进行材质创建/检查: {mtl_path}")
            entries = _parse_mtl(mtl_path)
            if entries:
                created_from_mtl = _create_usd_materials_from_mtl(stage, mtl_path, entries)
                if created_from_mtl:
                    # 暂不保存，稍后统一在有绑定变化时保存
                    print(f"✅ 基于 MTL 创建了 {len(created_from_mtl)} 个材质")
        
        # 检查现有材质
        materials, material_bindings = check_existing_materials(stage)
        
        print(f"📊 现有材质检查:")
        print(f"   材质数量: {len(materials)}")
        print(f"   材质绑定数量: {len(material_bindings)}")
        
        if materials:
            print(f"🎨 找到的材质:")
            for mat in materials:
                print(f"   - {mat}")
        
        if material_bindings:
            print(f"🔗 材质绑定:")
            for mesh_path, material_path in material_bindings:
                print(f"   {mesh_path} -> {material_path}")
        
        # 查找Gibson材质
        gibson_material = None
        for mat_path in materials:
            if "Gibson" in str(mat_path):
                gibson_material = stage.GetPrimAtPath(mat_path)
                break
        
        # 如果已经有材质绑定，检查是否绑定的是Gibson材质
        if material_bindings:
            # 检查是否绑定的是Gibson材质
            bound_to_gibson = False
            for mesh_path, material_path in material_bindings:
                if "Gibson" in str(material_path):
                    bound_to_gibson = True
                    print(f"✅ 网格已正确绑定到Gibson材质: {mesh_path} -> {material_path}")
            
            if bound_to_gibson:
                print(f"✅ USD文件已包含正确的Gibson材质绑定")
                return True
            else:
                print(f"⚠️ 网格绑定的是默认材质，需要重新绑定到Gibson材质")
        
        # 如果有Gibson材质，绑定到Gibson材质
        if gibson_material:
            print(f"🎨 绑定网格到Gibson材质: {gibson_material.GetPath()}")
            
            # 为所有网格绑定Gibson材质
            mesh_count = 0
            for prim in stage.Traverse():
                if prim.GetTypeName() == "Mesh":
                    # 应用材质绑定API
                    material_binding = UsdShade.MaterialBindingAPI.Apply(prim)
                    # 绑定到Gibson材质
                    gibson_material_obj = UsdShade.Material(gibson_material)
                    material_binding.Bind(gibson_material_obj)
                    mesh_count += 1
            
            # 保存文件
            stage.Save()
            print(f"✅ 为 {mesh_count} 个网格绑定了Gibson材质")
            return True
        
        # 若没有 Gibson 材质，且存在基于 MTL 创建的材质，则兜底绑定第一个 MTL 材质
        if created_from_mtl:
            first_mat = created_from_mtl[0]
            print(f"🎨 未找到 Gibson 材质，使用 MTL 材质兜底绑定: {first_mat}")
            num = _bind_material_to_all_meshes(stage, first_mat)
            stage.Save()
            print(f"✅ 为 {num} 个网格绑定了 MTL 材质")
            return True

        # 如果没有 Gibson 材质与 MTL 材质，添加默认材质
        print(f"⚠️ 未找到 Gibson/MTL 材质，添加默认材质...")
        
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
        print(f"✅ 为 {mesh_count} 个网格添加了默认材质绑定")
        
        return True
        
    except Exception as e:
        print(f"❌ 检查/添加材质失败: {e}")
        return False

def convert_gibson_obj_to_usd_preserve_materials(scene_name):
    """
    将 Gibson OBJ 场景转换为 USD 格式，保留原始材质
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
    # gibson_path = f"/home/vergil/GibsonEnv/gibson/assets/dataset/{scene_name}"
    gibson_path = f"/home/vergil/Downloads/gibson_v2_4+/gibson_v2_selected/{scene_name}"
    
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
    
    print(f"🔄 转换 {scene_name} (保留原始材质)")
    print(f"   输入: {obj_file}")
    print(f"   输出: {output_usd}")
    
    try:
        print("📥 使用 Isaac Lab MeshConverter 转换 OBJ 文件...")
        print("💡 Isaac Lab 会自动处理 Gibson 的材质和纹理")
        
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
        
        # 检查并添加材质（如果需要）
        print("🎨 检查材质信息...")
        if add_materials_if_needed(conv.usd_path):
            print("✅ 材质处理完成")
        else:
            print("⚠️ 材质处理失败，但转换仍然成功")
        
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
        print("=== Gibson 场景转换工具 (保留原始材质) ===")
        print("用法:")
        print("   ~/IsaacLab/isaaclab.sh -p tools/convert_gibson_scene_preserve_materials.py <scene_name>")
        print("   例如: ~/IsaacLab/isaaclab.sh -p tools/convert_gibson_scene_preserve_materials.py Allensville")
        print()
        print("💡 这个工具会:")
        print("   1. 使用 Isaac Lab 的 MeshConverter 自动处理 Gibson 材质")
        print("   2. 保留原始的 Gibson 材质和纹理")
        print("   3. 只在没有材质的情况下添加默认材质")
        return
    
    scene_name = sys.argv[1]
    
    print(f"=== Gibson 场景转换 (保留原始材质): {scene_name} ===")
    
    success = convert_gibson_obj_to_usd_preserve_materials(scene_name)
    
    if success:
        print("🎉 转换完成！")
        print(f"现在可以使用: --env_name gibson-{scene_name}")
        print("✅ USD文件包含原始 Gibson 材质和纹理")
    else:
        print("💡 转换失败")

if __name__ == "__main__":
    main()
