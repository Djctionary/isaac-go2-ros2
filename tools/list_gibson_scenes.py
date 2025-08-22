#!/usr/bin/env python3
"""
列出所有可用的 Gibson 场景
"""

import os
import json

def list_gibson_scenes():
    """列出所有可用的 Gibson 场景"""
    gibson_dataset_path = "/home/vergil/GibsonEnv/gibson/assets/dataset"
    
    if not os.path.exists(gibson_dataset_path):
        print("❌ Gibson 数据集路径不存在")
        return []
    
    scenes = []
    for item in os.listdir(gibson_dataset_path):
        scene_path = os.path.join(gibson_dataset_path, item)
        if os.path.isdir(scene_path):
            # 检查是否有网格文件
            mesh_files = [
                os.path.join(scene_path, "mesh_z_up.obj"),
                os.path.join(scene_path, "mesh.obj")
            ]
            
            has_mesh = any(os.path.exists(f) for f in mesh_files)
            
            scene_info = {
                "name": item,
                "path": scene_path,
                "has_mesh": has_mesh,
                "files": os.listdir(scene_path) if has_mesh else []
            }
            scenes.append(scene_info)
    
    return scenes

def print_scenes_info(scenes):
    """打印场景信息"""
    print(f"🗺️ 发现 {len(scenes)} 个 Gibson 场景:")
    print("=" * 50)
    
    valid_scenes = []
    for i, scene in enumerate(scenes, 1):
        status = "✅" if scene["has_mesh"] else "❌"
        print(f"{i:2d}. {status} {scene['name']}")
        
        if scene["has_mesh"]:
            valid_scenes.append(scene["name"])
            files = scene["files"]
            mesh_files = [f for f in files if f.endswith('.obj')]
            if mesh_files:
                print(f"     网格文件: {', '.join(mesh_files)}")
            if "pano" in files:
                print(f"     全景图像: 可用")
            if "camera_poses.csv" in files:
                print(f"     相机位置: 可用")
        print()
    
    print("=" * 50)
    print(f"✅ 可用场景数量: {len(valid_scenes)}")
    
    return valid_scenes

def create_scene_config(valid_scenes):
    """创建场景配置文件"""
    config = {
        "gibson_scenes": {
            "total_count": len(valid_scenes),
            "available_scenes": valid_scenes,
            "recommended_scenes": valid_scenes[:5],  # 前5个作为推荐
            "usage_examples": [
                f"python isaac_go2_ros2.py env_name=gibson-{scene}"
                for scene in valid_scenes[:3]
            ]
        }
    }
    
    with open("gibson_scenes_config.json", 'w') as f:
        json.dump(config, f, indent=2)
    
    print(f"📝 场景配置已保存到: gibson_scenes_config.json")

def main():
    print("=== Gibson 场景列表 ===")
    
    scenes = list_gibson_scenes()
    if not scenes:
        print("未找到 Gibson 场景")
        return
    
    valid_scenes = print_scenes_info(scenes)
    
    if valid_scenes:
        create_scene_config(valid_scenes)
        
        print("\n🚀 使用方法:")
        print("# 使用默认场景")
        print("python isaac_go2_ros2.py --config-name gibson")
        print()
        print("# 使用特定场景")
        for scene in valid_scenes[:3]:
            print(f"python isaac_go2_ros2.py env_name=gibson-{scene}")
        print()
        print("# 可用场景总数:", len(valid_scenes))
        
    else:
        print("⚠️ 没有找到有效的 Gibson 场景")

if __name__ == "__main__":
    main()
