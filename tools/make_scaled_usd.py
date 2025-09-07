#!/usr/bin/env python3
"""
根据环境前缀（gibson-*/igibson-*）定位源 USD，并生成一个引用包装的等比例缩放 USD：{USD_name}_80x.usd。

用法示例：
  python tools/make_scaled_usd.py gibson-Allensville --scale 80
  python tools/make_scaled_usd.py igibson-RS --scale 80

规则：
  - gibson-*   → 源路径: project_root/gibson_usd/{name}/{name}.usd
  - igibson-*  → 源路径: /home/vergil/data/igibson/{name}/{name}.usd
输出：
  - 与源文件同目录：{name}_80x.usd
"""

import os
import sys
import argparse
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)

try:
	from pxr import Usd, UsdGeom, Gf
except Exception as e:
	print(f"❌ 无法导入 pxr 模块（Usd/UsdGeom/Gf）：{e}")
	print("请在 Isaac/Omniverse USD 可用的 Python 环境中运行该脚本。")
	sys.exit(1)


def make_scaled_wrapper(wrapper_usd: str, target_usd: str, target_prim_path: str = None, scale: float = 80.0, translate_x: float = 0.0, translate_y: float = 0.0, translate_z: float = 0.0, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, up_axis: str = "auto") -> str:
	"""创建一个包裹 USD：引用 target_usd，并整体缩放到给定比例。

	返回生成的 USD 路径。
	"""
	# 确保目录存在
	os.makedirs(os.path.dirname(wrapper_usd), exist_ok=True)

	stage = Usd.Stage.CreateNew(wrapper_usd)
	# 先打印包装 USD 的初始 upAxis（未显式设置时，通常为 Y）
	try:
		print(f"Wrapper USD initial upAxis: {UsdGeom.GetStageUpAxis(stage)}")
	except Exception as e:
		print(f"⚠️ Cannot read initial Wrapper upAxis: {e}")
	# 以源 USD 的 defaultPrim 名称作为包裹根名，若不存在则回退为 GibsonScaled
	try:
		print(f"Opening source USD: {target_usd}")
		src_stage = Usd.Stage.Open(target_usd)
		# 打印源 USD 的 upAxis
		try:
			src_up = UsdGeom.GetStageUpAxis(src_stage)
			print(f"Source USD upAxis: {src_up}")
		except Exception as e:
			print(f"⚠️ Cannot read Source upAxis: {e}")

		# 根据 up_axis 选项设置 wrapper 的 upAxis（仅元数据，不旋转几何）
		try:
			if up_axis == "y":
				UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
			elif up_axis == "z":
				UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
			else:  # auto：对齐源 USD
				if src_up == UsdGeom.Tokens.z:
					UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
				else:
					UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
			print(f"Wrapper USD set upAxis: {UsdGeom.GetStageUpAxis(stage)}")
		except Exception as e:
			print(f"⚠️ Failed to set Wrapper upAxis: {e}")
		default_prim = src_stage.GetDefaultPrim() if src_stage else None
		root_name = default_prim.GetName() if default_prim and default_prim.IsValid() else "GibsonScaled"
	except Exception:
		root_name = "GibsonScaled"
	root_xform = UsdGeom.Xform.Define(stage, f"/{root_name}").GetPrim()
	xform_api = UsdGeom.XformCommonAPI(root_xform)
	# 平移（XYZ）
	if (translate_x != 0.0) or (translate_y != 0.0) or (translate_z != 0.0):
		xform_api.SetTranslate(Gf.Vec3d(float(translate_x), float(translate_y), float(translate_z)))
	# 旋转（欧拉角，度）
	if (roll != 0.0) or (pitch != 0.0) or (yaw != 0.0):
		xform_api.SetRotate(Gf.Vec3f(float(roll), float(pitch), float(yaw)))
	# 缩放
	xform_api.SetScale(Gf.Vec3f(float(scale), float(scale), float(scale)))

	refs = root_xform.GetReferences()
	if target_prim_path:
		refs.AddReference(target_usd, target_prim_path)
	else:
		refs.AddReference(target_usd)

	stage.SetDefaultPrim(root_xform)
	stage.GetRootLayer().Save()
	return wrapper_usd


def resolve_source_usd(env_token: str, project_root: str) -> str:
	"""根据前缀解析源 USD 路径。"""
	if env_token.startswith("gibson-"):
		name = env_token[len("gibson-"):]
		return os.path.abspath(os.path.join(project_root, f"gibson_usd/{name}/{name}.usd"))
	elif env_token.startswith("igibson-"):
		name = env_token[len("igibson-"):]
		return os.path.abspath(f"/home/vergil/dataset/igibson/{name}/{name}.usd")
	else:
		# 允许用户直接传绝对/相对 USD 路径
		return os.path.abspath(env_token)


def main():
	parser = argparse.ArgumentParser(description="生成缩放包装 USD（{USD_name}_80x.usd）")
	parser.add_argument("env", help="环境标识（gibson-*/igibson-*），或直接传 USD 路径")
	parser.add_argument("--scale", type=float, default=80.0, help="缩放倍数，默认 80.0")
	parser.add_argument("--prim", type=str, default=None, help="可选：目标文件内要引用的 prim 路径（默认使用 defaultPrim）")
	parser.add_argument("--x", type=float, default=-1.9, help="X 轴平移（米），默认 0.0")
	parser.add_argument("--y", type=float, default=-0.6, help="Y 轴平移（米），默认 0.0")
	parser.add_argument("--z", type=float, default=-4.3, help="Z 轴平移（米），默认 0.0")
	parser.add_argument("--roll", type=float, default=0.5, help="绕 X 轴旋转（度），默认 0.0")
	parser.add_argument("--pitch", type=float, default=4.6, help="绕 Y 轴旋转（度），默认 0.0")
	parser.add_argument("--yaw", type=float, default=90.0, help="绕 Z 轴旋转（度），默认 0.0")
	parser.add_argument("--up-axis", dest="up_axis", choices=["auto", "y", "z"], default="auto", help="包装 USD 的 upAxis：auto 与源对齐，或强制为 y/z")
	args = parser.parse_args()

	project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
	target_usd = resolve_source_usd(args.env, project_root)

	if not os.path.exists(target_usd):
		print(f"❌ 源 USD 不存在：{target_usd}")
		sys.exit(2)

	src_dir = os.path.dirname(target_usd)
	parent_dir = os.path.dirname(src_dir)  # 使用上级目录
	usd_name = os.path.splitext(os.path.basename(target_usd))[0]
	# 创建以 scale 为后缀的子目录
	scale_suffix = f"{int(args.scale)}x"
	out_dir = os.path.join(parent_dir, f"{usd_name}_{scale_suffix}")
	out_path = os.path.join(out_dir, f"{usd_name}_{scale_suffix}.usd")

	result = make_scaled_wrapper(out_path, target_usd, target_prim_path=args.prim, scale=args.scale, translate_x=args.x, translate_y=args.y, translate_z=args.z, roll=args.roll, pitch=args.pitch, yaw=args.yaw, up_axis=args.up_axis)
	print(f"✅ 已生成：{result}")
	print("   - 引用：", target_usd)
	print("   - 缩放：", args.scale)
	print("   - 平移XYZ：", (args.x, args.y, args.z))
	print("   - 旋转RPY(deg)：", (args.roll, args.pitch, args.yaw))
	print("   - upAxis：", args.up_axis)


if __name__ == "__main__":
	main()


