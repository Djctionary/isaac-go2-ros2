import argparse
from isaaclab.app import AppLauncher
import numpy as np

def add_uv_to_terrain(prim_path="/World/ground/terrain/mesh"):
    from pxr import Usd, UsdGeom, Sdf, Gf
    import omni.usd
    SCALE = 0.1 
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    mesh = UsdGeom.Mesh(prim)
    api = UsdGeom.PrimvarsAPI(prim)

    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float32)
    uv = np.stack([pts[:, 0], pts[:, 2]], axis=1)
    uv = (uv - uv.min(axis=0)) / (uv.max(axis=0) - uv.min(axis=0) + 1e-6)

    st = api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(float(u * SCALE), float(v * SCALE)) for u, v in uv])

    print(f"Generated UV for {prim_path}")



def main():
    parser = argparse.ArgumentParser(description="Generate rough terrain in IsaacSim")
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()

    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    import isaaclab.sim as sim_utils
    from isaaclab.terrains import TerrainImporter, TerrainImporterCfg
    from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG
    from isaaclab.sim.spawners.materials.visual_materials_cfg import MdlFileCfg
    
    render_cfg = sim_utils.RenderCfg(rendering_mode="balanced")
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device="cuda:0", render=render_cfg)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view(eye=[5.0, 5.0, 5.0], target=[0.0, 0.0, 0.0])

    terrain_gen_cfg = ROUGH_TERRAINS_CFG.replace(curriculum=False, color_scheme="none")

    material_cfg = MdlFileCfg(
        mdl_path="/opt/nvidia/mdl/vMaterials_2/Ground/Ground_Aggregate_Exposed.mdl",
        # texture_scale=(3.0, 3.0),
    )

    terrain_imp_cfg = TerrainImporterCfg(
        num_envs=1,
        env_spacing=3.0,
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=terrain_gen_cfg,
        debug_vis=False,
        visual_material=material_cfg
    )
    TerrainImporter(terrain_imp_cfg)

    add_uv_to_terrain("/World/ground/terrain/mesh")

    sim.reset()
    print("[INFO] Terrain generated and imported.")

    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()

if __name__ == "__main__":
    main()
