"""
Limo Nav2 자율주행 준비용 1회 실행 스크립트 (headless).

- 0907.usd를 열어 /limo 밑에 RTX Lidar(2D)를 추가
- 콜리전 메시(/World/gauss/mesh)의 월드 바운딩박스를 자동 계산해 2D occupancy map 생성
- ROS map_server용 nav2/limo_map.png + limo_map.yaml 로 export
- /limo의 BehaviorScript를 limo_wasd_teleop_behavior.py -> limo_nav2_behavior.py 로 교체
- 스테이지 저장

실행 전 Isaac Sim GUI는 반드시 닫아둘 것 (VRAM 8GB 노트북 GPU, 동시 실행 시 크래시 이력 있음).

실행:
    cd /home/jeon/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
    ./python.sh "/home/jeon/t3/isaac/rccar/prepare_nav2_assets_limo.py"
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import carb
import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, Sdf, Usd, UsdGeom

enable_extension("isaacsim.asset.gen.omap")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()

PROJECT_DIR = "/home/jeon/t3/isaac/rccar"
USD_PATH = os.path.join(PROJECT_DIR, "0907.usd")
NAV2_DIR = os.path.join(PROJECT_DIR, "nav2")
os.makedirs(NAV2_DIR, exist_ok=True)

ctx = omni.usd.get_context()
ctx.open_stage(USD_PATH)
simulation_app.update()
stage = ctx.get_stage()
if stage is None:
    carb.log_error(f"[prepare_nav2_assets_limo] 스테이지 로드 실패: {USD_PATH}")
    simulation_app.close()
    raise SystemExit(1)

# ---------------------------------------------------------------------------
# 1) RTX Lidar (2D) 를 /limo 밑에 생성 — 차체 위로 확실히 벗어나게 z=0.3 에 배치해
#    자기 몸체에 가려지지 않게 함 (바퀴 중심 높이 0.05m, 차체 상단은 그보다 낮음)
# ---------------------------------------------------------------------------
existing_lidar = stage.GetPrimAtPath("/limo/lidar")
if existing_lidar.IsValid():
    print(f"[prepare_nav2_assets_limo] lidar prim 이미 존재함, 재생성 생략: {existing_lidar.GetPath()}")
else:
    _, lidar_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="lidar",
        parent="/limo",
        config="Example_Rotary_2D",
        translation=(0.0, 0.0, 0.3),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    simulation_app.update()
    if lidar_prim is None or not lidar_prim.GetPrim().IsValid():
        carb.log_error("[prepare_nav2_assets_limo] RTX Lidar 생성 실패")
        simulation_app.close()
        raise SystemExit(1)
    print(f"[prepare_nav2_assets_limo] lidar prim 생성됨: {lidar_prim.GetPath()}")

# ---------------------------------------------------------------------------
# 2) 콜리전 메시 월드 바운딩박스 계산
#    /World/gauss 는 NuRec Gaussian Splat "볼륨"이라 노이즈성 외곽 포인트 때문에
#    바운딩박스가 실제 방 크기보다 훨씬 커짐(특히 z 방향). gauss.usda를 참조 합성하면
#    실제 콜리전 프록시 메시는 /World/gauss/mesh 로 들어오므로 그걸 기준으로 계산한다.
# ---------------------------------------------------------------------------
collision_mesh_prim = stage.GetPrimAtPath("/World/gauss/mesh")
if not collision_mesh_prim.IsValid():
    carb.log_warn("[prepare_nav2_assets_limo] /World/gauss/mesh 없음, /World/gauss로 폴백")
    collision_mesh_prim = stage.GetPrimAtPath("/World/gauss")
if not collision_mesh_prim.IsValid():
    carb.log_error("[prepare_nav2_assets_limo] 콜리전 메시 프림을 찾을 수 없음")
    simulation_app.close()
    raise SystemExit(1)
print(f"[prepare_nav2_assets_limo] 바운딩박스 계산 대상: {collision_mesh_prim.GetPath()}")

bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
bbox_range = bbox_cache.ComputeWorldBound(collision_mesh_prim).ComputeAlignedRange()
min_pt = bbox_range.GetMin()
max_pt = bbox_range.GetMax()
print(f"[prepare_nav2_assets_limo] 콜리전 메시 bbox min={min_pt} max={max_pt}")

mid_x = (min_pt[0] + max_pt[0]) / 2.0
mid_y = (min_pt[1] + max_pt[1]) / 2.0
# bbox 최저점(min_pt[2])은 NuRec Gaussian Splat 볼륨의 노이즈성 아웃라이어라 실제 바닥이 아니다.
# 이 씬은 (이전 Spot 프로젝트의 mesh와 달리) 바닥이 world z=0.0이 아니어서 그대로 재사용하면
# occupancy map 전체가 UNKNOWN으로 나오는 문제가 있었다(로봇 스폰 지점이 맵상 free가 아니라
# unknown 픽셀에 찍히는 것으로 발견). points 배열의 z 히스토그램에서 바닥/천장에 해당하는
# 두 개의 뚜렷한 스파이크를 찾아 검증함(바닥 z=-0.887 부근에 894개, 천장 z=2.10 부근에 3319개,
# 실내 벽/가구는 그 사이에 300~450개 수준으로 고르게 분포 — 방 높이 약 2.99m로 사무실 규모에 부합).
points = UsdGeom.Mesh(collision_mesh_prim).GetPointsAttr().Get()
mesh_xform = UsdGeom.XformCache().GetLocalToWorldTransform(collision_mesh_prim)
zs = [mesh_xform.Transform(p)[2] for p in points]
zs_sorted = sorted(zs)
floor_z = zs_sorted[int(len(zs_sorted) * 0.02)]  # 하위 2% 지점 = 바닥 스파이크 근방
print(f"[prepare_nav2_assets_limo] 자동 계산된 floor_z={floor_z:.3f} (z min={zs_sorted[0]:.3f}, max={zs_sorted[-1]:.3f})")

cell_size = 0.05
# 이 씬의 콜리전 메시(Gaussian Splat 재구성)는 바닥 바로 위(수 cm~수십 cm)에 노이즈성 지오메트리가
# 두껍게 깔려있어서, Spot 프로젝트처럼 "바닥 바로 위 2cm~70cm" 밴드로 스캔하면 노이즈를 바닥의
# 장애물로 오인해 실내 전체가 OCCUPIED로 나온다. 반대로 바닥이 0.0이라고 잘못 가정한 원래 밴드는
# 실제 바닥(-0.89 부근)보다 한참 위라 아무것도 못 찾고 UNKNOWN이 된다. 실측으로 여러 높이 밴드를
# 비교한 결과 "바닥 위 0.8m~2.0m"(허리~어깨 높이) 구간이 바닥 노이즈는 피하면서 벽/가구는 제대로
# 잡아 FREE 비중이 가장 높았다 — 이 씬에 한해 재검증 필요(다른 맵으로 바꾸면 다시 비교할 것).
origin = (mid_x, mid_y, floor_z + 1.4)
lower_bound = (min_pt[0] - mid_x, min_pt[1] - mid_y, -0.6)
upper_bound = (max_pt[0] - mid_x, max_pt[1] - mid_y, 0.6)

# ---------------------------------------------------------------------------
# 3) Occupancy map 생성 — isaacsim.asset.gen.omap.ui 확장의 _generate_map/_fill_image
#    로직을 그대로 따름 (physx 콜리전 지오메트리를 직접 사용, 임시 CollisionAPI 부여 없이)
# ---------------------------------------------------------------------------
from isaacsim.asset.gen.omap.bindings import _omap

om = _omap.acquire_omap_interface()
om.set_cell_size(cell_size)
om.set_transform(origin, lower_bound, upper_bound)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()
om.generate()
simulation_app.update()
timeline.stop()
simulation_app.update()

dims = om.get_dimensions()
width, height = int(dims[0]), int(dims[1])
buffer = om.get_buffer()
print(f"[prepare_nav2_assets_limo] occupancy map dims=({width},{height}), cells={len(buffer)}")

OCCUPIED = 0
FREE = 254
UNKNOWN = 205
pixels = bytearray([UNKNOWN]) * (width * height)
for idx, b in enumerate(buffer):
    if b == 1.0:
        pixels[idx] = OCCUPIED
    elif b == 0.0:
        pixels[idx] = FREE

from PIL import Image

# isaacsim.asset.gen.omap.ui의 generate_image()/_fill_image() 공식 구현을 그대로 재현한다.
# 기본 설정(Rotate Image = "180", 첫 번째 드롭다운 항목)과 동일하게, buffer를 raw 순서로
# 이미지화한 뒤 180도 회전한 것을 최종 이미지로 저장해야 origin 공식과 좌표가 서로 맞는다.
im_raw = Image.frombytes("L", (width, height), bytes(pixels))
im = im_raw.rotate(-180, expand=True)
map_png_path = os.path.join(NAV2_DIR, "limo_map.png")
im.save(map_png_path)

min_b = om.get_min_bound()
max_b = om.get_max_bound()
half_w = cell_size * 0.5
# compute_coordinates()에서 raw 좌상단(top_left_raw = max_b-half_w, min_b+half_w)이
# 180도 회전 후 우하단이 되고, raw 우상단(top_right_raw = min_b+half_w, min_b+half_w)이
# 180도 회전 후 좌하단(bottom_left, ROS map.yaml의 origin)이 된다.
bottom_left = (min_b[0] + half_w, min_b[1] + half_w)

yaml_text = (
    f"image: limo_map.png\n"
    f"resolution: {cell_size}\n"
    f"origin: [{bottom_left[0]}, {bottom_left[1]}, 0.0]\n"
    f"negate: 0\n"
    f"occupied_thresh: 0.65\n"
    f"free_thresh: 0.196\n"
)
map_yaml_path = os.path.join(NAV2_DIR, "limo_map.yaml")
with open(map_yaml_path, "w") as f:
    f.write(yaml_text)

_omap.release_omap_interface(om)
print(f"[prepare_nav2_assets_limo] map 저장됨: {map_png_path}")
print(f"[prepare_nav2_assets_limo] yaml 저장됨: {map_yaml_path}")

limo_prim = stage.GetPrimAtPath("/limo")
limo_pos = limo_prim.GetAttribute("xformOp:translate").Get()
print(f"[prepare_nav2_assets_limo] origin(bottom_left)={bottom_left}, robot spawn(before)={limo_pos}, map mid=({mid_x},{mid_y})")

# ---------------------------------------------------------------------------
# 3-1) /limo 스폰 높이를 실제 바닥(floor_z) 위 바퀴 반지름만큼으로 보정.
#      기존 z=0은 Spot을 두던 자리를 그대로 물려받은 값인데, 이 씬의 진짜 바닥은 floor_z
#      (약 -0.89) 부근이라 z=0이면 바닥보다 한참 위에 떠서 Play 시 중력으로 떨어지게 된다.
#      떨어지는 동안 초기 odom/TF가 잠깐 튀는 것을 피하기 위해 처음부터 바닥에 붙여 스폰한다.
# ---------------------------------------------------------------------------
_WHEEL_RADIUS = 0.045
fixed_z = floor_z + _WHEEL_RADIUS
translate_attr = limo_prim.GetAttribute("xformOp:translate")
translate_attr.Set(Gf.Vec3d(limo_pos[0], limo_pos[1], fixed_z))
print(f"[prepare_nav2_assets_limo] robot spawn(after)=({limo_pos[0]}, {limo_pos[1]}, {fixed_z})")

# ---------------------------------------------------------------------------
# 4) /limo 에 붙은 BehaviorScript 를 limo_nav2_behavior.py 로 교체
# ---------------------------------------------------------------------------
scripts_attr = limo_prim.GetAttribute("omni:scripting:scripts")
if scripts_attr and scripts_attr.IsValid():
    scripts_attr.Set([Sdf.AssetPath("./limo_nav2_behavior.py")])
    print("[prepare_nav2_assets_limo] omni:scripting:scripts -> ./limo_nav2_behavior.py 로 교체")
else:
    carb.log_error("[prepare_nav2_assets_limo] omni:scripting:scripts 속성을 찾을 수 없음")

stage.GetRootLayer().Save()
print("[prepare_nav2_assets_limo] 0907.usd 저장 완료")

simulation_app.close()
