# point_lio_orig — Go2 실기용 Point-LIO launch (원본 + in-filter ZUPT)
#
# 발행 토픽은 C++ 코드를 고치지 않고 remap 으로 /hknu 접두어를 붙입니다.
# 노드는 /hknu/laserMapping 으로 뜹니다 (다른 Point-LIO 와 이름이 겹치지 않게).
# TF(camera_init -> aft_mapped)는 그대로입니다.
#
# 인자 (비워 두면 config/go2_orig.yaml 값 그대로):
#   zupt_en:=true|false       in-filter ZUPT (yaml 기본 false). 켜면 /hknu/zupt_active 발행 노드(zvd_node.py)가 필요
#   path_en:=true|false       /hknu/path 발행 (yaml 기본 false, 켜면 주행 내내 포즈가 쌓임)
#   pcd_save_en:=true|false   종료 시 PCD 저장 (yaml 기본 false)
#   pcd_interval:=<정수>       PCD 한 파일에 담을 스캔 수 (-1 = 전체를 한 파일, 메모리 주의)
#   config:=<yaml 경로>        다른 설정 파일을 쓸 때
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

PKG = 'point_lio_orig'
NS = 'hknu'

# 원본 발행 토픽 (laserMapping.cpp 에서 절대 이름) -> /hknu/...
REMAPS = [
    ('/aft_mapped_to_init', f'/{NS}/aft_mapped_to_init'),
    ('/cloud_registered', f'/{NS}/cloud_registered'),
    ('/cloud_registered_body', f'/{NS}/cloud_registered_body'),
    ('/cloud_effected', f'/{NS}/cloud_effected'),
    ('/Laser_map', f'/{NS}/Laser_map'),
    ('/path', f'/{NS}/path'),
    ('/odom_corrected', f'/{NS}/odom_corrected'),
]

BOOL_ARGS = {'zupt_en': 'zupt_en', 'path_en': 'publish.path_en', 'pcd_save_en': 'pcd_save.pcd_save_en'}


def _setup(context, *args, **kwargs):
    cfg = LaunchConfiguration('config').perform(context).strip()
    if not cfg:
        cfg = os.path.join(FindPackageShare(PKG).perform(context), 'config', 'go2_orig.yaml')
    overrides = {}
    for arg, param in BOOL_ARGS.items():
        val = LaunchConfiguration(arg).perform(context).strip().lower()
        if not val:
            continue
        if val not in ('true', 'false'):
            raise RuntimeError(f"[{PKG}] {arg}:={val} - 'true' 또는 'false' 만 받습니다")
        overrides[param] = (val == 'true')
    iv = LaunchConfiguration('pcd_interval').perform(context).strip()
    if iv:
        overrides['pcd_save.interval'] = int(iv)
    params = [cfg] + ([overrides] if overrides else [])
    return [Node(
        package=PKG,
        executable='pointlio_mapping',
        name='laserMapping',
        namespace=NS,
        output='screen',
        parameters=params,
        remappings=REMAPS,
    )]


def generate_launch_description():
    args = [DeclareLaunchArgument(a, default_value='', description=f'{a} 덮어쓰기 (빈 값 = yaml 값)')
            for a in list(BOOL_ARGS) + ['pcd_interval', 'config']]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
