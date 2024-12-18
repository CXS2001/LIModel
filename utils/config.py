import os
import sys
import glob

config_path = os.path.abspath(__file__)
print(config_path)
src_dit = os.path.dirname(config_path)
print(src_dit)
proj_root = os.path.dirname(src_dit)
print(proj_root)
# def get_proj_root():
#     config_path = os.path.abspath(__file__)
#     src_dir = os.path.dirname(config_path)
#     proj_root = os.path.dirname(src_dir)
#     return proj_root

# def set_carla_api_path():
#     proj_root = get_proj_root()
#     dist_path = os.path.join(proj_root, "scenario_runner/PythonAPI/carla/dist")
#     glob_path = os.path.join(dist_path, "carla-*%d.%d-%s.egg" % (
#         sys.version_info.major,
#         7,
#         "win-amd64" if os.name == "nt" else "linux-x86_64"
#     ))

#     try:
#         api_path = glob.glob(glob_path)[0]
#     except IndexError:
#         print("Couldn't set carla API path.")
#         exit(-1)
    
#     if api_path not in sys.path:
#         sys.path.append(api_path)


# class Config:
#     def __init__(self):
#         import carla
#         self._town = 5
#         self._FRAME_RATE = 30