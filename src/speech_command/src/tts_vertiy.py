import ctypes
import os
import sys

# 1. 强制指定你的库目录
lib_dir = "/home/ucar/instant_ws/src/speech_command/libs"
lib_path = os.path.join(lib_dir, "libaikit.so")

# 2. 将 lib 目录加入到系统的动态库搜索路径中
os.environ['LD_LIBRARY_PATH'] = lib_dir + ":" + os.environ.get('LD_LIBRARY_PATH', '')

print(f"尝试加载库: {lib_path}")

try:
    # 3. 使用 RTLD_GLOBAL 模式加载，以便 libaikit 能找到它的依赖库
    lib = ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
    print("【成功】libaikit.so 加载成功！")
    
    # 4. 初始化 SDK
    ret = lib.AIKIT_Init()
    print(f"AIKIT_Init 返回值: {ret}")
    
    if ret == 0:
        print("初始化成功，SDK 正在运行！")
    else:
        print(f"初始化失败，错误码: {ret}，请查阅错误码文档")

except OSError as e:
    print(f"【失败】加载失败，错误信息: {e}")