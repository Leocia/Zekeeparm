"""Export only Studio and its runtime sources/assets as an independent ZIP."""
import argparse
from pathlib import Path
import shutil
import tempfile
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
NAME = "ZekeeparmStudio-1023"
import os
MINIFORGE = Path(os.environ.get("ZKEEP_WINDOWS_MINIFORGE_ZIP", "Miniforge3-Windows-x86_64.zip")).expanduser()

WINDOWS_README = """# Zekeeparm Studio 1023（Windows）

这是可独立解压的 Windows x64 Studio 包，已包含网页、Blockly 离线资源、机械臂模型、SDK、控制核心和 Miniforge 安装包。无需 Ubuntu、ROS、MoveIt 或 Node.js。

## 首次安装与启动

1. 将整个 `ZekeeparmStudio-1023.zip` 解压到普通目录，不要在压缩包内运行。
2. 解压包内的 `Miniforge3-26.7.2-1-Windows-x86_64.zip`，运行解出的 `Miniforge3-26.7.2-1-Windows-x86_64.exe`。按安装程序提示完成安装，建议使用当前 Windows 用户可写的默认路径。
3. 双击 `1-setup.cmd`。首次安装会联网下载 Studio 所需 Python 依赖、运行自检并创建桌面快捷方式；保持窗口打开直到完成。
4. 双击桌面上的 `Zekeeparm Studio` 启动。也可用 `2-start.cmd` 启动，或在已安装环境中运行 `4-create-desktop-shortcut.cmd` 重建快捷方式。
5. 初次使用可选择“模拟设备”，点击“连接”“使能”后运行程序，或点击“预览”查看目标动作。

Miniforge 安装包保存在本 ZIP 内，Studio 的 Python 依赖仍需首次联网下载。安装完成后，基础编程和模拟运行无需持续联网。快捷方式会记录 Miniforge 与解压目录；移动目录后请重建快捷方式。关闭浏览器不会退出 Studio 后台，更新前先关闭旧后台。

## 生成独立 EXE

双击 `3-build-exe.cmd`。脚本运行测试、构建应用并执行 EXE 自检。生成目录为 `apps\\studio\\dist\\ZekeeparmStudio`，分发时复制整个目录，不能只复制 EXE。也可在 Miniforge Prompt 中运行：

    conda run --no-capture-output -n zekeep-studio python build_bundle.py

需要安装 Inno Setup 6 并将 `ISCC.exe` 加入 PATH，才可额外生成安装器：

    powershell -ExecutionPolicy Bypass -File apps\\studio\\build_windows.ps1 -Installer

## 连接机械臂

在设备面板选择“真机 · USB 串口”，填写实际 COM 端口。按设备要求安装 USB 驱动。启动软件不会连接或使能机械臂；连接后仍需在界面显式操作。硬件参数位于 `src\\zekeep_bringup\\config\\zekeep_hardware.yaml`，项目数据位于 `%LOCALAPPDATA%\\ZekeeparmStudio`。

机械臂运行前预览跟随关节滑块显示目标姿态，运行后显示实时状态；碰撞目标会标红并阻止执行。全屏预览底部提供六轴目标滑条、“移动到目标”和“运行积木”按钮；右侧实时角度窗可拖动，或聚焦后用方向键微调位置。当前碰撞模型包含机械臂自碰撞和固定工作台，不含外部障碍物、持物几何或动态场景。路径检查为离散采样，不能替代现场安全措施。

## 验证边界

Linux 上可验证源码与模拟逻辑；Windows 安装、EXE 构建、USB 驱动和真机链路需在目标 Windows 电脑单独验证。本压缩包不是预编译的 Windows EXE，也不包含离线 Python 依赖或 USB 驱动。运行前检查急停、工作区和周边环境，首次真机动作使用低速并安排人员监护。

更多功能说明见 `apps\\studio\\README.md`。许可说明见 `SOURCE_LICENSE.md`，Blockly 许可证随离线资源提供。
"""


def copy(source, destination):
    if source.is_dir():
        shutil.copytree(source, destination, ignore=shutil.ignore_patterns(
            "__pycache__", "*.pyc", ".pytest_cache", ".DS_Store"))
    else:
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)


def package(destination):
    destination = Path(destination).resolve()
    if destination.exists():
        raise FileExistsError(f"Refusing to overwrite {destination}")
    if not (HERE / "public/vendor/blockly/blockly_compressed.js").is_file():
        raise FileNotFoundError("Prepare the offline Blockly assets with npm ci first")
    if not MINIFORGE.is_file():
        raise FileNotFoundError(f"Windows Miniforge archive not found: {MINIFORGE}")
    with tempfile.TemporaryDirectory(prefix="zekeep-windows-export-") as directory:
        bundle = Path(directory) / NAME
        studio = bundle / "apps/studio"
        for name in ("studio.py", "core.py", "robot.py", "python_block.py", "environment.yml", "README.md",
                     "start_windows.cmd", "find_conda.cmd", "setup_windows.cmd", "create_desktop_shortcut.cmd", "start.sh",
                     "build_windows.ps1", "build_bundle.py", "installer.iss", "hooks", "public",
                     "package.json", "package-lock.json", "prepare-assets.cjs", "package_windows.py",
                     "playwright.config.cjs", "tests"):
            copy(HERE / name, studio / name)
        for relative in ("zekeeparm_SDK", "src/zekeep_bringup/description",
                         "src/zekeep_bringup/config/zekeep_hardware.yaml",
                         "src/zekeep_grasp/config/sixaxis.urdf",
                         "src/zekeep_moveit_config/config/zekeep.srdf",
                         "src/zekeep_bringup/web/public/lib"):
            copy(ROOT / relative, bundle / relative)
        controller = "src/zekeepcontroller/zekeepcontroller"
        for name in ("__init__.py", "hardware_manager.py", "hardware_config.py", "joint_mapping.py",
                     "dynamics_validation.py", "gripper_grasp.py", "conversions.py"):
            copy(ROOT / controller / name, bundle / controller / name)
        (bundle / "README_Windows.md").write_text(WINDOWS_README, encoding="utf-8")
        copy(MINIFORGE, bundle / MINIFORGE.name)
        if (ROOT / "SOURCE_LICENSE.md").is_file():
            copy(ROOT / "SOURCE_LICENSE.md", bundle / "SOURCE_LICENSE.md")
        else:
            original = (ROOT / "README.md").read_text(encoding="utf-8")
            license_text = original.split("## 许可", 1)[1].split("## ", 1)[0].strip()
            (bundle / "SOURCE_LICENSE.md").write_text("# 来源项目许可\n\n" + license_text + "\n", encoding="utf-8")
        for filename, target in (("1-setup.cmd", "setup_windows.cmd"),
                                 ("2-start.cmd", "start_windows.cmd"),
                                 ("4-create-desktop-shortcut.cmd", "create_desktop_shortcut.cmd")):
            (bundle / filename).write_bytes((f'@echo off\r\ncall "%~dp0apps\\studio\\{target}"\r\n').encode())
        (bundle / "3-build-exe.cmd").write_bytes(
            b'@echo off\r\nsetlocal\r\ncall "%~dp0apps\\studio\\find_conda.cmd"\r\n'
            b'if errorlevel 1 goto failed\r\n'
            b'powershell.exe -NoProfile -File "%~dp0apps\\studio\\build_windows.ps1" %*\r\n'
            b'if errorlevel 1 goto failed\r\npause\r\nexit /b 0\r\n'
            b':failed\r\necho Build failed. Check the error above.\r\npause\r\nexit /b 1\r\n')
        destination.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(destination, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for path in sorted(bundle.rglob("*")):
                if path.is_file():
                    content = path.read_bytes()
                    if path.suffix.lower() in (".cmd", ".ps1", ".iss"):
                        content = content.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
                    archive.writestr(path.relative_to(bundle.parent).as_posix(), content)
        with zipfile.ZipFile(destination) as archive:
            if archive.testzip() is not None:
                raise RuntimeError("ZIP integrity check failed")
            count = len(archive.namelist())
        print(f"{destination}\n{count} files, {destination.stat().st_size / 1024**2:.2f} MiB")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    package(parser.parse_args().destination)
