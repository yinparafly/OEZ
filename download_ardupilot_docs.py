"""
ArduPilot 文档离线下载器
下载与我们项目最相关的文档到本地
"""
import os
import re
import time
import urllib.request
import urllib.error

BASE = "https://ardupilot.org/dev/docs/"
OUT_DIR = r"D:\oezcon\ardupilot_docs"

# 核心文档列表 - 按项目相关度排序
DOCS = {
    # === SITL 仿真 ===
    "SITL": {
        "sitl-simulator-software-in-the-loop.html": "SITL仿真器概述",
        "sitl-on-windows-wsl.html": "WSL下的SITL设置",
        "setting-up-sitl-on-linux.html": "Linux下的SITL设置",
        "SITL-setup-landingpage.html": "SITL设置入口",
        "using-sitl-for-ardupilot-testing.html": "使用SITL测试",
        "SITL_simulation_parameters.html": "SITL仿真参数",
        "sitl-parameters.html": "SITL参数列表",
        "plane-sitlmavproxy-tutorial.html": "Plane SITL教程",
        "sitl-serial-mapping.html": "SITL串口映射",
    },
    # === Lua 脚本 ===
    "Lua_Scripts": {
        "common-lua-scripts.html": "Lua脚本概述",
        "common-scripting-step-by-step.html": "Lua脚本入门教程",
        "common-scripting-parameters.html": "Lua脚本参数",
    },
    # === MAVLink 接口 ===
    "MAVLink": {
        "mavlink-commands.html": "MAVLink命令概述",
        "mavlink-basics.html": "MAVLink基础",
        "mavlink-requesting-data.html": "请求数据流",
        "mavlink-get-set-params.html": "获取和设置参数",
        "mavlink-arming-and-disarming.html": "解锁和上锁",
        "mavlink-get-set-flightmode.html": "获取和设置飞行模式",
        "mavlink-mission-upload-download.html": "任务上传下载",
        "mavlink-routing-in-ardupilot.html": "MAVLink路由",
        "common-mavlink-mission-command-messages-mav_cmd.html": "MAVLink任务命令列表",
    },
    # === Plane 固定翼 ===
    "Plane": {
        "plane-architecture.html": "Plane架构概述",
        "plane-navigation-overview.html": "Plane导航和高度控制",
    },
    # === 编译构建 ===
    "Build": {
        "building-the-code.html": "编译代码概述",
        "building-setup-linux.html": "Linux编译环境设置",
        "building-setup-windows11.html": "Windows11 WSL2编译环境",
    },
    # === Lua 脚本 API ===
    "Lua_API": {
        "common-lua-scripts.html": "Lua脚本",
        "common-scripting-step-by-step.html": "脚本开发步骤",
        "common-scripting-parameters.html": "脚本参数",
    },
}

def sanitize_filename(name):
    """清理文件名"""
    name = re.sub(r'[<>:"/\\|?*]', '_', name)
    name = re.sub(r'\s+', '_', name)
    return name[:100]

def download_doc(url, filepath):
    """下载单个文档"""
    try:
        req = urllib.request.Request(url, headers={
            'User-Agent': 'Mozilla/5.0 (compatible; ArduPilotDocDownloader/1.0)'
        })
        with urllib.request.urlopen(req, timeout=30) as resp:
            content = resp.read()
            # 只保存 HTML 的主要部分
            text = content.decode('utf-8', errors='replace')
            # 提取正文
            body_match = re.search(r'<div\s+role="main"[^>]*>(.*?)</div>\s*</div>\s*</div>', text, re.DOTALL)
            if body_match:
                text = body_match.group(1)
            # 清理HTML标签，保留纯文本和链接
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(f"<!-- Source: {url} -->\n")
                f.write(f"<!-- Downloaded: {time.strftime('%Y-%m-%d %H:%M')} -->\n\n")
                f.write(text)
            return True
    except Exception as e:
        print(f"  [FAIL] {e}")
        return False

def main():
    total = sum(len(v) for v in DOCS.values())
    downloaded = 0
    failed = 0
    print(f"ArduPilot Documentation Downloader")
    print(f"Target: {total} pages to {OUT_DIR}")
    print("=" * 50)

    for category, pages in DOCS.items():
        cat_dir = os.path.join(OUT_DIR, category)
        os.makedirs(cat_dir, exist_ok=True)
        print(f"\n[{category}] ({len(pages)} pages)")

        for filename, title in pages.items():
            url = BASE + filename
            out_name = sanitize_filename(title) + ".html"
            out_path = os.path.join(cat_dir, out_name)

            if os.path.exists(out_path):
                print(f"  [SKIP] {title}")
                downloaded += 1
                continue

            print(f"  [GET] {title} ...", end=" ", flush=True)
            if download_doc(url, out_path):
                print("OK")
                downloaded += 1
            else:
                failed += 1
            time.sleep(0.5)  # 限速

    print(f"\n{'=' * 50}")
    print(f"Done: {downloaded} downloaded, {failed} failed")
    print(f"Location: {OUT_DIR}")

if __name__ == "__main__":
    main()
