#!/usr/bin/env bash
# 重新 configure 已构建的 colcon 包并合并编译命令，不编译或安装目标。
# 用法：bash tools/gen_compile_commands.sh [额外构建目录 ...]
# 默认处理仓库 build/；额外目录优先覆盖同一源文件（例如 build-adaptive）。
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$WS/build"
OUT="$BUILD/compile_commands.json"
BUILD_DIRS=("$BUILD")
for extra in "$@"; do
  BUILD_DIRS+=("$(cd "$extra" && pwd)")
done

# 各包依赖 ROS 和本工作区已经生成的消息/库；沿用缓存中的其他配置。
set +u
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$WS/install/local_setup.bash" ]; then
  source "$WS/install/local_setup.bash"
fi
set -u
# 可选的本地开发包（按原始 deb 布局解压），不会修改系统安装。
if [ -d "$BUILD/clangd-deps/usr" ]; then
  export CMAKE_PREFIX_PATH="$BUILD/clangd-deps/usr:$BUILD/clangd-deps/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}"
fi

echo "[1/2] 重新 configure 各包（打开 CMAKE_EXPORT_COMPILE_COMMANDS）"
for base in "${BUILD_DIRS[@]}"; do
  for cache in "$base"/*/CMakeCache.txt; do
    [ -e "$cache" ] || continue
    bdir="$(dirname "$cache")"
    if [ ! -f "$bdir/Makefile" ] && [ ! -f "$bdir/build.ninja" ]; then
      echo "  跳过未完成配置的包：$bdir（先解决构建依赖并 colcon build）" >&2
      continue
    fi
    sdir="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache" | head -1)"
    if [ -z "$sdir" ] || [ ! -f "$sdir/CMakeLists.txt" ]; then
      echo "错误：$cache 中的源码目录无效（$sdir），请先重新 colcon build。" >&2
      exit 1
    fi
    echo "  $bdir"
    cmake -S "$sdir" -B "$bdir" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >"$bdir/clangd-configure.log" 2>&1 || {
      cat "$bdir/clangd-configure.log" >&2
      exit 1
    }
  done
done

echo "[2/2] 合并 → $OUT"
python3 - "$OUT" "${BUILD_DIRS[@]}" <<'PYTHON'
import json
from pathlib import Path
import sys

out = Path(sys.argv[1])
merged, sources = {}, []
# 无 sudo 环境可把 libomp 开发包解压到此目录；仅影响编辑器编译库。
local_omp = sorted((out.parent / "clangd-deps/usr/lib").glob("llvm-*/lib/clang/*/include/omp.h"))

for base in map(Path, sys.argv[2:]):
    # 只读取包级数据库，不重复读上次合并结果或嵌套第三方库。
    for path in sorted(base.glob("*/compile_commands.json")):
        sources.append(path)
        for entry in json.loads(path.read_text()):
            directory = Path(entry["directory"])
            if not directory.is_absolute():
                directory = path.parent / directory
            directory = directory.resolve()
            source = Path(entry["file"])
            if not source.is_absolute():
                source = directory / source
            source = source.resolve()
            if not source.is_file():
                raise SystemExit(f"失效的源文件：{source}（来自 {path}）")
            # 预编译头是构建辅助文件，无需作为独立翻译单元索引。
            if source.name.startswith("cmake_pch."):
                continue
            entry["directory"], entry["file"] = str(directory), str(source)
            if local_omp:
                include = str(local_omp[-1].parent.resolve())
                if "arguments" in entry:
                    entry["arguments"] += ["-idirafter", include]
                else:
                    import shlex
                    entry["command"] += " -idirafter " + shlex.quote(include)
            merged[str(source)] = entry
if not merged:
    raise SystemExit("没有找到编译命令，请先 colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
out.parent.mkdir(parents=True, exist_ok=True)
tmp = out.with_suffix(".json.tmp")
tmp.write_text(json.dumps(list(merged.values()), indent=2) + "\n")
tmp.replace(out)
print(f"  来源数据库 {len(sources)} 份，条目 {len(merged)} 条")
PYTHON

# 为只沿父目录搜索编译库的工具提供入口；不覆盖手动维护的普通文件。
if [ ! -e "$WS/compile_commands.json" ] || [ -L "$WS/compile_commands.json" ]; then
  ln -sfn build/compile_commands.json "$WS/compile_commands.json"
fi
