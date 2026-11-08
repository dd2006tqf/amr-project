#!/usr/bin/env bash
# ==============================================================================
# inner_docker_shim.sh —— 让"靠 docker exec 驱动容器"的 e2e 脚本能在容器**内部**
# 直接运行（CI 的 e2e-fleet job 就是这样：脚本本身跑在 ros:jazzy-ros-base 里，
# 没有 Docker socket，也不可能 docker exec 到自己）。
#
# 用法（只在 AMR_E2E_INNER=1 时 source，且必须在脚本调用任何 docker 之前）：
#
#     if [ -n "${AMR_E2E_INNER:-}" ]; then
#         . "${SCRIPT_DIR}/lib/inner_docker_shim.sh"
#     fi
#
# 它用同名 shell 函数覆盖下面这几个**被实际用到**的子命令，其余一律显式失败：
#
#     docker ps  --format '{{.Names}}'      -> 打印 ${INNER_CONTAINER_NAME}
#     docker exec [-d] <name> <cmd...>      -> 本地执行（-d 则后台脱离）
#     docker cp <src> <name>:<dst>          -> cp 到 <dst>
#     docker cp <name>:<src> <dst>          -> cp 从 <src>
#
# 设计取舍（为什么不干脆重写脚本）：
#   * 这两个 e2e 的价值在于"命令序列本身"（kill -9、重启、改文件、等日志），
#     重写成"本地 / 容器"双份会让断言分叉，而断言是这套测试唯一的产出。
#   * shim 只需覆盖 4 个语义明确的子命令，任何未覆盖的用法会直接报错退出——
#     宁可炸掉，也不要静默跑在错误的上下文里。
#   * shim 不做 PATH 处理：脚本里所有容器内命令本来就写成
#     `bash -c "source /opt/ros/jazzy/setup.bash && ..."`，在容器内执行同样成立。
# ==============================================================================

INNER_CONTAINER_NAME="${INNER_CONTAINER_NAME:-${AMR_RUNNER_CONTAINER:-amr_dispatcher}}"

docker() {
    local subcmd="${1:-}"
    [ $# -gt 0 ] && shift

    case "${subcmd}" in
        ps)
            # 只支持 `--format '{{.Names}}'` 这一种用法（脚本里也只用这种）
            printf '%s\n' "${INNER_CONTAINER_NAME}"
            return 0
            ;;
        exec)
            local detach=0
            while [ $# -gt 0 ]; do
                case "$1" in
                    -d|--detach) detach=1; shift ;;
                    -i|-t|-it|-ti|--interactive|--tty) shift ;;
                    --) shift; break ;;
                    -*) echo "[docker-shim] 不支持的 exec 选项: $1" >&2; return 2 ;;
                    *) break ;;
                esac
            done
            # 第一个非选项参数是容器名
            shift || { echo "[docker-shim] exec 缺少容器名" >&2; return 2; }
            if [ "${detach}" -eq 1 ]; then
                # 与 `docker exec -d` 一致：不等结束、不占用当前终端
                nohup "$@" >/dev/null 2>&1 &
                disown 2>/dev/null || true
                return 0
            fi
            "$@"
            ;;
        cp)
            local src="${1:-}" dst="${2:-}"
            [ $# -eq 2 ] || { echo "[docker-shim] cp 需要恰好两个参数" >&2; return 2; }
            case "${src}" in
                *:*) src="${src#*:}" ;;
            esac
            case "${dst}" in
                *:*) dst="${dst#*:}" ;;
            esac
            cp "${src}" "${dst}"
            ;;
        "")
            echo "[docker-shim] 空调用" >&2
            return 2
            ;;
        *)
            echo "[docker-shim] 内层模式下不支持 'docker ${subcmd}'；" \
                 "若脚本确实需要它，请改用本地等价实现而不是扩充 shim" >&2
            return 2
            ;;
    esac
}

export -f docker 2>/dev/null || true
