#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
측판7.yaml 같은 joint_path 모션을 ~40포인트로 자동 축약(RDP) + 부드럽게 실행되도록 보정.
생성물:
  - <원본명>_reduced40.yaml
  - <원본명>_reduced40_smoke10.yaml
"""
import sys, os, math, copy, yaml

# ---- (선택) numpy 없이 동작하도록 간단 벡터 유틸 ----
def v_sub(a, b): return [ai - bi for ai, bi in zip(a, b)]
def v_add(a, b): return [ai + bi for ai, bi in zip(a, b)]
def v_dot(a, b): return sum(ai*bi for ai, bi in zip(a, b))
def v_scale(a, s): return [ai*s for ai in a]
def v_norm(a): return math.sqrt(v_dot(a, a))

def point_to_segment_distance(p, a, b):
    """6D joint-space에서 점 p와 선분 ab의 최소거리."""
    v = v_sub(b, a)
    v2 = v_dot(v, v) + 1e-12
    w = v_sub(p, a)
    t = max(0.0, min(1.0, v_dot(w, v) / v2))
    closest = v_add(a, v_scale(v, t))
    return v_norm(v_sub(p, closest))

def rdp_indices(points, epsilon):
    """
    Ramer–Douglas–Peucker in 6D joint space.
    points: list[list[float]]  (N x 6)
    epsilon: 허용 오차(도)
    return: 보존할 인덱스 리스트(오름차순)
    """
    keep = set([0, len(points)-1])

    def _rdp(i0, i1):
        if i1 <= i0 + 1:
            return
        a, b = points[i0], points[i1]
        max_d, max_i = -1.0, None
        for i in range(i0+1, i1):
            d = point_to_segment_distance(points[i], a, b)
            if d > max_d:
                max_d, max_i = d, i
        if max_i is not None and max_d > epsilon:
            keep.add(max_i)
            _rdp(i0, max_i)
            _rdp(max_i, i1)
    _rdp(0, len(points)-1)
    return sorted(keep)

def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def save_yaml(obj, path):
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(obj, f, allow_unicode=True, sort_keys=False)

def reduce_to_target(points, target=40, min_keep=30, lo=0.0, hi=10.0, iters=40):
    """
    epsilon을 이분 탐색해서 ~target개에 맞춤.
    너무 적게 나오면 min_keep까지는 늘려줌.
    """
    best = list(range(len(points)))
    l, h = lo, hi
    for _ in range(iters):
        mid = (l + h) / 2.0
        idx = rdp_indices(points, mid)
        if len(idx) > target:
            l = mid
        else:
            best = idx
            h = mid
    if len(best) < min_keep and len(points) >= min_keep:
        # 최소 유지 개수 맞추기 위해 epsilon 낮춰 재시도
        l2, h2 = 0.0, h
        for _ in range(iters):
            mid = (l2 + h2) / 2.0
            idx = rdp_indices(points, mid)
            if len(idx) < min_keep:
                l2 = mid
            else:
                best = idx
                h2 = mid
    return best

def main():
    if len(sys.argv) < 2:
        print("사용법: python3 reduce_motion.py 측판7.yaml")
        sys.exit(1)

    src = sys.argv[1]
    base, ext = os.path.splitext(src)
    out40 = f"{base}_reduced40.yaml"
    out10 = f"{base}_reduced40_smoke10.yaml"

    data = load_yaml(src)
    motion = data["motions"][0]  # joint_path/j_add 가정
    wps = [w for w in motion["waypoints"] if "joints" in w]
    if len(wps) < 2:
        print("웨이포인트가 부족합니다.")
        sys.exit(2)

    # 6D joint array
    J = [[float(x) for x in w["joints"]] for w in wps]

    # ~40개로 축약
    keep_idx = reduce_to_target(J, target=40, min_keep=30)
    reduced_wps = []
    for new_id, old_i in enumerate(keep_idx):
        src_wp = wps[old_i]
        speed = float(src_wp.get("speed", motion.get("parameters", {}).get("joint_speed_deg_s", 40)))
        blend = float(src_wp.get("blend_radius", 5.0))
        # 더 부드럽게: blend 최소 5
        blend = max(5.0, blend)
        reduced_wps.append({
            "id": new_id,
            "type": "joint",
            "joints": [float(x) for x in src_wp["joints"]],
            "speed": speed,
            "blend_radius": blend,
        })

    # 전역 파라미터 보정(너가 쓰는 파이프라인 호환)
    reduced_motion = copy.deepcopy(motion)
    reduced_motion["name"] = motion.get("name", "motion") + "_reduced40"
    params = reduced_motion.setdefault("parameters", {})
    params["joint_speed_deg_s"] = max(40, int(params.get("joint_speed_deg_s", 40)))
    params["joint_accel_deg_s2"] = max(80, int(params.get("joint_accel_deg_s2", 80)))
    params["blend_option"] = "RADIUS"
    params["blend_type"] = params.get("blend_type", "INTENDED")
    reduced_motion["waypoints"] = reduced_wps

    # 첫 포인트 진입용 preposition
    pre = {
        "name": reduced_motion["name"] + "_preposition",
        "motion_type": "joint_path",
        "execution_mode": "j_add",
        "coordinate_frame": reduced_motion.get("coordinate_frame", "robot_base_frame"),
        "parameters": {
            "joint_speed_deg_s": 20,
            "joint_accel_deg_s2": 40,
            "blend_option": "RADIUS",
            "blend_type": params["blend_type"],
        },
        "waypoints": [{
            "id": 0,
            "type": "joint",
            "joints": reduced_wps[0]["joints"],
            "speed": 20,
            "blend_radius": 10
        }]
    }

    # 저장: full(40) + smoke10
    reduced_doc = {"motions": [pre, reduced_motion]}
    save_yaml(reduced_doc, out40)

    smoke_motion = copy.deepcopy(reduced_motion)
    smoke_motion["name"] = reduced_motion["name"] + "_smoke10"
    smoke_motion["waypoints"] = [copy.deepcopy(wp) for wp in reduced_wps[:10]]
    smoke_doc = {"motions": [pre, smoke_motion]}
    save_yaml(smoke_doc, out10)

    print("생성 완료:")
    print(" -", out40)
    print(" -", out10)

if __name__ == "__main__":
    main()

