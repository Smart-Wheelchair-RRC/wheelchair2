# wheelchair2 — Localization, Mapping & Navigation Docs

This folder documents the wheelchair2 **3D localization + odometry + navigation** stack:
three forked upstream modules — **DLIO** (LiDAR-inertial odometry), **3D-BBS** (GPU
branch-and-bound global localization), and **ICP Localization** (pose refinement against a
prior `.pcd` map) — wired together to feed **Nav2**. The stack runs on a Livox MID-360 and
replaces AMCL for 3D/`mid360`-based runs by producing a stable `map → odom → base_footprint`
TF chain that Nav2 plans and drives on.

## Contents

| Document | Purpose |
| :------- | :------ |
| [LOCALIZATION_NAV_DOCS.md](./LOCALIZATION_NAV_DOCS.md) | Full technical reference: architecture diagram, per-module changes vs upstream, ROS 2 interfaces (topics/TF/params), Nav2 integration, and the maintainer divergence table. |
| [RUNNING_THE_STACK.md](./RUNNING_THE_STACK.md) | Operator runbook: prerequisites, the full-stack launch sequence, every tmux session, individual-module commands, and common issues + fixes. |

## Keeping these docs in sync with upstream

These modules are git submodules tracking **forks** of upstream repos. When you pull or
re-baseline an upstream module:

1. **Re-diff** the affected submodule against its upstream fork-point:
   - ICP → `upstream/main` (`baiyeweiguang/icp_localization_ros2`)
   - DLIO → `upstream/feature/ros2` (`vectr-ucla/direct_lidar_inertial_odometry`) — **not** `master`
   - 3D-BBS → `KOKIAOKI/3d_bbs` `main` (compare the `ros2_test/rviz2` package against `wheelchair2_3dbbs`)
   ```bash
   cd dependencies/<module> && git fetch upstream && git diff upstream/<branch>...HEAD
   ```
2. **Update** the matching "Changes From Upstream" subsection in `LOCALIZATION_NAV_DOCS.md`.
3. **Update the in-code anchors.** Source files carry `// [DOCS]` / `# [DOCS]` comments that
   link back to anchors in `LOCALIZATION_NAV_DOCS.md`. The three anchors are:
   `#icp-changes-from-upstream`, `#dlio-changes-from-upstream`, `#3d-bbs-changes-from-upstream`.
   If you add/remove a fork change, add/remove the corresponding `[DOCS]` comment and keep the
   anchor text identical on both ends.

> Anything that could not be verified directly from source/config in these docs is marked
> `⚠️ NEEDS VERIFICATION` — resolve those before relying on the affected detail.
