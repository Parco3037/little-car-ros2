# Scoped bondcpp compatibility library

For the **aarch64 MentorPi ROS Humble installation checked on 2026-08-31 only**.
Loaded by `navigation_staging.launch.py` for its Nav2 processes, not globally.
The launch verifies architecture and binary hash before starting. `/opt/ros` is unchanged.
Heartbeat monitoring and the lifecycle manager's default bond timeout remain enabled.

Original source: <https://github.com/ros/bond_core/tree/3.0.2/bondcpp>

- Archive: <https://codeload.github.com/ros/bond_core/tar.gz/refs/tags/3.0.2>
- Archive SHA-256: `16f0a741c238057146c50f4537d0e41c6c1842f7890a9fea1e782743ec9909f3`
- Installed original library SHA-256: `944ada3e657db9af3c6382905170e5b9152df63164e0a0c9d1acc8c7065fa673`
- This library SHA-256: `d110777939d51480fa4529c68c3b8f220c2b27d0a39cae48a00f052e558e539c`
- Patch: `deploy/bondcpp_3_0_2_wait_lock.patch`

Only `waitUntilFormed` and `waitUntilBroken` state reads are protected with the existing
mutex; sleeping is outside the lock. No class layout, header, state machine, timeout
or heartbeat protocol is changed. `bond.hpp` compared byte-for-byte with the installed
header; public `bond::` exported functions match; `ldd -r` with ROS sourced has no
unresolved dependencies. This does not assert compatibility with other ROS builds.

Build on the checked Pi in a fresh, timestamped directory after applying the patch:

```sh
source /opt/ros/humble/setup.bash
cmake -S bondcpp -B bond_build -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/bond_overlay"
cmake --build bond_build --parallel 2
cmake --install bond_build
```

Build result is copied into this directory and the installed RAICOM share directory.
Do not copy it into `/opt/ros`, globally change `LD_LIBRARY_PATH`, or replace it with
a new major version without checking ABI and runtime behavior.

Rollback: stop only the staging launch; restore the saved launch/code. The original
system library, base runtime, AMCL and camera containers were not changed.
