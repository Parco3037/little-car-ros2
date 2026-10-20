# SLAM Toolbox 2.6.8 heading-gate integration tools

The original device used a local build of the SLAM Toolbox 2.6.8 scan-admission gate: scans could pass after either sufficient translation or wrapped heading change. The degree/radian convention in that exact Karto version matters.

This public source copy retains the self-authored observation, replay, static-trial and linked-method test tools, but does not distribute upstream source, the patch helper containing upstream source context, or a precompiled library. No upstream work is relicensed under the repository Apache license.

Upstream version: [SLAM Toolbox 2.6.8](https://github.com/SteveMacenski/slam_toolbox/tree/2.6.8). Its repository LICENSE is [LGPL 2.1](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.8/LICENSE); individual files also retain their own original notices. Obtain and review the exact source and terms separately before constructing a local modification.

The CMake file requires a separately prepared upstream/ and patched/ source tree. The historical runtime_evidence/slam_heading_gate_20260909 directory is not in this repository, so the original build inputs and physical acceptance evidence are not reproduced by this public tree alone.

The retained replay tool requires ROS_DOMAIN_ID=87 and localhost-only discovery; its laser/TF messages are synthetic. It must not run on the production robot domain. The static-trial and observation utilities interact with the existing ROS environment; read their preconditions before use.

Historical deployed library SHA-256: 1788d199ad57e4b868a245b37e5353ceabb34333370eefe3e52754b9aeba83be. This fingerprint is a compatibility gate for that build, not a claim that a new machine has reproduced it. Do not replace system libraries or disable runtime gates to bypass missing local assets.
