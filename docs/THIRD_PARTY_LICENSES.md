# Third-Party Licenses

This repository contains third-party code and assets under their original
licenses. The list below documents the main third-party components currently
distributed in-tree.

## Source Code And Assets Included In This Repository

| Path | Component | License |
| --- | --- | --- |
| `ORB_SLAM3_files/ORB_SLAM3/` | ORB-SLAM3 | GNU GPL v3.0 (`ORB_SLAM3_files/ORB_SLAM3/LICENSE`) |
| `ORB_SLAM3_files/ORB_SLAM3/Thirdparty/DBoW2/` | DBoW2 (vendored in ORB-SLAM3) | BSD-style license with additional redistribution notification clause (`ORB_SLAM3_files/ORB_SLAM3/Thirdparty/DBoW2/LICENSE.txt`) |
| `ORB_SLAM3_files/ORB_SLAM3/Thirdparty/g2o/` | g2o (vendored in ORB-SLAM3) | BSD (`ORB_SLAM3_files/ORB_SLAM3/Thirdparty/g2o/license-bsd.txt`) |
| `ORB_SLAM3_files/ORB_SLAM3/Thirdparty/Sophus/` | Sophus (vendored in ORB-SLAM3) | MIT (`ORB_SLAM3_files/ORB_SLAM3/Thirdparty/Sophus/LICENSE.txt`) |
| `gz/` | PX4 Gazebo models/worlds and derivatives | BSD 3-Clause at repo level (`gz/LICENSE`) plus per-model licenses where present |
| `ws/src/mission/mission/edra_colab_model.pt` | EDRA model | MIT terms (see `NOTICE`) |
| `docs/figures/` | Mixed provenance figure assets | Original copyright and license terms of each source apply |

## Components Referenced But Not Distributed Here

| Component | Status |
| --- | --- |
| QGroundControl AppImage | Used by runtime workflow but not committed in this repository |
| Ultralytics package (`ultralytics`) | Imported by mission scripts; verify license terms of the exact installed version before redistribution |

## Notes

- Root `LICENSE` (MIT) applies to project-authored files unless overridden by a
  directory/file-specific license.
- When redistributing this repository (source or images), keep all upstream
  license files and notices intact.
