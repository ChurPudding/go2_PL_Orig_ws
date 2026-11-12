# go2_PL_Orig_ws

Point-LIO (original algorithm, optional in-filter ZUPT) packaged for the Unitree Go2 with the L1 LiDAR, ROS2 Humble.

- 간편 사용법: [README_간편.md](README_간편.md)
- 상세 설명: [README_상세.md](README_상세.md)
- 토픽·필드 설명: [TOPICS_필드설명.txt](TOPICS_필드설명.txt)

```bash
colcon build --symlink-install
live/run_live.sh            # add --zupt to enable the in-filter ZUPT
```

## Credits and license

- `src/point_lio_orig/` is derived from [Point-LIO](https://github.com/hku-mars/Point-LIO) (HKU-MARS) through the ROS2 port [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) (commit `a8e2d0d`) and stays under **GPL-2.0**. The only functional change is an in-filter zero-velocity update (off by default); see [NOTICE](NOTICE).
- `tools/`, `live/` and the documentation are **MIT** licensed ([LICENSE](LICENSE)).
- Please cite Point-LIO if you use this work ([CITATION.cff](CITATION.cff)):

> D. He, W. Xu, N. Chen, F. Kong, C. Yuan, F. Zhang, "Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry," *Advanced Intelligent Systems*, 2023, 2200459. doi:10.1002/aisy.202200459
