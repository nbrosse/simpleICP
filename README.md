# simpleICP

This repo is a slight python adaptation from https://github.com/pglira/simpleICP.

This repo contains implementations of a rather simple version of the [Iterative Closest Point (ICP) algorithm](https://en.wikipedia.org/wiki/Iterative_closest_point) in various languages.

## Features of the ICP algorithm

### Basic features

The following basic features are implemented in all languages:

- Usage of the signed **point-to-plane distance** (instead of the point-to-point distance) as error metric. Main reasons:
  - higher convergence speed, see e.g. [here](https://www.youtube.com/watch?v=LcghboLgTiA) and [here](https://ieeexplore.ieee.org/abstract/document/924423)
  - better final point cloud alignment (under the assumption that both point clouds are differently sampled, i.e. no real point-to-point correspondences exist)
- Estimation of a **rigid-body transformation** (rotation + translation) for the movable point cloud. The final transformation is given as homogeneous transformation matrix H:

  ```
  H = [R(0,0) R(0,1) R(0,2)   tx]
      [R(1,0) R(1,1) R(1,2)   ty]
      [R(2,0) R(2,1) R(2,2)   tz]
      [     0      0      0    1]
  ```

  where ``R`` is the rotation matrix and ``tx``, ``ty``, and ``tz`` are the components of the translation vector. Using ``H``, the movable point cloud can be transformed with:

  ```
  Xt = H*X
  ```

  where ``X`` is a 4-by-n matrix holding in each column the homogeneous coordinates ``x``, ``y``, ``z``, ``1`` of a single point, and ``Xt`` is the resulting 4-by-n matrix with the transformed points.
- Selection of a **fixed number of correspondences** between the fixed and the movable point cloud. Default is ``correspondences = 1000``.
- Automatic **rejection of potentially wrong correspondences** on the basis of
  1. the [median of absolute deviations](https://en.wikipedia.org/wiki/Median_absolute_deviation). A correspondence ``i`` is rejected if ``|dist_i-median(dists)| > 3*sig_mad``, where ``sig_mad = 1.4826*mad(dists)``.
  2. the planarity of the plane used to estimate the normal vector (see below). The planarity is defined as ``P = (ev2-ev3)/ev1`` (``ev1 >= ev2 >= ev3``), where ``ev`` are the eigenvalues of the covariance matrix of the points used to estimate the normal vector. A correspondence ``i`` is rejected if ``P_i < min_planarity``. Default is ``min_planarity = 0.3``.
- After each iteration a **convergence criteria** is tested: if the mean and the standard deviation of the point-to-plane distances do not change more than ``min_change`` percent, the iteration is stopped. Default is ``min_change = 1``.
- The normal vector of the plane (needed to compute the point-to-plane distance) is estimated from the fixed point cloud using a fixed number of neighbors. Default is ``neighbors = 10``.
- The point clouds must not fully overlap, i.e. a partial overlap of the point cloud is allowed. An example for such a case is the *Bunny* dataset, see [here](#test-data-sets). The initial overlapping area between two point  clouds can be defined by the parameter ``max_overlap_distance``. More specifically, the correspondences are only selected across points of the fixed point cloud for which the initial distance to the nearest neighbor of the movable point cloud is ``<= max_overlap_distance``.

### Extended features

#### Extended feature: **observation of rigid-body transformation parameters**

This is useful in at least these cases:

1. If only a subset of the 6 rigid-body transformation parameters should be estimated. This can be accomplished by setting the weight of individual parameters to infinite, see example below.

2. If all or a subset of the 6 rigid-body transformation parameters have been directly observed in any other way, e.g. by means of a manual measurement.

3. If estimates for the rigid-body transformation parameters exist, e.g. from a previous run of simpleICP. In this case the observation weight should be set (according to the theory of least squares adjustments) to ``w = 1/observation_error^2`` whereby the ``observation_error`` is defined as ``std(observation_value)``. The observation error of all parameters is reported by simpleICP as "est.uncertainty" in the logging output.

This feature introduces two new parameters: ``rbtp_observed_values`` and ``rbtp_observation_weights``. Both parameters have exactly 6 elements which correspond to the rigid-body transformation parameters in the following order:

1. ``alpha1``: rotation angle around the x-axis
2. ``alpha2``: rotation angle around the y-axis
3. ``alpha3``: rotation angle around the z-axis
4. ``tx``: x component of translation vector
5. ``ty``: y component of translation vector
6. ``tz``: z component of translation vector

The rigid-body transformation is defined in non-homogeneous coordinates as follows:

```
Xt = RX + t
```

where ``X`` and ``Xt`` are n-by-3 matrices of the original and transformed movable point cloud, resp., ``t`` is the translation vector, and ``R`` the rotation matrix. ``R`` is thereby defined as:

```
R = [ca2*ca3               -ca2*sa3                sa2    ]
    [ca1*sa3+sa1*sa2*ca3    ca1*ca3-sa1*sa2*sa3   -sa1*ca2]
    [sa1*sa3-ca1*sa2*ca3    sa1*ca3+ca1*sa2*sa3    ca1*ca2]
```

with the substitutions:

```
sa1 := sin(alpha1), ca1 := cos(alpha1)
sa2 := sin(alpha2), ca2 := cos(alpha2)
sa3 := sin(alpha3), ca3 := cos(alpha3)
```

The two parameters ``rbtp_observed_values`` and ``rbtp_observation_weights`` can be used to introduce an additional observation to the least squares optimization for each transformation parameter:

```
residual = observation_weight * (estimated_value - observed_value)
```

Example which demonstrates the most important combinations:

```python
# parameters:              alpha1   alpha2   alpha3   tx      ty     tz
rbtp_observed_values =     (10.0     0.0     -5.0      0.20   -0.15   0.0)
rbtp_observation_weights = (100.0    0.0      0.0      40.0    40.0   inf)
```

Consequently:

- ``alpha1``: is observed to be 10 degrees with an observation weight of 100.

- ``alpha2``: is not observed since the corresponding weight is zero. However, the observed value is used as initial value for ``alpha2`` in the non-linear least squares optimization.

- ``alpha3``: is also not observed, but has an initial value of -5 degrees.

- ``tx``: is observed to be 0.20 with an observation weight of 40.

- ``ty``: is observed to be -0.15 with an observation weight of 40.

- ``tz``: is observed to be 0 with an infinite observation weight, i.e. this parameter becomes a constant and is fixed to be exactly the observation value. Thus, in this case only 5 (out of 6) rigid-body transformation parameters are estimated.

## Output

All implementations generate the same screen output. This is an example from the C++ version for the *Bunny* dataset:

```
$ run_simpleicp.sh
Processing dataset "Dragon"
Create point cloud objects ...
Select points for correspondences in fixed point cloud ...
Estimate normals of selected points ...
Start iterations ...
Iteration | correspondences | mean(residuals) |  std(residuals)
   orig:0 |             767 |          0.0001 |          0.3203
        1 |             767 |         -0.0061 |          0.2531
        2 |             773 |         -0.0035 |          0.1669
        3 |             771 |         -0.0008 |          0.0835
        4 |             741 |         -0.0006 |          0.0196
        5 |             762 |          0.0000 |          0.0025
        6 |             775 |          0.0001 |          0.0022
Convergence criteria fulfilled -> stop iteration!
Estimated transformation matrix H:
[    0.998696     0.052621    -0.034179    -0.206737]
[   -0.052090     0.999028     0.020119    -0.408088]
[    0.034822    -0.018663     0.999436    -0.593361]
[    0.000000     0.000000     0.000000     1.000000]
Finished in 1.729 seconds!
```

## References

Please cite related papers if you use this code:

```
@article{glira2015a,
  title={A Correspondence Framework for ALS Strip Adjustments based on Variants of the ICP Algorithm},
  author={Glira, Philipp and Pfeifer, Norbert and Briese, Christian and Ressl, Camillo},
  journal={Photogrammetrie-Fernerkundung-Geoinformation},
  volume={2015},
  number={4},
  pages={275--289},
  year={2015},
  publisher={E. Schweizerbart'sche Verlagsbuchhandlung}
}
```

## Related projects

- [globalICP](https://github.com/pglira/Point_cloud_tools_for_Matlab): A multi-scan ICP implementation for Matlab

## How to use

```python
from simpleicp import PointCloud, SimpleICP
import numpy as np

# Read point clouds from xyz files into n-by-3 numpy arrays
X_fix = np.genfromtxt("bunny_part1.xyz")
X_mov = np.genfromtxt("bunny_part2.xyz")

# Create point cloud objects
pc_fix = PointCloud(X_fix, columns=["x", "y", "z"])
pc_mov = PointCloud(X_mov, columns=["x", "y", "z"])

# Create simpleICP object, add point clouds, and run algorithm!
icp = SimpleICP()
icp.add_point_clouds(pc_fix, pc_mov)
H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=1)
```

This should give this output:

```
Consider partial overlap of point clouds ...
Select points for correspondences in fixed point cloud ...
Estimate normals of selected points ...
Start iterations ...
iteration | correspondences | mean(residuals) |  std(residuals)
   orig:0 |             863 |          0.0403 |          0.1825
        1 |             862 |          0.0096 |          0.1113
        2 |             775 |          0.0050 |          0.0553
        3 |             807 |          0.0022 |          0.0407
        4 |             825 |          0.0016 |          0.0346
        5 |             825 |          0.0010 |          0.0253
        6 |             816 |          0.0008 |          0.0198
        7 |             785 |         -0.0000 |          0.0142
        8 |             764 |          0.0008 |          0.0091
        9 |             753 |          0.0003 |          0.0061
       10 |             735 |          0.0002 |          0.0040
       11 |             742 |         -0.0001 |          0.0032
       12 |             747 |         -0.0000 |          0.0030
       13 |             752 |         -0.0000 |          0.0030
       14 |             752 |         -0.0000 |          0.0029
Convergence criteria fulfilled -> stop iteration!
Estimated transformation matrix H:
[    0.984798    -0.173702    -0.000053     0.000676]
[    0.173702     0.984798     0.000084    -0.001150]
[    0.000038    -0.000092     1.000000     0.000113]
[    0.000000     0.000000     0.000000     1.000000]
... which corresponds to the following rigid-body transformation parameters:
parameter |       est.value | est.uncertainty |       obs.value |      obs.weight
   alpha1 |       -0.004804 |        0.004491 |        0.000000 |       0.000e+00
   alpha2 |       -0.003061 |        0.002104 |        0.000000 |       0.000e+00
   alpha3 |       10.003124 |        0.005680 |        0.000000 |       0.000e+00
       tx |        0.000676 |        0.000418 |        0.000000 |       0.000e+00
       ty |       -0.001150 |        0.000885 |        0.000000 |       0.000e+00
       tz |        0.000113 |        0.000189 |        0.000000 |       0.000e+00
(Unit of est.value, est.uncertainty, and obs.value for alpha1/2/3 is degree)
Finished in 4.737 seconds!
```
