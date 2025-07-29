# IKFoM for IMU Orientation Estimation

This node is a simple demo of using https://github.com/hku-mars/IKFoM for state estimation.

## Filter Formulation

State:

$$
\mathbf{x} = \mathbf{R}
$$

Input:

$$
\mathbf{u} = \mathbf{\omega}_m
$$

Process noise:

$$
\mathbf{w} = \mathbf{n}_{\omega}
$$

Measurement noise:

$$
\mathbf{v} = \mathbf{n}_a
$$

System dynamics:

$$
\mathbf{f}(\mathbf{x}, \mathbf{u}, \mathbf{w}) = \mathbf{u} - \mathbf{w}
$$

Measurement model:

$$
\mathbf{h}(\mathbf{x}, \mathbf{v}) = \mathbf{R}^T \mathbf{g} + \mathbf{n}_a
$$

Partial derivatives of system dynamics:

$$
\frac{\partial \mathbf{f}(\mathbf{x \boxplus \delta x}, \mathbf{u}, \mathbf{0})}{\partial \mathbf{\delta x}} = \mathbf{0}_{3}
$$

$$
\frac{\partial \mathbf{f}(\mathbf{x}, \mathbf{u}, \mathbf{w})}{\partial \mathbf{w}} = \mathbf{I}_{3}
$$

Partial derivatives of measurement model:

$$
\frac{\partial (\mathbf{h}(\mathbf{x \boxplus \delta x}, \mathbf{0})\boxminus \mathbf{h}(\mathbf{x}, \mathbf{0}))}{\partial \mathbf{\delta x}} = [\mathbf{R}^T\mathbf{g}]_{\times}
$$

$$
\frac{\partial (\mathbf{h}(\mathbf{x}, \mathbf{v})\boxminus \mathbf{h}(\mathbf{x}, \mathbf{0}))}{\partial \mathbf{v}} = \mathbf{I}_3
$$

## Example Launch

```python
Node(
    package="imu_ikfom",
    executable="imu_ikfom_node",
    parameters=[
        {"ikfom_eps": 1e-6},
        {"ikfom_max_iter": 1000},
        {"ang_vel_var": 0.001},
        {"lin_acc_var": 0.01},
        {"enable_runtime_printing": True},
    ],
    remappings=[
        ("/imu/data", "YOUR/IMU/TOPIC"),
    ],
    output="screen",
)
```
