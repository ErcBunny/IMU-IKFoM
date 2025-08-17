# IKFoM for IMU Orientation Estimation

This node is a simple demo of IKFoM with CasADi generated code.

## Filter Formulation

See https://github.com/ErcBunny/IKFoM-Designer/blob/main/tests/imu_filter_designer.py.

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
        {"acc_confidence_decay": 0.1},
    ],
    remappings=[
        ("/imu/data", "YOUR/IMU/TOPIC"),
    ],
    output="screen",
)
```
