# Controllers/Shooter

本目录只放**单飞轮原型**：

- `PrototypeShooter/PrototypeShooter.java` — 单飞轮控制器（`@Config`，用 `FlywheelParams.BIG`）。
  用于整定、单发测试、离线回放，**不是比赛用控制器**。
  控制律本体在 `utility/Shooter/FlywheelCalculator.java`，参数在 `utility/Shooter/FlywheelParams.java`。

**比赛用的双飞轮控制器在 `Controllers/Turret/Shooter.java`**：
它内嵌 `Params{ FlywheelParams flywheel; 舵机与到位参数 }`，由 `Turret` 分别用
`Shooter.BIG_PARAMS`（大球）与 `Shooter.SMALL_PARAMS`（小球）各构造一份，
两个飞轮共用同一套计算逻辑、完全独立的参数与状态。

控制律说明见 `doc/现在的控制系统.md`；整定流程见 `OpModes/Tuning/Guide.md`。
