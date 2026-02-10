# ROS2 API 開発のための学習・参考プロンプト集

> **対象プロジェクト**: `my_fanuc_description` — FANUC産業用ロボット6シリーズ（CRX, LR Mate, M-10, M-20, R-1000iA, R-2000）のROS2 Descriptionパッケージ群

---

## 目次

1. [プロジェクト全体構造の理解](#1-プロジェクト全体構造の理解)
2. [ROS2パッケージ構成の学習](#2-ros2パッケージ構成の学習)
3. [URDF/Xacro設計パターンの理解](#3-urdfxacro設計パターンの理解)
4. [Launchファイル設計の学習](#4-launchファイル設計の学習)
5. [ビルドシステム (ament_cmake) の理解](#5-ビルドシステム-ament_cmake-の理解)
6. [ROS2 API拡張開発のためのプロンプト](#6-ros2-api拡張開発のためのプロンプト)
7. [Control系パッケージ開発のプロンプト](#7-control系パッケージ開発のプロンプト)
8. [MoveIt2連携開発のプロンプト](#8-moveit2連携開発のプロンプト)
9. [カスタムノード/サービス開発のプロンプト](#9-カスタムノードサービス開発のプロンプト)
10. [テスト・CI/CD構築のプロンプト](#10-テストcicd構築のプロンプト)

---

## 1. プロジェクト全体構造の理解

### プロンプト 1-1: リポジトリ構造の全体マップ

```
このFANUCロボットROS2 Descriptionリポジトリの全体構造を解析してください。

確認すべきポイント:
- モノレポとして6つのパッケージがどう整理されているか
- 各パッケージ間で共通するディレクトリ構造パターン
- 設定ファイル群（.pre-commit-config.yaml, .prettierrc.cjs）の役割
- ライセンス体系（LICENSES/配下の4種のライセンス）の使い分け

分析対象ファイル:
- README.md, CONTRIBUTING.md
- 各パッケージのpackage.xml, CMakeLists.txt
- .pre-commit-config.yaml

出力形式: ディレクトリツリーと各要素の役割を対応づけた表
```

### プロンプト 1-2: パッケージ間の共通性と差分

```
6つのDescriptionパッケージ（CRX, LR Mate, M-10, M-20, R-1000iA, R-2000）を比較分析し、
以下を明確にしてください:

1. 全パッケージに共通する構造（ファイル配置、命名規則）
2. パッケージごとの差異（バリアント数、メッシュファイル数）
3. 命名規則のルール:
   - パッケージ名: fanuc_{シリーズ}_description
   - ファイル名: {モデル名}.urdf.xacro / {モデル名}_urdf_macro.xacro
   - メッシュディレクトリ: meshes/{モデル名}/visual|collision/
4. 新しいロボットモデルを追加する場合に必要なファイル一式

出力形式: 比較表とテンプレート構成図
```

---

## 2. ROS2パッケージ構成の学習

### プロンプト 2-1: package.xml の設計パターン

```
fanuc_crx_description/package.xml を教材として、ROS2パッケージの
メタデータ定義を学習させてください。

解説すべき項目:
1. format="3" の意味とROS2での標準
2. 依存関係の種類と使い分け:
   - buildtool_depend (ament_cmake)
   - exec_depend (joint_state_publisher_gui, launch, launch_ros,
     robot_state_publisher, rviz2, urdf, xacro)
   - build_depend / test_depend（本プロジェクトでは未使用だが比較として）
3. <export><build_type>ament_cmake</build_type></export> の意味
4. Descriptionパッケージ特有の依存関係パターン

応用課題: 自作ROS2 APIパッケージのpackage.xmlを設計する場合、
Control/MoveIt/カスタムノードそれぞれでどの依存関係が必要になるか
```

### プロンプト 2-2: CMakeLists.txt の最小構成

```
fanuc_crx_description/CMakeLists.txt（16行）を教材として、
ROS2 ament_cmakeビルドシステムの最小構成を解説してください。

解説すべき項目:
1. cmake_minimum_required(VERSION 3.8) の推奨バージョン
2. find_package(ament_cmake REQUIRED) の役割
3. install(DIRECTORY ... DESTINATION share/${PROJECT_NAME}/) の仕組み:
   - なぜ share/ にインストールするのか
   - launch, meshes, urdf, robot, rviz 各ディレクトリの役割
4. ament_package() の内部動作

応用課題: C++ノードやPythonノードを追加する場合に
CMakeLists.txt に追加すべきセクションを具体的に示す
```

---

## 3. URDF/Xacro設計パターンの理解

### プロンプト 3-1: 2層構造（robot/*.xacro と urdf/*_macro.xacro）

```
本プロジェクトのURDF/Xacro設計は2層に分かれています。この設計を詳細に解説してください。

【第1層】robot/crx5ia.urdf.xacro（トップレベル）:
- <xacro:include> でマクロファイルを読み込み
- world リンクと ee_link リンクを定義
- <xacro:crx5ia parent="world" child="ee_link"> でロボットを配置
- origin xyz="0 0 .75" でロボット設置高さを指定

【第2層】urdf/crx5ia_urdf_macro.xacro（マクロ定義）:
- xacro:macro 定義 (name, params: prefix, parent, *origin, child)
- リンク寸法のxacro:property定義
- 関節制限パラメータの定義
- 7リンク（base_link + J1〜J6_link）の定義
- 6回転関節（J1〜J6）の定義
- 補助フレーム（wbase, flange）の定義
- 親子リンクへの接続（base_joint, child_joint）

分析ポイント:
1. なぜ2層に分離しているのか（再利用性、マルチロボット対応）
2. prefix パラメータの用途（同一モデル複数台配置）
3. *origin ブロックパラメータの仕組み
4. この設計を自分のロボットに適用するためのガイドライン
```

### プロンプト 3-2: キネマティクスチェーン構造の分析

```
crx5ia_urdf_macro.xacro のキネマティクスチェーンを数学的に分析してください。

分析項目:
1. DHパラメータ（Denavit-Hartenberg）への対応付け:
   - l_base=0.185, l_1=0.0, l_2=0.410, l_3=0.0, l_4=0.430, l_5=0.130, l_6=0.145
   - 各関節の回転軸: J1(Z), J2(Y), J3(-Y), J4(-X), J5(-Y), J6(-X)

2. 関節制限の表現方法:
   - ${radians(-200)} から ${radians(200)} のような角度→ラジアン変換
   - effort_limit（Nm）とvelocity_limit（rad/s）の物理的意味

3. 慣性パラメータの記述方法:
   - <inertial>要素: mass, origin, inertia テンソル
   - シミュレーション（Gazebo等）での利用目的

4. メッシュ参照パターン:
   - visual (DAE): レンダリング用（スケールなし）
   - collision (STL): 衝突判定用（scale="0.001 0.001 0.001" でmm→m変換）

出力: キネマティクスチェーンの図解と各パラメータの意味
```

### プロンプト 3-3: 補助フレーム（wbase, flange）の設計意図

```
本プロジェクトのURDFには、ロボットの物理的なリンク/関節とは別に
補助的なフレームが定義されています。その設計意図を解説してください。

対象フレーム:
1. wbase（World Base）:
   - base_link から l_base（0.185m）上方にオフセット
   - FANUCワールド座標系を表現
   - ロボットコントローラの座標系との対応

2. flange:
   - J6_link から l_6（0.145m）先にオフセット
   - エンドエフェクタ取り付け面を表現
   - ツール座標系の基準点

3. world / ee_link（トップレベルxacroで定義）:
   - world: ROS2のTFツリーのルート
   - ee_link: エンドエフェクタへの接続点

API開発への応用:
- これらのフレーム命名規則はROS2のどの標準に準拠しているか
- 自作APIでTFフレームを扱う際の参考ポイント
- ros2_control と連携する場合のフレーム設計
```

---

## 4. Launchファイル設計の学習

### プロンプト 4-1: Python Launchファイルの構造分析

```
view_crx.launch.py を教材として、ROS2のPython Launchシステムを
体系的に解説してください。

解説ポイント:
1. インポート構造:
   - ament_index_python: パッケージパス解決
   - launch: LaunchDescription, DeclareLaunchArgument, substitutions
   - launch_ros: Node, FindPackageShare

2. LaunchArgument パターン:
   - DeclareLaunchArgument("robot_model", choices=[...])
   - LaunchConfiguration("robot_model") での参照

3. Xacro処理パイプライン:
   - FindPackageShare → PathJoinSubstitution → PythonExpression
   - FindExecutable(name="xacro") → Command でXacro実行
   - 結果をrobot_descriptionパラメータとして渡す

4. ノード起動パターン:
   - joint_state_publisher_gui: GUIによる関節操作
   - robot_state_publisher: URDFからTF配信
   - rviz2: 可視化（--display-config で設定ファイル指定）

応用課題: このLaunchファイルを拡張して以下を追加する場合の設計
- Gazeboシミュレーション起動
- ros2_controlコントローラ起動
- カスタムノード追加
```

### プロンプト 4-2: Substitution/Expression パターン集

```
本プロジェクトのLaunchファイルで使用されている
Substitution/Expression パターンを全て抽出し解説してください。

使用されているパターン:
1. FindPackageShare("fanuc_crx_description")
   → パッケージの share/ ディレクトリパスを取得

2. PathJoinSubstitution([..., PythonExpression(["'", robot_model, ".urdf.xacro'"])])
   → 動的にURDFファイルパスを構築

3. Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", description_file])
   → xacroコマンドを実行してURDFを生成

4. LaunchConfiguration("robot_model")
   → コマンドラインから渡されたパラメータを参照

ROS2 API開発で追加的に必要になるSubstitution:
- IfCondition / UnlessCondition
- GroupAction / IncludeLaunchDescription
- SetEnvironmentVariable
- TimerAction
```

---

## 5. ビルドシステム (ament_cmake) の理解

### プロンプト 5-1: ament_cmake パッケージの構築フロー

```
本プロジェクトをベースに、ROS2 ament_cmake パッケージの
ビルドから実行までのフローを解説してください。

フロー:
1. colcon build --packages-select fanuc_crx_description
   → CMakeLists.txt の処理:
     a. find_package(ament_cmake REQUIRED)
     b. install(DIRECTORY launch meshes urdf robot rviz ...)
     c. ament_package()
   → install/ 配下にファイル配置

2. source install/setup.bash
   → ament_index にパッケージ登録
   → FindPackageShare() で検索可能に

3. ros2 launch fanuc_crx_description view_crx.launch.py robot_model:=crx5ia
   → ament_index_python でパッケージパス解決
   → xacro でURDF生成
   → ノード起動

開発用プロンプト:
- colcon build --symlink-install の利点と使い分け
- ament_python との違い（C++/Python混在パッケージ）
- カスタムメッセージ/サービスを追加する場合のビルド設定
```

---

## 6. ROS2 API拡張開発のためのプロンプト

### プロンプト 6-1: Description → Control パッケージ拡張

```
既存のfanuc_crx_descriptionパッケージをベースに、
ros2_controlを用いたControlパッケージを新規開発してください。

要件:
1. パッケージ名: fanuc_crx_control
2. 必要ファイル:
   - package.xml（ros2_control依存追加）
   - CMakeLists.txt
   - config/controllers.yaml（JointTrajectoryController等）
   - urdf/crx5ia_ros2_control.xacro（<ros2_control>タグ追加）
   - launch/control.launch.py（controller_manager起動）

3. 既存Descriptionとの連携:
   - 既存URDFマクロを<xacro:include>で再利用
   - ros2_control用のハードウェアインターフェース定義を追加
   - command_interface: position, velocity
   - state_interface: position, velocity, effort

4. コントローラ構成:
   - joint_state_broadcaster
   - joint_trajectory_controller
   - forward_position_controller（オプション）

参考とすべき本プロジェクトの設計パターン:
- 2層Xacro構造の踏襲
- Launchファイルのパラメータ化パターン
- パッケージメタデータの記述方法
```

### プロンプト 6-2: カスタムハードウェアインターフェース開発

```
FANUC CRX用のカスタムハードウェアインターフェースを
ros2_controlフレームワーク上に開発してください。

本プロジェクトのURDFから読み取るべきパラメータ:
- 関節数: 6（J1〜J6, 全てrevolute）
- 関節制限: crx5ia_urdf_macro.xacro の j*_lower_limit, j*_upper_limit
- 速度制限: j*_velocity_limit
- トルク制限: j*_effort_limit

開発すべきファイル:
1. include/fanuc_crx_hardware/fanuc_crx_hardware_interface.hpp
2. src/fanuc_crx_hardware_interface.cpp
3. fanuc_crx_hardware.xml（pluginlib記述）

実装すべきメソッド:
- on_init(): URDFからパラメータ読み込み
- export_state_interfaces(): position/velocity/effort
- export_command_interfaces(): position/velocity
- read(): 実機/シミュレータからの状態読み取り
- write(): コマンド書き込み

参考: 本プロジェクトの慣性パラメータ（<inertial>）は
シミュレーション環境での物理演算に活用可能
```

---

## 7. Control系パッケージ開発のプロンプト

### プロンプト 7-1: JointTrajectoryController の設定

```
本プロジェクトのCRX5iAロボット（6軸回転関節）に対して、
JointTrajectoryControllerを設定してください。

URDFから読み取る情報:
- 関節名: J1, J2, J3, J4, J5, J6（prefix付き対応も考慮）
- 関節タイプ: 全てrevolute
- 速度制限: 150, 150, 180, 225, 225, 225 [deg/s]

必要な設定ファイル (config/controllers.yaml):
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints: [J1, J2, J3, J4, J5, J6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    # 制約設定（URDFの関節制限に基づく）

Launch統合: 既存のview_crx.launch.pyパターンを参考に
コントローラの起動・切替ロジックを設計
```

### プロンプト 7-2: Gazebo シミュレーション統合

```
本プロジェクトの Description パッケージをGazebo（Ignition/Classic）
シミュレーションに統合するための開発を行ってください。

活用できる既存リソース:
1. URDFモデル: 完全な運動学定義 + 慣性パラメータ
2. メッシュ:
   - visual/: DAE形式（Gazeboでレンダリング可能）
   - collision/: STL形式（物理エンジン衝突判定用）
   - 注意: collision の scale="0.001" はmm→m変換

追加が必要な要素:
1. <gazebo> タグ:
   - 各リンクの物理特性（mu, kd, kp, material）
   - センサプラグイン（カメラ、力覚など）
2. gazebo_ros2_control プラグイン設定
3. World ファイル（SDF形式）
4. Launch ファイル（Gazebo + コントローラ起動）

設計指針: 本プロジェクトの2層Xacro構造を維持し、
Gazebo固有の設定は別ファイル（*_gazebo.xacro）に分離する
```

---

## 8. MoveIt2連携開発のプロンプト

### プロンプト 8-1: MoveIt2 Config パッケージ生成

```
本プロジェクトのfanuc_crx_descriptionをベースに、
MoveIt2 Configパッケージを構築してください。

URDFから抽出すべき情報:
1. キネマティクスチェーン:
   - base_link → J1 → J2 → J3 → J4 → J5 → J6 → flange → ee_link
   - Planning Group: "manipulator"
   - Base Frame: base_link
   - Tip Frame: flange (or ee_link)

2. 関節制限（crx5ia_urdf_macro.xacroより）:
   - J1: ±200° / 150°/s / 160Nm
   - J2: ±179.9° / 150°/s / 160Nm
   - J3: -68°〜248° / 180°/s / 160Nm
   - J4: ±190° / 225°/s / 40Nm
   - J5: ±179.9° / 225°/s / 40Nm
   - J6: ±225° / 225°/s / 40Nm

3. 自己衝突マトリックス: メッシュ（collision/）を使用して生成

必要な設定ファイル:
- config/crx5ia.srdf（セマンティックロボット記述）
- config/kinematics.yaml（KDL/TRAC-IK設定）
- config/joint_limits.yaml（計画用関節制限）
- config/ompl_planning.yaml（OMPL設定）
- config/moveit_controllers.yaml（コントローラ接続）
- launch/move_group.launch.py
```

### プロンプト 8-2: MoveIt2 カスタムプランナー統合

```
MoveIt2で本プロジェクトのFANUCロボットを制御する際の
カスタム動作計画APIを開発してください。

ロボット仕様（本プロジェクトから取得）:
- 6軸垂直多関節ロボット
- 作業範囲: リンク長から計算（l_2=0.410, l_4=0.430, l_5=0.130, l_6=0.145）
- prefixパラメータによるマルチロボット対応が可能な設計

開発するAPI:
1. MoveGroupInterface ラッパー:
   - go_to_joint_state(joints: List[float])
   - go_to_pose(pose: geometry_msgs/Pose)
   - plan_cartesian_path(waypoints: List[Pose])
   - attach_object / detach_object

2. Service Server:
   - /fanuc/plan_trajectory (custom_msgs/PlanTrajectory)
   - /fanuc/execute_trajectory (custom_msgs/ExecuteTrajectory)
   - /fanuc/get_robot_state (custom_msgs/GetRobotState)

3. Action Server:
   - /fanuc/follow_path (custom_msgs/FollowPath)

フレーム関係（本プロジェクトのTF構造を考慮）:
world → base_link → [J1..J6_link] → flange → ee_link
```

---

## 9. カスタムノード/サービス開発のプロンプト

### プロンプト 9-1: ロボット状態監視ノード

```
本プロジェクトのFANUCロボットの状態を監視する
カスタムROS2ノードを開発してください。

入力（本プロジェクトのTF/Topic構造から）:
- /joint_states (sensor_msgs/JointState):
  J1〜J6の position, velocity, effort
- /tf (tf2_msgs/TFMessage):
  base_link → J1_link → ... → J6_link → flange

開発するノード: fanuc_monitor_node
パブリッシュ:
- /fanuc/tcp_pose (geometry_msgs/PoseStamped):
  flangeフレームの現在位置・姿勢
- /fanuc/joint_limits_status (custom_msgs/JointLimitsStatus):
  各関節が制限値にどれだけ近いか（%）
- /fanuc/workspace_status (custom_msgs/WorkspaceStatus):
  作業範囲内/外の判定

サブスクライブ:
- /joint_states
- /tf

パラメータ（URDFから導出）:
- joint_names: [J1, J2, J3, J4, J5, J6]
- joint_limits: crx5ia_urdf_macro.xacro から取得した値
- monitor_rate: 100 Hz
```

### プロンプト 9-2: カスタムメッセージ/サービス定義

```
本プロジェクトのFANUCロボット制御API用の
カスタムメッセージ/サービス/アクションを定義してください。

ロボットのパラメータ基準（本プロジェクトのURDFに基づく）:
- 6軸ロボット、全回転関節
- 各関節にposition, velocity, effort制限あり

定義するインターフェース:

# msg/JointLimitsStatus.msg
std_msgs/Header header
string[] joint_names          # [J1, J2, J3, J4, J5, J6]
float64[] current_positions   # 現在角度 [rad]
float64[] lower_limits        # 下限 [rad]
float64[] upper_limits        # 上限 [rad]
float64[] limit_proximity     # 制限値への近さ [0.0〜1.0]
bool[] near_limit             # 制限値近接フラグ

# srv/GetKinematicsInfo.srv
string robot_model            # e.g., "crx5ia"
---
string[] joint_names
float64[] link_lengths        # [l_base, l_1, ..., l_6]
float64[] lower_limits
float64[] upper_limits
float64[] velocity_limits
float64[] effort_limits

# action/MoveToJointTarget.action
float64[] target_positions    # 目標角度 [rad]
float64 max_velocity_scaling  # 速度スケーリング [0.0〜1.0]
---
bool success
string message
---
float64[] current_positions
float64 progress              # 0.0〜1.0

パッケージ構成: fanuc_msgs/
- package.xml (rosidl_default_generators 依存)
- CMakeLists.txt (rosidl_generate_interfaces)
- msg/, srv/, action/ ディレクトリ
```

---

## 10. テスト・CI/CD構築のプロンプト

### プロンプト 10-1: URDFバリデーションテスト

```
本プロジェクトの全URDFファイルに対する自動テストを構築してください。

テスト対象（本プロジェクトの全モデル）:
- crx5ia, crx10ia, crx10ia_l, crx10ia_lp, crx20ia_l, crx30ia
- lrmate200id, lrmate200id7l
- m10_12-14d
- m20_25-18d, m20_35-18d
- r1000ia_100f
- r2000ic_125l

テスト項目:
1. Xacro → URDF 変換が成功するか
2. URDFのXML構文が正しいか（check_urdf）
3. キネマティクスチェーンが正しく構成されているか
4. 全メッシュファイルが存在し参照可能か
5. 関節制限値が物理的に妥当か（lower < upper）
6. 慣性パラメータが正値定値か

テストフレームワーク: pytest + launch_testing
CI統合: 本プロジェクトの .pre-commit-config.yaml を参考に
GitHub Actions ワークフローを構築
```

### プロンプト 10-2: コード品質・リンティング設定

```
本プロジェクトの品質管理設定を分析し、
自作ROS2 APIパッケージに適用してください。

本プロジェクトの既存設定:
1. pre-commit hooks:
   - trailing-whitespace, end-of-file-fixer
   - check-yaml, check-ast
   - Prettier (XML/Xacro フォーマット)
   - codespell (スペルチェック)
   - markdownlint-cli2 (Markdown品質)
   - Ruff (Python linting/formatting)

2. .prettierrc.cjs:
   - xmlWhitespaceSensitivity: "preserve"
   - xmlQuoteAttributes: "double"

自作APIパッケージ向けの追加設定:
- C++: clang-format, clang-tidy, cppcheck
- Python: mypy（型チェック）、Ruffの拡張ルール
- ROS2固有: ament_lint_auto, ament_lint_common
- URDF: xacro --check, check_urdf
- colcon test 統合
```

---

## 補足: プロジェクト解析サマリー

### アーキテクチャ概要

```
my_fanuc_description/
├── [共通設定] .pre-commit-config.yaml, .prettierrc.cjs
├── [ライセンス] LICENSES/ (Apache-2.0 他)
│
├── fanuc_crx_description/        # CRXシリーズ (6モデル, 100ファイル)
├── fanuc_lrmate_description/     # LR Mateシリーズ (2モデル, 36ファイル)
├── fanuc_m10_description/        # M-10シリーズ (1モデル, 20ファイル)
├── fanuc_m20_description/        # M-20シリーズ (2モデル, 36ファイル)
├── fanuc_r1000ia_description/    # R-1000iAシリーズ (1モデル, 20ファイル)
└── fanuc_r2000_description/      # R-2000シリーズ (1モデル, 20ファイル)

各パッケージ内部:
  package.xml        → メタデータ・依存関係
  CMakeLists.txt     → ビルド設定（ament_cmake）
  robot/*.xacro      → トップレベルURDF（ロボット配置）
  urdf/*_macro.xacro → マクロ定義（リンク・関節・物理パラメータ）
  meshes/            → visual(DAE) + collision(STL)
  launch/*.launch.py → Python Launch（ノード起動）
  rviz/*.rviz        → RViz可視化設定
```

### 設計パターンの要約

| パターン | 本プロジェクトでの適用 | API開発への応用 |
|---------|---------------------|---------------|
| 2層Xacro構造 | robot/*.xacro + urdf/*_macro.xacro | 再利用可能なロボット定義 |
| prefixパラメータ | マルチロボット対応 | 同一モデル複数台制御 |
| parent/child接続 | world↔base, flange↔ee_link | 柔軟な座標系連結 |
| 補助フレーム | wbase, flange | TFツリー設計の基盤 |
| LaunchArgument | robot_model選択 | パラメータ化された起動 |
| ament_cmake install | share/配下へのリソース配置 | パッケージ配布・共有 |
| pre-commit | Prettier, Ruff, codespell | コード品質の自動担保 |
