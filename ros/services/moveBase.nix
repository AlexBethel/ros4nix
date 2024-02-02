# Autonomous navigation.
{ config, lib, pkgs, ... }:

with lib;
let rosLib = import ./services/rosLib.nix { inherit lib; }; in
{
  options.services.ros = {
    moveBase = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          Autonomous navigation.
        '';
      };

      robotSize = {
        width = mkOption {
          type = types.number;
          description = ''
            Width of the robot, left to right.
          '';
        };

        length = mkOption {
          type = types.number;
          description = ''
            Length of the robot, front to back.
          '';
        };
      };

      globalResolution = mkOption {
        type = types.number;
        description = ''
          Resolution of the global costmap, in meters.
        '';
        default = 0.1;
      };

      globalSize = mkOption {
        type = types.number;
        description = ''
          Edge length of the global costmap, in meters.
        '';
        default = 10;
      };

      localResolution = mkOption {
        type = types.number;
        description = ''
          Resolution of the local costmap, in meters. Should be lower
          than globalResolution.
        '';
        default = 0.05;
      };

      localSize = mkOption {
        type = types.number;
        description = ''
          Edge length of the local costmap, in meters.
        '';
        default = 3;
      };

      odomFrame = mkOption {
        type = types.str;
        description = ''
          Name of the odometry TF frame.
        '';
      };

      limits = {
        forward = mkOption {
          type = types.number;
          description = ''
            Maximum forward velocity of the robot. Meters/second.
          '';
          default = 1.0;
        };

        backward = mkOption {
          type = types.number;
          description = ''
            Maximum backwards velocity of the robot. Meters/second.
          '';
          default = 1.0;
        };

        left_right = mkOption {
          type = types.number;
          description = ''
            Maximum side-to-side velocity of the robot. Meters/second.
          '';
          default = 0.0;
        };

        theta = mkOption {
          type = types.number;
          description = ''
            Maximum rotation velocity of the robot. Radians/second.
          '';
          default = 2.0;
        };

        accel_forward = mkOption {
          type = types.number;
          description = ''
            Maximum forward acceleration. Meters/second^2.
          '';
          default = 0.5;
        };

        accel_theta = mkOption {
          type = types.number;
          description = ''
            Maximum rotational acceleration. Radians/second^2.
          '';
          default = 0.5;
        };

        turning_radius = mkOption {
          type = types.number;
          description = ''
            Turning radius. Meters.
          '';
          default = 1.0;
        };
      };

      tolerance = {
        xy = mkOption {
          type = types.number;
          description = ''
            Tolerance for reaching the goal, in meters.
          '';
          default = 0.2;
        };

        yaw = mkOption {
          type = types.number;
          description = ''
            Tolerance for pointing in the goal direction, in radians.
          '';
          default = 0.1;
        };
      };
    };
  };

  config =
    let
      cfg = config.services.ros.moveBase;
    in
    mkIf cfg.enable {
      programs.ros.packages = [
        "move-base"
        "teb-local-planner"
      ];

      services.ros = {
        enable = true;
        runServices.moveBase = {
          packageName = "move_base";
          executable = "move_base";
          # FIXME: this probably isn't how we're supposed to pass this
          remap.base_local_planner = "teb_local_planner/TebLocalPlannerROS";
          rosParams =
            let
              commonCostmapParams = {
                # (Partially) documented at https://wiki.ros.org/costmap_2d.

                # Include objects that are 2.5 meters away in the local cost map.
                obstacle_range = 2.5;
                transform_tolerance = 30;

                # Raytrace free space to 3 meters. TODO: what does this mean?
                raytrace_range = 3.0;

                # Convex hull of the robot, in meters.
                footprint = [
                  [ (-cfg.robotSize.width / 2) (-cfg.robotSize.length / 2) ]
                  [ (-cfg.robotSize.width / 2) (cfg.robotSize.length / 2) ]
                  [ (cfg.robotSize.width / 2) (cfg.robotSize.length / 2) ]
                  [ (cfg.robotSize.width / 2) (-cfg.robotSize.length / 2) ]
                ];

                #robot_radius: ir_of_robot
                inflation_radius =
                  let
                    value = (
                      cfg.robotSize.width * cfg.robotSize.width +
                      cfg.robotSize.length * cfg.robotSize.length
                    );
                    # Floating-point square root. Yes, it's not in the Nix language.
                    # This version is close enough for most purposes for small-ish
                    # numbers (<10000).
                    iter1 = 1.0;
                    iter2 = (iter1 + value / iter1) / 2;
                    iter3 = (iter2 + value / iter2) / 2;
                    iter4 = (iter3 + value / iter3) / 2;
                    iter5 = (iter4 + value / iter4) / 2;
                    iter6 = (iter5 + value / iter5) / 2;
                    iter7 = (iter6 + value / iter6) / 2;
                    iter8 = (iter7 + value / iter7) / 2;
                  in
                  iter8 / 2;

                plugins = [
                  { name = "inflation"; type = "costmap_2d::InflationLayer"; }
                  { name = "obstacle"; type = "costmap_2d::ObstacleLayer"; }
                  { name = "static_layer"; type = "costmap_2d::StaticLayer"; }
                ];

                # From https://answers.ros.org/question/293696/move_base-to-use-map-published-on-a-certain-topic/
                always_send_full_costmap = true;
                global_frame = "map";
                map_topic = "map";
                map_type = "costmap";
                robot_base_frame = "base_link";
                rolling_window = true;
                static_map = false;
              };
            in
            pkgs.writeText "params.yaml"
              (builtins.toJSON {
                # https://wiki.ros.org/navigation/Tutorials/RobotSetup.

                # ---- move_base_config.yaml ----
                controller_frequency = 5.0;
                recovery_behavior_enabled = true; # possible alias issue with "behaviour" spelling
                NavfnRos = {
                  # Traverse unknown space.
                  allow_unknown = true;

                  # A tolerance on the goal point for the planner.
                  default_tolerance = 0.1;
                };
                global_costmap = commonCostmapParams // {
                  # ---- global_costmap_params.yaml ----

                  resolution = cfg.globalResolution;
                  update_frequency = 5;

                  width = cfg.globalSize;
                  height = cfg.globalSize;
                };
                local_costmap = commonCostmapParams // {
                  # ---- local_costmap_params.yaml ----

                  resolution = cfg.localResolution;
                  update_frequency = 10;

                  publish_frequency = 5;
                  width = cfg.localSize;
                  height = cfg.localSize;
                };
                # ---- base_local_planner_params.yaml ----
                TebLocalPlannerROS = {
                  odom_topic = cfg.odomFrame;

                  # Trajectory
                  teb_autosize = true;
                  dt_ref = 0.3;
                  dt_hysteresis = 0.1;
                  max_samples = 500;
                  global_plan_overwrite_orientation = true;
                  allow_init_with_backwards_motion = false;
                  max_global_plan_lookahead_dist = 3.0;
                  global_plan_viapoint_sep = -1;
                  global_plan_prune_distance = 1;
                  exact_arc_length = false;
                  feasibility_check_no_poses = 5;
                  publish_feedback = false;

                  # Robot
                  max_vel_x = cfg.limits.forward;
                  max_vel_x_backwards = cfg.limits.backward;
                  max_vel_y = cfg.limits.left_right;
                  max_vel_theta = cfg.limits.theta;
                  acc_lim_x = cfg.limits.accel_forward;
                  acc_lim_theta = cfg.limits.accel_theta;
                  min_turning_radius = cfg.limits.turning_radius;

                  footprint_model.type = "point";

                  # Goal tolerance
                  xy_goal_tolerance = cfg.tolerance.xy;
                  yaw_goal_tolerance = cfg.tolerance.yaw;
                  free_goal_vel = false;
                  complete_global_plan = true;

                  # Obstacles
                  min_obstacle_dist = 0.7; # This value must also include our robot radius, since footprint_model is set to "point".
                  inflation_dist = 0.6;
                  include_costmap_obstacles = true;
                  costmap_obstacles_behind_robot_dist = 1.5;
                  obstacle_poses_affected = 15;

                  dynamic_obstacle_inflation_dist = 0.6;
                  include_dynamic_obstacles = true;

                  costmap_converter_plugin = "";
                  costmap_converter_spin_thread = true;
                  costmap_converter_rate = 5;

                  # Optimization
                  no_inner_iterations = 5;
                  no_outer_iterations = 4;
                  optimization_activate = true;
                  optimization_verbose = false;
                  penalty_epsilon = 0.1;
                  obstacle_cost_exponent = 4;
                  weight_max_vel_x = 2;
                  weight_max_vel_theta = 1;
                  weight_acc_lim_x = 1;
                  weight_acc_lim_theta = 1;
                  weight_kinematics_nh = 1000;
                  weight_kinematics_forward_drive = 1;
                  weight_kinematics_turning_radius = 1;
                  weight_optimaltime = 1; # must be > 0
                  weight_shortest_path = 0;
                  weight_obstacle = 100;
                  weight_inflation = 0.2;
                  weight_dynamic_obstacle = 10;
                  weight_dynamic_obstacle_inflation = 0.2;
                  weight_viapoint = 1;
                  weight_adapt_factor = 2;

                  # Homotopy Class Planner
                  enable_homotopy_class_planning = true;
                  enable_multithreading = true;
                  max_number_classes = 4;
                  selection_cost_hysteresis = 1.0;
                  selection_prefer_initial_plan = 0.9;
                  selection_obst_cost_scale = 100.0;
                  selection_alternative_time_cost = false;

                  roadmap_graph_no_samples = 15;
                  roadmap_graph_area_width = 5;
                  roadmap_graph_area_length_scale = 1.0;
                  h_signature_prescaler = 0.5;
                  h_signature_threshold = 0.1;
                  obstacle_heading_threshold = 0.45;
                  switching_blocking_period = 0.0;
                  viapoints_all_candidates = true;
                  delete_detours_backwards = true;
                  max_ratio_detours_duration_best_duration = 3.0;
                  visualize_hc_graph = false;
                  visualize_with_time_as_z_axis_scale = false;

                  # Recovery
                  shrink_horizon_backup = true;
                  shrink_horizon_min_duration = 10;
                  oscillation_recovery = true;
                  oscillation_v_eps = 0.1;
                  oscillation_omega_eps = 0.1;
                  oscillation_recovery_min_duration = 10;
                  oscillation_filter_duration = 10;
                };
              });
        };
      };
    };
}
