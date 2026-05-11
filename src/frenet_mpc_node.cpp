/**
 * F1TENTH Frenet MPC Node — acados SQP-RTI LINEAR_LS
 * ===================================================
 * State:   [s, e_y, e_psi, v, delta]
 * Control: [delta_dot, a]
 * Param:   [kappa]
 * Output:  /drive AckermannDriveStamped
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_f1tenth_frenet.h"
#include "geometry_msgs/msg/point.hpp"

static constexpr int NX = 5;
static constexpr int NU = 2;
static constexpr int NY = 6;
static constexpr int NY_E = 4;
static constexpr int NP = 1;
static constexpr int N = 30;
static constexpr double DT = 0.08;
static constexpr double L_WB = 0.33;

// Bounds — must match the regenerated solver (acados_generator.py)
static constexpr double EY_BOUND = 1.0;
static constexpr double EPSI_BOUND = 0.7;

struct RawPoint { double x = 0, y = 0; };

struct TrackPoint {
    double s = 0, x = 0, y = 0, yaw = 0, kappa = 0, v_ref = 0;
    double ey_min = -1.0;
    double ey_max =  1.0;
};

struct Projection {
    double s = 0, e_y = 0, yaw_ref = 0, kappa = 0, x_ref = 0, y_ref = 0;
    int seg_idx = 0;
};

class ObstacleKF {
public:
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity() * 10.0;
    bool active = false;
    int miss_count = 0;
    static constexpr int MAX_MISS = 8;

    void predict(double dt) {
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = dt;  F(1, 3) = dt;
        Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
        Q(0,0) = 0.01; Q(1,1) = 0.01; Q(2,2) = 2.0; Q(3,3) = 1.0;
        Q *= dt;
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(double s_meas, double ey_meas, double track_len) {
        Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
        H(0, 0) = 1.0;  H(1, 1) = 1.0;
        Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.05;
        double s_innov = s_meas - x(0);
        if (s_innov > track_len * 0.5)  s_innov -= track_len;
        if (s_innov < -track_len * 0.5) s_innov += track_len;
        Eigen::Vector2d y(s_innov, ey_meas - x(1));
        Eigen::Matrix2d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
        x = x + K * y;
        if (track_len > 0) {
            x(0) = std::fmod(x(0), track_len);
            if (x(0) < 0) x(0) += track_len;
        }
        P = (Eigen::Matrix4d::Identity() - K * H) * P;
        miss_count = 0;
        active = true;
    }

    void miss() { if (++miss_count > MAX_MISS) active = false; }

    double s()      const { return x(0); }
    double ey()     const { return x(1); }
    double s_dot()  const { return x(2); }
    double ey_dot() const { return x(3); }
};

static double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static double quat_to_yaw(double qx, double qy, double qz, double qw) {
    return std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
}

static std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(item);
    return out;
}

class FrenetMPC : public rclcpp::Node {
public:
    FrenetMPC() : Node("frenet_mpc") {
        declare_parameter<std::string>("centerline_file", "/home/nvidia/f1tenth_ws/src/centerline.csv");
        declare_parameter<std::string>("pose_topic", "/pf/viz/inferred_pose");
        declare_parameter<std::string>("odom_topic", "/odom_ekf");
        declare_parameter<std::string>("drive_topic", "/drive");
        declare_parameter<std::string>("scan_topic", "/scan");
        declare_parameter<std::string>("map_topic", "/map");
        declare_parameter<std::string>("global_frame", "map");

        declare_parameter<double>("target_speed", 2.0);
        declare_parameter<double>("min_speed_cmd", 0.30);
        declare_parameter<double>("max_speed", 6.0);
        declare_parameter<double>("resample_ds", 0.05);
        declare_parameter<double>("v_min_corner", 1.0);
        declare_parameter<double>("curvature_speed_gain", 3.5);
        declare_parameter<double>("friction_mu", 0.7);
        declare_parameter<double>("max_search_dist_warn", 1.5);
        declare_parameter<double>("max_steering_angle", 0.6189);
        declare_parameter<double>("max_steering_rate", 3.2);
        declare_parameter<bool>("reverse_centerline", false);
        declare_parameter<int>("rti_iterations", 3);
        declare_parameter<double>("max_brake_decel", 6.0);
        declare_parameter<double>("max_accel_forward", 6.0);

        declare_parameter<bool>("enable_obstacle_avoidance", true);
        declare_parameter<double>("obs_safety_margin", 0.35);
        declare_parameter<double>("obs_car_half_width", 0.18);
        declare_parameter<double>("obs_car_half_length", 0.25);
        declare_parameter<double>("obs_detect_range", 6.0);
        declare_parameter<double>("obs_cluster_dist", 0.20);
        declare_parameter<int>("obs_min_cluster_pts", 3);
        declare_parameter<double>("obs_side_commit_time", 0.5);
        declare_parameter<double>("obs_map_diff_thresh", 0.15);
        declare_parameter<double>("track_width_default", 0.80);
        declare_parameter<double>("track_wall_margin", 0.22);
        declare_parameter<double>("emergency_brake_speed", 0.0);

        declare_parameter<double>("w_ey", 35.0);
        declare_parameter<double>("w_epsi", 15.0);
        declare_parameter<double>("w_v", 3.0);
        declare_parameter<double>("w_delta", 12.0);
        declare_parameter<double>("w_delta_dot", 65.0);
        declare_parameter<double>("w_a", 0.20);
        declare_parameter<double>("w_ey_e", 45.0);
        declare_parameter<double>("w_epsi_e", 18.0);
        declare_parameter<double>("w_v_e", 3.0);
        declare_parameter<double>("w_delta_e", 1.0);

        centerline_file_  = get_parameter("centerline_file").as_string();
        pose_topic_       = get_parameter("pose_topic").as_string();
        odom_topic_       = get_parameter("odom_topic").as_string();
        drive_topic_      = get_parameter("drive_topic").as_string();
        scan_topic_       = get_parameter("scan_topic").as_string();
        map_topic_        = get_parameter("map_topic").as_string();
        global_frame_     = get_parameter("global_frame").as_string();
        target_speed_     = get_parameter("target_speed").as_double();
        min_speed_cmd_    = get_parameter("min_speed_cmd").as_double();
        max_speed_        = get_parameter("max_speed").as_double();
        resample_ds_      = get_parameter("resample_ds").as_double();
        v_min_corner_     = get_parameter("v_min_corner").as_double();
        curvature_speed_gain_ = get_parameter("curvature_speed_gain").as_double();
        friction_mu_          = get_parameter("friction_mu").as_double();
        max_search_dist_warn_ = get_parameter("max_search_dist_warn").as_double();
        max_steering_angle_ = get_parameter("max_steering_angle").as_double();
        max_steering_rate_  = get_parameter("max_steering_rate").as_double();
        reverse_centerline_ = get_parameter("reverse_centerline").as_bool();
        rti_iterations_   = get_parameter("rti_iterations").as_int();
        max_brake_decel_  = get_parameter("max_brake_decel").as_double();
        max_accel_forward_= get_parameter("max_accel_forward").as_double();
        enable_obs_       = get_parameter("enable_obstacle_avoidance").as_bool();
        obs_margin_       = get_parameter("obs_safety_margin").as_double();
        obs_hw_           = get_parameter("obs_car_half_width").as_double();
        obs_hl_           = get_parameter("obs_car_half_length").as_double();
        obs_range_        = get_parameter("obs_detect_range").as_double();
        obs_cluster_d_    = get_parameter("obs_cluster_dist").as_double();
        obs_min_pts_      = get_parameter("obs_min_cluster_pts").as_int();
        side_commit_t_    = get_parameter("obs_side_commit_time").as_double();
        map_diff_thresh_  = get_parameter("obs_map_diff_thresh").as_double();
        track_w_default_  = get_parameter("track_width_default").as_double();
        track_wall_margin_= get_parameter("track_wall_margin").as_double();
        emergency_brake_speed_ = get_parameter("emergency_brake_speed").as_double();

        for (int k = 0; k <= N; ++k) prev_s_[k] = 0.0;
        for (int k = 0; k <= N; ++k) kappa_horizon_[k] = 0.0;

        load_and_prepare_track(centerline_file_);
        init_solver();
        configure_solver_constraints();
        configure_cost_weights();

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10, std::bind(&FrenetMPC::pose_callback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&FrenetMPC::odom_callback, this, std::placeholders::_1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10, std::bind(&FrenetMPC::scan_callback, this, std::placeholders::_1));
        map_sub_  = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, rclcpp::QoS(1).transient_local(),
            std::bind(&FrenetMPC::map_callback, this, std::placeholders::_1));

        drive_pub_      = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
        solve_time_pub_ = create_publisher<std_msgs::msg::Float64>("/frenet_mpc/solve_time_ms", 10);
        state_pub_      = create_publisher<std_msgs::msg::Float64MultiArray>("/frenet_mpc/state", 10);
        traj_pub_       = create_publisher<visualization_msgs::msg::MarkerArray>("/frenet_mpc/predicted_trajectory", 10);
        ref_pub_        = create_publisher<visualization_msgs::msg::MarkerArray>("/frenet_mpc/reference_trajectory", 10);
        frame_pub_      = create_publisher<visualization_msgs::msg::MarkerArray>("/frenet_mpc/frame", 10);
        obs_pub_        = create_publisher<visualization_msgs::msg::MarkerArray>("/frenet_mpc/obstacle", 10);
        corridor_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("/frenet_mpc/corridor", 10);

        RCLCPP_INFO(get_logger(), "Frenet MPC+OBS ready | track=%zu | L=%.2f | v=%.1f | obs=%s",
                    track_.size(), track_length_, target_speed_, enable_obs_?"ON":"OFF");
    }

    ~FrenetMPC() override {
        if (capsule_) { f1tenth_frenet_acados_free(capsule_); f1tenth_frenet_acados_free_capsule(capsule_); }
    }

private:
    // config
    std::string centerline_file_, pose_topic_, odom_topic_, drive_topic_;
    std::string scan_topic_, map_topic_, global_frame_;
    double target_speed_=2, min_speed_cmd_=0.3, max_speed_=6;
    double resample_ds_=0.05, v_min_corner_=1, curvature_speed_gain_=3.5, friction_mu_=0.7;
    double max_search_dist_warn_=1.5, max_steering_angle_=0.6189, max_steering_rate_=3.2;
    bool reverse_centerline_=false;
    int rti_iterations_=3;
    double max_brake_decel_=6, max_accel_forward_=6;
    bool enable_obs_=true;
    double obs_margin_=0.35, obs_hw_=0.18, obs_hl_=0.25, obs_range_=6;
    double obs_cluster_d_=0.20; int obs_min_pts_=3;
    double side_commit_t_=0.5, map_diff_thresh_=0.15;
    double track_w_default_=0.80, track_wall_margin_=0.22, emergency_brake_speed_=0;

    // track
    std::vector<TrackPoint> track_;
    double track_length_ = 0;

    // ego
    double current_v_=0, ego_x_=0, ego_y_=0, ego_yaw_=0, ego_s_=0;
    bool has_odom_=false, has_pose_=false;
    double last_delta_cmd_ = 0;
    int last_projection_seg_ = 0;

    // warm-start + diagnostics
    double prev_s_[N + 1];
    double kappa_horizon_[N + 1];
    bool has_prev_solution_ = false;
    int solve_failure_count_ = 0;

    // map
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    bool has_map_=false, track_widths_computed_=false;

    // scan
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    bool has_scan_ = false;

    // obstacle
    ObstacleKF obs_kf_;
    int committed_side_ = 0;
    rclcpp::Time side_commit_time_{0, 0, RCL_ROS_TIME};
    int minstep_count_ = 0;

    // solver
    f1tenth_frenet_solver_capsule* capsule_ = nullptr;
    ocp_nlp_config* nlp_config_ = nullptr;
    ocp_nlp_dims* nlp_dims_ = nullptr;
    ocp_nlp_in* nlp_in_ = nullptr;
    ocp_nlp_out* nlp_out_ = nullptr;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr solve_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_, ref_pub_, frame_pub_, obs_pub_, corridor_pub_;

    // ═══════════════════ DIAGNOSTICS ════════════════════════════════════════

    // Pull all the relevant solver stats and print them when a solve fails.
    // Rate-limited by solve_failure_count_ so we don't flood the console
    // when MINSTEP fires on every callback through a hairpin.
    void log_solver_failure(int status, const char* tag, const double x0[NX]) {
        solve_failure_count_++;
        if (solve_failure_count_ != 1 && solve_failure_count_ % 10 != 0) return;

        const char* status_str = "?";
        switch (status) {
            case 0: status_str = "SUCCESS"; break;
            case 1: status_str = "INFEASIBLE"; break;
            case 2: status_str = "MAXITER"; break;
            case 3: status_str = "MINSTEP"; break;
            case 4: status_str = "QP_FAIL"; break;
        }

        RCLCPP_WARN(get_logger(),
            "[MPC FAIL #%d %s] status=%d (%s)",
            solve_failure_count_, tag, status, status_str);

        // ocp_nlp_get signature in this acados version: (solver, field, value)
        double res[4] = {0,0,0,0};
        ocp_nlp_get(capsule_->nlp_solver, "residuals", res);
        RCLCPP_WARN(get_logger(),
            "  residuals: stat=%.2e eq=%.2e ineq=%.2e comp=%.2e",
            res[0], res[1], res[2], res[3]);

        int sqp_iter = 0, qp_iter_total = 0, qp_stat = 0;
        ocp_nlp_get(capsule_->nlp_solver, "sqp_iter", &sqp_iter);
        ocp_nlp_get(capsule_->nlp_solver, "qp_iter",  &qp_iter_total);
        ocp_nlp_get(capsule_->nlp_solver, "qp_stat",  &qp_stat);
        RCLCPP_WARN(get_logger(),
            "  sqp_iter=%d qp_iter=%d qp_stat=%d",
            sqp_iter, qp_iter_total, qp_stat);

        RCLCPP_WARN(get_logger(),
            "  x0: s=%.2f e_y=%.3f e_psi=%.3f v=%.2f delta=%.3f",
            x0[0], x0[1], x0[2], x0[3], x0[4]);

        double kmin = 1e9, kmax = -1e9, kmean = 0, kjump_max = 0;
        for (int k = 0; k <= N; ++k) {
            kmin = std::min(kmin, kappa_horizon_[k]);
            kmax = std::max(kmax, kappa_horizon_[k]);
            kmean += kappa_horizon_[k];
            if (k > 0) {
                double dj = std::abs(kappa_horizon_[k] - kappa_horizon_[k-1]);
                kjump_max = std::max(kjump_max, dj);
            }
        }
        kmean /= (N + 1);
        RCLCPP_WARN(get_logger(),
            "  kappa: min=%.3f max=%.3f mean=%.3f max_jump=%.3f",
            kmin, kmax, kmean, kjump_max);
    }

    // ═══════════════════ SOLVER INIT ════════════════════════════════════════

    void init_solver() {
        capsule_ = f1tenth_frenet_acados_create_capsule();
        if (f1tenth_frenet_acados_create(capsule_) != 0) {
            RCLCPP_FATAL(get_logger(), "acados create failed"); rclcpp::shutdown(); return;
        }
        nlp_config_ = f1tenth_frenet_acados_get_nlp_config(capsule_);
        nlp_dims_ = f1tenth_frenet_acados_get_nlp_dims(capsule_);
        nlp_in_ = f1tenth_frenet_acados_get_nlp_in(capsule_);
        nlp_out_ = f1tenth_frenet_acados_get_nlp_out(capsule_);
        double x0[NX]={0,0,0,0,0}; double u0[NU]={0,0};
        for (int k=0; k<=N; ++k) ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, k, "x", x0);
        for (int k=0; k<N; ++k)  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, k, "u", u0);
    }

    void configure_solver_constraints() {
        double lbu[NU]={-max_steering_rate_,-8}, ubu[NU]={max_steering_rate_,8};
        // Order matches idxbx = [1,2,3,4] = [e_y, e_psi, v, delta]
        double lbx[4] = {-EY_BOUND, -EPSI_BOUND, 0.0,         -max_steering_angle_};
        double ubx[4] = { EY_BOUND,  EPSI_BOUND, max_speed_,   max_steering_angle_};
        for (int k=0; k<N; ++k) {
            ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,k,"lbu",lbu);
            ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,k,"ubu",ubu);
        }
        for (int k=1; k<=N; ++k) {
            ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,k,"lbx",lbx);
            ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,k,"ubx",ubx);
        }
    }

    void configure_cost_weights() {
        Eigen::Matrix<double,NY,NY> W = Eigen::Matrix<double,NY,NY>::Zero();
        W(0,0)=get_parameter("w_ey").as_double();
        W(1,1)=get_parameter("w_epsi").as_double();
        W(2,2)=get_parameter("w_v").as_double();
        W(3,3)=get_parameter("w_delta").as_double();
        W(4,4)=get_parameter("w_delta_dot").as_double();
        W(5,5)=get_parameter("w_a").as_double();
        Eigen::Matrix<double,NY_E,NY_E> We = Eigen::Matrix<double,NY_E,NY_E>::Zero();
        We(0,0)=get_parameter("w_ey_e").as_double();
        We(1,1)=get_parameter("w_epsi_e").as_double();
        We(2,2)=get_parameter("w_v_e").as_double();
        We(3,3)=get_parameter("w_delta_e").as_double();
        for (int k=0; k<N; ++k) ocp_nlp_cost_model_set(nlp_config_,nlp_dims_,nlp_in_,k,"W",W.data());
        ocp_nlp_cost_model_set(nlp_config_,nlp_dims_,nlp_in_,N,"W",We.data());
    }

    // ═══════════════════ CALLBACKS ══════════════════════════════════════════

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr m) { current_v_=std::abs(m->twist.twist.linear.x); has_odom_=true; }
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr m) { last_scan_=m; has_scan_=true; }
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        map_msg_=m; has_map_=true;
        if (!track_.empty() && !track_widths_computed_) { compute_track_widths_from_map(); track_widths_computed_=true; }
    }

    // ═══════════════════ TRACK LOADING ══════════════════════════════════════

    void load_and_prepare_track(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) { RCLCPP_FATAL(get_logger(),"Cannot open %s",path.c_str()); rclcpp::shutdown(); return; }
        std::vector<RawPoint> raw;
        std::string line;
        while (std::getline(f,line)) {
            if (line.empty()||line[0]=='#') continue;
            auto t=split_csv(line); if(t.size()<2) continue;
            try { raw.push_back({std::stod(t[0]),std::stod(t[1])}); } catch(...) {}
        }
        if (raw.size()<5) { RCLCPP_FATAL(get_logger(),"Need >=5 pts"); rclcpp::shutdown(); return; }
        if (reverse_centerline_) std::reverse(raw.begin(),raw.end());
        std::vector<RawPoint> cl; cl.push_back(raw[0]);
        for (size_t i=1;i<raw.size();++i) if(std::hypot(raw[i].x-cl.back().x,raw[i].y-cl.back().y)>1e-4) cl.push_back(raw[i]);
        raw=cl;
        std::vector<double> sr(raw.size()+1,0);
        for (size_t i=0;i<raw.size();++i) { size_t j=(i+1)%raw.size(); sr[i+1]=sr[i]+std::hypot(raw[j].x-raw[i].x,raw[j].y-raw[i].y); }
        track_length_=sr.back();
        if (track_length_<1) { RCLCPP_FATAL(get_logger(),"Track too short"); rclcpp::shutdown(); return; }
        int M=std::max(20,(int)std::round(track_length_/std::max(0.01,resample_ds_)));
        track_.clear(); track_.reserve(M);
        for (int m=0;m<M;++m) {
            double s=track_length_*m/(double)M;
            RawPoint p=interp_raw(raw,sr,s);
            TrackPoint tp; tp.s=s; tp.x=p.x; tp.y=p.y; tp.v_ref=target_speed_;
            tp.ey_min=-track_w_default_; tp.ey_max=track_w_default_;
            track_.push_back(tp);
        }
        std::vector<double> sn(track_.size()+1,0);
        for (size_t i=0;i<track_.size();++i) { size_t j=(i+1)%track_.size(); sn[i+1]=sn[i]+std::hypot(track_[j].x-track_[i].x,track_[j].y-track_[i].y); }
        track_length_=sn.back();
        for (size_t i=0;i<track_.size();++i) track_[i].s=sn[i];
        compute_yaw_curvature_speed();
    }

    RawPoint interp_raw(const std::vector<RawPoint>& raw, const std::vector<double>& sr, double s) const {
        double sm=wrap_s(s);
        auto it=std::upper_bound(sr.begin(),sr.end(),sm);
        size_t i=(size_t)std::max(0,(int)std::distance(sr.begin(),it)-1);
        if(i>=raw.size()) i=raw.size()-1;
        size_t j=(i+1)%raw.size();
        double a=(sm-sr[i])/std::max(1e-9,sr[i+1]-sr[i]);
        return {raw[i].x+a*(raw[j].x-raw[i].x), raw[i].y+a*(raw[j].y-raw[i].y)};
    }

    void compute_yaw_curvature_speed() {
        const int n=(int)track_.size();
        std::vector<double> yaw(n);
        for (int i=0;i<n;++i) { int ip=(i+1)%n,im=(i-1+n)%n; yaw[i]=std::atan2(track_[ip].y-track_[im].y,track_[ip].x-track_[im].x); }
        for (int i=1;i<n;++i) { double d=yaw[i]-yaw[i-1]; while(d>M_PI){yaw[i]-=2*M_PI;d-=2*M_PI;} while(d<-M_PI){yaw[i]+=2*M_PI;d+=2*M_PI;} }
        for (int i=0;i<n;++i) {
            int ip=(i+1)%n,im=(i-1+n)%n;
            double yi=yaw[ip],ym=yaw[im];
            if(i==0) ym-=2*M_PI*std::round((ym-yaw[i])/(2*M_PI));
            if(i==n-1) yi+=2*M_PI*std::round((yaw[i]-yi)/(2*M_PI));
            double ds=std::max(1e-6,arc_diff(track_[ip].s,track_[i].s)+arc_diff(track_[i].s,track_[im].s));
            double kappa=normalize_angle(yi-ym)/ds;
            track_[i].yaw=normalize_angle(yaw[i]);
            track_[i].kappa=std::clamp(kappa,-3.5,3.5);

            double abs_kappa = std::abs(kappa);
            double v_friction = (abs_kappa > 0.01)
                ? std::sqrt(friction_mu_ * 9.81 / abs_kappa)
                : target_speed_;
            double v_exponential = target_speed_ * std::exp(-curvature_speed_gain_ * abs_kappa);
            track_[i].v_ref = std::clamp(std::min(v_friction, v_exponential), v_min_corner_, target_speed_);
        }
        for (int iter=0;iter<4;++iter) {
            std::vector<double> v(n);
            for (int i=0;i<n;++i) { int ip=(i+1)%n,im=(i-1+n)%n; v[i]=0.25*track_[im].v_ref+0.5*track_[i].v_ref+0.25*track_[ip].v_ref; }
            for (int i=0;i<n;++i) track_[i].v_ref=std::clamp(v[i],v_min_corner_,target_speed_);
        }
        for (int p=0;p<3;++p) {
            for (int i=n-1;i>=0;--i) { int j=(i+1)%n; double ds=arc_diff(track_[j].s,track_[i].s);
                track_[i].v_ref=std::min(track_[i].v_ref,std::sqrt(track_[j].v_ref*track_[j].v_ref+2*max_brake_decel_*ds)); }
            for (int i=0;i<n;++i) { int j=(i+1)%n; double ds=arc_diff(track_[j].s,track_[i].s);
                double v_i = track_[i].v_ref;
                double a_lat = v_i * v_i * std::abs(track_[i].kappa);
                double a_max_sq = max_accel_forward_ * max_accel_forward_;
                double a_lon_available = 0.0;
                if (a_lat < max_accel_forward_) {
                    a_lon_available = std::sqrt(std::max(0.0, a_max_sq - a_lat * a_lat));
                }
                a_lon_available = std::max(a_lon_available, 0.3);
                track_[j].v_ref=std::min(track_[j].v_ref,std::sqrt(v_i*v_i+2*a_lon_available*ds)); }
        }
        for (int i=0;i<n;++i) track_[i].v_ref=std::clamp(track_[i].v_ref,v_min_corner_,target_speed_);
        double minv=1e9,maxv=-1e9,maxk=0;
        for (auto& p:track_) { minv=std::min(minv,p.v_ref); maxv=std::max(maxv,p.v_ref); maxk=std::max(maxk,std::abs(p.kappa)); }
        RCLCPP_INFO(get_logger(),"Track: %zu pts L=%.2f v=[%.2f,%.2f] max|k|=%.3f",track_.size(),track_length_,minv,maxv,maxk);
    }

    void compute_track_widths_from_map() {
        if (!has_map_||track_.empty()) return;
        const auto& info=map_msg_->info;
        double res=info.resolution, ox=info.origin.position.x, oy=info.origin.position.y;
        int w=(int)info.width, h=(int)info.height;
        auto occ=[&](double wx,double wy)->bool {
            int mx=(int)((wx-ox)/res), my=(int)((wy-oy)/res);
            if(mx<0||mx>=w||my<0||my>=h) return true;
            int8_t v=map_msg_->data[my*w+mx]; return v>50||v<0;
        };
        double step=res*0.5, maxcast=3.0;
        for (auto& tp:track_) {
            double nx=-std::sin(tp.yaw), ny=std::cos(tp.yaw);
            double dl=0, dr=0;
            for (double d=step;d<=maxcast;d+=step) { if(occ(tp.x+d*nx,tp.y+d*ny)) break; dl=d; }
            for (double d=step;d<=maxcast;d+=step) { if(occ(tp.x-d*nx,tp.y-d*ny)) break; dr=d; }
            tp.ey_max =  (dl - track_wall_margin_);
            tp.ey_min = -(dr - track_wall_margin_);
            tp.ey_min = std::min(tp.ey_min, -0.05);
            tp.ey_max = std::max(tp.ey_max,  0.05);
            // Also clamp to the solver's hard e_y bound so we never set
            // a corridor that exceeds what the QP allows.
            tp.ey_max = std::min(tp.ey_max,  EY_BOUND - 0.02);
            tp.ey_min = std::max(tp.ey_min, -EY_BOUND + 0.02);
        }
        double minw=1e9,maxw=-1e9;
        for (auto& tp:track_) { double tw=tp.ey_max-tp.ey_min; minw=std::min(minw,tw); maxw=std::max(maxw,tw); }
        RCLCPP_INFO(get_logger(),"Track widths: [%.2f,%.2f] m (margin=%.2f)",minw,maxw,track_wall_margin_);
    }

    struct DynPoint { double x,y,s,ey; };

    bool is_near_wall(int gx, int gy, int map_w, int map_h, int radius) const {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                int cx = gx + dx, cy2 = gy + dy;
                if (cx < 0 || cx >= map_w || cy2 < 0 || cy2 >= map_h) continue;
                int8_t val = map_msg_->data[cy2 * map_w + cx];
                if (val > 30 || val < 0) return true;
            }
        }
        return false;
    }

    std::vector<DynPoint> detect_dynamic_points() {
        std::vector<DynPoint> pts;
        if (!has_scan_||!has_map_||!has_pose_) return pts;
        const auto& scan=*last_scan_;
        const auto& info=map_msg_->info;
        double res=info.resolution,ox=info.origin.position.x,oy=info.origin.position.y;
        int w=(int)info.width,h=(int)info.height;
        double cyaw=std::cos(ego_yaw_),syaw=std::sin(ego_yaw_);

        int nbr_radius = std::max(1, (int)std::ceil(map_diff_thresh_ / res));

        for (size_t i=0;i<scan.ranges.size();++i) {
            float r=scan.ranges[i];
            if (!std::isfinite(r)||r<0.15||r>obs_range_) continue;
            double ang=scan.angle_min+i*scan.angle_increment;
            double lx=r*std::cos(ang), ly=r*std::sin(ang);
            double mx=ego_x_+cyaw*lx-syaw*ly, my=ego_y_+syaw*lx+cyaw*ly;
            int gx=(int)((mx-ox)/res), gy=(int)((my-oy)/res);
            if (gx<0||gx>=w||gy<0||gy>=h) continue;

            if (is_near_wall(gx, gy, w, h, nbr_radius)) continue;

            Projection proj=project_to_track(mx,my);
            double ds=signed_arc_diff(proj.s, ego_s_);
            if (ds>obs_range_||ds<-0.5||std::abs(proj.e_y)>2.0) continue;
            pts.push_back({mx,my,proj.s,proj.e_y});
        }
        return pts;
    }

    struct Cluster { double cx,cy,cs,cey; int count; };

    Cluster find_best_cluster(const std::vector<DynPoint>& pts) {
        std::vector<bool> used(pts.size(),false);
        Cluster best{0,0,0,0,0};
        for (size_t i=0;i<pts.size();++i) {
            if (used[i]) continue;
            double sx=pts[i].x,sy=pts[i].y,ss=pts[i].s,se=pts[i].ey; int cnt=1; used[i]=true;
            for (size_t j=i+1;j<pts.size();++j) {
                if (used[j]) continue;
                if (std::hypot(pts[j].x-pts[i].x,pts[j].y-pts[i].y)<obs_cluster_d_) {
                    sx+=pts[j].x; sy+=pts[j].y; ss+=pts[j].s; se+=pts[j].ey; cnt++; used[j]=true;
                }
            }
            if (cnt>=obs_min_pts_&&cnt>best.count) best={sx/cnt,sy/cnt,ss/cnt,se/cnt,cnt};
        }
        return best;
    }

    int select_passing_side(double obs_s, double obs_ey, double margin_total) {
        TrackPoint ref=interp_track(obs_s);
        double sl=(obs_ey-margin_total)-ref.ey_min;
        double sr=ref.ey_max-(obs_ey+margin_total);
        double min_gap=obs_hw_*2+0.05;
        bool lo=sl>min_gap, ro=sr>min_gap;
        if (!lo&&!ro) return 0;
        if (lo&&!ro) return -1;
        if (!lo&&ro) return +1;
        return (sl>=sr)?-1:+1;
    }

    struct CorridorBounds { double ey_min[N+1]; double ey_max[N+1]; bool active=false; };

    double signed_arc_diff(double s1, double s2) const {
        double d = wrap_s(s1) - wrap_s(s2);
        if (d > track_length_ * 0.5)  d -= track_length_;
        if (d < -track_length_ * 0.5) d += track_length_;
        return d;
    }

    CorridorBounds compute_corridor(const double s_horizon[], int pass_side, double margin_scale=1.0) {
        CorridorBounds cb; cb.active=false;
        double mt=(obs_margin_+obs_hw_)*margin_scale;
        double s_env=obs_hl_+0.4+current_v_*0.12;
        for (int k=0;k<=N;++k) {
            TrackPoint ref=interp_track(s_horizon[k]);
            cb.ey_min[k]=ref.ey_min; cb.ey_max[k]=ref.ey_max;
            if (!obs_kf_.active||pass_side==0) continue;
            double tk=k*DT;
            double os=obs_kf_.s()+obs_kf_.s_dot()*tk;
            double oe=obs_kf_.ey()+obs_kf_.ey_dot()*tk;
            double ds = signed_arc_diff(s_horizon[k], os);
            if (std::abs(ds)<s_env) {
                cb.active=true;
                if (pass_side==-1) {
                    cb.ey_max[k]=std::min(cb.ey_max[k], oe - mt);
                } else {
                    cb.ey_min[k]=std::max(cb.ey_min[k], oe + mt);
                }
                if (cb.ey_min[k] > cb.ey_max[k] - 0.02) {
                    cb.ey_min[k]=ref.ey_min;
                    cb.ey_max[k]=ref.ey_max;
                }
            }
        }
        return cb;
    }

    void apply_corridor(const CorridorBounds& cb) {
        for (int k=1;k<=N;++k) {
            double ey_lo = std::max(cb.ey_min[k], -EY_BOUND);
            double ey_hi = std::min(cb.ey_max[k],  EY_BOUND);
            double lbx[4] = {ey_lo, -EPSI_BOUND, 0.0,         -max_steering_angle_};
            double ubx[4] = {ey_hi,  EPSI_BOUND, max_speed_,   max_steering_angle_};
            ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,k,"lbx",lbx);
            ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,k,"ubx",ubx);
        }
    }

    void reset_corridor() {
        for (int k=1;k<=N;++k) {
            double lbx[4] = {-EY_BOUND, -EPSI_BOUND, 0.0,         -max_steering_angle_};
            double ubx[4] = { EY_BOUND,  EPSI_BOUND, max_speed_,   max_steering_angle_};
            ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,k,"lbx",lbx);
            ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,k,"ubx",ubx);
        }
    }

    // ═══════════════════ ADAPTIVE WEIGHTS ═══════════════════════════════════

    struct AdaptiveWeights {
        Eigen::Matrix<double, NY,   NY>   W;
        Eigen::Matrix<double, NY_E, NY_E> We;
    };

    AdaptiveWeights get_adaptive_weights(double kappa) const {
        AdaptiveWeights aw;
        aw.W  = Eigen::Matrix<double, NY,   NY>::Zero();
        aw.We = Eigen::Matrix<double, NY_E, NY_E>::Zero();

        double abs_k = std::abs(kappa);
        double blend = std::clamp((abs_k - 0.6) / (1.2 - 0.6), 0.0, 1.0);
        auto lerp = [&](double fast, double hairpin) {
            return fast + blend * (hairpin - fast);
        };

        aw.W(0,0) = lerp(get_parameter("w_ey").as_double(),       60.0);
        aw.W(1,1) = lerp(get_parameter("w_epsi").as_double(),     30.0);
        aw.W(2,2) = get_parameter("w_v").as_double();
        aw.W(3,3) = lerp(get_parameter("w_delta").as_double(),    20.0);
        aw.W(4,4) = lerp(get_parameter("w_delta_dot").as_double(), 30.0);
        aw.W(5,5) = lerp(get_parameter("w_a").as_double(),         0.5);

        aw.We(0,0) = lerp(get_parameter("w_ey_e").as_double(),    80.0);
        aw.We(1,1) = lerp(get_parameter("w_epsi_e").as_double(),  40.0);
        aw.We(2,2) = get_parameter("w_v_e").as_double();
        aw.We(3,3) = get_parameter("w_delta_e").as_double();

        return aw;
    }

    // ═══════════════════ MAIN POSE CALLBACK ═══════════════════════════════

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (track_.empty()||!capsule_) return;
        ego_x_=msg->pose.position.x; ego_y_=msg->pose.position.y;
        ego_yaw_=quat_to_yaw(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
        has_pose_=true;
        double cv=has_odom_?current_v_:0;

        Projection proj=project_to_track(ego_x_,ego_y_);
        ego_s_=proj.s;
        double e_psi=normalize_angle(ego_yaw_-proj.yaw_ref);
        double delta0=std::clamp(last_delta_cmd_,-max_steering_angle_,max_steering_angle_);
        double x0[NX]={proj.s,proj.e_y,e_psi,cv,delta0};
        ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,0,"lbx",x0);
        ocp_nlp_constraints_model_set(nlp_config_,nlp_dims_,nlp_in_,nlp_out_,0,"ubx",x0);

        bool use_prev=has_prev_solution_;
        if (use_prev) { double se=std::abs(wrap_s(proj.s)-wrap_s(prev_s_[1])); if(se>track_length_*0.5)se=track_length_-se; if(se>2)use_prev=false; }
        double s_horizon[N+1];
        if (use_prev) { s_horizon[0]=proj.s; for(int k=1;k<=N;++k) s_horizon[k]=prev_s_[std::min(k+1,N)]; }
        else {
            s_horizon[0] = proj.s;
            double sk = proj.s;
            for (int k = 1; k <= N; ++k) {
                TrackPoint r = interp_track(sk);
                sk += r.v_ref * DT;   // was: ((1-bl)*cv + bl*r.v_ref)*DT
                s_horizon[k] = sk;
            }
        }
// kappa + yref + adaptive weights
        for (int k=0;k<N;++k) {
            TrackPoint r=interp_track(s_horizon[k]);
            kappa_horizon_[k] = r.kappa;
            double p[NP]={r.kappa};
            f1tenth_frenet_acados_update_params(capsule_,k,p,NP);
            double yr[NY]={0,0,r.v_ref,0,0,0};
            ocp_nlp_cost_model_set(nlp_config_,nlp_dims_,nlp_in_,k,"yref",yr);
            auto aw = get_adaptive_weights(r.kappa);
            ocp_nlp_cost_model_set(nlp_config_,nlp_dims_,nlp_in_,k,"W",aw.W.data());
        }
        {
            TrackPoint re=interp_track(s_horizon[N]);
            kappa_horizon_[N] = re.kappa;
            double pe[NP]={re.kappa};
            f1tenth_frenet_acados_update_params(capsule_,N,pe,NP);
            double yre[NY_E]={0,0,re.v_ref,0};
            ocp_nlp_cost_model_set(nlp_config_,nlp_dims_,nlp_in_,N,"yref",yre);
            auto aw_e = get_adaptive_weights(re.kappa);
            ocp_nlp_cost_model_set(nlp_config_,nlp_dims_,nlp_in_,N,"W",aw_e.We.data());
        }

        // ── OBSTACLE AVOIDANCE ──
        int pass_side=0; bool obstacle_active=false;
        if (enable_obs_&&has_map_&&has_scan_) {
            int saved_seg = last_projection_seg_;
            auto dyn=detect_dynamic_points();
            last_projection_seg_ = saved_seg;
            Cluster cl=find_best_cluster(dyn);
            obs_kf_.predict(DT);
            if (cl.count>=obs_min_pts_) obs_kf_.update(cl.cs,cl.cey,track_length_); else obs_kf_.miss();

            if (obs_kf_.active) {
                obstacle_active=true;
                double mt=obs_margin_+obs_hw_;
                int best=select_passing_side(obs_kf_.s(),obs_kf_.ey(),mt);
                rclcpp::Time nt=now();
                double since=(committed_side_!=0)?(nt-side_commit_time_).seconds():1e9;

                if (best == 0) {
                    committed_side_ = 0;
                    pass_side = 0;
                } else if (committed_side_==0) {
                    committed_side_=best; side_commit_time_=nt;
                    pass_side = committed_side_;
                } else if (since>side_commit_t_) {
                    if (best!=committed_side_) {
                        TrackPoint ref=interp_track(obs_kf_.s());
                        double cs=(committed_side_==-1)?(obs_kf_.ey()-mt)-ref.ey_min:ref.ey_max-(obs_kf_.ey()+mt);
                        if (cs<obs_hw_*2) { committed_side_=best; side_commit_time_=nt; }
                    }
                    pass_side=committed_side_;
                } else {
                    pass_side=committed_side_;
                }
            } else { committed_side_=0; }
        }

        CorridorBounds corridor;
        if (obstacle_active&&pass_side!=0) {
            corridor=compute_corridor(s_horizon,pass_side,1.0);
            apply_corridor(corridor);

            for (int k=0;k<N;++k) {
                double corridor_center = 0.5 * (corridor.ey_min[k] + corridor.ey_max[k]);
                TrackPoint ref = interp_track(s_horizon[k]);
                bool is_constrained = (corridor.ey_max[k] < ref.ey_max - 0.01) ||
                                       (corridor.ey_min[k] > ref.ey_min + 0.01);
                if (is_constrained) {
                    double yr[NY] = {corridor_center, 0, ref.v_ref, 0, 0, 0};
                    ocp_nlp_cost_model_set(nlp_config_,nlp_dims_,nlp_in_,k,"yref",yr);
                }
            }
            {
                double corridor_center_e = 0.5 * (corridor.ey_min[N] + corridor.ey_max[N]);
                TrackPoint ref_e = interp_track(s_horizon[N]);
                bool is_constrained_e = (corridor.ey_max[N] < ref_e.ey_max - 0.01) ||
                                         (corridor.ey_min[N] > ref_e.ey_min + 0.01);
                if (is_constrained_e) {
                    double yre[NY_E] = {corridor_center_e, 0, ref_e.v_ref, 0};
                    ocp_nlp_cost_model_set(nlp_config_,nlp_dims_,nlp_in_,N,"yref",yre);
                }
            }
        } else {
            corridor.active=false;
            for(int k=0;k<=N;++k) { TrackPoint r=interp_track(s_horizon[k]); corridor.ey_min[k]=r.ey_min; corridor.ey_max[k]=r.ey_max; }
            if (track_widths_computed_) apply_corridor(corridor); else reset_corridor();
        }

        // ── SOLVE WITH FEASIBILITY FALLBACK + DIAGNOSTICS ──
        auto t0=std::chrono::high_resolution_clock::now();
        int status=0;
        for (int i=0;i<rti_iterations_;++i) status=f1tenth_frenet_acados_solve(capsule_);
        bool emergency_brake=false;

        // Log every non-zero status for diagnostics
        if (status != 0) {
            log_solver_failure(status, "primary", x0);
        }

        bool truly_infeasible = (status == 1 || status == 4);

        if (truly_infeasible) {
            if (obstacle_active) {
                RCLCPP_WARN(get_logger(),"Infeasible status=%d, relaxing margins...",status);
                auto relaxed=compute_corridor(s_horizon,pass_side,0.5);
                apply_corridor(relaxed);
                for (int i=0;i<rti_iterations_;++i) status=f1tenth_frenet_acados_solve(capsule_);
                if (status != 0) log_solver_failure(status, "relaxed", x0);
            }
            if (status==1||status==4) {
                RCLCPP_WARN(get_logger(),"Still infeasible, removing obstacle constraints + resetting warm-start");
                reset_corridor();
                double u_reset[NU] = {0.0, 0.0};
                for (int k=0;k<=N;++k) {
                    double alpha = (double)k / (double)N;
                    double x_k[NX] = {
                        x0[0] + alpha * (s_horizon[N] - x0[0]),
                        x0[1] * (1.0 - alpha),
                        x0[2] * (1.0 - alpha),
                        x0[3],
                        x0[4] * (1.0 - alpha)
                    };
                    ocp_nlp_out_set(nlp_config_,nlp_dims_,nlp_out_,nlp_in_,k,"x",x_k);
                }
                for (int k=0;k<N;++k)
                    ocp_nlp_out_set(nlp_config_,nlp_dims_,nlp_out_,nlp_in_,k,"u",u_reset);

                for (int i=0;i<rti_iterations_;++i) status=f1tenth_frenet_acados_solve(capsule_);
                if (status != 0) log_solver_failure(status, "after-reset", x0);
                if (status==1||status==4) {
                    RCLCPP_ERROR(get_logger(),"ALL INFEASIBLE after reset → EMERGENCY BRAKE");
                    emergency_brake=true;
                }
            }
        }

        if (status == 3) {
            minstep_count_++;
            if (minstep_count_ % 50 == 1) {
                RCLCPP_WARN(get_logger(), "MINSTEP (status=3) count=%d — solver warm-start may be stale", minstep_count_);
            }
            if (minstep_count_ > 12) {
                RCLCPP_WARN(get_logger(), "Persistent MINSTEP — resetting solver warm-start");
                double u_reset[NU] = {0.0, 0.0};
                for (int k=0;k<=N;++k) {
                    double alpha = (double)k / (double)N;
                    double x_k[NX] = {
                        x0[0] + alpha * (s_horizon[N] - x0[0]),
                        x0[1] * (1.0 - alpha),
                        x0[2] * (1.0 - alpha),
                        x0[3],
                        x0[4] * (1.0 - alpha)
                    };
                    ocp_nlp_out_set(nlp_config_,nlp_dims_,nlp_out_,nlp_in_,k,"x",x_k);
                }
                for (int k=0;k<N;++k)
                    ocp_nlp_out_set(nlp_config_,nlp_dims_,nlp_out_,nlp_in_,k,"u",u_reset);
                for (int i=0;i<rti_iterations_;++i) status=f1tenth_frenet_acados_solve(capsule_);
                minstep_count_ = 0;
            }
        } else {
            minstep_count_ = 0;
        }

        // Reset failure counter on a clean solve so the rate-limiter resets
        if (status == 0) solve_failure_count_ = 0;

        double solve_ms=std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t0).count();
        std_msgs::msg::Float64 st; st.data=solve_ms; solve_time_pub_->publish(st);

        for (int k=0;k<=N;++k) { double xk[NX]; ocp_nlp_out_get(nlp_config_,nlp_dims_,nlp_out_,k,"x",xk); prev_s_[k]=xk[0]; }
        has_prev_solution_=true;

        double delta_cmd,speed_cmd;
        if (emergency_brake) { delta_cmd=0; speed_cmd=emergency_brake_speed_; }
        else {
            double u[NU]; ocp_nlp_out_get(nlp_config_,nlp_dims_,nlp_out_,0,"u",u);
            double x1[NX]; ocp_nlp_out_get(nlp_config_,nlp_dims_,nlp_out_,1,"x",x1);
            delta_cmd=std::clamp(x1[4],-max_steering_angle_,max_steering_angle_);
            double ms=max_steering_rate_*DT;
            delta_cmd=std::clamp(delta_cmd,last_delta_cmd_-ms,last_delta_cmd_+ms);
            delta_cmd=std::clamp(delta_cmd,-max_steering_angle_,max_steering_angle_);
            speed_cmd=std::clamp(x1[3],min_speed_cmd_,max_speed_);
        }
        last_delta_cmd_=delta_cmd;

        ackermann_msgs::msg::AckermannDriveStamped drive;
        drive.header.stamp=now(); drive.header.frame_id="base_link";
        drive.drive.steering_angle=delta_cmd; drive.drive.speed=speed_cmd;
        drive_pub_->publish(drive);

        std_msgs::msg::Float64MultiArray sm;
        sm.data={proj.s,proj.e_y,e_psi,cv,proj.kappa,delta0,0.0,delta_cmd,speed_cmd,solve_ms,(double)pass_side,
                 obs_kf_.active?obs_kf_.s():-1.0, obs_kf_.active?obs_kf_.ey():0.0};
        state_pub_->publish(sm);

        publish_prediction(); publish_reference(proj.s,cv);
        publish_frenet_frame(ego_x_,ego_y_,ego_yaw_,proj,e_psi);
        if (enable_obs_) publish_obstacle_viz(s_horizon,corridor,pass_side);
    }

    // ═══════════════════ TRACK HELPERS ═════════════════════════════════════

    double wrap_s(double s) const { if(track_length_<=0)return 0; double o=std::fmod(s,track_length_); if(o<0)o+=track_length_; return o; }
    double arc_diff(double to,double fr) const { double d=to-fr; if(d<0)d+=track_length_; return d; }

    TrackPoint interp_track(double s) const {
        double sm=wrap_s(s);
        auto comp=[](double v,const TrackPoint& p){return v<p.s;};
        auto it=std::upper_bound(track_.begin(),track_.end(),sm,comp);
        int i=(int)std::distance(track_.begin(),it)-1; if(i<0)i=0; if(i>=(int)track_.size())i=(int)track_.size()-1;
        int j=(i+1)%(int)track_.size();
        double s0=track_[i].s, s1=(j==0)?track_length_:track_[j].s;
        double a=(sm-s0)/std::max(1e-9,s1-s0);
        TrackPoint o; o.s=sm;
        o.x=track_[i].x+a*(track_[j].x-track_[i].x);
        o.y=track_[i].y+a*(track_[j].y-track_[i].y);
        o.yaw=normalize_angle(track_[i].yaw+a*normalize_angle(track_[j].yaw-track_[i].yaw));
        o.kappa=track_[i].kappa+a*(track_[j].kappa-track_[i].kappa);
        o.v_ref=track_[i].v_ref+a*(track_[j].v_ref-track_[i].v_ref);
        o.ey_min=track_[i].ey_min+a*(track_[j].ey_min-track_[i].ey_min);
        o.ey_max=track_[i].ey_max+a*(track_[j].ey_max-track_[i].ey_max);
        return o;
    }

    Projection project_to_track(double x, double y) {
        const int n=(int)track_.size(); double best_d2=1e18; Projection best;
        const int window=80;
        bool did_local=(last_projection_seg_>=0&&last_projection_seg_<n);
        int start=did_local?-window:0, end=did_local?window:n-1;
        auto search=[&](int a,int b) {
            for (int off=a;off<=b;++off) {
                int i=did_local?((last_projection_seg_+off+n)%n):off;
                int j2=(i+1)%n;
                double vx=track_[j2].x-track_[i].x, vy=track_[j2].y-track_[i].y;
                double wx=x-track_[i].x, wy=y-track_[i].y;
                double len2=vx*vx+vy*vy; if(len2<1e-12) continue;
                double t=std::clamp((wx*vx+wy*vy)/len2,0.0,1.0);
                double px=track_[i].x+t*vx, py=track_[i].y+t*vy;
                double dx=x-px,dy=y-py,d2=dx*dx+dy*dy;
                if (d2<best_d2) {
                    best_d2=d2; double sl=std::sqrt(len2); double sp=track_[i].s+t*sl;
                    if(sp>=track_length_) sp-=track_length_;
                    double yr=std::atan2(vy,vx), nx2=-std::sin(yr), ny2=std::cos(yr);
                    best.s=sp; best.e_y=dx*nx2+dy*ny2; best.yaw_ref=normalize_angle(yr);
                    best.kappa=interp_track(sp).kappa; best.x_ref=px; best.y_ref=py; best.seg_idx=i;
                }
            }
        };
        search(start,end);
        if (std::sqrt(best_d2)>max_search_dist_warn_) { did_local=false; best_d2=1e18; search(0,n-1); }
        last_projection_seg_=best.seg_idx;
        return best;
    }

    // ═══════════════════ VISUALIZATION ═════════════════════════════════════

    static geometry_msgs::msg::Point mp(double x,double y,double z) { geometry_msgs::msg::Point p; p.x=x;p.y=y;p.z=z; return p; }

    void publish_obstacle_viz(const double sh[], const CorridorBounds& cb, int ps) {
        visualization_msgs::msg::MarkerArray ma;
        if (obs_kf_.active) {
            TrackPoint r=interp_track(obs_kf_.s());
            double gx=r.x-std::sin(r.yaw)*obs_kf_.ey(), gy=r.y+std::cos(r.yaw)*obs_kf_.ey();
            visualization_msgs::msg::Marker m; m.header.frame_id=global_frame_; m.header.stamp=now();
            m.ns="obstacle"; m.id=0; m.type=visualization_msgs::msg::Marker::CUBE; m.action=visualization_msgs::msg::Marker::ADD;
            m.pose.position.x=gx; m.pose.position.y=gy; m.pose.position.z=0.15;
            m.pose.orientation.z=std::sin(r.yaw/2); m.pose.orientation.w=std::cos(r.yaw/2);
            m.scale.x=obs_hl_*2; m.scale.y=obs_hw_*2; m.scale.z=0.2;
            m.color.r=1; m.color.a=0.8; ma.markers.push_back(m);
            visualization_msgs::msg::Marker t; t.header=m.header; t.ns="obstacle"; t.id=1;
            t.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING; t.action=visualization_msgs::msg::Marker::ADD;
            t.pose.position.x=gx; t.pose.position.y=gy; t.pose.position.z=0.5; t.scale.z=0.2;
            t.color.r=t.color.g=t.color.b=t.color.a=1;
            t.text=(ps==-1)?"PASS L":(ps==1)?"PASS R":"BRAKE"; ma.markers.push_back(t);
        } else {
            visualization_msgs::msg::Marker d; d.header.frame_id=global_frame_; d.header.stamp=now();
            d.ns="obstacle"; d.id=0; d.action=visualization_msgs::msg::Marker::DELETE; ma.markers.push_back(d);
            d.id=1; ma.markers.push_back(d);
        }
        obs_pub_->publish(ma);

        visualization_msgs::msg::MarkerArray cma;
        for (int side=0;side<2;++side) {
            visualization_msgs::msg::Marker l; l.header.frame_id=global_frame_; l.header.stamp=now();
            l.ns="corridor"; l.id=side; l.type=visualization_msgs::msg::Marker::LINE_STRIP; l.action=visualization_msgs::msg::Marker::ADD;
            l.scale.x=0.03; if(side==0){l.color.g=1;l.color.a=0.7;}else{l.color.r=1;l.color.a=0.7;}
            for (int k=0;k<=N;++k) {
                TrackPoint r=interp_track(sh[k]); double ev=(side==0)?cb.ey_min[k]:cb.ey_max[k];
                l.points.push_back(mp(r.x-std::sin(r.yaw)*ev, r.y+std::cos(r.yaw)*ev, 0.05));
            }
            cma.markers.push_back(l);
        }
        corridor_pub_->publish(cma);
    }

    void publish_frenet_frame(double cx,double cy,double cw,const Projection& proj,double ep) {
        visualization_msgs::msg::MarkerArray ma; const auto st=now(); const double z=0.16;
        auto sphere=[&](int id,double x,double y,double sc,float r,float g,float b){
            visualization_msgs::msg::Marker m; m.header.frame_id=global_frame_; m.header.stamp=st;
            m.ns="frenet_frame"; m.id=id; m.type=visualization_msgs::msg::Marker::SPHERE; m.action=visualization_msgs::msg::Marker::ADD;
            m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=z; m.scale.x=m.scale.y=m.scale.z=sc;
            m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=1; ma.markers.push_back(m);};
        auto arrow=[&](int id,double x0,double y0,double x1,double y1,double sz,float r,float g,float b){
            visualization_msgs::msg::Marker m; m.header.frame_id=global_frame_; m.header.stamp=st;
            m.ns="frenet_frame"; m.id=id; m.type=visualization_msgs::msg::Marker::ARROW; m.action=visualization_msgs::msg::Marker::ADD;
            m.points.push_back(mp(x0,y0,z)); m.points.push_back(mp(x1,y1,z));
            m.scale.x=sz;m.scale.y=sz*2;m.scale.z=sz*3; m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=1; ma.markers.push_back(m);};
        sphere(0,proj.x_ref,proj.y_ref,0.16,1,1,0);
        arrow(1,proj.x_ref,proj.y_ref,proj.x_ref+0.7*std::cos(proj.yaw_ref),proj.y_ref+0.7*std::sin(proj.yaw_ref),0.05,0,1,0);
        arrow(2,proj.x_ref,proj.y_ref,proj.x_ref+0.45*(-std::sin(proj.yaw_ref)),proj.y_ref+0.45*std::cos(proj.yaw_ref),0.04,1,0,0);
        visualization_msgs::msg::Marker el; el.header.frame_id=global_frame_; el.header.stamp=st;
        el.ns="frenet_frame"; el.id=3; el.type=visualization_msgs::msg::Marker::LINE_STRIP; el.action=visualization_msgs::msg::Marker::ADD;
        el.points.push_back(mp(proj.x_ref,proj.y_ref,z+0.02)); el.points.push_back(mp(cx,cy,z+0.02));
        el.scale.x=0.045; el.color.r=el.color.g=el.color.b=el.color.a=1; ma.markers.push_back(el);
        arrow(4,cx,cy,cx+0.55*std::cos(cw),cy+0.55*std::sin(cw),0.04,0.2,0.6,1);
        visualization_msgs::msg::Marker txt; txt.header.frame_id=global_frame_; txt.header.stamp=st;
        txt.ns="frenet_frame"; txt.id=5; txt.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING; txt.action=visualization_msgs::msg::Marker::ADD;
        txt.pose.position.x=cx; txt.pose.position.y=cy; txt.pose.position.z=z+0.45; txt.scale.z=0.22;
        txt.color.r=txt.color.g=txt.color.b=txt.color.a=1;
        std::ostringstream ss; ss.setf(std::ios::fixed); ss.precision(2);
        ss<<"s="<<proj.s<<" ey="<<proj.e_y<<" ep="<<ep<<" k="<<proj.kappa;
        if(obs_kf_.active) ss<<" OBS"; txt.text=ss.str(); ma.markers.push_back(txt);
        frame_pub_->publish(ma);
    }

    void publish_prediction() {
        visualization_msgs::msg::MarkerArray ma;
        for (int k=0;k<=N;++k) { double xf[NX]; ocp_nlp_out_get(nlp_config_,nlp_dims_,nlp_out_,k,"x",xf);
            TrackPoint r=interp_track(xf[0]); double gx=r.x-std::sin(r.yaw)*xf[1], gy=r.y+std::cos(r.yaw)*xf[1];
            visualization_msgs::msg::Marker m; m.header.frame_id=global_frame_; m.header.stamp=now();
            m.ns="frenet_pred"; m.id=k; m.type=visualization_msgs::msg::Marker::SPHERE; m.action=visualization_msgs::msg::Marker::ADD;
            m.pose.position.x=gx;m.pose.position.y=gy;m.pose.position.z=0.08; m.scale.x=m.scale.y=m.scale.z=0.1;
            m.color.r=0.1;m.color.g=0.8;m.color.b=1;m.color.a=0.9; ma.markers.push_back(m); }
        traj_pub_->publish(ma);
    }

    void publish_reference(double s0,double cv) {
        visualization_msgs::msg::MarkerArray ma;
        for (int k=0;k<=N;++k) { double xk[NX]; ocp_nlp_out_get(nlp_config_,nlp_dims_,nlp_out_,k,"x",xk);
            TrackPoint r=interp_track(xk[0]);
            visualization_msgs::msg::Marker m; m.header.frame_id=global_frame_; m.header.stamp=now();
            m.ns="frenet_ref"; m.id=k; m.type=visualization_msgs::msg::Marker::SPHERE; m.action=visualization_msgs::msg::Marker::ADD;
            m.pose.position.x=r.x;m.pose.position.y=r.y;m.pose.position.z=0.04; m.scale.x=m.scale.y=m.scale.z=0.07;
            m.color.r=1;m.color.g=0.6;m.color.b=0.1;m.color.a=0.8; ma.markers.push_back(m); }
        ref_pub_->publish(ma);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrenetMPC>());
    rclcpp::shutdown();
    return 0;
}