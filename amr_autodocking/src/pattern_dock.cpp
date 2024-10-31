#include "pattern_dock.h"

PatternDock::PatternDock(ros::NodeHandle& nh, ros::NodeHandle& nh_local):
    nh_(nh),
    nh_local_(nh_local),
    debug(false)
{
    // Parameters:
    loadParameters();
    pattern_distance_bc = 2*patterm_length_b*std::sin(pattern_angle_bc/2);

    // Subscribes:
    sub_line_segments_ = nh_.subscribe("/front_line_segments", 1, &PatternDock::lineSegmentCB, this);

    // Publishers:
    if (debug){
        pub_dock_pose_ = nh_.advertise<geometry_msgs::Pose2D>("charger_frame", 5);
    }
}

PatternDock::~PatternDock()
{
}

void PatternDock::loadParameters() {
    printf("*************************************\n");
    printf("PARAMETERS:\n");

    // Parameters used by this node
    nh_local_.param<double>("patterm_length_a", patterm_length_a, 0.15);
    printf("patterm_length_a: %f\n", patterm_length_a);

    nh_local_.param<double>("patterm_length_b", patterm_length_b, 0.15);
    printf("patterm_length_b: %f\n", patterm_length_b);

    nh_local_.param<double>("pattern_angle_ab", pattern_angle_ab, 3.84);
    printf("pattern_angle_ab: %f\n", pattern_angle_ab);

    nh_local_.param<double>("pattern_angle_bc", pattern_angle_bc, 1.745);
    printf("pattern_angle_bc: %f\n", pattern_angle_bc);

    nh_local_.param<double>("pattern_angle_cd", pattern_angle_cd, 3.84);
    printf("pattern_angle_cd: %f\n", pattern_angle_cd);

    nh_local_.param<double>("pattern_angle_ad", pattern_angle_ad, M_PI);
    printf("pattern_angle_ad: %f\n", pattern_angle_ad);

    nh_local_.param<double>("distance_tolerance", distance_tolerance, 0.1);
    printf("distance_tolerance: %f\n", distance_tolerance);

    nh_local_.param<double>("length_tolerance", length_tolerance, 0.1);
    printf("length_tolerance: %f\n", length_tolerance);

    nh_local_.param<double>("angle_tolerance", angle_tolerance, 0.15);
    printf("angle_tolerance: %f\n", angle_tolerance);

    nh_local_.param<double>("angle_threshold", angle_threshold, 0.5);
    printf("angle_threshold: %f\n", angle_threshold);

    nh_local_.param<std::string>("laser_frame_id", laser_frame_id, "front_laser_link");
    printf("laser_frame_id: %s\n", laser_frame_id.c_str());

    nh_local_.param<std::string>("dock_frame_id", dock_frame_id, "dock_frame");
    printf("dock_frame_id: %s\n", dock_frame_id.c_str());

    nh_local_.param<bool>("debug", debug, false);
    printf("debug: %s\n", debug ? "True" : "False");

    printf("*************************************\n");
}

double PatternDock::calcVectorLength(const boost::array<float, 2UL>& start_point,
                                       const boost::array<float, 2UL>& end_point) 
{
    return std::sqrt(std::pow(start_point[0] - end_point[0], 2) + std::pow(start_point[1] - end_point[1], 2));
}

// bool PatternDock::checkAngle(double a, double b, double angle_pattern_ab, double angle_tolerance) {
//     double angle;
//     if (a * b > 0) {
//         angle = std::abs(a - b);
//     } else {
//         angle = 2 * M_PI - std::abs(a - b);
//         if (std::abs(angle - (2 * M_PI - angle_pattern_ab)) <= angle_tolerance) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     if (std::abs(angle_pattern_ab - angle) <= angle_tolerance) {
//         return true;
//     } else {
//         return false;
//     }
// }

bool PatternDock::checkAngle(double a, double b, double angle_pattern_ab, double angle_tolerance){
    double angle;
    angle = fabs(a-b);
    if (angle < M_PI and angle > 1.7 ){
        angle = angle - M_PI_2 ;
    }
    else if (angle> (M_PI_2 + M_PI)){ 
        angle = angle - M_PI - M_PI_2;
    }
    else if (angle> M_PI and angle < (M_PI_2 + M_PI)){
        angle = angle - M_PI ;
    }
    // if (debug) printf("angle: %f , angle_pattern: %f \n",angle,angle_pattern_ab);
    if (fabs(angle_pattern_ab-angle)<=angle_tolerance){
        return true;}
    else return false;
}

bool PatternDock::checkAngle(double angle_1, double angle_2, double angle_threshold)
{
    return (abs(angle_1) - abs(angle_2) <= angle_threshold);
}

bool PatternDock::checkDistance(const boost::array<float, 2UL>& start_point,
                                  const boost::array<float, 2UL>& end_point,
                                  double pattern_distance, double distance_tolerance) {
    double distance = 0;
    distance = calcVectorLength(start_point, end_point);
    // if (debug) printf("distance: %f , pattern_distance: %f \n",distance,pattern_distance);
    return std::abs(distance - pattern_distance) <= distance_tolerance;
}

std::vector<double> PatternDock::midPoint(const laser_line_extraction::LineSegment& vector) {
    std::vector<double> midPoint(2);
    midPoint[0] = (vector.start[0] + vector.end[0]) / 2;
    midPoint[1] = (vector.start[1] + vector.end[1]) / 2;
    return midPoint;
}

std::vector<double> PatternDock::midTwoPoints(const std::vector<double>& point1, const std::vector<double>& point2) {
    std::vector<double> midPoint(2);
    midPoint[0] = (point1[0] + point2[0]) / 2;
    midPoint[1] = (point1[1] + point2[1]) / 2;
    return midPoint;
}

void PatternDock::transformTFDock(double x, double y, double theta, const std::string& child_frame_id, const std::string& frame_id) {
    // publish dock_frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,0));
    tf::Quaternion q;
    q.setRPY(0,0,theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),frame_id,child_frame_id));
}

void PatternDock::lineSegmentCB(const laser_line_extraction::LineSegmentList::ConstPtr& msg) {
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> line_segments = msg->line_segments;
    laser_line_extraction::LineSegment vector_a, vector_b, vector_c, vector_d;
    const size_t line_num = line_segments.size();
    bool find_bc = false;
    bool find_a = false;
    bool find_d = false;
    bool find_success = false;

    if (line_num < 3) {
        if (debug){
            ROS_WARN("There isn't enough line in the laser field!");
        }
        return;
    }

    // std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> line_segments_ad;
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> line_segments_b;
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> line_segments_c;
    // Find vector b, c
    for (size_t i = 0; i < line_num - 1; ++i) {
        for (size_t j = i + 1; j < line_num; ++j) {
            if (checkAngle(line_segments[i].angle, line_segments[j].angle,
                           M_PI - pattern_angle_bc, angle_tolerance)) {

                // Check distance bc
                if (checkDistance(line_segments[i].start, line_segments[j].end,
                                    pattern_distance_bc, distance_tolerance) &&
                    checkDistance(line_segments[j].start, line_segments[i].end,
                                    0.0, distance_tolerance)) {
                    line_segments_b.push_back(line_segments[i]);
                    line_segments_c.push_back(line_segments[j]);
                    find_bc = true;
                    // printf("Vector i: angle:%f\n", line_segments[i].angle);
                    // printf("Vector j: angle:%f\n", line_segments[j].angle);
                    // printf("----\n");
                } else if (checkDistance(line_segments[j].start, line_segments[i].end,
                                            pattern_distance_bc, distance_tolerance) &&
                            checkDistance(line_segments[i].start, line_segments[j].end,
                                            0.0, distance_tolerance)) {
                    line_segments_b.push_back(line_segments[j]);
                    line_segments_c.push_back(line_segments[i]);
                    find_bc = true;
                    // printf("Vector i: angle:%f\n", line_segments[i].angle);
                    // printf("Vector j: angle:%f\n", line_segments[j].angle);
                    // printf("----\n");
                }
            }
        }
    }

    // Find vector a, d
    if (find_bc) {
        // const size_t line_ad_num = line_segments_ad.size();
        const size_t line_b_num = line_segments_b.size();
        if (debug) {
            printf("*************************************************\n");
            printf("Found vector b,c: %ld vectors\n", line_b_num);
            // printf("Found vector a,d: %ld vectors\n", line_ad_num);
        }
        for (size_t i = 0; i < line_b_num; ++i) {
            for (size_t j = 0; j < line_num; ++j) {
                if (checkAngle(line_segments[j].angle, line_segments_b[i].angle,
                            pattern_angle_ab - M_PI, angle_tolerance)) {
                    // Check distance ab
                    if (checkDistance(line_segments[j].end, line_segments_b[i].start,
                                    0.0, distance_tolerance)) {
                        // Check length a
                        if (checkDistance(line_segments[j].start, line_segments[j].end,
                                        patterm_length_a, length_tolerance)) {
                            vector_a = line_segments[j];
                            vector_b = line_segments_b[i];
                            find_a = true;
                            if (debug) {
                                printf("Found vector_a: start[%f:%f], end[%f:%f], angle: %f\n",
                                        vector_a.start[0], vector_a.start[1], vector_a.end[0], vector_a.end[1],vector_a.angle);
                                printf("Found vector_b: start[%f:%f], end[%f:%f], angle: %f\n",
                                        vector_b.start[0], vector_b.start[1], vector_b.end[0], vector_b.end[1],vector_b.angle); 
                            }
                        }
                    }
                }

                if (checkAngle(line_segments_c[i].angle, line_segments[j].angle,
                            pattern_angle_cd - M_PI, angle_tolerance)) {
                    // Check distance cd
                    if (checkDistance(line_segments_c[i].end, line_segments[j].start,
                                    0.0, distance_tolerance)) {
                        // Check length d
                        if (checkDistance(line_segments[j].start, line_segments[j].end,
                                        patterm_length_a, length_tolerance)) {
                            vector_d = line_segments[j];
                            vector_c = line_segments_c[i];
                            find_d = true;
                            if (debug) {
                                printf("Found vector_c: start[%f:%f], end[%f:%f], angle: %f\n",
                                        vector_c.start[0], vector_c.start[1], vector_c.end[0], vector_c.end[1],vector_c.angle);
                                printf("Found vector_d: start[%f:%f], end[%f:%f], angle: %f\n",
                                        vector_d.start[0], vector_d.start[1], vector_d.end[0], vector_d.end[1],vector_d.angle);
                            }
                        }
                    }
                }

                if ((find_a && vector_d.radius != 0.0) ||
                    (find_d && vector_a.radius != 0.0)) {
                    // Check angle both vectors
                    if (checkAngle(vector_a.angle, vector_d.angle,
                        M_PI - pattern_angle_ad, angle_tolerance)) {
                        // Check distance ad
                        if (checkDistance(vector_a.start, vector_d.end,
                                        0.15*2 + pattern_distance_bc, distance_tolerance) &&
                            checkDistance(vector_d.start, vector_a.end,
                                        pattern_distance_bc, distance_tolerance)) {
                            find_success = true;
                            if (debug){
                                printf("FOUND %s SUCCESS!!!!!!!!!!! \n", dock_frame_id.c_str());
                            }
                            break;
                        } else if (checkDistance(vector_d.start, vector_a.end,
                                                0.15*2 + pattern_distance_bc, distance_tolerance) &&
                                    checkDistance(vector_a.start, vector_d.end,
                                                pattern_distance_bc, distance_tolerance)) {
                            find_success = true;
                            if (debug){
                                printf("FOUND %s SUCCESS!!!!!!!!!!! \n", dock_frame_id.c_str());
                            }
                            break;
                        } else {
                            find_a = false;
                            find_d = false;
                        }
                    } else {
                        find_a = false;
                        find_d = false;
                    }
                }
            }
            if (find_success) {
                break;
            }
        }
    }

    if (find_bc) {
        double x, y, theta;
        if (find_success) {
            std::vector<double> point_temp_a = midPoint(vector_a);
            std::vector<double> point_temp_b = midPoint(vector_b);
            std::vector<double> point_temp_c = midPoint(vector_c);
            std::vector<double> point_temp_d = midPoint(vector_d);

            std::vector<double> theta_point1 = midTwoPoints(point_temp_a, point_temp_b);
            std::vector<double> theta_point2 = midTwoPoints(point_temp_c, point_temp_d);
            x = (theta_point1[0] + theta_point2[0]) / 2;
            y = (theta_point1[1] + theta_point2[1]) / 2;
            theta = std::atan2(theta_point1[1] - theta_point2[1], theta_point1[0] - theta_point2[0]) - M_PI / 2;
        } else {
            // return;
            const size_t line_b_num = line_segments_b.size();
            if (vector_b.radius != 0.0 && vector_c.radius != 0.0) return;

            if (vector_b.radius != 0.0) {
                for (size_t i = 0; i < line_b_num; ++i) {
                    if (vector_b.radius == line_segments_b[i].radius) {
                        vector_c = line_segments_c[i];
                        if (debug){
                            printf("FOUND %s SUCCESS-(a-b-c)!!!!!!!!!!! \n", dock_frame_id.c_str());
                        }
                    }
                }
            } else if (vector_c.radius != 0.0) {
                for (size_t i = 0; i < line_b_num; ++i) {
                    if (vector_c.radius == line_segments_c[i].radius) {
                        vector_b = line_segments_b[i];
                        if (debug){
                            printf("FOUND %s SUCCESS-(b-c-d)!!!!!!!!!!! \n", dock_frame_id.c_str());
                        }              
                    }
                }  
            } else return;
            std::vector<double> point_temp_b = midPoint(vector_b);
            std::vector<double> point_temp_c = midPoint(vector_c);
            x = (point_temp_b[0] + point_temp_c[0]) / 2;
            y = (point_temp_b[1] + point_temp_c[1]) / 2;
            theta = std::atan2(vector_b.start[1] - vector_c.end[1], vector_b.start[0] - vector_c.end[0]) - M_PI / 2;
        }

        geometry_msgs::Pose2D dock_pose;
        dock_pose.x = x;
        dock_pose.y = y;
        dock_pose.theta = theta;
        if (debug) pub_dock_pose_.publish(dock_pose);
        
        transformTFDock(x, y, theta, dock_frame_id, laser_frame_id);
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "find_charger_tf");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    PatternDock find_charger_tf(nh, nh_local);
    ros::spin();
    return 0;
}
