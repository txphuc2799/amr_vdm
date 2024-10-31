#include "pattern_special_dock.h"

PatternSpecialDock::PatternSpecialDock(ros::NodeHandle& nh, ros::NodeHandle& nh_local):
    nh_(nh),
    nh_local_(nh_local),
    debug(false)
{
    // Parameters:
    loadParameters();

    // Subscribes:
    sub_line_segments_ = nh_.subscribe("/back_line_segments", 1, &PatternSpecialDock::lineSegmentCB, this);

    // Publishers:
    if (debug){
        pub_dock_pose_ = nh_.advertise<geometry_msgs::Pose2D>("first_frame", 5);
    }
}

PatternSpecialDock::~PatternSpecialDock()
{
}

void PatternSpecialDock::loadParameters() {
    printf("*************************************\n");
    printf("PARAMETERS:\n");

    // Parameters used by this node
    nh_local_.param<double>("pattern_length_a", pattern_length_a, 0.15);
    printf("pattern_length_a: %f\n", pattern_length_a);

    nh_local_.param<double>("pattern_length_b", pattern_length_b, 0.15);
    printf("pattern_length_b: %f\n", pattern_length_b);

    nh_local_.param<double>("pattern_angle_ab", pattern_angle_ab, 3.84);
    printf("pattern_angle_ab: %f\n", pattern_angle_ab);

    nh_local_.param<double>("pattern_angle_bc", pattern_angle_bc, 1.745);
    printf("pattern_angle_bc: %f\n", pattern_angle_bc);

    nh_local_.param<double>("pattern_angle_cd", pattern_angle_cd, 3.84);
    printf("pattern_angle_cd: %f\n", pattern_angle_cd);

    nh_local_.param<double>("pattern_angle_ad", pattern_angle_ad, M_PI);
    printf("pattern_angle_ad: %f\n", pattern_angle_ad);

    nh_local_.param<double>("pattern_distance_ad", pattern_distance_ad, 0.4874);
    printf("pattern_distance_ad: %f\n", pattern_distance_ad);

    nh_local_.param<double>("pattern_distance_bc", pattern_distance_bc, 0.4874);
    printf("pattern_distance_bc: %f\n", pattern_distance_bc);

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

double PatternSpecialDock::calcVectorLength(const boost::array<float, 2UL>& start_point,
                                       const boost::array<float, 2UL>& end_point) 
{
    return std::sqrt(std::pow(start_point[0] - end_point[0], 2) + std::pow(start_point[1] - end_point[1], 2));
}

// bool checkAngle(double angle_a, double angle_b, double angle_pattern_ab, double angle_tolerance) {
//     double angle;
//     if (angle_a * angle_b > 0) {
//         angle = std::abs(angle_a - angle_b);
//     } else {
//         angle = 2 * M_PI - std::abs(angle_a - angle_b);
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

bool PatternSpecialDock::checkAngle(double a, double b, double angle_pattern_ab, double angle_tolerance){
    double angle;
    angle = fabs(a-b);
    // if (angle < 3.1 and angle > 1.7 ){
    //     angle = angle - M_PI_2 ;
    // }
    // else if (angle> (M_PI_2 + M_PI)){
    //     angle = angle - M_PI - M_PI_2 ;
    // }
    // else if (angle >= 3.1 and angle < (M_PI_2 + M_PI)){
    //     angle = angle - M_PI ;
    // }
    if (angle > 3.0) {
        angle = angle - M_PI;
    } 
    // if (debug) printf("angle: %f , angle_pattern: %f \n",angle,angle_pattern_ab);
    if (fabs(angle_pattern_ab-angle)<=angle_tolerance){
        return true;}
    else return false;
}

bool PatternSpecialDock::checkAngle(double angle_1, double angle_2, double angle_threshold)
{
    return (abs(angle_1) - abs(angle_2) <= angle_threshold);
}

bool PatternSpecialDock::checkDistance(const boost::array<float, 2UL>& start_point,
                                  const boost::array<float, 2UL>& end_point,
                                  double pattern_distance, double distance_tolerance) {
    double distance = 0;
    distance = calcVectorLength(start_point, end_point);
    // if (debug) printf("distance: %f , pattern_distance: %f \n",distance,pattern_distance);
    return std::abs(distance - pattern_distance) <= distance_tolerance;
}

const boost::array<float, 2UL> PatternSpecialDock::midPoint(const boost::array<float, 2UL>& start_point,
                                                            const boost::array<float, 2UL>& end_point) {
    boost::array<float, 2UL> midPoint;
    midPoint[0] = (start_point[0] + end_point[0]) / 2;
    midPoint[1] = (start_point[1] + end_point[1]) / 2;
    return midPoint;
}

// std::vector<double> PatternSpecialDock::midTwoPoints(const std::vector<double>& point1, const std::vector<double>& point2) {
//     std::vector<double> midPoint(2);
//     midPoint[0] = (point1[0] + point2[0]) / 2;
//     midPoint[1] = (point1[1] + point2[1]) / 2;
//     return midPoint;
// }

void PatternSpecialDock::transformTFDock(double x, double y, double theta, const std::string& child_frame_id, const std::string& frame_id) {
    // publish dock_frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,0));
    tf::Quaternion q;
    q.setRPY(0,0,theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),frame_id,child_frame_id));
}

void PatternSpecialDock::lineSegmentCB(const laser_line_extraction::LineSegmentList::ConstPtr& msg) {
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> line_segments = msg->line_segments;
    laser_line_extraction::LineSegment vector_a, vector_b, vector_c, vector_d;
    const size_t line_num = line_segments.size();
    bool find_ad = false;
    bool find_b = false;
    bool find_c = false;
    bool find_success = false;
    // printf("ONE SUBCRIBE: ////////////////////////////////////////////////////\n");
    if (line_num < 2) {
        if (debug){
            ROS_WARN("There isn't enough line in the laser field!");
        }
        return;
    }

    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> line_segments_a;
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> line_segments_d;
    // Find vector a, d
    for (size_t i = 0; i < line_num - 1; ++i) {
        for (size_t j = i + 1; j < line_num; ++j) {
            if (checkAngle(line_segments[i].angle, line_segments[j].angle,
                           pattern_angle_ad, angle_tolerance)) {
                // printf("Detect two line match pattern_angle_ad: %f\n", pattern_angle_ad);

                // Check length vector a,d
                if (checkDistance(line_segments[i].start, line_segments[i].end,
                                  pattern_length_a, length_tolerance) &&
                    checkDistance(line_segments[j].start, line_segments[j].end,
                                  pattern_length_a, length_tolerance)) {
                    // printf("Detect two line match length a and d: %f\n", pattern_length_a);
                    // printf("Distance i.start -> j.end: %f\n", calcVectorLength(line_segments[i].start, line_segments[j].end));
                    // printf("Distance j.start -> i.end: %f\n", calcVectorLength(line_segments[i].end, line_segments[j].start));

                    // Check distance ad
                    if (checkDistance(line_segments[i].start, line_segments[j].end,
                                      pattern_distance_ad, distance_tolerance) &&
                        checkDistance(line_segments[j].start, line_segments[i].end,
                                      pattern_distance_bc, distance_tolerance)) {
                        line_segments_a.push_back(line_segments[i]);
                        line_segments_d.push_back(line_segments[j]);
                        find_ad = true;
                        // printf("Find pair a,d OK!\n");
                    } else if (checkDistance(line_segments[j].start, line_segments[i].end,
                                             pattern_distance_ad, distance_tolerance) &&
                                checkDistance(line_segments[i].start, line_segments[j].end,
                                             pattern_distance_bc, distance_tolerance)) {
                        line_segments_a.push_back(line_segments[j]);
                        line_segments_d.push_back(line_segments[i]);
                        find_ad = true;
                        // printf("Find pair a,d OK!\n");
                    }
                }

            }
        }
    }

    // Find vector b, c
    if (find_ad) {
        const size_t line_a_num = line_segments_a.size();
        if (debug) {
            printf("Found vector a,d: %ld vectors\n", line_a_num);
        }
        for (size_t i = 0; i < line_a_num; ++i) {
            // printf("Vector_a: start[%f:%f], end[%f:%f], angle: %f\n", line_segments_a[i].start[0],
            // line_segments_a[i].start[1], line_segments_a[i].end[0], line_segments_a[i].end[1],line_segments_a[i].angle);
            // printf("Vector_d: start[%f:%f], end[%f:%f], angle: %f\n", line_segments_d[i].start[0],
            // line_segments_d[i].start[1], line_segments_d[i].end[0], line_segments_d[i].end[1],line_segments_d[i].angle);
            for (size_t j = 0; j < line_num; ++j) {
                if (checkAngle(line_segments_a[i].angle, line_segments[j].angle,
                               pattern_angle_ab - M_PI, angle_tolerance)) {
                    // Check distance ab
                    if (checkDistance(line_segments_a[i].end, line_segments[j].start,
                                      0.08, distance_tolerance)) {
                        // Check length b
                        double length_b_detect = calcVectorLength(line_segments[j].start, line_segments[j].end);
                        if (pattern_length_b > length_b_detect > 0.1 ) {
                            vector_a = line_segments_a[i];
                            vector_b = line_segments[j];
                            find_b = true;
                            if (debug) {
                                printf("Found vector_a: start[%f:%f], end[%f:%f], angle: %f\n",
                                        vector_a.start[0], vector_a.start[1], vector_a.end[0], vector_a.end[1],vector_a.angle);
                                printf("Found vector_b: start[%f:%f], end[%f:%f], angle: %f\n",
                                        vector_b.start[0], vector_b.start[1], vector_b.end[0], vector_b.end[1],vector_b.angle); 
                            }
                        }
                    }
                }

                if (checkAngle(line_segments[j].angle, line_segments_d[i].angle,
                               pattern_angle_cd - M_PI, angle_tolerance)) {
                    // Check distance cd
                    if (checkDistance(line_segments[j].end, line_segments_d[i].start,
                                      0.08, distance_tolerance)) {
                        // Check length c
                        double length_c_detect = calcVectorLength(line_segments[j].start, line_segments[j].end);
                        if (pattern_length_b > length_c_detect > 0.1) {
                            vector_c = line_segments[j];
                            vector_d = line_segments_d[i];
                            find_c = true;
                            if (debug) {
                                printf("Found vector_c: start[%f:%f], end[%f:%f], angle: %f\n",
                                        vector_c.start[0], vector_c.start[1], vector_c.end[0], vector_c.end[1],vector_c.angle);
                                printf("Found vector_d: start[%f:%f], end[%f:%f], angle: %f\n",
                                        vector_d.start[0], vector_d.start[1], vector_d.end[0], vector_d.end[1],vector_d.angle);
                            }
                        }
                    }
                }

                if ((find_b && vector_c.radius != 0.0) ||
                    (find_c && vector_b.radius != 0.0)) {
                    // Check angle both vectors bc
                    if (checkAngle(vector_b.angle, vector_c.angle,
                                   pattern_angle_bc - M_PI, angle_tolerance)) {
                        // Check distance bc
                        if (checkDistance(vector_b.start, vector_c.end, pattern_distance_bc,
                                          distance_tolerance)) {
                            find_success = true;
                            if (debug){
                                printf("FOUND %s SUCCESS!!!!!!!!!!! \n", dock_frame_id.c_str());
                            }
                            break;
                        }  else {
                            find_b = false;
                            find_c = false;
                        }
                    } else {
                        find_b = false;
                        find_c = false;
                    }
                }
            }
            if (find_success) {
                break;
            }
        }
    }

    if (find_ad) {
        double x, y, theta;
        if (find_success) {
            boost::array<float, 2UL> point_temp_a = midPoint(vector_a.start, vector_a.end);
            boost::array<float, 2UL> point_temp_d = midPoint(vector_d.start, vector_d.end);
            x = (point_temp_a[0] + point_temp_d[0]) / 2;
            y = (vector_b.start[1] + vector_c.end[1]) / 2;
            // theta = std::atan2(vector_b.start[1] - vector_c.end[1], point_temp_a[0] - point_temp_d[0]) - M_PI / 2;
            theta = M_PI + (vector_a.angle + vector_d.angle)/2;
        } else {
            vector_a = line_segments_a[0];
            vector_d = line_segments_d[0];
            boost::array<float, 2UL> point_temp_a = midPoint(vector_a.start, vector_a.end);
            boost::array<float, 2UL> point_temp_d = midPoint(vector_d.start, vector_d.end);
            x = (point_temp_a[0] + point_temp_d[0]) / 2;
            y = (point_temp_a[1] + point_temp_d[1]) / 2;
            // theta = std::atan2(point_temp_a[1] - point_temp_d[1], point_temp_a[0] - point_temp_d[0]) - M_PI / 2;
            theta = M_PI + (vector_a.angle + vector_d.angle)/2;
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
    ros::init(argc, argv, "find_dock_tf");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    PatternSpecialDock find_charger_tf(nh, nh_local);
    ros::spin();
    return 0;
}
