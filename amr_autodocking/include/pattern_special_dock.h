#ifndef PATTERN_SPECIAL_DOCK_H_
#define PATTERN_SPECIAL_DOCK_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <boost/array.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <laser_line_extraction/LineSegmentList.h>

class PatternSpecialDock {
    public:
        PatternSpecialDock(ros::NodeHandle&, ros::NodeHandle&);
        ~PatternSpecialDock();

        void loadParameters();
        
        double calcVectorLength(const boost::array<float, 2UL>& start_point,
                                const boost::array<float, 2UL>& end_point);

        bool checkAngle(double a, double b, double angle_pattern_ab, double detect_angle_tolerance);

        bool checkAngle(double angle_1, double angle_2, double angle_threshold);

        bool checkDistance(const boost::array<float, 2UL>& start_point,
                                        const boost::array<float, 2UL>& end_point,
                                        double pattern_distance, double distance_tolerance);

        const boost::array<float, 2UL> midPoint(const boost::array<float, 2UL>& start_point,
                                                const boost::array<float, 2UL>& end_point);

        // const boost::array<float, 2UL> midTwoPoints(const boost::array<float, 2UL>& point1,
        //                                  const boost::array<float, 2UL>& point2);

        void transformTFDock(double x, double y, double theta, const std::string& child_frame_id, const std::string& frame_id);

        void lineSegmentCB(const laser_line_extraction::LineSegmentList::ConstPtr& msg);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_local_;
        ros::Subscriber sub_line_segments_;
        ros::Publisher pub_dock_pose_;

        double pattern_length_a;
        double pattern_length_b;
        double pattern_angle_bc;
        double pattern_angle_ab;
        double pattern_angle_cd;
        double pattern_angle_ad;
        double pattern_distance_ad;
        double pattern_distance_bc;
        double distance_tolerance;
        double length_tolerance;
        double angle_tolerance;
        double angle_threshold;
        std::string laser_frame_id;
        std::string dock_frame_id;
        bool debug;
};

#endif 