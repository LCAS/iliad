/*/
 * Copyright (c) 2015 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Fri Jul 24 2015
 */

#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <hanp_filters/laser_filter.h>

TEST_CASE("scan filter test")
{
    // create scan messages
    sensor_msgs::LaserScan scan_in;
    scan_in.header.frame_id = "laser";
    scan_in.angle_min = -1.5;
    scan_in.angle_max = 1.5;
    scan_in.angle_increment = 0.1;
    scan_in.time_increment = 0.1;
    scan_in.scan_time = 0.1;
    scan_in.range_min = 0.5;
    scan_in.range_max = 5.5;
    scan_in.ranges = {5.0, 5.0, 4.5, 4.0, 3.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 4.5, 4.0, 3.5, 3.0, 3.5, 4.0, 4.5, 5.0};

    sensor_msgs::LaserScan scan_out;
    scan_out.header.frame_id = "laser";
    scan_out.angle_min = -1.5;
    scan_out.angle_max = 1.5;
    scan_out.angle_increment = 0.1;
    scan_out.time_increment = 0.1;
    scan_out.scan_time = 0.1;
    scan_out.range_min = 0.5;
    scan_out.range_max = 5.5;
    scan_out.ranges = {5.0, 5.0, 4.5, 4.0, 3.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 4.5, 4.0, 3.5, 5.5, 3.5, 4.0, 4.5, 5.0};

    // create human message
    hanp_msgs::TrackedHumans humans;
    humans.header.frame_id = "laser";
    hanp_msgs::TrackedHuman human;
    human.pose.pose.position.x = 1.5;
    human.pose.pose.position.y = 2.5;
    humans.tracks.push_back(human);
    double human_radius = 0.25;

    // create test fixture class to test protected memebers
    class LaserFilterTestsFixture : public hanp_filters::LaserFilter
    {
        public:
            bool testFilterScan(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out, hanp_msgs::TrackedHumans& humans, double human_radius)
            {
                return filterScan(scan_in, scan_out, humans, human_radius);
            }
    };
    int argc = 0;
    ros::init(argc, NULL, "name"); // have to initialize ros because of pluginlib, needs running roscore
    LaserFilterTestsFixture laserFilterTestsFixture;

    // placeholder for filtered scan
    sensor_msgs::LaserScan scan_out_test;

    // check if function returns true with correct filtered scan
    REQUIRE(laserFilterTestsFixture.testFilterScan(scan_in, scan_out_test, humans, human_radius));
    REQUIRE(scan_out_test.ranges.size() == scan_out.ranges.size());
    // CHECK(scan_out_test.ranges == scan_out.ranges);
    for (auto i = 0; i < scan_out_test.ranges.size(); i++)
    {
        CHECK(scan_out_test.ranges[i] == scan_out.ranges[i]);
    }
}
