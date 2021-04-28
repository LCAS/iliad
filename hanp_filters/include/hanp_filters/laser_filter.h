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
 *                                  Harmish Khambhaita on Wed Jul 22 2015
 */

#ifndef HANP_FILTERS_LASER_FILTER_H
#define HANP_FILTERS_LASER_FILTER_H

#include <ros/ros.h>
#include <filters/filter_base.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

namespace hanp_filters
{
    class LaserFilter : public filters::FilterBase<sensor_msgs::LaserScan>
    {
    public:
        LaserFilter();
        ~LaserFilter();

        // inherited methods
        bool configure();
        bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

        // ros message callbacks
        void trackedHumansReceived(const spencer_tracking_msgs::TrackedPersons& tracked_humans);

    protected:
        // class variables
        double human_radius_;
        spencer_tracking_msgs::TrackedPersons last_tracked_humans_;

        // ros variables
        tf::TransformListener tf_;
        ros::Subscriber tracked_humans_sub_;

        // functions
        // removes scan points that are nearby humans
        //  given scan and humans are in same frame
        bool filterScan(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out, spencer_tracking_msgs::TrackedPersons& humans, double human_radius);
    };
}

#endif // HANP_FILTERS_LASER_FILTER_H
