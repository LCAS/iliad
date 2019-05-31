
/**
 * ToDo: parametrize decay time, prob publish time, intensity
 *
 *
 * */


#include "constrain_maps/constrain_plotter.hpp"


namespace constrain_maps {


    constrain_plotter::constrain_plotter(ros::NodeHandle& n)
    : nodeHandle_(n){

      loadROSParams();

      last_traj_num_chunks= 0;
      currentTrajectoryChunkIdx_ =-1;

      map_.setGeometry(Length(size_x, size_y), resolution, Position(orig_x, orig_y));
      map_.setFrameId(grid_map_frame_id);
      map_.clearAll(); // careful, this would delete values on all layers
      map_.add(layerName, 0.0);

      // test line iterator
      // map_.add("recta_1",0.0);
      // Position startPose(-10,6);
      // Position endPose(5,3);
      // Index startIndex;
      // Index endIndex;
      // map_.getIndex(startPose,startIndex);
      // map_.getIndex(endPose,endIndex);
      //
      // for (grid_map::LineIterator iterator(map_, startIndex, endIndex);
      //       !iterator.isPastEnd(); ++iterator) {
      //     map_.at("recta_1", *iterator) = 1.0;
      // }

      gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(grid_map_topic_name, 1, true);

      // subscribe to input trajectories
      traj_sub_ = nodeHandle_.subscribe(traj_topic_name, 1000, &constrain_plotter::constrainCallback, this);

      // subscribe to reports
      rep_sub_ = nodeHandle_.subscribe(report_topic_name, 1000, &constrain_plotter::reportsCallback, this);

      // Update displayed map periodically
      timer_ = nodeHandle_.createTimer(ros::Duration(map_update_period),  &constrain_plotter::updateMapCallback,this);

      showROSParams();

      ros::spin();

    }

    constrain_plotter::~constrain_plotter(){}

    void constrain_plotter::showROSParams(){

      //...........................................
      ROS_DEBUG("Configuration params:");

      ROS_DEBUG("GRID MAP________________________");
      ROS_DEBUG("size_x [m.]: %2.2f", size_x);
      ROS_DEBUG("size_y [m.]: %2.2f", size_y);
      ROS_DEBUG("orig_x [m.]: %2.2f", orig_x);
      ROS_DEBUG("orig_y [m.]: %2.2f", orig_y);
      ROS_DEBUG("lower_value [-]: %2.2f", lower_value);
      ROS_DEBUG("upper_value [-]: %2.2f", upper_value);

      ROS_DEBUG("resolution [px/m.]: %2.2f", resolution);
      ROS_DEBUG("frame_id: \"%s\"", grid_map_frame_id.c_str());
      ROS_DEBUG("layerName: \"%s\"", layerName.c_str());

      ROS_DEBUG("ROS Params________________________");
      ROS_DEBUG("constrains (by DynamicConstraints) read from: \"%s\"", traj_topic_name.c_str());
      ROS_DEBUG("reports (by mpc controller) read from: \"%s\"", report_topic_name.c_str());
      ROS_DEBUG("grid map published at: \"%s\"", grid_map_topic_name.c_str());
      ROS_DEBUG("map published every: %2.2f seconds", map_update_period);

  }

    void constrain_plotter::loadROSParams(){
              ros::NodeHandle private_node_handle("~");

      // LOAD ROS PARAMETERS ....................................
      std::string temp;

      private_node_handle.param("traj_topic_name", traj_topic_name, std::string("/robot4/control/controller/trajectories_mpc"));
      private_node_handle.param("report_topic_name", report_topic_name, std::string("/robot4/control/controller/reports_mpc"));
      private_node_handle.param("grid_map_topic_name", grid_map_topic_name, std::string("/constrain_grid"));

      private_node_handle.param("grid_map_frame_id", grid_map_frame_id, std::string("/world"));

      private_node_handle.param("resolution", temp,std::string("0.1"));
      resolution=std::stod(temp);

      private_node_handle.param("size_x", temp,std::string("10.0"));
      size_x=std::stod(temp);

      private_node_handle.param("size_y", temp,std::string("10.0"));
      size_y=std::stod(temp);

      private_node_handle.param("orig_x", temp,std::string("0.0"));
      orig_x=std::stod(temp);

      private_node_handle.param("orig_y", temp,std::string("0.0"));
      orig_y=std::stod(temp);

      private_node_handle.param("lower_value", temp,std::string("0.0"));
      lower_value=std::stod(temp);

      private_node_handle.param("upper_value", temp,std::string("1.0"));
      upper_value=std::stod(temp);

      private_node_handle.param("map_update_period", temp,std::string("0.5"));
      map_update_period=std::stod(temp);

      private_node_handle.param("layerName", layerName, std::string("layer"));

  }


/**
 * Process controller report message.
 * @param msg Controller report message
 */
    void constrain_plotter::reportsCallback(const orunav_msgs::ControllerReportConstPtr& msg){
      ROS_DEBUG("[constrain_plotter@reportsCallback] Received report (status:%d, chunk: %d)",msg->status,msg->traj_chunk_sequence_num);
      bool update = false;
      bool hasFinished = false;
      g_lock.lock();
      last_report = *msg;
      // check last_report to find out current trajectory chunk id
      if (msg->status ==3) { // status 3 is running ... otherwise don't bother
          // do we still have at least 1 chunk to process?
          hasFinished = !(msg->traj_chunk_sequence_num<(last_traj_num_chunks-1));

          // AND, is it finished our active chunk?
          update = (!hasFinished)  & (currentTrajectoryChunkIdx_ == msg->traj_chunk_sequence_num);
      }

      // report said last chunk was processed, so no constrain is in place.
      if (hasFinished) {
        currentTrajectoryChunkIdx_ = -1;
      }

      if (update) {
        // forklift should be executing now this chunk
          currentTrajectoryChunkIdx_ = msg->traj_chunk_sequence_num + 1;
      }
      g_lock.unlock();

      if (update) {
          restartTimer();
      }

    }

/**
 * Process new trajectory
 * @param msg Active Trajectory in chunks.
 */
    void constrain_plotter::constrainCallback(const orunav_msgs::ControllerTrajectoryChunkVecConstPtr& msg){
      ROS_DEBUG("[constrain_plotter@constrainCallback] Received ControllerTrajectoryChunkVecConstPtr");

      g_lock.lock();

      last_traj_vec = *msg;
      last_traj_num_chunks = last_traj_vec.chunks.size();

      // upon receving a new trajectory, we set as active the first chunk
      currentTrajectoryChunkIdx_=0;

      g_lock.unlock();

      // force a timer restart to apply and publish constraints
      restartTimer();

    }

    void constrain_plotter::restartTimer(){
      ros::TimerEvent dummy;
      updateMapCallback(dummy);
      timer_.setPeriod(ros::Duration(map_update_period),true);
    }

    void constrain_plotter::updateMapCallback(const ros::TimerEvent&){
        //ROS_DEBUG("[constrain_plotter@updateMapCallback] update and publish event");

        g_lock.lock();
        applyConstraints();
        publishMap();
        g_lock.unlock();
     }

    void constrain_plotter::applyConstraints(){
       orunav_msgs::ControllerConstraints curr_constraints;
       vector<double> a0;
       vector<double> a1;
       vector<double> b;
       unsigned int constraints_num;
       double a0i,a1i,bi;

      // fake vector
      // ROS_ERROR("[constrain_plotter@applyConstraints] Faking CONSTRAINTS!");
      // a0.push_back(2);
      // a1.push_back(1);
      // b.push_back(14);
      //
      // a0.push_back(-1);
      // a1.push_back(1);
      // b.push_back(3);
      //
      // a0.push_back(-1);
      // a1.push_back(-1);
      // b.push_back(5);
      //
      // a0.push_back(3);
      // a1.push_back(-20);
      // b.push_back(-1);



        // get space constraints A0,A1 and b from current chunk
        if (currentTrajectoryChunkIdx_ > (-1) ) {
            ROS_DEBUG("[constrain_plotter@applyConstraints] Constraints active (%d)!",currentTrajectoryChunkIdx_);
            curr_constraints = last_traj_vec.chunks[currentTrajectoryChunkIdx_].constraints;

            a0 =  curr_constraints.spatial_coef_a0;
            a1 =  curr_constraints.spatial_coef_a1;
            b =    curr_constraints.spatial_coef_b;
            constraints_num = a0.size();
            map_[layerName].setConstant(0.0);

            // apply all the constraints
            for (unsigned int c = 0; c < constraints_num; c++){
              ROS_DEBUG("[constrain_plotter@applyConstraints] Aplying constraint %d of %d",c+1,constraints_num);

              std::string constrainLayer = std::to_string(c);
              a0i=a0[c];
              a1i=a1[c];
              bi=b[c];
              ROS_DEBUG("a0=%3.3f, a1=%3.3f, b=%3.3f",a0i,a1i,bi);


              // this has a weird offset. Let's try something else.
              // map_.add(constrainLayer, - bi);
              // double x_offset=0.0;
              // Position point;
              // for (GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
              //     map_.getPosition(*iterator, point);
              //     map_.at(constrainLayer,*iterator) += ( a0i* point.x()) + (a1i* point.y());
              // }

              Polygon pol_cons = getPolygon(a0i,a1i,bi);
              map_.add(constrainLayer, 1.0);
              // whatever is inside, it's ok
              for (grid_map::PolygonIterator iterator(map_, pol_cons);
                  !iterator.isPastEnd(); ++iterator) {
                        map_.at(constrainLayer, *iterator) = 0.0;
              }

              //ROS_DEBUG("[constrain_plotter@applyConstraints] Linear constraint");
              //printLayerCorners(constrainLayer);

              // we add constraints!
              map_[layerName] = map_[layerName]+(map_[constrainLayer]/constraints_num);
            }
          } else{
            ROS_DEBUG("[constrain_plotter@applyConstraints] No constraints active (%d)!",currentTrajectoryChunkIdx_);
          }
     }

    void constrain_plotter::printLayerCorners(std::string layer){
      Size siz= map_.getSize();

      ROS_DEBUG("\n\n......................................\n[constrain_plotter@printLayerCorners]");



      // iterating over indexes
      // vector<int> indexes;
      // indexes.push_back(0);
      // indexes.push_back( (siz[0]-1)/2 );
      // indexes.push_back( (siz[0]-1) );
      //
      // for (unsigned int ix = 0; ix < indexes.size() ; ix++){
      //   for (unsigned int iy = 0; iy < indexes.size() ; iy++){
      //     Index currInd(indexes[ix],indexes[iy]);
      //     Position  currPos;
      //     map_.getPosition(currInd,currPos);
      //       ROS_DEBUG("map[%s][%d, %d] (%3.2f, %3.2f) \t==> %3.3f",layer.c_str(), currInd[0], currInd[1], currPos[0], currPos[1],map_.at(layer, currInd));
      //   }
      // }


      // poking with indexes and corresponding positions...

      // Index startIndex(0,0);
      // Position  startPose;
      // map_.getPosition(startIndex,startPose);
      //
      // Index bottRightCorner(siz[0]-1,1);
      // Position  bottRightCornerPose;
      // map_.getPosition(bottRightCorner,bottRightCornerPose);
      //
      // Index endIndex(siz[0]-1, siz[1]-1);
      // Position  endPose;
      // map_.getPosition(endIndex,endPose);
      //
      // Index uppLeftCorner(0, siz[1]-1);
      // Position  uppLeftCornerPose;
      // map_.getPosition(uppLeftCorner,uppLeftCornerPose);

      // ROS_DEBUG("map[%s][%d, %d] (%3.3f, %3.3f) == %3.3f",layer.c_str(), startIndex[0], startIndex[1], startPose[0], startPose[1], map_.at(layer, startIndex));
      // ROS_DEBUG("map[%s][%d, %d] (%3.3f, %3.3f) == %3.3f",layer.c_str(), bottRightCorner[0], bottRightCorner[1], bottRightCornerPose[0], bottRightCornerPose[1], map_.at(layer, bottRightCorner));
      // ROS_DEBUG("map[%s][%d, %d] (%3.3f, %3.3f) == %3.3f",layer.c_str(), endIndex[0], endIndex[1], endPose[0], endPose[1], map_.at(layer, endIndex));
      // ROS_DEBUG("map[%s][%d, %d] (%3.3f, %3.3f) == %3.3f",layer.c_str(), uppLeftCorner[0], uppLeftCorner[1], uppLeftCornerPose[0], uppLeftCornerPose[1], map_.at(layer, uppLeftCorner));



      // taking fixed  positions and printint them

      // Position p1(-4.99,-4.99);
      // Position p2(-4.99,0);
      // Position p3(-4.99,4.99);
      //
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p1[0], p1[1],map_.atPosition(layer, p1));
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p2[0], p2[1],map_.atPosition(layer, p2));
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p3[0], p3[1],map_.atPosition(layer, p3));
      //
      // p1=Position(0,-4.99);
      // p2=Position(0,0);
      // p3=Position(0,4.99);
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p1[0], p1[1],map_.atPosition(layer, p1));
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p2[0], p2[1],map_.atPosition(layer, p2));
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p3[0], p3[1],map_.atPosition(layer, p3));
      //
      // p1=Position(4.99,-4.99);
      // p2=Position(4.99,0);
      // p3=Position(4.99,4.99);
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p1[0], p1[1],map_.atPosition(layer, p1));
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p2[0], p2[1],map_.atPosition(layer, p2));
      // ROS_DEBUG("map[%s][%3.3f, %3.3f] == %3.3f",layer.c_str(), p3[0], p3[1],map_.atPosition(layer, p3));

      //iterate over y
      for (unsigned int iy = 0; iy < siz[1] ; iy=iy+10){
          Index currInd(0,iy);
          Position  currPos;
          map_.getPosition(currInd,currPos);
            ROS_DEBUG("map[%s][%d, %d] (%3.2f, %3.2f) \t==> %3.3f",layer.c_str(), currInd[0], currInd[1], currPos[0], currPos[1],map_.at(layer, currInd));
        }



      ROS_DEBUG("......................................\n\n");
    }


     Polygon constrain_plotter::getPolygon(double a0, double a1, double b){
      double x0,xN,y0,yN;
      Position c1,c2,c3,c4,r1,r2,r3,r4;
      Polygon pol1,pol2,ans;

      Size siz= map_.getSize();

      Position  poseA;
      map_.getPosition(Index(0,0),poseA);

      Position  poseB;
      map_.getPosition(Index(siz[0]-1, siz[1]-1),poseB);

      x0 = std::min(poseA[0],poseB[0]);
      xN = std::max(poseA[0],poseB[0]);
      y0 = std::min(poseA[1],poseB[1]);
      yN = std::max(poseA[1],poseB[1]);

      c1=Position(x0,y0);
      c2=Position(x0,yN);
      c3=Position(xN,yN);
      c4=Position(xN,y0);

      // constraint/map boundaries intersections ...
      r1=Position(getOther(a0,a1,b,yN),yN);
      r2=Position(x0,getOther(a1,a0,b,x0));
      r3=Position(xN,getOther(a1,a0,b,xN));
      r4=Position(getOther(a0,a1,b,y0),y0);

      if (map_.isInside(r1)){
         pol1.addVertex(r1);
      }
      if (map_.isInside(r2)){
         pol1.addVertex(r2);
      }
      if (map_.isInside(r3)){
         pol1.addVertex(r3);
      }
      if (map_.isInside(r4)){
         pol1.addVertex(r4);
      }

      if (obeysConstrain(a0,a1,b,c1)){
         pol2.addVertex(c1);
      }
      if (obeysConstrain(a0,a1,b,c2)){
         pol2.addVertex(c2);
      }
      if (obeysConstrain(a0,a1,b,c3)){
         pol2.addVertex(c3);
      }
      if (obeysConstrain(a0,a1,b,c4)){
         pol2.addVertex(c4);
      }
      ans = Polygon::convexHull(pol1,pol2);

      std::vector<Position> points= ans.getVertices();
      ROS_DEBUG("[constrain_plotter@getPolygon] Polygon points");
      for (int i=0;i<points.size();i++) {
        ROS_DEBUG("(%3.3f,%3.3f)",points[i][0],points[i][1]);
      }

      return ans;
    }

    double constrain_plotter::getOther(double a0, double a1, double b,double y){
      double ans;
      if (std::abs(a0)>0.0){
         ans = (b-(a1*y))/(a0);
      } else {
         ans = 0;
         ROS_FATAL("[constrain_plotter@publishMap] a is 0 in constrain!");
      }

      return ans;
    }

    bool constrain_plotter::obeysConstrain(double a0, double a1, double b,Position p){
      double ans = a0*p[0]+ a1*p[1] - b;
      return (ans<=0);
    }

    void constrain_plotter::publishMap(){
      ROS_DEBUG("[constrain_plotter@publishMap] Publishing gridmap");
      map_.setTimestamp(ros::Time::now().toNSec());
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(map_, message);
      gridMapPublisher_.publish(message);

    }


} // end of namespace constrain_maps
