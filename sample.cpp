#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cstddef>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <stdint.h>
#include <list>
#include <set>

#include "hd_map_client.h"
#include "route_manager.h"


TArcPair drawNodes(std::list<TArcPair> &current_visual, TomTom::AutoStream::HdMap::CHdMapAccess* mapAccess, TomTom::AutoStream::HdMap::CHdMapAccess* speed_mapAccess,bk::block& car, cv::Mat& mapVisualizer);
void drawlanemarkings(LaneMarkings& lanemark, bk::block& car, cv::Mat& mapVisualizer, std::list<EgoConverter> &lane_ego_transform, TomTom::AutoStream::HdMap::CHdMapAccess *traffic_mapAccess, TArcPair current_lane);
bool checkNet()
{
    system("ping www.google.com -c 2 -w 2 >netlog.bat"); 
    sleep(2);
 
    ifstream infile;
    infile.open("netlog.bat"); 
    string s;
    vector<string> v;
    while(infile)
    {
        getline(infile,s);
        if(infile.fail()) 
            break;
        v.push_back(s);
    }
    infile.close();
    if (v.size() > 1)
    {
        string data = v[v.size()-2];    
        int iPos = data.find("received,");  
        if (iPos != -1 )
        {
            data = data.substr(iPos+10,3);
            int  n = atoi(data.c_str()); 
            if(n == 0)
             return 1; 
            else
            return 0 ; 
        }
       
    }else{
        return 0;
    }
    return 0;
}
int main()
{
  std::string config_file_path ="../database/config.ini";
  hdMapConfig::Config::Ptr config(new hdMapConfig::Config(config_file_path));
  LAT = config->LAT;
  LNG = config->LNG;
  bool eHorizon = config->ehorizon_switch;

  LAT_B = LAT-0.005;
  LNG_B = LNG-0.005;
  bk::block car;
  LaneMarkings lanemark;
  TrafficSignMarkings traffic_signs;
  TomTom::AutoStream::Reference::CSqlitePersistentTileCacheV2 persistentTileCache;
  TomTom::AutoStream::Reference::CHttpDataUsageLogger httpDataUsageLogger;
  TomTom::AutoStream::CAutoStream autoStream;
  TomTom::AutoStream::CAutoStreamSettings settings;
  TomTom::AutoStream::CAllocatorSettings allocSettings;
  TomTom::AutoStream::CAllocatorFactory  allocatorFactory(allocSettings);
  TomTom::AutoStream::CMapVersionAndHash mapVersionAndHash;
  TomTom::AutoStream::CMapVersionAndHash mapVersionAndHash_traffic;
  TomTom::AutoStream::CMapVersionAndHash mapVersionAndHash_speed;

  using TomTom::AutoStream::CImmutableStringReference;
 

  const CImmutableStringReference                             cacheFile(kPersistentTileCache,
                                            std::strlen(kPersistentTileCache));
  if (!persistentTileCache.initialize(cacheFile))
  { /* error */
    return 1;
  }
  TomTom::AutoStream::Reference::CSqlitePersistentTileCacheV2 readOnlyPersistentTileCache(true);
  TomTom::AutoStream::CImmutableStringReference cacheFile_offline(kPersistentTileCache, std::strlen(kPersistentTileCache));
  if (!readOnlyPersistentTileCache.initialize(cacheFile_offline))
  { /* error */
    std::cout<<"read cache error"<<std::endl;
    return 1;
  }
  

  const CImmutableStringReference                     dataUsageLogFile(kHttpDataUsageLogFile,
                                                   std::strlen(kHttpDataUsageLogFile));
  const bool clearHttpDataUsageLog = true;
  bool       httpDataUsageLoggerInitialized =
    httpDataUsageLogger.initialize(dataUsageLogFile, clearHttpDataUsageLog);
  if (!httpDataUsageLoggerInitialized)
  { /* error */
    return 1;
  }

  const CImmutableStringReference caCertificatesFile(kAutoStreamCaCertificatesFile,
                                                     std::strlen(kAutoStreamCaCertificatesFile));
  TomTom::AutoStream::Reference::CBoostHttpsClientV2 httpsClient(
    caCertificatesFile, kNumberOfServerConnections, httpDataUsageLogger);


 
  auto mapVersion = mapVersionAndHash.mapVersion;
  auto traffic_mapVersion = mapVersionAndHash_traffic.mapVersion;
  auto speed_mapVersion = mapVersionAndHash_speed.mapVersion;
  //int start = init_prog(autoStream, settings, allocatorFactory, mapVersionAndHash, mapVersionAndHash_traffic, httpsClient,persistentTileCache);
  if (checkNet()){
    std::cout<<"success in connection"<<std::endl;
    int start = init_prog(autoStream, settings, allocatorFactory, mapVersionAndHash, mapVersionAndHash_traffic,mapVersionAndHash_speed, httpsClient,persistentTileCache);
    mapVersion = mapVersionAndHash.mapVersion;
    traffic_mapVersion = mapVersionAndHash_traffic.mapVersion;
    speed_mapVersion = mapVersionAndHash_speed.mapVersion;
  }else{
    std::cout<<"error in connection"<<std::endl;
    bool test_offline = init_offline(autoStream, settings, allocatorFactory,readOnlyPersistentTileCache);
    if (test_offline){
      std::cout<<"error in init offline"<<std::endl;
    }
    const TomTom::AutoStream::CSpan<const uint8_t> mapVersionSerialized(
      &kMapVersionSerialized[0U], sizeof(kMapVersionSerialized) / sizeof(uint8_t));
    mapVersion = TomTom::AutoStream::CMapVersion(mapVersionSerialized);
    const TomTom::AutoStream::CSpan<const uint8_t> mapVersionSerialized_traffic(
      &kTrafficMapVersionSerialized[0U], sizeof(kTrafficMapVersionSerialized) / sizeof(uint8_t));
    traffic_mapVersion = TomTom::AutoStream::CMapVersion(mapVersionSerialized_traffic);
    
  }


  const TomTom::AutoStream::HdMap::CHdMap& hdMap = autoStream.getHdMap();
  TomTom::AutoStream::HdMap::CHdMapAccess* mapAccess = hdMap.createHdMapAccess(
    TomTom::AutoStream::CLayerBitset::kHdRoad, mapVersion);
  if (mapAccess == NULL)
  { /* error */
    return 1;
  }

  TomTom::AutoStream::HdMap::CHdMapAccess* traffic_mapAccess = hdMap.createHdMapAccess(
    TomTom::AutoStream::CLayerBitset::kHdTrafficSigns, traffic_mapVersion);
  if (mapAccess == NULL)
  { /* error */
  return 1;
  }

  TomTom::AutoStream::HdMap::CHdMapAccess* speed_mapAccess = hdMap.createHdMapAccess(
    TomTom::AutoStream::CLayerBitset::kHdSpeedRestrictions, speed_mapVersion);
  if (mapAccess == NULL)
  { /* error */
  return 1;
  }

  TomTom::AutoStream::CMapVersion currentMapVersion = mapAccess->getCurrentMapVersion();
  

  TomTom::AutoStream::CHintingService& hintingService = autoStream.getHintingService();

///////////////// parse kml route file
    string route_filename = "../database/us101.kml";
    RouteManager route_manager(route_filename);
    uint rp_index = 0;
    uint rp_index_max = route_manager.route_full_.size() -1;
    if(route_manager.route_full_.empty()){
        rp_index_max = 0;
        eHorizon = false;
    }

////////////////
  
  int offsetMapVisualizer = 0;
  int key = 0;
  float angle = 180;
  
  
  init_car(car, angle, offsetMapVisualizer);
  car.set_scale(config->block_scale);

  std::list<TArcPair> current_visual;

  std::vector<TArcPair> temp_stack;

  const TomTom::AutoStream::CHintingService::TApplicationId applicationId = 1U;
  const uint8_t                                             priority      = 10U;
  TomTom::AutoStream::CHintingService::THintHandle          hint =
    hintingService.providePositionOfInterestHint(applicationId,
                                       priority,
                                       mapAccess->getCurrentMapVersion(),
                                       TomTom::AutoStream::CLayerBitset::kHdRoad,
                                       5000 /* meter */);
  TomTom::AutoStream::CHintingService::THintHandle          hint_traffic =
    hintingService.providePositionOfInterestHint(applicationId,
                                       priority,
                                       traffic_mapAccess->getCurrentMapVersion(),
                                       TomTom::AutoStream::CLayerBitset::kHdTrafficSigns,
                                       5000 /* meter */);
  TomTom::AutoStream::CHintingService::THintHandle          hint_speed =
    hintingService.providePositionOfInterestHint(applicationId,
                                       priority,
                                       speed_mapAccess->getCurrentMapVersion(),
                                       TomTom::AutoStream::CLayerBitset::kHdSpeedRestrictions,
                                       5000 /* meter */);
    TomTom::AutoStream::TCoordinate posJohnCLodgeFwyDetroit =
    TomTom::AutoStream::TCoordinate::createFromDegrees(LAT_B + 0.005, LNG + 0.005);

    hintingService.updatePositionOfInterest(posJohnCLodgeFwyDetroit);

  try{
    while(key!=27){
      cv::Mat mapVisualizer = cv::Mat(cv::Size(1500,900), CV_8UC3, cv::Scalar(255, 255, 255));
      //cv::circle(mapVisualizer, cv::Point2f(car.get_pixelLat(), car.get_pixelLon()), 6.0, cv::Scalar(0, 0, 255), 2, 8);
      cv::circle(mapVisualizer, cv::Point2f(500, 800), 12.0, cv::Scalar(0, 0, 255), 2, 8);

      if (car.get_lon()-LNG_B < 0.0025 || car.get_lon()-LNG_B > 0.0075 ||car.get_lat()-LAT_B < 0.0025 || car.get_lat()-LAT_B > 0.0075){
        LAT_B = car.get_lat()-0.005;
        LNG_B = car.get_lon()-0.005;
        car.update_map();
        car.set_block_point();
        posJohnCLodgeFwyDetroit = TomTom::AutoStream::TCoordinate::createFromDegrees(LAT_B + 0.005, LNG_B + 0.005);
        hintingService.updatePositionOfInterest(posJohnCLodgeFwyDetroit);

      }
      
      if (eHorizon){
        tomtom_parsing(car, mapAccess, traffic_mapAccess, speed_mapAccess, current_visual, temp_stack, lanemark, traffic_signs,false, route_manager.route_full_, rp_index, config);
      }
      else {
        tomtom_parsing_custom(car, mapAccess, traffic_mapAccess, speed_mapAccess, current_visual, temp_stack, lanemark, traffic_signs, false);
      }
      auto current_lane = drawNodes(current_visual, mapAccess, speed_mapAccess, car, mapVisualizer);
      
      std::list<EgoConverter> lane_ego_transform;
      // car.convert_to_ego(lanemark, lane_ego_transform);

        // route waypoints display
        auto route_point0 = car.get_transform(0, 0); //init default value
        for (int i =1;i<route_manager.route_ori_.size();i++){
            if(i==1){
                route_point0 = car.get_transform(route_manager.route_ori_[0].latitude, route_manager.route_ori_[0].longitude);
            }
            auto route_point1 = car.get_transform(route_manager.route_ori_[i].latitude, route_manager.route_ori_[i].longitude);
            cv::line(mapVisualizer, cv::Point2f(route_point1[0], route_point1[1]),cv::Point2f(route_point0[0], route_point0[1]), cv::Scalar(200, 0, 0), 2, 8);
            route_point0 = route_point1;
        }

      drawlanemarkings(lanemark, car, mapVisualizer, lane_ego_transform, traffic_mapAccess, current_lane);
      cv::imshow("output", mapVisualizer);

      float ang_rad = car.get_angle()/180*MY_PI;
      
      //WAITKEY:
      key = cv::waitKey(-1);
      if (key == 'a') {
          angle -= 2;
          car.set_angle(angle);
          car.set_block_point();
      }
      else if (key == 'd') {
          angle += 2;
          car.set_angle(angle);
          car.set_block_point();
      }
      else if (key == 'i'){
          car.set_carP(car.get_pixelLat()+10*sin(-ang_rad), car.get_pixelLon()+10*cos(-ang_rad));
          car.set_block_point();
      }
      else if (key == 'k'){
          car.set_carP(car.get_pixelLat()-10*sin(-ang_rad), car.get_pixelLon()-10*cos(-ang_rad));
          car.set_block_point();
      }
      else if (key == 'j'){
          car.set_carP(car.get_pixelLat()+10*cos(ang_rad), car.get_pixelLon()+10*sin(ang_rad));
          car.set_block_point();
      }
      else if (key == 'l'){
          car.set_carP(car.get_pixelLat()-10*cos(ang_rad), car.get_pixelLon()-10*sin(ang_rad));
          car.set_block_point();
      }
      else if (key == 'c'){
          if(rp_index < rp_index_max){
              rp_index += 10;
          }

          angle = route_manager.route_full_[rp_index].heading;
          car.set_angle(angle);
          car.set_carM(route_manager.route_full_[rp_index].latitude, route_manager.route_full_[rp_index].longitude);
          car.set_block_point();


      }
      else if (key == 'z'){
          if(rp_index > 0){
              rp_index--;
          }
          angle = route_manager.route_full_[rp_index].heading;
          car.set_angle(angle);
          car.set_carM(route_manager.route_full_[rp_index].latitude, route_manager.route_full_[rp_index].longitude);
          car.set_block_point();
          //std::cout<<"rp_index: "<<rp_index<<std::endl;

      }
      else if (key == '='){
          car.set_scale(car.get_scale()+1);
          car.set_block_point();
          //std::cout<<"rp_index: "<<rp_index<<std::endl;
      }
      else if (key == '-'){
          car.set_scale(car.get_scale()-1);
          car.set_block_point();
          //std::cout<<"rp_index: "<<rp_index<<std::endl;
      }
      else if (key == 27){
        continue;
      }

    }
        
    
  }
  catch (const std::exception& e)
  {
      std::cerr << "Exception thrown" <<e.what()<<std::endl;
      return -1;
  }

  hintingService.forgetHint(hint);
  hdMap.destroyHdMapAccess(mapAccess);
  autoStream.stop();

  return 0;
}

TArcPair drawNodes(std::list<TArcPair> &current_visual, TomTom::AutoStream::HdMap::CHdMapAccess* mapAccess,TomTom::AutoStream::HdMap::CHdMapAccess* speed_mapAccess, bk::block& car, cv::Mat& mapVisualizer){
  int counter = 0;
  int shortest_distance = 999;
  auto closest_arc = current_visual.back();

  for(auto i : current_visual){
    uint32_t nr_lanes = mapAccess->nrOfLanesOrTrajectories(i.khdroad);
   
    for (int lane_idx = 0; lane_idx< nr_lanes; lane_idx++){
      auto lane_Trajectory = mapAccess->getLaneOrTrajectory(i.khdroad, lane_idx);
      // auto carType = TomTom::AutoStream::HdMap::HdMapSpeedRestrictionLayer::TVehicleType::kVehicleTypePassengerCar;
      // auto speed_rest = speed_mapAccess->getSpeedRestrictions();
      // uint32_t nr_lanes_speed = speed_mapAccess->nrOfLanesOrTrajectories(i.kspeedrestriction);
      // auto speed_idx = lane_idx>(nr_lanes_speed-1)? (nr_lanes_speed-1): lane_idx;
      // auto current_speedlimit = speed_rest.getSpeedRestrictions(i.kspeedrestriction, speed_idx, carType, TomTom::AutoStream::CCallParameters());
      // auto speed_index = current_speedlimit.getNrLaneSpeedRestrictions();
      // if (speed_index == 0 && lane_idx<nr_lanes-1){
      //   current_speedlimit = speed_rest.getSpeedRestrictions(i.kspeedrestriction, lane_idx+1, carType, TomTom::AutoStream::CCallParameters());
      //   speed_index = current_speedlimit.getNrLaneSpeedRestrictions();
      // }else if (speed_index == 0 && lane_idx == nr_lanes-1){
      //   current_speedlimit = speed_rest.getSpeedRestrictions(i.kspeedrestriction, lane_idx-1, carType, TomTom::AutoStream::CCallParameters());
      //   speed_index = current_speedlimit.getNrLaneSpeedRestrictions();
      // }

      // auto nr_connection = lane_Trajectory.nrOfOutgoingLaneConnections();
      // if (lane_Trajectory.laneType() == 6 || nr_connection == 0) continue;
      // TomTom::AutoStream::HdMap::HdRoad::CIterable3DShapePoints cl_coords = lane_Trajectory.laneCenterLine();
      // auto xyt = (*cl_coords.begin()).getXY();
      // auto temp2 = car.get_transform(xyt.getLatDegree(), xyt.getLonDegree());
      
      // for (auto&& cl_coord: cl_coords){
      //     auto xy = cl_coord.getXY();
      //     auto temp1 = car.get_transform(xy.getLatDegree(), xy.getLonDegree());
      //     auto temp_ego = car.get_transform(xyt.getLatDegree(), xyt.getLonDegree());
          
      //     cv::line(mapVisualizer, cv::Point2f(temp1[0], temp1[1]),cv::Point2f(temp2[0], temp2[1]), cv::Scalar(100*(counter%3), 100*((counter+1)%3), 100*((counter+2)%3)), 2, 8);
      //     // if (speed_index == 0){
      //     //   cv::line(mapVisualizer, cv::Point2f(temp1[0], temp1[1]),cv::Point2f(temp2[0], temp2[1]), cv::Scalar(lane_Trajectory.isExitLane()* 255, 0, (1-lane_Trajectory.isExitLane()* 255 )), 2, 8);
          
      //     // }else{
      //     //   cv::line(mapVisualizer, cv::Point2f(temp1[0], temp1[1]),cv::Point2f(temp2[0], temp2[1]), cv::Scalar(255-(current_speedlimit.getLaneSpeedRestrictions(speed_index-1).getSpeedLimitValue(0)-30)*255, current_speedlimit.getLaneSpeedRestrictions(speed_index-1).getSpeedLimitValue(0)*4, (current_speedlimit.getLaneSpeedRestrictions(speed_index-1).getSpeedLimitValue(0)-50)*255), 2, 8);
      //     // }
      //     temp2 = temp1;
      // }
      auto near = mapAccess->findNearestPointOnLine(TomTom::AutoStream::TCoordinate::createFromDegrees(car.get_lat(), (car.get_lon())), lane_Trajectory);
      auto nLat = near.projection().getLatDegree();
      auto nLng = near.projection().getLonDegree();
      
      auto temp = car.get_distance(nLat, nLng);
      if (temp < shortest_distance){
        shortest_distance = temp;
        closest_arc = i;
        closest_arc.arc_idx = lane_idx;

      }
    }
    counter += 1;
  }
  uint32_t nr_lanes = mapAccess->nrOfLanesOrTrajectories(closest_arc.khdroad);
  for (int lane_idx = 0; lane_idx< nr_lanes; lane_idx++){
      auto lane_Trajectory = mapAccess->getLaneOrTrajectory(closest_arc.khdroad, lane_idx);
      TomTom::AutoStream::HdMap::HdRoad::CIterable3DShapePoints cl_coords = lane_Trajectory.laneCenterLine();
      auto xyt = (*cl_coords.begin()).getXY();
      auto temp2 = car.get_transform(xyt.getLatDegree(), xyt.getLonDegree());
      
      for (auto&& cl_coord: cl_coords){
          auto xy = cl_coord.getXY();
          auto temp1 = car.get_transform(xy.getLatDegree(), xy.getLonDegree());
          //auto temp_ego = car.get_transform(xyt.getLatDegree(), xyt.getLonDegree());
          
          cv::line(mapVisualizer, cv::Point2f(temp1[0], temp1[1]),cv::Point2f(temp2[0], temp2[1]), cv::Scalar(100*(counter%3), 100*((counter+1)%3), 100*((counter+2)%3)), 2, 8);
          temp2 = temp1;
      }
  }
  return closest_arc;
  //found cloest arc


  ////////////////////////////////////////////////////////////////////////////////////
  // double shortest_distance_inlane = 999;
  // int current_lane_idx = 0;
  // uint32_t nr_lanes = mapAccess->nrOfLanesOrTrajectories(closest_arc);
  // for (int i = 0; i < nr_lanes; i++){
  //   auto lane_Trajectory_temp = mapAccess->getLaneOrTrajectory(closest_arc, i);
      
  //   //Get all Information of a Lane
  //   auto nr_connection = lane_Trajectory_temp.nrOfOutgoingLaneConnections();

  //   if (lane_Trajectory_temp.laneType() == 6 || nr_connection == 0) continue;    
  //   //Check if the line is in the boundary
  //   auto near = mapAccess->findNearestPointOnLine(TomTom::AutoStream::TCoordinate::createFromDegrees(car.get_lat(), (car.get_lon())), lane_Trajectory_temp);
  //   auto nLat = near.projection().getLatDegree();
  //   auto nLng = near.projection().getLonDegree();
    
  //   auto temp = car.get_distance(nLat, nLng);
  //   if (temp < shortest_distance){
  //     shortest_distance_inlane = temp;
  //     current_lane_idx = i;
  //   }
  // }
  /////////////////////////////////////////////////////////////////////////
  //  int counter = 0;
  // for(auto i : current_visual){
  //   uint32_t nr_lanes = mapAccess->nrOfLanesOrTrajectories(i.khdroad);
  //   for (int lane_idx = 0; lane_idx< nr_lanes; lane_idx++){
  //     auto lane_Trajectory = mapAccess->getLaneOrTrajectory(i.khdroad, lane_idx);
  //     auto nr_connection = lane_Trajectory.nrOfOutgoingLaneConnections();
  //     if (lane_Trajectory.laneType() == 6 || nr_connection == 0) continue;
  //     TomTom::AutoStream::HdMap::HdRoad::CIterable3DShapePoints cl_coords = lane_Trajectory.laneCenterLine();
  //     auto exit = lane_Trajectory.isExitLane();
  //     auto xyt = (*cl_coords.begin()).getXY();
  //     auto temp2 = car.get_transform(xyt.getLatDegree(), xyt.getLonDegree());
  //     for (auto&& cl_coord: cl_coords){
  //         auto xy = cl_coord.getXY();
  //         auto temp1 = car.get_transform(xy.getLatDegree(), xy.getLonDegree());
  //         auto temp_ego = car.get_transform(xyt.getLatDegree(), xyt.getLonDegree());
  //         //cv::line(mapVisualizer, cv::Point2f(temp1[0], temp1[1]),cv::Point2f(temp2[0], temp2[1]), cv::Scalar(100*(counter%3), 100*((counter+1)%3), 100*((counter+2)%3)), 2, 8);
  //         cv::line(mapVisualizer, cv::Point2f(temp1[0], temp1[1]),cv::Point2f(temp2[0], temp2[1]), cv::Scalar(255*exit,0,0), 2, 8);

  //         temp2 = temp1;
  //     }
  //   }
  //   counter += 1;
  // }
}


void drawlanemarkings(LaneMarkings& lanemark, bk::block& car, cv::Mat& mapVisualizer, std::list<EgoConverter> &lane_ego_transform, TomTom::AutoStream::HdMap::CHdMapAccess *traffic_mapAccess, TArcPair current_lane){
  const TomTom::AutoStream::HdMap::CHdMapTrafficSigns& traffic_signs = traffic_mapAccess->getTrafficSigns();
 //std::cout<<lanemark.lanemarking_list.size()<<std::endl;
 // std::cout << "Group: " << lanemark.lanemarking_list.size()<<std::endl;
 for (auto&& i : lanemark.lanemarking_list){
  // std::cout<< i.first.size() <<std::endl;
  auto temp1 = car.get_transform((*i.first.begin()).x_f, (*i.first.begin()).y_f);
  // int current_lane_id = -1;
  // int current_arc_id = -1;
  //std::cout<<"speed limit: "<<i.first.back().speed_limit<<std::endl;
// 
//   // for (auto j : lane_ego_transform){
//   //   if (j.lane_id == i.second.lane_count) {
//   //     i.second.ego_idx = j.ego_id;
//   //     //std::cout<<i.second.ego_idx<<std::endl;
//   //     break;
//   //   }
//   // }

  int color_idx;// = i.second.ego_idx;
 // auto ptr = i.first.begin();
  //advance(ptr,2);
 // auto text_temp = car.get_transform((*ptr).x_f, (*ptr).y_f);
//   // Eigen::Vector3f cloest_point = {100,100,1}; 
//   // double cloest_dist = 10000;
  bool onetime = true;
  for (auto j : i.first){
    auto temp = car.get_transform(j.x_f, j.y_f);
    //std::cout<<"jLane_loc:"<<j.lane_loc<<", current_lane:"<<current_lane.visual_count<<std::endl;
    std::cout<<current_lane.arc_idx<<std::endl;
    if (j.lane_loc == current_lane.visual_count && onetime){
      onetime = false;
      if (j.lane_idx < current_lane.arc_idx){
        color_idx = (current_lane.arc_idx-j.lane_idx)*2-1;
      }else{
        color_idx = (j.lane_idx - current_lane.arc_idx)*2;
      }
      cv::putText(mapVisualizer, std::to_string(color_idx) ,cv::Point2f(temp[0], 800), cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(118, 185, 0), 2);//color_idx for lane number

    }
    
    for (auto k : j.traffic_markings.traffic_sign_list){
      auto sign_localtion_info = traffic_signs.getCenterOfMass(k).getXY();
      auto sign = car.get_transform(sign_localtion_info.getLatDegree() , sign_localtion_info.getLonDegree());
      cv::circle(mapVisualizer, cv::Point2f(sign[0], sign[1]), 6.0, cv::Scalar(0, 255, 0), 2, 8);
    }
    //if (color_idx>=0){
      // auto current_dist = car.get_distance(j.x_f, j.y_f);
      // if (current_dist < cloest_dist){
      //   cloest_dist = current_dist;
      //   cloest_point = temp;
      // }
    cv::circle(mapVisualizer, cv::Point2f(temp[0], temp[1]), 5.0, cv::Scalar(255- (color_idx * 20) % 255, 255 - (color_idx * 20) % 255, (color_idx * 20) % 255), 2, 8);
    //cv::line(mapVisualizer, cv::Point2f(temp[0], temp[1]),cv::Point2f(temp1[0], temp1[1]), cv::Scalar(255- (j.speed_limit * 2) ,0,50), 4,8);
   // std::cout<<"Output speed Limit: "<< j.speed_limit<<std::endl;
   //std::cout<<color_idx<<std::endl;
    cv::line(mapVisualizer, cv::Point2f(temp[0], temp[1]),cv::Point2f(temp1[0], temp1[1]), cv::Scalar(255- (color_idx * 20) , 255 - (color_idx * 20) , (color_idx * 20) ), 4,8);
    temp1 = temp;

    }

  }
}


std::vector<std::vector<std::pair<double, double>>> get_lanemarkings_ego(LaneMarkings& lanemark, bk::block& car){
  std::vector<std::vector<std::pair<double, double>>> lanemarkings_ego;
  // std::cout << "Group: " << lanemark.lanemarking_list.size()<<std::endl;
  uint idx = 0;
  for (auto i : lanemark.lanemarking_list){
    // if (i.first.size() < 4) 
    //   continue;
    lanemarkings_ego.push_back({});
    for (auto j : i.first){
      auto temp = car.get_transform_ego(j.x_f, j.y_f);
      lanemarkings_ego[idx].push_back(std::make_pair(temp[0], temp[1]));
    }
    idx++;
  }
  return lanemarkings_ego;
}