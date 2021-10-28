#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include "helper.hpp"

#define EARTH_R	6372797.56085			// mean earth radius in meter
#define deg2rad(angleDegrees) ((angleDegrees) * MY_PI / 180.0)
#define rad2deg(rad) (rad) * 180.0 / MY_PI

double lon2dist(const double lon1, const double lon2,const double lat) // in meter
{
    return 2.0*asin(cos(deg2rad(lat))*(sin((deg2rad(lon1-lon2))/2.0f)) )*EARTH_R;
}
double lat2dist(const double lat1, const double lat2) // in meter
{
    return ((deg2rad(lat1-lat2))*EARTH_R);
}

double dist2lat(const double dist){
    return rad2deg(dist/EARTH_R);
}

double dist2lon(const double dist, const double lat){
    return rad2deg(2.0f * asin(dist/EARTH_R/2/asin(cos(deg2rad(lat)))));
}
// bool compareEgo(const std::pair<LaneMarkings::single_lanemark_s,LaneMarkings::lane_index> &a, const std::pair<LaneMarkings::single_lanemark_s,LaneMarkings::lane_index> &b)
// {
//     return (a.first.y_f < b.first.y_f);
// }

void bk::block::set_block(float len, float wid){
    if(len <1 || wid <1) return;
    backLeft = Eigen::Vector3f(wid,-len,1);
    backRight = Eigen::Vector3f(-wid,-len,1);
    frontLeft = Eigen::Vector3f(wid,len,1);
    frontRight = Eigen::Vector3f(-wid,len,1);
}

void bk::block::set_carP(float lat, float lon){
    carP = Eigen::Vector3f(lat, lon, 1);
    carM = Eigen::Vector3f(/*lat/111235*/ dist2lat(lat)+latB, /*(lon-offsetMapVisualizer)/87642*/ dist2lon((double)lon, (double)(dist2lat(lat)+latB)) + lonB, 1);
    view_matrix = get_view_matrix(carP);
}

void bk::block::set_carM(float lat, float lon){
    carP = Eigen::Vector3f(lat2dist(lat, latB), lon2dist(lon, lonB, lat), 1);
    carM = Eigen::Vector3f(lat, lon, 1);
    view_matrix = get_view_matrix(carP);
}

void bk::block::set_angle(float ang){
    angle = ang;
    model_matrix = get_model_matrix(angle);
    p_model_matrix = get_model_matrix(180-angle);
    ego_model_matrix = get_model_matrix(angle);
}

void bk::block::set_scale(float size){
    scale = size;
    scale_matrix = get_scale_matrix(size);
}

void bk::block::set_offset(int offset){
    offsetMapVisualizer = offset;
}

void bk::block::set_map(float x, float y){
    latB = x;
    lonB = y;
}

void bk::block::set_route_car(double length){
    route_car = length;
}

// bool bk::block::add_arc_length(double length){
//     route_length += length;
//     return checkLength();
// }



void bk::block::update_map(){
    latB = carM[0]-0.005;
    lonB = carM[1]-0.005;
    set_carM(carM[0], carM[1]);
}

Eigen::Matrix3f bk::block::get_view_matrix(Eigen::Vector3f pos)
{
    Eigen::Matrix3f view = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f translate;
    translate << 1, 0, pos[0],
                 0, 1, pos[1],
                 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix3f bk::block::get_model_matrix(float rotation_angle)
{
    Eigen::Matrix3f model = Eigen::Matrix3f::Identity();

    float ang = rotation_angle/180*MY_PI;
    Eigen::Matrix3f rotate;
    rotate << cos(ang), -sin(ang),0,
              sin(ang), cos(ang), 0,
                0, 0, 1;

    
    model = rotate * model;


    return model;
}

Eigen::Matrix3f bk::block::get_scale_matrix(float size){
    Eigen::Matrix3f model = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f scale;
    scale << size, 0, 0,
              0, size, 0,
              0, 0,    1;

    
    model = scale * model;

    return model;
}

bool bk::block::checkInBox(Eigen::Vector2f pointTemp, int offset){
    Eigen::Vector3f point ({(pointTemp[0]-latB)*111235, offset + (pointTemp[1]-lonB)*87642, 1});
    Eigen::Vector3f back = vectorBox.back();
    bool mark;
    bool start = false;

    for (Eigen::Vector3f i : vectorBox){
        Eigen::Vector3f vectorA = i - back;
        Eigen::Vector3f vectorB = point - back;

        if (start == false){
            start = true;
            mark = ((vectorA.cross(vectorB))[2]>0);
        }

        if ((vectorA.cross(vectorB))[2]>0){
            if (mark == false)
                return false;
        }
        else{
            if (mark == true)
                return false;
        }
        back = i;
    }
    return true;
}

bool bk::block::checkLength(double route_limit){
    return (route_length-route_car) < route_limit;
}
bool bk::block::check_add_arc_length(double length, double route_limit){
    if ((route_length-route_car+length) < route_limit){
        route_length+=length;
        return true;
    }else{
        return false;
    }
}


void bk::block::addTestPoints(){
    queueA.clear();
    queueA.push_back({100,100,1});
    queueA.push_back({200,200,1});
    queueA.push_back({300,300,1});
    queueA.push_back({150,170,1});
    queueA.push_back({290,600,1});
    queueA.push_back({300,500,1});
    queueA.push_back({600,250,1});
    queueA.push_back({100,110,1});
    queueA.push_back({150,300,1});
}

// void bk::block::addArc(TomTom::AutoStream::HdMap::TArc arc){
//     arcInBlock.push_back(arc);
// }

void bk::block::set_block_point(){
    Eigen::Matrix3f mv = view_matrix * model_matrix;

    pointA = mv * backLeft;
    pointB = mv * backRight;
    pointC = mv * frontLeft;
    pointD = mv * frontRight;
    vectorBox.clear();

    vectorBox.push_back(pointA);
    vectorBox.push_back(pointB);
    vectorBox.push_back(pointD);
    vectorBox.push_back(pointC);
}

std::vector<Eigen::Vector3f> bk::block::get_vectorBox(){
    return vectorBox;
}



float bk::block::get_pixelLat(){
    return carP[0];
}
float bk::block::get_pixelLon(){
    return carP[1];
}

float bk::block::get_lat(){
    return carM[0];
}
float bk::block::get_lon(){
    return carM[1];
}
float bk::block::get_angle(){
    return angle;
}


Eigen::Vector3f bk::block::get_transform(float lat, float lon){
    Eigen::Vector3f point = Eigen::Vector3f(lat2dist(lat, carM[0]), lon2dist(lon, carM[1], carM[0]), 1);
    Eigen::Vector3f val = get_view_matrix(Eigen::Vector3f({500,500,1}))* scale_matrix * p_model_matrix * point;
    return val;
}
Eigen::Vector3f bk::block::get_transform_ego(float lat, float lon){
    return ego_model_matrix * Eigen::Vector3f(lon2dist(lon, carM[1], carM[0]), lat2dist(lat, carM[0]), 1);
}

double bk::block::get_distance(double lat, double lon){
    return get_distance_2p(carM[0], carM[1], lat, lon);
}
double bk::block:: get_distance_2p(double lat1d, double lon1d, double lat2d, double lon2d){
    double lat1r = deg2rad(lat1d);
    double lon1r = deg2rad(lon1d);
    double lat2r = deg2rad(lat2d);
    double lon2r = deg2rad(lon2d);
    double haversine = (pow(sin((1.0 / 2) * (lat2r - lat1r)), 2)) + ((cos(lat1r)) * (cos(lat2r)) * (pow(sin((1.0 / 2) * (lon2r - lon1r)), 2)));
    return EARTH_R * 2 * asin(std::min(1.0, sqrt(haversine)));;
}

int bk::block::get_offsetVisualizer(){
    return offsetMapVisualizer;
}


double bk::block::get_route_length(){
    return route_length;
}

double bk::block::get_route_car(){
    return route_car;
}
double bk::block::get_scale(){
    return scale;
}

// void bk::block::convert_to_ego(LaneMarkings& lanemark, std::list<EgoConverter>& lane_ego_transform){
//     std::vector<std::pair<LaneMarkings::single_lanemark_s, LaneMarkings::lane_index>> ego_trans_points;
//       for (auto i : lanemark.lanemarking_list){
//         Eigen::Vector3f temp_xf = {9999,0,1};
//         for (auto j : i.first){
//           auto ego_point = get_transform_ego(j.x_f, j.y_f);
//           if (ego_point[0]>0){
//             temp_xf = ego_point;
//           }else{
//             if (abs(temp_xf[0]) < abs(ego_point[0])){
//               if(temp_xf[0] > 50){
//                 temp_xf = {(temp_xf[0]+ego_point[0])/2, (temp_xf[1]+ego_point[1])/2, 1};
//               }
//               j.x_f = temp_xf[0];
//               j.y_f = temp_xf[1];
//               ego_trans_points.emplace_back(j,i.second);
//               break;
//             }else{
//               if(abs(ego_point[0]) > 50){
                
//                 ego_point = {(temp_xf[0]+ego_point[0])/2, (temp_xf[1]+ego_point[1])/2, 1};
//               }
//               j.x_f = ego_point[0];
//               j.y_f = ego_point[1];
//               ego_trans_points.emplace_back(j,i.second);
//               break;
//             }
//           }
          
//         }
//       }
//       std::sort(ego_trans_points.begin(), ego_trans_points.end(), compareEgo);
//       int count_right = 0;
      
//       for (auto i : ego_trans_points){
        
//         if (i.first.y_f<0){
//           count_right += 1;
//         }else{
//           count_right += 1;
//           break;
//         }
//       }
//       int num_count = 1;
//       double last_y = 0;
//       int prevego;
//       bool notFirst = false;
//       for (auto i : ego_trans_points){
        
//         int ego_id = (count_right-num_count)*2-1;
//         if (ego_id<0){
//           ego_id = -ego_id-1;
//         }
//         if (i.first.y_f == last_y && notFirst){
//           ego_id = prevego;
//         }
//         if (!notFirst) notFirst = true;

//         lane_ego_transform.emplace_back<EgoConverter>({.lane_id = i.second.lane_count, .ego_id = ego_id});
//         num_count+=1;
//         last_y = i.first.y_f;
//         prevego = ego_id;
//       }
// }