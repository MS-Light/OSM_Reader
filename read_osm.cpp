#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/highgui.hpp"

#include "helper.hpp"
#include "read_osm.hpp"

float LAT_B = 999;
float LNG_B = 999;

float LAT_M = -999;
float LNG_M = -999;


static int
parse_node (const void *user_data, const readosm_node * node)
{
    /* callback function consuming Node objects */
    struct mapSaver *my_struct = (struct mapSaver *) user_data;
    char buf[128];
    int i;
    const readosm_tag *tag;
    if (user_data != NULL) user_data = NULL;

    data_point* temp = new data_point();
    temp->id = node->id;
    if (node->latitude != READOSM_UNDEFINED) temp->lat = node->latitude;
    if (node->longitude != READOSM_UNDEFINED) temp->lon = node->longitude;
    if (node->version != READOSM_UNDEFINED) temp->version = node->version;
   
    if (node->tag_count != 0)
    {
        for (i = 0; i < node->tag_count; i++)
        {
            tag = node->tags + i;
            temp->tags.emplace(tag->key,tag->value);
        }
    }
    my_struct->pointSaver.emplace(node->id, temp);
    return READOSM_OK;
}
static int parse_way (const void *user_data, const readosm_way * way)
{
    /* callback function consuming Way objects */
    struct mapSaver *my_struct = (struct mapSaver *) user_data;
    int i;
    const readosm_tag *tag;
    way_point* temp = new way_point();
    temp->id = way->id;
    if (user_data != NULL) user_data = NULL;
    
    if (way->version != READOSM_UNDEFINED)temp->version = way->version;
    
    
    if (way->tag_count == 0 && way->node_ref_count == 0) {}
    else
    {
        for (i = 0; i < way->node_ref_count; i++)
        {   
            temp->points.push_back((long long)*(way->node_refs + i));//problem
        }

        for (i = 0; i < way->tag_count; i++)
        {
            tag = way->tags + i;
            temp->tags.emplace(tag->key,tag->value);
        }
    }
    my_struct->waySaver.emplace(way->id, temp);

    return READOSM_OK;
}


static int
parse_relation (const void *user_data, const readosm_relation * relation)
{
    /* callback function consuming Relation objects */
    struct mapSaver *my_struct = (struct mapSaver *) user_data;

        int i;
        const readosm_member *member;
        const readosm_tag *tag;

        relations* temp = new relations();
        temp->id = relation->id;
        temp->member_count = relation->member_count;
       
        if (relation->tag_count == 0 && relation->member_count == 0)
            {}
        else
        {
            for (i = 0; i < relation->member_count; i++)
                {
                    /* we'll now print each <member> for this way */
                    member = relation->members + i;
                    relation_member* mb_temp = new relation_member();
                    mb_temp->reference = member->id;
                    mb_temp->type = member->member_type;
                    // std::cout<<relation->id<<","<<member->id<<std::endl;
                    if (member->role != NULL) mb_temp->role = member->role;
                    temp->relation_members.push_back(mb_temp);
                    
                }
            for (i = 0; i < relation->tag_count; i++)
                {
                    /* we'll now print each <tag> for this way */
                    tag = relation->tags + i;
                    temp->tags.emplace(tag->key,tag->value);
                    if (temp->tags[tag->key] == "parking_access"){
                        my_struct->parking_relation.push_back((long long)relation->id);
                    }
                }
            my_struct->relationSaver.emplace(relation->id, temp);
        }

    //... some smart code ...
    return READOSM_OK;
}

int main(){
    int ret;
    const void *handle;
    mapSaver data_saved;

    ret = readosm_open("../test/vinpearl_nt_1009.osm", &handle);
    ret = readosm_parse (handle, &data_saved, parse_node, parse_way, parse_relation);
    ret = readosm_close(handle);

    for (auto& i : data_saved.relationSaver){

        for (auto j : i.second->relation_members)
        {
            /* we'll now print each <member> for this way */
            switch (j->type)
            {
            case READOSM_MEMBER_NODE:
                data_saved.pointSaver[j->reference]->relation.push_back((long long)(i.second->id));
                break;
            case READOSM_MEMBER_WAY:
                data_saved.waySaver[j->reference]->relation.push_back((long long)(i.second->id));
                break;
            case READOSM_MEMBER_RELATION:
                data_saved.relationSaver[j->reference]->relation_nb.push_back((long long)(i.second->id));
                break;
            default:
                
                break;
            };
        }
    }
    std::vector<long long> parking_area;
    for (auto i : data_saved.parking_relation){
        parking_area.push_back(i);
        auto& temp_relation = data_saved.relationSaver[i];
        std::string parking_spots = temp_relation->tags["parking_spots"];
        std::replace(parking_spots.begin(), parking_spots.end(), ',', ' ');
        std::stringstream ss;    
        ss << parking_spots;
        int found;
        while (!ss.eof()) {
            std::string temp;
            ss >> temp;
            std::cout<<temp<<std::endl;
            
            if (std::stringstream(temp) >> found){
                std::string::size_type sz;
                parking_area.push_back(std::stoi(temp,&sz));
            }
        }

    }
    for (long long park_relative : parking_area){
        for (auto& way_member : data_saved.relationSaver[park_relative]->relation_members){
            for(long long point_member : data_saved.waySaver[way_member->reference]->points){
             
                auto& point_data = data_saved.pointSaver[point_member];
    
                LAT_B = fmin(LAT_B, point_data->lat);
                LNG_B = fmin(LNG_B, point_data->lon);

                LAT_M = fmax(LAT_M, point_data->lat);
                LNG_M = fmax(LNG_M, point_data->lon);

            }
        }
    }

    int offsetMapVisualizer = 0;
    int key = 0;
    float angle = 180;
    std::cout<<LAT_B<<","<<LNG_B<<std::endl;
    std::cout<<LAT_M<<","<<LNG_M<<std::endl;
    bk::block car;
    car.set_block(50,50);
    car.set_map(LAT_B-0.000405, LNG_B-0.000285);
    car.set_angle(angle);
    car.set_offset(offsetMapVisualizer);
    car.set_carP(50, 50);
    car.set_block_point();
    car.set_scale(5);

    while(key!=27){
        cv::Mat mapVisualizer = cv::Mat(cv::Size(100,100), CV_8UC1, cv::Scalar(255));
        // if (car.get_lon()-LNG_B < 0.0025 || car.get_lon()-LNG_B > 0.0075 ||car.get_lat()-LAT_B < 0.0025 || car.get_lat()-LAT_B > 0.0075){
        //     LAT_B = car.get_lat()-0.005;
        //     LNG_B = car.get_lon()-0.005;
        //     car.update_map();
        //     car.set_block_point();
        // }
        cv::Mat mask;
        compare(mapVisualizer, cv::Scalar::all(235), mask, cv::CMP_GT);
        mapVisualizer.setTo(cv::Scalar::all(0), mask);
  
        //cv::circle(mapVisualizer, cv::Point2f(/*car.get_pixelLat(), car.get_pixelLon()*/50,50), 8.0, cv::Scalar(255), 2, 8);
        // for(auto& i : data_saved.pointSaver){
        //     auto data = i.second;
        //     auto temp = car.get_transform(data->lat, data->lon);
        //     cv::circle(mapVisualizer, cv::Point2f(temp[0], temp[1]), 1.0, cv::Scalar(0, 255, 0), 2, 8);
        // }

        int color_idx = 0;
        for (long long park_relative : parking_area){
            color_idx++;
            for (auto& way_member : data_saved.relationSaver[park_relative]->relation_members){
                bool first_round = true;
                Eigen::Vector3f temp;
                for(long long point_member : data_saved.waySaver[way_member->reference]->points){
                    auto point_data = data_saved.pointSaver[point_member];
                    auto temp2 = car.get_transform(point_data->lat, point_data->lon);
                    if(first_round){
                        first_round = false;
                        //cv::circle(mapVisualizer, cv::Point2f(temp2[0], temp2[1]), 1.0, cv::Scalar(0, 255, 0), 2, 8);
                    }else{
                        //cv::circle(mapVisualizer, cv::Point2f(temp2[0], temp2[1]), 1.0, cv::Scalar(0, 255, 0), 2, 8);
                        //cv::line(mapVisualizer, cv::Point2f(temp[0], temp[1]),cv::Point2f(temp2[0], temp2[1]), cv::Scalar(255- (color_idx%2)*255 , 255 - ((color_idx+1)%2)*255 , 0), 4,8);
                        cv::line(mapVisualizer, cv::Point2f(temp[0], temp[1]),cv::Point2f(temp2[0], temp2[1]), cv::Scalar(255), 4,8);

                    }
                    temp = temp2;
                    
                    
                    
                }
            }
        }



        cv::imshow("output", mapVisualizer);
        
        std::ofstream outputFile("costmap.csv");
        outputFile << format(mapVisualizer, cv::Formatter::FMT_CSV) << std::endl;
        outputFile.close();

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



    return 0;
}
