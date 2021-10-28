#pragma once

#include <eigen3/Eigen/Eigen>
#include <queue>
#include <vector>

constexpr double MY_PI = 3.1415926;

struct EgoConverter{
    int lane_id;
    int ego_id;
};



using namespace Eigen;
namespace bk{

    class block
    {
        public:
            int drop_count = 0;
            int visual_count = 0;
            //void addArc (TomTom::AutoStream::HdMap::TArc arc);
            void addTestPoints();
            void popArc ();

            /**
             * set the  length and width of the visual block.
             * @param len length of the block
             * @param wid width of the block
             */
            void set_block(float len, float wid);

            /**
             * set the car location in unit of Opencv pixel
             * @param lat latitude in pixel scale
             * @param lon longitude in pixel scale
             */
            void set_carP(float lat, float lon);

            /**
             * set the car location in unit of GPS
             * @param lat GPS latitude
             * @param lon GPS longitude
             */
            void set_carM(float lat, float lon);

            /**
             * set the pose of the car
             * @param ang the direction of the car towards
             */
            void set_angle(float ang);

            /**
             * set the scale of the map
             * @param size how many times we scale the map
             */
            void set_scale(float size);

            void set_block_point();

            /**
             * set the offset of the map
             * @param offset
             */
            void set_offset(int offset);

            /**
             * set the boundary of the map (left up corner)
             * @param latB latitude in GPS
             * @param lonB longitude in GPS
             */
            void set_map(float latB, float lonB);

            void set_route_car(double length);

            /**
             * add length to calculate total arc length
             * @param length the length of the arc added to the route
             */
            //bool add_arc_length(double length);

            void update_map();

            /**
             * check if the 2Dpoint is in the block
             * @param point a 2D point on the map
             * @param offset the offset of the map
             * @return true if the point is in block
             */
            bool checkInBox(Eigen::Vector2f point, int offset);

            /**
             * check if we need to add more arcs
             * @return return true if we need to parse more arcs
             */
            bool checkLength(double route_limit);
            bool check_add_arc_length(double length, double route_limit);

            /**
             * get lat/lon of the car
             */
            float get_lat();
            float get_lon();

            /**
             * get the pixel location of the car
             */
            float get_pixelLat();
            float get_pixelLon();

            /**
             * get the pose of the car
             */
            float get_angle();
            double get_scale();

            /**
             * get the distance from a point to the center of the car
             * @param lat latitude in GPS
             * @param lon longitude in GPS
             */
            double get_distance(double lat, double lon);

            /**
             * get the distance from a point to the center of the car
             * @param lat_1 latitude in GPS
             * @param lon_1 longitude in GPS
             * @param lat_2 latitude in GPS
             * @param lon_2 longitude in GPS
             */
            double get_distance_2p(double lat1, double lon1, double lat2, double lon2);
            
            int get_offsetVisualizer();
            std::vector<Vector3f> get_vectorBox();

            /**
             * transform GPS location to OpenCV pixel location
             * @param lat latitude in GPS
             * @param lon longitude in GPS
             * @output return the transformed location on OpenCV mapvisualizer. {lat_pixel, lon_pixel, 1}
             */
            Vector3f get_transform(float lat, float lon);
            
            /**
             * transform GPS location to ego frame format
             * @param lat latitude in GPS
             * @param lon longitude in GPS
             * @output return the transformed location for ego {lat_pixel, lon_pixel, 1}
             */
            Vector3f get_transform_ego(float lat, float lon);

            //void convert_to_ego(LaneMarkings& lanemark, std::list<EgoConverter>& lane_ego_transform);

            double get_route_length();
            double get_route_car();

        private:
            Matrix3f get_model_matrix(float rotation_angle);
            Matrix3f get_view_matrix(Eigen::Vector3f pos);
            Matrix3f get_scale_matrix(float size);

        private:
            //std::vector<TomTom::AutoStream::HdMap::TArc> arcInBlock;
            std::vector<Vector3f> queueA;
            std::vector<Eigen::Vector3f> vectorBox;


            Vector3f carP;
            Vector3f carM;
            Vector3f backLeft;
            Vector3f backRight;
            Vector3f frontLeft;
            Vector3f frontRight;

            
            float angle;
            float latB;
            float lonB;
            double route_length;
            double route_car;
            double scale;

            int offsetMapVisualizer;

            Matrix3f view_matrix;
            Matrix3f model_matrix;
            Matrix3f p_model_matrix;
            Matrix3f ego_model_matrix;
            Matrix3f scale_matrix;

            Eigen::Vector3f pointA;
            Eigen::Vector3f pointB;
            Eigen::Vector3f pointC;
            Eigen::Vector3f pointD;


    };
}