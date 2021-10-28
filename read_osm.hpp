#pragma once

#include <readosm.h>
#include <iostream>
#include <map>

class data_point{
    public:
    long long id;
    float lat;
    float lon;
    float version;
    std::vector<long long> relation;

    std::map<std::string, std::string> tags;

};

class way_point{
    public:
    long long id;
    float version;
    std::vector<long long> relation;
    std::vector<long long> points;

    std::map<std::string, std::string> tags;
};

class relation_member{
    public:
    int type;
    long long reference;
    std::string role;
};

class relations{
    public:
    long long id;
    float version;
    int member_count;
    std::vector<long long> relation_nb;

    std::vector<relation_member*> relation_members;
    std::map<std::string, std::string> tags;
};

struct mapSaver{
    std::map<long long, data_point*> pointSaver;
    std::map<long long, way_point*> waySaver;
    std::map<long long, relations*> relationSaver;
    std::vector<long long> parking_relation;
};