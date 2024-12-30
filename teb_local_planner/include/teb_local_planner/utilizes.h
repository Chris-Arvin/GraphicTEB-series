#ifndef _UTILIZES_GTEB_H_
#define _UTILIZES_GTEB_H_

struct Point2D{
    Point2D(int x=0, int y=0, int value=0, int id=0, bool is_fill_corner=false, bool can_connect_start=false, bool can_connect_goal=false){
        this->x = x;
        this->y = y;
        this->value = value;
        this->id = id;
        this->is_fill_corner = is_fill_corner;
        this->can_connect_start = can_connect_start;
        this->can_connect_goal = can_connect_goal;
        this->goal_point.first = -1;
        this->goal_point.second = -1;
        this->goalID = -1;
        this->is_temp = false;
    }
    void setFill(){
        this->is_fill_corner = true;
    }
    void setStartConnection(){
        this->can_connect_start = true;
    }
    void setGoalConnection(std::pair<int,int> goal_point, int ID){
        this->can_connect_goal = true;
        this->goal_point.first = goal_point.first;
        this->goal_point.second = goal_point.second;
        this->goalID = ID;
    }
    int x;
    int y;
    int value;
    int id;
    bool is_fill_corner;
    bool can_connect_start;
    bool can_connect_goal;
    int goalID;
    std::pair<int, int> goal_point;
    std::vector<std::pair<int,int>> candidate_connection;

    bool is_temp;
};


struct Point2D_float{
    Point2D_float(float x=0, float y=0){
        this->x = x;
        this->y = y;
        this->is_shorten_key_point = false;
    }
    void setShorten(){
        this->is_shorten_key_point = true;
    }

    float x;
    float y;
    bool is_shorten_key_point;
};

#endif