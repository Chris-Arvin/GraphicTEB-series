#ifndef _UTILIZES_GTEB_H_
#define _UTILIZES_GTEB_H_

struct Point2D{
    Point2D(int x=0, int y=0, int value=0, int id=0){
        this->x = x;
        this->y = y;
        this->value = value;
        this->id = id;
    }
    int x;
    int y;
    int value;
    int id;
};

#endif