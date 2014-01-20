#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <vector>
#include <stdint.h>

struct Command {
  float steering;
  float acceleration;
  float brake;
  int gear;
};

struct Status {
  float rpm;
  int gear;
  float gearRatio;
  float lowerGearRatio;
  float maxRPM;
  float wheelRadius;
  float trackYaw;
  float trackDistance;
  float trackCurvature;
  float trackWidth;
  float nextCurvature;
  float nextDistance;
  float speed;
  float yaw;
  float x;
  float y;
};

struct Obstacle {
  uint8_t id;
  float x;
  float y;
  float theta;
  float vX;
  float vY;
  float width;
  float height;
};

struct Buffer {
  Command command;
  Status status;
  uint8_t nObstacles;
  Obstacle obstacles[100];
};


typedef std::vector<Obstacle> Obstacles;


#endif //STRUCTS_H_

