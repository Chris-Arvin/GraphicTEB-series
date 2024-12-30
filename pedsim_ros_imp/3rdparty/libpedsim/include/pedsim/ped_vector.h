//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2012 by Christian Gloor
//

#ifndef _ped_vector_h_
#define _ped_vector_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <cmath>
#include "ped_angle.h"
#include <iostream>

namespace Ped {
/// Vector helper class. This is basically a struct with some related functions
/// attached.
/// x, y, and z are public, so that they can be accessed easily.
/// \author  chgloor
/// \date    2010-02-12
class LIBEXPORT Tvector {
 public:
  // Default constructor
  Tvector();

  // Initializing constructor
  Tvector(double px, double py, double pz = 0) : x(px), y(py), z(pz){};
  Tvector(double px, double py, double yaw, double vx, double vy) : x(px), y(py), yaw(yaw), vx(vx), vy(vy){};
  Tvector(double px, double py, double pz, double pw): x(px), y(py),z(pz),yaw(pw){};
  // Static Methods
  static Tvector fromPolar(const Tangle& angleIn, double radiusIn = 1);

  // Methods
  bool isValid() const;
  double length() const;
  double lengthSquared() const;
  void normalize();
  Tvector normalized() const;
  void scale(double factor);
  Tvector scaled(double factor) const;

  Tvector leftNormalVector() const;
  Tvector rightNormalVector() const;

  double polarRadius() const;
  Tangle polarAngle() const;
  Tvector rotateTo(double angle);

  Tangle angleTo(const Tvector& other) const;

  static double scalar(const Tvector& a, const Tvector& b);
  static double dotProduct(const Tvector& a, const Tvector& b);
  static Tvector crossProduct(const Tvector& a, const Tvector& b);

  // Operators
  Tvector operator+(const Tvector& other) const;
  Tvector operator-(const Tvector& other) const;
  Tvector operator*(double factor) const;
  Tvector operator/(double divisor) const;
  Tvector& operator+=(const Tvector& vectorIn);
  Tvector& operator-=(const Tvector& vectorIn);
  Tvector& operator*=(double factor);
  Tvector& operator*=(const Tvector& vectorIn);
  Tvector& operator/=(double divisor);

  // Attributes
  double x;
  double y;
  double z;
  double yaw;
  double vx;
  double vy;
};
}

bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
Ped::Tvector operator-(const Ped::Tvector& vectorIn);
Ped::Tvector operator*(double factor, const Ped::Tvector& vector);

std::ostream &operator<<(std::ostream &output, const Ped::Tvector& vectorIn);
#endif
