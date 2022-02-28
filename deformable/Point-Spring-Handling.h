#pragma once
#include "RigidBody.h"
#include <vector>
#include <functional>
#include <map>

using namespace std;
using namespace glm;

#define gravity 9.80665f

void recalculatePointForces(std::vector<RigidBody> &points, std::vector<std::vector<float>> restingDist, int pointIndex, float dampFactor, float kFactor);

void ffdRecalculatePointForces(vector<RigidBody>& points, vector<vec3> restingDist, int pointIndex, float dampFactor, float kFactor);