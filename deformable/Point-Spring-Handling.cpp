#include "Point-Spring-Handling.h"
#include <vector>
#include <functional>
#include <map>
using namespace std;
using namespace glm;


void recalculatePointForces(std::vector<RigidBody> &points, std::vector<std::vector<float>> restingDist, int pointIndex, float dampFactor, float kFactor) {
    points[pointIndex].forcing = [&, restingDist, pointIndex](float t, const vector<float>& y)->vector<float> {
        vector<float> f(3, 0.0f);
        for (int i = 0; i < points.size(); i++)
        {
            if (pointIndex == i)
                continue;
            vec3 dist = points[pointIndex].x - points[i].x;
            float power = 100.0f * kFactor * (length(dist) - restingDist[pointIndex][i]);
            vec3 springForce = normalize(dist) * (-power);
            vec3 damp = normalize(dist) * dot(points[pointIndex].v, dist) * dampFactor;
            //vec3 damp = points[pointIndex].v * dampFactor;
            f[0] += springForce.x - damp.x;
            f[1] += springForce.y - damp.y;
            f[2] += springForce.z - damp.z;
        }
        //f[0] += 0.5;
        f[1] -= points[pointIndex].m * gravity;
        return f;
    };
}

void ffdRecalculatePointForces(vector<RigidBody>& points, vector<vec3> restingDist, int pointIndex, float dampFactor, float kFactor) {
    points[pointIndex].forcing = [&, restingDist, pointIndex](float t, const vector<float>& y)->vector<float> {
        vector<float> f(3, 0.0f);
        vec3 dist = points[pointIndex].x - restingDist[pointIndex];
        float power = 100.0f * kFactor * length(dist);
        vec3 springForce = normalize(dist) * (-power);
        //vec3 damp = normalize(dist) * dot(points[pointIndex].v, dist) * dampFactor;
        vec3 damp = points[pointIndex].v * dampFactor;
        f[0] += springForce.x - damp.x;
        f[1] += springForce.y - damp.y;
        f[2] += springForce.z - damp.z;
        return f;
    };
}