#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <vector>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

class RigidBody {
public:
    static const int STATES = 6;
    // m: mass
    float m;
    // x: position, v: velocity, P: momentum
    glm::vec3 x, v, P;
    // set the forces
    std::function<std::vector<float>(float t, const std::vector<float>& y)> forcing =
        [](float t, const std::vector<float>& y)->std::vector<float> {
        std::vector<float> f(3, 0.0f);
        return f;
    };

    RigidBody();
    ~RigidBody();
    /** Get state vector y */
    std::vector<float> getY();
    /** Get state vector y */
    void setY(const std::vector<float>& y);
    /** Get state derivative vector dy / dt */
    std::vector<float> dydt(float t, const std::vector<float>& y);
    /** Euler method for advancing the state y(t + h) = y(t) + h dy(t) / dt */
    std::vector<float> euler(float t, float h, const std::vector<float>& y0);
    /** Runge-Kutta 4th order for advancing the state (error/step ~ O(h^5) */
    std::vector<float> rungeKuta4th(float t, float h, const std::vector<float>& y0);
    /** Advances the state from t to t + h using Euler or RunkeKutta */
    void advanceState(float t, float h);
};

#endif
