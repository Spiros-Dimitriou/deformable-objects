#include "RigidBody.h"
#include <vector>
#include <functional>
#include <map>

using namespace glm;

RigidBody::RigidBody() {
    m = 1;
    x = v = P = vec3(0, 0, 0);
}

RigidBody::~RigidBody() {
}

std::vector<float> RigidBody::getY() {
    std::vector<float> state(STATES);
    int k = 0;

    state[k++] = x.x;
    state[k++] = x.y;
    state[k++] = x.z;

    state[k++] = P.x;
    state[k++] = P.y;
    state[k++] = P.z;

    return state;
}

void RigidBody::setY(const std::vector<float>& y) {
    int k = 0;
    x.x = y[k++];
    x.y = y[k++];
    x.z = y[k++];

    P.x = y[k++];
    P.y = y[k++];
    P.z = y[k++];

    // momentum
    v = P / m;
}

std::vector<float> RigidBody::dydt(float t, const std::vector<float>& y) {
    // store initial state and override state
    std::vector<float> y0 = getY();
    setY(y);

    std::vector<float> yDot(STATES);
    int k = 0;

    //x_dot = u
    yDot[k++] = v.x;
    yDot[k++] = v.y;
    yDot[k++] = v.z;

    auto f = forcing(t, y);
    //P_dot = f
    yDot[k++] = f[0];
    yDot[k++] = f[1];
    yDot[k++] = f[2];

    // restore initial state
    setY(y0);

    return yDot;
}

std::vector<float> RigidBody::euler(float t, float h, const std::vector<float>& y0) {
    std::vector<float> y1(STATES);
    std::vector<float> dydt0 = dydt(t, y0);
    for (int i = 0; i < STATES; i++) {
        y1[i] = y0[i] + h * dydt0[i];
    }
    return y1;
}

std::vector<float> RigidBody::rungeKuta4th(float t, float h, const std::vector<float>& y0) {
    // dydt0 = dydt(x0)
    std::vector<float> dydt0 = dydt(t, y0);

    // y1 = y0 + h * dydt0 / 2
    std::vector<float> y1(STATES);
    for (int i = 0; i < STATES; i++) {
        y1[i] = y0[i] + h * dydt0[i] / 2.0f;
    }
    std::vector<float> dydt1 = dydt(t + h / 2.0f, y1);

    // y2 = y0 + h * dydt1 / 2
    std::vector<float> y2(STATES);
    for (int i = 0; i < STATES; i++) {
        y2[i] = y0[i] + h * dydt1[i] / 2.0f;
    }
    std::vector<float> dydt2 = dydt(t + h / 2.0f, y2);

    // y2 = y0 + h * dydt1 / 2
    std::vector<float> y3(STATES);
    for (int i = 0; i < STATES; i++) {
        y3[i] = y0[i] + h * dydt2[i];
    }
    std::vector<float> dydt3 = dydt(t + h, y3);

    // combine them to estimate the solution.
    std::vector<float> y4(STATES);
    for (int i = 0; i < STATES; i++) {
        y4[i] = y0[i] + h * (dydt0[i] + 2.0f * dydt1[i]
                             + 2.0f * dydt2[i] + dydt3[i]) / 6.0f;
    }
    return y4;
}

void RigidBody::advanceState(float t, float h) {
    //setY(euler(t, h, getY()));
    setY(rungeKuta4th(t, h, getY()));
}