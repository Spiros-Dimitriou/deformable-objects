#pragma once
#include <glm/glm.hpp>
#include <vector>

class Grab {
public:
    GLFWwindow* window;
    Grab(GLFWwindow* window);
    void update();
    float mouseSpeed;
    float horizontalOffset;
    float verticalOffset;
    float deltaTime;
};

