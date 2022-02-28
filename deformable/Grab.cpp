#include <glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include "Grab.h"

using namespace glm;
using namespace std;

Grab::Grab(GLFWwindow* window) : window(window) {
    horizontalOffset = 0.0f;
    verticalOffset = 0.0f;
    mouseSpeed = 0.001f;
    deltaTime = 0.1f;
}

void Grab::update() {
    // glfwGetTime is called only once, the first time this function is called
    static double lastTime = glfwGetTime();

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    // Compute time difference between current and last frame
    double currentTime = glfwGetTime();
    deltaTime = float(currentTime - lastTime);

    // Get mouse position
    double xPos, yPos;
    glfwGetCursorPos(window, &xPos, &yPos);

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Reset mouse position for next frame
    glfwSetCursorPos(window, width / 2, height / 2);

    horizontalOffset = mouseSpeed * float(width / 2 - xPos);
    verticalOffset = mouseSpeed * float(height / 2 - yPos);

    lastTime = currentTime;
}