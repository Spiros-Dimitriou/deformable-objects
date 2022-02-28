#include "Collision.h"
using namespace glm;

void handleTopCollision(RigidBody& point, float top);
bool checkTopStep(RigidBody& point, float top, float side);

bool checkSideStep(RigidBody& point, float top, float side);
void handleSideCollision(RigidBody& point, float side);


void checkStairCollision(RigidBody &point) {

	if (checkSideStep(point, -1.0f, 0.25f))
	{
		handleSideCollision(point, 0.25);
	}

	else if (checkTopStep(point, -1.0f, 0.25f)) {
		handleTopCollision(point, -1.0f);
	}

	if (checkSideStep(point, -1.5f, 1.25f))
	{
		handleSideCollision(point, 1.25);
	}

	else if (checkTopStep(point, -1.5f, 1.25f)) {
		handleTopCollision(point, -1.5f);
	}

	if (checkSideStep(point, -2.0f, 2.25f))
	{
		handleSideCollision(point, 2.25);
	}

	else if (checkTopStep(point, -2.0f, 2.25f)) {
		handleTopCollision(point, -2.0f);
	}

	else if (checkTopStep(point, -2.5f, 15.25f)) {
		handleTopCollision(point, -2.5f);
	}
}

bool checkTopStep(RigidBody& point, float top, float side) {
	if (point.x.y < top && point.x.x < side)
		return true;
	return false;
}

void handleTopCollision(RigidBody& point, float top) {
	point.x.y = top;
	point.v = vec3(point.v.x, 0, point.v.z);
	point.P = point.m * point.v;
}

bool checkSideStep(RigidBody& point, float top, float side) {
	if (point.x.y < top - 0.02f && point.x.x < side && point.x.x > side - 0.02f)
		return true;
	return false;
}

void handleSideCollision(RigidBody& point, float side) {
	point.x.x = side;
	point.v = vec3(0, point.v.y, point.v.z);
	point.P = point.m * point.v;
}
