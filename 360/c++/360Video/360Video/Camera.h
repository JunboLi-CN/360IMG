#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include "vector3d.h"
#include "common.h"

class Camera
{
public:
	// Constructor
	Camera();

	// Set
	void setPosition(Vector3d position);
	void setForwardDirection(Vector3d forwardDirection);
	void setUpDirection(Vector3d upDirection);
	void setFovAngle(float fovAngle);
	void setRay(float x, float y);

	// Get
	Vector3d getRayPos() { return this->raypos; }
	Vector3d getRayDir() { return this->raydir; }

private:
	// Camera
	Vector3d origin;
	Vector3d up;
	Vector3d forward;
	Vector3d right;

	// Ray
	Vector3d raypos;
	Vector3d raydir;

	// Focus
	float focus;
};

#endif