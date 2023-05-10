#include "camera.h"

Camera::Camera() : origin(0, 0, 0), up(0, 0, 1), forward(0, -1, 0), right(1, 0, 0)
{
	setFovAngle(90);
}

void Camera::setPosition(Vector3d position)
{
	this->origin = position;
}

void Camera::setForwardDirection(Vector3d forwardDirection)
{
	// Set up a left-handed coordinate system,
	// in which the camera looks along the positive z-Axis
	std::tie(this->forward, this->up, this->right) = orthoNormalized(forwardDirection, this->up, crossProduct(this->up, forwardDirection));
}

void Camera::setUpDirection(Vector3d upDirection)
{
	// Set up a left-handed coordinate system,
	// in which the camera looks along the positive z-Axis
	std::tie(this->forward, this->up, this->right) = orthoNormalized(this->forward, upDirection, crossProduct(upDirection, this->forward));
}

void Camera::setFovAngle(float fovAngle)
{
	// Calculate the focus
	this->focus = 1.0f / std::tan((fovAngle * PI / 180) / 2.0f);
}

void Camera::setRay(float x, float y) {
	this->raypos = this->origin;
	this->raydir = normalized(this->right * x + this->up * y + this->forward * this->focus);
}