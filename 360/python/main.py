import cv2
from vector3 import *
from camera import *
from math import pi, acos, atan2, floor, ceil, sqrt
import numpy as np

def spherical_map(pos_3d):
    pos_3d.normalize()
    phi = acos(pos_3d.y)
    rho = atan2(pos_3d.z, pos_3d.x) + pi
    UV = (rho / (2 * pi), phi / pi)
    return UV

def getPixelAt(img, width, height, x, y):
    x %= width
    y %= height
    if x < 0:
        x += width
    if y < 0:
        y += height
    color = img[y][x]
    return color


def interpolation(img, width, height, UV):
    # bilinear interpolation
    # adjacent pixel coordinates
    left = floor(UV[0] * width)
    right = ceil(UV[0] * width)
    top = floor(UV[1] * height)
    bottom = ceil(UV[1] * height)

    # weights
    w = [0] * 4
    w[0] = right - UV[0] * width
    w[1] = 1 - w[0]
    w[2] = bottom - UV[1] * height
    w[3] = 1 - w[2]

    # get color values and interpolate
    val = [0] * 4
    val[0] = getPixelAt(img, width, height, left, top)
    val[1] = getPixelAt(img, width, height, right, top)
    val[2] = getPixelAt(img, width, height, left, bottom)
    val[3] = getPixelAt(img, width, height, right, bottom)
    result_color = w[2] * w[0] * val[0] + w[2] * w[1] * val[1] + w[3] * w[0] * val[2] + w[3] * w[1] * val[3]
    return result_color

def interaction(ray_pos, ray_dir):
    a = 1.0
    b = 2.0 * dotProduct(ray_dir, ray_pos)
    c = dotProduct(ray_pos, ray_pos) - 1
    discriminant = b * b - 4 * a * c

    if (discriminant <= 0):
        print("camera position out of range!")
        return Vector3(0, 0, 0)

    root = sqrt(discriminant)

    if(b < 0):
        q = -0.5 * (b - root)
    else:
        q = -0.5 * (b + root)

    t0 = q / a
    t1 = c / q
    t = min(t0, t1)
    if (t < 0.000001):
        t = max(t0, t1)
    hitPoint_on_sphere = ray_pos + ray_dir * t
    return hitPoint_on_sphere

#def callback():


if __name__ == '__main__':
    file_name = "data/DJI_0097.JPG"
    img = cv2.imread(file_name)
    img_dir_h = -1
    img_dir_w = -1
    height = img.shape[0]
    width = img.shape[1]   
    camera_center = Vector3(0, 0.8, 0)
    camera_up = Vector3(0, 0, 1)
    camera_forward = Vector3(0, -1.0, 0)
    camera_right = Vector3(1, 0, 0)
    camera_fov = 120
    maincamera = camera(camera_center, camera_up, camera_forward, camera_right)
    maincamera.setFovAngle(camera_fov)

    result_img_height = 512
    result_img_width = 512
    result_img = np.zeros((result_img_height, result_img_width, 3), np.uint8)
    aspectRatio = result_img_height / result_img_width

#--------------
    for h in range(result_img_height):
        for w in range(result_img_width):
            maincamera.setRay(img_dir_w * (w / result_img_width * 2 - 1), img_dir_h * (h / result_img_height * 2 - 1) * aspectRatio)
            pos_3d = interaction(maincamera.raypos, maincamera.raydir)
            UV = spherical_map(pos_3d)
            result_img[h][w] = interpolation(img, width, height, UV)

    cv2.imshow("result", result_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("OK!")


#--------------
    # cv2.namedWindow("result")
    # cv2.createTrackbar('camera_Y', 'result', 0, 100, lambda x:x)
    # cv2.createTrackbar('fov', 'result', 0, 90, lambda x:x)
    # while True:
    #     camera_Y = cv2.getTrackbarPos('camera_Y','result')
    #     fov = cv2.getTrackbarPos('fov','result')
    #     maincamera.center.y = camera_Y / 100
    #     maincamera.setFovAngle(fov + 90)
    #     for h in range(result_img_height):
    #         for w in range(result_img_width):
    #             maincamera.setRay(img_dir_w * (w / result_img_width * 2 - 1), img_dir_h * (h / result_img_height * 2 - 1) * aspectRatio)
    #             pos_3d = interaction(maincamera.raypos, maincamera.raydir)
    #             UV = spherical_map(pos_3d)
    #             result_img[h][w] = interpolation(img, width, height, UV)

    #     cv2.imshow("result", result_img)
    #     key=cv2.waitKey(1)
    #     if key==27:
    #         break
    # cv2.destroyAllWindows()
    # print("OK!")