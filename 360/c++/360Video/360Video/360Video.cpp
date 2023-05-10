#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>

#include "vector3d.h"
#include "camera.h"

using namespace std;
using namespace cv;

Vec2f spherical_map(Vector3d pos_3d)
{
    normalized(pos_3d);
    float phi = acos(pos_3d.y);
    float rho = atan2(pos_3d.z, pos_3d.x) + PI;
    Vec2f UV = Vec2f(rho / (2 * PI), phi / PI);
    return UV;
}

Vector3d interaction(Vector3d ray_pos, Vector3d ray_dir)
{
    float a = 1.0;
    float b = 2.0 * dotProduct(ray_dir, ray_pos);
    float c = dotProduct(ray_pos, ray_pos) - 1;
    float discriminant = b * b - 4 * a * c;

    if (discriminant <= 0)
    {
        cout << "camera position out of range!" << endl;
        return Vector3d(0, 0, 0);
    }

    float root = sqrt(discriminant);
    float q = -0.5f * (b < 0 ? (b - root) : (b + root));
    float t0 = q / a;
    float t1 = c / q;
    float t = min(t0, t1);
    if (t < EPSILON)
        t = max(t0, t1);
    Vector3d hitPoint_on_sphere = ray_pos + ray_dir * t;
    return hitPoint_on_sphere;
}

Vec3f getPixelAt(Mat img, int width, int height, int x, int y) 
{
    if ((x %= width) < 0)
        x += width;
    if ((y %= height) < 0)
        y += height;
    return img.at<Vec3f>(y, x);
}

Vec3f interpolation(Mat img, int width, int height, float u, float v) {
    // bilinear interpolation
    // adjacent pixel coordinates
    int left = int(floorf(u * width));
    int right = int(ceilf(u * width));
    int top = int(floorf(v * height));
    int bottom = int(ceilf(v * height));

    // weights
    float w[4];
    w[0] = right - u * width;
    w[1] = 1 - w[0];
    w[2] = bottom - v * height;
    w[3] = 1 - w[2];
    
    // get color values and interpolate
    Vec3f val[4];
    val[0] = getPixelAt(img, width, height, left, top);
    val[1] = getPixelAt(img, width, height, right, top);
    val[2] = getPixelAt(img, width, height, left, bottom);
    val[3] = getPixelAt(img, width, height, right, bottom);
    Vec3f color = w[2] * w[0] * val[0] + w[2] * w[1] * val[1] + w[3] * w[0] * val[2] + w[3] * w[1] * val[3];
    return color;
}

void MultiThreadRender(Camera camera, Mat input_img, int input_img_width, int input_img_height, Mat result_img, int result_img_width, int img_dir_w, int widthStep, int widthOffset, int result_img_height, int img_dir_h, int heightStep, int heightOffset)
{
    float const aspectRatio = static_cast<float>(result_img_height) / result_img_width;
    for (int h = heightOffset; h < result_img_height; h += heightStep) {
        for (int w = widthOffset; w < result_img_width; w += widthStep) {
            camera.setRay(img_dir_w * (static_cast<float>(w) / result_img_width * 2 - 1), img_dir_h * (static_cast<float>(h) / result_img_height * 2 - 1) * aspectRatio);
            Vector3d pos_3d = interaction(camera.getRayPos(), camera.getRayDir());
            Vec2f UV = spherical_map(pos_3d);
            result_img.at<Vec3f>(h, w) = interpolation(input_img, input_img_width, input_img_height, UV[0], UV[1]);
        }
    }
}

int main()
{
    string file_name = "data/DJI_0080.JPG";
    Mat input_img = imread(file_name);
    if (input_img.empty())
    {
        cout << "Could not find or load the image..." << endl;
        return -1;
    }
    input_img.convertTo(input_img, CV_32F, 1 / 255.);
    int input_img_height = input_img.rows;
    int input_img_width = input_img.cols;
    int img_dir_h = 1;
    int img_dir_w = 1;
    Vector3d camera_origin = Vector3d(0, 0.8, 0);
    Vector3d camera_up = Vector3d(0, 0, 1);
    Vector3d camera_forward = Vector3d(0, -1.0, 0);
    float camera_fov = 120;

    Camera maincamera;
    maincamera.setPosition(Vector3d(0, 0.8, 0));
    maincamera.setForwardDirection(Vector3d(0, -1.0, 0));
    maincamera.setUpDirection(Vector3d(-1, 0, 0));
    maincamera.setFovAngle(camera_fov);

    int result_img_height = 512;
    int result_img_width = 512;
    Mat result_img = Mat::zeros(Size(result_img_width, result_img_height), CV_32FC3);

    // ++++++++ render single imagine ++++++++ //
    
    //// Spawn a thread for every logical processor -1, calling the renderThread function
    //int const nThreads = thread::hardware_concurrency();
    //vector<thread> threads;
    //for (int t = 0; t < nThreads - 1; ++t) 
    //{
    //    threads.emplace_back(MultiThreadRender, maincamera, input_img, input_img_width, input_img_height, result_img, result_img_width, img_dir_w, nThreads, t, result_img_height, img_dir_h, 1, 0);
    //}
    
    //// Call the MultiThreadRender function
    //MultiThreadRender(maincamera, input_img, input_img_width, input_img_height, result_img, result_img_width, img_dir_w, nThreads, nThreads - 1, result_img_height, img_dir_h, 1, 0);
    //
    //// Rejoin the threads
    //for (int t = 0; t < nThreads - 1; ++t) 
    //{
    //    threads[t].join();
    //}
    //imshow("result", result_img);
    //waitKey(0);
    //destroyAllWindows();
    //cout << "OK!" << endl;

    // ++++++++ render single imagine ++++++++ //


    // -------- adjust -------- //

    namedWindow("result");
    createTrackbar("camera_Y", "result", 0, 100, NULL);
    createTrackbar("fov", "result", 0, 90, NULL);
    // Spawn a thread for every logical processor -1, calling the renderThread function
    int const nThreads = thread::hardware_concurrency();
    while (true)
    {
        int camera_Y = getTrackbarPos("camera_Y", "result");
        int fov = getTrackbarPos("fov", "result");
        maincamera.setPosition(Vector3d(0, camera_Y / 100.0f, 0));
        maincamera.setFovAngle(float(fov + 90));

        vector<thread> threads;
        for (int t = 0; t < nThreads - 1; ++t)
        {
            threads.emplace_back(MultiThreadRender, maincamera, input_img, input_img_width, input_img_height, result_img, result_img_width, img_dir_w, nThreads, t, result_img_height, img_dir_h, 1, 0);
        }

        // Call the MultiThreadRender function
        MultiThreadRender(maincamera, input_img, input_img_width, input_img_height, result_img, result_img_width, img_dir_w, nThreads, nThreads - 1, result_img_height, img_dir_h, 1, 0);

        // Rejoin the threads
        for (int t = 0; t < nThreads - 1; ++t) 
        {
            threads[t].join();
        }

        imshow("result", result_img);
        int key = waitKey(1);
        if (key == 's')
        {
            int render_img_height = 1024;
            int render_img_width = 1024;
            Mat render_img = Mat::zeros(Size(render_img_width, render_img_height), CV_32FC3);
            vector<thread> threads;
            for (int t = 0; t < nThreads - 1; ++t)
            {
                threads.emplace_back(MultiThreadRender, maincamera, input_img, input_img_width, input_img_height, render_img, render_img_width, img_dir_w, nThreads, t, render_img_height, img_dir_h, 1, 0);
            }

            // Call the MultiThreadRender function
            MultiThreadRender(maincamera, input_img, input_img_width, input_img_height, render_img, render_img_width, img_dir_w, nThreads, nThreads - 1, render_img_height, img_dir_h, 1, 0);

            // Rejoin the threads
            for (int t = 0; t < nThreads - 1; ++t)
            {
                threads[t].join();
            }
            time_t t = time(0);
            char timestamp[32] = { NULL };
            strftime(timestamp, sizeof(timestamp), "%Y_%m_%d__%H_%M_%S", localtime(&t));
            string path = "result/";
            string format = ".png";
            string output_file_name = path + timestamp + format;
            render_img.convertTo(render_img, CV_8U, 255.0);
            imwrite(output_file_name, render_img);
            cout << "Imagine saved! At: " << output_file_name << endl;
        }
        else if (key == 27) break;
    }
    destroyAllWindows();
    cout << "OK!" << endl;
    // -------- adjust -------- //
}

