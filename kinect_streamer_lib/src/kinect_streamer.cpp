#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <kinect_depth/kinect_depth.h>
#include <cuda_runtime.h>
#include <cstdio>
#include <cstring>

#include <kinect_streamer/kinect_streamer.hpp>

#define USE_CUDA

namespace KinectStreamer {

KinectDevice::KinectDevice(std::string serial) {

#ifdef USE_CUDA
    pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
    pipeline = new libfreenect2::CpuPacketPipeline();
#endif

    freenect2 = new libfreenect2::Freenect2();
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    
    if (freenect2->enumerateDevices() == 0) {
        std::cout << "No devices found!" << std::endl;
        throw std::exception();
    }
    if (serial == "") {
        serial = freenect2->getDefaultDeviceSerialNumber();
    }
    dev = freenect2->openDevice(serial, pipeline);
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
}
void KinectDevice::set_color_params(float cx, float cy, float fx, float fy) {
    color_params.cx = cx;
    color_params.cy = cy;
    color_params.fx = fx;
    color_params.fy = fy;
    dev->setColorCameraParams(color_params);
}


void KinectDevice::get_color_params(float& cx, float& cy, float& fx, float& fy) {
    cx = color_params.cx;
    cy = color_params.cy;
    fx = color_params.fx;
    fy = color_params.fy;
}

void KinectDevice::set_ir_params(float cx, float cy, float fx, float fy, float k1, float k2, float k3, float p1, float p2) {
    ir_params.cx = cx;
    ir_params.cy = cy;
    ir_params.fx = fx;
    ir_params.fy = fy;
    ir_params.k1 = k1;
    ir_params.k2 = k2;
    ir_params.k3 = k3;
    ir_params.p1 = p1;
    ir_params.p2 = p2;
    dev->setIrCameraParams(ir_params);
}

void KinectDevice::get_ir_params(float& cx, float& cy, float& fx, float& fy, float k1, float k2, float k3, float p1, float p2) {
    cx = ir_params.cx;
    cy = ir_params.cy;
    fx = ir_params.fx;
    fy = ir_params.fy;
    k1 = ir_params.k1;
    k2 = ir_params.k2;
    k3 = ir_params.k3;
    p1 = ir_params.p1;
    p2 = ir_params.p2;
}

void KinectDevice::init_registration() {
    registration = new libfreenect2::Registration(ir_params, color_params);
}


void KinectDevice::init_params() {
    color_params = dev->getColorCameraParams();
    ir_params = dev->getIrCameraParams();   
}


int KinectDevice::start() {
    return dev->start();
    
}

int KinectDevice::stop() {
    return dev->stop();
}


/**
 * @brief Converts small array of depth values (n = numPoints) to points in cartesians space (X,Y,Z)
 * 
 * @param depth         Depth image
 * @param registered    Registered image
 * @param x             x-coordinates
 * @param y             y-coordinates
 * @param z             z-coordinates
 * @param numPoints     Number of points to process
 */
void KinectDevice::rowColDepthToXYZ(const float* row_arr, const float* col_arr, const float* depth_arr, float* x_arr, float* y_arr, float* z_arr, int numPoints) {
    
    const float cx = ir_params.cx;
    const float cy = ir_params.cy;
    const float fx = 1 / ir_params.fx;
    const float fy = 1 / ir_params.fy;

    for (int n = 0; n < numPoints; n++) {

        const float row = row_arr[n];
        const float col = col_arr[n];
        const float depth_val = depth_arr[n] / 1000.0f;

        if (!isnan(depth_val) && depth_val > 0.001) {

            x_arr[n] = -(col + 0.5 - cx) * fx * depth_val;
            y_arr[n] = (row + 0.5 - cy) * fy * depth_val;
            z_arr[n] = depth_val;
        } else {

            x_arr[n] = 0;
            y_arr[n] = 0;
            z_arr[n] = 0;
        }
    }
}

/**
 * @brief Uses CPU to process depth information to get Point Cloud (ROS - PCL)
 * 
 * @param depth         Depth Image
 * @param registered    Registered Image
 * @param cloud_data    ROS Point Cloud Data
 * @param width         Width of Image
 * @param height        Height of Image
 */
void KinectDevice::getPointCloudCpu(const float* depth, const uint32_t* registered, uint8_t* cloud_data, int width, int height) {

    const float cx = ir_params.cx;
    const float cy = ir_params.cy;
    const float fx = 1 / ir_params.fx;
    const float fy = 1 / ir_params.fy;

    int numElements = width * height;
    const int point_step = 32;
    for (int i = 0; i < numElements; i++) {
        const int row = i / width;
        const int col = i % width;
        const float depth_val = depth[width * row + col] / 1000.0f;
        if (!isnan(depth_val) && depth_val > 0.001) {
            uint8_t* ptr = cloud_data + i * point_step;
            /* x-value */
            *(float*)(ptr + 0) = (col + 0.5 - cx) * fx * depth_val;
            /* y-value */
            *(float*)(ptr + 4) = (row + 0.5 - cy) * fy * depth_val;
            /* z-value */
            *(float*)(ptr + 8) = depth_val;
            /* rgb-value */
            *(uint32_t*)(ptr + 16) = registered[i];
        }
    }
}

void KinectDevice::getPointCloudCuda(const float* depth, const uint32_t* registered, uint8_t* cloud_data, int width, int height) {

    uint8_t* cloud_data_gpu = NULL;
    float* depth_gpu = NULL;
    uint32_t* registered_gpu = NULL;

    cudaError_t err = cudaSuccess;
    const int point_step = 32;
    err = cudaMalloc((void**)&cloud_data_gpu, DEPTH_W * DEPTH_H * point_step);

    err = cudaMalloc((void**)&depth_gpu, DEPTH_W * DEPTH_H * sizeof(float));
    err = cudaMalloc((void**)&registered_gpu, DEPTH_W * DEPTH_H * sizeof(uint32_t));

    err = cudaMemcpy(depth_gpu, depth, DEPTH_W * DEPTH_H * sizeof(float), cudaMemcpyHostToDevice);
    err = cudaMemcpy(registered_gpu, registered, DEPTH_W * DEPTH_H * sizeof(uint32_t), cudaMemcpyHostToDevice);

    float cx = ir_params.cx;
    float cy = ir_params.cy;
    float fx = 1 / ir_params.fx;
    float fy = 1 / ir_params.fy;

    getPointXYZHelper(depth_gpu, registered_gpu, cloud_data_gpu, cx, cy, fx, fy, DEPTH_W, DEPTH_H);

    err = cudaDeviceSynchronize();
    err = cudaMemcpy(cloud_data, cloud_data_gpu, DEPTH_W * DEPTH_H * point_step, cudaMemcpyDeviceToHost);

    err = cudaFree(cloud_data_gpu);
    err = cudaFree(depth_gpu);
    err = cudaFree(registered_gpu);

    err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("%s %d\n\r", cudaGetErrorString(err), __LINE__);
    }

}

void KinectDevice::depth_to_color(float mx, float my, float& rx, float& ry) {

    static const float depth_q = 0.01;
    static const float color_q = 0.002199;
    
    mx = (mx - ir_params.cx) * depth_q;
    my = (my - ir_params.cy) * depth_q;

    float wx =
        (mx * mx * mx * color_params.mx_x3y0) + (my * my * my * color_params.mx_x0y3) +
        (mx * mx * my * color_params.mx_x2y1) + (my * my * mx * color_params.mx_x1y2) +
        (mx * mx * color_params.mx_x2y0) + (my * my * color_params.mx_x0y2) + (mx * my * color_params.mx_x1y1) +
        (mx * color_params.mx_x1y0) + (my * color_params.mx_x0y1) + (color_params.mx_x0y0);

    float wy =
        (mx * mx * mx * color_params.my_x3y0) + (my * my * my * color_params.my_x0y3) +
        (mx * mx * my * color_params.my_x2y1) + (my * my * mx * color_params.my_x1y2) +
        (mx * mx * color_params.my_x2y0) + (my * my * color_params.my_x0y2) + (mx * my * color_params.my_x1y1) +
        (mx * color_params.my_x1y0) + (my * color_params.my_x0y1) + (color_params.my_x0y0);

    rx = (wx / (color_params.fx * color_q)) - (color_params.shift_m / color_params.shift_d);
    ry = (wy / color_q) + color_params.cy;
}

void KinectDevice::wait_frames() {
    if (!listener->waitForNewFrame(frames, 10 * 1000)) {
        std::cout << "Error!" << std::endl;
        throw std::exception();
    }
}
libfreenect2::Frame* KinectDevice::get_frame(libfreenect2::Frame::Type type) {
    return frames[type];
}

void KinectDevice::release_frames() {
    listener->release(frames);
}

KinectDevice::~KinectDevice() {
    delete pipeline;
    delete freenect2;
    delete listener;
    delete registration;
}

libfreenect2::Registration* KinectDevice::get_registration() {
    return registration;
}

}