#include <stdio.h>
#include <stdlib.h>
#include <cuda_runtime.h>
#include <iostream>

static const unsigned int FACE_NUM = 2000;
static const unsigned int FACE_FEATURE_SIZE = 512;

float *d_gallary_buffer;

__global__ void vectorMulti(const float *A, const float *B, float *C, int numElements)
{
    // a 2000*512 , b 512*1, c 2000*1
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx < numElements)
    {
        float sum = 0;
        float a_squaresum = 0;//a元素的平方和
        float b_squaresum = 0;//b元素的平方和
        for (int k = 0; k < FACE_FEATURE_SIZE; k++)
        {
            sum += A[idx * FACE_FEATURE_SIZE + k] * B[k];
            a_squaresum += A[idx * FACE_FEATURE_SIZE + k] * A[idx * FACE_FEATURE_SIZE + k];
            b_squaresum += B[k] * B[k];
        }
        C[idx] = sum / sqrt(a_squaresum * b_squaresum);
    }
}

// 本函数仅执行一次！人脸Gallary常驻显存
extern "C" void InitFaceGallaryToDevice(float *h_gallary_buffer)
{
    cudaMalloc((void **) &d_gallary_buffer, FACE_NUM * FACE_FEATURE_SIZE * sizeof(float));
    cudaMemcpy(d_gallary_buffer, h_gallary_buffer, FACE_NUM * FACE_FEATURE_SIZE * sizeof(float), cudaMemcpyHostToDevice);

    for (int i = 0; i < 10; i++)
    {
        std::cout << h_gallary_buffer[i] << " ";
    }
    std::cout << std::endl << "人脸Gallary数据已经上传到GPU上" << std::endl;
}

extern "C" int GetSimilarityIndex(float *d_face_buffers)    // 直接处理cuda端数据，其内存由调用者申请和释放
{
    float *h_similar_result = (float *) malloc(FACE_NUM * sizeof(float));   // 保存输出结果的2000个相似度，但并不需要返回，所以最后要释放

    //初始化cuda端数组
    float *d_similar_result = NULL;
    cudaMalloc((void **) &d_similar_result, FACE_NUM * sizeof(float)); // CUDA 端 2000 个相似度输出

    int threadsPerBlock = FACE_FEATURE_SIZE;
    int blocksPerGrid = (FACE_NUM + threadsPerBlock - 1) / threadsPerBlock;

    vectorMulti<<<blocksPerGrid, threadsPerBlock>>>(d_gallary_buffer, d_face_buffers, d_similar_result, FACE_NUM);

    cudaMemcpy(h_similar_result, d_similar_result, FACE_NUM * sizeof(float), cudaMemcpyDeviceToHost);
    //std::cout << "h_similar_result[0]:  " << h_similar_result[0] << std::endl;

    int h_id = 0;                                                    //返回0表示没找到
    for (unsigned int i = 1; i < FACE_NUM; i++)                     //从下标1开始找
    {
        //std::cout << "[i]:  " << i<< "h_similar_result[i]:  " << (float)h_similar_result[i] << std::endl;
        if (h_similar_result[i] > h_similar_result[h_id])
        {
            h_id = i;
        }
    }

#pragma region 测试
//    std::cout <<"调试gallary数据："<< std::endl;
//    float *h_gallary_buffer_test = (float *) malloc(FACE_NUM * FACE_FEATURE_SIZE * sizeof(float));
//    cudaMemcpy(h_gallary_buffer_test, d_gallary_buffer, FACE_NUM * FACE_FEATURE_SIZE * sizeof(float), cudaMemcpyDeviceToHost);
//    for (unsigned int i = 0; i < 10; i++)
//    {
//        std::cout << h_gallary_buffer_test[i] << ", ";
//    }
//    std::cout << std::endl;
//    free(h_gallary_buffer_test);
//
//    std::cout <<"调试face数据："<< std::endl;
//    float *h_face_buffer_test = (float *) malloc(FACE_FEATURE_SIZE * sizeof(float));
//    cudaMemcpy(h_face_buffer_test, d_face_buffers,  FACE_FEATURE_SIZE * sizeof(float), cudaMemcpyDeviceToHost);
//    for (unsigned int i = 0; i < 10; i++)
//    {
//        std::cout << h_face_buffer_test[i] << ", ";
//    }
//    std::cout << std::endl;
//    free(h_face_buffer_test);
#pragma endregion

    std::cout << "h_id: " << h_id << "  h_similar_result[h_id]: "<< h_similar_result[h_id] << std::endl;
    //给出阈值，对于是别到的id，若最大的相似度不达标，返回结果为识别失败
    if(h_similar_result[h_id] <= 0.6)
        h_id = 0;

    cudaFree(d_similar_result);
    free(h_similar_result);
    // std::cout << "h_id: " << h_id << "  h_similar_result[h_id]: "<< h_similar_result[h_id] << std::endl;
    return h_id;
}


