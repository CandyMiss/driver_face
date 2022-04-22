#include "arcface.h"

extern "C" void InitFaceGallaryToDevice(float *h_gallary_buffer);
extern "C" int GetSimilarityIndex(float *d_face_buffers);
//extern "C" int GetSimilarityIndex(float *h_face_buffers);

namespace ArcFace
{
    const int INPUT_H = 112;
    const int INPUT_W = 112;
    const int BATCH_SIZE = 1;  // currently, only support BATCH=1

    const std::string face_data_filename = "/home/nvidia/wdq/ros_vision/src/driver_face/src/res/face.data";
    const char *INPUT_BLOB_NAME = "data";
    const char *OUTPUT_BLOB_NAME = "prob";

    // 3 个内部变量，在程序main函数开始阶段即通过init操作进行初始化
    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;

    // 人脸数据——理论上为2000人，每个人为512个特征数据
    float FaceDataBase[GALLARY_NUM * FACE_FEATURE_DIMENSION] = {0.0};

    void ReadFaceDataToGPU()
    {
        float gallaryData[GALLARY_NUM * FACE_FEATURE_DIMENSION];
        std::ifstream inFStream(face_data_filename.c_str(), std::ios::binary);
        if(!inFStream){
            std::cout << "Failed to open face data file." << std::endl;
        }
        else{
            std::cout<< "Face data file opened successfully." << std::endl;
        }
        inFStream.read((char *) &gallaryData, sizeof(gallaryData));
        inFStream.close();

        memcpy(FaceDataBase, gallaryData, GALLARY_NUM * FACE_FEATURE_DIMENSION * sizeof(float));

        int tmplenth = ArcFace::GALLARY_NUM * ArcFace::FACE_FEATURE_DIMENSION;
        //test
        // for(int i=0; i<1024; ++i){
        //     std::cout << ArcFace::FaceDataBase[i] << "    ";
        // }
        // std::cout << std::endl;

        InitFaceGallaryToDevice(FaceDataBase);
    }

#pragma region 层结构

    //BN层
    IScaleLayer *addBatchNorm2d(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input,
                                std::string lname, float eps)
    {
        float *gamma = (float *) weightMap[lname + "_gamma"].values;
        float *beta = (float *) weightMap[lname + "_beta"].values;
        float *mean = (float *) weightMap[lname + "_moving_mean"].values;
        float *var = (float *) weightMap[lname + "_moving_var"].values;
        int len = weightMap[lname + "_moving_var"].count;

        float *scval = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            scval[i] = gamma[i] / sqrt(var[i] + eps);
        }
        Weights scale{DataType::kFLOAT, scval, len};

        float *shval = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            shval[i] = beta[i] - mean[i] * gamma[i] / sqrt(var[i] + eps);
        }
        Weights shift{DataType::kFLOAT, shval, len};

        float *pval = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            pval[i] = 1.0;
        }
        Weights power{DataType::kFLOAT, pval, len};

        weightMap[lname + ".scale"] = scale;
        weightMap[lname + ".shift"] = shift;
        weightMap[lname + ".power"] = power;
        IScaleLayer *scale_1 = network->addScale(input, ScaleMode::kCHANNEL, shift, scale, power);
        assert(scale_1);
        return scale_1;
    }

    //prelu层
    ILayer *
    addPRelu(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, std::string lname)
    {
        float *gamma = (float *) weightMap[lname + "_gamma"].values;
        int len = weightMap[lname + "_gamma"].count;

        float *scval_1 = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        float *scval_2 = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            scval_1[i] = -1.0;
            scval_2[i] = -gamma[i];
        }
        Weights scale_1{DataType::kFLOAT, scval_1, len};
        Weights scale_2{DataType::kFLOAT, scval_2, len};

        float *shval = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            shval[i] = 0.0;
        }
        Weights shift{DataType::kFLOAT, shval, len};

        float *pval = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            pval[i] = 1.0;
        }
        Weights power{DataType::kFLOAT, pval, len};

        auto relu1 = network->addActivation(input, ActivationType::kRELU);
        assert(relu1);
        IScaleLayer *scale1 = network->addScale(input, ScaleMode::kCHANNEL, shift, scale_1, power);
        assert(scale1);
        auto relu2 = network->addActivation(*scale1->getOutput(0), ActivationType::kRELU);
        assert(relu2);
        IScaleLayer *scale2 = network->addScale(*relu2->getOutput(0), ScaleMode::kCHANNEL, shift, scale_2, power);
        assert(scale2);
        IElementWiseLayer *ew1 = network->addElementWise(*relu1->getOutput(0), *scale2->getOutput(0),
                                                         ElementWiseOperation::kSUM);
        assert(ew1);
        return ew1;
    }

    //残差单元层
    ILayer *
    resUnit(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int num_filters,
            int s, bool dim_match, std::string lname)
    {
        Weights emptywts{DataType::kFLOAT, nullptr, 0};
        auto bn1 = addBatchNorm2d(network, weightMap, input, lname + "_bn1", 2e-5);
        IConvolutionLayer *conv1 = network->addConvolutionNd(*bn1->getOutput(0), num_filters, DimsHW{3, 3},
                                                             weightMap[lname + "_conv1_weight"], emptywts);
        assert(conv1);
        conv1->setPaddingNd(DimsHW{1, 1});
        auto bn2 = addBatchNorm2d(network, weightMap, *conv1->getOutput(0), lname + "_bn2", 2e-5);
        auto act1 = addPRelu(network, weightMap, *bn2->getOutput(0), lname + "_relu1");
        IConvolutionLayer *conv2 = network->addConvolutionNd(*act1->getOutput(0), num_filters, DimsHW{3, 3},
                                                             weightMap[lname + "_conv2_weight"], emptywts);
        assert(conv2);
        conv2->setStrideNd(DimsHW{s, s});
        conv2->setPaddingNd(DimsHW{1, 1});
        auto bn3 = addBatchNorm2d(network, weightMap, *conv2->getOutput(0), lname + "_bn3", 2e-5);

        IElementWiseLayer *ew1;
        if (dim_match)
        {
            ew1 = network->addElementWise(input, *bn3->getOutput(0), ElementWiseOperation::kSUM);
        }
        else
        {
            IConvolutionLayer *conv1sc = network->addConvolutionNd(input, num_filters, DimsHW{1, 1},
                                                                   weightMap[lname + "_conv1sc_weight"], emptywts);
            assert(conv1sc);
            conv1sc->setStrideNd(DimsHW{s, s});
            auto bn1sc = addBatchNorm2d(network, weightMap, *conv1sc->getOutput(0), lname + "_sc", 2e-5);
            ew1 = network->addElementWise(*bn1sc->getOutput(0), *bn3->getOutput(0), ElementWiseOperation::kSUM);
        }
        assert(ew1);
        return ew1;
    }

#pragma endregion

    std::map<std::string, Weights> loadWeights(const std::string file)
    {
        std::cout << "Loading weights: " << file << std::endl;
        std::map<std::string, Weights> weightMap;

        // Open weights file
        std::ifstream input(file);
        assert(input.is_open() && "Unable to load weight file.");

        // Read number of weight blobs
        int32_t count;
        input >> count;
        assert(count > 0 && "Invalid weight map file.");

        while (count--)
        {
            Weights wt{DataType::kFLOAT, nullptr, 0};
            uint32_t size;

            // Read name and type of blob
            std::string name;
            input >> name >> std::dec >> size;
            wt.type = DataType::kFLOAT;

            // Load blob
            uint32_t *val = reinterpret_cast<uint32_t *>(malloc(sizeof(val) * size));
            for (uint32_t x = 0, y = size; x < y; ++x)
            {
                input >> std::hex >> val[x];
            }
            wt.values = val;

            wt.count = size;
            weightMap[name] = wt;
        }

        return weightMap;
    }

    ICudaEngine *createEngine(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
    {
        INetworkDefinition *network = builder->createNetworkV2(0U);

        // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
        ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
        assert(data);

        std::map<std::string, Weights> weightMap = loadWeights("../arcface-r50.wts");
        Weights emptywts{DataType::kFLOAT, nullptr, 0};

        IConvolutionLayer *conv0 = network->addConvolutionNd(*data, 64, DimsHW{3, 3}, weightMap["conv0_weight"],
                                                             emptywts);
        assert(conv0);
        conv0->setPaddingNd(DimsHW{1, 1});
        auto bn0 = addBatchNorm2d(network, weightMap, *conv0->getOutput(0), "bn0", 2e-5);
        auto relu0 = addPRelu(network, weightMap, *bn0->getOutput(0), "relu0");

        auto s1u1 = resUnit(network, weightMap, *relu0->getOutput(0), 64, 2, false, "stage1_unit1");
        auto s1u2 = resUnit(network, weightMap, *s1u1->getOutput(0), 64, 1, true, "stage1_unit2");
        auto s1u3 = resUnit(network, weightMap, *s1u2->getOutput(0), 64, 1, true, "stage1_unit3");

        auto s2u1 = resUnit(network, weightMap, *s1u3->getOutput(0), 128, 2, false, "stage2_unit1");
        auto s2u2 = resUnit(network, weightMap, *s2u1->getOutput(0), 128, 1, true, "stage2_unit2");
        auto s2u3 = resUnit(network, weightMap, *s2u2->getOutput(0), 128, 1, true, "stage2_unit3");
        auto s2u4 = resUnit(network, weightMap, *s2u3->getOutput(0), 128, 1, true, "stage2_unit4");

        auto s3u1 = resUnit(network, weightMap, *s2u4->getOutput(0), 256, 2, false, "stage3_unit1");
        auto s3u2 = resUnit(network, weightMap, *s3u1->getOutput(0), 256, 1, true, "stage3_unit2");
        auto s3u3 = resUnit(network, weightMap, *s3u2->getOutput(0), 256, 1, true, "stage3_unit3");
        auto s3u4 = resUnit(network, weightMap, *s3u3->getOutput(0), 256, 1, true, "stage3_unit4");
        auto s3u5 = resUnit(network, weightMap, *s3u4->getOutput(0), 256, 1, true, "stage3_unit5");
        auto s3u6 = resUnit(network, weightMap, *s3u5->getOutput(0), 256, 1, true, "stage3_unit6");
        auto s3u7 = resUnit(network, weightMap, *s3u6->getOutput(0), 256, 1, true, "stage3_unit7");
        auto s3u8 = resUnit(network, weightMap, *s3u7->getOutput(0), 256, 1, true, "stage3_unit8");
        auto s3u9 = resUnit(network, weightMap, *s3u8->getOutput(0), 256, 1, true, "stage3_unit9");
        auto s3u10 = resUnit(network, weightMap, *s3u9->getOutput(0), 256, 1, true, "stage3_unit10");
        auto s3u11 = resUnit(network, weightMap, *s3u10->getOutput(0), 256, 1, true, "stage3_unit11");
        auto s3u12 = resUnit(network, weightMap, *s3u11->getOutput(0), 256, 1, true, "stage3_unit12");
        auto s3u13 = resUnit(network, weightMap, *s3u12->getOutput(0), 256, 1, true, "stage3_unit13");
        auto s3u14 = resUnit(network, weightMap, *s3u13->getOutput(0), 256, 1, true, "stage3_unit14");

        auto s4u1 = resUnit(network, weightMap, *s3u14->getOutput(0), 512, 2, false, "stage4_unit1");
        auto s4u2 = resUnit(network, weightMap, *s4u1->getOutput(0), 512, 1, true, "stage4_unit2");
        auto s4u3 = resUnit(network, weightMap, *s4u2->getOutput(0), 512, 1, true, "stage4_unit3");

        auto bn1 = addBatchNorm2d(network, weightMap, *s4u3->getOutput(0), "bn1", 2e-5);
        IFullyConnectedLayer *fc1 = network->addFullyConnected(*bn1->getOutput(0), 512, weightMap["pre_fc1_weight"],
                                                               weightMap["pre_fc1_bias"]);
        assert(fc1);
        auto bn2 = addBatchNorm2d(network, weightMap, *fc1->getOutput(0), "fc1", 2e-5);

        bn2->getOutput(0)->setName(OUTPUT_BLOB_NAME);
        network->markOutput(*bn2->getOutput(0));

        // Build engine
        builder->setMaxBatchSize(maxBatchSize);
        config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
#ifdef USE_FP16
        config->setFlag(BuilderFlag::kFP16);
#endif
        std::cout << "Building engine, please wait for a while..." << std::endl;
        ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
        std::cout << "Build engine successfully!" << std::endl;

        // Don't need the network any more
        network->destroy();

        // Release host memory
        for (auto &mem : weightMap)
        {
            free((void *) (mem.second.values));
        }

        return engine;
    }

    void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream)
    {
        // Create builder
        IBuilder *builder = createInferBuilder(gLogger);
        IBuilderConfig *config = builder->createBuilderConfig();

        // Create model to populate the network, then set the outputs and create an engine
        ICudaEngine *engine = createEngine(maxBatchSize, builder, config, DataType::kFLOAT);
        assert(engine != nullptr);

        // Serialize the engine
        (*modelStream) = engine->serialize();

        // Close everything down
        engine->destroy();
        builder->destroy();
    }

    void InitArcFaceEngine()
    {
        char *trtModelStream{nullptr};
        size_t size{0};

        std::ifstream file("/home/nvidia/wdq/ArcFaceGenEngine/build/arcface.engine", std::ios::binary);
        if(!file.is_open()){
            std::cout << "Failed to open arcface.engine." << std::endl;
        }
        else{
            std::cout<< "arcface.engine file opened successfully." << std::endl;
        }
        if (file.good())
        {
            file.seekg(0, file.end);
            size = file.tellg();
            file.seekg(0, file.beg);
            trtModelStream = new char[size];
            assert(trtModelStream);
            file.read(trtModelStream, size);
            file.close();
        }

        runtime = createInferRuntime(gLogger);
        assert(runtime != nullptr);
        engine = runtime->deserializeCudaEngine(trtModelStream, size);
        assert(engine != nullptr);
        context = engine->createExecutionContext();
        assert(context != nullptr);
        delete[] trtModelStream;
    }

    void ReleasePFLDEngine()
    {
        context->destroy();
        engine->destroy();
        runtime->destroy();
    }

    void preImg(cv::Mat faceImg, float *h_data)
    {
//        std::cout << "In preImg function" << std::endl;
        cv::Mat img(INPUT_H, INPUT_W, CV_8UC3);  //申请的cv中存图片的格式CV_8UC3
//        std::cout << "start resize image" << std::endl;
        cv::resize(faceImg, img, img.size(), 0, 0);
//        std::cout << "resize image finish" << std::endl; 

        for (int i = 0; i < INPUT_H * INPUT_W; i++)
        {
            h_data[i] = ((float) img.at<cv::Vec3b>(i)[2] - 127.5) * 0.0078125;
            h_data[i + INPUT_H * INPUT_W] = ((float) img.at<cv::Vec3b>(i)[1] - 127.5) * 0.0078125;
            h_data[i + 2 * INPUT_H * INPUT_W] = ((float) img.at<cv::Vec3b>(i)[0] - 127.5) * 0.0078125;
        }
    }

    void GetFaceFeature(cv::Mat faceImg, float *faceFeature)
    {
        float h_data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
        preImg(faceImg, h_data);
        doInference(h_data, faceFeature);
    }

    inline void printVector(float *tmpFaceFeature, int lenth){
        for(int i=0; i<lenth; ++i){
            std::cout << tmpFaceFeature[i] << " ";
        }
        std::cout << std::endl;
    }

    void DetectFaceID(cv::Mat faceImg, int &faceID, float *faceBestFeature)
    {
        auto start = std::chrono::system_clock::now();
        float h_data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];   //输入大小：batch size×3×h×w
        memset(h_data, 0, sizeof(h_data)); 
         // std::cout << "test h_data" <<std::endl;
        // printVector(h_data, BATCH_SIZE * 3 * INPUT_H * INPUT_W);
        preImg(faceImg, h_data);
        //test h_data
        // std::cout << "test h_data" <<std::endl;
        // printVector(h_data, BATCH_SIZE * 3 * INPUT_H * INPUT_W);     //get h_data ok
        // std::cout << "preImg finish" << std::endl;

        doInferenceGetID(h_data, faceBestFeature, faceID);
        auto end = std::chrono::system_clock::now();
        std::cout << "ArcFace RUN time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }

    void doInference(float *input, float *output)
    {
        const ICudaEngine &engine = context->getEngine();

        assert(engine.getNbBindings() == 2);
        void *buffers[2];

        const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);//data
        const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);

        CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
        CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * FACE_FEATURE_DIMENSION * sizeof(float)));

        cudaStream_t stream;
        CHECK(cudaStreamCreate(&stream));

        CHECK(cudaMemcpyAsync(buffers[inputIndex], input, BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
        context->enqueue(BATCH_SIZE, buffers, stream, nullptr);
        CHECK(cudaMemcpyAsync(output, buffers[outputIndex], BATCH_SIZE * FACE_FEATURE_DIMENSION * sizeof(float), cudaMemcpyDeviceToHost, stream));

        cudaStreamSynchronize(stream);

        // Release stream and buffers
        cudaStreamDestroy(stream);

        CHECK(cudaFree(buffers[inputIndex]));
        CHECK(cudaFree(buffers[outputIndex]));
    }

    //第一次人脸识别，返回这个人的id（cos相似度取最大值），以及这个人的第一张feature map
    void doInferenceGetID(float *input, float *output, int &faceId)
    {
        //形参为输入host_data数组，输入一张人脸的数据：BATCH_SIZE * 3 * INPUT_H * INPUT_W，输出特征图结果和id（从0开始）
        const ICudaEngine &engine = context->getEngine();

        assert(engine.getNbBindings() == 2);
        void *buffers[2];

        const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);//data 0号引擎
        const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);
      
        //&引用传递的参数，修改效果会和实参保持一致
        CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
        CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * FACE_FEATURE_DIMENSION * sizeof(float)));
        //buffers[outputIndex]存一批人脸特征

        int lenth = BATCH_SIZE * FACE_FEATURE_DIMENSION;
        // // 测试buffers[outputIndex]里面的数据
        // std::cout << "buffers[outputIndex]:  "<< std::endl;
        // float* tmp = (float*)buffers[outputIndex];
        // for(int i=0; i<lenth; ++i){
        // std::cout << *(tmp++) << " ";
        // }
        // std::cout << std::endl;

        cudaStream_t stream;
        CHECK(cudaStreamCreate(&stream));

        CHECK(cudaMemcpyAsync(buffers[inputIndex], input, BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
     
        context->enqueue(BATCH_SIZE, buffers, stream, nullptr);
        CHECK(cudaMemcpyAsync(output, buffers[outputIndex], BATCH_SIZE * FACE_FEATURE_DIMENSION * sizeof(float), cudaMemcpyDeviceToHost, stream));

        cudaStreamSynchronize(stream);

        //
        // // 测试buffers[outputIndex]里面的数据
        // std::cout << "buffers[outputIndex]:  "<< std::endl;
        // tmp = (float*)buffers[outputIndex];
        // for(int i=0; i<lenth; ++i){
        // std::cout << *(tmp++) << " ";
        // }
        // std::cout << std::endl;


        //cuda加速得到人脸的id
        faceId = GetSimilarityIndex((float *) buffers[outputIndex]);    //注意一定要传入cuda端的数据

        std::cout << "faceID: " << faceId <<  std::endl;
        // Release stream and buffers
        cudaStreamDestroy(stream);
        CHECK(cudaFree(buffers[inputIndex]));
        CHECK(cudaFree(buffers[outputIndex]));

    }

    //计算余弦相似度
    float GetSimilarOfTwoFace(float *faceCur, float *faceBest)
    {
        // a 2000*512 , b 512*1, c 2000*1
        float sum = 0;
        float faceCurSquaresum = 0;//a元素的平方和
        float faceBestSquaresum = 0;//b元素的平方和
        for (unsigned int i = 0; i < FACE_FEATURE_DIMENSION; i++)
        {
            sum += faceCur[i] * faceBest[i];
            faceCurSquaresum += faceCur[i] * faceCur[i];
            faceBestSquaresum += faceBest[i] * faceBest[i];
        }
        return sum / sqrt(faceCurSquaresum * faceBestSquaresum);
    }
}