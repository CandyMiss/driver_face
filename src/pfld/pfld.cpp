#include"pfld.h"
#include "../yolov5/common.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace PFLD
{
    static const int INPUT_H = 112; //输入112
    static const int INPUT_W = 112;

    const char *INPUT_BLOB_NAME = "data";
    const char *OUTPUT_BLOB_NAME = "prob";

    static Logger gLogger;

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

    //定义BN层
    IScaleLayer *addBatchNorm2d(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input,
                                std::string lname, float eps)
    {
        float *gamma = (float *) weightMap[lname + ".weight"].values;
        float *beta = (float *) weightMap[lname + ".bias"].values;
        float *mean = (float *) weightMap[lname + ".running_mean"].values;
        float *var = (float *) weightMap[lname + ".running_var"].values;
        int len = weightMap[lname + ".running_var"].count;

        float *scval = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            scval[i] = gamma[i] / sqrt(var[i] + eps);  //  =weight值/标准差
        }
        Weights scale{DataType::kFLOAT, scval, len};  //得到weights的scale

        float *shval = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            shval[i] = beta[i] - mean[i] * gamma[i] / sqrt(var[i] + eps); //  =（bias值-runningmean的值*weight值）/标准差
        }
        Weights shift{DataType::kFLOAT, shval, len};  //得到weights的shift

        float *pval = reinterpret_cast<float *>(malloc(sizeof(float) * len));
        for (int i = 0; i < len; i++)
        {
            pval[i] = 1.0;
        }
        Weights power{DataType::kFLOAT, pval, len};   //得到扩增倍数1.0

        weightMap[lname + ".scale"] = scale;
        weightMap[lname + ".shift"] = shift;
        weightMap[lname + ".power"] = power;
        IScaleLayer *scale_1 = network->addScale(input, ScaleMode::kCHANNEL, shift, scale, power);  //scale层，相当于将大小重新定义
        assert(scale_1);
        return scale_1;
    }

    //倒残差函数，重新定义
    ILayer *
    InvertedResidual(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int stride,
                     int outch, bool use_res_connect, float g, std::string lname)
    {
        Weights emptywts{DataType::kFLOAT, nullptr, 0};
        IConvolutionLayer *conv1 = network->addConvolutionNd(input, g, DimsHW{1, 1},
                                                             weightMap[lname + ".conv.0.weight"],
                                                             emptywts);
        auto bn1 = addBatchNorm2d(network, weightMap, *conv1->getOutput(0), lname + ".conv.1", 1e-5);
        IActivationLayer *relu1 = network->addActivation(*bn1->getOutput(0), ActivationType::kRELU);
        IConvolutionLayer *conv2 = network->addConvolutionNd(*relu1->getOutput(0), g, DimsHW{3, 3},
                                                             weightMap[lname + ".conv.3.weight"], emptywts);
        conv2->setStrideNd(DimsHW{stride, stride});
        conv2->setPaddingNd(DimsHW{1, 1});
        conv2->setNbGroups(g);
        auto bn2 = addBatchNorm2d(network, weightMap, *conv2->getOutput(0), lname + ".conv.4", 1e-5);
        IActivationLayer *relu2 = network->addActivation(*bn2->getOutput(0), ActivationType::kRELU);
        IConvolutionLayer *conv3 = network->addConvolutionNd(*relu2->getOutput(0), outch, DimsHW{1, 1},
                                                             weightMap[lname + ".conv.6.weight"], emptywts);
        auto bn3 = addBatchNorm2d(network, weightMap, *conv3->getOutput(0), lname + ".conv.7", 1e-5);
        if (use_res_connect)
        {
            auto ew = network->addElementWise(input, *bn3->getOutput(0), ElementWiseOperation::kSUM);
            return ew;
        }
        else
        {
            return bn3;
        }

    }

    //conv和bn和relu
    ILayer *
    conv_bn(INetworkDefinition *network, std::map<std::string, Weights> &weightMap, ITensor &input, int stride,
            int outch,
            int kernel, std::string lname)
    {
        Weights emptywts{DataType::kFLOAT, nullptr, 0};
        IConvolutionLayer *conv = network->addConvolutionNd(input, outch, DimsHW{kernel, kernel},
                                                            weightMap[lname + ".0.weight"], emptywts);
        conv->setStrideNd(DimsHW{stride, stride});
        conv->setPaddingNd(DimsHW{1, 1});
        auto bn = addBatchNorm2d(network, weightMap, *conv->getOutput(0), lname + ".1", 1e-5);
        IActivationLayer *relu = network->addActivation(*bn->getOutput(0), ActivationType::kRELU);
        return relu;

    }

    //构建网络
    // Creat the engine using only the API and not any parser.
    ICudaEngine *createEngine(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
    {
        INetworkDefinition *network = builder->createNetworkV2(0U);

        // Create input tensor of shape { 1, 1, 32, 32 } with name INPUT_BLOB_NAME
        ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
        assert(data);

        std::map<std::string, Weights> weightMap = loadWeights("../pfld.wts");
        Weights emptywts{DataType::kFLOAT, nullptr, 0};
        IConvolutionLayer *conv1 = network->addConvolutionNd(*data, 64, DimsHW{3, 3}, weightMap["conv1.weight"],
                                                             emptywts);
        conv1->setStrideNd(DimsHW{2, 2});//设置步长和padding，在conv1后单独设置的，需注意
        conv1->setPaddingNd(DimsHW{1, 1});//
        //展示conv1
        auto h_conv1 = conv1->getOutput(0)->getDimensions();
        std::cout << "conv1: " << h_conv1.d[0] << " " << h_conv1.d[1] << " " << h_conv1.d[2] << std::endl; //
        //
        auto bn1 = addBatchNorm2d(network, weightMap, *conv1->getOutput(0), "bn1", float(1e-5));
        //展示bn1
        auto h_bn1 = bn1->getOutput(0)->getDimensions();
        std::cout << "bn1: " << h_bn1.d[0] << " " << h_bn1.d[1] << " " << h_bn1.d[2] << std::endl; //
        //
        IActivationLayer *relu1 = network->addActivation(*bn1->getOutput(0), ActivationType::kRELU);
        //展示relu1
        auto h_relu1 = relu1->getOutput(0)->getDimensions();
        std::cout << "relu1: " << h_relu1.d[0] << " " << h_relu1.d[1] << " " << h_relu1.d[2] << std::endl; // 64 56 56

        IConvolutionLayer *conv2 = network->addConvolutionNd(*relu1->getOutput(0), 64, DimsHW{3, 3},
                                                             weightMap["conv2.weight"], emptywts);
        conv1->setPaddingNd(DimsHW{1, 1});  //莫忘设置padding
        auto bn2 = addBatchNorm2d(network, weightMap, *conv2->getOutput(0), "bn2", float(1e-5));
        IActivationLayer *relu2 = network->addActivation(*bn2->getOutput(0), ActivationType::kRELU);

        //展示relu2
        auto h_relu2 = relu2->getOutput(0)->getDimensions();
        std::cout << "relu2: " << h_relu2.d[0] << " " << h_relu2.d[1] << " " << h_relu2.d[2] << std::endl; // 64 56 56

        auto conv3_1 = InvertedResidual(network, weightMap, *relu2->getOutput(0), 2, 64, false, float(2 * 64),
                                        "conv3_1");
        auto block3_2 = InvertedResidual(network, weightMap, *conv3_1->getOutput(0), 1, 64, true, float(2 * 64),
                                         "block3_2");
        auto block3_3 = InvertedResidual(network, weightMap, *block3_2->getOutput(0), 1, 64, true, float(2 * 64),
                                         "block3_3");
        auto block3_4 = InvertedResidual(network, weightMap, *block3_3->getOutput(0), 1, 64, true, float(2 * 64),
                                         "block3_4");
        auto block3_5 = InvertedResidual(network, weightMap, *block3_4->getOutput(0), 1, 64, true, float(2 * 64),
                                         "block3_5");
        auto conv4_1 = InvertedResidual(network, weightMap, *block3_5->getOutput(0), 2, 128, false, float(2 * 64),
                                        "conv4_1");

        //展示conv4_1
        auto h_conv4_1 = conv4_1->getOutput(0)->getDimensions();
        std::cout << "conv4_1: " << h_conv4_1.d[0] << " " << h_conv4_1.d[1] << " " << h_conv4_1.d[2]
                  << std::endl; // 128 14 14


        auto conv5_1 = InvertedResidual(network, weightMap, *conv4_1->getOutput(0), 1, 128, false, float(4 * 128),
                                        "conv5_1");
        auto block5_2 = InvertedResidual(network, weightMap, *conv5_1->getOutput(0), 1, 128, true, float(4 * 128),
                                         "block5_2");
        auto block5_3 = InvertedResidual(network, weightMap, *block5_2->getOutput(0), 1, 128, true, float(4 * 128),
                                         "block5_3");
        auto block5_4 = InvertedResidual(network, weightMap, *block5_3->getOutput(0), 1, 128, true, float(4 * 128),
                                         "block5_4");
        auto block5_5 = InvertedResidual(network, weightMap, *block5_4->getOutput(0), 1, 128, true, float(4 * 128),
                                         "block5_5");
        auto block5_6 = InvertedResidual(network, weightMap, *block5_5->getOutput(0), 1, 128, true, float(4 * 128),
                                         "block5_6");
        auto conv6_1 = InvertedResidual(network, weightMap, *block5_6->getOutput(0), 1, 16, false, float(2 * 128),
                                        "conv6_1");

        //展示conv6_1，后面网络兵分三路
        auto h_conv6_1 = conv6_1->getOutput(0)->getDimensions();
        std::cout << "conv6_1: " << h_conv6_1.d[0] << " " << h_conv6_1.d[1] << " " << h_conv6_1.d[2]
                  << std::endl; //16 14 14

        auto conv7 = conv_bn(network, weightMap, *conv6_1->getOutput(0), 2, 32, 3, "conv7");

        //展示conv7 = conv6_1 + conv_bn
        auto h_conv7 = conv7->getOutput(0)->getDimensions();
        std::cout << "conv7: " << h_conv7.d[0] << " " << h_conv7.d[1] << " " << h_conv7.d[2] << std::endl; // 32 7 7

        //conv6_1和conv7后面都加pooling
        IPoolingLayer *avg_pool1 = network->addPoolingNd(*conv6_1->getOutput(0), PoolingType::kAVERAGE, DimsHW{14, 14});
        IPoolingLayer *avg_pool2 = network->addPoolingNd(*conv7->getOutput(0), PoolingType::kAVERAGE, DimsHW{7, 7});

        //conv8加relu
        IConvolutionLayer *conv8 = network->addConvolutionNd(*conv7->getOutput(0), 128, DimsHW{7, 7},
                                                             weightMap["conv8.weight"], emptywts);
        IActivationLayer *relu3 = network->addActivation(*conv8->getOutput(0), ActivationType::kRELU);


        //Resize
        auto h1 = avg_pool1->getOutput(0)->getDimensions(); //第6路，conv6直出
        assert(h1.nbDims == 3);
        std::cout << "6lu: " << h1.d[0] << " " << h1.d[1] << " " << h1.d[2] << std::endl; //16 1 1
        IResizeLayer *resize1 = network->addResize(*avg_pool1->getOutput(0));
        resize1->setOutputDimensions(DimsCHW{h1.d[0] * h1.d[1] * h1.d[2], 1, 1});
        std::cout << "6lu的resize: " << h1.d[0] * h1.d[1] * h1.d[2] << std::endl; //16

        auto h2 = avg_pool2->getOutput(0)->getDimensions();//第7路
        assert(h2.nbDims == 3);
        std::cout << "7lu: " << h2.d[0] << " " << h2.d[1] << " " << h2.d[2] << std::endl;  //32 1 1
        IResizeLayer *resize2 = network->addResize(*avg_pool2->getOutput(0));
        resize2->setOutputDimensions(DimsCHW{h2.d[0] * h2.d[1] * h2.d[2], 1, 1});
        std::cout << "7lu的resize: " << h2.d[0] * h2.d[1] * h2.d[2] << std::endl;  //32

        auto h3 = relu3->getOutput(0)->getDimensions(); //第8路
        assert(h3.nbDims == 3);
        std::cout << "8lu: " << h3.d[0] << " " << h3.d[1] << " " << h3.d[2] << std::endl;  //128 1 1
        IResizeLayer *resize3 = network->addResize(*relu3->getOutput(0));
        resize3->setOutputDimensions(DimsCHW{h3.d[0] * h3.d[1] * h3.d[2], 1, 1});
        std::cout << "8lu的resize: " << h3.d[0] * h3.d[1] * h3.d[2] << std::endl;  //128

        ITensor *inputTensors[] = {resize1->getOutput(0), resize2->getOutput(0), resize3->getOutput(0)};

        //concat  +  FC
        auto cat = network->addConcatenation(inputTensors, 3);
        auto h_cat = cat->getOutput(0)->getDimensions();
        std::cout << "cat3lu: " << h_cat.d[0] << " " << h_cat.d[1] << " " << h_cat.d[2] << std::endl; //应该是176 1 1

        IFullyConnectedLayer *fc = network->addFullyConnected(*cat->getOutput(0), 196, weightMap["fc.weight"],
                                                              weightMap["fc.bias"]);
        assert(fc);

        fc->getOutput(0)->setName(OUTPUT_BLOB_NAME);
        std::cout << "set name out" << std::endl;
        network->markOutput(*fc->getOutput(0));

        // Build engine
        builder->setMaxBatchSize(maxBatchSize);
        // builder->setMaxWorkspaceSize(1 << 20);
        config->setMaxWorkspaceSize(1 << 20);  // 16MB
        ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
        std::cout << "build out" << std::endl;

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

    void doInference(IExecutionContext &context, float *input, float *output, int batchSize)
    {
        const ICudaEngine &engine = context.getEngine();

        // Pointers to input and output device buffers to pass to engine.
        // Engine requires exactly IEngine::getNbBindings() number of buffers.
        assert(engine.getNbBindings() == 2);  //engine需要buffer数量
        void *buffers[2];

        // In order to bind the buffers, we need to know the names of the input and output tensors.
        // Note that indices are guaranteed to be less than IEngine::getNbBindings()
        const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);  //data
        const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);  //prob置信度

        // Create GPU buffers on device
        CHECK(cudaMalloc(&buffers[inputIndex],
                         batchSize * 3 * INPUT_H * INPUT_W * sizeof(float)));  //GPU上输入的空间申请3*112*112
        CHECK(cudaMalloc(&buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float)));  //GPU上输出的空间申请196

        // Create stream
        cudaStream_t stream;   //创建流
        CHECK(cudaStreamCreate(&stream));   //根据stream的地址去找，是否已创建

        // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
        CHECK(cudaMemcpyAsync(buffers[inputIndex], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float),
                              cudaMemcpyHostToDevice, stream));
        context.enqueue(batchSize, buffers, stream, nullptr);
        CHECK(cudaMemcpyAsync(output, buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float),
                              cudaMemcpyDeviceToHost,
                              stream));
        cudaStreamSynchronize(stream);

        // Release stream and buffers
        cudaStreamDestroy(stream);
        CHECK(cudaFree(buffers[inputIndex]));
        CHECK(cudaFree(buffers[outputIndex]));
    }

    IRuntime *runtime;
    ICudaEngine *engine;
    IExecutionContext *context;

    void InitPFLDEngine()
    {
        char *trtModelStream{nullptr};
        size_t size{0};

        std::ifstream file("/home/nvidia/wdq/PFLDGenEngin/build/pfld.engine", std::ios::binary);
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

        runtime = createInferRuntime(gLogger);  //runtime
        assert(runtime != nullptr);
        engine = runtime->deserializeCudaEngine(trtModelStream, size);  //engine
        assert(engine != nullptr);
        context = engine->createExecutionContext();   //context包含engine
        assert(context != nullptr);
        delete[] trtModelStream;
    }

    void ReleasePFLDEngine()
    {
        context->destroy();
        engine->destroy();
        runtime->destroy();
    }

    int AnalyzeOneFace(cv::Mat &frame, float *prob)
    {
        float data[3 * INPUT_H * INPUT_W];

        if (frame.empty())
        {
            cout << "帧数据损坏！" << endl;
            return -1;
        }

        //read img
        cv::Mat img = frame;
        if (img.empty())
        {
            std::cout << "no file" << endl;
        }
        //resize 112
        cv::Mat pr_img(112, 112, CV_8UC3);
        cv::resize(img, pr_img, pr_img.size(), 0, 0);
        //pr_img enter data
        int i = 0;
        for (int row = 0; row < INPUT_H; ++row)
        {
            //cout << "for" << endl;
            uchar *uc_pixel = pr_img.data + row * pr_img.step;
            for (int col = 0; col < INPUT_W; ++col)
            {
                data[i] = (float) uc_pixel[2] / 255.0; //B
                data[i + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0; //G
                data[i + 2 * INPUT_H * INPUT_W] = (float) uc_pixel[0] / 255.0;  //R
                uc_pixel += 3;
                ++i;
            }
        }
        //cal period
        auto start = std::chrono::system_clock::now();
        doInference(*context, data, prob, 1);  //验证，输入data，输出prob，batch为1
        auto end = std::chrono::system_clock::now();
        std::cout << "PFLD RUN time: "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

        return 0;
    }

    // 测试绘制全脸关键点
    void DrawFaceOutput(cv::Mat &frame, float *prob)
    {
        for (int i = 0; i < POINT_NUM; i++)
        {
            cv::Point point(prob[i * 2], prob[i * 2 + 1]);
            cv::circle(frame, YoloV5::get_point(frame, point), 3, cv::Scalar(0x00, 0x00, 0xFF), -1);
        }
    }

}