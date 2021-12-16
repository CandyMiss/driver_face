// #include "yolov5.h"
// #include "../utils/dataset.h"
// #include "../utils/utils.h"

// using namespace std;

// namespace YoloV5
// {
//     static const int INPUT_H = Yolo::INPUT_H;
//     static const int INPUT_W = Yolo::INPUT_W;
//     static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than 1000 boxes that conf >= 0.1

//     static Logger gLogger;

//     static const char *INPUT_BLOB_NAME = "data";
//     static const char *OUTPUT_BLOB_NAME = "prob";

//     // Creat the engine using only the API and not any parser.
//     ICudaEngine *createEngine_s(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
//     {
//         INetworkDefinition *network = builder->createNetworkV2(0U);

//         // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
//         ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, YoloV5::INPUT_H, YoloV5::INPUT_W});
//         assert(data);

//         std::map<std::string, Weights> weightMap = loadWeights("/home/nvidia/wdq/YoloGenEngine/build/yolov5s.wts");
//         Weights emptywts{DataType::kFLOAT, nullptr, 0};

//         // yolov5 backbone
//         auto focus0 = focus(network, weightMap, *data, 3, 32, 3, "model.0");
//         auto conv1 = convBnLeaky(network, weightMap, *focus0->getOutput(0), 64, 3, 2, 1, "model.1");
//         auto bottleneck_CSP2 = bottleneckCSP(network, weightMap, *conv1->getOutput(0), 64, 64, 1, true, 1, 0.5, "model.2");
//         auto conv3 = convBnLeaky(network, weightMap, *bottleneck_CSP2->getOutput(0), 128, 3, 2, 1, "model.3");
//         auto bottleneck_csp4 = bottleneckCSP(network, weightMap, *conv3->getOutput(0), 128, 128, 3, true, 1, 0.5,
//                                              "model.4");
//         auto conv5 = convBnLeaky(network, weightMap, *bottleneck_csp4->getOutput(0), 256, 3, 2, 1, "model.5");
//         auto bottleneck_csp6 = bottleneckCSP(network, weightMap, *conv5->getOutput(0), 256, 256, 3, true, 1, 0.5,
//                                              "model.6");
//         auto conv7 = convBnLeaky(network, weightMap, *bottleneck_csp6->getOutput(0), 512, 3, 2, 1, "model.7");
//         auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 512, 512, 5, 9, 13, "model.8");

//         // yolov5 head
//         auto bottleneck_csp9 = bottleneckCSP(network, weightMap, *spp8->getOutput(0), 512, 512, 1, false, 1, 0.5,
//                                              "model.9");
//         auto conv10 = convBnLeaky(network, weightMap, *bottleneck_csp9->getOutput(0), 256, 1, 1, 1, "model.10");

//         float *deval = reinterpret_cast<float *>(malloc(sizeof(float) * 256 * 2 * 2));
//         for (int i = 0; i < 256 * 2 * 2; i++)
//         {
//             deval[i] = 1.0;
//         }
//         Weights deconvwts11{DataType::kFLOAT, deval, 256 * 2 * 2};
//         IDeconvolutionLayer *deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 256, DimsHW{2, 2}, deconvwts11,
//                                                                     emptywts);
//         deconv11->setStrideNd(DimsHW{2, 2});
//         deconv11->setNbGroups(256);
//         weightMap["deconv11"] = deconvwts11;

//         ITensor *inputTensors12[] = {deconv11->getOutput(0), bottleneck_csp6->getOutput(0)};
//         auto cat12 = network->addConcatenation(inputTensors12, 2);
//         auto bottleneck_csp13 = bottleneckCSP(network, weightMap, *cat12->getOutput(0), 512, 256, 1, false, 1, 0.5,
//                                               "model.13");
//         auto conv14 = convBnLeaky(network, weightMap, *bottleneck_csp13->getOutput(0), 128, 1, 1, 1, "model.14");

//         Weights deconvwts15{DataType::kFLOAT, deval, 128 * 2 * 2};
//         IDeconvolutionLayer *deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 128, DimsHW{2, 2}, deconvwts15,
//                                                                     emptywts);
//         deconv15->setStrideNd(DimsHW{2, 2});
//         deconv15->setNbGroups(128);
//         //weightMap["deconv15"] = deconvwts15;

//         ITensor *inputTensors16[] = {deconv15->getOutput(0), bottleneck_csp4->getOutput(0)};
//         auto cat16 = network->addConcatenation(inputTensors16, 2);
//         auto bottleneck_csp17 = bottleneckCSP(network, weightMap, *cat16->getOutput(0), 256, 128, 1, false, 1, 0.5,
//                                               "model.17");
//         IConvolutionLayer *conv18 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
//                                                               weightMap["model.24.m.0.bias"]);

//         auto conv19 = convBnLeaky(network, weightMap, *bottleneck_csp17->getOutput(0), 128, 3, 2, 1, "model.18");
//         ITensor *inputTensors20[] = {conv19->getOutput(0), conv14->getOutput(0)};
//         auto cat20 = network->addConcatenation(inputTensors20, 2);
//         auto bottleneck_csp21 = bottleneckCSP(network, weightMap, *cat20->getOutput(0), 256, 256, 1, false, 1, 0.5,
//                                               "model.20");
//         IConvolutionLayer *conv22 = network->addConvolutionNd(*bottleneck_csp21->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
//                                                               weightMap["model.24.m.1.bias"]);

//         auto conv23 = convBnLeaky(network, weightMap, *bottleneck_csp21->getOutput(0), 256, 3, 2, 1, "model.21");
//         ITensor *inputTensors24[] = {conv23->getOutput(0), conv10->getOutput(0)};
//         auto cat24 = network->addConcatenation(inputTensors24, 2);
//         auto bottleneck_csp25 = bottleneckCSP(network, weightMap, *cat24->getOutput(0), 512, 512, 1, false, 1, 0.5,
//                                               "model.23");
//         IConvolutionLayer *conv26 = network->addConvolutionNd(*bottleneck_csp25->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
//                                                               weightMap["model.24.m.2.bias"]);

//         auto creator = getPluginRegistry()->getPluginCreator("YoloLayer_TRT", "1");
//         const PluginFieldCollection *pluginData = creator->getFieldNames();
//         IPluginV2 *pluginObj = creator->createPlugin("yololayer", pluginData);
//         ITensor *inputTensors_yolo[] = {conv26->getOutput(0), conv22->getOutput(0), conv18->getOutput(0)};
//         auto yolo = network->addPluginV2(inputTensors_yolo, 3, *pluginObj);

//         yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
//         network->markOutput(*yolo->getOutput(0));

//         // Build engine
//         builder->setMaxBatchSize(maxBatchSize);
//         config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
//     #ifdef USE_FP16
//         config->setFlag(BuilderFlag::kFP16);
//     #endif
//         std::cout << "Building engine, please wait for a while..." << std::endl;
//         ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
//         std::cout << "Build engine successfully!" << std::endl;

//         // Don't need the network any more
//         network->destroy();

//         // Release host memory
//         for (auto &mem : weightMap)
//         {
//             free((void *) (mem.second.values));
//         }

//         return engine;
//     }

//     ICudaEngine *createEngine_m(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
//     {
//         INetworkDefinition *network = builder->createNetworkV2(0U);

//         // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
//         ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
//         assert(data);

//         std::map<std::string, Weights> weightMap = loadWeights("../yolov5m.wts");
//         Weights emptywts{DataType::kFLOAT, nullptr, 0};

//         /* ------ yolov5 backbone------ */
//         auto focus0 = focus(network, weightMap, *data, 3, 48, 3, "model.0");
//         auto conv1 = convBnLeaky(network, weightMap, *focus0->getOutput(0), 96, 3, 2, 1, "model.1");
//         auto bottleneck_CSP2 = bottleneckCSP(network, weightMap, *conv1->getOutput(0), 96, 96, 2, true, 1, 0.5, "model.2");
//         auto conv3 = convBnLeaky(network, weightMap, *bottleneck_CSP2->getOutput(0), 192, 3, 2, 1, "model.3");
//         auto bottleneck_csp4 = bottleneckCSP(network, weightMap, *conv3->getOutput(0), 192, 192, 6, true, 1, 0.5,
//                                              "model.4");
//         auto conv5 = convBnLeaky(network, weightMap, *bottleneck_csp4->getOutput(0), 384, 3, 2, 1, "model.5");
//         auto bottleneck_csp6 = bottleneckCSP(network, weightMap, *conv5->getOutput(0), 384, 384, 6, true, 1, 0.5,
//                                              "model.6");
//         auto conv7 = convBnLeaky(network, weightMap, *bottleneck_csp6->getOutput(0), 768, 3, 2, 1, "model.7");
//         auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 768, 768, 5, 9, 13, "model.8");
//         /* ------ yolov5 head ------ */
//         auto bottleneck_csp9 = bottleneckCSP(network, weightMap, *spp8->getOutput(0), 768, 768, 2, false, 1, 0.5,
//                                              "model.9");
//         auto conv10 = convBnLeaky(network, weightMap, *bottleneck_csp9->getOutput(0), 384, 1, 1, 1, "model.10");

//         float *deval = reinterpret_cast<float *>(malloc(sizeof(float) * 384 * 2 * 2));
//         for (int i = 0; i < 384 * 2 * 2; i++)
//         {
//             deval[i] = 1.0;
//         }
//         Weights deconvwts11{DataType::kFLOAT, deval, 384 * 2 * 2};
//         IDeconvolutionLayer *deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 384, DimsHW{2, 2}, deconvwts11,
//                                                                     emptywts);
//         deconv11->setStrideNd(DimsHW{2, 2});
//         deconv11->setNbGroups(384);
//         weightMap["deconv11"] = deconvwts11;
//         ITensor *inputTensors12[] = {deconv11->getOutput(0), bottleneck_csp6->getOutput(0)};
//         auto cat12 = network->addConcatenation(inputTensors12, 2);

//         auto bottleneck_csp13 = bottleneckCSP(network, weightMap, *cat12->getOutput(0), 768, 384, 2, false, 1, 0.5,
//                                               "model.13");

//         auto conv14 = convBnLeaky(network, weightMap, *bottleneck_csp13->getOutput(0), 192, 1, 1, 1, "model.14");

//         Weights deconvwts15{DataType::kFLOAT, deval, 192 * 2 * 2};
//         IDeconvolutionLayer *deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 192, DimsHW{2, 2}, deconvwts15,
//                                                                     emptywts);
//         deconv15->setStrideNd(DimsHW{2, 2});
//         deconv15->setNbGroups(192);

//         ITensor *inputTensors16[] = {deconv15->getOutput(0), bottleneck_csp4->getOutput(0)};
//         auto cat16 = network->addConcatenation(inputTensors16, 2);

//         auto bottleneck_csp17 = bottleneckCSP(network, weightMap, *cat16->getOutput(0), 384, 192, 2, false, 1, 0.5,
//                                               "model.17");

//         //yolo layer 1
//         IConvolutionLayer *conv18 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
//                                                               weightMap["model.24.m.0.bias"]);

//         auto conv19 = convBnLeaky(network, weightMap, *bottleneck_csp17->getOutput(0), 192, 3, 2, 1, "model.18");

//         ITensor *inputTensors20[] = {conv19->getOutput(0), conv14->getOutput(0)};
//         auto cat20 = network->addConcatenation(inputTensors20, 2);

//         auto bottleneck_csp21 = bottleneckCSP(network, weightMap, *cat20->getOutput(0), 384, 384, 2, false, 1, 0.5,
//                                               "model.20");

//         //yolo layer 2
//         IConvolutionLayer *conv22 = network->addConvolutionNd(*bottleneck_csp21->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
//                                                               weightMap["model.24.m.1.bias"]);

//         auto conv23 = convBnLeaky(network, weightMap, *bottleneck_csp21->getOutput(0), 384, 3, 2, 1, "model.21");

//         ITensor *inputTensors24[] = {conv23->getOutput(0), conv10->getOutput(0)};
//         auto cat24 = network->addConcatenation(inputTensors24, 2);

//         auto bottleneck_csp25 = bottleneckCSP(network, weightMap, *cat24->getOutput(0), 768, 768, 2, false, 1, 0.5,
//                                               "model.23");

//         // yolo layer 3
//         IConvolutionLayer *conv26 = network->addConvolutionNd(*bottleneck_csp25->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
//                                                               weightMap["model.24.m.2.bias"]);

//         auto creator = getPluginRegistry()->getPluginCreator("YoloLayer_TRT", "1");
//         const PluginFieldCollection *pluginData = creator->getFieldNames();
//         IPluginV2 *pluginObj = creator->createPlugin("yololayer", pluginData);
//         ITensor *inputTensors_yolo[] = {conv26->getOutput(0), conv22->getOutput(0), conv18->getOutput(0)};
//         auto yolo = network->addPluginV2(inputTensors_yolo, 3, *pluginObj);

//         yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
//         network->markOutput(*yolo->getOutput(0));

//         // Build engine
//         builder->setMaxBatchSize(maxBatchSize);
//         config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
//     #ifdef USE_FP16
//         config->setFlag(BuilderFlag::kFP16);
//     #endif
//         std::cout << "Building engine, please wait for a while..." << std::endl;
//         ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
//         std::cout << "Build engine successfully!" << std::endl;

//         // Don't need the network any more
//         network->destroy();

//         // Release host memory
//         for (auto &mem : weightMap)
//         {
//             free((void *) (mem.second.values));
//         }

//         return engine;
//     }

//     ICudaEngine *createEngine_l(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
//     {
//         INetworkDefinition *network = builder->createNetworkV2(0U);

//         // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
//         ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
//         assert(data);

//         std::map<std::string, Weights> weightMap = loadWeights("../yolov5l.wts");
//         Weights emptywts{DataType::kFLOAT, nullptr, 0};

//         /* ------ yolov5 backbone------ */
//         auto focus0 = focus(network, weightMap, *data, 3, 64, 3, "model.0");
//         auto conv1 = convBnLeaky(network, weightMap, *focus0->getOutput(0), 128, 3, 2, 1, "model.1");
//         auto bottleneck_CSP2 = bottleneckCSP(network, weightMap, *conv1->getOutput(0), 128, 128, 3, true, 1, 0.5,
//                                              "model.2");
//         auto conv3 = convBnLeaky(network, weightMap, *bottleneck_CSP2->getOutput(0), 256, 3, 2, 1, "model.3");
//         auto bottleneck_csp4 = bottleneckCSP(network, weightMap, *conv3->getOutput(0), 256, 256, 9, true, 1, 0.5,
//                                              "model.4");
//         auto conv5 = convBnLeaky(network, weightMap, *bottleneck_csp4->getOutput(0), 512, 3, 2, 1, "model.5");
//         auto bottleneck_csp6 = bottleneckCSP(network, weightMap, *conv5->getOutput(0), 512, 512, 9, true, 1, 0.5,
//                                              "model.6");
//         auto conv7 = convBnLeaky(network, weightMap, *bottleneck_csp6->getOutput(0), 1024, 3, 2, 1, "model.7");
//         auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 1024, 1024, 5, 9, 13, "model.8");

//         /* ------ yolov5 head ------ */
//         auto bottleneck_csp9 = bottleneckCSP(network, weightMap, *spp8->getOutput(0), 1024, 1024, 3, false, 1, 0.5,
//                                              "model.9");
//         auto conv10 = convBnLeaky(network, weightMap, *bottleneck_csp9->getOutput(0), 512, 1, 1, 1, "model.10");

//         float *deval = reinterpret_cast<float *>(malloc(sizeof(float) * 512 * 2 * 2));
//         for (int i = 0; i < 512 * 2 * 2; i++)
//         {
//             deval[i] = 1.0;
//         }
//         Weights deconvwts11{DataType::kFLOAT, deval, 512 * 2 * 2};
//         IDeconvolutionLayer *deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 512, DimsHW{2, 2}, deconvwts11,
//                                                                     emptywts);
//         deconv11->setStrideNd(DimsHW{2, 2});
//         deconv11->setNbGroups(512);
//         weightMap["deconv11"] = deconvwts11;

//         ITensor *inputTensors12[] = {deconv11->getOutput(0), bottleneck_csp6->getOutput(0)};
//         auto cat12 = network->addConcatenation(inputTensors12, 2);
//         auto bottleneck_csp13 = bottleneckCSP(network, weightMap, *cat12->getOutput(0), 1024, 512, 3, false, 1, 0.5,
//                                               "model.13");
//         auto conv14 = convBnLeaky(network, weightMap, *bottleneck_csp13->getOutput(0), 256, 1, 1, 1, "model.14");

//         Weights deconvwts15{DataType::kFLOAT, deval, 256 * 2 * 2};
//         IDeconvolutionLayer *deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 256, DimsHW{2, 2}, deconvwts15,
//                                                                     emptywts);
//         deconv15->setStrideNd(DimsHW{2, 2});
//         deconv15->setNbGroups(256);
//         ITensor *inputTensors16[] = {deconv15->getOutput(0), bottleneck_csp4->getOutput(0)};
//         auto cat16 = network->addConcatenation(inputTensors16, 2);

//         auto bottleneck_csp17 = bottleneckCSP(network, weightMap, *cat16->getOutput(0), 512, 256, 3, false, 1, 0.5,
//                                               "model.17");

//         //yolo layer 1
//         IConvolutionLayer *conv18 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
//                                                               weightMap["model.24.m.0.bias"]);

//         auto conv19 = convBnLeaky(network, weightMap, *bottleneck_csp17->getOutput(0), 256, 3, 2, 1, "model.18");

//         // yolo layer 2
//         ITensor *inputTensors20[] = {conv19->getOutput(0), conv14->getOutput(0)};
//         auto cat20 = network->addConcatenation(inputTensors20, 2);

//         auto bottleneck_csp21 = bottleneckCSP(network, weightMap, *cat20->getOutput(0), 512, 512, 3, false, 1, 0.5,
//                                               "model.20");

//         //yolo layer 3
//         IConvolutionLayer *conv22 = network->addConvolutionNd(*bottleneck_csp21->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
//                                                               weightMap["model.24.m.1.bias"]);

//         auto conv23 = convBnLeaky(network, weightMap, *bottleneck_csp21->getOutput(0), 512, 3, 2, 1, "model.21");

//         ITensor *inputTensors24[] = {conv23->getOutput(0), conv10->getOutput(0)};
//         auto cat24 = network->addConcatenation(inputTensors24, 2);

//         auto bottleneck_csp25 = bottleneckCSP(network, weightMap, *cat24->getOutput(0), 1024, 1024, 3, false, 1, 0.5,
//                                               "model.23");

//         IConvolutionLayer *conv26 = network->addConvolutionNd(*bottleneck_csp25->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
//                                                               weightMap["model.24.m.2.bias"]);

//         auto creator = getPluginRegistry()->getPluginCreator("YoloLayer_TRT", "1");
//         const PluginFieldCollection *pluginData = creator->getFieldNames();
//         IPluginV2 *pluginObj = creator->createPlugin("yololayer", pluginData);
//         ITensor *inputTensors_yolo[] = {conv26->getOutput(0), conv22->getOutput(0), conv18->getOutput(0)};
//         auto yolo = network->addPluginV2(inputTensors_yolo, 3, *pluginObj);

//         yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
//         network->markOutput(*yolo->getOutput(0));

//         // Build engine
//         builder->setMaxBatchSize(maxBatchSize);
//         config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
//     #ifdef USE_FP16
//         config->setFlag(BuilderFlag::kFP16);
//     #endif
//         std::cout << "Building engine, please wait for a while..." << std::endl;
//         ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
//         std::cout << "Build engine successfully!" << std::endl;

//         // Don't need the network any more
//         network->destroy();

//         // Release host memory
//         for (auto &mem : weightMap)
//         {
//             free((void *) (mem.second.values));
//         }

//         return engine;
//     }

//     ICudaEngine *createEngine_x(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
//     {
//         INetworkDefinition *network = builder->createNetworkV2(0U);

//         // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
//         ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
//         assert(data);

//         std::map<std::string, Weights> weightMap = loadWeights("../yolov5x.wts");
//         Weights emptywts{DataType::kFLOAT, nullptr, 0};

//         /* ------ yolov5 backbone------ */
//         auto focus0 = focus(network, weightMap, *data, 3, 80, 3, "model.0");
//         auto conv1 = convBnLeaky(network, weightMap, *focus0->getOutput(0), 160, 3, 2, 1, "model.1");
//         auto bottleneck_CSP2 = bottleneckCSP(network, weightMap, *conv1->getOutput(0), 160, 160, 4, true, 1, 0.5,
//                                              "model.2");
//         auto conv3 = convBnLeaky(network, weightMap, *bottleneck_CSP2->getOutput(0), 320, 3, 2, 1, "model.3");
//         auto bottleneck_csp4 = bottleneckCSP(network, weightMap, *conv3->getOutput(0), 320, 320, 12, true, 1, 0.5,
//                                              "model.4");
//         auto conv5 = convBnLeaky(network, weightMap, *bottleneck_csp4->getOutput(0), 640, 3, 2, 1, "model.5");
//         auto bottleneck_csp6 = bottleneckCSP(network, weightMap, *conv5->getOutput(0), 640, 640, 12, true, 1, 0.5,
//                                              "model.6");
//         auto conv7 = convBnLeaky(network, weightMap, *bottleneck_csp6->getOutput(0), 1280, 3, 2, 1, "model.7");
//         auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 1280, 1280, 5, 9, 13, "model.8");

//         /* ------- yolov5 head ------- */
//         auto bottleneck_csp9 = bottleneckCSP(network, weightMap, *spp8->getOutput(0), 1280, 1280, 4, false, 1, 0.5,
//                                              "model.9");
//         auto conv10 = convBnLeaky(network, weightMap, *bottleneck_csp9->getOutput(0), 640, 1, 1, 1, "model.10");

//         float *deval = reinterpret_cast<float *>(malloc(sizeof(float) * 640 * 2 * 2));
//         for (int i = 0; i < 640 * 2 * 2; i++)
//         {
//             deval[i] = 1.0;
//         }
//         Weights deconvwts11{DataType::kFLOAT, deval, 640 * 2 * 2};
//         IDeconvolutionLayer *deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 640, DimsHW{2, 2}, deconvwts11,
//                                                                     emptywts);
//         deconv11->setStrideNd(DimsHW{2, 2});
//         deconv11->setNbGroups(640);
//         weightMap["deconv11"] = deconvwts11;

//         ITensor *inputTensors12[] = {deconv11->getOutput(0), bottleneck_csp6->getOutput(0)};
//         auto cat12 = network->addConcatenation(inputTensors12, 2);

//         auto bottleneck_csp13 = bottleneckCSP(network, weightMap, *cat12->getOutput(0), 1280, 640, 4, false, 1, 0.5,
//                                               "model.13");
//         auto conv14 = convBnLeaky(network, weightMap, *bottleneck_csp13->getOutput(0), 320, 1, 1, 1, "model.14");

//         Weights deconvwts15{DataType::kFLOAT, deval, 320 * 2 * 2};
//         IDeconvolutionLayer *deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 320, DimsHW{2, 2}, deconvwts15,
//                                                                     emptywts);
//         deconv15->setStrideNd(DimsHW{2, 2});
//         deconv15->setNbGroups(320);
//         ITensor *inputTensors16[] = {deconv15->getOutput(0), bottleneck_csp4->getOutput(0)};
//         auto cat16 = network->addConcatenation(inputTensors16, 2);

//         auto bottleneck_csp17 = bottleneckCSP(network, weightMap, *cat16->getOutput(0), 640, 320, 4, false, 1, 0.5,
//                                               "model.17");

//         // yolo layer 1
//         IConvolutionLayer *conv18 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
//                                                               weightMap["model.24.m.0.bias"]);

//         auto conv19 = convBnLeaky(network, weightMap, *bottleneck_csp17->getOutput(0), 320, 3, 2, 1, "model.18");

//         ITensor *inputTensors20[] = {conv19->getOutput(0), conv14->getOutput(0)};
//         auto cat20 = network->addConcatenation(inputTensors20, 2);

//         auto bottleneck_csp21 = bottleneckCSP(network, weightMap, *cat20->getOutput(0), 640, 640, 4, false, 1, 0.5,
//                                               "model.20");

//         // yolo layer 2
//         IConvolutionLayer *conv22 = network->addConvolutionNd(*bottleneck_csp21->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
//                                                               weightMap["model.24.m.1.bias"]);

//         auto conv23 = convBnLeaky(network, weightMap, *bottleneck_csp21->getOutput(0), 640, 3, 2, 1, "model.21");

//         ITensor *inputTensors24[] = {conv23->getOutput(0), conv10->getOutput(0)};
//         auto cat24 = network->addConcatenation(inputTensors24, 2);

//         auto bottleneck_csp25 = bottleneckCSP(network, weightMap, *cat24->getOutput(0), 1280, 1280, 4, false, 1, 0.5,
//                                               "model.23");

//         // yolo layer 3
//         IConvolutionLayer *conv26 = network->addConvolutionNd(*bottleneck_csp25->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
//                                                               DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
//                                                               weightMap["model.24.m.2.bias"]);

//         auto creator = getPluginRegistry()->getPluginCreator("YoloLayer_TRT", "1");
//         const PluginFieldCollection *pluginData = creator->getFieldNames();
//         IPluginV2 *pluginObj = creator->createPlugin("yololayer", pluginData);
//         ITensor *inputTensors_yolo[] = {conv26->getOutput(0), conv22->getOutput(0), conv18->getOutput(0)};
//         auto yolo = network->addPluginV2(inputTensors_yolo, 3, *pluginObj);

//         yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
//         network->markOutput(*yolo->getOutput(0));

//         // Build engine
//         builder->setMaxBatchSize(maxBatchSize);
//         config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
//     #ifdef USE_FP16
//         config->setFlag(BuilderFlag::kFP16);
//     #endif
//         std::cout << "Building engine, please wait for a while..." << std::endl;
//         ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
//         std::cout << "Build engine successfully!" << std::endl;

//         // Don't need the network any more
//         network->destroy();

//         // Release host memory
//         for (auto &mem : weightMap)
//         {
//             free((void *) (mem.second.values));
//         }

//         return engine;
//     }

//     void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream)
//     {
//         // Create builder
//         IBuilder *builder = createInferBuilder(YoloV5::gLogger);
//         IBuilderConfig *config = builder->createBuilderConfig();

//         // Create model to populate the network, then set the outputs and create an engine
//         ICudaEngine *engine = (CREATENET(NET))(maxBatchSize, builder, config, DataType::kFLOAT);
//         //ICudaEngine* engine = createEngine(maxBatchSize, builder, config, DataType::kFLOAT);
//         assert(engine != nullptr);

//         // Serialize the engine
//         (*modelStream) = engine->serialize();

//         // Close everything down
//         engine->destroy();
//         builder->destroy();
//     }

//     void doInference(IExecutionContext &context, float *input, float *output, int batchSize)
//     {
//         const ICudaEngine &engine = context.getEngine();

//         // Pointers to input and output device buffers to pass to engine.
//         // Engine requires exactly IEngine::getNbBindings() number of buffers.
//         assert(engine.getNbBindings() == 2);
//         void *buffers[2];

//         // In order to bind the buffers, we need to know the names of the input and output tensors.
//         // Note that indices are guaranteed to be less than IEngine::getNbBindings()
//         const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);
//         const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);

//         // Create GPU buffers on device
//         CHECK(cudaMalloc(&buffers[inputIndex], batchSize * 3 * INPUT_H * INPUT_W * sizeof(float)));
//         CHECK(cudaMalloc(&buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float)));

//         // Create stream
//         cudaStream_t stream;
//         CHECK(cudaStreamCreate(&stream));

//         // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
//         CHECK(cudaMemcpyAsync(buffers[inputIndex], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float),
//                               cudaMemcpyHostToDevice, stream));
//         context.enqueue(batchSize, buffers, stream, nullptr);
//         CHECK(cudaMemcpyAsync(output, buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost,
//                               stream));
//         cudaStreamSynchronize(stream);

//         // Release stream and buffers
//         cudaStreamDestroy(stream);
//         CHECK(cudaFree(buffers[inputIndex]));
//         CHECK(cudaFree(buffers[outputIndex]));
//     }

//     IRuntime *runtime;
//     ICudaEngine *engine;
//     IExecutionContext *context;
//     void InitYoloV5Engine()
//     {
//         char *trtModelStream{nullptr};
//         size_t size{0};
//         std::string engine_name = STR2(NET);

//         // 测试使用
//         if (BATCH_SIZE == 2)
//         {
//             engine_name = "yolov5" + engine_name + ".engine";
//         }
//         else
//         {
//             engine_name = "yolov5" + engine_name + "_b1.engine";
//         }

//         std::ifstream file(engine_name, std::ios::binary);
//         if (file.good())
//         {
//             file.seekg(0, file.end);
//             size = file.tellg();
//             file.seekg(0, file.beg);
//             trtModelStream = new char[size];
//             assert(trtModelStream);
//             file.read(trtModelStream, size);
//             file.close();
//         }

//         runtime = createInferRuntime(gLogger);
//         assert(runtime != nullptr);
//         engine = runtime->deserializeCudaEngine(trtModelStream, size);
//         assert(engine != nullptr);
//         context = engine->createExecutionContext();
//         assert(context != nullptr);
//         delete[] trtModelStream;
//     }


//     void ReleaseYoloV5Engine()
//     {
//         context->destroy();
//         engine->destroy();
//         runtime->destroy();
//     }

//     static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
//     static float prob[BATCH_SIZE * OUTPUT_SIZE];

//     vector<vector<Yolo::Detection>> AnalyzeBatch(vector<cv::Mat> &frames)
//     {
//         int batch_size = frames.size();

//         for (int batch_index = 0; batch_index < batch_size; batch_index++)
//         {
//             if (frames[batch_index].empty())
//             {
//                 continue;
//             }
//             cv::Mat pr_img = preprocess_img(frames[batch_index]);

//             int pixel_pos = 0;
//             for (int row = 0; row < INPUT_H; ++row)
//             {
//                 uchar *uc_pixel = pr_img.data + row * pr_img.step;
//                 for (int col = 0; col < INPUT_W; ++col)
//                 {
//                     data[batch_index * 3 * INPUT_H * INPUT_W + pixel_pos] = (float) uc_pixel[2] / 255.0;
//                     data[batch_index * 3 * INPUT_H * INPUT_W + pixel_pos + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
//                     data[batch_index * 3 * INPUT_H * INPUT_W + pixel_pos + 2 * INPUT_H * INPUT_W] =
//                             (float) uc_pixel[0] / 255.0;
//                     uc_pixel += 3;
//                     ++pixel_pos;
//                 }
//             }
//         }

//         // Run inference
//         auto start = std::chrono::system_clock::now();
//         doInference(*context, data, prob, batch_size);
//         auto end = std::chrono::system_clock::now();
//         std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "micro s" << std::endl;

//         vector<vector<Yolo::Detection>> batch_results(batch_size);
//         for (int batch_index = 0; batch_index < batch_size; batch_index++)
//         {
//             auto &res = batch_results[batch_index];
//             nms(res, &prob[batch_index * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);

//             cout << "batch[" << batch_index << "] 有目标数：" << res.size() << endl;
//         }

//         return batch_results;
//     }

//     vector<Yolo::Detection> AnalyzeOneShot(cv::Mat &frame)
//     {
//         int fcount = 0;
//         int frame_count = 0;

//         fcount++;
//         frame_count++;

//         for (int b = 0; b < fcount; b++)
//         {
//             if (frame.empty())
//             {
//                 continue;
//             }
//             cv::Mat pr_img = preprocess_img(frame); // letterbox BGR to RGB

//             int i = 0;
//             for (int row = 0; row < INPUT_H; ++row)
//             {
//                 uchar *uc_pixel = pr_img.data + row * pr_img.step;
//                 for (int col = 0; col < INPUT_W; ++col)
//                 {
//                     data[b * 3 * INPUT_H * INPUT_W + i] = (float) uc_pixel[2] / 255.0;
//                     data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
//                     data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float) uc_pixel[0] / 255.0;
//                     uc_pixel += 3;
//                     ++i;
//                 }
//             }
//         }

//         // Run inference
//         auto start = std::chrono::system_clock::now();
//         doInference(*context, data, prob, 1);
//         auto end = std::chrono::system_clock::now();
//         std::cout << "YOLO v5 s RUN time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "micro s" << std::endl;


//         std::vector<Yolo::Detection> myres;
//         nms(myres, &prob[0], CONF_THRESH, NMS_THRESH);

//         std::vector<std::vector<Yolo::Detection>> batch_res(fcount);
//         for (int b = 0; b < fcount; b++)
//         {
//             auto &res = batch_res[b];
//             nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
//         }

//         return batch_res[0];
//     }

//     // 测试获取yolov5的结果
//     void DrawYoloOutput(cv::Mat &frame, std::vector<Yolo::Detection> result)
//     {
//         for (size_t j = 0; j < result.size(); j++)
//         {
//             cv::Rect r = get_rect(frame, result[j].bbox);
//             cv::rectangle(frame, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
//             cv::putText(frame, GetNameFromID(result[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
//                         cv::Scalar(0xFF, 0xFF, 0xFF), 2);
//         }
//     }

// }
#include "yolov5.h"
#include "../utils/dataset.h"
#include "../utils/utils.h"

using namespace std;

namespace YoloV5
{
    static const int INPUT_H = Yolo::INPUT_H;
    static const int INPUT_W = Yolo::INPUT_W;
    static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than 1000 boxes that conf >= 0.1

    static Logger gLogger;

    static const char *INPUT_BLOB_NAME = "data";
    static const char *OUTPUT_BLOB_NAME = "prob";

    // Creat the engine using only the API and not any parser.
    ICudaEngine *createEngine_s(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
    {
        INetworkDefinition *network = builder->createNetworkV2(0U);

        // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
        ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, YoloV5::INPUT_H, YoloV5::INPUT_W});
        assert(data);

        std::map<std::string, Weights> weightMap = loadWeights("/home/nvidia/wdq/YoloGenEngine/build/yolov5s.wts");
        Weights emptywts{DataType::kFLOAT, nullptr, 0};

        // yolov5 backbone
        auto focus0 = focus(network, weightMap, *data, 3, 32, 3, "model.0");
        auto conv1 = convBnLeaky(network, weightMap, *focus0->getOutput(0), 64, 3, 2, 1, "model.1");
        auto bottleneck_CSP2 = bottleneckCSP(network, weightMap, *conv1->getOutput(0), 64, 64, 1, true, 1, 0.5, "model.2");
        auto conv3 = convBnLeaky(network, weightMap, *bottleneck_CSP2->getOutput(0), 128, 3, 2, 1, "model.3");
        auto bottleneck_csp4 = bottleneckCSP(network, weightMap, *conv3->getOutput(0), 128, 128, 3, true, 1, 0.5,
                                             "model.4");
        auto conv5 = convBnLeaky(network, weightMap, *bottleneck_csp4->getOutput(0), 256, 3, 2, 1, "model.5");
        auto bottleneck_csp6 = bottleneckCSP(network, weightMap, *conv5->getOutput(0), 256, 256, 3, true, 1, 0.5,
                                             "model.6");
        auto conv7 = convBnLeaky(network, weightMap, *bottleneck_csp6->getOutput(0), 512, 3, 2, 1, "model.7");
        auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 512, 512, 5, 9, 13, "model.8");

        // yolov5 head
        auto bottleneck_csp9 = bottleneckCSP(network, weightMap, *spp8->getOutput(0), 512, 512, 1, false, 1, 0.5,
                                             "model.9");
        auto conv10 = convBnLeaky(network, weightMap, *bottleneck_csp9->getOutput(0), 256, 1, 1, 1, "model.10");

        float *deval = reinterpret_cast<float *>(malloc(sizeof(float) * 256 * 2 * 2));
        for (int i = 0; i < 256 * 2 * 2; i++)
        {
            deval[i] = 1.0;
        }
        Weights deconvwts11{DataType::kFLOAT, deval, 256 * 2 * 2};
        IDeconvolutionLayer *deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 256, DimsHW{2, 2}, deconvwts11,
                                                                    emptywts);
        deconv11->setStrideNd(DimsHW{2, 2});
        deconv11->setNbGroups(256);
        weightMap["deconv11"] = deconvwts11;

        ITensor *inputTensors12[] = {deconv11->getOutput(0), bottleneck_csp6->getOutput(0)};
        auto cat12 = network->addConcatenation(inputTensors12, 2);
        auto bottleneck_csp13 = bottleneckCSP(network, weightMap, *cat12->getOutput(0), 512, 256, 1, false, 1, 0.5,
                                              "model.13");
        auto conv14 = convBnLeaky(network, weightMap, *bottleneck_csp13->getOutput(0), 128, 1, 1, 1, "model.14");

        Weights deconvwts15{DataType::kFLOAT, deval, 128 * 2 * 2};
        IDeconvolutionLayer *deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 128, DimsHW{2, 2}, deconvwts15,
                                                                    emptywts);
        deconv15->setStrideNd(DimsHW{2, 2});
        deconv15->setNbGroups(128);
        //weightMap["deconv15"] = deconvwts15;

        ITensor *inputTensors16[] = {deconv15->getOutput(0), bottleneck_csp4->getOutput(0)};
        auto cat16 = network->addConcatenation(inputTensors16, 2);
        auto bottleneck_csp17 = bottleneckCSP(network, weightMap, *cat16->getOutput(0), 256, 128, 1, false, 1, 0.5,
                                              "model.17");
        IConvolutionLayer *conv18 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
                                                              weightMap["model.24.m.0.bias"]);

        auto conv19 = convBnLeaky(network, weightMap, *bottleneck_csp17->getOutput(0), 128, 3, 2, 1, "model.18");
        ITensor *inputTensors20[] = {conv19->getOutput(0), conv14->getOutput(0)};
        auto cat20 = network->addConcatenation(inputTensors20, 2);
        auto bottleneck_csp21 = bottleneckCSP(network, weightMap, *cat20->getOutput(0), 256, 256, 1, false, 1, 0.5,
                                              "model.20");
        IConvolutionLayer *conv22 = network->addConvolutionNd(*bottleneck_csp21->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
                                                              weightMap["model.24.m.1.bias"]);

        auto conv23 = convBnLeaky(network, weightMap, *bottleneck_csp21->getOutput(0), 256, 3, 2, 1, "model.21");
        ITensor *inputTensors24[] = {conv23->getOutput(0), conv10->getOutput(0)};
        auto cat24 = network->addConcatenation(inputTensors24, 2);
        auto bottleneck_csp25 = bottleneckCSP(network, weightMap, *cat24->getOutput(0), 512, 512, 1, false, 1, 0.5,
                                              "model.23");
        IConvolutionLayer *conv26 = network->addConvolutionNd(*bottleneck_csp25->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
                                                              weightMap["model.24.m.2.bias"]);

        auto creator = getPluginRegistry()->getPluginCreator("YoloLayer_TRT", "1");
        const PluginFieldCollection *pluginData = creator->getFieldNames();
        IPluginV2 *pluginObj = creator->createPlugin("yololayer", pluginData);
        ITensor *inputTensors_yolo[] = {conv26->getOutput(0), conv22->getOutput(0), conv18->getOutput(0)};
        auto yolo = network->addPluginV2(inputTensors_yolo, 3, *pluginObj);

        yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
        network->markOutput(*yolo->getOutput(0));

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

    ICudaEngine *createEngine_m(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
    {
        INetworkDefinition *network = builder->createNetworkV2(0U);

        // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
        ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
        assert(data);

        std::map<std::string, Weights> weightMap = loadWeights("../yolov5m.wts");
        Weights emptywts{DataType::kFLOAT, nullptr, 0};

        /* ------ yolov5 backbone------ */
        auto focus0 = focus(network, weightMap, *data, 3, 48, 3, "model.0");
        auto conv1 = convBnLeaky(network, weightMap, *focus0->getOutput(0), 96, 3, 2, 1, "model.1");
        auto bottleneck_CSP2 = bottleneckCSP(network, weightMap, *conv1->getOutput(0), 96, 96, 2, true, 1, 0.5, "model.2");
        auto conv3 = convBnLeaky(network, weightMap, *bottleneck_CSP2->getOutput(0), 192, 3, 2, 1, "model.3");
        auto bottleneck_csp4 = bottleneckCSP(network, weightMap, *conv3->getOutput(0), 192, 192, 6, true, 1, 0.5,
                                             "model.4");
        auto conv5 = convBnLeaky(network, weightMap, *bottleneck_csp4->getOutput(0), 384, 3, 2, 1, "model.5");
        auto bottleneck_csp6 = bottleneckCSP(network, weightMap, *conv5->getOutput(0), 384, 384, 6, true, 1, 0.5,
                                             "model.6");
        auto conv7 = convBnLeaky(network, weightMap, *bottleneck_csp6->getOutput(0), 768, 3, 2, 1, "model.7");
        auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 768, 768, 5, 9, 13, "model.8");
        /* ------ yolov5 head ------ */
        auto bottleneck_csp9 = bottleneckCSP(network, weightMap, *spp8->getOutput(0), 768, 768, 2, false, 1, 0.5,
                                             "model.9");
        auto conv10 = convBnLeaky(network, weightMap, *bottleneck_csp9->getOutput(0), 384, 1, 1, 1, "model.10");

        float *deval = reinterpret_cast<float *>(malloc(sizeof(float) * 384 * 2 * 2));
        for (int i = 0; i < 384 * 2 * 2; i++)
        {
            deval[i] = 1.0;
        }
        Weights deconvwts11{DataType::kFLOAT, deval, 384 * 2 * 2};
        IDeconvolutionLayer *deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 384, DimsHW{2, 2}, deconvwts11,
                                                                    emptywts);
        deconv11->setStrideNd(DimsHW{2, 2});
        deconv11->setNbGroups(384);
        weightMap["deconv11"] = deconvwts11;
        ITensor *inputTensors12[] = {deconv11->getOutput(0), bottleneck_csp6->getOutput(0)};
        auto cat12 = network->addConcatenation(inputTensors12, 2);

        auto bottleneck_csp13 = bottleneckCSP(network, weightMap, *cat12->getOutput(0), 768, 384, 2, false, 1, 0.5,
                                              "model.13");

        auto conv14 = convBnLeaky(network, weightMap, *bottleneck_csp13->getOutput(0), 192, 1, 1, 1, "model.14");

        Weights deconvwts15{DataType::kFLOAT, deval, 192 * 2 * 2};
        IDeconvolutionLayer *deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 192, DimsHW{2, 2}, deconvwts15,
                                                                    emptywts);
        deconv15->setStrideNd(DimsHW{2, 2});
        deconv15->setNbGroups(192);

        ITensor *inputTensors16[] = {deconv15->getOutput(0), bottleneck_csp4->getOutput(0)};
        auto cat16 = network->addConcatenation(inputTensors16, 2);

        auto bottleneck_csp17 = bottleneckCSP(network, weightMap, *cat16->getOutput(0), 384, 192, 2, false, 1, 0.5,
                                              "model.17");

        //yolo layer 1
        IConvolutionLayer *conv18 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
                                                              weightMap["model.24.m.0.bias"]);

        auto conv19 = convBnLeaky(network, weightMap, *bottleneck_csp17->getOutput(0), 192, 3, 2, 1, "model.18");

        ITensor *inputTensors20[] = {conv19->getOutput(0), conv14->getOutput(0)};
        auto cat20 = network->addConcatenation(inputTensors20, 2);

        auto bottleneck_csp21 = bottleneckCSP(network, weightMap, *cat20->getOutput(0), 384, 384, 2, false, 1, 0.5,
                                              "model.20");

        //yolo layer 2
        IConvolutionLayer *conv22 = network->addConvolutionNd(*bottleneck_csp21->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
                                                              weightMap["model.24.m.1.bias"]);

        auto conv23 = convBnLeaky(network, weightMap, *bottleneck_csp21->getOutput(0), 384, 3, 2, 1, "model.21");

        ITensor *inputTensors24[] = {conv23->getOutput(0), conv10->getOutput(0)};
        auto cat24 = network->addConcatenation(inputTensors24, 2);

        auto bottleneck_csp25 = bottleneckCSP(network, weightMap, *cat24->getOutput(0), 768, 768, 2, false, 1, 0.5,
                                              "model.23");

        // yolo layer 3
        IConvolutionLayer *conv26 = network->addConvolutionNd(*bottleneck_csp25->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
                                                              weightMap["model.24.m.2.bias"]);

        auto creator = getPluginRegistry()->getPluginCreator("YoloLayer_TRT", "1");
        const PluginFieldCollection *pluginData = creator->getFieldNames();
        IPluginV2 *pluginObj = creator->createPlugin("yololayer", pluginData);
        ITensor *inputTensors_yolo[] = {conv26->getOutput(0), conv22->getOutput(0), conv18->getOutput(0)};
        auto yolo = network->addPluginV2(inputTensors_yolo, 3, *pluginObj);

        yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
        network->markOutput(*yolo->getOutput(0));

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

    ICudaEngine *createEngine_l(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
    {
        INetworkDefinition *network = builder->createNetworkV2(0U);

        // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
        ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
        assert(data);

        std::map<std::string, Weights> weightMap = loadWeights("../yolov5l.wts");
        Weights emptywts{DataType::kFLOAT, nullptr, 0};

        /* ------ yolov5 backbone------ */
        auto focus0 = focus(network, weightMap, *data, 3, 64, 3, "model.0");
        auto conv1 = convBnLeaky(network, weightMap, *focus0->getOutput(0), 128, 3, 2, 1, "model.1");
        auto bottleneck_CSP2 = bottleneckCSP(network, weightMap, *conv1->getOutput(0), 128, 128, 3, true, 1, 0.5,
                                             "model.2");
        auto conv3 = convBnLeaky(network, weightMap, *bottleneck_CSP2->getOutput(0), 256, 3, 2, 1, "model.3");
        auto bottleneck_csp4 = bottleneckCSP(network, weightMap, *conv3->getOutput(0), 256, 256, 9, true, 1, 0.5,
                                             "model.4");
        auto conv5 = convBnLeaky(network, weightMap, *bottleneck_csp4->getOutput(0), 512, 3, 2, 1, "model.5");
        auto bottleneck_csp6 = bottleneckCSP(network, weightMap, *conv5->getOutput(0), 512, 512, 9, true, 1, 0.5,
                                             "model.6");
        auto conv7 = convBnLeaky(network, weightMap, *bottleneck_csp6->getOutput(0), 1024, 3, 2, 1, "model.7");
        auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 1024, 1024, 5, 9, 13, "model.8");

        /* ------ yolov5 head ------ */
        auto bottleneck_csp9 = bottleneckCSP(network, weightMap, *spp8->getOutput(0), 1024, 1024, 3, false, 1, 0.5,
                                             "model.9");
        auto conv10 = convBnLeaky(network, weightMap, *bottleneck_csp9->getOutput(0), 512, 1, 1, 1, "model.10");

        float *deval = reinterpret_cast<float *>(malloc(sizeof(float) * 512 * 2 * 2));
        for (int i = 0; i < 512 * 2 * 2; i++)
        {
            deval[i] = 1.0;
        }
        Weights deconvwts11{DataType::kFLOAT, deval, 512 * 2 * 2};
        IDeconvolutionLayer *deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 512, DimsHW{2, 2}, deconvwts11,
                                                                    emptywts);
        deconv11->setStrideNd(DimsHW{2, 2});
        deconv11->setNbGroups(512);
        weightMap["deconv11"] = deconvwts11;

        ITensor *inputTensors12[] = {deconv11->getOutput(0), bottleneck_csp6->getOutput(0)};
        auto cat12 = network->addConcatenation(inputTensors12, 2);
        auto bottleneck_csp13 = bottleneckCSP(network, weightMap, *cat12->getOutput(0), 1024, 512, 3, false, 1, 0.5,
                                              "model.13");
        auto conv14 = convBnLeaky(network, weightMap, *bottleneck_csp13->getOutput(0), 256, 1, 1, 1, "model.14");

        Weights deconvwts15{DataType::kFLOAT, deval, 256 * 2 * 2};
        IDeconvolutionLayer *deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 256, DimsHW{2, 2}, deconvwts15,
                                                                    emptywts);
        deconv15->setStrideNd(DimsHW{2, 2});
        deconv15->setNbGroups(256);
        ITensor *inputTensors16[] = {deconv15->getOutput(0), bottleneck_csp4->getOutput(0)};
        auto cat16 = network->addConcatenation(inputTensors16, 2);

        auto bottleneck_csp17 = bottleneckCSP(network, weightMap, *cat16->getOutput(0), 512, 256, 3, false, 1, 0.5,
                                              "model.17");

        //yolo layer 1
        IConvolutionLayer *conv18 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
                                                              weightMap["model.24.m.0.bias"]);

        auto conv19 = convBnLeaky(network, weightMap, *bottleneck_csp17->getOutput(0), 256, 3, 2, 1, "model.18");

        // yolo layer 2
        ITensor *inputTensors20[] = {conv19->getOutput(0), conv14->getOutput(0)};
        auto cat20 = network->addConcatenation(inputTensors20, 2);

        auto bottleneck_csp21 = bottleneckCSP(network, weightMap, *cat20->getOutput(0), 512, 512, 3, false, 1, 0.5,
                                              "model.20");

        //yolo layer 3
        IConvolutionLayer *conv22 = network->addConvolutionNd(*bottleneck_csp21->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
                                                              weightMap["model.24.m.1.bias"]);

        auto conv23 = convBnLeaky(network, weightMap, *bottleneck_csp21->getOutput(0), 512, 3, 2, 1, "model.21");

        ITensor *inputTensors24[] = {conv23->getOutput(0), conv10->getOutput(0)};
        auto cat24 = network->addConcatenation(inputTensors24, 2);

        auto bottleneck_csp25 = bottleneckCSP(network, weightMap, *cat24->getOutput(0), 1024, 1024, 3, false, 1, 0.5,
                                              "model.23");

        IConvolutionLayer *conv26 = network->addConvolutionNd(*bottleneck_csp25->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
                                                              weightMap["model.24.m.2.bias"]);

        auto creator = getPluginRegistry()->getPluginCreator("YoloLayer_TRT", "1");
        const PluginFieldCollection *pluginData = creator->getFieldNames();
        IPluginV2 *pluginObj = creator->createPlugin("yololayer", pluginData);
        ITensor *inputTensors_yolo[] = {conv26->getOutput(0), conv22->getOutput(0), conv18->getOutput(0)};
        auto yolo = network->addPluginV2(inputTensors_yolo, 3, *pluginObj);

        yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
        network->markOutput(*yolo->getOutput(0));

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

    ICudaEngine *createEngine_x(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt)
    {
        INetworkDefinition *network = builder->createNetworkV2(0U);

        // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
        ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
        assert(data);

        std::map<std::string, Weights> weightMap = loadWeights("../yolov5x.wts");
        Weights emptywts{DataType::kFLOAT, nullptr, 0};

        /* ------ yolov5 backbone------ */
        auto focus0 = focus(network, weightMap, *data, 3, 80, 3, "model.0");
        auto conv1 = convBnLeaky(network, weightMap, *focus0->getOutput(0), 160, 3, 2, 1, "model.1");
        auto bottleneck_CSP2 = bottleneckCSP(network, weightMap, *conv1->getOutput(0), 160, 160, 4, true, 1, 0.5,
                                             "model.2");
        auto conv3 = convBnLeaky(network, weightMap, *bottleneck_CSP2->getOutput(0), 320, 3, 2, 1, "model.3");
        auto bottleneck_csp4 = bottleneckCSP(network, weightMap, *conv3->getOutput(0), 320, 320, 12, true, 1, 0.5,
                                             "model.4");
        auto conv5 = convBnLeaky(network, weightMap, *bottleneck_csp4->getOutput(0), 640, 3, 2, 1, "model.5");
        auto bottleneck_csp6 = bottleneckCSP(network, weightMap, *conv5->getOutput(0), 640, 640, 12, true, 1, 0.5,
                                             "model.6");
        auto conv7 = convBnLeaky(network, weightMap, *bottleneck_csp6->getOutput(0), 1280, 3, 2, 1, "model.7");
        auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 1280, 1280, 5, 9, 13, "model.8");

        /* ------- yolov5 head ------- */
        auto bottleneck_csp9 = bottleneckCSP(network, weightMap, *spp8->getOutput(0), 1280, 1280, 4, false, 1, 0.5,
                                             "model.9");
        auto conv10 = convBnLeaky(network, weightMap, *bottleneck_csp9->getOutput(0), 640, 1, 1, 1, "model.10");

        float *deval = reinterpret_cast<float *>(malloc(sizeof(float) * 640 * 2 * 2));
        for (int i = 0; i < 640 * 2 * 2; i++)
        {
            deval[i] = 1.0;
        }
        Weights deconvwts11{DataType::kFLOAT, deval, 640 * 2 * 2};
        IDeconvolutionLayer *deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 640, DimsHW{2, 2}, deconvwts11,
                                                                    emptywts);
        deconv11->setStrideNd(DimsHW{2, 2});
        deconv11->setNbGroups(640);
        weightMap["deconv11"] = deconvwts11;

        ITensor *inputTensors12[] = {deconv11->getOutput(0), bottleneck_csp6->getOutput(0)};
        auto cat12 = network->addConcatenation(inputTensors12, 2);

        auto bottleneck_csp13 = bottleneckCSP(network, weightMap, *cat12->getOutput(0), 1280, 640, 4, false, 1, 0.5,
                                              "model.13");
        auto conv14 = convBnLeaky(network, weightMap, *bottleneck_csp13->getOutput(0), 320, 1, 1, 1, "model.14");

        Weights deconvwts15{DataType::kFLOAT, deval, 320 * 2 * 2};
        IDeconvolutionLayer *deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 320, DimsHW{2, 2}, deconvwts15,
                                                                    emptywts);
        deconv15->setStrideNd(DimsHW{2, 2});
        deconv15->setNbGroups(320);
        ITensor *inputTensors16[] = {deconv15->getOutput(0), bottleneck_csp4->getOutput(0)};
        auto cat16 = network->addConcatenation(inputTensors16, 2);

        auto bottleneck_csp17 = bottleneckCSP(network, weightMap, *cat16->getOutput(0), 640, 320, 4, false, 1, 0.5,
                                              "model.17");

        // yolo layer 1
        IConvolutionLayer *conv18 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
                                                              weightMap["model.24.m.0.bias"]);

        auto conv19 = convBnLeaky(network, weightMap, *bottleneck_csp17->getOutput(0), 320, 3, 2, 1, "model.18");

        ITensor *inputTensors20[] = {conv19->getOutput(0), conv14->getOutput(0)};
        auto cat20 = network->addConcatenation(inputTensors20, 2);

        auto bottleneck_csp21 = bottleneckCSP(network, weightMap, *cat20->getOutput(0), 640, 640, 4, false, 1, 0.5,
                                              "model.20");

        // yolo layer 2
        IConvolutionLayer *conv22 = network->addConvolutionNd(*bottleneck_csp21->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
                                                              weightMap["model.24.m.1.bias"]);

        auto conv23 = convBnLeaky(network, weightMap, *bottleneck_csp21->getOutput(0), 640, 3, 2, 1, "model.21");

        ITensor *inputTensors24[] = {conv23->getOutput(0), conv10->getOutput(0)};
        auto cat24 = network->addConcatenation(inputTensors24, 2);

        auto bottleneck_csp25 = bottleneckCSP(network, weightMap, *cat24->getOutput(0), 1280, 1280, 4, false, 1, 0.5,
                                              "model.23");

        // yolo layer 3
        IConvolutionLayer *conv26 = network->addConvolutionNd(*bottleneck_csp25->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                              DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
                                                              weightMap["model.24.m.2.bias"]);

        auto creator = getPluginRegistry()->getPluginCreator("YoloLayer_TRT", "1");
        const PluginFieldCollection *pluginData = creator->getFieldNames();
        IPluginV2 *pluginObj = creator->createPlugin("yololayer", pluginData);
        ITensor *inputTensors_yolo[] = {conv26->getOutput(0), conv22->getOutput(0), conv18->getOutput(0)};
        auto yolo = network->addPluginV2(inputTensors_yolo, 3, *pluginObj);

        yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
        network->markOutput(*yolo->getOutput(0));

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
        IBuilder *builder = createInferBuilder(YoloV5::gLogger);
        IBuilderConfig *config = builder->createBuilderConfig();

        // Create model to populate the network, then set the outputs and create an engine
        ICudaEngine *engine = (CREATENET(NET))(maxBatchSize, builder, config, DataType::kFLOAT);
        //ICudaEngine* engine = createEngine(maxBatchSize, builder, config, DataType::kFLOAT);
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
        assert(engine.getNbBindings() == 2);
        void *buffers[2];

        // In order to bind the buffers, we need to know the names of the input and output tensors.
        // Note that indices are guaranteed to be less than IEngine::getNbBindings()
        const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);
        const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);

        // Create GPU buffers on device
        CHECK(cudaMalloc(&buffers[inputIndex], batchSize * 3 * INPUT_H * INPUT_W * sizeof(float)));
        CHECK(cudaMalloc(&buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float)));

        // Create stream
        cudaStream_t stream;
        CHECK(cudaStreamCreate(&stream));

        // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
        CHECK(cudaMemcpyAsync(buffers[inputIndex], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float),
                              cudaMemcpyHostToDevice, stream));
        context.enqueue(batchSize, buffers, stream, nullptr);
        CHECK(cudaMemcpyAsync(output, buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost,
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
    void InitYoloV5Engine()
    {
        char *trtModelStream{nullptr};
        size_t size{0};
        std::string engine_name = STR2(NET);

        // 测试使用
        if (BATCH_SIZE == 2)
        {
            engine_name = ENGIN_PATH + "yolov5" + engine_name + ".engine";
            cout << "使用 BATCH_SIZE = 2 的引擎: " << engine_name << endl;
        }
        else
        {
            engine_name = ENGIN_PATH + "yolov5" + engine_name + "_b1.engine";
            cout << "使用 BATCH_SIZE = 1 的引擎: " << engine_name << endl;
        }

        std::ifstream file(engine_name, std::ios::binary);
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


    void ReleaseYoloV5Engine()
    {
        context->destroy();
        engine->destroy();
        runtime->destroy();
    }

    static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
    static float prob[BATCH_SIZE * OUTPUT_SIZE];

    vector<vector<Yolo::Detection>> AnalyzeBatch(vector<cv::Mat> &frames)
    {
        int batch_size = frames.size();

        for (int batch_index = 0; batch_index < batch_size; batch_index++)
        {
            if (frames[batch_index].empty())
            {
                continue;
            }
            cv::Mat pr_img = preprocess_img(frames[batch_index]);

            int pixel_pos = 0;
            for (int row = 0; row < INPUT_H; ++row)
            {
                uchar *uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col)
                {
                    data[batch_index * 3 * INPUT_H * INPUT_W + pixel_pos] = (float) uc_pixel[2] / 255.0;
                    data[batch_index * 3 * INPUT_H * INPUT_W + pixel_pos + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
                    data[batch_index * 3 * INPUT_H * INPUT_W + pixel_pos + 2 * INPUT_H * INPUT_W] =
                            (float) uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++pixel_pos;
                }
            }
        }

        // Run inference
        auto start = std::chrono::system_clock::now();
        doInference(*context, data, prob, batch_size);
        auto end = std::chrono::system_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

        vector<vector<Yolo::Detection>> batch_results(batch_size);
        for (int batch_index = 0; batch_index < batch_size; batch_index++)
        {
            auto &res = batch_results[batch_index];
            nms(res, &prob[batch_index * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);

            cout << "batch[" << batch_index << "] 有目标数：" << res.size() << endl;
        }

        return batch_results;
    }

    vector<Yolo::Detection> AnalyzeOneShot(cv::Mat &frame)
    {
        int fcount = 0;
        int frame_count = 0;

        fcount++;
        frame_count++;

        for (int b = 0; b < fcount; b++)
        {
            if (frame.empty())
            {
                continue;
            }
            cv::Mat pr_img = preprocess_img(frame); // letterbox BGR to RGB

            int i = 0;
            for (int row = 0; row < INPUT_H; ++row)
            {
                uchar *uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col)
                {
                    data[b * 3 * INPUT_H * INPUT_W + i] = (float) uc_pixel[2] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float) uc_pixel[1] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float) uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }
        }

        // Run inference
        auto start = std::chrono::system_clock::now();
        doInference(*context, data, prob, 1);
        auto end = std::chrono::system_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

        std::vector<Yolo::Detection> myres;
        nms(myres, &prob[0], CONF_THRESH, NMS_THRESH);

        std::vector<std::vector<Yolo::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++)
        {
            auto &res = batch_res[b];
            nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
        }

        return batch_res[0];
    }

    // 测试获取yolov5的结果
    void DrawYoloOutput(cv::Mat &frame, std::vector<Yolo::Detection> result)
    {
        for (size_t j = 0; j < result.size(); j++)
        {
            cv::Rect r = get_rect(frame, result[j].bbox);
            cv::rectangle(frame, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(frame, GetNameFromID(result[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                        cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }
    }

}