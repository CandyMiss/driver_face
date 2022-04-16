#include "cfg_params.h"

using namespace std;

const string ConfigXmlParams::cfgFilename = "cfg.xml";
ModuleConfigs ConfigXmlParams::Configs; // 静态变量一定要在class外部初始化

void ConfigXmlParams::Init()
{
    readModuleConfigs();
}

bool ConfigXmlParams::Set_RECOGNITION_TIME_PERIOD(int time)
{
    Configs.FaceRecognitionCfg.RECOGNITION_TIME_PERIOD = time;
    return saveModuleConfigs();
}

bool ConfigXmlParams::Set_EYES_OCCLUSION_TIME_THRD(int time)
{
    Configs.FaceOcclusionCfg.EYES_OCCLUSION_TIME_THRD = time;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_MOUTH_OCCLUSION_TIME_THRD(int time)
{
    Configs.FaceOcclusionCfg.MOUTH_OCCLUSION_TIME_THRD = time;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_FACE_ANGLE_HORIZ_TIME_THRD(int time)
{
    Configs.AbnormalBehaviorCfg.FACE_ANGLE_HORIZ_TIME_THRD = time;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_FACE_ANGLE_VIRT_TIME_THRD(int time)
{
    Configs.AbnormalBehaviorCfg.FACE_ANGLE_VIRT_TIME_THRD = time;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_ANGLE_HORIZ_THRD(double angle)
{
    Configs.AbnormalBehaviorCfg.ANGLE_HORIZ_THRD = angle;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_ANGLE_VERT_THRD(double angle)
{
    Configs.AbnormalBehaviorCfg.ANGLE_VERT_THRD = angle;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_EYE_CLOSE_TIME_THRD(int time)
{
    Configs.FatigueStateCfg.EYE_CLOSE_TIME_THRD = time;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_MOUTH_OPEN_TIME_THRD(int time)
{
    Configs.FatigueStateCfg.MOUTH_OPEN_TIME_THRD = time;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_EYE_DGREE_THRD(double dgree)
{
    Configs.FatigueStateCfg.EYE_DGREE_THRD = dgree;
    return saveModuleConfigs();
}
bool ConfigXmlParams::Set_MOUTH_DGREE_THRD(double dgree)
{
    Configs.FatigueStateCfg.MOUTH_DGREE_THRD = dgree;
    return saveModuleConfigs();
}

bool ConfigXmlParams::readModuleConfigs()
{
    xmlDocPtr pDoc = xmlReadFile(cfgFilename.c_str(), "UTF-8", XML_PARSE_RECOVER); //获取XML文档的指针
    if (NULL == pDoc)
    {
        cout << "解析配置文件失败！" << endl;
        fprintf(stderr, "xmlParseFile Error in %s %d\n", __FUNCTION__, __LINE__);
        return false;
    }

    xmlNodePtr pRoot = xmlDocGetRootElement(pDoc); //获取根节点
    if (NULL == pRoot)
    {
        cout << "xmlDoc获取根节点错误: " << endl;
        fprintf(stderr, "xmlDocGetRootElement Error in %s %d\n", __FUNCTION__, __LINE__);
        xmlFreeDoc(pDoc);
        return false;
    }

    xmlNodePtr pModule = pRoot->xmlChildrenNode;
    while (NULL != pModule)
    {
        parseModule(pModule);
        pModule = pModule->next;
    }
    xmlFreeDoc(pDoc);

    return true;
}

bool ConfigXmlParams::saveModuleConfigs()
{
    //建立XML文档和根结点
    xmlDocPtr doc = xmlNewDoc(BAD_CAST"1.0");
    xmlNodePtr root = xmlNewNode(NULL, BAD_CAST"config");

    //将根节点绑定到ML文档
    xmlDocSetRootElement(doc, root);

    // 处理 FaceRecognition
    xmlNodePtr nodeModule = xmlNewNode(NULL, BAD_CAST"module");
    xmlNewProp(nodeModule, BAD_CAST"name", BAD_CAST"FaceRecognition");
    xmlAddChild(root, nodeModule);

    xmlNodePtr nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"RECOGNITION_TIME_PERIOD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.FaceRecognitionCfg.RECOGNITION_TIME_PERIOD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    // 处理 FaceOcclusion
    nodeModule = xmlNewNode(NULL, BAD_CAST"module");
    xmlNewProp(nodeModule, BAD_CAST"name", BAD_CAST"FaceOcclusion");
    xmlAddChild(root, nodeModule);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"EYES_OCCLUSION_TIME_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.FaceOcclusionCfg.EYES_OCCLUSION_TIME_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"MOUTH_OCCLUSION_TIME_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.FaceOcclusionCfg.MOUTH_OCCLUSION_TIME_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    // 处理 AbnormalBehavior
    nodeModule = xmlNewNode(NULL, BAD_CAST"module");
    xmlNewProp(nodeModule, BAD_CAST"name", BAD_CAST"AbnormalBehavior");
    xmlAddChild(root, nodeModule);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"FACE_ANGLE_HORIZ_TIME_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.AbnormalBehaviorCfg.FACE_ANGLE_HORIZ_TIME_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"FACE_ANGLE_VIRT_TIME_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.AbnormalBehaviorCfg.FACE_ANGLE_VIRT_TIME_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"ANGLE_HORIZ_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.AbnormalBehaviorCfg.ANGLE_HORIZ_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"ANGLE_VERT_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.AbnormalBehaviorCfg.ANGLE_VERT_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    // 处理 FatigueState
    nodeModule = xmlNewNode(NULL, BAD_CAST"module");
    xmlNewProp(nodeModule, BAD_CAST"name", BAD_CAST"FatigueState");
    xmlAddChild(root, nodeModule);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"EYE_CLOSE_TIME_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.FatigueStateCfg.EYE_CLOSE_TIME_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"MOUTH_OPEN_TIME_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.FatigueStateCfg.MOUTH_OPEN_TIME_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"EYE_DGREE_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.FatigueStateCfg.EYE_DGREE_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    nodeParams = xmlNewNode(NULL, BAD_CAST"param");
    xmlNewProp(nodeParams, BAD_CAST"name", BAD_CAST"MOUTH_DGREE_THRD");
    xmlNewProp(nodeParams, BAD_CAST"value", BAD_CAST(std::to_string(Configs.FatigueStateCfg.MOUTH_DGREE_THRD).c_str()));
    xmlAddChild(nodeModule, nodeParams);

    //保存XML文档
    int nRel = xmlSaveFile(cfgFilename.c_str(), doc);
    if(nRel == -1)
    {
        cout << "配置文件" << cfgFilename << "保存失败！" << endl;
    }
    else
    {
        cout << "设置已保存到文件：" << cfgFilename << endl;
    }

    //释放资源
    xmlFreeDoc(doc);
    xmlCleanupParser();

    return true;
}


// 根据功能模块解析参数——需要继续添加功能模块
void ConfigXmlParams::parseModule(xmlNodePtr& pModule)
{
    xmlChar* attr_value = NULL;
    if(!xmlStrcmp(pModule->name, (const xmlChar*)"module"))
    {
        attr_value = xmlGetProp(pModule, (const xmlChar*)"name");

        if(!xmlStrcmp(attr_value, (const xmlChar*)"FaceRecognition"))
        {
            parseFaceRecognition(pModule);
        }

        if(!xmlStrcmp(attr_value, (const xmlChar*)"FaceOcclusion"))
        {
            parseFaceOcclusion(pModule);
        }

        if(!xmlStrcmp(attr_value, (const xmlChar*)"AbnormalBehavior"))
        {
            parseAbnormalBehavior(pModule);
        }

        if(!xmlStrcmp(attr_value, (const xmlChar*)"FatigueState"))
        {
            parseFatigueState(pModule);
        }

        xmlFree(attr_value);
    }
}

// 解析人脸识别
void ConfigXmlParams::parseFaceRecognition(xmlNodePtr& pFaceRecogModule)
{
    xmlNodePtr pParamNode = pFaceRecogModule->xmlChildrenNode;

    xmlChar* attr_name = NULL;
    xmlChar* attr_value = NULL;
    while (NULL != pParamNode)
    {
        if(!xmlStrcmp(pParamNode->name, (const xmlChar*)"param"))
        {
            attr_name = xmlGetProp(pParamNode, (const xmlChar*)"name");

            if(!xmlStrcmp(attr_name, (const xmlChar*)"RECOGNITION_TIME_PERIOD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.FaceRecognitionCfg.RECOGNITION_TIME_PERIOD = convertXmlCharToInt(attr_value);
            }


            xmlFree(attr_name);
            xmlFree(attr_value);
        }
        pParamNode = pParamNode->next;
    }
}

// 解析人脸遮挡
void ConfigXmlParams::parseFaceOcclusion(xmlNodePtr& pFaceOcclModule)
{
    xmlNodePtr pParamNode = pFaceOcclModule->xmlChildrenNode;

    xmlChar* attr_name = NULL;
    xmlChar* attr_value = NULL;
    while (NULL != pParamNode)
    {
        if(!xmlStrcmp(pParamNode->name, (const xmlChar*)"param"))
        {
            attr_name = xmlGetProp(pParamNode, (const xmlChar*)"name");

            if(!xmlStrcmp(attr_name, (const xmlChar*)"EYES_OCCLUSION_TIME_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.FaceOcclusionCfg.EYES_OCCLUSION_TIME_THRD = convertXmlCharToInt(attr_value);
            }
            else if(!xmlStrcmp(attr_name, (const xmlChar*)"MOUTH_OCCLUSION_TIME_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.FaceOcclusionCfg.MOUTH_OCCLUSION_TIME_THRD = convertXmlCharToInt(attr_value);
            }

            xmlFree(attr_name);
            xmlFree(attr_value);
        }
        pParamNode = pParamNode->next;
    }


}

// 解析异常行为
void ConfigXmlParams::parseAbnormalBehavior(xmlNodePtr& pAbnormBehavModule)
{
    xmlNodePtr pParamNode = pAbnormBehavModule->xmlChildrenNode;

    xmlChar* attr_name = NULL;
    xmlChar* attr_value = NULL;
    while (NULL != pParamNode)
    {
        if(!xmlStrcmp(pParamNode->name, (const xmlChar*)"param"))
        {
            attr_name = xmlGetProp(pParamNode, (const xmlChar*)"name");
            if(!xmlStrcmp(attr_name, (const xmlChar*)"FACE_ANGLE_HORIZ_TIME_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.AbnormalBehaviorCfg.FACE_ANGLE_HORIZ_TIME_THRD = convertXmlCharToInt(attr_value);
            }
            else if(!xmlStrcmp(attr_name, (const xmlChar*)"FACE_ANGLE_VIRT_TIME_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.AbnormalBehaviorCfg.FACE_ANGLE_VIRT_TIME_THRD = convertXmlCharToInt(attr_value);
            }
            else if(!xmlStrcmp(attr_name, (const xmlChar*)"ANGLE_HORIZ_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.AbnormalBehaviorCfg.ANGLE_HORIZ_THRD = convertXmlCharToDouble(attr_value);
            }
            else if(!xmlStrcmp(attr_name, (const xmlChar*)"ANGLE_VERT_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.AbnormalBehaviorCfg.ANGLE_VERT_THRD = convertXmlCharToDouble(attr_value);
            }

            xmlFree(attr_name);
            xmlFree(attr_value);
        }

        pParamNode = pParamNode->next;
    }
}

// 解析疲劳状态
void ConfigXmlParams::parseFatigueState(xmlNodePtr& pFatigueModule)
{
    xmlNodePtr pParamNode = pFatigueModule->xmlChildrenNode;

    xmlChar* attr_name = NULL;
    xmlChar* attr_value = NULL;
    while (NULL != pParamNode)
    {
        if(!xmlStrcmp(pParamNode->name, (const xmlChar*)"param"))
        {
            attr_name = xmlGetProp(pParamNode, (const xmlChar*)"name");
            if(!xmlStrcmp(attr_name, (const xmlChar*)"EYE_CLOSE_TIME_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.FatigueStateCfg.EYE_CLOSE_TIME_THRD = convertXmlCharToInt(attr_value);
            }
            else if(!xmlStrcmp(attr_name, (const xmlChar*)"MOUTH_OPEN_TIME_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.FatigueStateCfg.MOUTH_OPEN_TIME_THRD = convertXmlCharToInt(attr_value);
            }
            else if(!xmlStrcmp(attr_name, (const xmlChar*)"EYE_DGREE_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.FatigueStateCfg.EYE_DGREE_THRD = convertXmlCharToDouble(attr_value);
            }
            else if(!xmlStrcmp(attr_name, (const xmlChar*)"MOUTH_DGREE_THRD"))
            {
                attr_value = xmlGetProp(pParamNode, (const xmlChar*)"value");
                Configs.FatigueStateCfg.MOUTH_DGREE_THRD = convertXmlCharToDouble(attr_value);
            }

            xmlFree(attr_name);
            xmlFree(attr_value);
        }

        pParamNode = pParamNode->next;
    }
}

int ConfigXmlParams::convertXmlCharToInt(xmlChar* xmlChars)
{
    int digit;
    std::stringstream str2digit;
    str2digit << xmlChars;
    str2digit >> digit;

    return digit;
}

double ConfigXmlParams::convertXmlCharToDouble(xmlChar* xmlChars)
{
    double digit;
    std::stringstream str2digit;
    str2digit << xmlChars;
    str2digit >> digit;

    return digit;
}









