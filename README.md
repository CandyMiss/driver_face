# driver_face
//本地打开的文件夹时，.vscode不要嵌套
//rostopic echo 话题名称 可以查看当前话题发布的消息


//fatal: unable to access 'https://hub.fastgit.org/CandyMiss/driver_face.git/': The requested URL returned error: 503
//需要修改项目的配置文件，code服务器不要用代理服务器fastgit
//git config --show
//git config --global --get-all
//git config --local remote.origin.url https://github.com/CandyMiss/driver_face.git

//test//

--------------------------------
bug1：计算相似度为nan：
0号下标的相似度值为nan，所有图片未录入的位置，特征全0,相似度都为nan;C++调用cuda的逻辑部分，内存与cuda设备相互读写，cout出来结果都为0,但是实际特征数据已经传进入cuda，可以正确计算相似度。
现在已经确定用face_feature功能包生成的face.data特征可以嵌入原识别逻辑使用。