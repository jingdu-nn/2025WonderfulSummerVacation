#include "PlayersColorCluster.h"

PlayersColorCluster::PlayersColorCluster()
{
    // 构造函数实现，可初始化参数
}

void PlayersColorCluster::update(ObstaclesFieldPercept& theObstaclesFieldPercept)
{
    // 1.1 选色空间
    int Cb1 = colorConfig.yellowBoardCbMin;
    int Cb2 = colorConfig.yellowBoardCbMax;
    int Cr1 = colorConfig.yellowBoardCrMin;
    int Cr2 = colorConfig.yellowBoardCrMax;

    // 1.2 创建 mask
    std::vector<std::vector<unsigned char>> mask(theECImage.height, std::vector<unsigned char>(theECImage.width, 0));

    for(int y = 0; y < theECImage.height; ++y)
    {
        for(int x = 0; x < theECImage.width; ++x)
        {
            unsigned char cb = theECImage.cb[y][x];
            unsigned char cr = theECImage.cr[y][x];
            if(cb >= Cb1 && cb <= Cb2 && cr >= Cr1 && cr <= Cr2)
                mask[y][x] = 1;
            else
                mask[y][x] = 0;
        }
    }
    // TODO: 后续处理
}

void PlayersColorCluster::update(ObstaclesImagePercept& theObstaclesImagePercept)
{
    // TODO: 实现颜色识别聚合主流程
}

void PlayersColorCluster::update(ObstaclesPerceptorData& theObstaclesPerceptorData)
{
    // TODO: 实现颜色识别聚合主流程
} 