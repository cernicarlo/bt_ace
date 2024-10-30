#pragma once

namespace IauvGirona1000Survey {


enum SurveyType
{
    SCAN = 1,     // 1 represents scan
    CIRCULAR = 2  // 2 represents circular
};

// Custom type
struct Pose3D
{
    double x, y, z;
};

}