#include "robo_model.h"
#include "sight_model.h"

using domain::RoboModel;
using domain::SightModel;

RoboModel::RoboModel()
{
    m_sight = new SightModel();
}

RoboModel::~RoboModel()
{
    delete m_sight;
//    delete m_status;
//    delete m_tracking;
}

SightModel *RoboModel::sight() const
{
    return m_sight;
}

//StatusModel *RoboModel::status() const
//{
//    return nullptr;
//}

//TrackingModel *RoboModel::tracking() const
//{
//    return nullptr;
//}
