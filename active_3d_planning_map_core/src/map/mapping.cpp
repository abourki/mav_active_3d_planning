#include "active_3d_planning_voxblox/map/mapping.h"



#include "active_3d_planning_core/data/system_constraints.h"

namespace active_3d_planning {
namespace map {

ModuleFactoryRegistry::Registration<MapCore> MapCore::registration(
        "MapCore");

MapCore::MapCore(PlannerI& planner) : TSDFMap(planner) {}

MapCore::~MapCore()
{
    _mapServer->logExploration();
}

void MapCore::setupFromParamMap(ModuleBase::ParamMap *param_map)
{
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    _mapServer.reset(new MapCoreServer(nh,nh_private));

    //    esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));
    //    esdf_server_->setTraversabilityRadius(
    //        planner_.getSystemConstraints().collision_radius);

    // cache constants
    c_voxel_size_ = _mapServer->getResolution();
    //    c_block_size_ = esdf_server_->getEsdfMapPtr()->block_size();
    //    c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private)
    //                            .max_weight;  // direct access is not exposed

}
bool MapCore::isTraversable(const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& orientation) {
    float distance = 0.0;
    Eigen::Vector3f pos = position.cast<float>();
    if(!_mapServer->is_inside(pos))
        return false;

    if(_mapServer->getState(pos) != 0){//observed
        _mapServer->getDistance(pos,distance);
        return (distance > planner_.getSystemConstraints().collision_radius);
    }
    return false;
}

bool MapCore::isObserved(const Eigen::Vector3d& point) {
    Eigen::Vector3f pos = point.cast<float>();
    if(!_mapServer->is_inside(pos))
        return false;

    return _mapServer->getState(pos) != 0;
}

// get occupancy
unsigned char MapCore::getVoxelState(const Eigen::Vector3d& point) {
    Eigen::Vector3f pos = point.cast<float>();
    if(!_mapServer->is_inside(pos))
        return MapCore::UNKNOWN;

    uint8_t flag = _mapServer->getState(pos)  & ~(1u << 2);;

    if(flag & VoxelBuffer::occupied_flag)
        return MapCore::OCCUPIED;
    else if(flag & VoxelBuffer::visited_flag)
        return MapCore::FREE;
    else
        return MapCore::UNKNOWN;
}

// get voxel size
double MapCore::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
bool MapCore::getVoxelCenter(Eigen::Vector3d* center,
                                const Eigen::Vector3d& point) {

    Eigen::Vector3f pos = point.cast<float>();
    Eigen::Vector3i c;
    _mapServer->convert(pos,c);
    _mapServer->convert(c,pos);
    *center = pos.cast<double>();
    return true;
}

// get the stored TSDF distance not used
double MapCore::getVoxelDistance(const Eigen::Vector3d& point) {

    return 0.0;
}

// get the stored weight
double MapCore::getVoxelWeight(const Eigen::Vector3d& point) {
    Eigen::Vector3f pos = point.cast<float>();
    if(!_mapServer->is_inside(pos))
        return getMaximumWeight();

    Eigen::Vector3i coord;
    size_t idx;
    _mapServer->convert(pos,coord);
    idx = _mapServer->get_idx(coord);

    return _mapServer->getWeight(idx);
}

//not used
double MapCore::getMaximumWeight()
{
    return 100;
}


}
}
