// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <sstream>

#include "RGLServerPluginInstance.hh"
#include "Utils.hh"

#define PARAM_UPDATE_RATE_ID "update_rate"
#define PARAM_RANGE_ID "range"
#define PARAM_RANGE_MIN_ID "min"
#define PARAM_RANGE_MAX_ID "max"
#define PARAM_TOPIC_ID "topic"
#define PARAM_FRAME_ID "frame"
#define PARAM_UPDATE_ON_PAUSED_SIM_ID "update_on_paused_sim"
#define PARAM_NOISE_ID "noise"
#define PARAM_NOISE_ANGULAR_RAY_ID "angular_ray"
#define PARAM_NOISE_ANGULAR_HITPOINT_ID "angular_hitpoint"
#define PARAM_NOISE_DISTANCE_ID "distance"
#define PARAM_NOISE_MEAN_ID "mean"
#define PARAM_NOISE_STDDEV_ID "stddev"
#define PARAM_NOISE_STDDEV_RISE_PER_METER_ID "stddev_rise_per_meter"
#define PARAM_NOISE_AXIS_ID "axis"

namespace rgl
{

bool RGLServerPluginInstance::LoadConfiguration(const std::shared_ptr<const sdf::Element>& sdf)
{
    // Required parameters
    if (!sdf->HasElement(PARAM_UPDATE_RATE_ID)) {
        gzerr << "No '" << PARAM_UPDATE_RATE_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_RANGE_ID) ||
        !sdf->FindElement(PARAM_RANGE_ID)->HasElement(PARAM_RANGE_MIN_ID) ||
        !sdf->FindElement(PARAM_RANGE_ID)->HasElement(PARAM_RANGE_MAX_ID)) {
        gzerr << "Range parameter is not defined correctly. Disabling plugin. Should be: \n"
               << "<" PARAM_RANGE_ID << ">\n"
               << "  <" << PARAM_RANGE_MIN_ID << ">" << "</" << PARAM_RANGE_MIN_ID << ">\n"
               << "  <" << PARAM_RANGE_MAX_ID << ">" << "</" << PARAM_RANGE_MAX_ID << ">\n"
               << "</" PARAM_RANGE_ID << ">\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_TOPIC_ID)) {
        gzerr << "No '" << PARAM_TOPIC_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    if (!sdf->HasElement(PARAM_FRAME_ID)) {
        gzerr << "No '" << PARAM_FRAME_ID << "' parameter specified for the RGL lidar. Disabling plugin.\n";
        return false;
    }

    // Optional parameters
    if (!sdf->HasElement(PARAM_UPDATE_ON_PAUSED_SIM_ID)) {
        gzwarn << "No '" << PARAM_UPDATE_ON_PAUSED_SIM_ID << "' parameter specified for the RGL lidar. "
                << "Using default value: " << updateOnPausedSim << "\n";
    } else {
        updateOnPausedSim = sdf->Get<bool>(PARAM_UPDATE_ON_PAUSED_SIM_ID);
    }

    // Load configuration
    float updateRateHz = sdf->Get<float>(PARAM_UPDATE_RATE_ID);
    raytraceIntervalTime = std::chrono::microseconds(static_cast<int64_t>(1e6 / updateRateHz));
    lidarMinMaxRange.value[0] = sdf->FindElement(PARAM_RANGE_ID)->Get<float>(PARAM_RANGE_MIN_ID);
    lidarMinMaxRange.value[1] = sdf->FindElement(PARAM_RANGE_ID)->Get<float>(PARAM_RANGE_MAX_ID);
    topicName = sdf->Get<std::string>(PARAM_TOPIC_ID);
    frameId = sdf->Get<std::string>(PARAM_FRAME_ID);

    if (!LoadNoiseConfiguration(sdf)) {
        return false;
    }

    if (!LidarPatternLoader::Load(sdf, lidarPattern, lidarPatternSampleSize)) {
        return false;
    }

    if ((lidarPattern.size() % lidarPatternSampleSize) != 0) {
        gzerr << "Total pattern size (" << lidarPattern.size() << ") must be a multiple of the sample size (" << lidarPatternSampleSize << "). Disabling plugin.\n";
        return false;
    }

    // Check for 2d pattern and get LaserScan parameters
    if (sdf->HasElement("pattern_lidar2d")) {
        gzmsg << "Lidar is 2D, switching to publish LaserScan messages";
        publishLaserScan = true;
        scanHMin = sdf->FindElement("pattern_lidar2d")->FindElement("horizontal")->Get<float>("min_angle");
        scanHMax = sdf->FindElement("pattern_lidar2d")->FindElement("horizontal")->Get<float>("max_angle");
        scanHSamples = lidarPatternSampleSize;
    }

    return true;
}

bool RGLServerPluginInstance::LoadNoiseConfiguration(const std::shared_ptr<const sdf::Element>& sdf)
{
    if (!sdf->HasElement(PARAM_NOISE_ID)) {
        return true;
    }
    const auto noiseSdf = sdf->FindElement(PARAM_NOISE_ID);

    for (auto child = noiseSdf->GetFirstElement(); child != nullptr; child = child->GetNextElement()) {
        const std::string name = child->GetName();
        if (name != PARAM_NOISE_ANGULAR_RAY_ID && name != PARAM_NOISE_ANGULAR_HITPOINT_ID && name != PARAM_NOISE_DISTANCE_ID) {
            gzwarn << "Unknown '" << PARAM_NOISE_ID << "' child element '" << name << "' for the RGL lidar. Ignoring it.\n";
        }
    }

    auto loadNoise = [&](const char* noiseTypeId, bool isAngular, GaussianNoise& outNoise) -> bool {
        if (!noiseSdf->HasElement(noiseTypeId)) {
            return true;
        }
        const auto noiseTypeSdf = noiseSdf->FindElement(noiseTypeId);
        if (!noiseTypeSdf->HasElement(PARAM_NOISE_STDDEV_ID)) {
            gzerr << "No '" << PARAM_NOISE_STDDEV_ID << "' parameter specified for the RGL lidar '"
                  << noiseTypeId << "' noise. Disabling plugin.\n";
            return false;
        }
        const char* ignoredParam = isAngular ? PARAM_NOISE_STDDEV_RISE_PER_METER_ID : PARAM_NOISE_AXIS_ID;
        if (noiseTypeSdf->HasElement(ignoredParam)) {
            gzwarn << "Parameter '" << ignoredParam << "' does not apply to the RGL lidar '" << noiseTypeId
                   << "' noise. Ignoring it.\n";
        }

        outNoise.stDev = noiseTypeSdf->Get<float>(PARAM_NOISE_STDDEV_ID);
        outNoise.mean = noiseTypeSdf->Get<float>(PARAM_NOISE_MEAN_ID, outNoise.mean).first;
        outNoise.stDevRisePerMeter = noiseTypeSdf->Get<float>(PARAM_NOISE_STDDEV_RISE_PER_METER_ID, outNoise.stDevRisePerMeter).first;
        if (outNoise.stDev < 0.0f || outNoise.stDevRisePerMeter < 0.0f) {
            gzerr << "Standard deviation of the RGL lidar '" << noiseTypeId << "' noise must not be negative. Disabling plugin.\n";
            return false;
        }
        outNoise.enabled = outNoise.mean != 0.0f || outNoise.stDev != 0.0f || outNoise.stDevRisePerMeter != 0.0f;
        if (!outNoise.enabled) {
            gzmsg << "RGL lidar '" << noiseTypeId << "' noise is configured with zero mean and stddev, skipping it.\n";
            return true;
        }

        static const std::map<std::string, rgl_axis_t> axes = {{"x", RGL_AXIS_X}, {"y", RGL_AXIS_Y}, {"z", RGL_AXIS_Z}};
        const std::string axis = noiseTypeSdf->Get<std::string>(PARAM_NOISE_AXIS_ID, "z").first;
        if (axes.count(axis) == 0) {
            gzerr << "Invalid '" << PARAM_NOISE_AXIS_ID << "' parameter '" << axis << "' for the RGL lidar '"
                  << noiseTypeId << "' noise. Expected 'x', 'y' or 'z'. Disabling plugin.\n";
            return false;
        }
        outNoise.axis = axes.at(axis);

        std::ostringstream details;
        if (isAngular) {
            details << ", axis: " << axis;
        } else {
            details << ", stddev rise per meter: " << outNoise.stDevRisePerMeter;
        }
        gzmsg << "RGL lidar '" << noiseTypeId << "' noise enabled (mean: " << outNoise.mean << ", stddev: " << outNoise.stDev
              << details.str() << ")\n";
        return true;
    };

    return loadNoise(PARAM_NOISE_ANGULAR_RAY_ID, true, noiseAngularRay) &&
           loadNoise(PARAM_NOISE_ANGULAR_HITPOINT_ID, true, noiseAngularHitpoint) &&
           loadNoise(PARAM_NOISE_DISTANCE_ID, false, noiseDistance);
}

void RGLServerPluginInstance::CreateLidar(gz::sim::Entity entity,
                                          gz::sim::EntityComponentManager& ecm)
{
    thisLidarEntity = entity;

    rgl_mat3x4f identity = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0
    };

    // Resize result data containers with the maximum possible point count (number of lasers).
    // This improves performance in runtime because no additional allocations are needed.
    resultPointCloud.data.resize(resultPointCloud.pointSize * lidarPatternSampleSize);
    if (publishLaserScan)
    {
        resultLaserScan.distances.resize(lidarPatternSampleSize);
        resultLaserScan.intensities.resize(lidarPatternSampleSize);
    }

    for (std::size_t i = 0; i < lidarPattern.size(); i += lidarPatternSampleSize)
    {
      rglNodesUseRays.emplace_back();
      if (!CheckRGL(rgl_node_rays_from_mat3x4f(&rglNodesUseRays.back(),
                                               lidarPattern.data() + i,
                                               lidarPatternSampleSize))) {
          gzerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
          return;
      }
    }

    if (!CheckRGL(rgl_node_rays_set_range(&rglNodeSetRange, &lidarMinMaxRange, 1)) ||
        !CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &identity)) ||
        !CheckRGL(rgl_node_raytrace(&rglNodeRaytrace, nullptr)) ||
        !CheckRGL(rgl_node_points_compact_by_field(&rglNodeCompact, RGL_FIELD_IS_HIT_I32)) ||
        !CheckRGL(rgl_node_points_yield(&rglNodeYieldLaserScan, resultLaserScan.rglFields.data(), resultLaserScan.rglFields.size())) ||
        !CheckRGL(rgl_node_points_format(&rglNodeFormatPointCloudSensor, resultPointCloud.rglFields.data(), resultPointCloud.rglFields.size())) ||
        !CheckRGL(rgl_node_points_format(&rglNodeFormatPointCloudWorld, resultPointCloud.rglFields.data(), resultPointCloud.rglFields.size())) ||
        !CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &identity))) {

        gzerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    // Optional Gaussian noise, see https://github.com/RobotecAI/RobotecGPULidar/blob/main/docs/GaussianNoise.md
    rgl_node_t raysTail = rglNodeLidarPose;   // last node before raytracing
    rgl_node_t pointsTail = rglNodeRaytrace;  // last node producing hit points
    if (noiseAngularRay.enabled) {
        rgl_node_t node = nullptr;
        if (!CheckRGL(rgl_node_gaussian_noise_angular_ray(&node, noiseAngularRay.mean, noiseAngularRay.stDev, noiseAngularRay.axis)) ||
            !CheckRGL(rgl_graph_node_add_child(raysTail, node))) {
            gzerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
            return;
        }
        raysTail = node;
    }
    if (noiseDistance.enabled) {
        rgl_node_t node = nullptr;
        if (!CheckRGL(rgl_node_gaussian_noise_distance(&node, noiseDistance.mean, noiseDistance.stDev, noiseDistance.stDevRisePerMeter)) ||
            !CheckRGL(rgl_graph_node_add_child(pointsTail, node))) {
            gzerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
            return;
        }
        pointsTail = node;
    }
    if (noiseAngularHitpoint.enabled) {
        rgl_node_t node = nullptr;
        if (!CheckRGL(rgl_node_gaussian_noise_angular_hitpoint(&node, noiseAngularHitpoint.mean, noiseAngularHitpoint.stDev, noiseAngularHitpoint.axis)) ||
            !CheckRGL(rgl_graph_node_add_child(pointsTail, node))) {
            gzerr << "Failed to create RGL nodes when initializing lidar. Disabling plugin.\n";
            return;
        }
        pointsTail = node;
    }

    if (!CheckRGL(rgl_graph_node_add_child(rglNodesUseRays.front(), rglNodeSetRange)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeSetRange, rglNodeLidarPose)) ||
        !CheckRGL(rgl_graph_node_add_child(raysTail, rglNodeRaytrace)) ||
        !CheckRGL(rgl_graph_node_add_child(pointsTail, rglNodeCompact)) ||
        !CheckRGL(rgl_graph_node_add_child(rglNodeCompact, rglNodeFormatPointCloudWorld))) {

        gzerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        return;
    }

    if (publishLaserScan) {
        if(!CheckRGL(rgl_graph_node_add_child(pointsTail, rglNodeYieldLaserScan)) ||
           // Optimization: rglNodeYieldLaserScan should be prioritized because it will be requested first
           !CheckRGL(rgl_graph_node_set_priority(rglNodeYieldLaserScan, 1))) {
            gzerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        }
        gzmsg << "Start publishing LaserScan messages on topic '" << topicName << "'\n";
        laserScanPublisher = gazeboNode.Advertise<gz::msgs::LaserScan>(topicName);
    } else {  // publish PointCloud
        if(!CheckRGL(rgl_graph_node_add_child(rglNodeCompact, rglNodeToLidarFrame)) ||
           !CheckRGL(rgl_graph_node_add_child(rglNodeToLidarFrame, rglNodeFormatPointCloudSensor)) ||
           // Optimization: rglNodeFormatPointCloudSensor should be prioritized because it will be requested first
           !CheckRGL(rgl_graph_node_set_priority(rglNodeFormatPointCloudSensor, 1))) {
            gzerr << "Failed to connect RGL nodes when initializing lidar. Disabling plugin.\n";
        }
        gzmsg << "Start publishing PointCloudPacked messages on topic '" << topicName << "'\n";
        pointCloudPublisher = gazeboNode.Advertise<gz::msgs::PointCloudPacked>(topicName);
    }
    pointCloudWorldPublisher = gazeboNode.Advertise<gz::msgs::PointCloudPacked>(topicName + worldTopicPostfix);

    isLidarInitialized = true;
}

void RGLServerPluginInstance::UpdateLidarPose(const gz::sim::EntityComponentManager& ecm)
{
    gz::math::Pose3<double> ignLidarToWorld = FindWorldPose(thisLidarEntity, ecm);
    gz::math::Pose3<double> ignWorldToLidar = ignLidarToWorld.Inverse();
    rgl_mat3x4f rglLidarToWorld = IgnPose3dToRglMatrix(ignLidarToWorld);
    rgl_mat3x4f rglWorldToLidar = IgnPose3dToRglMatrix(ignWorldToLidar);
    CheckRGL(rgl_node_rays_transform(&rglNodeLidarPose, &rglLidarToWorld));
    CheckRGL(rgl_node_points_transform(&rglNodeToLidarFrame, &rglWorldToLidar));
}

void RGLServerPluginInstance::UpdateAlternatingLidarPattern()
{
    // remove old child
    if(!CheckRGL(rgl_graph_node_remove_child(rglNodesUseRays[alternatingPatternIndex], rglNodeSetRange)))
    {
        gzerr << "Failed to update alternating lidar pattern, not able to remove child.\n";
        return;
    }

    alternatingPatternIndex = (alternatingPatternIndex + 1) % rglNodesUseRays.size();

    // add new child
    if(!CheckRGL(rgl_graph_node_add_child(rglNodesUseRays[alternatingPatternIndex], rglNodeSetRange)))
    {
        gzerr << "Failed to update alternating lidar pattern, not able to add new child.\n";
        return;
    }
}

bool RGLServerPluginInstance::ShouldRayTrace(std::chrono::steady_clock::duration simTime,
                                             bool paused)
{
    if (!isLidarInitialized) {
        return false;
    }

    if (paused) {
        ++onPausedSimUpdateCounter;
        if (!updateOnPausedSim) {
            return false;
        }
        if (onPausedSimUpdateCounter < onPausedSimRaytraceInterval) {
            return false;
        }
        onPausedSimUpdateCounter = 0;
        return true;
    }

    // Simulation running
    if (simTime < lastRaytraceTime + raytraceIntervalTime) {
        return false;
    }
    return true;
}

void RGLServerPluginInstance::RayTrace(std::chrono::steady_clock::duration simTime)
{
    if (rglNodesUseRays.size() > 1) {
        UpdateAlternatingLidarPattern();
    }

    lastRaytraceTime = simTime;

    if (!CheckRGL(rgl_graph_run(rglNodeRaytrace))) {
        gzerr << "Failed to perform raytrace.\n";
        return;
    }

    if (publishLaserScan) {
        if (!FetchLaserScanResult()) {
            gzerr << "Failed to fetch LaserScan result data from RGL lidar.\n";
            return;
        }
        auto msg = CreateLaserScanMsg(simTime, frameId);
        laserScanPublisher.Publish(msg);
    } else {  // publish PointCloud
        if (!FetchPointCloudResult(rglNodeFormatPointCloudSensor)) {
            gzerr << "Failed to fetch PointCloud result data (sensor frame) from RGL lidar.\n";
            return;
        }
        auto msg = CreatePointCloudMsg(simTime, frameId);
        pointCloudPublisher.Publish(msg);
    }

    if (pointCloudWorldPublisher.HasConnections()) {
        if (!FetchPointCloudResult(rglNodeFormatPointCloudWorld)) {
            gzerr << "Failed to fetch PointCloud result data (world frame) from RGL lidar.\n";
            return;
        }
        auto msg = CreatePointCloudMsg(simTime, worldFrameId);
        pointCloudWorldPublisher.Publish(msg);
    }
}

bool RGLServerPluginInstance::FetchLaserScanResult()
{
    if (!CheckRGL(rgl_graph_get_result_data(rglNodeYieldLaserScan, RGL_FIELD_DISTANCE_F32, resultLaserScan.distances.data())) ||
        !CheckRGL(rgl_graph_get_result_data(rglNodeYieldLaserScan, RGL_FIELD_LASER_RETRO_F32, resultLaserScan.intensities.data()))) {
        return false;
    }
    return true;
}

bool RGLServerPluginInstance::FetchPointCloudResult(rgl_node_t formatNode)
{
    int32_t rglPointSize = -1;
    if (!CheckRGL(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &resultPointCloud.hitPointCount, &rglPointSize))) {
        return false;
    }
    assert(rglPointSize == resultPointCloud.pointSize);
    if (!CheckRGL(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT, resultPointCloud.data.data()))) {
        return false;
    }
    return true;
}

gz::msgs::LaserScan RGLServerPluginInstance::CreateLaserScanMsg(std::chrono::steady_clock::duration simTime, const std::string& frame)
{
    auto pointCount = resultLaserScan.distances.size();
    gz::msgs::LaserScan outMsg;
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(simTime);
    auto _frame = outMsg.mutable_header()->add_data();
    _frame->set_key("frame_id");
    _frame->add_value(frame);

    outMsg.set_frame(frame);
    outMsg.set_count(pointCount);

    outMsg.set_range_min(lidarMinMaxRange.value[0]);
    outMsg.set_range_max(lidarMinMaxRange.value[1]);

    gz::math::Angle hStep((scanHMax-scanHMin)/scanHSamples);

    outMsg.set_angle_min(scanHMin.Radian());
    outMsg.set_angle_max(scanHMax.Radian());
    outMsg.set_angle_step(hStep.Radian());

    for (int i = 0; i < pointCount; ++i) {
        // Distance noise turns the infinite range of non-hits into NaN. Report non-hits as infinity again.
        outMsg.add_ranges(std::isnan(resultLaserScan.distances[i]) ? std::numeric_limits<float>::infinity() : resultLaserScan.distances[i]);
        outMsg.add_intensities(resultLaserScan.intensities[i]);
    }

    return outMsg;
}

gz::msgs::PointCloudPacked RGLServerPluginInstance::CreatePointCloudMsg(std::chrono::steady_clock::duration simTime, const std::string& frame)
{
    gz::msgs::PointCloudPacked outMsg;
    gz::msgs::InitPointCloudPacked(outMsg, frame, false,
                                         {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
                                          {"intensity",gz::msgs::PointCloudPacked::Field::FLOAT32}});
    outMsg.mutable_data()->resize(resultPointCloud.hitPointCount * outMsg.point_step());
    *outMsg.mutable_header()->mutable_stamp() = gz::msgs::Convert(simTime);
    outMsg.set_height(1);
    outMsg.set_width(resultPointCloud.hitPointCount);
    outMsg.set_row_step(resultPointCloud.hitPointCount * outMsg.point_step());

    gz::msgs::PointCloudPackedIterator<float> xIter(outMsg, "x");
    memcpy(&(*xIter), resultPointCloud.data.data(), resultPointCloud.hitPointCount * resultPointCloud.pointSize);
    return outMsg;
}

void RGLServerPluginInstance::DestroyLidar()
{
    if (!isLidarInitialized) {
        return;
    }

    if (!CheckRGL(rgl_graph_destroy(rglNodeRaytrace))) {
        gzerr << "Failed to destroy RGL lidar.\n";
    }
    // Reset publishers
    if (!publishLaserScan) {
        pointCloudPublisher = gz::transport::Node::Publisher();
    } else {
        laserScanPublisher = gz::transport::Node::Publisher();
    }
    pointCloudWorldPublisher = gz::transport::Node::Publisher();
    isLidarInitialized = false;
}

}  // namespace rgl
