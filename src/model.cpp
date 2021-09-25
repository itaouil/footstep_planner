/*
 * model.cpp
 *
 *  Created on: Aug 24, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include <model.hpp>

/**
 * Constructor
 */
Model::Model(ros::NodeHandle& p_nh):
    m_nh(p_nh),
    m_listener(m_buffer)
{
    // Footstep prediction service client
    m_footstepPredictionServiceClient = m_nh.serviceClient<aliengo_navigation::FootstepPrediction>(FOOTSTEP_PREDICTION_SERVICE_TOPIC);

    // Feet configuration marker array publisher
    m_feetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(FEET_CONFIGURATION_MARKERS_TOPIC, 1);
}

/**
 * Destructor
 */
Model::~Model() = default;

/**
 * Transform absolute footstep sizes
 * from CoM reference frame to map frame.
 *
 * @param p_rotation
 * @param p_predictionsCoMFrame
 * @param p_predictionsMapFrame
 */
void Model::transformPredictionsToMapFrame(const tf2::Quaternion &p_rotation,
                                           const std::vector<float> &p_predictionsCoMFrame,
                                           std::vector<double> &p_predictionsMapFrame)
{
    for (auto &prediction: p_predictionsCoMFrame)
    {
        ROS_INFO_STREAM("Pre-transformed predictions: " << prediction);
    }

    // Map to CoM rotation matrix
    geometry_msgs::TransformStamped l_rotationTransform;
    l_rotationTransform.header.stamp = ros::Time::now();
    l_rotationTransform.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_rotationTransform.transform.translation.x = 0;
    l_rotationTransform.transform.translation.y = 0;
    l_rotationTransform.transform.translation.z = 0;
    l_rotationTransform.transform.rotation.x = p_rotation.x();
    l_rotationTransform.transform.rotation.y = p_rotation.y();
    l_rotationTransform.transform.rotation.z = p_rotation.z();
    l_rotationTransform.transform.rotation.w = p_rotation.w();

    // Common message
    geometry_msgs::PointStamped l_common;
    l_common.header.stamp = ros::Time::now();
    l_common.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_common.point.z = 0;

    // FL and RR are support feet so
    // only transform FR/RL/CoM predictions
    if (p_predictionsCoMFrame[0] == 0)
    {
        // Prediction vector for FR displacement in CoM frame
        geometry_msgs::PointStamped l_frPredictionCoM;
        l_frPredictionCoM = l_common;
        l_frPredictionCoM.point.x = p_predictionsCoMFrame[2];
        l_frPredictionCoM.point.y = p_predictionsCoMFrame[3];

        // Prediction vector for RL displacement in CoM frame
        geometry_msgs::PointStamped l_rlPredictionCoM;
        l_rlPredictionCoM = l_common;
        l_rlPredictionCoM.point.x = p_predictionsCoMFrame[4];
        l_rlPredictionCoM.point.y = p_predictionsCoMFrame[5];

        // Prediction vector for CoM displacement in CoM frame
        geometry_msgs::PointStamped l_comPredictionCoM;
        l_comPredictionCoM = l_common;
        l_comPredictionCoM.point.x = p_predictionsCoMFrame[8];
        l_comPredictionCoM.point.y = p_predictionsCoMFrame[9];

        // Transformation
        geometry_msgs::PointStamped l_frPredictionMap;
        geometry_msgs::PointStamped l_rlPredictionMap;
        geometry_msgs::PointStamped l_comPredictionMap;
        tf2::doTransform(l_frPredictionCoM, l_frPredictionMap, l_rotationTransform);
        tf2::doTransform(l_rlPredictionCoM, l_rlPredictionMap, l_rotationTransform);
        tf2::doTransform(l_comPredictionCoM, l_comPredictionMap, l_rotationTransform);

        // Overwrite non-support feet predictions (the rest are zero)
        p_predictionsMapFrame[2] = l_frPredictionMap.point.x;
        p_predictionsMapFrame[3] = l_frPredictionMap.point.y;
        p_predictionsMapFrame[4] = l_rlPredictionMap.point.x;
        p_predictionsMapFrame[5] = l_rlPredictionMap.point.y;
        p_predictionsMapFrame[8] = l_comPredictionMap.point.x;
        p_predictionsMapFrame[9] = l_comPredictionMap.point.y;

        ROS_INFO_STREAM("Model: FR swinging " << l_frPredictionMap.point.x << ", " << l_frPredictionMap.point.y);
    }
    // FR and RL are support feet so
    // only transform FL/RR/CoM predictions
    else
    {
        // Prediction vector for FR displacement in CoM frame
        geometry_msgs::PointStamped l_flPredictionCoM;
        l_flPredictionCoM = l_common;
        l_flPredictionCoM.point.x = p_predictionsCoMFrame[0];
        l_flPredictionCoM.point.y = p_predictionsCoMFrame[1];

        // Prediction vector for RL displacement in CoM frame
        geometry_msgs::PointStamped l_rrPredictionCoM;
        l_rrPredictionCoM = l_common;
        l_rrPredictionCoM.point.x = p_predictionsCoMFrame[6];
        l_rrPredictionCoM.point.y = p_predictionsCoMFrame[7];

        // Prediction vector for CoM displacement in CoM frame
        geometry_msgs::PointStamped l_comPredictionCoM;
        l_comPredictionCoM = l_common;
        l_comPredictionCoM.point.x = p_predictionsCoMFrame[8];
        l_comPredictionCoM.point.y = p_predictionsCoMFrame[9];

        // Transformation
        geometry_msgs::PointStamped l_flPredictionMap;
        geometry_msgs::PointStamped l_rrPredictionMap;
        geometry_msgs::PointStamped l_comPredictionMap;
        tf2::doTransform(l_flPredictionCoM, l_flPredictionMap, l_rotationTransform);
        tf2::doTransform(l_rrPredictionCoM, l_rrPredictionMap, l_rotationTransform);
        tf2::doTransform(l_comPredictionCoM, l_comPredictionMap, l_rotationTransform);

        // Overwrite non-support feet predictions (the rest are zero)
        p_predictionsMapFrame[0] = l_flPredictionMap.point.x;
        p_predictionsMapFrame[1] = l_flPredictionMap.point.y;
        p_predictionsMapFrame[6] = l_rrPredictionMap.point.x;
        p_predictionsMapFrame[7] = l_rrPredictionMap.point.y;
        p_predictionsMapFrame[8] = l_comPredictionMap.point.x;
        p_predictionsMapFrame[9] = l_comPredictionMap.point.y;
    }

    for (auto &prediction: p_predictionsMapFrame)
    {
        ROS_INFO_STREAM("Transformed predictions: " << prediction);
    }

    ROS_INFO_STREAM("\n");
}

/**
 * Compute new (map) feet configuration
 * using the predicted footsteps.
 *
 * @param l_predictionsMapFrame
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeMapFeetConfiguration(const std::vector<double> &l_predictionsMapFrame,
                                        const FeetConfiguration &p_currentFeetConfiguration,
                                        FeetConfiguration &p_newFeetConfiguration)
{
    ROS_INFO_STREAM("Model: fl (curr) map " << p_currentFeetConfiguration.flMap.x << ", " << p_currentFeetConfiguration.flMap.y);
    ROS_INFO_STREAM("Model: fr (curr) map " << p_currentFeetConfiguration.frMap.x << ", " << p_currentFeetConfiguration.frMap.y);
    ROS_INFO_STREAM("Model: rl (curr)map " << p_currentFeetConfiguration.rlMap.x << ", " << p_currentFeetConfiguration.rlMap.y);
    ROS_INFO_STREAM("Model: rr (curr)map " << p_currentFeetConfiguration.rrMap.x << ", " << p_currentFeetConfiguration.rrMap.y);

    // New world coordinates after adding footsteps
    p_newFeetConfiguration.flMap.x = p_currentFeetConfiguration.flMap.x + l_predictionsMapFrame[0];
    p_newFeetConfiguration.flMap.y = p_currentFeetConfiguration.flMap.y + l_predictionsMapFrame[1];
    p_newFeetConfiguration.frMap.x = p_currentFeetConfiguration.frMap.x + l_predictionsMapFrame[2];
    p_newFeetConfiguration.frMap.y = p_currentFeetConfiguration.frMap.y + l_predictionsMapFrame[3];
    p_newFeetConfiguration.rlMap.x = p_currentFeetConfiguration.rlMap.x + l_predictionsMapFrame[4];
    p_newFeetConfiguration.rlMap.y = p_currentFeetConfiguration.rlMap.y + l_predictionsMapFrame[5];
    p_newFeetConfiguration.rrMap.x = p_currentFeetConfiguration.rrMap.x + l_predictionsMapFrame[6];
    p_newFeetConfiguration.rrMap.y = p_currentFeetConfiguration.rrMap.y + l_predictionsMapFrame[7];

    ROS_INFO_STREAM("Model: fl (new) map " << p_newFeetConfiguration.flMap.x << ", " << p_newFeetConfiguration.flMap.y);
    ROS_INFO_STREAM("Model: fr (new) map " << p_newFeetConfiguration.frMap.x << ", " << p_newFeetConfiguration.frMap.y);
    ROS_INFO_STREAM("Model: rl (new) map " << p_newFeetConfiguration.rlMap.x << ", " << p_newFeetConfiguration.rlMap.y);
    ROS_INFO_STREAM("Model: rr (new) map " << p_newFeetConfiguration.rrMap.x << ", " << p_newFeetConfiguration.rrMap.y << "\n");
}

/**
  * Compute new CoM in world coordinates.
  *
  * @param p_angularVelocity
  * @param p_predictedCoMDisplacementX
  * @param p_predictedCoMDisplacementY
  * @param p_currentWorldCoordinatesCoM
  * @param p_newWorldCoordinatesCoM
  */
void Model::computeNewCoM(double p_angularVelocity,
                          const double p_predictedCoMDisplacementX,
                          const double p_predictedCoMDisplacementY,
                          const World2D &p_currentWorldCoordinatesCoM,
                          World2D &p_newWorldCoordinatesCoM)
{
    ROS_INFO_STREAM("Model: COM movement in x: " << p_predictedCoMDisplacementX);
    ROS_INFO_STREAM("Model: COM movement in y: " << p_predictedCoMDisplacementY);

    // Map to CoM rotation matrix
    geometry_msgs::TransformStamped l_rotationTransform;
    l_rotationTransform.header.stamp = ros::Time::now();
    l_rotationTransform.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_rotationTransform.transform.translation.x = 0;
    l_rotationTransform.transform.translation.y = 0;
    l_rotationTransform.transform.translation.z = 0;
    l_rotationTransform.transform.rotation.x = p_currentWorldCoordinatesCoM.q.x();
    l_rotationTransform.transform.rotation.y = p_currentWorldCoordinatesCoM.q.y();
    l_rotationTransform.transform.rotation.z = p_currentWorldCoordinatesCoM.q.z();
    l_rotationTransform.transform.rotation.w = p_currentWorldCoordinatesCoM.q.w();

    // Populate robot frame displacement vector
    geometry_msgs::PointStamped l_displacementRobotFrame;
    l_displacementRobotFrame.header.stamp = ros::Time::now();
    l_displacementRobotFrame.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_displacementRobotFrame.point.x = p_predictedCoMDisplacementX;
    l_displacementRobotFrame.point.y = p_predictedCoMDisplacementY;
    l_displacementRobotFrame.point.z = 0;

    // Apply CoM rotation w.r.t to the
    // map frame to the displacement vector
    // (in order to deal with orientations)
    geometry_msgs::PointStamped l_displacementMapFrame;
    tf2::doTransform(l_displacementRobotFrame, l_displacementMapFrame, l_rotationTransform);

    // Update CoM coordinates
    p_newWorldCoordinatesCoM.x = p_currentWorldCoordinatesCoM.x + l_displacementMapFrame.point.x;
    p_newWorldCoordinatesCoM.y = p_currentWorldCoordinatesCoM.y + l_displacementMapFrame.point.y;

    // Compute new CoM rotation
    // if angular velocity was applied
    if (p_angularVelocity != 0)
    {
        // Get yaw rotation in quaternion form
        tf2::Quaternion l_velocityCommandQuaternion;
        l_velocityCommandQuaternion.setRPY(0, 0, p_angularVelocity * TIMESTAMP);

        // Apply rotation command rotation to CoM quaternion
        tf2::Quaternion l_newCoMRotation = p_currentWorldCoordinatesCoM.q * l_velocityCommandQuaternion;
        l_newCoMRotation.normalize();

        // Set new CoM rotation
        p_newWorldCoordinatesCoM.q = l_newCoMRotation;
    }
    else
    {
        p_newWorldCoordinatesCoM.q = p_currentWorldCoordinatesCoM.q;
    }

    ROS_INFO_STREAM("Model: new CoM (" << p_newWorldCoordinatesCoM.x << ", "
                                            << p_newWorldCoordinatesCoM.y << ", "
                                            << p_newWorldCoordinatesCoM.q.x() << ","
                                            << p_newWorldCoordinatesCoM.q.y() << ","
                                            << p_newWorldCoordinatesCoM.q.z() << ","
                                            << p_newWorldCoordinatesCoM.q.w() << ")");
}

/**
 * Compute new CoM feet configuration
 * using the newly computed map feet
 * configuration and kinematic transforms.
 *
 * @param p_relativeStepPredictions
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeNewCoMFeetConfiguration(const std::vector<double> &p_relativeStepPredictions,
                                           const FeetConfiguration &p_currentFeetConfiguration,
                                           FeetConfiguration &p_newFeetConfiguration)
{
    ROS_INFO_STREAM("Model: Prev FL CoM: " << p_currentFeetConfiguration.flCoM.x << ", " << p_currentFeetConfiguration.flCoM.y);
    ROS_INFO_STREAM("Model: Prev FR CoM: " << p_currentFeetConfiguration.frCoM.x << ", " << p_currentFeetConfiguration.frCoM.y);
    ROS_INFO_STREAM("Model: Prev RL CoM: " << p_currentFeetConfiguration.rlCoM.x << ", " << p_currentFeetConfiguration.rlCoM.y);
    ROS_INFO_STREAM("Model: Prev RR CoM: " << p_currentFeetConfiguration.rrCoM.x << ", " << p_currentFeetConfiguration.rrCoM.y << "\n");

    for (auto &prediction: p_relativeStepPredictions)
        ROS_INFO_STREAM(prediction);

    p_newFeetConfiguration.flCoM.x = p_currentFeetConfiguration.flCoM.x + p_relativeStepPredictions[0];
    p_newFeetConfiguration.flCoM.y = p_currentFeetConfiguration.flCoM.y + p_relativeStepPredictions[1];

    p_newFeetConfiguration.frCoM.x = p_currentFeetConfiguration.frCoM.x + p_relativeStepPredictions[2];
    p_newFeetConfiguration.frCoM.y = p_currentFeetConfiguration.frCoM.y + p_relativeStepPredictions[3];

    p_newFeetConfiguration.rlCoM.x = p_currentFeetConfiguration.rlCoM.x + p_relativeStepPredictions[4];
    p_newFeetConfiguration.rlCoM.y = p_currentFeetConfiguration.rlCoM.y + p_relativeStepPredictions[5];

    p_newFeetConfiguration.rrCoM.x = p_currentFeetConfiguration.rrCoM.x + p_relativeStepPredictions[6];
    p_newFeetConfiguration.rrCoM.y = p_currentFeetConfiguration.rrCoM.y + p_relativeStepPredictions[7];

    ROS_INFO_STREAM("Model: New FL CoM: " << p_newFeetConfiguration.flCoM.x << ", " << p_newFeetConfiguration.flCoM.y);
    ROS_INFO_STREAM("Model: New FR CoM: " << p_newFeetConfiguration.frCoM.x << ", " << p_newFeetConfiguration.frCoM.y);
    ROS_INFO_STREAM("Model: New RL CoM: " << p_newFeetConfiguration.rlCoM.x << ", " << p_newFeetConfiguration.rlCoM.y);
    ROS_INFO_STREAM("Model: New RR CoM: " << p_newFeetConfiguration.rrCoM.x << ", " << p_newFeetConfiguration.rrCoM.y << "\n");
}

/**
 * Predicts new feet configuration using
 * the learnt models and extracts new CoM
 * from them.
 *
 * @param p_velocity
 * @param p_action
 * @param p_currentWorldCoordinatesCoM
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 * @param p_newWorldCoordinatesCoM
 */
void Model::predictNewConfiguration(double p_velocity,
                                    const Action &p_action,
                                    const World2D &p_currentWorldCoordinatesCoM,
                                    const FeetConfiguration &p_currentFeetConfiguration,
                                    FeetConfiguration &p_newFeetConfiguration,
                                    World2D &p_newWorldCoordinatesCoM)
{
    // Request message
    aliengo_navigation::FootstepPrediction l_footstepPredictionSrv;
    l_footstepPredictionSrv.request.fr_rl_swinging = p_currentFeetConfiguration.fr_rl_swinging;
    l_footstepPredictionSrv.request.x_velocity = p_action.x * p_velocity;
    l_footstepPredictionSrv.request.y_velocity = p_action.y * p_velocity;
    l_footstepPredictionSrv.request.theta_velocity = p_action.theta * p_velocity;
    l_footstepPredictionSrv.request.fl_x = p_currentFeetConfiguration.flCoM.x;
    l_footstepPredictionSrv.request.fl_y = p_currentFeetConfiguration.flCoM.y;
    l_footstepPredictionSrv.request.fr_x = p_currentFeetConfiguration.frCoM.x;
    l_footstepPredictionSrv.request.fr_y = p_currentFeetConfiguration.frCoM.y;
    l_footstepPredictionSrv.request.rl_x = p_currentFeetConfiguration.rlCoM.x;
    l_footstepPredictionSrv.request.rl_y = p_currentFeetConfiguration.rlCoM.y;
    l_footstepPredictionSrv.request.rr_x = p_currentFeetConfiguration.rrCoM.x;
    l_footstepPredictionSrv.request.rr_y = p_currentFeetConfiguration.rrCoM.y;

    // Call prediction service
    std::vector<float> l_predictions(10);
    if (m_footstepPredictionServiceClient.call(l_footstepPredictionSrv))
    {
        // Store service predictions
        l_predictions = l_footstepPredictionSrv.response.predictions;
    }
    else
    {
        ROS_ERROR("Model: Footstep prediction failed.");
    }

    // Compute new CoM
    computeNewCoM(p_action.theta * p_velocity,
                  l_predictions[0],
                  l_predictions[1],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    // Compute feet configuration w.r.t CoM
    std::vector<double> l_relativeStepPredictions = std::vector<double>(l_predictions.begin() + 2,
                                                                        l_predictions.end());
    computeNewCoMFeetConfiguration(l_relativeStepPredictions,
                                   p_currentFeetConfiguration,
                                   p_newFeetConfiguration);

    // Compute new map feet configuration
//    computeMapFeetConfiguration(l_predictions,
//                                p_currentFeetConfiguration,
//                                p_newFeetConfiguration);

    // Change swinging feet pair
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

//    int j = 0;
//
//    // Populate array
//    visualization_msgs::Marker l_footCommonMarker;
//    l_footCommonMarker.header.stamp = ros::Time::now();
//    l_footCommonMarker.header.frame_id = ROBOT_REFERENCE_FRAME;
//    l_footCommonMarker.type = 2;
//    l_footCommonMarker.action = 0;
//    l_footCommonMarker.lifetime = ros::Duration(0.005);
//    l_footCommonMarker.pose.orientation.x = 0;
//    l_footCommonMarker.pose.orientation.y = 0;
//    l_footCommonMarker.pose.orientation.z = 0;
//    l_footCommonMarker.pose.orientation.w = 1;
//    l_footCommonMarker.scale.x = 0.025;
//    l_footCommonMarker.scale.y = 0.025;
//    l_footCommonMarker.scale.z = 0.025;
//    l_footCommonMarker.color.r = 0;
//    l_footCommonMarker.color.g = 0;
//    l_footCommonMarker.color.b = 1;
//    l_footCommonMarker.color.a = 0.5;
//
//    visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
//    l_CoMMarker.id = j++;
//    l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//    l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
//    l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
//    l_CoMMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
//    l_flFootMarker.id = j++;
//    l_flFootMarker.pose.position.x = p_newFeetConfiguration.flCoM.x;
//    l_flFootMarker.pose.position.y = p_newFeetConfiguration.flCoM.y;
//    l_flFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//    l_frFootMarker.id = j++;
//    l_frFootMarker.pose.position.x = p_newFeetConfiguration.frCoM.x;
//    l_frFootMarker.pose.position.y = p_newFeetConfiguration.frCoM.y;
//    l_frFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//    l_rlFootMarker.id = j++;
//    l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlCoM.x;
//    l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlCoM.y;
//    l_rlFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//    l_rrFootMarker.id = j++;
//    l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrCoM.x;
//    l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrCoM.y;
//    l_rrFootMarker.pose.position.z = 0;
//
//    // Feet configuration array
//    // for visualization purposes
//    visualization_msgs::MarkerArray l_pathFeetConfiguration;
//    l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
//    l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);
//
//    m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
//    ros::Duration(0.1).sleep();
}