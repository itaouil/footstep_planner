/*
 * model.cpp
 *
 *  Created on: Aug 24, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "model.hpp"

/**
 * Constructor
 */
Model::Model(ros::NodeHandle &p_nh) :
        m_nh(p_nh), m_listener(m_buffer) {
    // Feet configuration marker array publisher
    m_feetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(
            PREDICTED_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

    // Set models coefficients
    setModelsCoefficients();
}

/**
 * Destructor
 */
Model::~Model() = default;

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of continuous
 * velocity commands.
 */
void Model::setModelsCoefficients() {
    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x.resize(15);
    m_fr_rl_com_x << 0.01009882, -0.00138146,  0.00591929,  0.1437222 , -0.00433467,
            0.00130813,  0.07772784,  0.16929027, -0.17838602,  0.00798173,
            -0.04074072,  0.04963839,  0.10051764,  0.03709051, 0.00490574;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << -0.00971856, -0.01017369,  0.00118772, -0.01158384,  0.15330004,
            -0.00105279, -0.11278145, -0.24258692, -0.02812024, -0.18596375,
            -0.09274634, -0.15458673, -0.07468321, -0.1887789, 0.00334893;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << -0.00118848,  0.00476276, -0.00575461,  0.13961594,  0.01091284,
            -0.00196097, -0.18913551,  0.15876759, -0.08021554,  0.19394194,
            -0.08546236,  0.30269309, -0.0932808 ,  0.16249433, 0.00825741;
    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << 0.00176666,  0.00383346, -0.00223724,  0.00324382,  0.15506061,
            -0.00033925, -0.01565562, -0.18706853,  0.06250759, -0.07208814,
            -0.03400149,  0.03670775,  0.10393649, -0.09758938, -0.00280065;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << -0.03221593,  0.04414323, -0.05045393,  0.22381182,  0.00345068,
            -0.00772306,  0.06273285,  0.04079152, -0.42993439,  0.60059794,
            -0.51035987,  0.55116634, -0.00925446,  0.15409359, 0.01148435;
    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << 0.00395575, -0.0080326 ,  0.04506687,  0.00806839,  0.2276758 ,
            0.02314851, -0.11235131,  0.11012614, -0.13762396,  0.01803263,
            -0.13334832,  0.09655311, -0.04729901,  0.0939064, 0.00013964;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << -0.03553982, -0.01727127,  0.05305336,  0.23106679, -0.02735418,
            0.00833077, -0.26936718,  0.70910105,  0.07247322,  0.37863626,
            0.06284863,  0.28592323, -0.40075097,  0.75532028, 0.01342904;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << -0.03401956, -0.00950205, -0.04755827,  0.00127786,  0.22282807,
            -0.02113752, -0.08170869, -0.41872454,  0.16559434, -0.12996355,
            0.10822455, -0.13786639, -0.06991281, -0.38518903, 0.00108043;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << -0.03422896, -0.01169688, -0.05116886,  0.23290612, -0.02949314,
            -0.00997868, -0.214117  ,  0.73927802,  0.01575591,  0.42919767,
            0.11276529,  0.23439538, -0.45176622,  0.68855996, 0.01012326;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y << -0.03401956, -0.00950205, -0.04755827,  0.00127786,  0.22282807,
            -0.02113752, -0.08170869, -0.41872454,  0.16559434, -0.12996355,
            0.10822455, -0.13786639, -0.06991281, -0.38518903, 0.00108043;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << -0.03228989,  0.04333071,  0.04734508,  0.22535789,  0.00342114,
            0.01462493,  0.07372918,  0.04560112, -0.44280749,  0.59339176,
            -0.47473278,  0.52993172, -0.01489859,  0.11369561, 0.0101464;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y << 0.00430843, -0.0094233 , -0.04601245,  0.0095601 ,  0.22908025,
            -0.02334645, -0.14012686,  0.07798112, -0.09587673, -0.00097512,
            -0.17499111,  0.12768332, -0.01252009,  0.14206591, -0.00334501;
}

/**
 * Compute CoM and feet displacements
 * predictions when a discontinuous velocity
 * is used (i.e. acceleration).
 *
 * @param p_previousVelocityX
 * @param p_previousVelocityY
 * @param p_previousAngularVelocity
 * @param p_nextVelocityX
 * @param p_nextVelocityY
 * @param p_nextAngularVelocity
 * @param p_currentFeetConfiguration
 * @param p_predictions
 */
void Model::prediction(double p_previousVelocityX,
                       double p_previousVelocityY,
                       double p_previousAngularVelocity,
                       double p_nextVelocityX,
                       double p_nextVelocityY,
                       double p_nextAngularVelocity,
                       const geometry_msgs::Twist &p_odomVelocityState,
                       const FeetConfiguration &p_currentFeetConfiguration,
                       std::vector<double> &p_predictions) {
    // Common input to all models
    Eigen::VectorXd l_modelInput(15);
    l_modelInput << p_previousVelocityX,
            p_previousVelocityY,
            p_previousAngularVelocity,
            p_nextVelocityX,
            p_nextVelocityY,
            p_nextAngularVelocity,
            p_currentFeetConfiguration.flCoM.x,
            p_currentFeetConfiguration.flCoM.y,
            p_currentFeetConfiguration.frCoM.x,
            p_currentFeetConfiguration.frCoM.y,
            p_currentFeetConfiguration.rlCoM.x,
            p_currentFeetConfiguration.rlCoM.y,
            p_currentFeetConfiguration.rrCoM.x,
            p_currentFeetConfiguration.rrCoM.y,
            1;

    ROS_DEBUG_STREAM("FL CoM Pose: " << p_currentFeetConfiguration.flCoM.x << ", " << p_currentFeetConfiguration.flCoM.y);
    ROS_DEBUG_STREAM("FR CoM Pose: " << p_currentFeetConfiguration.frCoM.x << ", " << p_currentFeetConfiguration.frCoM.y);
    ROS_DEBUG_STREAM("RL CoM Pose: " << p_currentFeetConfiguration.rlCoM.x << ", " << p_currentFeetConfiguration.rlCoM.y);
    ROS_DEBUG_STREAM("RR CoM Pose: " << p_currentFeetConfiguration.rrCoM.x << ", " << p_currentFeetConfiguration.rrCoM.y);

    // FR/RL are swinging
    if (p_currentFeetConfiguration.fr_rl_swinging) {
        p_predictions[0] = m_fr_rl_com_x * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y * l_modelInput;

        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        p_predictions[4] = m_fr_swinging_x * l_modelInput;
        p_predictions[5] = m_fr_swinging_y * l_modelInput;

        p_predictions[6] = m_rl_swinging_x * l_modelInput;
        p_predictions[7] = m_rl_swinging_y * l_modelInput;

        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;
    }
        // FL/RR are swinging
    else {
        p_predictions[0] = m_fl_rr_com_x * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y * l_modelInput;

        p_predictions[2] = m_fl_swinging_x * l_modelInput;
        p_predictions[3] = m_fl_swinging_y * l_modelInput;

        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

        p_predictions[8] = m_rr_swinging_x * l_modelInput;
        p_predictions[9] = m_rr_swinging_y * l_modelInput;
    }
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
                          const World3D &p_currentWorldCoordinatesCoM,
                          World3D &p_newWorldCoordinatesCoM) {
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
    if (p_angularVelocity != 0) {
        // Get yaw rotation in quaternion form
        tf2::Quaternion l_velocityCommandQuaternion;
        l_velocityCommandQuaternion.setRPY(0, 0, p_angularVelocity * TIMESTAMP);

        // Apply rotation command rotation to CoM quaternion
        tf2::Quaternion l_newCoMRotation = p_currentWorldCoordinatesCoM.q * l_velocityCommandQuaternion;
        l_newCoMRotation.normalize();

        // Set new CoM rotation
        p_newWorldCoordinatesCoM.q = l_newCoMRotation;
    } else {
        p_newWorldCoordinatesCoM.q = p_currentWorldCoordinatesCoM.q;
    }
}

/**
 * Compute new CoM feet configuration
 * using the newly computed map feet
 * configuration and kinematic transforms.
 *
 * @param p_predictions
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeNewFeetConfiguration(const World3D &p_newWorldCoordinatesCoM,
                                        const std::vector<double> &p_predictions,
                                        const FeetConfiguration &p_currentFeetConfiguration,
                                        FeetConfiguration &p_newFeetConfiguration) {
    // Map poses
    p_newFeetConfiguration.flMap.x = p_currentFeetConfiguration.flMap.x + p_predictions[2];
    p_newFeetConfiguration.flMap.y = p_currentFeetConfiguration.flMap.y + p_predictions[3];

    p_newFeetConfiguration.frMap.x = p_currentFeetConfiguration.frMap.x + p_predictions[4];
    p_newFeetConfiguration.frMap.y = p_currentFeetConfiguration.frMap.y + p_predictions[5];

    p_newFeetConfiguration.rlMap.x = p_currentFeetConfiguration.rlMap.x + p_predictions[6];
    p_newFeetConfiguration.rlMap.y = p_currentFeetConfiguration.rlMap.y + p_predictions[7];

    p_newFeetConfiguration.rrMap.x = p_currentFeetConfiguration.rrMap.x + p_predictions[8];
    p_newFeetConfiguration.rrMap.y = p_currentFeetConfiguration.rrMap.y + p_predictions[9];

    // CoM Poses
    p_newFeetConfiguration.flCoM.x = p_newFeetConfiguration.flMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.flCoM.y = p_newFeetConfiguration.flMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.frCoM.x = p_newFeetConfiguration.frMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.frCoM.y = p_newFeetConfiguration.frMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.rlCoM.x = p_newFeetConfiguration.rlMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.rlCoM.y = p_newFeetConfiguration.rlMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.rrCoM.x = p_newFeetConfiguration.rrMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.rrCoM.y = p_newFeetConfiguration.rrMap.y - p_newWorldCoordinatesCoM.y;

//    ROS_INFO_STREAM("Planner: CoM (MAP) " << p_newWorldCoordinatesCoM.x << ", " << p_newWorldCoordinatesCoM.y);
//
//
//    ROS_INFO_STREAM("Model: FL (MAP) " << p_newFeetConfiguration.flMap.x << ", " << p_newFeetConfiguration.flMap.y);
//    ROS_INFO_STREAM("Model: FL (COM) " << p_newFeetConfiguration.flCoM.x << ", " << p_newFeetConfiguration.flCoM.y << "\n");
//
//    ROS_INFO_STREAM("Model: FR (MAP) " << p_newFeetConfiguration.frMap.x << ", " << p_newFeetConfiguration.frMap.y);
//    ROS_INFO_STREAM("Model: FL (COM) " << p_newFeetConfiguration.frCoM.x << ", " << p_newFeetConfiguration.frCoM.y << "\n");
//
//    ROS_INFO_STREAM("Model: RL (MAP) " << p_newFeetConfiguration.rlMap.x << ", " << p_newFeetConfiguration.rlMap.y);
//    ROS_INFO_STREAM("Model: RL (COM) " << p_newFeetConfiguration.rlCoM.x << ", " << p_newFeetConfiguration.rlCoM.y << "\n");
//
//    ROS_INFO_STREAM("Model: RR (MAP) " << p_newFeetConfiguration.rrMap.x << ", " << p_newFeetConfiguration.rrMap.y);
//    ROS_INFO_STREAM("Model: RR (COM) " << p_newFeetConfiguration.rrCoM.x << ", " << p_newFeetConfiguration.rrCoM.y << "\n");

    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;
}

/**
 * Predicts new feet configuration using
 * the learnt models and extracts new CoM
 * from them.
 *
 * @param p_accelerating
 * @param p_previousVelocity
 * @param p_nextVelocity
 * @param p_action
 * @param p_currentWorldCoordinatesCoM
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 * @param p_newWorldCoordinatesCoM
 */
void Model::predictNextState(bool p_accelerating,
                             double p_previousVelocity,
                             double p_nextVelocity,
                             const Action &p_action,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    // Predict feet and CoM displacements
    std::vector<double> l_predictions(10);
    prediction(p_action.x * p_previousVelocity,
               p_action.y * p_previousVelocity,
               p_action.theta * p_previousVelocity,
               p_action.x * p_nextVelocity,
               p_action.y * p_nextVelocity,
               p_action.theta * p_nextVelocity,
               p_odomVelocityState,
               p_currentFeetConfiguration,
               l_predictions);

    ROS_DEBUG_STREAM("Prev Velocity: " << p_previousVelocity);
    ROS_DEBUG_STREAM("Next Velocity: " << p_nextVelocity);

    ROS_DEBUG_STREAM("Predictions: " << l_predictions[0] << ", "
                                    << l_predictions[1] << ", "
                                    << l_predictions[2] << ", "
                                    << l_predictions[3] << ", "
                                    << l_predictions[4] << ", "
                                    << l_predictions[5] << ", "
                                    << l_predictions[6] << ", "
                                    << l_predictions[7] << ", "
                                    << l_predictions[8] << ", "
                                    << l_predictions[9] << "\n");

    // Compute new CoM
    computeNewCoM(p_action.theta * p_nextVelocity,
                  l_predictions[0],
                  l_predictions[1],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    // Compute new feet configuration
    computeNewFeetConfiguration(p_newWorldCoordinatesCoM,
                                l_predictions,
                                p_currentFeetConfiguration,
                                p_newFeetConfiguration);

    // Change swinging feet pair
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

    // Publish predicted CoM and feet poses
    int j = 0;
    visualization_msgs::Marker l_footCommonMarker;
    l_footCommonMarker.header.stamp = ros::Time::now();
    l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_footCommonMarker.type = 2;
    l_footCommonMarker.action = 0;
    l_footCommonMarker.lifetime = ros::Duration(1);
    l_footCommonMarker.pose.orientation.x = 0;
    l_footCommonMarker.pose.orientation.y = 0;
    l_footCommonMarker.pose.orientation.z = 0;
    l_footCommonMarker.pose.orientation.w = 1;
    l_footCommonMarker.scale.x = 0.035;
    l_footCommonMarker.scale.y = 0.035;
    l_footCommonMarker.scale.z = 0.035;
    l_footCommonMarker.color.r = 0;
    l_footCommonMarker.color.g = 1;
    l_footCommonMarker.color.b = 0;
    l_footCommonMarker.color.a = 0.5;

    visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
    l_CoMMarker.id = j++;
    l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
    l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
    l_CoMMarker.pose.position.z = 0;

    visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
    l_flFootMarker.id = j++;
    l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
    l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
    l_flFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
    l_frFootMarker.id = j++;
    l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
    l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
    l_frFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
    l_rlFootMarker.id = j++;
    l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
    l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
    l_rlFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
    l_rrFootMarker.id = j++;
    l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
    l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
    l_rrFootMarker.pose.position.z = 0;

    visualization_msgs::MarkerArray l_pathFeetConfiguration;
    l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
    l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

    m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
    ros::Duration(1).sleep();
}