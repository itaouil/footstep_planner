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

void Model::setModelsCoefficients() {
    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x.resize(11);
    m_fr_rl_com_x << 0.21180696,  0.08733433,  0.39375494,
                     0.20741955, -0.45455263, 0.29056373,
                     0.26806324, -0.04667911, -0.66246393,
                     -0.27890029, -0.10674002;
    m_fr_rl_com_y.resize(11);
    m_fr_rl_com_y << -0.04381483,  0.0099718 , -0.01461114,
                     0.10550614, -0.09817355, -0.37950534,
                     0.03110533, -0.31907396,  0.23638129,
                     0.26928122, 0.10893133;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(11);
    m_fl_rr_com_x << 0.16289642,  0.1023686,  0.13849986,
                     -0.13895984, -0.61910475, 0.399871,
                     0.45832429, -0.12584575, -0.49940014,
                     -0.25856767, 0.15796873;
    m_fl_rr_com_y.resize(11);
    m_fl_rr_com_y << 6.91454260e-02,  1.41992545e-04,  1.43289851e-02,
                     -4.67633707e-01, -2.52764358e-01,  2.31037194e-01,
                     -7.74581834e-02,  1.73254821e-01, -5.46286110e-03,
                     -3.29681899e-01, 0.06044258;

    // FL models coefficients
    m_fl_swinging_x.resize(11);
    m_fl_swinging_x << 0.29666337,  0.18482673, -0.7765262,
                       -0.16526346, -1.07857768, 0.5772457,
                       0.45551224,  0.10528592, -0.69932453,
                       -0.55829636, 0.39820581;
    m_fl_swinging_y.resize(11);
    m_fl_swinging_y << 9.09916869e-02,  5.23674531e-04,  1.70177436e-01,
                       -1.74003141e+00,  3.77223407e-02,  6.58163952e-01,
                       -4.82553917e-01,  2.46362760e-01, -1.71504402e-01,
                       -3.69134583e-01, 0.01024131;

    // FR models coefficients
    m_fr_swinging_x.resize(11);
    m_fr_swinging_x << 0.38168104,  0.15677369,  0.44126682,
                       0.30818634, -1.62732594, 0.34270941,
                       0.44436638,  0.1156409 , -1.20889462,
                       -0.88097806, -0.0350675;
    m_fr_swinging_y.resize(11);
    m_fr_swinging_y << -0.07741833,  0.01497329, -0.51843135,
                       0.45648639, -0.39901844, -1.47489132,
                       0.32354615, -0.29951593,  0.8586617,
                       0.58523714, 0.38954682;

    // RL models coefficients
    m_rl_swinging_x.resize(11);
    m_rl_swinging_x << 0.37867129,  0.15654508,  0.44412706,
                       0.31105931, -1.64181971, 0.33225906,
                       0.44500254,  0.10838261, -1.20622499,
                       -0.88374732, -0.03230986;
    m_rl_swinging_y.resize(11);
    m_rl_swinging_y << -0.07804146,  0.0147216 , -0.52369991,
                       0.43705526, -0.39184752, -1.48769178,
                       0.31375446, -0.28275444,  0.86634309,
                       0.59015206, 0.38787695;

    // RR models coefficients
    m_rr_swinging_x.resize(11);
    m_rr_swinging_x << 0.30877573,  0.18274829, -0.77255194,
                       -0.19316678, -1.10284751, 0.58621226,
                       0.44564968,  0.10471284, -0.66951646,
                       -0.5662069, 0.41195238;
    m_rr_swinging_y.resize(11);
    m_rr_swinging_y << 8.94437610e-02,  4.14451818e-04,  1.79056412e-01,
                       -1.74286504e+00,  4.86028860e-02,  6.38335933e-01,
                       -4.79851807e-01,  2.59397546e-01, -1.76368133e-01,
                       -3.47457899e-01, 0.00390601;
}

/**
 * Motion prediction.
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
void Model::motionPrediction(double p_previousVelocityX,
                             double p_previousVelocityY,
                             double p_previousAngularVelocity,
                             double p_nextVelocityX,
                             double p_nextVelocityY,
                             double p_nextAngularVelocity,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             std::vector<double> &p_predictions) {
    Eigen::VectorXd l_modelInput(12);
    l_modelInput << p_previousVelocityX,
                    p_nextVelocityX,
                    p_odomVelocityState.linear.x,
                    p_currentFeetConfiguration.flCoM.x,
                    p_currentFeetConfiguration.flCoM.y,
                    p_currentFeetConfiguration.frCoM.x,
                    p_currentFeetConfiguration.frCoM.y,
                    p_currentFeetConfiguration.rlCoM.x,
                    p_currentFeetConfiguration.rlCoM.y,
                    p_currentFeetConfiguration.rrCoM.x,
                    p_currentFeetConfiguration.rrCoM.y,
                    1;

    ROS_INFO_STREAM("Input: " << l_modelInput);

    if (p_currentFeetConfiguration.fr_rl_swinging) {
        // CoM prediction
        p_predictions[0] = m_fr_rl_com_x_fs * l_modelInput;
        p_predictions[1] = 0.0;

        // FL prediction
        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        // FR prediction
        p_predictions[4] = m_fr_swinging_x_fs * l_modelInput;
        p_predictions[5] = 0.0;

        // RL prediction
        p_predictions[6] = m_rl_swinging_x_fs * l_modelInput;
        p_predictions[7] = 0.0;

        // RR prediction
        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;

        // Theta (CoM) prediction
        p_predictions[10] = 0.0;
    }
    else {
        // CoM prediction
        p_predictions[0] = m_fl_rr_com_x_fs * l_modelInput;
        p_predictions[1] = 0.0;

        // FL prediction
        p_predictions[2] = m_fl_swinging_x_fs * l_modelInput;
        p_predictions[3] = 0.0;

        // FR prediction
        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        // RL prediction
        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

        // RR prediction
        p_predictions[8] = m_rr_swinging_x_fs * l_modelInput;
        p_predictions[9] = 0.0;

        // Theta (CoM) prediction
        p_predictions[10] = 0.0;
    }
}

/**
  * Compute new CoM in world frame.
  *
  * @param p_predictedCoMDisplacementX
  * @param p_predictedCoMDisplacementY
  * @param p_predictedCoMDisplacementTheta,
  * @param p_currentWorldCoordinatesCoM
  * @param p_newWorldCoordinatesCoM
  */
void Model::computeNewCoM(const double p_predictedCoMDisplacementX,
                          const double p_predictedCoMDisplacementY,
                          const double p_predictedCoMDisplacementTheta,
                          const World3D &p_currentWorldCoordinatesCoM,
                          World3D &p_newWorldCoordinatesCoM) {
    // Get rotation matrix of 
    // the base w.r.t world
    Eigen::Quaterniond q;
    q.x() = p_currentWorldCoordinatesCoM.q.x();
    q.y() = p_currentWorldCoordinatesCoM.q.y();
    q.z() = p_currentWorldCoordinatesCoM.q.z();
    q.w() = p_currentWorldCoordinatesCoM.q.w();
    Eigen::Matrix3d RWorldBase = q.normalized().toRotationMatrix();

    // CoM displacement
    Eigen::Vector3d l_displacementCoMFrame{p_predictedCoMDisplacementX, p_predictedCoMDisplacementY, 0.0};

    // Map displacement
    Eigen::Vector3d l_displacementMapFrame = RWorldBase.transpose() * l_displacementCoMFrame;

    // Update CoM position in world frame
    p_newWorldCoordinatesCoM.x = p_currentWorldCoordinatesCoM.x + l_displacementMapFrame(0);
    p_newWorldCoordinatesCoM.y = p_currentWorldCoordinatesCoM.y + l_displacementMapFrame(1);

    // Compute quaternion representation of predicted rotation
    tf2::Quaternion l_velocityCommandQuaternion;
    //l_velocityCommandQuaternion.setRPY(0, 0, p_predictedCoMDisplacementTheta);

    // Apply predicted quaternion rotation to
    // current CoM's rotation in world frame
//     tf2::Quaternion l_newCoMRotation = p_currentWorldCoordinatesCoM.q * l_velocityCommandQuaternion;
//     l_newCoMRotation.normalize();
//     p_newWorldCoordinatesCoM.q = l_newCoMRotation;
    p_newWorldCoordinatesCoM.q = p_currentWorldCoordinatesCoM.q;
}

/**
 * Compute the new feet configuration.
 *
 * @param p_predictions
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeNewFeetConfiguration(const World3D &p_newWorldCoordinatesCoM,
                                        const std::vector<double> &p_predictions,
                                        const FeetConfiguration &p_currentFeetConfiguration,
                                        FeetConfiguration &p_newFeetConfiguration) {
    // Get rotation matrix of
    // the base w.r.t world
    Eigen::Quaterniond q;
    q.x() = p_newWorldCoordinatesCoM.q.x();
    q.y() = p_newWorldCoordinatesCoM.q.y();
    q.z() = p_newWorldCoordinatesCoM.q.z();
    q.w() = p_newWorldCoordinatesCoM.q.w();
    Eigen::Matrix3d RWorldBase = q.normalized().toRotationMatrix();

    // Transform feet prediction 
    // from CoM to world frame
    std::vector<double> l_feetPredictionWorldFrame;
    Eigen::Vector3d l_feetPredictionCoMFrame;
    for (int x = 0; x < 4; x++) {
        l_feetPredictionCoMFrame << p_predictions[2 * x + 2], p_predictions[2 * x + 3], 0.0;
        Eigen::Vector3d l_footDisplacementWorldFrame = RWorldBase.transpose() * l_feetPredictionCoMFrame;
        l_feetPredictionWorldFrame.push_back(l_footDisplacementWorldFrame(0));
        l_feetPredictionWorldFrame.push_back(l_footDisplacementWorldFrame(1));

        ROS_DEBUG_STREAM("CoM (x): " << l_feetPredictionCoMFrame(0) << ". CoM (y): " << l_feetPredictionCoMFrame(1));
        ROS_DEBUG_STREAM("Map (x): " << l_footDisplacementWorldFrame(0) << ". Map(y): " << l_footDisplacementWorldFrame(1));
    }

    // Feet poses in world frame
    p_newFeetConfiguration.flMap.x = p_currentFeetConfiguration.flMap.x + l_feetPredictionWorldFrame[0];
    p_newFeetConfiguration.flMap.y = p_currentFeetConfiguration.flMap.y + l_feetPredictionWorldFrame[1];

    p_newFeetConfiguration.frMap.x = p_currentFeetConfiguration.frMap.x + l_feetPredictionWorldFrame[2];
    p_newFeetConfiguration.frMap.y = p_currentFeetConfiguration.frMap.y + l_feetPredictionWorldFrame[3];

    p_newFeetConfiguration.rlMap.x = p_currentFeetConfiguration.rlMap.x + l_feetPredictionWorldFrame[4];
    p_newFeetConfiguration.rlMap.y = p_currentFeetConfiguration.rlMap.y + l_feetPredictionWorldFrame[5];

    p_newFeetConfiguration.rrMap.x = p_currentFeetConfiguration.rrMap.x + l_feetPredictionWorldFrame[6];
    p_newFeetConfiguration.rrMap.y = p_currentFeetConfiguration.rrMap.y + l_feetPredictionWorldFrame[7];

    // Feet poses in CoM frame
    p_newFeetConfiguration.flCoM.x = std::abs(p_newFeetConfiguration.flMap.x - p_newWorldCoordinatesCoM.x);
    p_newFeetConfiguration.flCoM.y = std::abs(p_newFeetConfiguration.flMap.y - p_newWorldCoordinatesCoM.y);

    p_newFeetConfiguration.frCoM.x = std::abs(p_newFeetConfiguration.frMap.x - p_newWorldCoordinatesCoM.x);
    p_newFeetConfiguration.frCoM.y = -std::abs(p_newFeetConfiguration.frMap.y - p_newWorldCoordinatesCoM.y);

    p_newFeetConfiguration.rlCoM.x = -std::abs(p_newFeetConfiguration.rlMap.x - p_newWorldCoordinatesCoM.x);
    p_newFeetConfiguration.rlCoM.y = std::abs(p_newFeetConfiguration.rlMap.y - p_newWorldCoordinatesCoM.y);

    p_newFeetConfiguration.rrCoM.x = -std::abs(p_newFeetConfiguration.rrMap.x - p_newWorldCoordinatesCoM.x);
    p_newFeetConfiguration.rrCoM.y = -std::abs(p_newFeetConfiguration.rrMap.y - p_newWorldCoordinatesCoM.y);

    ROS_DEBUG_STREAM("Pose: " << p_newWorldCoordinatesCoM.x << ", " << p_newWorldCoordinatesCoM.y);
    ROS_DEBUG_STREAM("FL: " << p_newFeetConfiguration.flMap.x << ", " << p_newFeetConfiguration.flMap.y);
    ROS_DEBUG_STREAM("FR: " << p_newFeetConfiguration.frMap.x << ", " << p_newFeetConfiguration.frMap.y);
    ROS_DEBUG_STREAM("RL: " << p_newFeetConfiguration.rlMap.x << ", " << p_newFeetConfiguration.rlMap.y);
    ROS_DEBUG_STREAM("RR: " << p_newFeetConfiguration.rrMap.x << ", " << p_newFeetConfiguration.rrMap.y << "\n\n");

    // Change swinging sequence
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;
}

/**
 * Predicts new robot state given
 * the previous state and a new
 * velocity.
 *
 * @param p_previousVelocity
 * @param p_nextVelocity
 * @param p_action
 * @param p_currentWorldCoordinatesCoM
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 * @param p_newWorldCoordinatesCoM
 */
void Model::predictNextState(uint p_plannedFootstep,
                             double p_previousVelocity,
                             double p_nextVelocity,
                             const Action &p_action,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    std::vector<double> l_predictions(11);

    motionPrediction(p_action.x * p_previousVelocity,
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
    ROS_INFO_STREAM("Predictions: " << l_predictions[0] << ", "
                                    << l_predictions[1] << ", "
                                    << l_predictions[2] << ", "
                                    << l_predictions[3] << ", "
                                    << l_predictions[4] << ", "
                                    << l_predictions[5] << ", "
                                    << l_predictions[6] << ", "
                                    << l_predictions[7] << ", "
                                    << l_predictions[8] << ", "
                                    << l_predictions[9] << ", "
                                    << l_predictions[10] <<"\n");

    computeNewCoM(l_predictions[0],
                  l_predictions[1],
                  l_predictions[10],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    computeNewFeetConfiguration(p_newWorldCoordinatesCoM,
                                l_predictions,
                                p_currentFeetConfiguration,
                                p_newFeetConfiguration);

//     // Publish predicted CoM and feet poses
//     int j = 0;
//     visualization_msgs::Marker l_footCommonMarker;
//     l_footCommonMarker.header.stamp = ros::Time::now();
//     l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//     l_footCommonMarker.type = 2;
//     l_footCommonMarker.action = 0;
//     l_footCommonMarker.lifetime = ros::Duration(0.5);
//     l_footCommonMarker.pose.orientation.x = p_currentWorldCoordinatesCoM.q.x();
//     l_footCommonMarker.pose.orientation.y = p_currentWorldCoordinatesCoM.q.y();
//     l_footCommonMarker.pose.orientation.z = p_currentWorldCoordinatesCoM.q.z();
//     l_footCommonMarker.pose.orientation.w = p_currentWorldCoordinatesCoM.q.w();
//     l_footCommonMarker.scale.x = 0.05;
//     l_footCommonMarker.scale.y = 0.035;
//     l_footCommonMarker.scale.z = 0.035;
//     l_footCommonMarker.color.r = 0;
//     l_footCommonMarker.color.g = 1;
//     l_footCommonMarker.color.b = 0;
//     l_footCommonMarker.color.a = 0.7;

//     visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
//     l_CoMMarker.id = j++;
//     l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//     l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
//     l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
//     l_CoMMarker.pose.position.z = 0.170;

//     visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
//     l_flFootMarker.id = j++;
//     l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
//     l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
//     l_flFootMarker.pose.position.z = 0.170;

//     visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//     l_frFootMarker.id = j++;
//     l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
//     l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
//     l_frFootMarker.pose.position.z = 0.170;

//     visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//     l_rlFootMarker.id = j++;
//     l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
//     l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
//     l_rlFootMarker.pose.position.z = 0.170;

//     visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//     l_rrFootMarker.id = j++;
//     l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
//     l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
//     l_rrFootMarker.pose.position.z = 0.170;

//     visualization_msgs::MarkerArray l_pathFeetConfiguration;
//     l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
//     l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

//     m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
//     ros::Duration(0.51).sleep();
}