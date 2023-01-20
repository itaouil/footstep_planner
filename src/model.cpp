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
    /**
     * Models to predict CoM and feet displacements.
     */
    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x.resize(12);
    m_fr_rl_com_x << 0.06178605,  0.02591675,  0.19915162,
                     0.19162222,  0.07683036, -0.42039465,
                     0.34899808,  0.20692257, -0.14697638,
                     -0.36643112, -0.29781153, 0.02649472;
    m_fr_rl_com_y.resize(12);
    m_fr_rl_com_y << 0.00250727,  0.00822523, -0.04769517,
                     -0.02852027,  0.06069605, -0.13059516,
                     -0.40028706,  0.06454635, -0.31499078,
                     0.28044478, 0.29414499, 0.14520039;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(12);
    m_fl_rr_com_x << 0.06712592,  0.0263571 ,  0.18082497,
                     0.11632034, -0.08490007, -0.69271222,
                     0.48354626,  0.54327484, -0.14376796,
                     -0.43494328, -0.21201951, 0.2300637;
    m_fl_rr_com_y.resize(12);
    m_fl_rr_com_y << 0.00319445, -0.00564064,  0.06980717,
                     0.01053094, -0.45284026, -0.23117932,
                     0.17187332, -0.12034161,  0.1666867,
                     -0.02167592, -0.3114089, 0.03511225;

    // FL models coefficients
    m_fl_swinging_x.resize(12);
    m_fl_swinging_x << 0.12520463,  0.04246041,  0.34119206,
                       -0.81304698, -0.15552554, -1.24333084,
                       0.71232389,  0.55751168,  0.07107448,
                       -0.6045821, -0.52870332, 0.5194798;
    m_fl_swinging_y.resize(12);
    m_fl_swinging_y << 0.00397156, -0.00971387,  0.09850974,
                       0.15516211, -1.72035123, 0.03239494,
                       0.56516293, -0.55017488,  0.25105531,
                       -0.20210869, -0.32158347, -0.01860778;

    // FR models coefficients
    m_fr_swinging_x.resize(12);
    m_fr_swinging_x << 0.11804343,  0.0376525,  0.37260917,
                       0.0615454 ,  0.1015548, -1.50822778,
                       0.42237875,  0.2656965 ,  0.02732254,
                       -0.77314339, -0.87880512, 0.13918681;
    m_fr_swinging_y.resize(12);
    m_fr_swinging_y << 1.22387265e-03,  1.60324823e-02, -9.17800056e-02,
                       -5.68583957e-01,  3.30730090e-01, -4.47811069e-01,
                       -1.58265704e+00,  3.38569794e-01, -2.31948866e-01,
                       9.91330438e-01,  6.99098002e-01, 0.45803713;

    // RL models coefficients
    m_rl_swinging_x.resize(12);
    m_rl_swinging_x << 0.11712274,  0.03824899,  0.37737969,
                       0.02868657,  0.08206114, -1.48466829,
                       0.40986129,  0.25010099,  0.01424349,
                       -0.76032008, -0.89068153, 0.1418243;
    m_rl_swinging_y.resize(12);
    m_rl_swinging_y << 1.21906729e-03,  1.60029208e-02, -9.29178073e-02,
                       -5.75681714e-01,  3.30542938e-01, -4.37761688e-01,
                       -1.59421453e+00,  3.22897730e-01, -2.31592362e-01,
                       1.00051970e+00,  7.02782209e-01, 0.45464506;

    // RR models coefficients
    m_rr_swinging_x.resize(12);
    m_rr_swinging_x << 0.12478286,  0.04263852,  0.34180889,
                       -0.81072397, -0.15555825, -1.22410527,
                       0.7084508 ,  0.5458503 ,  0.09856872,
                       -0.59584767, -0.51355142, 0.51142222;
    m_rr_swinging_y.resize(12);
    m_rr_swinging_y << 0.00417166, -0.0096639 ,  0.09754762,
                       0.15618925, -1.72599809, 0.04252583,
                       0.55357672, -0.55715066,  0.25263708,
                       -0.20381262, -0.32270022, -0.02462579;

    /**
     * Models to predict next CoM velocity.
     */
    m_fr_rl_com_velocity.resize(12);
    m_fr_rl_com_velocity << 0.35423353,  0.21762567,  1.04715725,
                            -0.36719765, -0.76525679, -0.60520531,
                            1.43117104, -0.00609701, -0.51357836,
                            -3.92469285, -3.92751117, -0.93484729;
    m_fl_rr_com_velocity.resize(12);
    m_fl_rr_com_velocity << 0.39327239,  0.24016242,  0.89910553,
                            -0.16229841, -1.31370701, -3.67790885,
                            2.90730904, -0.37216612,  0.90804431,
                            -1.04003443, -1.01995773, 0.85769116;
}

/**
 * Motion prediction for the first step.
 *
 * @param p_plannedHorizon
 * @param p_previousVelocityX
 * @param p_previousVelocityY
 * @param p_previousAngularVelocity
 * @param p_nextVelocityX
 * @param p_nextVelocityY
 * @param p_nextAngularVelocity
 * @param p_currentFeetConfiguration
 * @param p_predictions
 */
void Model::motionPrediction(uint p_plannedHorizon,
                             double p_previousVelocityX,
                             double p_previousVelocityY,
                             double p_previousAngularVelocity,
                             double p_nextVelocityX,
                             double p_nextVelocityY,
                             double p_nextAngularVelocity,
                             const double &p_baseVelocity,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             std::vector<double> &p_predictions) {
    Eigen::VectorXd l_modelInput(12);
    l_modelInput << p_previousVelocityX,
                    p_nextVelocityX,
                    p_baseVelocity,
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
        p_predictions[0] = m_fr_rl_com_x * l_modelInput;
        p_predictions[1] = 0.0;

        // FL prediction
        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        // FR prediction
        p_predictions[4] = m_fr_swinging_x * l_modelInput;
        p_predictions[5] = 0.0;

        // RL prediction
        p_predictions[6] = m_rl_swinging_x * l_modelInput;
        p_predictions[7] = 0.0;

        // RR prediction
        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;

        // Theta (CoM) prediction
        p_predictions[10] = 0.0;
    }
    else {
        // CoM prediction
        p_predictions[0] = m_fl_rr_com_x * l_modelInput;
        p_predictions[1] = 0.0;

        // FL prediction
        p_predictions[2] = m_fl_swinging_x * l_modelInput;
        p_predictions[3] = 0.0;

        // FR prediction
        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        // RL prediction
        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

        // RR prediction
        p_predictions[8] = m_rr_swinging_x * l_modelInput;
        p_predictions[9] = 0.0;

        // Theta (CoM) prediction
        p_predictions[10] = 0.0;
    }

    // Predicted CoM velocity
    p_predictions[11] = velocityPrediction(p_previousVelocityX,
                                           p_previousVelocityY,
                                           p_previousAngularVelocity,
                                           p_nextVelocityX,
                                           p_nextVelocityY,
                                           p_nextAngularVelocity,
                                           p_baseVelocity,
                                           p_currentFeetConfiguration);
}

/**
 * Motion prediction for second step onwards.
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
double Model::velocityPrediction(double p_previousVelocityX,
                                 double p_previousVelocityY,
                                 double p_previousAngularVelocity,
                                 double p_nextVelocityX,
                                 double p_nextVelocityY,
                                 double p_nextAngularVelocity,
                                 double p_baseVelocity,
                                 const FeetConfiguration &p_currentFeetConfiguration) {
    Eigen::VectorXd l_modelInput(12);
    l_modelInput << p_previousVelocityX,
                    p_nextVelocityX,
                    p_baseVelocity,
                    p_currentFeetConfiguration.flCoM.x,
                    p_currentFeetConfiguration.flCoM.y,
                    p_currentFeetConfiguration.frCoM.x,
                    p_currentFeetConfiguration.frCoM.y,
                    p_currentFeetConfiguration.rlCoM.x,
                    p_currentFeetConfiguration.rlCoM.y,
                    p_currentFeetConfiguration.rrCoM.x,
                    p_currentFeetConfiguration.rrCoM.y,
                    1;

    ROS_DEBUG_STREAM("Velocity Input: " << l_modelInput);

    if (p_currentFeetConfiguration.fr_rl_swinging) {
        return m_fr_rl_com_velocity * l_modelInput;
    }
    else {
        return m_fl_rr_com_velocity * l_modelInput;
    }
}

/**
  * Compute new CoM in world frame.
  *
  * @param p_predictedCoMVelocity
  * @param p_predictedCoMDisplacementX
  * @param p_predictedCoMDisplacementY
  * @param p_predictedCoMDisplacementTheta,
  * @param p_currentWorldCoordinatesCoM
  * @param p_newWorldCoordinatesCoM
  */
void Model::computeNewCoM(const double p_predictedCoMDisplacementX,
                          const double p_predictedCoMDisplacementY,
                          const double p_predictedCoMDisplacementTheta,
                          const double p_predictedCoMVelocity,
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

    // Update predicted CoM velocity
    p_newWorldCoordinatesCoM.v = p_predictedCoMVelocity;

    // Update CoM position in world frame
    p_newWorldCoordinatesCoM.x = p_currentWorldCoordinatesCoM.x + l_displacementMapFrame(0);
    p_newWorldCoordinatesCoM.y = p_currentWorldCoordinatesCoM.y + l_displacementMapFrame(1);

    ROS_DEBUG_STREAM("CoM position: " << p_currentWorldCoordinatesCoM.x << ", " << p_currentWorldCoordinatesCoM.y);
    ROS_DEBUG_STREAM("CoM position2: " << p_newWorldCoordinatesCoM.x << ", " << p_newWorldCoordinatesCoM.y);

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
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    std::vector<double> l_predictions(12);
    motionPrediction(p_plannedFootstep,
                     p_action.x * p_previousVelocity,
                     p_action.y * p_previousVelocity,
                     p_action.theta * p_previousVelocity,
                     p_action.x * p_nextVelocity,
                     p_action.y * p_nextVelocity,
                     p_action.theta * p_nextVelocity,
                     p_currentWorldCoordinatesCoM.v,
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
                                    << l_predictions[10] <<","
                                    << l_predictions[11] <<"\n");

    computeNewCoM(l_predictions[0],
                  l_predictions[1],
                  l_predictions[10],
                  l_predictions[11],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    computeNewFeetConfiguration(p_newWorldCoordinatesCoM,
                                l_predictions,
                                p_currentFeetConfiguration,
                                p_newFeetConfiguration);

    //  // Publish predicted CoM and feet poses
    //  int j = 0;
    //  visualization_msgs::Marker l_footCommonMarker;
    //  l_footCommonMarker.header.stamp = ros::Time::now();
    //  l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    //  l_footCommonMarker.type = 2;
    //  l_footCommonMarker.action = 0;
    //  l_footCommonMarker.lifetime = ros::Duration(0.5);
    //  l_footCommonMarker.pose.orientation.x = p_currentWorldCoordinatesCoM.q.x();
    //  l_footCommonMarker.pose.orientation.y = p_currentWorldCoordinatesCoM.q.y();
    //  l_footCommonMarker.pose.orientation.z = p_currentWorldCoordinatesCoM.q.z();
    //  l_footCommonMarker.pose.orientation.w = p_currentWorldCoordinatesCoM.q.w();
    //  l_footCommonMarker.scale.x = 0.05;
    //  l_footCommonMarker.scale.y = 0.035;
    //  l_footCommonMarker.scale.z = 0.035;
    //  l_footCommonMarker.color.r = 0;
    //  l_footCommonMarker.color.g = 0;
    //  l_footCommonMarker.color.b = 1;
    //  l_footCommonMarker.color.a = 1;

    //  visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
    //  l_CoMMarker.id = j++;
    //  l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    //  l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
    //  l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
    //  l_CoMMarker.pose.position.z = 0.170;

    //  visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
    //  l_flFootMarker.id = j++;
    //  l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
    //  l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
    //  l_flFootMarker.pose.position.z = 0.170;

    //  visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
    //  l_frFootMarker.id = j++;
    //  l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
    //  l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
    //  l_frFootMarker.pose.position.z = 0.170;

    //  visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
    //  l_rlFootMarker.id = j++;
    //  l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
    //  l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
    //  l_rlFootMarker.pose.position.z = 0.170;

    //  visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
    //  l_rrFootMarker.id = j++;
    //  l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
    //  l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
    //  l_rrFootMarker.pose.position.z = 0.170;

    //  visualization_msgs::MarkerArray l_pathFeetConfiguration;
    //  l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
    //  l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
    //  l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
    //  l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
    //  l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

    //  m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
    //  ros::Duration(0.5).sleep();
}