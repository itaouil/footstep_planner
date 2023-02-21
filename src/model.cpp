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

    // Set models coefficients (real robot)
    if (SIMULATION) {
        setModelsCoefficientsSimulation();
    }
    else {
        setModelsCoefficientsReal();
    }


}

/**
 * Destructor
 */
Model::~Model() = default;

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of continuous
 * velocity commands for the real robot;
 */
void Model::setModelsCoefficientsReal() {
    // CoM models coefficients
    m_com_x.resize(13);
    m_com_x << 0.06264909,  0.02733264, -0.05036305,  0.244094  ,  0.02991019,
            -0.28931173, -0.18867609,  0.36820637,  0.2773765 ,  0.21856981,
            0.06446532, -0.11773677, 0.16608383;
    m_com_y.resize(13);
    m_com_y << 0.00405434,  0.00166785, -0.00588239,  0.01704784, -0.06352501,
            -0.29390253, -0.01308521, -0.15528665,  0.0446051 ,  0.03824913,
            0.10537825,  0.03770023, 0.07477347;

    // Feet models coefficients
    m_feet_x.resize(13);
    m_feet_x << 0.10610534,  0.05715044, -0.07611036,  0.50669026, -0.30422955,
            0.24863965, -0.30175035, -0.24864653, -0.03481643, -0.85619672,
            -0.03234298,  0.86331222, 0.283676;
    m_feet_y.resize(13);
    m_feet_y << 0.02144497,  0.00482488, -0.02557683,  0.00066486,  0.12787462,
            -0.31741812,  0.12455281,  0.31194578,  0.47594208,  0.1570708 ,
            0.47252198, -0.15308805, 0.22118819;

    m_com_velocity.resize(5);
    m_com_velocity << 0.53686853,  0.29476966, -0.2297831 ,  0.43152223, -0.00918872;
}

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of continuous
 * velocity commands for the simulated robot;
 */
void Model::setModelsCoefficientsSimulation() {
    // CoM models coefficients
    m_com_x.resize(13);
    m_com_x << 0.45633932,  0.14248853,  0.05553078, -0.37462352, -0.25703703,
            0.34199386, -0.20939287, -0.39717463,  0.02317998, -0.076498  ,
            0.07556964,  0.13677518, 0.0723266;
    m_com_y.resize(11);
    m_com_y << -0.0042293 ,  0.0011165 ,  0.00043951,  0.00694202,  0.11345369,
            0.1047109 , -0.0470131 ,  0.08688261,  0.03744656, -0.0103423 ,
            -0.15063025,  0.03841101, -0.0420708;

    // Feet models coefficients
    m_feet_x.resize(13);
    m_feet_x << 0.20658539,  0.30275748, -0.03327917,  0.0893492 , -0.44618519,
            0.12027723, -0.471221  , -0.1374103 ,  0.01967117,  0.16829186,
            -0.00392792, -0.16360007, 0.15609747;
    m_feet_y.resize(13);
    m_feet_y << 0.07292048,  0.00122696, -0.00972269, -0.07540138,  0.055338  ,
            -0.00063982,  0.05034544,  0.00198942, -0.0410162 , -0.14807041,
            -0.04678586,  0.14940953, -0.00957753;

    m_com_velocity.resize(5);
    m_com_velocity << 0.1066685 ,  0.86477766,  0.00261115, -0.04178764, -0.01119369;
}

/**
 * Motion prediction.
 *
 * @param p_plannedHorizon
 * @param p_previousVelocityX
 * @param p_previousVelocityY
 * @param p_previousAngularVelocity
 * @param p_nextVelocityX
 * @param p_nextVelocityY
 * @param p_nextAngularVelocity
 * @param p_previousCoMVelocity
 * @param p_currentCoMVelocity
 * @param p_currentFeetConfiguration
 * @param p_predictions
 */
void Model::motionPrediction(const uint &p_plannedHorizon,
                             const double &p_previousVelocityX,
                             const double &p_previousVelocityY,
                             const double &p_previousAngularVelocity,
                             const double &p_nextVelocityX,
                             const double &p_nextVelocityY,
                             const double &p_nextAngularVelocity,
                             const double &p_previousCoMVelocity,
                             const double &p_currentCoMVelocity,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             std::vector<double> &p_predictions) {
    Eigen::VectorXd l_modelInput(13);
    l_modelInput << p_previousVelocityX,
                    p_nextVelocityX,
                    p_previousCoMVelocity,
                    p_currentCoMVelocity,
                    p_currentFeetConfiguration.flCoM.x,
                    p_currentFeetConfiguration.flCoM.y,
                    p_currentFeetConfiguration.frCoM.x,
                    p_currentFeetConfiguration.frCoM.y,
                    p_currentFeetConfiguration.rlCoM.x,
                    p_currentFeetConfiguration.rlCoM.y,
                    p_currentFeetConfiguration.rrCoM.x,
                    p_currentFeetConfiguration.rrCoM.y,
                    1;

    ROS_DEBUG_STREAM("Input: " << l_modelInput);

    // CoM prediction
    p_predictions[0] = m_com_x * l_modelInput;
    p_predictions[1] = 0;

    // Feet prediction
    p_predictions[2] = m_feet_x * l_modelInput;
    p_predictions[3] = 0;

    // Theta (CoM) prediction
    p_predictions[4] = 0.0;

    // Predicted CoM velocity
    p_predictions[5] = velocityPrediction(p_previousVelocityX,
                                          p_previousVelocityY,
                                          p_previousAngularVelocity,
                                          p_nextVelocityX,
                                          p_nextVelocityY,
                                          p_nextAngularVelocity,
                                          p_previousCoMVelocity,
                                          p_currentCoMVelocity);
}

/**
 * Predict next velocity.
 *
 * @param p_previousVelocityX
 * @param p_previousVelocityY
 * @param p_previousAngularVelocity
 * @param p_nextVelocityX
 * @param p_nextVelocityY
 * @param p_nextAngularVelocity
 * @param p_previousCoMVelocity
 * @param p_currentCoMVelocity
 */
double Model::velocityPrediction(const double &p_previousVelocityX,
                                 const double &p_previousVelocityY,
                                 const double &p_previousAngularVelocity,
                                 const double &p_nextVelocityX,
                                 const double &p_nextVelocityY,
                                 const double &p_nextAngularVelocity,
                                 const double &p_previousCoMVelocity,
                                 const double &p_currentCoMVelocity) {
    Eigen::VectorXd l_modelInput(5);
    l_modelInput << p_previousVelocityX,
                    p_nextVelocityX,
                    p_previousCoMVelocity,
                    p_currentCoMVelocity,
                    1;

    ROS_DEBUG_STREAM("Velocity Input: " << l_modelInput);

    return m_com_velocity * l_modelInput;
}

/**
 * Compute new CoM in world frame.
 *
 * @param p_predictedCoMDisplacementX
 * @param p_predictedCoMDisplacementY
 * @param p_predictedCoMDisplacementTheta
 * @param p_currentCoMVelocity
 * @param p_predictedCoMVelocity
 * @param p_currentWorldCoordinatesCoM
 * @param p_newWorldCoordinatesCoM
 */
void Model::computeNewCoM(const double &p_predictedCoMDisplacementX,
                          const double &p_predictedCoMDisplacementY,
                          const double &p_predictedCoMDisplacementTheta,
                          const double &p_predictedCoMVelocity,
                          const World3D &p_currentWorldCoordinatesCoM,
                          World3D &p_newWorldCoordinatesCoM) {
    // Get rotation matrix of the base w.r.t world
    Eigen::Quaterniond q;
    q.x() = p_currentWorldCoordinatesCoM.q.x();
    q.y() = p_currentWorldCoordinatesCoM.q.y();
    q.z() = p_currentWorldCoordinatesCoM.q.z();
    q.w() = p_currentWorldCoordinatesCoM.q.w();
    Eigen::Matrix3d RWorldBase = q.normalized().toRotationMatrix();

    // Transform CoM prediction from base to world frame
    Eigen::Vector3d l_displacementCoMFrame{p_predictedCoMDisplacementX, p_predictedCoMDisplacementY, 0.0};
    Eigen::Vector3d l_displacementMapFrame = RWorldBase * l_displacementCoMFrame;

    // Update CoM attributes
    p_newWorldCoordinatesCoM.a_v = p_predictedCoMVelocity;
    p_newWorldCoordinatesCoM.p_v = p_currentWorldCoordinatesCoM.a_v;
    p_newWorldCoordinatesCoM.x = p_currentWorldCoordinatesCoM.x + l_displacementMapFrame(0);
    p_newWorldCoordinatesCoM.y = p_currentWorldCoordinatesCoM.y + l_displacementMapFrame(1);

//     std::cout << "CoM predictions: " << p_predictedCoMDisplacementX << std::endl;
//     std::cout << "World predictions: " << l_displacementMapFrame(0) << std::endl;
//     std::cout << "Previous com: " << p_currentWorldCoordinatesCoM.x << std::endl;
//     std::cout << "Current com: " << p_newWorldCoordinatesCoM.x << std::endl;
//     std::cout << "\n" << std::endl;

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
    // Get rotation matrix of the base w.r.t world
    Eigen::Quaterniond q;
    q.x() = p_newWorldCoordinatesCoM.q.x();
    q.y() = p_newWorldCoordinatesCoM.q.y();
    q.z() = p_newWorldCoordinatesCoM.q.z();
    q.w() = p_newWorldCoordinatesCoM.q.w();
    Eigen::Matrix3d RWorldBase = q.normalized().toRotationMatrix();

    // Transform feet prediction from base to world frame
    Eigen::Vector3d l_feetPredictionBaseFrame{p_predictions[2], p_predictions[3], 0.0};
    Eigen::Vector3d l_feetDisplacementWorldFrame = RWorldBase * l_feetPredictionBaseFrame;

    // Add predictions to respective swinging feet
    if (p_currentFeetConfiguration.fr_rl_swinging) {
        p_newFeetConfiguration.frMap.x = p_currentFeetConfiguration.frMap.x + l_feetDisplacementWorldFrame(0);
        p_newFeetConfiguration.frMap.y = p_currentFeetConfiguration.frMap.y + l_feetDisplacementWorldFrame(1);
        p_newFeetConfiguration.rlMap.x = p_currentFeetConfiguration.rlMap.x + l_feetDisplacementWorldFrame(0);
        p_newFeetConfiguration.rlMap.y = p_currentFeetConfiguration.rlMap.y + l_feetDisplacementWorldFrame(1);

        p_newFeetConfiguration.flMap.x = p_currentFeetConfiguration.flMap.x;
        p_newFeetConfiguration.flMap.y = p_currentFeetConfiguration.flMap.y;
        p_newFeetConfiguration.rrMap.x = p_currentFeetConfiguration.rrMap.x;
        p_newFeetConfiguration.rrMap.y = p_currentFeetConfiguration.rrMap.y;
    }
    else {
        p_newFeetConfiguration.flMap.x = p_currentFeetConfiguration.flMap.x + l_feetDisplacementWorldFrame(0);
        p_newFeetConfiguration.flMap.y = p_currentFeetConfiguration.flMap.y + l_feetDisplacementWorldFrame(1);
        p_newFeetConfiguration.rrMap.x = p_currentFeetConfiguration.rrMap.x + l_feetDisplacementWorldFrame(0);
        p_newFeetConfiguration.rrMap.y = p_currentFeetConfiguration.rrMap.y + l_feetDisplacementWorldFrame(1);

        p_newFeetConfiguration.frMap.x = p_currentFeetConfiguration.frMap.x;
        p_newFeetConfiguration.frMap.y = p_currentFeetConfiguration.frMap.y;
        p_newFeetConfiguration.rlMap.x = p_currentFeetConfiguration.rlMap.x;
        p_newFeetConfiguration.rlMap.y = p_currentFeetConfiguration.rlMap.y;
    }

//     std::cout << "Feet predictions: " << p_predictions[2] << std::endl;
//     std::cout << "Feet world predictions: " << l_feetDisplacementWorldFrame(0) << std::endl;
//     std::cout << "FL world: " << p_currentFeetConfiguration.flMap.x << ", " << p_newFeetConfiguration.flMap.x << std::endl;
//     std::cout << "FR world: " << p_currentFeetConfiguration.frMap.x << ", " << p_newFeetConfiguration.frMap.x << std::endl;
//     std::cout << "\n" << std::endl;

    // Compute new feet configuration in CoM frame
    Eigen::Vector3d l_flCoMWorld{p_newFeetConfiguration.flMap.x - p_newWorldCoordinatesCoM.x, 
                                 p_newFeetConfiguration.flMap.y - p_newWorldCoordinatesCoM.y, 
                                 0.0};
//     std::cout << "Before FL: " << l_flCoMWorld << std::endl;
    Eigen::Vector3d l_flCoMBase = RWorldBase.transpose() * l_flCoMWorld;
    p_newFeetConfiguration.flCoM.x = l_flCoMBase(0);
    p_newFeetConfiguration.flCoM.y = l_flCoMBase(1);
//     std::cout << "After FL: " << l_flCoMBase << std::endl;

    Eigen::Vector3d l_frCoMWorld{p_newFeetConfiguration.frMap.x - p_newWorldCoordinatesCoM.x, 
                                 p_newFeetConfiguration.frMap.y - p_newWorldCoordinatesCoM.y, 
                                 0.0};
//     std::cout << "Before FR: " << l_frCoMWorld << std::endl;
    Eigen::Vector3d l_frCoMBase = RWorldBase.transpose() * l_frCoMWorld;
    p_newFeetConfiguration.frCoM.x = l_frCoMBase(0);
    p_newFeetConfiguration.frCoM.y = l_frCoMBase(1);
//     std::cout << "After FR: " << l_frCoMBase << std::endl;

    Eigen::Vector3d l_rlCoMWorld{p_newFeetConfiguration.rlMap.x - p_newWorldCoordinatesCoM.x, 
                                 p_newFeetConfiguration.rlMap.y - p_newWorldCoordinatesCoM.y, 
                                 0.0};
    Eigen::Vector3d l_rlCoMBase = RWorldBase.transpose() * l_rlCoMWorld;
    p_newFeetConfiguration.rlCoM.x = l_rlCoMBase(0);
    p_newFeetConfiguration.rlCoM.y = l_rlCoMBase(1);

    Eigen::Vector3d l_rrCoMWorld{p_newFeetConfiguration.rrMap.x - p_newWorldCoordinatesCoM.x, 
                                 p_newFeetConfiguration.rrMap.y - p_newWorldCoordinatesCoM.y, 
                                 0.0};
    Eigen::Vector3d l_rrCoMBase = RWorldBase.transpose() * l_rrCoMWorld;
    p_newFeetConfiguration.rrCoM.x = l_rrCoMBase(0);
    p_newFeetConfiguration.rrCoM.y = l_rrCoMBase(1);

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
 * @param p_plannedFootstep
 * @param p_previousVelocity
 * @param p_nextVelocity
 * @param p_action
 * @param p_currentWorldCoordinatesCoM
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 * @param p_newWorldCoordinatesCoM
 */
void Model::predictNextState(const uint p_plannedFootstep,
                             const double p_previousVelocity,
                             const double p_nextVelocity,
                             const Action &p_action,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    std::vector<double> l_predictions(6);
    motionPrediction(p_plannedFootstep,
                     p_action.x * p_previousVelocity,
                     p_action.y * p_previousVelocity,
                     p_action.theta * p_previousVelocity,
                     p_action.x * p_nextVelocity,
                     p_action.y * p_nextVelocity,
                     p_action.theta * p_nextVelocity,
                     p_currentWorldCoordinatesCoM.p_v,
                     p_currentWorldCoordinatesCoM.a_v,
                     p_currentFeetConfiguration,
                     l_predictions);

    ROS_DEBUG_STREAM("Prev Velocity: " << p_previousVelocity);
    ROS_DEBUG_STREAM("Next Velocity: " << p_nextVelocity);
    ROS_DEBUG_STREAM("Predictions: " << l_predictions[0] << ", "
                                    << l_predictions[1] << ", "
                                    << l_predictions[2] << ", "
                                    << l_predictions[3] << ", "
                                    << l_predictions[4] << ", "
                                    << l_predictions[5] <<"\n");

    computeNewCoM(l_predictions[0],
                  l_predictions[1],
                  l_predictions[4],
                  l_predictions[5],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    computeNewFeetConfiguration(p_newWorldCoordinatesCoM,
                                l_predictions,
                                p_currentFeetConfiguration,
                                p_newFeetConfiguration);

    //   // Publish predicted CoM and feet poses
    //   int j = 0;
    //   visualization_msgs::Marker l_footCommonMarker;
    //   l_footCommonMarker.header.stamp = ros::Time::now();
    //   l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    //   l_footCommonMarker.type = 2;
    //   l_footCommonMarker.action = 0;
    //   l_footCommonMarker.lifetime = ros::Duration(0.5);
    //   l_footCommonMarker.pose.orientation.x = p_currentWorldCoordinatesCoM.q.x();
    //   l_footCommonMarker.pose.orientation.y = p_currentWorldCoordinatesCoM.q.y();
    //   l_footCommonMarker.pose.orientation.z = p_currentWorldCoordinatesCoM.q.z();
    //   l_footCommonMarker.pose.orientation.w = p_currentWorldCoordinatesCoM.q.w();
    //   l_footCommonMarker.scale.x = 0.05;
    //   l_footCommonMarker.scale.y = 0.035;
    //   l_footCommonMarker.scale.z = 0.035;
    //   l_footCommonMarker.color.r = 0;
    //   l_footCommonMarker.color.g = 0;
    //   l_footCommonMarker.color.b = 1;
    //   l_footCommonMarker.color.a = 1;

    //   visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
    //   l_CoMMarker.id = j++;
    //   l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    //   l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
    //   l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
    //   l_CoMMarker.pose.position.z = 0.170;

    //   visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
    //   l_flFootMarker.id = j++;
    //   l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
    //   l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
    //   l_flFootMarker.pose.position.z = 0.170;

    //   visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
    //   l_frFootMarker.id = j++;
    //   l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
    //   l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
    //   l_frFootMarker.pose.position.z = 0.170;

    //   visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
    //   l_rlFootMarker.id = j++;
    //   l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
    //   l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
    //   l_rlFootMarker.pose.position.z = 0.170;

    //   visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
    //   l_rrFootMarker.id = j++;
    //   l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
    //   l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
    //   l_rrFootMarker.pose.position.z = 0.170;

    //   visualization_msgs::MarkerArray l_pathFeetConfiguration;
    //   l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
    //   l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
    //   l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
    //   l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
    //   l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

    //   m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
    //   ros::Duration(0.5).sleep();
}