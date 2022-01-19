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
    m_fr_rl_com_x << 4.63028816e-03, 1.80411242e-16, -1.60461922e-16,
            1.19860834e-01, 4.16333634e-17, 1.52655666e-16,
            4.37501754e-02, 9.30692625e-01, 1.16193353e-01,
            7.53660558e-02, 1.89459571e-01, -1.99287383e-02,
            6.64589061e-02, -8.97049564e-02, -0.13682552;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << -5.40070997e-03, -4.51028104e-17, -1.38777878e-17,
            -3.79320115e-02, -1.11022302e-16, 5.55111512e-17,
            1.37126576e-01, 5.10276036e-02, -1.75545696e-01,
            3.44530615e-01, -3.46190727e-02, 1.56475485e-01,
            1.48954386e-01, 7.84449105e-02, 0.06921374;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 2.61358914e-03, 1.66533454e-16, -9.71445147e-17,
            1.14483383e-01, -1.80411242e-16, 0.00000000e+00,
            2.76681552e-01, 2.83321991e-01, 7.63934395e-02,
            -5.43539710e-01, 1.48353532e-01, -6.35000878e-02,
            6.26671857e-02, -3.37465663e-01, -0.22011284;
    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << 9.58097480e-03, -1.31838984e-16, 4.16333634e-17,
            3.28087221e-02, -1.94289029e-16, 5.55111512e-17,
            4.04192458e-01, 8.61055818e-01, -1.61089326e-01,
            2.73453611e-01, -1.07303182e-01, -3.16108148e-02,
            -2.22274028e-01, -3.44686610e-01, -0.29100435;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 2.12405952e-04, -2.77555756e-17, -5.55111512e-17,
            2.14983569e-01, -8.32667268e-17, -5.55111512e-17,
            -2.79901546e-01, 3.67153335e-01, 8.90326423e-02,
            -2.09595211e-01, 8.14745963e-02, -7.94048845e-02,
            -1.69858356e-02, -3.55939494e-01, -0.0895945;
    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << 3.84205183e-03, -7.28583860e-17, 2.77555756e-17,
            2.47058922e-02, 1.04083409e-17, 0.00000000e+00,
            2.97413228e-01, 4.07457989e-02, -8.76821489e-02,
            1.65017261e-01, -6.73048322e-02, -3.82785402e-02,
            -1.68168627e-01, -2.70623064e-01, -0.12480851;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << 3.96984284e-03, -1.66533454e-16, -2.08166817e-16,
            2.17679742e-01, 5.55111512e-17, 5.55111512e-17,
            -1.61086781e-02, 3.85241242e-01, -4.40238160e-01,
            -7.83306293e-02, 1.29333152e-01, 5.71382937e-02,
            8.28608263e-02, 4.88050334e-02, 0.07273362;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << -3.60149148e-04, 9.54097912e-18, -8.32667268e-17,
            -2.80540299e-02, -1.38777878e-17, 2.77555756e-17,
            5.82257984e-02, 7.58113324e-03, -9.15410748e-02,
            1.41373755e-01, -4.93862376e-02, -3.88317439e-01,
            1.10109191e-01, 5.13040273e-03, 0.10912904;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << 8.17039826e-03, -1.94289029e-16, -2.22044605e-16,
            2.13054670e-01, 4.51028104e-17, -2.77555756e-17,
            -4.60401692e-02, 4.48879325e-01, 9.54721107e-02,
            -9.97748409e-02, -4.48068086e-01, 9.34014613e-02,
            9.32634567e-02, -5.19054120e-02, -0.22238487;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y << -3.60149148e-04, 9.54097912e-18, -8.32667268e-17,
            -2.80540299e-02, -1.38777878e-17, 2.77555756e-17,
            5.82257984e-02, 7.58113324e-03, -9.15410748e-02,
            1.41373755e-01, -4.93862376e-02, -3.88317439e-01,
            1.10109191e-01, 5.13040273e-03, 0.10912904;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << -2.39600967e-03, 0.00000000e+00, -8.32667268e-17,
            2.13808033e-01, -8.32667268e-17, -5.55111512e-17,
            2.78825266e-01, 3.78073944e-01, 9.98187558e-02,
            -2.16376607e-01, 9.44990214e-02, -8.58580716e-02,
            -5.95677198e-01, -3.84988182e-01, -0.36804565;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y << 5.24114429e-03, -5.55111512e-17, 0.00000000e+00,
            2.34891875e-02, -1.11022302e-16, -5.55111512e-17,
            2.60329861e-01, 5.28907677e-01, -1.00852317e-01,
            1.43110999e-01, -6.79526783e-02, -4.05586383e-02,
            -1.34780052e-01, -7.57837470e-01, -0.27440417;
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

    ROS_INFO_STREAM("Prev Velocity: " << p_previousVelocity);
    ROS_INFO_STREAM("Next Velocity: " << p_nextVelocity);

    ROS_INFO_STREAM("Predictions: " << l_predictions[0] << ", "
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
//    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

//    // Publish predicted CoM and feet poses
//    int j = 0;
//    visualization_msgs::Marker l_footCommonMarker;
//    l_footCommonMarker.header.stamp = ros::Time::now();
//    l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//    l_footCommonMarker.type = 2;
//    l_footCommonMarker.action = 0;
//    l_footCommonMarker.lifetime = ros::Duration(0.5);
//    l_footCommonMarker.pose.orientation.x = 0;
//    l_footCommonMarker.pose.orientation.y = 0;
//    l_footCommonMarker.pose.orientation.z = 0;
//    l_footCommonMarker.pose.orientation.w = 1;
//    l_footCommonMarker.scale.x = 0.035;
//    l_footCommonMarker.scale.y = 0.035;
//    l_footCommonMarker.scale.z = 0.035;
//    l_footCommonMarker.color.r = 0;
//    l_footCommonMarker.color.g = 1;
//    l_footCommonMarker.color.b = 0;
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
//    l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
//    l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
//    l_flFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//    l_frFootMarker.id = j++;
//    l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
//    l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
//    l_frFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//    l_rlFootMarker.id = j++;
//    l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
//    l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
//    l_rlFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//    l_rrFootMarker.id = j++;
//    l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
//    l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
//    l_rrFootMarker.pose.position.z = 0;
//
//    visualization_msgs::MarkerArray l_pathFeetConfiguration;
//    l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
//    l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);
//
//    m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
//    ros::Duration(1).sleep();
}