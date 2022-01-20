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
    m_fr_rl_com_x << 5.95531571e-03, -1.38777878e-17, -1.21430643e-16,
            1.17011661e-01,  1.66533454e-16, -1.45716772e-16,
            1.38878316e-01,  8.33823586e-01,  1.06381719e-01,
            5.27861313e-02,  2.30631618e-01, -1.06176361e-02,
            3.88892218e-03, -1.16743654e-01, -0.15508161;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << -5.47178969e-03, -9.36750677e-17,  6.93889390e-17,
            -3.59367108e-02,  2.77555756e-17,  0.00000000e+00,
            -1.27577139e-03, -1.60864465e-01, -1.36768427e-01,
            2.94581950e-01, -5.21285594e-02,  1.61102193e-01,
            2.77147734e-01,  2.01539600e-01, 0.16712354;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 6.54563092e-03,  1.52655666e-16,  3.46944695e-18,
            1.15896646e-01,  9.02056208e-17,  2.77555756e-17,
            1.38169944e-01,  1.72517079e-02,  2.12918012e-02,
            -6.22313894e-01,  1.15911155e-01,  5.61062676e-02,
            2.25539252e-01,  7.80928988e-03, -0.07322172;
    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << 1.22518324e-02,  2.94902991e-17,  0.00000000e+00,
            3.47193195e-02,  0.00000000e+00,  0.00000000e+00,
            1.39640048e-01,  3.72102951e-01, -1.33375853e-01,
            -6.58828109e-02, -2.21245720e-01,  1.99051695e-01,
            4.12489598e-02,  1.78561346e-01, -0.12347908;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 3.75173243e-03,  2.77555756e-16,  0.00000000e+00,
            2.16249200e-01,  1.38777878e-17, -5.55111512e-17,
            -4.37275992e-01,  8.66824578e-02,  2.01762314e-02,
            -3.26681332e-01,  7.61717521e-02,  2.30484606e-02,
            1.43994130e-01, -2.10093794e-02, 0.06845021;
    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << 6.82868733e-03,  1.73472348e-18,  2.77555756e-17,
            2.61125416e-02,  4.16333634e-17, -2.77555756e-17,
            1.10421322e-01, -3.11962935e-01, -7.38121839e-02,
            -5.49774669e-02, -1.48292572e-01,  1.31865459e-01,
            2.14841932e-02,  1.03544737e-01, -0.00018941;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << 4.09260532e-03,  2.77555756e-17, -1.52655666e-16,
            2.16630928e-01, -1.38777878e-17, -5.55111512e-17,
            4.27166908e-02,  3.58960295e-01, -4.25192278e-01,
            -7.02370396e-02,  1.38133361e-01,  3.85080003e-02,
            4.89690367e-02, -4.43623583e-02, 0.04269201;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << -1.93880257e-03, -2.42861287e-17,  5.55111512e-17,
            -2.59402081e-02,  0.00000000e+00,  1.11022302e-16,
            -2.81232772e-02, -1.64594499e-01, -1.00907205e-01,
            1.62076315e-01, -2.95123514e-02, -4.35536985e-01,
            2.05423942e-01,  1.56383934e-01, 0.22594642;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << 7.01047466e-03, -5.55111512e-17, -2.77555756e-17,
            2.13310955e-01, -2.77555756e-17,  0.00000000e+00,
            -1.81846558e-02,  3.67072642e-01,  1.01207529e-01,
            -6.77516941e-02, -4.25042757e-01,  4.35565456e-02,
            9.18248663e-02, -3.10746502e-02, -0.19367975;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y << -1.93880257e-03, -2.42861287e-17,  5.55111512e-17,
            -2.59402081e-02,  0.00000000e+00,  1.11022302e-16,
            -2.81232772e-02, -1.64594499e-01, -1.00907205e-01,
            1.62076315e-01, -2.95123514e-02, -4.35536985e-01,
            2.05423942e-01,  1.56383934e-01, 0.22594642;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << 3.84661705e-03,  0.00000000e+00, -5.55111512e-17,
            2.14041386e-01,  5.55111512e-17,  1.66533454e-16,
            1.37347913e-01,  1.22308010e-01, -6.28149193e-03,
            -3.05815018e-01,  1.13662666e-01,  2.97044394e-02,
            -4.40460998e-01, -5.68849983e-02, -0.20351494;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y << 6.11004943e-03, -4.51028104e-17,  5.55111512e-17,
            2.50236691e-02,  8.32667268e-17,  1.11022302e-16,
            6.09178223e-02,  1.63966224e-01, -2.88520490e-02,
            -1.85752265e-01, -1.95492323e-01,  1.44730707e-01,
            4.23508864e-02, -4.10894540e-01, -0.19817056;
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