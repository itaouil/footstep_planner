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
    m_fr_rl_com_x << 1.27608154e-01,  1.40627616e+09,  5.87919846e+12,
            9.35323238e-02, -9.17424075e+09, -5.87919846e+12,
            1.64172649e-01,  6.04940310e-01,  7.19354898e-02,
            -3.15462530e-01,  1.96177810e-02, -7.62230754e-02,
            2.44806409e-02, -2.69220147e-01, -0.19123681;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << 8.33200803e-03,  1.37896532e+08,  5.76502043e+11,
            3.08222622e-02, -8.99607075e+08, -5.76502043e+11,
            -1.22493770e-01,  9.45340525e-02, -2.29250910e-02,
            -2.58467495e-02,  1.09096943e-01, -1.95437263e-01,
            -6.64886460e-03,  2.13660796e-01, 0.09964802;

    m_fr_rl_com_theta.resize(15);
    m_fr_rl_com_theta << -4.82120047e-03, -1.15547983e+08, -4.83069789e+11,
            6.69462979e-03,  7.53809991e+08,  4.83069789e+11,
            5.79082515e-01, -5.28907365e-01, -3.57574810e-01,
            -5.57573666e-01,  4.69077374e-01,  5.97462879e-01,
            -5.62740196e-01,  4.99864754e-01, -0.0921434;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 1.15640461e-01,  2.35210600e+08,  5.00022616e+10,
            1.02041930e-01,  1.75486912e+08, -5.00022616e+10,
            -1.09037831e-02,  2.00220840e-01,  1.03882238e-01,
            -5.54988282e-01,  1.10161625e-01,  3.28755803e-01,
            1.04123121e-01,  1.19731731e-01, -0.09113394;

    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << 3.32751373e-03, -1.20269866e+09, -2.55675778e+11,
            -3.04236412e-02, -8.97314465e+08,  2.55675778e+11,
            5.59079275e-02,  1.23547763e-03, -6.34575188e-02,
            8.62759724e-02,  1.51131749e-01,  1.60955861e-01,
            -1.31967738e-01, -1.61060452e-01, -0.0223178;

    m_fl_rr_com_theta.resize(15);
    m_fl_rr_com_theta << 2.75797795e-03, -1.40457516e+09, -2.98591707e+11,
            -6.28590584e-03, -1.04793133e+09,  2.98591707e+11,
            1.93734869e-01, -3.42363261e-01, -5.74413538e-01,
            -4.42491665e-01,  5.73188007e-01,  4.22596335e-01,
            -2.81285569e-01,  3.93991828e-01, 0.15391401;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 2.41549573e-01, -1.79198331e+07,  2.84488712e+11,
            1.96172990e-01,  1.17793287e+08, -2.84488712e+11,
            -5.73344342e-01,  4.79932587e-01,  8.03244330e-01,
            -1.95762539e+00, -1.70765880e+00,  2.48332765e+00,
            -2.30944301e-01, -8.07740847e-01, -1.33612258;

    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << 5.49655237e-02, -9.38860465e+06,  1.49050051e+11,
            -4.17727213e-02,  6.17145590e+07, -1.49050051e+11,
            1.05509114e-01, -6.56532713e-01, -3.81981265e-01,
            -2.37142897e-01,  2.16529494e-01,  4.53854708e-01,
            -6.61787772e-02, -4.33368561e-02, 0.10301452;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << 2.93273694e-01, -1.40483312e+09,  2.26751219e+12,
            1.96469307e-01, -2.00413566e+09, -2.26751219e+12,
            1.69436187e+00,  1.61206178e+00, -8.83887827e-01,
            -9.17198107e-01,  1.53424576e-01,  8.51080611e-01,
            -2.78526413e+00, -1.85729951e+00, -1.60397396;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << -1.04362134e-02, -3.29590006e+08,  5.31984436e+11,
            1.62805915e-02, -4.70193272e+08, -5.31984436e+11,
            5.82553089e-01,  2.46194430e-01, -7.35857189e-02,
            3.66420709e-02,  4.50676829e-02, -7.99261633e-01,
            -5.19630194e-01,  1.62783118e-01, -0.15751274;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << 2.85334868e-01, -1.25349088e+09,  2.02323379e+12,
            2.01140404e-01, -1.78823073e+09, -2.02323379e+12,
            1.50726801e+00,  1.50604673e+00,  3.19673717e-02,
            -9.80773211e-01, -7.45203100e-01,  9.07179311e-01,
            -2.57644302e+00, -1.78252668e+00, -1.9632776;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y <<  -1.04362134e-02, -3.29590006e+08,  5.31984436e+11,
            1.62805915e-02, -4.70193272e+08, -5.31984436e+11,
            5.82553089e-01,  2.46194430e-01, -7.35857189e-02,
            3.66420709e-02,  4.50676829e-02, -7.99261633e-01,
            -5.19630194e-01,  1.62783118e-01, -0.15751274;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x <<  2.85334868e-01, -1.25349088e+09,  2.02323379e+12,
            2.01140404e-01, -1.78823073e+09, -2.02323379e+12,
            1.50726801e+00,  1.50604673e+00,  3.19673717e-02,
            -9.80773211e-01, -7.45203100e-01,  9.07179311e-01,
            -2.57644302e+00, -1.78252668e+00, -1.9632776;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y <<  -1.04362134e-02, -3.29590006e+08,  5.31984436e+11,
            1.62805915e-02, -4.70193272e+08, -5.31984436e+11,
            5.82553089e-01,  2.46194430e-01, -7.35857189e-02,
            3.66420709e-02,  4.50676829e-02, -7.99261633e-01,
            -5.19630194e-01,  1.62783118e-01, -0.15751274;
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

    ROS_INFO_STREAM("Model input: " << l_modelInput);

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

        p_predictions[10] = m_fr_rl_com_theta * l_modelInput;
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

        p_predictions[10] = m_fl_rr_com_theta * l_modelInput;
    }
}

/**
  * Compute new CoM in world coordinates.
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

    // Get yaw rotation in quaternion form
    tf2::Quaternion l_velocityCommandQuaternion;
    l_velocityCommandQuaternion.setRPY(0, 0, p_predictedCoMDisplacementTheta);

    // Apply rotation command rotation to CoM quaternion
    tf2::Quaternion l_newCoMRotation = p_currentWorldCoordinatesCoM.q * l_velocityCommandQuaternion;
    l_newCoMRotation.normalize();

    // Set new CoM rotation
    p_newWorldCoordinatesCoM.q = l_newCoMRotation;
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
 * @param p_previousVelocity
 * @param p_nextVelocity
 * @param p_action
 * @param p_currentWorldCoordinatesCoM
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 * @param p_newWorldCoordinatesCoM
 */
void Model::predictNextState(double p_previousVelocity,
                             double p_nextVelocity,
                             const Action &p_action,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    // Predict feet and CoM displacements
    std::vector<double> l_predictions(11);
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

    // Compute new CoM
    computeNewCoM(l_predictions[0],
                  l_predictions[1],
                  l_predictions[10],
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
    l_footCommonMarker.scale.x = 0.04;
    l_footCommonMarker.scale.y = 0.04;
    l_footCommonMarker.scale.z = 0.04;
    l_footCommonMarker.color.r = 0;
    l_footCommonMarker.color.g = 1;
    l_footCommonMarker.color.b = 0;
    l_footCommonMarker.color.a = 0.7;

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