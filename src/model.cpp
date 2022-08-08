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
    m_fr_rl_com_x.resize(18);
    m_fr_rl_com_x << 1.75502392e-01, -2.35150644e-14,  1.29757316e-14,
        7.78771461e-03,  1.32624738e-01, -5.67902198e+00,
        1.95269333e-01, -7.83170795e-01,  6.00219195e-01,
        1.07520234e-01,  3.97976044e-01,  5.20891289e-01,
        1.48392450e-01, -8.32157994e-01,  6.49324414e-02,
        2.04032435e-03, -2.18611264e-01, -0.0314833 ;

    m_fr_rl_com_y.resize(18);
    m_fr_rl_com_y << 7.22494953e-03, -3.27619715e-14,  3.97933097e-14,
        -3.24406624e-03, -1.00300293e-01,  3.15076726e-02,
        -6.51028842e-03,  1.76932974e-03,  2.52860177e+00,
        -2.76484955e-02, -5.78783487e-02,  7.28414999e-04,
        7.44661728e-02,  3.68076465e-02,  8.88853386e-02,
        -5.18452207e-02,  2.72408742e-02, 0.02423595;

    m_fr_rl_com_theta.resize(18);
    m_fr_rl_com_theta << 6.14386359e-03,  9.44966897e-15, -1.14075416e-14,
        6.26288344e-02,  2.43891667e-02,  8.10883768e-02,
        1.43458211e-02, -2.98265665e-02, -7.34657227e-01,
        6.12161351e-03, -3.35216136e-02, -6.22920687e-02,
        -5.77383136e-02, -2.07669593e-02,  9.82403471e-02,
        5.11479335e-03,  3.62274292e-02, -0.04461337;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(18);
    m_fl_rr_com_x << 2.45997569e-01, -1.04659738e-12,  5.79258863e-14,
        2.62299073e-01,  9.51598062e-01,  7.61253049e+00,
        -4.69443326e-02, -5.54930950e-01,  1.63183970e+00,
        5.03807949e-01, -2.14029290e-01,  2.04155767e-01,
        4.19125425e-01, -9.23187186e-01, -5.96078730e-01,
        -4.39463923e-03, -2.36028427e-01, 0.03125234;

    m_fl_rr_com_y.resize(18);
    m_fl_rr_com_y << 1.51609682e-02,  2.40422428e-13, -1.39679934e-14,
         2.39453358e-02, -2.27468766e-03, -1.82294736e+00,
        -4.15679749e-02, -1.41957997e-01, -2.24011318e-01,
        -2.99572356e-02, -1.13310450e-01,  6.91970077e-02,
        -7.08503375e-03, -2.15001404e-02,  2.70973357e-01,
        -3.64473894e-02, -3.14972681e-02, 0.011811;

    m_fl_rr_com_theta.resize(18);
    m_fl_rr_com_theta << 5.59168392e-04, -3.72404944e-13,  2.12217396e-14,
         7.59861784e-03,  2.07320842e-02,  2.81971926e+00,
        -6.63353496e-02, -1.74550784e-02,  3.35600640e-01,
         5.85396997e-02, -2.69102804e-02,  2.91443282e-03,
         2.76603503e-02, -2.73609078e-03, -5.03641281e-03,
         1.36821794e-02, -2.68983727e-02, 0.03397652;

    // FL models coefficients
    m_fl_swinging_x.resize(18);
    m_fl_swinging_x << 4.97343902e-01, -1.38777878e-17,  2.28983499e-15,
        -1.24073681e-01,  2.90118785e-01, -1.05602205e-02,
        -8.97296529e-03, -4.72003835e-02,  3.47662885e-02,
        -5.45472422e-01,  1.70020839e-01, -2.04010614e-04,
        -1.02523809e+00,  3.98169676e-02, -2.70022079e-01,
        -2.03553872e-01,  4.79959324e-01, 0.02518138;

    m_fl_swinging_y.resize(18);
    m_fl_swinging_y << 2.09617936e-02,  5.20417043e-17,  8.15320034e-17,
         3.20989860e-03,  2.17560574e-01, -4.11348209e-03,
        -8.82133705e-05, -5.48490775e-03, -1.31482667e-02,
         5.13376350e-02, -2.20422339e-01, -1.83947834e-01,
        -1.88093587e-01, -1.29180915e-01, -2.58520793e-01,
         4.12889659e-02, -1.03953257e-01, 0.03465639;

    // FR models coefficients
    m_fr_swinging_x.resize(18);
    m_fr_swinging_x << 4.97343902e-01, -1.38777878e-17,  2.28983499e-15,
        -1.24073681e-01,  2.90118785e-01, -1.05602205e-02,
        -8.97296529e-03, -4.72003835e-02,  3.47662885e-02,
        -5.45472422e-01,  1.70020839e-01, -2.04010614e-04,
        -1.02523809e+00,  3.98169676e-02, -2.70022079e-01,
        -2.03553872e-01,  4.79959324e-01, 0.02518138;
    m_fr_swinging_y.resize(18);
    m_fr_swinging_y << 2.09617936e-02,  5.20417043e-17,  8.15320034e-17,
         3.20989860e-03,  2.17560574e-01, -4.11348209e-03,
        -8.82133705e-05, -5.48490775e-03, -1.31482667e-02,
         5.13376350e-02, -2.20422339e-01, -1.83947834e-01,
        -1.88093587e-01, -1.29180915e-01, -2.58520793e-01,
         4.12889659e-02, -1.03953257e-01, 0.03465639;

    // RL models coefficients
    m_rl_swinging_x.resize(18);
    m_rl_swinging_x << 4.47406388e-01,  1.42247325e-16,  1.43635104e-15,
        -4.80618872e-02,  8.88366985e-02, -1.15191842e-02,
        -3.38108841e-03, -5.73246701e-03, -1.71874807e-02,
         1.26290203e-01,  4.77396746e-01, -1.50271520e-01,
        -5.43547657e-01,  3.52210216e-01, -5.00275874e-01,
        -6.28659351e-01,  3.62600000e-01, -0.09556052;
    m_rl_swinging_y.resize(18);
    m_rl_swinging_y << 4.17913765e-02,  1.24900090e-16, -1.90819582e-17,
        -2.09558552e-02,  1.96417469e-01, -1.98383483e-02,
        -2.25075683e-03, -5.24765258e-03,  2.43074364e-02,
         6.03616940e-02,  4.18346142e-02, -2.54678417e-01,
        -4.11239224e-01, -6.81949440e-02, -3.59850741e-01,
         4.72415978e-02, -5.74022774e-01, -0.03864264;

    // RR models coefficients
    m_rr_swinging_x.resize(18);
    m_rr_swinging_x << 2.92987596e-01, -5.55111512e-17, -3.22658567e-16,
         6.40695830e-02, -1.17860937e-01, -3.77764563e-03,
         5.59686141e-03, -2.66608954e-02, -5.16267313e-02,
        -2.12419992e-01,  2.54321125e-01,  4.23831014e-02,
        -2.50443599e-01, -7.35417359e-01, -1.11033689e-03,
         5.60298277e-01,  7.21198687e-01, 0.02166979;
    m_rr_swinging_y.resize(18);
    m_rr_swinging_y << 5.29045978e-02,  1.38777878e-17,  6.52256027e-16,
        -6.88764165e-02,  2.88670783e-01,  2.19864507e-02,
        -4.23788580e-03,  1.15637127e-02,  6.05532349e-02,
         3.72484728e-01, -2.72046224e-01, -1.47500155e-02,
         1.95762845e-02, -3.40988320e-03, -6.17378591e-01,
        -8.39475532e-02, -3.14366390e-01, -0.03497082;
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
    Eigen::VectorXd l_modelInput(18);
    l_modelInput << p_nextVelocityX,
            p_nextVelocityY,
            p_nextAngularVelocity,
            p_odomVelocityState.linear.x,
            p_odomVelocityState.linear.y,
            p_odomVelocityState.linear.z,
            p_odomVelocityState.angular.x,
            p_odomVelocityState.angular.y,
            p_odomVelocityState.angular.z,
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

    ROS_DEBUG_STREAM("Predictions: " << l_predictions[0] << ", "
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
    ros::Duration(2).sleep();
}