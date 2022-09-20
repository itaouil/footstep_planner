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
    m_fr_rl_com_x << 1.45274660e-02,  1.11022302e-16,  5.55111512e-17,
            1.21480029e-01,  0.00000000e+00, -2.22044605e-16,
            4.59183036e-01, -1.08546195e+00, -2.72558095e-01,
            5.21936188e-01, -3.38240962e-01,  1.65537374e-01,
            -1.89407942e-02,  8.96852488e-01, 0.16325123;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << 3.20974179e-03,  1.19695920e-16, -2.42861287e-17,
            -5.61711439e-03,  1.11022302e-16,  5.55111512e-17,
            2.21675430e-02,  9.67680434e-02, -1.22439462e-01,
            -3.63380029e-01,  2.00918606e-01, -4.12373699e-01,
            6.38717439e-02,  1.09094404e-01, 0.09900231;

    m_fr_rl_com_theta.resize(15);
    m_fr_rl_com_theta << -1.61920617e-03, -2.42861287e-17, -3.12250226e-17,
            2.26519780e-03,  6.93889390e-18,  1.38777878e-17,
            -6.60284389e-02,  1.05302352e-01, -2.36640580e-02,
            -2.15212000e-02, -4.06046357e-02,  2.23379477e-02,
            3.11566249e-02, -8.97357333e-02, -0.0110588;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 4.52958116e-02,  5.55111512e-17, -5.55111512e-17,
            1.17559859e-01, -2.49800181e-16,  2.77555756e-17,
            -3.76058725e-01,  3.65030691e-02,  5.07825311e-01,
            9.24184008e-01, -3.14873512e-01, -5.99854750e-01,
            -4.17385907e-01, -8.97012315e-01, -0.14768165;

    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << -4.09720989e-03, -4.68375339e-17, -6.96057795e-17,
            -2.65430680e-04,  0.00000000e+00, -1.11022302e-16,
            -9.78155604e-02, -6.39772941e-01,  5.51243049e-02,
            2.10072324e-01, -5.94327029e-02,  2.27566328e-01,
            4.61211123e-02, -2.35416061e-01, 0.05630789;

    m_fl_rr_com_theta.resize(15);
    m_fl_rr_com_theta << -2.06496524e-03,  1.73472348e-18, -4.33680869e-18,
            -5.77681429e-03, -1.73472348e-17, -2.77555756e-17,
            -7.33784330e-03, -1.05762075e-01,  3.86233177e-02,
            7.52059702e-02, -1.12792307e-02, -2.50944653e-02,
            4.56671218e-02,  1.07287847e-01, 0.04288928;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 6.85519221e-02, -1.11022302e-16, -3.33066907e-16,
            2.01715553e-01, -2.49800181e-16,  3.60822483e-16,
            -1.41099410e+00, -3.68749740e-02,  6.51826194e-01,
            1.42784351e+00, -4.64835955e-01, -5.19789116e-01,
            -7.89292004e-01, -1.46466502e+00, -0.07434123;

    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << -9.35006898e-03, -7.45931095e-17, -1.00613962e-16,
            4.71894356e-04, -2.22044605e-16, -1.11022302e-16,
            -1.91268911e-01, -1.70553196e+00,  8.97027490e-02,
            5.33735407e-01, -1.53124653e-01,  5.13288013e-01,
            1.71236294e-02, -5.86363472e-01, 0.13553003;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << 2.05675074e-02, -1.11022302e-16, -1.11022302e-16,
            2.19132907e-01, -1.11022302e-16,  1.11022302e-16,
            4.68616241e-01, -1.47294861e+00, -1.32181760e+00,
            8.81263120e-01, -7.38832561e-01,  1.72107747e-01,
            -4.90660517e-02,  1.05655314e+00, 0.42105654;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << 8.16716731e-03, -9.36750677e-17,  1.31838984e-16,
            -9.47637391e-03,  0.00000000e+00, -1.66533454e-16,
            9.70951400e-02,  3.76339322e-01, -1.20083170e-01,
            -7.77032352e-01,  2.26838811e-01, -1.45441731e+00,
            1.00942212e-02,  3.14341060e-01, 0.13845567;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << 2.05539721e-02, -1.11022302e-16,  1.11022302e-16,
            2.19522760e-01,  0.00000000e+00,  4.44089210e-16,
            4.68019098e-01, -1.48467052e+00, -3.52324776e-01,
            9.16546126e-01, -1.71138679e+00,  2.04332147e-01,
            -5.79293634e-02,  1.01039420e+00, -0.07144999;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y <<  8.16716731e-03, -9.36750677e-17,  1.31838984e-16,
            -9.47637391e-03,  0.00000000e+00, -1.66533454e-16,
            9.70951400e-02,  3.76339322e-01, -1.20083170e-01,
            -7.77032352e-01,  2.26838811e-01, -1.45441731e+00,
            1.00942212e-02,  3.14341060e-01, 0.13845567;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << 6.95139108e-02, -1.11022302e-16,  0.00000000e+00,
            2.02025576e-01, -3.05311332e-16, -3.33066907e-16,
            -4.76977358e-01, -6.90331502e-02,  6.54003083e-01,
            1.47679972e+00, -4.75345412e-01, -5.30164930e-01,
            -1.75380690e+00, -1.52964208e+00, -0.55095271;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y <<  -1.41321835e-02,  4.51028104e-17, -6.24500451e-17,
            -2.83917303e-03, -1.11022302e-16, -1.11022302e-16,
            -1.27478279e-01, -1.08764974e+00,  6.10081889e-02,
            5.25566231e-01, -5.53263506e-02,  5.00222634e-01,
            1.41043188e-02, -1.07131716e+00, 0.01381127;
}

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of continuous
 * velocity commands.
 */
// void Model::setModelsCoefficients() {
//     // CoM models coefficients when FR/RL are swinging
//     m_fr_rl_com_x.resize(17);
//     m_fr_rl_com_x << 1.23106222e-02, -2.77555756e-17,  9.71445147e-17,
//             9.62412990e-02,  1.11022302e-16,  6.93889390e-17,
//             6.66182234e-02, -1.38599575e-01,  4.08089918e-01,
//             -9.20182683e-01, -2.13648607e-01,  1.76496781e-01,
//             -2.28287220e-01,  3.42973888e-02, -1.17662874e-01,
//             7.44589959e-01, 0.09641896;

//     m_fr_rl_com_y.resize(17);
//     m_fr_rl_com_y << 3.43586705e-03,  1.12757026e-16, -1.73472348e-16,
//             -5.65124311e-04, -5.55111512e-17,  4.16333634e-17,
//             -3.06995643e-02, -1.17611372e-04,  6.87480609e-02,
//             9.66905695e-02, -1.47238863e-01, -3.58745927e-01,
//             1.52989544e-01, -4.12726497e-01,  1.25216324e-01,
//             1.51312496e-01, 0.10313211;

//     m_fr_rl_com_theta.resize(17);
//     m_fr_rl_com_theta << -1.65991214e-03,  6.07153217e-18, -3.03576608e-17,
//             2.50413598e-03, -3.16587034e-17,  1.38777878e-17,
//             -1.02505412e-02, -1.39009478e-02, -4.54453172e-02,
//             1.22635740e-01, -3.20047898e-02, -5.27735980e-02,
//             -5.52945568e-02,  7.83282969e-03,  5.49739523e-02,
//             -8.26919501e-02, -0.01516997;

//     // CoM models coefficients when FL/RR are swinging
//     m_fl_rr_com_x.resize(17);
//     m_fl_rr_com_x << 2.66751743e-02,  1.38777878e-16,  1.42247325e-16,
//             9.26221151e-02, -5.03069808e-17,  0.00000000e+00,
//             1.56269092e-01,  1.43314323e-01, -1.81078356e-01,
//             9.09167367e-02,  5.77755570e-02,  8.09996484e-01,
//             -3.18292769e-01, -4.74754053e-01, -2.64627878e-01,
//             -4.62764027e-01, -0.02635813;

//     m_fl_rr_com_y.resize(17);
//     m_fl_rr_com_y << -7.63795257e-03,  9.88792381e-17,  4.48859699e-17,
//             -5.20413775e-03,  8.32667268e-17,  8.32667268e-17,
//             2.65923829e-02,  3.50477475e-02, -6.55375587e-02,
//             -6.16992332e-01, -1.45162011e-02,  1.87841254e-01,
//             -5.79500971e-02,  2.42237013e-01,  7.56547034e-02,
//             -1.41740382e-01, 0.07838211;

//     m_fl_rr_com_theta.resize(17);
//     m_fl_rr_com_theta << -1.90015251e-03,  3.90312782e-17, -2.77555756e-17,
//             -5.77162931e-03,  1.38777878e-17,  3.46944695e-18,
//             -1.31021122e-03,  1.61281065e-03, -9.45502857e-03,
//             -1.02454999e-01,  4.40775613e-02,  7.49503730e-02,
//             -1.10810000e-02, -2.74835149e-02,  4.55750462e-02,
//             1.09063793e-01;

//     // FL models coefficients
//     m_fl_swinging_x.resize(17);
//     m_fl_swinging_x << 3.90717503e-02, -1.11022302e-16,  4.71844785e-16,
//             1.61295656e-01, -2.77555756e-16,  0.00000000e+00,
//             2.46253526e-01,  2.42325365e-01, -1.10629505e+00,
//             6.96867487e-02, -4.77819932e-02,  1.23889122e+00,
//             -4.68189930e-01, -3.29380154e-01, -5.41779005e-01,
//             -7.46933331e-01, 0.11899331;

//     m_fl_swinging_y.resize(17);
//     m_fl_swinging_y << -1.66694828e-02,  2.74086309e-16, -5.37764278e-17,
//             -8.98773404e-03,  2.22044605e-16,  0.00000000e+00,
//             5.60185462e-02,  5.97089205e-02, -1.21075373e-01,
//             -1.67501351e+00, -6.50610486e-02,  4.95510355e-01,
//             -1.51957082e-01,  5.49379440e-01,  7.34727364e-02,
//             -4.18357413e-01, 0.18013068;

//     // FR models coefficients
//     m_fr_swinging_x.resize(17);
//     m_fr_swinging_x << 1.98746252e-02, -5.55111512e-17,  3.26128013e-16,
//             1.74946698e-01, -3.60822483e-16, -2.22044605e-16,
//             1.28340592e-01, -2.10317751e-01,  3.86634677e-01,
//             -1.17476069e+00, -1.24326759e+00,  4.07553980e-01,
//             -4.86276652e-01, -6.64249759e-02, -2.91857757e-01,
//             7.30073073e-01, 0.3150823;
//     m_fr_swinging_y.resize(17);
//     m_fr_swinging_y << 8.39730913e-03,  4.83987850e-16,  2.86229374e-17,
//             3.05259911e-03,  5.55111512e-17, -1.11022302e-16,
//             -7.75461461e-02, -1.03412815e-02,  1.94404265e-01,
//             3.72094897e-01, -1.76476538e-01, -8.01011291e-01,
//             1.04251361e-01, -1.46745788e+00,  1.94111021e-01,
//             4.33070004e-01, 0.15823644;

//     // RL models coefficients
//     m_rl_swinging_x.resize(17);
//     m_rl_swinging_x << 1.98829811e-02, -5.55111512e-17, -1.24900090e-16,
//             1.74772141e-01,  1.38777878e-16,  3.33066907e-16,
//             1.31325617e-01, -2.10674328e-01,  3.82382374e-01,
//             -1.18533655e+00, -2.71575963e-01,  4.42101493e-01,
//             -1.45394852e+00, -3.47238295e-02, -3.07516821e-01,
//             6.78927974e-01, -0.1783594;
//     m_rl_swinging_y.resize(17);
//     m_rl_swinging_y <<  8.39730913e-03,  4.83987850e-16,  2.86229374e-17,
//             3.05259911e-03,  5.55111512e-17, -1.11022302e-16,
//             -7.75461461e-02, -1.03412815e-02,  1.94404265e-01,
//             3.72094897e-01, -1.76476538e-01, -8.01011291e-01,
//             1.04251361e-01, -1.46745788e+00,  1.94111021e-01,
//             4.33070004e-01, 0.15823644;

//     // RR models coefficients
//     m_rr_swinging_x.resize(17);
//     m_rr_swinging_x << 3.91820029e-02,  5.55111512e-17,  3.46944695e-17,
//             1.61264720e-01, -1.94289029e-16,  1.11022302e-16,
//             2.52839390e-01,  2.38320301e-01, -1.62148085e-01,
//             2.69276285e-02, -7.01669708e-02,  1.28938046e+00,
//             -4.79580039e-01, -3.31082856e-01, -1.50437170e+00,
//             -8.14391530e-01, -0.35375618;
//     m_rr_swinging_y.resize(17);
//     m_rr_swinging_y <<  -2.00856830e-02, -6.93889390e-18,  6.67868538e-17,
//             -1.11963008e-02,  1.11022302e-16,  1.66533454e-16,
//             4.39994481e-02,  6.05916021e-02, -7.42077575e-02,
//             -1.04625389e+00, -5.22764336e-02,  4.89383264e-01,
//             -5.25300828e-02,  5.22016626e-01,  6.36962525e-02,
//             -9.12444679e-01, 0.05062224;
// }

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

//     Eigen::VectorXd l_modelInput(17);
//     l_modelInput << p_previousVelocityX,
//             p_previousVelocityY,
//             p_previousAngularVelocity,
//             p_nextVelocityX,
//             p_nextVelocityY,
//             p_nextAngularVelocity,
//             p_odomVelocityState.linear.x,
//             p_odomVelocityState.linear.y,
//             p_currentFeetConfiguration.flCoM.x,
//             p_currentFeetConfiguration.flCoM.y,
//             p_currentFeetConfiguration.frCoM.x,
//             p_currentFeetConfiguration.frCoM.y,
//             p_currentFeetConfiguration.rlCoM.x,
//             p_currentFeetConfiguration.rlCoM.y,
//             p_currentFeetConfiguration.rrCoM.x,
//             p_currentFeetConfiguration.rrCoM.y,
//             1;

    ROS_DEBUG_STREAM("Model input: " << l_modelInput);
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
    l_footCommonMarker.scale.x = 0.05;
    l_footCommonMarker.scale.y = 0.035;
    l_footCommonMarker.scale.z = 0.035;
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
    ros::Duration(3).sleep();
}