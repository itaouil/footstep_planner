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
    m_fr_rl_com_x << 3.65843037e-03,  7.77156117e-16,  7.77156117e-16,
            1.34716990e-01,  3.33066907e-16,  6.24500451e-17,
            1.74451143e+00, -8.31531586e-01,  3.54007828e-01,
            7.76676380e-01, -1.02494920e+00,  1.91731825e-01,
            -1.24448812e+00,  9.53823960e-01, -0.79146451;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << -4.98024681e-04, -3.12250226e-17, -4.16333634e-17,
            -8.57600305e-03,  1.11022302e-16,  1.11022302e-16,
            -1.73524239e-01,  8.58471733e-02, -1.45135965e-01,
            -4.95910494e-01,  2.59773761e-01, -2.84247820e-01,
            3.09453809e-01,  2.55902230e-01, 0.22172862;

    m_fr_rl_com_theta.resize(15);
    m_fr_rl_com_theta << -4.30759868e-04, -2.60208521e-17, -2.60208521e-17,
            2.68842957e-03, -2.08166817e-17, -3.46944695e-18,
            -7.41039438e-02,  2.79472175e-02, -3.81684048e-02,
            2.83154415e-02, -1.50632364e-02, -1.72087663e-02,
            4.90469594e-02, -4.73424334e-02, 0.02805025;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 6.66322161e-02,  8.32667268e-16,  1.11022302e-16,
            1.25720867e-01,  1.66533454e-16, -2.22044605e-16,
            3.24126863e-01, -5.88653351e-01,  1.19297730e+00,
            6.62317918e-02, -1.03613129e+00, -3.33800555e-01,
            -9.54747461e-01,  4.15438972e-01, -0.69347558;

    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << 3.98780137e-03,  2.60208521e-16,  6.93889390e-17,
            1.13609441e-02,  0.00000000e+00, -5.55111512e-17,
            2.59657818e-01, -3.78959503e-01,  1.80363451e-01,
            1.84960451e-01, -2.94571354e-01,  1.35783713e-01,
            -3.92040916e-01, -2.69581716e-01, -0.26279804;

    m_fl_rr_com_theta.resize(15);
    m_fl_rr_com_theta << -1.12684884e-04, -6.93889390e-18, -3.81639165e-17,
            -2.88428471e-03, -3.46944695e-17,  4.16333634e-17,
            7.00417178e-02,  8.76459516e-02,  1.81544681e-01,
            9.40326610e-02, -1.80010043e-01, -1.26990207e-01,
            -2.54071509e-02, -7.31995267e-02, -0.10375172;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 9.21215537e-02,  2.22044605e-16, -7.77156117e-16,
            2.22933080e-01,  4.44089210e-16, -1.11022302e-16,
            -1.03811447e-01, -9.92177934e-01,  1.39991977e+00,
            3.30346889e-01, -1.19122471e+00, -2.23577066e-01,
            -1.87042129e+00,  5.52300992e-01, -0.8231559;

    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << 9.34460236e-03,  8.55435514e-17, -3.05311332e-16,
            1.68173998e-02,  4.44089210e-16, -1.11022302e-16,
            4.70876686e-01, -1.17181580e+00,  5.30713271e-01,
            4.36797759e-01, -7.18398484e-01,  3.89175176e-01,
            -7.05528256e-01, -4.82654390e-01, -0.51406359;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << 1.59041766e-02,  4.44089210e-16, -5.55111512e-16,
            2.45885593e-01,  2.22044605e-16, -8.32667268e-17,
            2.03714765e+00, -1.31605832e+00, -4.55392637e-01,
            1.12418889e+00, -1.61818941e+00,  6.48793954e-01,
            -1.52483477e+00,  1.01096750e+00, -0.83977299;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << 2.03375730e-03, -4.16984156e-16,  1.38777878e-16,
            -1.26726550e-02,  2.22044605e-16, -2.22044605e-16,
            -5.12280764e-01,  3.21886936e-01, -1.85506939e-01,
            -8.96773691e-01,  3.93966306e-01, -1.24265884e+00,
            6.75808751e-01,  5.71400241e-01, 0.52675242;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << 1.70366218e-02, -1.11022302e-16, -3.33066907e-16,
            2.46987747e-01,  0.00000000e+00,  2.22044605e-16,
            2.17283985e+00, -1.34653939e+00,  5.08311672e-01,
            1.19941330e+00, -2.62335561e+00,  6.64897183e-01,
            -1.68022444e+00,  1.01955256e+00, -1.39231793;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y <<  2.03375730e-03, -4.16984156e-16,  1.38777878e-16,
            -1.26726550e-02,  2.22044605e-16, -2.22044605e-16,
            -5.12280764e-01,  3.21886936e-01, -1.85506939e-01,
            -8.96773691e-01,  3.93966306e-01, -1.24265884e+00,
            6.75808751e-01,  5.71400241e-01, 0.52675242;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << 9.51593966e-02, -7.77156117e-16, -7.77156117e-16,
            2.25742902e-01,  0.00000000e+00, -2.22044605e-16,
            9.13961028e-01, -1.13946763e+00,  1.44831005e+00,
            4.28673894e-01, -1.26458250e+00, -2.15911001e-01,
            -2.93608838e+00,  5.60026359e-01, -1.34211109;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y << 9.89207881e-03, -2.88289358e-16, -2.22044605e-16,
            1.19624483e-02,  3.33066907e-16,  0.00000000e+00,
            3.82879365e-01, -5.92260792e-01,  4.44359062e-01,
            5.90540711e-01, -6.24888288e-01,  2.51350717e-01,
            -6.05902362e-01, -1.17589526e+00, -0.54626765;

    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x_fs.resize(17);
    m_fr_rl_com_x_fs << -8.45388554e-04,  2.77555756e-16,  2.68882139e-16,
            7.72536720e-02,  7.21644966e-16,  8.11308486e-16,
            2.74144107e-01, -1.87852390e-01,  1.30945846e+00,
            -6.68719058e-01,  5.42298288e-02,  3.83098616e-01,
            -1.19459536e-01, -1.23415616e-01, -1.65063744e+00,
            8.98692870e-01, -0.52081735;

    m_fr_rl_com_y_fs.resize(17);
    m_fr_rl_com_y_fs << 1.02416146e-03, -5.74627151e-17, -8.32667268e-17,
            2.49579538e-03, -9.02056208e-17, -5.55111512e-17,
            -2.36361211e-02,  7.80294642e-02, -1.90057925e-01,
            3.18190109e-02, -8.69277897e-02, -3.58701151e-01,
            1.52182603e-01, -1.62680071e-01,  3.61215539e-01,
            1.73856731e-01, 0.19437238;

    m_fr_rl_com_theta_fs.resize(17);
    m_fr_rl_com_theta_fs << -4.12210337e-04, -4.33680869e-18, -6.28837260e-18,
            2.27656023e-03, -2.42861287e-17, -2.08166817e-17,
            4.23184561e-03,  1.90305582e-03, -8.50174252e-02,
            2.73542265e-02, -4.02822576e-02,  3.02618333e-02,
            -3.37866147e-03, -1.47413878e-02,  4.40780154e-02,
            -5.49355034e-02, 0.03191574;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x_fs.resize(17);
    m_fl_rr_com_x_fs << 2.60532991e-02,  2.49800181e-16, -2.06432094e-16,
            8.48624496e-02,  1.66533454e-16, -3.05311332e-16,
            2.42177618e-01,  1.40143312e-01,  3.00187600e-01,
            -6.02238052e-01,  5.27283837e-01,  4.10856889e-01,
            -9.85758453e-01, -3.24312318e-01, -4.96082087e-01,
            4.12954122e-01, -0.35048034;

    m_fl_rr_com_y_fs.resize(17);
    m_fl_rr_com_y_fs << 6.02958755e-04,  8.50014503e-17, -1.56125113e-17,
            2.27303994e-03, -3.46944695e-17, -8.32667268e-17,
            1.01482181e-02,  9.05228433e-02,  1.75684379e-01,
            -2.73091641e-01,  1.68652106e-01,  1.14214816e-01,
            -2.54354672e-01,  6.30406900e-02, -2.89811914e-01,
            -1.26570935e-01, -0.19699106;

    m_fl_rr_com_theta_fs.resize(17);
    m_fl_rr_com_theta_fs << 8.51398224e-04,  0.00000000e+00,  5.16080234e-17,
            -1.97372469e-03,  2.42861287e-17,  6.93889390e-18,
            -5.86019583e-03, -2.49450739e-03,  6.97421239e-02,
            8.91021234e-02,  1.97824512e-01,  8.47910954e-02,
            -1.80825320e-01, -1.27994555e-01, -3.56266158e-02,
            -7.16234336e-02, -0.11150667;

    // FL models coefficients
    m_fl_swinging_x_fs.resize(17);
    m_fl_swinging_x_fs << 3.99236148e-02,  4.44089210e-16,  1.38777878e-16,
            1.59295696e-01,  0.00000000e+00, -1.11022302e-16,
            3.05825289e-01,  3.31840627e-01, -2.72860578e-01,
            -8.43987802e-01,  5.73404203e-01,  5.61016182e-01,
            -1.03697284e+00, -3.42834317e-01, -1.12164121e+00,
            8.53467084e-01, -0.28261992;

    m_fl_swinging_y_fs.resize(17);
    m_fl_swinging_y_fs << 3.16392596e-03, -1.89952221e-16,  7.63278329e-17,
            2.56130539e-03, -8.32667268e-17, -3.33066907e-16,
            1.67597725e-02,  1.44048492e-01,  3.61527556e-01,
            -1.01159190e+00,  4.99177853e-01,  3.32239977e-01,
            -6.36019071e-01,  2.67073822e-01, -5.61628165e-01,
            -2.53216534e-01, -0.40895725;

    // FR models coefficients
    m_fr_swinging_x_fs.resize(17);
    m_fr_swinging_x_fs << 8.68220025e-03,  1.11022302e-16,  4.75531073e-16,
            1.45489732e-01,  1.22124533e-15,  1.11022302e-16,
            4.49474324e-01, -3.48052601e-01,  1.40286629e+00,
            -1.00032007e+00, -8.65321432e-01,  4.87214660e-01,
            -2.15092121e-01, -9.19144009e-02, -2.23902247e+00,
            9.26364377e-01, -0.44208667;
    m_fr_swinging_y_fs.resize(17);
    m_fr_swinging_y_fs << 4.50562190e-03, -3.48245738e-16, -2.70616862e-16,
            7.58814845e-03, -5.55111512e-16,  5.55111512e-17,
            -4.55525990e-02,  1.35895735e-01, -5.33315318e-01,
            2.22480654e-01, -1.04439600e-01, -6.72096365e-01,
            2.18906415e-01, -9.95162751e-01,  7.74509662e-01,
            4.46542526e-01, 0.48382048;

    // RL models coefficients
    m_rl_swinging_x_fs.resize(17);
    m_rl_swinging_x_fs << 9.60066843e-03,  4.99600361e-16,  4.29344060e-16,
            1.46411594e-01,  1.55431223e-15,  3.33066907e-16,
            4.41333784e-01, -3.61687518e-01,  1.56695191e+00,
            -1.02316255e+00,  9.79756651e-02,  5.42246658e-01,
            -1.23915863e+00, -9.65632431e-02, -2.38669282e+00,
            9.62921720e-01, -1.00131348;
    m_rl_swinging_y_fs.resize(17);
    m_rl_swinging_y_fs <<  4.50562190e-03, -3.48245738e-16, -2.70616862e-16,
            7.58814845e-03, -5.55111512e-16,  5.55111512e-17,
            -4.55525990e-02,  1.35895735e-01, -5.33315318e-01,
            2.22480654e-01, -1.04439600e-01, -6.72096365e-01,
            2.18906415e-01, -9.95162751e-01,  7.74509662e-01,
            4.46542526e-01;

    // RR models coefficients
    m_rr_swinging_x_fs.resize(17);
    m_rr_swinging_x_fs << 4.20789807e-02,  2.22044605e-16,  3.26128013e-16,
            1.60602170e-01, -6.10622664e-16, -3.40005801e-16,
            3.09758431e-01,  3.44113680e-01,  7.36372043e-01,
            -9.79697382e-01,  6.12039999e-01,  6.54855503e-01,
            -1.10364486e+00, -3.44052386e-01, -2.17113581e+00,
            8.78608991e-01, -0.78982236;
    m_rr_swinging_y_fs.resize(17);
    m_rr_swinging_y_fs <<  3.76090069e-03, -1.09287579e-16,  9.19403442e-17,
            -1.99113114e-03,  3.88578059e-16,  0.00000000e+00,
            1.71706827e-02,  1.39959228e-01,  2.76909384e-01,
            -4.37318394e-01,  4.11217356e-01,  4.90515798e-01,
            -5.44965346e-01,  1.33253225e-01, -4.64702683e-01,
            -9.53725230e-01, -0.4431549;
}

/**
 * Footstep prediction for first step.
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
void Model::predictFirstStep(double p_previousVelocityX,
                             double p_previousVelocityY,
                           double p_previousAngularVelocity,
                           double p_nextVelocityX,
                           double p_nextVelocityY,
                           double p_nextAngularVelocity,
                           const geometry_msgs::Twist &p_odomVelocityState,
                           const FeetConfiguration &p_currentFeetConfiguration,
                           std::vector<double> &p_predictions) {
    // Regression input
     Eigen::VectorXd l_modelInput(17);
     l_modelInput << p_previousVelocityX,
             p_previousVelocityY,
             p_previousAngularVelocity,
             p_nextVelocityX,
             p_nextVelocityY,
             p_nextAngularVelocity,
             p_odomVelocityState.linear.x,
             p_odomVelocityState.linear.y,
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
        p_predictions[0] = m_fr_rl_com_x_fs * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y_fs * l_modelInput;

        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        p_predictions[4] = m_fr_swinging_x_fs * l_modelInput;
        p_predictions[5] = m_fr_swinging_y_fs * l_modelInput;

        p_predictions[6] = m_rl_swinging_x_fs * l_modelInput;
        p_predictions[7] = m_rl_swinging_y_fs * l_modelInput;

        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;

        p_predictions[10] = m_fr_rl_com_theta_fs * l_modelInput;
    }
        // FL/RR are swinging
    else {
        p_predictions[0] = m_fl_rr_com_x_fs * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y_fs * l_modelInput;

        p_predictions[2] = m_fl_swinging_x_fs * l_modelInput;
        p_predictions[3] = m_fl_swinging_y_fs * l_modelInput;

        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

        p_predictions[8] = m_rr_swinging_x_fs * l_modelInput;
        p_predictions[9] = m_rr_swinging_y_fs * l_modelInput;

        p_predictions[10] = m_fl_rr_com_theta_fs * l_modelInput;
    }
}

/**
 * Footstep prediction for after second step
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
void Model::predictOnwardSteps(double p_previousVelocityX,
                               double p_previousVelocityY,
                               double p_previousAngularVelocity,
                               double p_nextVelocityX,
                               double p_nextVelocityY,
                               double p_nextAngularVelocity,
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
    //l_velocityCommandQuaternion.setRPY(0, 0, p_predictedCoMDisplacementTheta);
    l_velocityCommandQuaternion.setRPY(0, 0, 0);

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
void Model::predictNextState(uint p_plannedFootstep,
                             double p_previousVelocity,
                             double p_nextVelocity,
                             const Action &p_action,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    // Predict feet and CoM displacements
    std::vector<double> l_predictions(11);
    if (p_plannedFootstep == 0) {
        predictFirstStep(p_action.x * p_previousVelocity,
                         p_action.y * p_previousVelocity,
                         p_action.theta * p_previousVelocity,
                         p_action.x * p_nextVelocity,
                         p_action.y * p_nextVelocity,
                         p_action.theta * p_nextVelocity,
                         p_odomVelocityState,
                         p_currentFeetConfiguration,
                         l_predictions);
    }
    else {
        predictOnwardSteps(p_action.x * p_previousVelocity,
                           p_action.y * p_previousVelocity,
                           p_action.theta * p_previousVelocity,
                           p_action.x * p_nextVelocity,
                           p_action.y * p_nextVelocity,
                           p_action.theta * p_nextVelocity,
                           p_currentFeetConfiguration,
                           l_predictions);
    }

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

//     // Publish predicted CoM and feet poses
//     int j = 0;
//     visualization_msgs::Marker l_footCommonMarker;
//     l_footCommonMarker.header.stamp = ros::Time::now();
//     l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//     l_footCommonMarker.type = 2;
//     l_footCommonMarker.action = 0;
//     l_footCommonMarker.lifetime = ros::Duration(1);
//     l_footCommonMarker.pose.orientation.x = 0;
//     l_footCommonMarker.pose.orientation.y = 0;
//     l_footCommonMarker.pose.orientation.z = 0;
//     l_footCommonMarker.pose.orientation.w = 1;
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
//     l_CoMMarker.pose.position.z = 0;

//     visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
//     l_flFootMarker.id = j++;
//     l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
//     l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
//     l_flFootMarker.pose.position.z = 0;

//     visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//     l_frFootMarker.id = j++;
//     l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
//     l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
//     l_frFootMarker.pose.position.z = 0;

//     visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//     l_rlFootMarker.id = j++;
//     l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
//     l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
//     l_rlFootMarker.pose.position.z = 0;

//     visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//     l_rrFootMarker.id = j++;
//     l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
//     l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
//     l_rrFootMarker.pose.position.z = 0;

//     visualization_msgs::MarkerArray l_pathFeetConfiguration;
//     l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
//     l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

//     m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
//     ros::Duration(0.1).sleep();
}