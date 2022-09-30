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
    m_fr_rl_com_x.resize(15);
    m_fr_rl_com_x << -6.18952261e-03, -5.55111512e-17, -3.33066907e-16,
            1.13715255e-01, -1.11022302e-16, -1.11022302e-16,
            1.64574462e+00, -1.04013158e+00,  3.91263962e-01,
            1.34018877e-01, -1.35697268e+00,  3.07097814e-01,
            -1.31066846e+00, -1.41898796e-01, -1.08745703;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << -5.57753542e-03,  3.81639165e-17,  4.85722573e-17,
            -4.90504824e-04,  1.66533454e-16,  0.00000000e+00,
            -1.96652040e-01,  1.90804664e-01, -1.99745317e-01,
            -1.68907797e-01,  2.76513777e-01, -3.12647794e-01,
            3.23181712e-01,  4.07075330e-01, 0.29745351;

    m_fr_rl_com_theta.resize(15);
    m_fr_rl_com_theta << -2.23899265e-03, -1.73472348e-18, -6.93889390e-18,
            8.30354025e-04,  1.73472348e-18,  1.04083409e-17,
            -3.82650019e-02, -5.30645949e-02, -1.27893636e-02,
            -5.30260737e-02, -6.04442231e-02,  4.35002729e-02,
            2.04268364e-02,  2.79697724e-03, -0.00520343;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 5.33967944e-04, -1.66533454e-16, -7.77156117e-16,
            1.17609849e-01,  2.22044605e-16,  1.11022302e-16,
            7.30313912e-01, -5.36473017e-01,  4.80428581e-01,
            9.93570985e-01,  8.16805628e-02, -7.82672587e-02,
            -1.58026444e+00, -6.58256026e-01, -0.55906468;

    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << 2.07666783e-03, -2.60208521e-17, -3.19189120e-16,
            7.57148290e-03, -5.55111512e-17,  1.11022302e-16,
            2.25393905e-01, -5.30319512e-01,  5.87699292e-02,
            1.95327386e-01, -1.01144659e-01,  2.33529118e-01,
            -3.45397327e-01, -2.43152644e-01, -0.15436019;

    m_fl_rr_com_theta.resize(15);
    m_fl_rr_com_theta << 1.97909932e-04,  3.46944695e-17,  1.04083409e-17,
            -2.59544990e-03, -6.07153217e-18,  3.46944695e-18,
            -3.24265507e-02, -1.84736530e-02,  1.33883569e-02,
            -5.40264974e-02, -6.96626508e-03, -4.57891997e-02,
            1.09816604e-01,  1.11068387e-01, 0.04954298;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 5.80524114e-03, -4.44089210e-16, -8.88178420e-16,
            2.06302299e-01,  2.22044605e-16,  2.22044605e-16,
            3.29927326e-01, -9.99173583e-01,  4.46565389e-01,
            1.47872526e+00,  2.64613473e-01,  1.92678948e-01,
            -2.58514598e+00, -1.20729352e+00, -0.64821702;

    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << 2.67892269e-03,  1.38777878e-17, -6.55725474e-16,
            9.70699168e-03,  0.00000000e+00,  1.11022302e-16,
            2.62501821e-01, -1.35041414e+00,  1.16711768e-01,
            3.57393160e-01, -2.04400017e-01,  4.73268162e-01,
            -4.93680102e-01, -4.44433434e-01, -0.17506776;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << -9.40592229e-03,  1.11022302e-16,  2.22044605e-16,
            2.02410657e-01, -4.44089210e-16, -3.33066907e-16,
            2.42513416e+00, -1.68683935e+00, -2.44558514e-01,
            2.23209062e-01, -2.27595623e+00,  6.55645447e-01,
            -2.01230479e+00, -6.56906561e-01, -1.55683452;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << -8.13426797e-03,  2.01227923e-16,  1.52655666e-16,
            -1.84107449e-03,  2.22044605e-16,  0.00000000e+00,
            -4.68975428e-01,  5.52041982e-01, -3.38497680e-01,
            -2.29424254e-01,  4.52721769e-01, -1.08173895e+00,
            6.15159518e-01,  8.35581693e-01, 0.61588349;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << -7.85527555e-03, -3.33066907e-16, -3.33066907e-16,
            2.03973132e-01, -6.66133815e-16,  0.00000000e+00,
            2.49848376e+00, -1.73719248e+00,  7.22527478e-01,
            2.62697386e-01, -3.25092439e+00,  6.61325186e-01,
            -2.11894219e+00, -7.51022697e-01, -2.08812562;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y << -8.13426797e-03,  2.01227923e-16,  1.52655666e-16,
            -1.84107449e-03,  2.22044605e-16,  0.00000000e+00,
            -4.68975428e-01,  5.52041982e-01, -3.38497680e-01,
            -2.29424254e-01,  4.52721769e-01, -1.08173895e+00,
            6.15159518e-01,  8.35581693e-01, 0.61588349;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << 5.68535333e-03, -8.88178420e-16, -1.33226763e-15,
            2.09583926e-01, -4.44089210e-16,  2.22044605e-16,
            1.25314280e+00, -1.07228066e+00,  4.04180408e-01,
            1.53673760e+00,  3.04358529e-01,  2.17394677e-01,
            -3.55599146e+00, -1.26152628e+00, -1.09648662;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y << 1.98410606e-03, -4.85722573e-17, -4.44089210e-16,
            5.30420523e-03,  1.11022302e-16,  5.55111512e-17,
            2.77089864e-01, -7.28364830e-01,  5.17911377e-02,
            3.88363660e-01, -8.04528198e-02,  3.29369386e-01,
            -4.18785721e-01, -6.47946453e-01, -0.19201532;

    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x_fs.resize(17);
    m_fr_rl_com_x_fs << 7.82035248e-03, -2.77555756e-17,  1.11022302e-16,
            4.89721866e-02,  2.77555756e-17,  1.11022302e-16,
            2.93113736e-01, -1.51579446e-01,  4.41249003e-01,
            -7.16959685e-01, -1.12674359e-01, -1.27025125e-01,
            8.12123807e-02, -3.04450239e-02, -8.96752638e-01,
            -1.40670087e-01, -0.225816;

    m_fr_rl_com_y_fs.resize(17);
    m_fr_rl_com_y_fs << -4.00821524e-03, -7.80625564e-17,  6.93889390e-17,
            1.55582321e-02,  8.32667268e-17,  0.00000000e+00,
            -5.11842229e-02,  3.05639567e-02, -1.66204996e-04,
            2.91770778e-02, -1.51376131e-01, -2.33365119e-01,
            4.34823239e-02, -1.91230522e-01,  2.08635265e-01,
            3.35670431e-01, 0.13543826;

    m_fr_rl_com_theta_fs.resize(17);
    m_fr_rl_com_theta_fs << -4.90843024e-04, -1.47451495e-17, -2.60208521e-18,
            3.21060177e-04, -5.20417043e-18,  0.00000000e+00,
            8.56367279e-03,  2.86251151e-03, -1.49866914e-02,
            -2.81282507e-03, -3.79557297e-02, -1.02850554e-02,
            -5.01182967e-03,  1.81472955e-02, -3.18023104e-02,
            -4.54588900e-02, -0.00763914;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x_fs.resize(17);
    m_fl_rr_com_x_fs << 3.75912288e-03,  2.77555756e-17, -2.77555756e-17,
            5.62522559e-02, -5.20417043e-17, -5.55111512e-17,
            2.28846181e-01,  2.06211771e-01, -5.80364133e-02,
            1.05269232e-02, -6.29745107e-02,  9.39519728e-01,
            -1.22945273e-01, -1.86813648e-01, -2.00832418e-01,
            1.45324058e-01, 0.11201191;

    m_fl_rr_com_y_fs.resize(17);
    m_fl_rr_com_y_fs << 2.96401104e-03,  9.36750677e-17,  5.89805982e-17,
            -1.13726129e-02, -2.77555756e-17,  4.16333634e-17,
            5.32170318e-02,  5.27727604e-02,  1.11655993e-01,
            -4.69088333e-01,  3.80951579e-02,  5.13672519e-02,
            -2.06511399e-01,  1.76229731e-01, -6.59591462e-02,
            2.99767591e-03, -0.06230189;

    m_fl_rr_com_theta_fs.resize(17);
    m_fl_rr_com_theta_fs << -9.69819437e-04, -1.73472348e-18,  5.20417043e-18,
            1.46323355e-04, -2.51534904e-17,  1.38777878e-17,
            -9.42565404e-03, -1.22945547e-02,  3.14725519e-03,
            -2.01648295e-02,  1.03209771e-02,  2.25842390e-02,
            3.17088871e-02, -1.28863230e-01,  5.18945449e-02,
            5.57207858e-02, 0.04966778;

    // FL models coefficients
    m_fl_swinging_x_fs.resize(17);
    m_fl_swinging_x_fs << 1.38695149e-02,  0.00000000e+00, -8.32667268e-17,
            9.60378763e-02,  1.11022302e-16, -2.22044605e-16,
            3.96485060e-01,  3.01568056e-01, -8.49135998e-01,
            9.91224808e-02, -2.88160278e-01,  1.12293750e+00,
            -3.14905614e-01, -3.23589663e-02, -4.21877548e-01,
            6.95478287e-02, 0.23591961;

    m_fl_swinging_y_fs.resize(17);
    m_fl_swinging_y_fs << 2.73628058e-03, -3.29597460e-17, -1.30104261e-16,
            -2.02023694e-02,  3.46944695e-17,  5.55111512e-17,
            7.90787678e-02,  9.94385822e-02,  9.49689913e-02,
            -1.29783949e+00,  1.42428827e-01,  1.89261779e-01,
            -4.03570892e-01,  3.69062175e-01, -6.50130480e-02,
            -6.76670594e-02, -0.04253508;

    // FR models coefficients
    m_fr_swinging_x_fs.resize(17);
    m_fr_swinging_x_fs << 1.68520316e-02,  5.55111512e-17,  4.71844785e-16,
            8.40901180e-02,  2.94902991e-16,  0.00000000e+00,
            4.86965800e-01, -2.32750645e-01,  5.15738382e-01,
            -9.51604699e-01, -1.02150273e+00, -1.61649902e-01,
            6.64602315e-02, -1.82583525e-01, -1.43961805e+00,
            -5.42957697e-01, -0.17462211;
    m_fr_swinging_y_fs.resize(17);
    m_fr_swinging_y_fs << -6.44554487e-03, -1.73472348e-16,  2.98372438e-16,
            2.37486144e-02,  2.22044605e-16,  0.00000000e+00,
            -8.30460916e-02,  5.66254760e-02, -1.51679171e-01,
            2.50757865e-01, -2.67388228e-01, -3.70967551e-01,
            6.30749988e-02, -8.10841086e-01,  4.34023112e-01,
            6.90536598e-01, 0.34199698;

    // RL models coefficients
    m_rl_swinging_x_fs.resize(17);
    m_rl_swinging_x_fs << 1.94965408e-02,  3.33066907e-16, -5.55111512e-17,
            8.03194136e-02, -5.55111512e-17,  0.00000000e+00,
            4.99637754e-01, -2.53447057e-01,  5.69603407e-01,
            -9.90703045e-01, -1.01435870e-01, -8.33045096e-02,
            -8.28674516e-01, -1.80852269e-01, -1.54933834e+00,
            -6.65545577e-01, -0.66872347;
    m_rl_swinging_y_fs.resize(17);
    m_rl_swinging_y_fs << -6.44554487e-03, -1.73472348e-16,  2.98372438e-16,
            2.37486144e-02,  2.22044605e-16,  0.00000000e+00,
            -8.30460916e-02,  5.66254760e-02, -1.51679171e-01,
            2.50757865e-01, -2.67388228e-01, -3.70967551e-01,
            6.30749988e-02, -8.10841086e-01,  4.34023112e-01,
            6.90536598e-01, 0.34199698;

    // RR models coefficients
    m_rr_swinging_x_fs.resize(17);
    m_rr_swinging_x_fs << 1.59163660e-02,  1.11022302e-16,  0.00000000e+00,
            9.40070420e-02,  1.11022302e-16,  0.00000000e+00,
            4.05478203e-01,  3.23071846e-01,  2.67458297e-03,
            -4.34415836e-02, -2.89169711e-01,  1.18154366e+00,
            -3.45379421e-01,  2.51665825e-02, -1.30172350e+00,
            7.92861303e-02, -0.18545623;
    m_rr_swinging_y_fs.resize(17);
    m_rr_swinging_y_fs << 3.88254758e-03,  6.93889390e-18, -4.16333634e-17,
            -1.76944886e-02, -3.12250226e-17,  6.93889390e-17,
            6.47179260e-02,  7.01027944e-02,  1.10183430e-01,
            -6.38920756e-01,  7.44724284e-02,  1.61983352e-01,
            -2.69421694e-01,  2.80811750e-01, -5.10515294e-02,
            -3.99948975e-01, -0.11287751;
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
//     ros::Duration(1).sleep();
}