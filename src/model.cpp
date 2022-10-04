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
    m_fr_rl_com_x_fs << 2.77526419e-03, -8.32667268e-17,  2.01227923e-16,
            4.69905194e-02,  1.11022302e-16,  0.00000000e+00,
            2.87532978e-01, -2.43530778e-01,  1.11308553e-01,
            -4.23240309e-01, -2.79146557e-01,  3.03353683e-01,
            3.36406895e-01, -4.72145102e-01, -4.57421347e-01,
            3.44044082e-02, 0.17190311;

    m_fr_rl_com_y_fs.resize(17);
    m_fr_rl_com_y_fs << -2.65796579e-03, -2.46330734e-16, -8.52182908e-17,
            1.21094016e-02, -1.17961196e-16, -2.77555756e-17,
            -5.73503822e-02,  4.26220227e-02,  7.25195293e-04,
            4.46100237e-03, -2.24295029e-01, -2.11571601e-01,
            1.05403322e-01, -2.34944736e-01,  2.07400980e-01,
            2.63792261e-01, 0.17240134;

    m_fr_rl_com_theta_fs.resize(17);
    m_fr_rl_com_theta_fs << -1.35111431e-03, -1.73472348e-18,  2.07489191e-17,
            4.37471165e-04,  0.00000000e+00, -3.46944695e-18,
            9.01170612e-03, -7.82359573e-03, -3.02896738e-02,
            4.14850033e-02, -5.78213433e-02,  2.16185249e-02,
            1.48086357e-02, -3.77938288e-02, -2.97692727e-02,
            -3.12415815e-02, 0.01474343;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x_fs.resize(17);
    m_fl_rr_com_x_fs << -1.47640789e-03,  1.11022302e-16, -3.81639165e-17,
            6.54107787e-02, -2.77555756e-17, -5.55111512e-17,
            1.65593551e-01,  2.93266375e-01,  1.53336474e-01,
            -2.30601728e-01,  9.13457021e-03,  8.62089957e-01,
            8.36096403e-02, -2.44100786e-01, -5.52420849e-01,
            4.71175427e-01, 0.07896422;

    m_fl_rr_com_y_fs.resize(17);
    m_fl_rr_com_y_fs << 2.83282075e-03,  7.84962373e-17, -4.94396191e-17,
            -1.17035915e-02, -6.80878964e-17,  8.32667268e-17,
            4.20458553e-02,  6.08543337e-02,  5.36304005e-02,
            -5.87345439e-01, -1.49718401e-01,  8.84559283e-02,
            1.43289981e-02,  2.38085799e-01, -2.55141729e-02,
            4.26387277e-02, 0.08207957;

    m_fl_rr_com_theta_fs.resize(17);
    m_fl_rr_com_theta_fs << 3.55165195e-04, -2.51534904e-17,  2.64545330e-17,
            1.93148724e-04, -7.80625564e-18,  2.77555756e-17,
            -1.33578481e-03, -3.10561553e-02,  1.68479235e-02,
            -2.28574763e-02,  2.32277937e-02,  8.14490533e-02,
            -2.38708084e-04, -1.45552482e-01,  6.73729935e-02,
            -1.86602281e-02, 0.03844364;

    // FL models coefficients
    m_fl_swinging_x_fs.resize(17);
    m_fl_swinging_x_fs << 3.46935156e-04,  5.55111512e-17, -1.94289029e-16,
            1.11387699e-01,  5.55111512e-17, -1.11022302e-16,
            3.08885361e-01,  4.35106508e-01, -7.34559817e-01,
            -4.51317781e-01, -3.62612211e-01,  1.02971014e+00,
            1.21480929e-01,  2.75909890e-02, -7.42530757e-01,
            6.79128886e-01, 0.39078435;

    m_fl_swinging_y_fs.resize(17);
    m_fl_swinging_y_fs << 2.87356090e-03, -3.79470760e-16, -5.85686014e-16,
            -1.79156003e-02,  2.22044605e-16, -2.77555756e-16,
            6.66218094e-02,  1.12799458e-01,  1.50651681e-01,
            -1.65504496e+00, -1.28186851e-01,  2.41455319e-01,
            -8.11347165e-02,  5.07727883e-01, -1.19298709e-01,
            4.81805284e-02, 0.12826388;

    // FR models coefficients
    m_fr_swinging_x_fs.resize(17);
    m_fr_swinging_x_fs << 3.98224341e-03, -5.55111512e-17,  4.85722573e-16,
            8.22144186e-02, -1.11022302e-16, -1.66533454e-16,
            4.94739690e-01, -3.73032500e-01, -2.75750954e-02,
            -4.10249226e-01, -1.40856655e+00,  4.75502400e-01,
            6.06154250e-01, -8.00377804e-01, -8.20494465e-01,
            -4.19619703e-01, 0.46027035;
    m_fr_swinging_y_fs.resize(17);
    m_fr_swinging_y_fs << -4.89229352e-03, -7.62844649e-16, -1.52655666e-16,
            2.12065524e-02, -1.11022302e-16, -5.55111512e-17,
            -1.00576930e-01,  7.31861869e-02, -5.10246423e-02,
            2.00576322e-01, -3.53361511e-01, -4.03343688e-01,
            1.16102068e-01, -1.04414038e+00,  3.63741882e-01,
            6.04855061e-01, 0.35768501;

    // RL models coefficients
    m_rl_swinging_x_fs.resize(17);
    m_rl_swinging_x_fs << 5.07896860e-03,  3.33066907e-16,  6.93889390e-16,
            7.91226524e-02,  0.00000000e+00,  1.11022302e-16,
            5.05991188e-01, -3.98561316e-01,  8.70109057e-02,
            -4.56166271e-01, -4.91056354e-01,  5.52613713e-01,
            -2.92097800e-01, -7.81038220e-01, -9.82197175e-01,
            -5.14440556e-01, -0.06001033;
    m_rl_swinging_y_fs.resize(17);
    m_rl_swinging_y_fs << -4.89229352e-03, -7.62844649e-16, -1.52655666e-16,
            2.12065524e-02, -1.11022302e-16, -5.55111512e-17,
            -1.00576930e-01,  7.31861869e-02, -5.10246423e-02,
            2.00576322e-01, -3.53361511e-01, -4.03343688e-01,
            1.16102068e-01, -1.04414038e+00,  3.63741882e-01,
            6.04855061e-01, 0.35768501;

    // RR models coefficients
    m_rr_swinging_x_fs.resize(17);
    m_rr_swinging_x_fs << 1.60697067e-03,  2.22044605e-16, -4.30211422e-16,
            1.10537141e-01, -2.22044605e-16, -4.44089210e-16,
            3.14518996e-01,  4.63574525e-01,  1.56979991e-01,
            -5.55036739e-01, -3.10396827e-01,  1.09124143e+00,
            4.92974314e-02,  9.29311633e-02, -1.67279217e+00,
            7.00680754e-01, -0.08075705;
    m_rr_swinging_y_fs.resize(17);
    m_rr_swinging_y_fs << 3.75212675e-03, -4.03323208e-17, -2.56739074e-16,
            -1.76154792e-02,  3.12250226e-17, -2.08166817e-16,
            5.51370021e-02,  7.61758671e-02,  1.12861183e-01,
            -8.61488127e-01, -1.61252310e-01,  2.28477024e-01,
            4.61505517e-03,  3.81286044e-01, -5.83409559e-02,
            -3.87265479e-01, 0.0383474;
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
//     ros::Duration(0.3).sleep();
}