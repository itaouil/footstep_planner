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
            TARGET_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

    // Set models coefficients
    setContinuousModelsCoefficients();
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
void Model::setContinuousModelsCoefficients() {
    // CoM models coeff. when FR/RL swinging
    m_fr_rl_com_x_continuous.resize(12);
    m_fr_rl_com_x_continuous << 0.20448312, 0.02240074, 0.00188171,
            0.11187166, -0.0808538, -0.04101755,
            -0.23039273, -0.36001609, 0.04022223,
            0.03627003, 0.0400849, -0.12861831;

    m_fr_rl_com_y_continuous.resize(12);
    m_fr_rl_com_y_continuous << 0.05117917, 0.15720341, 0.14881086,
            -0.08188562, 0.16416384, -0.1705173,
            0.0294997, -0.36337627, 0.2557664,
            0.08044179, 0.34774576, -0.01891646;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_continuous.resize(12);
    m_fl_rr_com_x_continuous << 0.208074, -0.0260617, -0.02767182,
            0.08265362, 0.28585941, 0.15437221,
            0.03577178, 0.1893044, -0.05469139,
            -0.44700136, -0.0270705, -0.16461866;
    m_fl_rr_com_y_continuous.resize(12);
    m_fl_rr_com_y_continuous << -0.03556395, 0.16703043, 0.16143274,
            0.07020055, 0.09660584, -0.01225719,
            0.26302163, -0.12424745, 0.38641548,
            0.23338804, 0.18419839, 0.00501586;

    // FL models coeff. in swing and support mode
    m_fl_support_x_continuous.resize(12);
    m_fl_support_x_continuous << 0.10658301, -0.00155091, 0.00635995,
            0.58309683, 0.09479599, 0.10273408,
            0.0943333, 0.09490395, 0.02371752,
            -0.04466508, -0.05406845, 0.06848236;
    m_fl_support_y_continuous.resize(12);
    m_fl_support_y_continuous << 0.00380356, 0.11609896, 0.12342861,
            0.07095975, 0.47267375, -0.05519404,
            0.00116808, 0.00156099, -0.07756335,
            -0.11221989, -0.11167796, 0.05274158;
    m_fl_swinging_x_continuous.resize(12);
    m_fl_swinging_x_continuous << -0.17641215, 0.03131934, 0.07871106,
            1.05227865, -0.08513168, -0.20973094,
            -0.01233789, -0.25000596, 0.11463451,
            0.00961209, -0.2744822, -0.06709406;
    m_fl_swinging_y_continuous.resize(12);
    m_fl_swinging_y_continuous << 0.03503179, -0.14626197, -0.21638538,
            0.04550731, 0.88492912, 0.13741106,
            -0.38422713, 0.1142604, -0.37038488,
            -0.29456627, -0.35413788, -0.12705611;

    // FR models coeff. in swing and support mode
    m_fr_support_x_continuous.resize(12);
    m_fr_support_x_continuous << 1.00664647e-01, 7.55042267e-03, 3.58554797e-04,
            2.14067080e-01, -9.22727270e-02, 5.54949445e-01,
            -8.59476903e-02, 3.39691184e-02, -4.38944884e-02,
            6.59389492e-02, -1.53755800e-01, 0.05774747;
    m_fr_support_y_continuous.resize(12);
    m_fr_support_y_continuous << -0.00802286, 0.11422631, 0.11751968,
            0.01632509, -0.04267717, -0.05139392,
            0.43730848, 0.05448194, -0.04376089,
            0.0547442, -0.00615641, -0.04498078;
    m_fr_swinging_x_continuous.resize(12);
    m_fr_swinging_x_continuous << -0.17380282, -0.02054916, -0.0643635,
            -0.10949882, 0.05845909, 0.99683475,
            0.0845313, 0.06441501, 0.09428347,
            -0.27935072, -0.20687726, -0.06473298;
    m_fr_swinging_y_continuous.resize(12);
    m_fr_swinging_y_continuous << -0.05294284, -0.14021495, -0.1968252,
            -0.08955578, -0.29841152, 0.1314537,
            0.92010731, 0.37725133, -0.35068304,
            0.00566134, -0.2940644, 0.13259666;

    // RL models coeff. in swing and support mode
    m_rl_support_x_continuous.resize(12);
    m_rl_support_x_continuous << 0.10358317, 0.00527787, 0.01485432,
            0.04319491, -0.02215411, -0.02752528,
            0.01071826, 0.49154416, 0.00477516,
            0.12695151, -0.09712314, -0.11146465;
    m_rl_support_y_continuous.resize(12);
    m_rl_support_y_continuous << 0.01074109, 0.12206866, 0.10396743,
            -0.01926315, 0.00820036, -0.02290314,
            -0.02577283, -0.01972549, 0.48587925,
            -0.10560013, -0.07473089, 0.04759789;
    m_rl_swinging_x_continuous.resize(12);
    m_rl_swinging_x_continuous << -0.19097837, -0.02138108, 0.02406218,
            -0.23042346, 0.0438194, -0.09814995,
            0.28208449, 1.38719173, 0.02709869,
            -0.16062742, -0.0184566, 0.16977393;
    m_rl_swinging_y_continuous.resize(12);
    m_rl_swinging_y_continuous << -0.03453704, -0.15958845, -0.07274127,
            0.21911934, -0.03941841, 0.14924886,
            0.0634269, 0.2834115, 0.83797488,
            -0.05058278, -0.31844141, -0.03807246;

    // RR models coeff. in swing and support mode
    m_rr_support_x_continuous.resize(12);
    m_rr_support_x_continuous << 0.1046991, -0.00120842, -0.00614089,
            -0.05673, -0.04772559, 0.08519632,
            -0.0304605, 0.08080738, 0.06929205,
            0.5117457, 0.0065906, -0.11729157;
    m_rr_support_y_continuous.resize(12);
    m_rr_support_y_continuous << -0.00792055, 0.11965755, 0.11289534,
            0.01376738, -0.01548227, 0.07151736,
            0.0383021, 0.03876793, -0.07270161,
            0.06026456, 0.50094288, -0.05823728;
    m_rr_swinging_x_continuous.resize(12);
    m_rr_swinging_x_continuous << -0.18948751, 0.03135044, -0.00926969,
            -0.18265266, -0.29318321, -0.18992702,
            -0.05755184, -0.18585136, 0.07530629,
            1.43151818, -0.08093772, 0.16859226;
    m_rr_swinging_y_continuous.resize(12);
    m_rr_swinging_y_continuous << 0.02831754, -0.16861656, -0.0953761,
            -0.08224199, 0.02359811, -0.10048776,
            -0.14292098, 0.11999834, -0.30567477,
            -0.21623308, 0.94606433, 0.03613883;
}

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of discontinuous
 * velocity commands.
 */
void Model::setAccelerationModelsCoefficients() {
    // CoM models coeff. when FR/RL swinging
    m_fr_rl_com_x_acceleration.resize(15);
    m_fr_rl_com_x_acceleration << 0.11001647, 0.02783948, 0.04073158,
            0.05846566, -0.01121587, -0.00521162,
            0.00387027, -0.03955413, 0.12278361,
            -0.16924034, -0.18056408, 0.04502309,
            0.12316514, 0.00832191, -0.07659243;

    m_fr_rl_com_y_acceleration.resize(15);
    m_fr_rl_com_y_acceleration << 0.01804221, 0.13285808, 0.1493604, -0.01451485, 0.04431237,
            0.05506978, -0.143388, 0.00765765, 0.15110374, -0.14170376,
            -0.14938378, 0.00959972, 0.26213222, 0.19404234, 0.03642354;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_acceleration.resize(15);
    m_fl_rr_com_x_acceleration << 0.10310523, -0.04248223, -0.0454877, 0.05772033, 0.01109563,
            0.00564731, 0.06121624, 0.20715478, -0.00027812, 0.1067157,
            0.10830793, 0.02200002, -0.03880562, 0.13421111, -0.00224403;
    m_fl_rr_com_y_acceleration.resize(15);
    m_fl_rr_com_y_acceleration << -0.03148482, 0.14047126, 0.16758582, 0.01519549, 0.04456748,
            0.0613687, 0.10037632, -0.21904149, 0.13816629, 0.14549049,
            -0.20060809, -0.00468627, 0.04287531, -0.05464679, -0.04688473;

    // FL models coeff. in swing and support mode
    m_fl_support_x_acceleration.resize(15);
    m_fl_support_x_acceleration << 1.29096873e-02, -2.63860273e-03, 1.56329979e-02,
            1.04755683e-01, 3.81081154e-04, -1.79749641e-02,
            4.03700919e-01, 1.30860788e-01, -2.17913950e-01,
            -1.76986521e-01, 1.95333487e-01, 2.43398185e-01,
            -1.01671413e-01, -1.94882684e-01, 0.08334157;
    m_fl_support_y_acceleration.resize(15);
    m_fl_support_y_acceleration << 6.35149238e-02, -4.15998549e-02, -1.32433736e-02,
            2.52861603e-04, 1.00710282e-01, 1.91604183e-01,
            -1.82484764e-01, 5.85253527e-01, 1.47603700e-01,
            5.39344394e-01, -8.33780381e-01, 9.68282178e-02,
            1.98060115e-02, 1.05471903e-03, -0.05176077;
    m_fl_swinging_x_acceleration.resize(15);
    m_fl_swinging_x_acceleration << -0.10509038, 0.03735759, 0.11685724, -0.0544782, -0.01001254,
            0.00160878, 0.97728556, -0.25455629, 0.05187412, -0.08716534,
            -0.27175396, -0.04998752, 0.03252996, -0.19477863, -0.05626477;
    m_fl_swinging_y_acceleration.resize(15);
    m_fl_swinging_y_acceleration << 0.05293856, -0.09071253, -0.21458831, -0.01587639, -0.04188412,
            -0.08901991, 0.14645452, 0.85853551, -0.15220421, -0.2628962,
            0.21059106, -0.05228912, -0.48656754, -0.1586353, -0.09959517;

    // FR models coeff. in swing and support mode
    m_fr_support_x_acceleration.resize(15);
    m_fr_support_x_acceleration << -2.20369038e-02, -1.40896067e-02, -2.40239291e-02,
            1.04523887e-01, -1.98485276e-04, 7.26833311e-03,
            2.53419644e-01, 2.83633427e-03, 5.54538153e-01,
            -4.57626068e-02, -4.89157898e-02, -1.47095453e-03,
            1.64291386e-01, 2.48700649e-02, 0.0656455;
    m_fr_support_y_acceleration.resize(15);
    m_fr_support_y_acceleration << -0.03875806, -0.02419546, -0.0111179, -0.00264573, 0.1031444,
            0.16616139, 0.03735991, 0.34677449, 0.16786598, 0.51771063,
            0.05097792, -0.0600186, 0.43818349, 0.02710729, -0.04993396;
    m_fr_swinging_x_acceleration.resize(15);
    m_fr_swinging_x_acceleration << -0.04971126, -0.04972579, -0.12536155, -0.05510811, 0.01027304,
            -0.00793679, -0.03434949, 0.21516733, 0.32945695, 0.12989169,
            0.06270029, 0.3237381, -0.37955112, -0.24186487, -0.02072315;
    m_fr_swinging_y_acceleration.resize(15);
    m_fr_swinging_y_acceleration << -0.04666457, -0.08932719, -0.22858309, 0.01694849, -0.04152483,
            -0.10641105, 0.13819043, -0.35376004, -0.35298952, 0.72883976,
            0.56013107, -0.14740541, -0.34839678, -0.12229748, 0.11902929;

    // RL models coeff. in swing and support mode
    m_rl_support_x_acceleration.resize(15);
    m_rl_support_x_acceleration << -0.01194095, -0.00794009, 0.01966228, 0.10418998, 0.00152168,
            -0.00350107, 0.22513921, -0.08314183, 0.04495293, 0.06642451,
            0.41322877, -0.10597716, 0.07476767, 0.04744409, -0.14169681;
    m_rl_support_y_acceleration.resize(15);
    m_rl_support_y_acceleration << -0.0422973, -0.02140077, 0.01396827, 0.00127373, 0.10205868,
            0.10537158, 0.06314382, 0.43674492, 0.08880562, 0.04639821,
            0.05681354, 0.47523043, 0.40734598, -0.04773779, 0.09497614;
    m_rl_swinging_x_acceleration.resize(15);
    m_rl_swinging_x_acceleration << -7.52284924e-02, -3.98099534e-02, -8.39012158e-03,
            -5.42908601e-02, 9.06821068e-03, -5.40644672e-05,
            -1.03608337e-01, 1.09548239e-01, -5.64605278e-01,
            3.15282372e-01, 1.24543335e+00, 8.49095745e-03,
            -2.33013046e-01, -1.12417497e-01, 0.17553432;
    m_rl_swinging_y_acceleration.resize(15);
    m_rl_swinging_y_acceleration << -0.0503605, -0.10136979, -0.0579076, 0.01411304, -0.04092923,
            -0.09116674, 0.17997489, -0.09777596, -0.2490051, -0.2979825,
            0.54298928, 1.05386997, -0.31220931, -0.27968325, -0.01789602;

    // RR models coeff. in swing and support mode
    m_rr_support_x_acceleration.resize(15);
    m_rr_support_x_acceleration << 0.00486416, 0.00119608, -0.01193982, 0.10531773, -0.00144003,
            -0.0012512, -0.11169108, 0.00078995, -0.09171185, -0.10997794,
            0.14711624, 0.14439952, 0.44424135, -0.05127216, -0.10838893;
    m_rr_support_y_acceleration.resize(15);
    m_rr_support_y_acceleration << 0.01496668, -0.01973009, 0.03339225, -0.00206447, 0.10296279,
            0.1261246, -0.11742698, 0.03805601, 0.39789799, 0.51792289,
            -0.52083761, -0.05243376, 0.14959623, 0.64207263, -0.12937615;
    m_rr_swinging_x_acceleration.resize(15);
    m_rr_swinging_x_acceleration << 0.13965662, 0.00639142, 0.01805187, -0.03651511, -0.00226514,
            -0.00642182, 0.04535437, -0.02437856, -0.02583746, 0.0088256,
            0.49331086, 0.01254572, 0.12948794, -0.07718626, -0.10922682;
    m_rr_swinging_y_acceleration.resize(15);
    m_rr_swinging_y_acceleration << -0.0019292, 0.12826301, 0.01495222, 0.01253768, -0.00615282,
            0.09494268, -0.02894828, 0.00971136, -0.01310489, -0.02393984,
            -0.02896051, 0.47161335, -0.09023618, -0.08713394, 0.04930409;
}

/**
 * Compute CoM and feet displacements
 * predictions when a continuous velocity
 * is used.
 *
 * @param p_velocityX
 * @param p_velocityY
 * @param p_angularVelocity
 * @param p_currentFeetConfiguration
 * @param p_predictions
 */
void Model::predictContinuousDisplacements(const double p_velocityX,
                                           const double p_velocityY,
                                           const double p_angularVelocity,
                                           const FeetConfiguration &p_currentFeetConfiguration,
                                           std::vector<double> &p_predictions) {
    // Common input to all models
    Eigen::VectorXd l_modelInput(12);
    l_modelInput << p_velocityX,
            p_velocityY,
            p_angularVelocity,
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
        p_predictions[0] = m_fr_rl_com_x_continuous * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y_continuous * l_modelInput;

        p_predictions[2] = m_fl_support_x_continuous * l_modelInput;
        p_predictions[3] = m_fl_support_y_continuous * l_modelInput;

        p_predictions[4] = m_fr_swinging_x_continuous * l_modelInput;
        p_predictions[5] = m_fr_swinging_y_continuous * l_modelInput;

        p_predictions[6] = m_rl_swinging_x_continuous * l_modelInput;
        p_predictions[7] = m_rl_swinging_y_continuous * l_modelInput;

        p_predictions[8] = m_rr_support_x_continuous * l_modelInput;
        p_predictions[9] = m_rr_support_y_continuous * l_modelInput;
    }
        // FL/RR are swinging
    else {
        p_predictions[0] = m_fl_rr_com_x_continuous * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y_continuous * l_modelInput;

        p_predictions[2] = m_fl_swinging_x_continuous * l_modelInput;
        p_predictions[3] = m_fl_swinging_y_continuous * l_modelInput;

        p_predictions[4] = m_fr_support_x_continuous * l_modelInput;
        p_predictions[5] = m_fr_support_y_continuous * l_modelInput;

        p_predictions[6] = m_rl_support_x_continuous * l_modelInput;
        p_predictions[7] = m_rl_support_y_continuous * l_modelInput;

        p_predictions[8] = m_rr_swinging_x_continuous * l_modelInput;
        p_predictions[9] = m_rr_swinging_y_continuous * l_modelInput;
    }
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
void Model::predictDiscontinuousDisplacements(double p_previousVelocityX,
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
        p_predictions[0] = m_fr_rl_com_x_acceleration * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y_acceleration * l_modelInput;

        p_predictions[2] = m_fl_support_x_acceleration * l_modelInput;
        p_predictions[3] = m_fl_support_y_acceleration * l_modelInput;

        p_predictions[4] = m_fr_swinging_x_acceleration * l_modelInput;
        p_predictions[5] = m_fr_swinging_y_acceleration * l_modelInput;

        p_predictions[6] = m_rl_swinging_x_acceleration * l_modelInput;
        p_predictions[7] = m_rl_swinging_y_acceleration * l_modelInput;

        p_predictions[8] = m_rr_support_x_acceleration * l_modelInput;
        p_predictions[9] = m_rr_support_y_acceleration * l_modelInput;
    }
        // FL/RR are swinging
    else {
        p_predictions[0] = m_fl_rr_com_x_acceleration * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y_acceleration * l_modelInput;

        p_predictions[2] = m_fl_swinging_x_acceleration * l_modelInput;
        p_predictions[3] = m_fl_swinging_y_acceleration * l_modelInput;

        p_predictions[4] = m_fr_support_x_acceleration * l_modelInput;
        p_predictions[5] = m_fr_support_y_acceleration * l_modelInput;

        p_predictions[6] = m_rl_support_x_acceleration * l_modelInput;
        p_predictions[7] = m_rl_support_y_acceleration * l_modelInput;

        p_predictions[8] = m_rr_swinging_x_acceleration * l_modelInput;
        p_predictions[9] = m_rr_swinging_y_acceleration * l_modelInput;
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
                          const World2D &p_currentWorldCoordinatesCoM,
                          World2D &p_newWorldCoordinatesCoM) {
    ROS_INFO_STREAM("Model: COM movement in x: " << p_predictedCoMDisplacementX);
    ROS_INFO_STREAM("Model: COM movement in y: " << p_predictedCoMDisplacementY);

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

    ROS_INFO_STREAM("Model: new CoM (" << p_newWorldCoordinatesCoM.x << ", "
                                       << p_newWorldCoordinatesCoM.y << ", "
                                       << p_newWorldCoordinatesCoM.q.x() << ","
                                       << p_newWorldCoordinatesCoM.q.y() << ","
                                       << p_newWorldCoordinatesCoM.q.z() << ","
                                       << p_newWorldCoordinatesCoM.q.w() << ")");
}

/**
 * Compute new CoM feet configuration
 * using the newly computed map feet
 * configuration and kinematic transforms.
 *
 * @param p_relativeStepPredictions
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeNewCoMFeetConfiguration(const std::vector<double> &p_relativeStepPredictions,
                                           const FeetConfiguration &p_currentFeetConfiguration,
                                           FeetConfiguration &p_newFeetConfiguration) {
    ROS_INFO_STREAM(
            "Model: Prev FL CoM: " << p_currentFeetConfiguration.flCoM.x << ", " << p_currentFeetConfiguration.flCoM.y);
    ROS_INFO_STREAM(
            "Model: Prev FR CoM: " << p_currentFeetConfiguration.frCoM.x << ", " << p_currentFeetConfiguration.frCoM.y);
    ROS_INFO_STREAM(
            "Model: Prev RL CoM: " << p_currentFeetConfiguration.rlCoM.x << ", " << p_currentFeetConfiguration.rlCoM.y);
    ROS_INFO_STREAM(
            "Model: Prev RR CoM: " << p_currentFeetConfiguration.rrCoM.x << ", " << p_currentFeetConfiguration.rrCoM.y
                                   << "\n");

    for (auto &prediction: p_relativeStepPredictions)
        ROS_INFO_STREAM(prediction);

    p_newFeetConfiguration.flCoM.x = p_currentFeetConfiguration.flCoM.x + p_relativeStepPredictions[0];
    p_newFeetConfiguration.flCoM.y = p_currentFeetConfiguration.flCoM.y + p_relativeStepPredictions[1];

    p_newFeetConfiguration.frCoM.x = p_currentFeetConfiguration.frCoM.x + p_relativeStepPredictions[2];
    p_newFeetConfiguration.frCoM.y = p_currentFeetConfiguration.frCoM.y + p_relativeStepPredictions[3];

    p_newFeetConfiguration.rlCoM.x = p_currentFeetConfiguration.rlCoM.x + p_relativeStepPredictions[4];
    p_newFeetConfiguration.rlCoM.y = p_currentFeetConfiguration.rlCoM.y + p_relativeStepPredictions[5];

    p_newFeetConfiguration.rrCoM.x = p_currentFeetConfiguration.rrCoM.x + p_relativeStepPredictions[6];
    p_newFeetConfiguration.rrCoM.y = p_currentFeetConfiguration.rrCoM.y + p_relativeStepPredictions[7];

    ROS_INFO_STREAM("Model: New FL CoM: " << p_newFeetConfiguration.flCoM.x << ", " << p_newFeetConfiguration.flCoM.y);
    ROS_INFO_STREAM("Model: New FR CoM: " << p_newFeetConfiguration.frCoM.x << ", " << p_newFeetConfiguration.frCoM.y);
    ROS_INFO_STREAM("Model: New RL CoM: " << p_newFeetConfiguration.rlCoM.x << ", " << p_newFeetConfiguration.rlCoM.y);
    ROS_INFO_STREAM(
            "Model: New RR CoM: " << p_newFeetConfiguration.rrCoM.x << ", " << p_newFeetConfiguration.rrCoM.y << "\n");
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
                             const World2D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World2D &p_newWorldCoordinatesCoM) {
    // Predict feet and CoM displacements
    std::vector<double> l_predictions(10);
    if (!p_accelerating) {
        predictContinuousDisplacements(p_action.x * p_previousVelocity,
                                       p_action.y * p_previousVelocity,
                                       p_action.theta * p_previousVelocity,
                                       p_currentFeetConfiguration,
                                       l_predictions);
    } else {
        predictDiscontinuousDisplacements(p_action.x * p_previousVelocity,
                                          p_action.y * p_previousVelocity,
                                          p_action.theta * p_previousVelocity,
                                          p_action.x * p_nextVelocity,
                                          p_action.y * p_nextVelocity,
                                          p_action.theta * p_nextVelocity,
                                          p_currentFeetConfiguration,
                                          l_predictions);
    }

    // Compute new CoM
    computeNewCoM(p_action.theta * p_nextVelocity,
                  l_predictions[0],
                  l_predictions[1],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    // Compute feet configuration w.r.t CoM
    std::vector<double> l_relativeStepPredictions = std::vector<double>(l_predictions.begin() + 2,
                                                                        l_predictions.end());
    computeNewCoMFeetConfiguration(l_relativeStepPredictions,
                                   p_currentFeetConfiguration,
                                   p_newFeetConfiguration);

    // Change swinging feet pair
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

//    int j = 0;
//
//    // Populate array
//    visualization_msgs::Marker l_footCommonMarker;
//    l_footCommonMarker.header.stamp = ros::Time::now();
//    l_footCommonMarker.header.frame_id = ROBOT_REFERENCE_FRAME;
//    l_footCommonMarker.type = 2;
//    l_footCommonMarker.action = 0;
//    l_footCommonMarker.lifetime = ros::Duration(4);
//    l_footCommonMarker.pose.orientation.x = 0;
//    l_footCommonMarker.pose.orientation.y = 0;
//    l_footCommonMarker.pose.orientation.z = 0;
//    l_footCommonMarker.pose.orientation.w = 1;
//    l_footCommonMarker.scale.x = 0.025;
//    l_footCommonMarker.scale.y = 0.025;
//    l_footCommonMarker.scale.z = 0.025;
//    l_footCommonMarker.color.r = 0;
//    l_footCommonMarker.color.g = 0;
//    l_footCommonMarker.color.b = 1;
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
//    l_flFootMarker.pose.position.x = p_newFeetConfiguration.flCoM.x;
//    l_flFootMarker.pose.position.y = p_newFeetConfiguration.flCoM.y;
//    l_flFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//    l_frFootMarker.id = j++;
//    l_frFootMarker.pose.position.x = p_newFeetConfiguration.frCoM.x;
//    l_frFootMarker.pose.position.y = p_newFeetConfiguration.frCoM.y;
//    l_frFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//    l_rlFootMarker.id = j++;
//    l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlCoM.x;
//    l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlCoM.y;
//    l_rlFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//    l_rrFootMarker.id = j++;
//    l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrCoM.x;
//    l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrCoM.y;
//    l_rrFootMarker.pose.position.z = 0;
//
//    // Feet configuration array
//    // for visualization purposes
//    visualization_msgs::MarkerArray l_pathFeetConfiguration;
//    l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
//    l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
//    l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);
//
//    m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
//    ros::Duration(5).sleep();
}