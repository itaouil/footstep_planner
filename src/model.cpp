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

    // Set continuous models coefficients
    setContinuousModelsCoefficients();

    // Set discontinuous models coefficients
    setDiscontinuousModelsCoefficients();
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
    m_fr_rl_com_x_continuous << 0.20808512, -0.02581947, -0.02910612, 0.07863871, 0.28301644,
            0.1515676, 0.03030656, 0.18986894, -0.04861574, -0.44381963,
            -0.02644216, -0.16344107;

    m_fr_rl_com_y_continuous.resize(12);
    m_fr_rl_com_y_continuous << -0.03492344, 0.16972226, 0.16547646, 0.07760413, 0.07543283,
            -0.00876985, 0.26355218, -0.12485505, 0.37117467, 0.2201663,
            0.16547018, 0.002223;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_continuous.resize(12);
    m_fl_rr_com_x_continuous << 0.20376512, 0.02402254, 0.0052527, 0.11267386, -0.07684928,
            -0.02902223, -0.24131625, -0.3580533, 0.03526293, 0.05176114,
            0.04349473, -0.12857186;
    m_fl_rr_com_y_continuous.resize(12);
    m_fl_rr_com_y_continuous << 0.05278749, 0.15672529, 0.14916124, -0.08732666, 0.16653137,
            -0.18461004, 0.0219723, -0.36701152, 0.26234807, 0.08219048,
            0.33983785, -0.01893889;

    // FL models coeff. in swing and support mode
    m_fl_support_x_continuous.resize(12);
    m_fl_support_x_continuous << -0.17651039, 0.03013708, 0.07733236, 1.03052492, -0.0703499,
            -0.20901254, -0.01897074, -0.25004334, 0.13221708, 0.03223032,
            -0.26693416, -0.06190629;
    m_fl_support_y_continuous.resize(12);
    m_fl_support_y_continuous << 0.03991046, -0.14231918, -0.21304368, 0.02344102, 0.87466979,
            0.13059303, -0.3834979, 0.11311654, -0.36504968, -0.32745468,
            -0.38018226, -0.13197438;
    m_fl_swinging_x_continuous.resize(12);
    m_fl_swinging_x_continuous << 0.10674814, -0.0013285, 0.00662535, 0.58076219, 0.09185988,
            0.10093299, 0.08806923, 0.09357027, 0.02328048, -0.04570679,
            -0.05767324, 0.06784542;
    m_fl_swinging_y_continuous.resize(12);
    m_fl_swinging_y_continuous << 0.0036456, 0.1148106, 0.12135236, 0.05764311, 0.46313939,
            -0.03950129, -0.00831456, -0.01385125, -0.04830008, -0.10231123,
            -0.09485336, 0.04872821;

    // FR models coeff. in swing and support mode
    m_fr_support_x_continuous.resize(12);
    m_fr_support_x_continuous << -0.17421352, -0.02060152, -0.06365695, -0.12200267, 0.06006506,
            0.98895833, 0.06447814, 0.07287175, 0.11170314, -0.27200883,
            -0.2149132, -0.06385854;
    m_fr_support_y_continuous.resize(12);
    m_fr_support_y_continuous << -0.05096832, -0.14180601, -0.20164394, -0.07953319, -0.29616777,
            0.12357593, 0.92647317, 0.37252178, -0.33056021, 0.00877083,
            -0.28121157, 0.13095961;
    m_fr_swinging_x_continuous.resize(12);
    m_fr_swinging_x_continuous << 0.10224215, 0.00774358, -0.00120814, 0.21853816, -0.12053275,
            0.56729092, -0.08996999, 0.02883841, -0.04427537, 0.05200231,
            -0.12888338, 0.05733808;
    m_fr_swinging_y_continuous.resize(12);
    m_fr_swinging_y_continuous << -0.00738562, 0.11558097, 0.12014261, 0.00913514, -0.04172134,
            -0.05418969, 0.43776464, 0.05206659, -0.04486018, 0.05278695,
            -0.0225549, -0.04631393;

    // RL models coeff. in swing and support mode
    m_rl_support_x_continuous.resize(12);
    m_rl_support_x_continuous << -0.19203141, -0.01645394, 0.03245595, -0.22479601, 0.04398352,
            -0.1013794, 0.27216886, 1.40037568, -0.0350885, -0.16770556,
            -0.04384422, 0.17526543;
    m_rl_support_y_continuous.resize(12);
    m_rl_support_y_continuous << -0.03331607, -0.16079069, -0.07397845, 0.21649395, -0.02441392,
            0.12345368, 0.05353666, 0.29766462, 0.86316614, -0.04339697,
            -0.32767008, -0.036162;
    m_rl_swinging_x_continuous.resize(12);
    m_rl_swinging_x_continuous << 0.10428485, 0.00510955, 0.0133214, 0.04817634, -0.02860632,
            -0.02405465, 0.01222853, 0.4919974, 0.00710649, 0.11687032,
            -0.08452207, -0.11274731;
    m_rl_swinging_y_continuous.resize(12);
    m_rl_swinging_y_continuous << 0.00923259, 0.12084786, 0.10030024, -0.01137223, 0.00521201,
            -0.02223453, -0.03500909, -0.01460259, 0.48887365, -0.09528546,
            -0.06372836, 0.0496509;

    // RR models coeff. in swing and support mode
    m_rr_support_x_continuous.resize(12);
    m_rr_support_x_continuous << -0.18997314, 0.03031421, -0.01026311, -0.18253901, -0.29307505,
            -0.19157483, -0.05203472, -0.19027302, 0.0773284, 1.43398481,
            -0.06544242, 0.17159577;
    m_rr_support_y_continuous.resize(12);
    m_rr_support_y_continuous << 0.03012836, -0.16934121, -0.09709706, -0.09489731, 0.00867749,
            -0.09187119, -0.14898529, 0.11455918, -0.29917055, -0.21759606,
            0.96996332, 0.0396129;
    m_rr_swinging_x_continuous.resize(12);
    m_rr_swinging_x_continuous << 0.10496364, -0.00202884, -0.00879802, -0.04560381, -0.04545276,
            0.07835627, -0.01284119, 0.0848572, 0.064494, 0.50086051,
            0.00811918, -0.11632034;
    m_rr_swinging_y_continuous.resize(12);
    m_rr_swinging_y_continuous << -0.00646717, 0.11814641, 0.10963655, 0.00454754, -0.02294627,
            0.06139013, 0.028314, 0.02817904, -0.05078295, 0.05669689,
            0.50267557, -0.06097956;
}

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of discontinuous
 * velocity commands.
 */
void Model::setDiscontinuousModelsCoefficients() {
    // CoM models coeff. when FR/RL swinging
    m_fr_rl_com_x_acceleration.resize(15);
    m_fr_rl_com_x_acceleration << 0.11426061, -0.03405834, -0.03619446, 0.05781373, 0.01151038,
            0.00568989, -0.0182129, 0.11413612, 0.09420014, 0.00213572,
            0.06405862, 0.06451047, -0.04711094, 0.09681007, -0.03377802;

    m_fr_rl_com_y_acceleration.resize(15);
    m_fr_rl_com_y_acceleration << -0.03994327, 0.13418997, 0.16830004, 0.01441762, 0.0439992,
            0.07297119, 0.00563244, -0.14518692, 0.21285505, 0.10113672,
            -0.2415017, 0.0470098, 0.23703236, -0.05494157, -0.0326304;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_acceleration.resize(15);
    m_fl_rr_com_x_acceleration << 0.11224987, 0.0415022, 0.04151491, 0.05764547, -0.01221991,
            -0.01842971, 0.05485544, -0.12854915, 0.10543137, -0.29915887,
            -0.16858387, -0.02726846, 0.08331259, 0.01020945, -0.08524986;
    m_fl_rr_com_y_acceleration.resize(15);
    m_fl_rr_com_y_acceleration << 0.06754555, 0.09836172, 0.10635071, -0.01552698, 0.04425489,
            0.05699594, -0.01419293, 0.25733514, -0.35208047, 0.23917549,
            -0.14241405, 0.08402686, 0.07386131, 0.07169628, 0.06500263;

    // FL models coeff. in swing and support mode
    m_fl_support_x_acceleration.resize(15);
    m_fl_support_x_acceleration << -1.15015295e-01, 3.24657245e-02, 9.52924599e-02,
            -5.39024336e-02, -1.09779716e-02, -2.97671042e-04,
            1.12510517e+00, -3.31915912e-02, -8.95729294e-02,
            -3.20528128e-02, -1.64754502e-01, -5.69553242e-02,
            -4.17698944e-02, -3.18195229e-01, -0.09664163;
    m_fl_support_y_acceleration.resize(15);
    m_fl_support_y_acceleration << 0.04995752, -0.0987363, -0.23260524, -0.01549772, -0.04251912,
            -0.09329067, 0.17188731, 0.94751833, -0.18286748, -0.26800173,
            0.23829437, 0.00532128, -0.49411737, -0.11208828, -0.11110893;
    m_fl_swinging_x_acceleration.resize(15);
    m_fl_swinging_x_acceleration << 0.006528, 0.0046194, 0.02301247, 0.10490247, 0.00046301,
            -0.01285304, 0.40300386, 0.07621529, -0.0958813, -0.17937189,
            0.12981206, 0.14650048, -0.11164553, -0.17192212, 0.06574568;
    m_fl_swinging_y_acceleration.resize(15);
    m_fl_swinging_y_acceleration << 6.25529627e-02, -4.22266556e-02, -1.16588822e-02,
            -3.04496871e-04, 1.00475785e-01, 1.93271910e-01,
            -1.82795236e-01, 6.16651332e-01, 1.32208086e-01,
            5.65309479e-01, -8.01999623e-01, 1.02191330e-01,
            3.23936008e-02, -6.69171292e-03, -0.04032747;

    // FR models coeff. in swing and support mode
    m_fr_support_x_acceleration.resize(15);
    m_fr_support_x_acceleration << -0.05773981, -0.03984761, -0.11861952, -0.05491489, 0.01002214,
            -0.00448269, -0.00990044, 0.14119325, 0.48693205, 0.13810112,
            -0.01726105, 0.19254457, -0.40899073, -0.19990415, -0.04675244;
    m_fr_support_y_acceleration.resize(15);
    m_fr_support_y_acceleration << -0.05038413, -0.08937018, -0.23462848, 0.01629998, -0.04161873,
            -0.10941222, 0.19535299, -0.30088353, -0.42589709, 0.76236709,
            0.68386896, -0.15606286, -0.39135357, -0.15292395, 0.13589433;
    m_fr_swinging_x_acceleration.resize(15);
    m_fr_swinging_x_acceleration << -3.04662652e-02, -2.01780294e-02, -3.60608891e-02,
            1.05837371e-01, -2.43627530e-04, 1.04990928e-02,
            2.94494649e-01, 1.64341655e-01, 4.77766266e-01,
            -8.98053280e-03, 2.12490229e-03, 3.75134579e-03,
            1.70991790e-01, -4.15584271e-02, 0.05537526;
    m_fr_swinging_y_acceleration.resize(15);
    m_fr_swinging_y_acceleration << -0.01600623, -0.0119403, 0.02402487, -0.00370003, 0.10287376,
            0.16828384, -0.20556818, -0.04497095, 0.3946536, 0.43983482,
            -0.12478522, -0.04975529, 0.51032921, 0.24197165, 0.01449789;

    // RL models coeff. in swing and support mode
    m_rl_support_x_acceleration.resize(15);
    m_rl_support_x_acceleration << -0.07620983, -0.0341861, -0.00438852, -0.05609396, 0.00913938,
            0.00146622, -0.06038432, 0.09222846, -0.51693145, 0.34303929,
            1.22377413, -0.09177112, -0.26146271, -0.10729221, 0.16734992;
    m_rl_support_y_acceleration.resize(15);
    m_rl_support_y_acceleration << -0.05267806, -0.09952309, -0.05728778, 0.01415518, -0.04145141,
            -0.08566804, 0.18837529, -0.14181614, -0.1296674, -0.24430783,
            0.4420524, 0.98513901, -0.331408, -0.23859042, -0.04233963;
    m_rl_swinging_x_acceleration.resize(15);
    m_rl_swinging_x_acceleration << -0.02131954, -0.01301363, 0.00061233, 0.10434056, 0.00142237,
            -0.00327612, 0.3480368, 0.13503034, -0.08275293, 0.11469235,
            0.51505229, -0.11701941, 0.01337772, -0.08347077, -0.17891217;
    m_rl_swinging_y_acceleration.resize(15);
    m_rl_swinging_y_acceleration << -4.25874083e-02, -2.52492862e-02, 8.93343974e-03,
            1.57694478e-03, 1.03524826e-01, 1.04203854e-01,
            -8.44407531e-03, 4.39877842e-01, 7.95378935e-02,
            2.58705189e-02, 3.04624338e-02, 5.29652493e-01,
            4.62647499e-01, -5.92108362e-05, 0.11571438;

    // RR models coeff. in swing and support mode
    m_rr_support_x_acceleration.resize(15);
    m_rr_support_x_acceleration << -8.02975181e-02, 5.96996062e-02, 7.02989439e-04,
            -5.68963261e-02, -9.02636145e-03, -1.86272473e-02,
            -7.61734166e-02, -3.82986938e-01, -2.07444317e-01,
            -7.85172766e-02, -7.94866494e-02, -7.38830340e-02,
            8.35878752e-01, -2.34113679e-01, 0.0356956;
    m_rr_support_y_acceleration.resize(15);
    m_rr_support_y_acceleration << 0.04737254, -0.11166211, -0.04595521, -0.01199884, -0.04227015,
            -0.06752972, -0.38007728, -0.18567893, -0.09219618, -0.19857315,
            0.13398683, -0.00276322, 0.0720705, 1.16785618, 0.19147109;
    m_rr_swinging_x_acceleration.resize(15);
    m_rr_swinging_x_acceleration << 0.00297236, 0.0010635, -0.01327839, 0.10505812, -0.00129393,
            0.00127863, -0.10770145, -0.02041676, -0.03315383, -0.07958078,
            0.10390234, 0.11916685, 0.42787035, -0.02966768, -0.12096103;
    m_rr_swinging_y_acceleration.resize(15);
    m_rr_swinging_y_acceleration << 0.02214655, -0.02275158, 0.03210138, -0.00209331, 0.10192995,
            0.12314313, -0.13457132, 0.08182237, 0.28536417, 0.48785319,
            -0.47519191, 0.01509147, 0.17235955, 0.60274951, -0.11290733;
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
        ROS_INFO_STREAM("Predicting FR/RL swinging");
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
        ROS_INFO_STREAM("Predicting FL/RR swinging");
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
                          const World3D &p_currentWorldCoordinatesCoM,
                          World3D &p_newWorldCoordinatesCoM) {
//    ROS_INFO_STREAM("Model: COM movement in x: " << p_predictedCoMDisplacementX);
//    ROS_INFO_STREAM("Model: COM movement in y: " << p_predictedCoMDisplacementY);

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

//    ROS_INFO_STREAM("Model: new CoM (" << p_newWorldCoordinatesCoM.x << ", "
//                                       << p_newWorldCoordinatesCoM.y << ", "
//                                       << p_newWorldCoordinatesCoM.q.x() << ","
//                                       << p_newWorldCoordinatesCoM.q.y() << ","
//                                       << p_newWorldCoordinatesCoM.q.z() << ","
//                                       << p_newWorldCoordinatesCoM.q.w() << ")");

//    ROS_INFO_STREAM("Model: new CoM (" << p_newWorldCoordinatesCoM.x << ", "
//                                       << p_newWorldCoordinatesCoM.y << ")");
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
//    ROS_INFO_STREAM(
//            "Model: Prev FL CoM: " << p_currentFeetConfiguration.flCoM.x << ", " << p_currentFeetConfiguration.flCoM.y);
//    ROS_INFO_STREAM(
//            "Model: Prev FR CoM: " << p_currentFeetConfiguration.frCoM.x << ", " << p_currentFeetConfiguration.frCoM.y);
//    ROS_INFO_STREAM(
//            "Model: Prev RL CoM: " << p_currentFeetConfiguration.rlCoM.x << ", " << p_currentFeetConfiguration.rlCoM.y);
//    ROS_INFO_STREAM(
//            "Model: Prev RR CoM: " << p_currentFeetConfiguration.rrCoM.x << ", " << p_currentFeetConfiguration.rrCoM.y
//                                   << "\n");

//    for (auto &prediction: p_relativeStepPredictions)
//        ROS_INFO_STREAM(prediction);

    p_newFeetConfiguration.flCoM.x = p_relativeStepPredictions[0];
    p_newFeetConfiguration.flCoM.y = p_relativeStepPredictions[1];

    p_newFeetConfiguration.frCoM.x = p_relativeStepPredictions[2];
    p_newFeetConfiguration.frCoM.y = p_relativeStepPredictions[3];

    p_newFeetConfiguration.rlCoM.x = p_relativeStepPredictions[4];
    p_newFeetConfiguration.rlCoM.y = p_relativeStepPredictions[5];

    p_newFeetConfiguration.rrCoM.x = p_relativeStepPredictions[6];
    p_newFeetConfiguration.rrCoM.y = p_relativeStepPredictions[7];

    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

//    ROS_INFO_STREAM("Model: New FL CoM: " << p_newFeetConfiguration.flCoM.x << ", " << p_newFeetConfiguration.flCoM.y);
//    ROS_INFO_STREAM("Model: New FR CoM: " << p_newFeetConfiguration.frCoM.x << ", " << p_newFeetConfiguration.frCoM.y);
//    ROS_INFO_STREAM("Model: New RL CoM: " << p_newFeetConfiguration.rlCoM.x << ", " << p_newFeetConfiguration.rlCoM.y);
//    ROS_INFO_STREAM(
//            "Model: New RR CoM: " << p_newFeetConfiguration.rrCoM.x << ", " << p_newFeetConfiguration.rrCoM.y << "\n");
//    ROS_INFO_STREAM("Swinging foot: " << p_currentFeetConfiguration.fr_rl_swinging);
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
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    ROS_INFO_STREAM("Next Velocity: " << p_action.x * p_nextVelocity << ", " << p_action.y * p_nextVelocity << ", "
                                      << p_action.theta * p_nextVelocity);
    ROS_INFO_STREAM(
            "Previous Velocity: " << p_action.x * p_previousVelocity << ", " << p_action.y * p_previousVelocity << ", "
                                  << p_action.theta * p_previousVelocity);

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

    // Compute new feet poses w.r.t to CoM
    std::vector<double> l_relativeStepPredictions = std::vector<double>(l_predictions.begin() + 2,
                                                                        l_predictions.end());
    computeNewCoMFeetConfiguration(l_relativeStepPredictions,
                                   p_currentFeetConfiguration,
                                   p_newFeetConfiguration);

    // Change swinging feet pair
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

    // Publish predicted footstep and CoM
//    int j = 0;
//
//    // Populate array
//    visualization_msgs::Marker l_footCommonMarker;
//    l_footCommonMarker.header.stamp = ros::Time::now();
//    l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//    l_footCommonMarker.type = 2;
//    l_footCommonMarker.action = 0;
//    l_footCommonMarker.lifetime = ros::Duration(0.3);
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
//    l_flFootMarker.pose.position.x = p_newWorldCoordinatesCoM.x + p_newFeetConfiguration.flCoM.x;
//    l_flFootMarker.pose.position.y = p_newWorldCoordinatesCoM.y + p_newFeetConfiguration.flCoM.y;
//    l_flFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//    l_frFootMarker.id = j++;
//    l_frFootMarker.pose.position.x = p_newWorldCoordinatesCoM.x + p_newFeetConfiguration.frCoM.x;
//    l_frFootMarker.pose.position.y = p_newWorldCoordinatesCoM.y + p_newFeetConfiguration.frCoM.y;
//    l_frFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//    l_rlFootMarker.id = j++;
//    l_rlFootMarker.pose.position.x = p_newWorldCoordinatesCoM.x + p_newFeetConfiguration.rlCoM.x;
//    l_rlFootMarker.pose.position.y = p_newWorldCoordinatesCoM.y + p_newFeetConfiguration.rlCoM.y;
//    l_rlFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//    l_rrFootMarker.id = j++;
//    l_rrFootMarker.pose.position.x = p_newWorldCoordinatesCoM.x + p_newFeetConfiguration.rrCoM.x;
//    l_rrFootMarker.pose.position.y = p_newWorldCoordinatesCoM.y + p_newFeetConfiguration.rrCoM.y;
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
//    ros::Duration(0.4).sleep();
}