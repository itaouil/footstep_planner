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
    m_fr_rl_com_x_continuous << 0.17536781, -0.04131492,  0.00218651, -0.07236059,  0.2668249 ,
            0.22290188, -0.02830222,  0.23198114,  0.05932601,  0.07712831,
            0.1316227, 0.00154903;

    m_fr_rl_com_y_continuous.resize(12);
    m_fr_rl_com_y_continuous << -0.04572257,  0.20068902,  0.01017332,  0.30552661, -0.15529948,
            0.0836753 ,  0.25474342,  0.00997837,  0.15795191,  0.22584636,
            -0.03101185, 0.00458742;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_continuous.resize(12);
    m_fl_rr_com_x_continuous << 0.15902414,  0.03734992, -0.00299171,  0.29550171, -0.02330536,
            0.19624929, -0.18534149,  0.03383843, -0.13956962,  0.26221657,
            0.03271384, -0.03901015;
    m_fl_rr_com_y_continuous.resize(12);
    m_fl_rr_com_y_continuous << 0.02665927,  0.19336283, -0.00163659,  0.03275442,  0.2112821 ,
            -0.04061494,  0.00222519, -0.21797579, -0.03280542, -0.00706436,
            0.27743106, -0.03742866;

    // FL models coeff. in swing and support mode
    m_fl_support_x_continuous.resize(12);
    m_fl_support_x_continuous << -0.16855607,  0.02056143,  0.03495056,  1.09292353, -0.09606331,
            -0.27894017, -0.0215084 , -0.27517637,  0.03328144, -0.14124173,
            -0.151369, -0.07798482;
    m_fl_support_y_continuous.resize(12);
    m_fl_support_y_continuous << 0.03785324, -0.19199364, -0.07886687, -0.23944398,  1.29608797,
            -0.04527965, -0.28828396, -0.02606887, -0.09368942, -0.20213011,
            -0.086528, -0.08755975;
    m_fl_swinging_x_continuous.resize(12);
    m_fl_swinging_x_continuous << 0.12405419, -0.00814678,  0.01036141,  0.46618838,  0.01156951,
            0.02480616,  0.0638548 , -0.08431654,  0.04055149, -0.0558645 ,
            -0.01480057, 0.08057133;
    m_fl_swinging_y_continuous.resize(12);
    m_fl_swinging_y_continuous << -0.00241718,  0.11840397,  0.01379421,  0.02871768,  0.5210759 ,
            -0.04024635, -0.06530582,  0.08665785, -0.00794945, -0.01373164,
            -0.08494682, 0.07771529;

    // FR models coeff. in swing and support mode
    m_fr_support_x_continuous.resize(12);
    m_fr_support_x_continuous << -0.15243056, -0.01915305, -0.02990016, -0.35182319,  0.02063016,
            0.94044984,  0.06349555, -0.20061705,  0.13735888, -0.28678201,
            -0.06429562, -0.05319318;
    m_fr_support_y_continuous.resize(12);
    m_fr_support_y_continuous << -0.02305543, -0.18265849, -0.07244259, -0.09085519, -0.27420565,
            0.00386504,  1.07510858,  0.19487525, -0.05028124,  0.02274368,
            -0.20282514, 0.10846718;
    m_fr_swinging_x_continuous.resize(12);
    m_fr_swinging_x_continuous << 0.12291868,  0.00898828, -0.01023661,  0.05789888, -0.04679688,
            0.45930558,  0.01133571, -0.05920621,  0.01212515, -0.11896231,
            -0.05488521, 0.06490464;
    m_fr_swinging_y_continuous.resize(12);
    m_fr_swinging_y_continuous << 0.00496042,  0.12061548,  0.01271581,  0.02149121, -0.08268287,
            -0.03249606,  0.51704459,  0.01296578, -0.09011762, -0.09639259,
            -0.01404072, -0.07282254;

    // RL models coeff. in swing and support mode
    m_rl_support_x_continuous.resize(12);
    m_rl_support_x_continuous << -0.1493428 , -0.04161807,  0.04128778, -0.34883797, -0.01134854,
            -0.44225518,  0.23183138,  1.16215719,  0.13146067, -0.25257326,
            -0.0881017, 0.16577759;
    m_rl_support_y_continuous.resize(12);
    m_rl_support_y_continuous << -0.01101414, -0.19511967,  0.06305649,  0.06214208, -0.15252498,
            -0.05683755, -0.0686491 ,  0.16717731,  1.20351812, -0.04357099,
            -0.33661391, -0.04785475;
    m_rl_swinging_x_continuous.resize(12);
    m_rl_swinging_x_continuous << 0.11793057,  0.00351566,  0.00818061, -0.0358029 , -0.04771123,
            -0.0726172 ,  0.03936611,  0.45658993, -0.03300512,  0.0197121 ,
            -0.00281967, -0.08618269;
    m_rl_swinging_y_continuous.resize(12);
    m_rl_swinging_y_continuous << 0.00450395,  0.11833623, -0.01339103, -0.07565816, -0.04025677,
            0.02142083, -0.10258319, -0.05294764,  0.5033058 ,  0.011871  ,
            -0.06316322, 0.06612192;

    // RR models coeff. in swing and support mode
    m_rr_support_x_continuous.resize(12);
    m_rr_support_x_continuous << -0.16061984,  0.03892103, -0.03515258, -0.40076567, -0.17086406,
            -0.2968109 , -0.01242232, -0.23393719,  0.10483421,  1.2716804 ,
            -0.16650155, 0.15376922;
    m_rr_support_y_continuous.resize(12);
    m_rr_support_y_continuous << 0.02591294, -0.2001021 ,  0.05064151, -0.1620021 , -0.08637795,
            -0.11233887, -0.20289319, -0.00615133, -0.23651586, -0.16199965,
            1.31382763, 0.09683165;
    m_rr_swinging_x_continuous.resize(12);
    m_rr_swinging_x_continuous << 0.1250222 , -0.00469259, -0.00888703, -0.09481968, -0.05368231,
            -0.05742682,  0.04825628, -0.02973203,  0.02171032,  0.46250241,
            0.06154079, -0.08349129;
    m_rr_swinging_y_continuous.resize(12);
    m_rr_swinging_y_continuous << -0.00394947,  0.11919255, -0.01565597, -0.02998044, -0.06601121,
            -0.01510047, -0.06574916,  0.05705113, -0.04226378,  0.03623258,
            0.46541475, -0.0503018;
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
    m_fr_rl_com_x_acceleration << -0.00785711, -0.09023743,  0.08701676,  0.06037753,  0.00483609,
            0.05642675,  0.08561308,  0.36527627,  0.41892593,  0.1820427 ,
            -0.2085264 , -0.05150759,  1.05555253,  0.39668034, 0.13359696;

    m_fr_rl_com_y_acceleration.resize(15);
    m_fr_rl_com_y_acceleration << -0.00539496,  0.14468128,  0.03597647,  0.01815237,  0.05197553,
            0.04127224, -0.0077449 ,  0.28577389, -0.20428162,  0.56630772,
            -0.02588052, -0.05371182, -0.23978806, -0.39057983, -0.02957964;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_acceleration.resize(15);
    m_fl_rr_com_x_acceleration << -0.01743247,  0.11228587, -0.05574868,  0.06494701, -0.00339751,
            -0.04923299,  0.24693141, -0.11797683,  0.06431632, -0.79792938,
            1.05317483, -0.3395625 , -0.19203325, -0.2166048, 0.0474062;
    m_fl_rr_com_y_acceleration.resize(15);
    m_fl_rr_com_y_acceleration << 0.00108863,  0.16256858,  0.04552127, -0.01591355,  0.05523767,
            0.03915911,  0.09090441,  0.55351979, -0.04822119,  0.0475149 ,
            0.28133229, -0.41184673,  0.06434356, -0.14449791, 0.03584066;

    // FL models coeff. in swing and support mode
    m_fl_support_x_acceleration.resize(15);
    m_fl_support_x_acceleration << 0.01413195,  0.08612233, -0.03156993, -0.05643011, -0.00413237,
            -0.0477759 ,  0.92487447, -0.29943536, -0.44119994, -0.20247431,
            0.12392624,  0.0290928 , -1.14179652, -0.55357766, -0.21199407;
    m_fl_support_y_acceleration.resize(15);
    m_fl_support_y_acceleration << 0.00204702, -0.14732721, -0.11998248, -0.016389  , -0.04688762,
            -0.05128605,  0.1281158 ,  0.8935781 ,  0.16080997, -0.5793612 ,
            0.1061211 ,  0.10422217,  0.16393235,  0.32489343, -0.04112222;
    m_fl_swinging_x_acceleration.resize(15);
    m_fl_swinging_x_acceleration << -0.01703928,  0.01609551, -0.04254703,  0.10599798,  0.00264294,
            -0.04259171,  0.63166733,  0.04994194, -0.44223554, -0.05830288,
            0.58786427, -0.10639846, -0.48353674, -0.21883149, 0.17541917;
    m_fl_swinging_y_acceleration.resize(15);
    m_fl_swinging_y_acceleration << 8.12269578e-03, -4.76045577e-03, -1.69002686e-02,
            4.69447911e-04,  1.03681944e-01,  3.32498926e-02,
            1.07024308e-01,  5.31917355e-01,  5.48521613e-03,
            2.40008829e-01, -6.67322679e-02, -3.82445142e-02,
            -9.18447123e-02, -2.59732341e-02, 0.05695342;

    // FR models coeff. in swing and support mode
    m_fr_support_x_acceleration.resize(15);
    m_fr_support_x_acceleration << 2.36603661e-02, -1.03297365e-01,  1.79849475e-02,
            -6.22484030e-02,  5.38636771e-04,  5.60915673e-02,
            -2.87922110e-01,  1.72286193e-01,  1.05166359e+00,
            7.80710636e-01, -1.23333434e+00,  4.13052896e-01,
            1.48662844e-01,  1.97562891e-01, -0.14907352;
    m_fr_support_y_acceleration.resize(15);
    m_fr_support_y_acceleration << -0.00305781, -0.15694505, -0.12464004,  0.01443477, -0.04971531,
            -0.04833134, -0.0471067 , -0.55646253, -0.07183681,  1.04271499,
            -0.13523389,  0.29609147, -0.11177883,  0.13035228, 0.04100676;
    m_fr_swinging_x_acceleration.resize(15);
    m_fr_swinging_x_acceleration << -0.013249  , -0.0144195 ,  0.05134892,  0.10626237, -0.00295817,
            0.04829771, -0.39201259,  0.03059673,  0.64178689,  0.03111159,
            -0.49453302,  0.12753447,  0.50052369,  0.1045202, 0.17001006;
    m_fr_swinging_y_acceleration.resize(15);
    m_fr_swinging_y_acceleration << -0.0081086 , -0.00645843, -0.01711352, -0.00078024,  0.10585147,
            0.03462222, -0.01911265,  0.26549494, -0.10174776,  0.52649049,
            0.10959985, -0.02400311,  0.09469579, -0.0440842, -0.05078439;

    // RL models coeff. in swing and support mode
    m_rl_support_x_acceleration.resize(15);
    m_rl_support_x_acceleration << 0.02002966, -0.09324606,  0.11996399, -0.06139114, -0.00280576,
            0.06880636, -0.38722816,  0.09341646, -0.22300685,  0.74894707,
            0.12816071,  0.1159985 ,  0.33208778,  0.10041595, 0.11470788;
    m_rl_support_y_acceleration.resize(15);
    m_rl_support_y_acceleration << -0.00459903, -0.14690234,  0.05629508,  0.01335583, -0.05174463,
            -0.02012723, -0.11997766, -0.45331667,  0.16445858, -0.2877582 ,
            -0.3045838 ,  1.45304342,  0.0443908 , -0.01489819, -0.1271132;
    m_rl_swinging_x_acceleration.resize(15);
    m_rl_swinging_x_acceleration << -0.01056211, -0.00983047,  0.08571666,  0.10554572, -0.00280913,
            0.03371477, -0.43762615, -0.17444098,  0.17436879,  0.0872416 ,
            -0.01024543,  0.02233514,  0.53929999,  0.23730463, 0.02352394;
    m_rl_swinging_y_acceleration.resize(15);
    m_rl_swinging_y_acceleration << -0.00546569,  0.00596224, -0.00810772,  0.00051411,  0.10442282,
            -0.03892197,  0.14531469,  0.11692772, -0.09356965, -0.04900882,
            0.10093611,  0.49464425, -0.09011943, -0.06210744, 0.03828725;

    // RR models coeff. in swing and support mode
    m_rr_support_x_acceleration.resize(15);
    m_rr_support_x_acceleration << 0.00333999,  0.08189158, -0.14699477, -0.05690816,  0.00142537,
            -0.07132747, -0.24129946, -0.35862363, -0.51662372, -0.21752103,
            0.39883625,  0.12311861,  0.24097318, -0.32811224, 0.03582277;
    m_rr_support_y_acceleration.resize(15);
    m_rr_support_y_acceleration << 0.00590366, -0.13765214,  0.05875166, -0.0162252 , -0.04997358,
            -0.02792623, -0.13564526, -0.48922649,  0.22697251, -0.49992638,
            -0.08883756, -0.02840549,  0.32323241,  1.5314414, 0.13431729;
    m_rr_swinging_x_acceleration.resize(15);
    m_rr_swinging_x_acceleration << -0.02134778,  0.00962076, -0.07616273,  0.10682787,  0.0029621 ,
            -0.03129208,  0.15666368,  0.02096288, -0.47364619,  0.11515189,
            0.67809416, -0.14882832,  0.00914182, -0.1072847, 0.01804047;
    m_rr_swinging_y_acceleration.resize(15);
    m_rr_swinging_y_acceleration << 6.82458104e-03,  5.21840104e-03,  7.22861576e-05,
            -6.67782379e-04,  1.03584182e-01, -3.47429104e-02,
            4.59899071e-02, -8.26286824e-02, -7.09404410e-02,
            7.62669983e-02,  2.11159748e-02, -4.99364815e-02,
            -3.29193382e-02,  5.25882377e-01, -0.04305125;
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
 * @param p_relativeStepPredictions
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeNewCoMFeetConfiguration(const std::vector<double> &p_relativeStepPredictions,
                                           const FeetConfiguration &p_currentFeetConfiguration,
                                           FeetConfiguration &p_newFeetConfiguration) {
    p_newFeetConfiguration.flCoM.x = p_relativeStepPredictions[0];
    p_newFeetConfiguration.flCoM.y = p_relativeStepPredictions[1];

    p_newFeetConfiguration.frCoM.x = p_relativeStepPredictions[2];
    p_newFeetConfiguration.frCoM.y = p_relativeStepPredictions[3];

    p_newFeetConfiguration.rlCoM.x = p_relativeStepPredictions[4];
    p_newFeetConfiguration.rlCoM.y = p_relativeStepPredictions[5];

    p_newFeetConfiguration.rrCoM.x = p_relativeStepPredictions[6];
    p_newFeetConfiguration.rrCoM.y = p_relativeStepPredictions[7];

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
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
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
    int j = 0;

    // Populate array
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
    l_footCommonMarker.scale.x = 0.035;
    l_footCommonMarker.scale.y = 0.035;
    l_footCommonMarker.scale.z = 0.035;
    l_footCommonMarker.color.r = 0;
    l_footCommonMarker.color.g = 1;
    l_footCommonMarker.color.b = 0;
    l_footCommonMarker.color.a = 0.5;

    visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
    l_CoMMarker.id = j++;
    l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
    l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
    l_CoMMarker.pose.position.z = 0;

    visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
    l_flFootMarker.id = j++;
    l_flFootMarker.pose.position.x = p_newWorldCoordinatesCoM.x + p_newFeetConfiguration.flCoM.x;
    l_flFootMarker.pose.position.y = p_newWorldCoordinatesCoM.y + p_newFeetConfiguration.flCoM.y;
    l_flFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
    l_frFootMarker.id = j++;
    l_frFootMarker.pose.position.x = p_newWorldCoordinatesCoM.x + p_newFeetConfiguration.frCoM.x;
    l_frFootMarker.pose.position.y = p_newWorldCoordinatesCoM.y + p_newFeetConfiguration.frCoM.y;
    l_frFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
    l_rlFootMarker.id = j++;
    l_rlFootMarker.pose.position.x = p_newWorldCoordinatesCoM.x + p_newFeetConfiguration.rlCoM.x;
    l_rlFootMarker.pose.position.y = p_newWorldCoordinatesCoM.y + p_newFeetConfiguration.rlCoM.y;
    l_rlFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
    l_rrFootMarker.id = j++;
    l_rrFootMarker.pose.position.x = p_newWorldCoordinatesCoM.x + p_newFeetConfiguration.rrCoM.x;
    l_rrFootMarker.pose.position.y = p_newWorldCoordinatesCoM.y + p_newFeetConfiguration.rrCoM.y;
    l_rrFootMarker.pose.position.z = 0;

    // Feet configuration array
    // for visualization purposes
    visualization_msgs::MarkerArray l_pathFeetConfiguration;
    l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
    l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

    m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
    ros::Duration(2).sleep();
}