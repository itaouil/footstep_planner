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
    m_fr_rl_com_x_acceleration << -1.93964729e-02, -1.12171437e-01, 7.06644965e-02,
            6.08323129e-02, 1.04709658e-02, 5.30272157e-02,
            2.71691942e-01, 6.37621090e-01, 5.01156364e-01,
            1.44718259e-01, 3.47378691e-02, -6.86982300e-04,
            1.14837311e+00, 4.02676501e-01, 0.09564384;

    m_fr_rl_com_y_acceleration.resize(15);
    m_fr_rl_com_y_acceleration << -0.00592053, 0.15215433, 0.00607686, 0.01733032, 0.05046679,
            0.01816035, -0.05712642, 0.16449881, -0.28494664, 0.38181276,
            -0.0158065, 0.09346874, -0.22024254, -0.3861907, -0.02605762;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_acceleration.resize(15);
    m_fl_rr_com_x_acceleration << -0.01000357, 0.10757214, -0.01991805, 0.06448734, -0.00821719,
            -0.02003747, 0.32804888, -0.00679768, 0.36072621, -0.61838319,
            0.88124677, -0.40047104, 0.10637566, -0.16808212, 0.02116798;
    m_fl_rr_com_y_acceleration.resize(15);
    m_fl_rr_com_y_acceleration << 0.01979146, 0.16346679, 0.01619423, -0.01671036, 0.05192439,
            0.01997212, 0.13530027, 0.37660438, -0.11731445, 0.03297675,
            0.1438652, -0.43096139, -0.02188988, -0.0140944, 0.03798774;

    // FL models coeff. in swing and support mode
    m_fl_support_x_acceleration.resize(15);
    m_fl_support_x_acceleration << 0.02664234, 0.09187139, -0.01325849, -0.05866921, -0.00744852,
            -0.04222457, 0.64286252, -0.43778285, -0.52473253, -0.15294053,
            -0.16515301, 0.08211498, -1.16996154, -0.45498624, -0.16750551;
    m_fl_support_y_acceleration.resize(15);
    m_fl_support_y_acceleration << 0.00366699, -0.14955415, -0.09620165, -0.01551183, -0.04740276,
            -0.03234282, 0.11904886, 0.99749446, 0.19459777, -0.46219381,
            0.08415954, -0.00827589, 0.14635128, 0.25089739, -0.04835997;
    m_fl_swinging_x_acceleration.resize(15);
    m_fl_swinging_x_acceleration << -2.22884244e-02, 1.13254764e-02, -1.62243543e-02,
            1.06974286e-01, 2.36758565e-04, -2.33404311e-02,
            6.28706465e-01, 3.50090697e-02, -4.75645898e-02,
            8.07475029e-03, 3.68032469e-01, -8.72033015e-02,
            -2.41505957e-01, -9.40315292e-02, 0.12155964;
    m_fl_swinging_y_acceleration.resize(15);
    m_fl_swinging_y_acceleration << 6.85291180e-03, -2.90946544e-03, -9.06125287e-03,
            1.29771671e-04, 1.04063578e-01, 3.60916080e-02,
            7.41178291e-02, 5.29463450e-01, 5.53989832e-02,
            1.85001950e-01, -9.33171125e-02, -2.20296291e-02,
            -3.87311921e-02, -2.81624301e-02, 0.04780996;

    // FR models coeff. in swing and support mode
    m_fr_support_x_acceleration.resize(15);
    m_fr_support_x_acceleration << 5.08267902e-04, -8.11219666e-02, -2.22271444e-02,
            -5.98263816e-02, 5.50363497e-03, 2.22672730e-02,
            -3.58221987e-01, -3.25189283e-02, 8.54130746e-01,
            4.65646779e-01, -1.00328390e+00, 3.40848218e-01,
            -1.51644072e-01, 1.40990633e-01, -0.11990625;
    m_fr_support_y_acceleration.resize(15);
    m_fr_support_y_acceleration << -0.0198517, -0.1506612, -0.10744551, 0.01570289, -0.04883138,
            -0.03655234, -0.07477366, -0.51466563, 0.06841371, 0.99727533,
            -0.08124677, 0.25820718, -0.03537239, 0.07453805, 0.03006918;
    m_fr_swinging_x_acceleration.resize(15);
    m_fr_swinging_x_acceleration << -1.77342890e-02, -1.92437863e-02, 2.00825409e-02,
            1.04301690e-01, -9.75592780e-05, 2.86956911e-02,
            -1.00947419e-01, 1.03526252e-01, 6.05666259e-01,
            4.86980835e-03, -2.37079592e-01, 1.02157475e-01,
            3.79220580e-01, 9.30513629e-02, 0.13037527;
    m_fr_swinging_y_acceleration.resize(15);
    m_fr_swinging_y_acceleration << -7.50068422e-03, -5.28136542e-03, -6.85038858e-03,
            -2.50730732e-04, 1.05725009e-01, 4.03727461e-02,
            -6.62385974e-02, 2.41906600e-01, -6.84962908e-02,
            5.37589922e-01, 6.98159712e-02, -4.45746470e-02,
            1.31804812e-01, -5.36881644e-02, -0.04072177;

    // RL models coeff. in swing and support mode
    m_rl_support_x_acceleration.resize(15);
    m_rl_support_x_acceleration << 0.00690815, -0.09454265, 0.07238637, -0.06137528, 0.00314194,
            0.03069758, -0.4145223, -0.03434678, -0.48158826, 0.60152257,
            0.33679105, 0.25502551, 0.01344136, 0.12510454, 0.13124609;
    m_rl_support_y_acceleration.resize(15);
    m_rl_support_y_acceleration << -0.01821793, -0.15128806, 0.06779233, 0.0150989, -0.05224896,
            -0.01256037, -0.13137834, -0.36408493, 0.2548328, -0.2409816,
            -0.23592591, 1.54416878, 0.10601964, -0.02349228, -0.13680925;
    m_rl_swinging_x_acceleration.resize(15);
    m_rl_swinging_x_acceleration << -2.18450989e-02, -2.54404570e-02, 5.49528538e-02,
            1.05624061e-01, 3.52491756e-04, 1.79183797e-02,
            -1.61539686e-01, 2.00457563e-02, 1.34424793e-01,
            7.13177027e-02, 2.38673369e-01, 4.38534939e-02,
            4.79844544e-01, 2.57851818e-01, -0.01985522;
    m_rl_swinging_y_acceleration.resize(15);
    m_rl_swinging_y_acceleration << -0.00772958, -0.00228094, -0.00710435, 0.00050274, 0.1056016,
            -0.03909717, 0.19374105, 0.14851831, -0.07317351, -0.00656907,
            0.10253546, 0.49562984, -0.10257232, 0.02033087, 0.03512298;

    // RR models coeff. in swing and support mode
    m_rr_support_x_acceleration.resize(15);
    m_rr_support_x_acceleration << 0.02365693, 0.10700916, -0.11851855, -0.05915781, -0.00458782,
            -0.05675388, -0.56226076, -0.60863772, -0.58661376, -0.1776179,
            0.09462304, 0.06642716, 0.18033662, -0.40758409, 0.08047134;
    m_rr_support_y_acceleration.resize(15);
    m_rr_support_y_acceleration << 0.00603035, -0.15568447, 0.08580185, -0.01690808, -0.04716746,
            -0.01071013, -0.10946554, -0.33364832, 0.30759134, -0.31898429,
            -0.1424417, -0.09442176, 0.31128082, 1.65676234, 0.12918359;
    m_rr_swinging_x_acceleration.resize(15);
    m_rr_swinging_x_acceleration << -2.36824220e-02, 1.74989463e-02, -5.00957812e-02,
            1.07963817e-01, 1.49070318e-04, -1.44309833e-02,
            1.30939471e-01, -3.23235768e-02, -1.43906887e-01,
            4.25938702e-02, 4.66137285e-01, -2.09098106e-01,
            2.39600154e-01, -4.18026232e-02, -0.02977327;
    m_rr_swinging_y_acceleration.resize(15);
    m_rr_swinging_y_acceleration << 0.00472366, -0.00586961, -0.0036153, -0.00065418, 0.10493878,
            -0.03870301, 0.06425996, -0.02613534, -0.0974414, 0.14293829,
            0.07774154, 0.08868879, -0.02575887, 0.56707569, -0.04027396;
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
//    int j = 0;
//
//    // Populate array
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
//    l_footCommonMarker.scale.x = 0.025;
//    l_footCommonMarker.scale.y = 0.025;
//    l_footCommonMarker.scale.z = 0.025;
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
//    ros::Duration(1).sleep();
}