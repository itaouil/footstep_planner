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
    m_fr_rl_com_x_continuous << 0.21674366, -0.03172167, -0.02416405, -0.15187736,  0.13316871,
            0.25660025, -0.01237345, -0.01050544,  0.05378222, -0.32570118,
            0.22634539, -0.11286262;

    m_fr_rl_com_y_continuous.resize(12);
    m_fr_rl_com_y_continuous << -0.01063686,  0.20068636,  0.19874541,  0.05406989, -0.34364412,
            0.11186589,  0.04677536,  0.03605651,  0.13799564,  0.13780997,
            -0.07374109, 0.03068899;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_continuous.resize(12);
    m_fl_rr_com_x_continuous << 0.20390175,  0.02400126,  0.00184282,  0.3131887 ,  0.01826099,
            -0.27270429,  0.02907303, -0.14502937, -0.33615819, -0.20625615,
            -0.16849949, -0.07202636;
    m_fl_rr_com_y_continuous.resize(12);
    m_fl_rr_com_y_continuous << 0.03193012,  0.18186303,  0.19440702, -0.01823795,  0.2826519 ,
            -0.44162686, -0.08450152,  0.04250324, -0.07350705, -0.08591253,
            -0.05687738, 0.0401159;

    // FL models coeff. in swing and support mode
    m_fl_support_x_continuous.resize(12);
    m_fl_support_x_continuous << -0.15184495,  0.03919922,  0.0708325 ,  0.91023942, -0.20440475,
            -0.18071851, -0.13032813, -0.24487214,  0.16202968, -0.08871389,
            -0.2624815, -0.07166776;
    m_fl_support_y_continuous.resize(12);
    m_fl_support_y_continuous << 0.02302193, -0.18148413, -0.24007166, -0.07091889,  1.10788938,
            0.09736814, -0.19662309, -0.08354826, -0.18826105, -0.14232872,
            0.05924581, -0.0698364;
    m_fl_swinging_x_continuous.resize(12);
    m_fl_swinging_x_continuous << 0.11652851,  0.00189472, -0.04504694,  0.83238274,  0.06823524,
            -0.04596665,  0.41990015,  0.18539538, -0.30730394, -0.24832586,
            0.04804271, 0.14635426;
    m_fl_swinging_y_continuous.resize(12);
    m_fl_swinging_y_continuous << -0.00357884,  0.11082693,  0.12282052,  0.05157578,  0.50063221,
            -0.02497993,  0.00296419,  0.04106386,  0.00903193, -0.09735778,
            -0.11417786, 0.04497386;

    // FR models coeff. in swing and support mode
    m_fr_support_x_continuous.resize(12);
    m_fr_support_x_continuous << -0.16138308, -0.02438681, -0.07134423, -0.11176547,  0.16311478,
            0.89093861,  0.14114674,  0.07129942,  0.20829109, -0.22483528,
            -0.11521336, -0.0405369;
    m_fr_support_y_continuous.resize(12);
    m_fr_support_y_continuous << -0.03533635, -0.17478856, -0.2227172 , -0.11401216, -0.22272676,
            0.33836212,  0.96104345,  0.0828417 ,  0.1653415 ,  0.27081116,
            -0.0598317, 0.02810415;
    m_fr_swinging_x_continuous.resize(12);
    m_fr_swinging_x_continuous << 0.11551556,  0.00163684,  0.0289484 , -0.01608893,  0.01987848,
            0.62389148, -0.02065363, -0.00974021,  0.02367121,  0.15292762,
            -0.06795528, 0.10070511;
    m_fr_swinging_y_continuous.resize(12);
    m_fr_swinging_y_continuous << 0.00912297,  0.1079365 ,  0.13100683, -0.10935009, -0.11798089,
            0.02232673,  0.48916645, -0.04830404, -0.04723069,  0.00663729,
            0.17521877, -0.01948352;

    // RL models coeff. in swing and support mode
    m_rl_support_x_continuous.resize(12);
    m_rl_support_x_continuous << -1.18838987e-01, -1.88251417e-02,  4.37989970e-02,
            -2.24631158e-01,  6.69926467e-04, -2.45582501e-01,
            2.74189194e-01,  8.85290936e-01, -8.91867675e-02,
            9.19248629e-03,  6.65138195e-02, 0.15415616;
    m_rl_support_y_continuous.resize(12);
    m_rl_support_y_continuous << 0.01181024, -0.18532179, -0.13370518,  0.13014812, -0.18112934,
            0.06781526,  0.22481193, -0.11915648,  1.05744577, -0.05901571,
            -0.0733179, -0.04744741;
    m_rl_swinging_x_continuous.resize(12);
    m_rl_swinging_x_continuous << 0.09666263, -0.01038443,  0.01841709,  0.01569752,  0.1877184 ,
            -0.00243372,  0.09683732,  0.47894133,  0.05016164,  0.2286775 ,
            0.00177181, -0.10110262;
    m_rl_swinging_y_continuous.resize(12);
    m_rl_swinging_y_continuous << -0.00271595,  0.11651606,  0.09669578,  0.075615  ,  0.10954938,
            -0.04544669, -0.0032865 , -0.0505534 ,  0.50989102, -0.10286815,
            -0.09419408, 0.00378027;

    // RR models coeff. in swing and support mode
    m_rr_support_x_continuous.resize(12);
    m_rr_support_x_continuous << -0.19227276,  0.02261951, -0.00162824, -0.0658471 ,  0.03034275,
            -0.2372897 ,  0.09278465, -0.05417402,  0.03055493,  1.37515115,
            -0.17800744, 0.13393372;
    m_rr_support_y_continuous.resize(12);
    m_rr_support_y_continuous << -0.00385966, -0.1929615 , -0.1286518 ,  0.08760647,  0.24399234,
            -0.12203393, -0.00211083, -0.00702502, -0.17471038, -0.13760298,
            1.19389043, -0.00297605;
    m_rr_swinging_x_continuous.resize(12);
    m_rr_swinging_x_continuous << 0.12876012,  0.00520883, -0.01179153,  0.12842299, -0.04365359,
            0.00800936,  0.28779961, -0.04581125, -0.32074631,  0.36661586,
            0.06766253, -0.08504516;
    m_rr_swinging_y_continuous.resize(12);
    m_rr_swinging_y_continuous << 0.03326041,  0.09204637,  0.09496362,  0.00737064,  0.04052509,
            -0.04995179,  0.28818213, -0.19121709,  0.06303849,  0.15896909,
            0.62296827, -0.03409872;
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
    m_fr_rl_com_x_acceleration << 0.08616859, -0.04134626, -0.0496516 ,  0.05945102,  0.00952706,
            -0.01539132,  0.27970343,  0.05853086, -0.04958753,  0.14737882,
            -0.01413498,  0.02327976, -0.16643397,  0.28529446, -0.04568119;

    m_fr_rl_com_y_acceleration.resize(15);
    m_fr_rl_com_y_acceleration << -0.03327943,  0.13659707,  0.16882684,  0.01626034,  0.04422556,
            0.07350738, -0.04279813, -0.13889739,  0.22035854,  0.08493568,
            -0.22000858,  0.06883889,  0.22163456, -0.07384795, -0.03220872;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_acceleration.resize(15);
    m_fl_rr_com_x_acceleration << 0.07095532,  0.06447278,  0.07152694,  0.05812389, -0.00967864,
            -0.00585743, -0.07517423, -0.23092365,  0.28561508, -0.41713247,
            -0.04346248, -0.26026371, -0.01753655, -0.0771894, -0.06648941;
    m_fl_rr_com_y_acceleration.resize(15);
    m_fl_rr_com_y_acceleration << 0.04811907,  0.11422384,  0.12882368, -0.01763643,  0.04549379,
            0.06760843, -0.07816699,  0.18005583, -0.20369763,  0.16190644,
            -0.11901309, -0.08045348,  0.06941773,  0.03465651, 0.07287382;

    // FL models coeff. in swing and support mode
    m_fl_support_x_acceleration.resize(15);
    m_fl_support_x_acceleration << -0.08657218,  0.05191882,  0.09989837, -0.05641254, -0.008118  ,
            0.00185887,  1.09047608, -0.04438799, -0.09568431, -0.14009462,
            0.11344721, -0.15655122, -0.15898238, -0.62952967, -0.09969976;
    m_fl_support_y_acceleration.resize(15);
    m_fl_support_y_acceleration << 0.04120403, -0.11645396, -0.24073119, -0.01618937, -0.04252555,
            -0.09002542,  0.17477286,  1.0719235 , -0.15638242, -0.19948911,
            0.11491228,  0.08260257, -0.4575788 ,  0.03922617, -0.13619293;
    m_fl_swinging_x_acceleration.resize(15);
    m_fl_swinging_x_acceleration << -0.01178144, -0.00534702,  0.01460952,  0.10401622,  0.00208394,
            -0.01346142,  0.35244999,  0.10079921, -0.07710348, -0.16856468,
            0.21052916,  0.27330024, -0.13317961, -0.16784818, 0.06572326;
    m_fl_swinging_y_acceleration.resize(15);
    m_fl_swinging_y_acceleration << 0.04722359, -0.01464848, -0.00189272, -0.00111866,  0.10014873,
            0.17202128, -0.1259219 ,  0.50207948,  0.07745918,  0.31336857,
            -0.63863279, -0.04369082, -0.05773615, -0.04624108, -0.02708169;

    // FR models coeff. in swing and support mode
    m_fr_support_x_acceleration.resize(15);
    m_fr_support_x_acceleration << -0.02110673, -0.06078277, -0.15093162, -0.05784646,  0.00739164,
            -0.02371516,  0.09997548,  0.19268172,  0.30342528,  0.20032851,
            -0.1452609 ,  0.44120685, -0.33363623, -0.0948195, -0.06748027;
    m_fr_support_y_acceleration.resize(15);
    m_fr_support_y_acceleration << -0.0258163 , -0.1250472 , -0.26364945,  0.0167839 , -0.04247691,
            -0.10538163,  0.17745448, -0.21460948, -0.47706315,  0.98625587,
            0.49684864,  0.11263439, -0.31420424, -0.05731277, 0.11715281;
    m_fr_swinging_x_acceleration.resize(15);
    m_fr_swinging_x_acceleration << -0.0382868 , -0.00900407, -0.02290048,  0.1076488 , -0.00190659,
            0.00256248,  0.33297949, -0.02464561,  0.48038974, -0.02807931,
            -0.04745311, -0.03227832,  0.19097485, -0.01516633, 0.07780335;
    m_fr_swinging_y_acceleration.resize(15);
    m_fr_swinging_y_acceleration << -0.01809453, -0.01077375,  0.02036496, -0.00126937,  0.10274441,
            0.16171618, -0.16819275, -0.07311762,  0.33114248,  0.44643303,
            -0.10049211, -0.06434175,  0.48633065,  0.24031912, 0.02863098;

    // RL models coeff. in swing and support mode
    m_rl_support_x_acceleration.resize(15);
    m_rl_support_x_acceleration << -0.03921768, -0.04494074, -0.0279131 , -0.0570089 ,  0.00621333,
            -0.01788349,  0.07324121,  0.14763443, -0.76450195,  0.37895452,
            1.18172399,  0.04110233, -0.20522942, -0.06600843, 0.1762453;
    m_rl_support_y_acceleration.resize(15);
    m_rl_support_y_acceleration << -0.03748126, -0.11253108, -0.06852588,  0.01558387, -0.04145404,
            -0.08317871,  0.2134796 , -0.10217792, -0.16280924, -0.13358949,
            0.36165859,  1.0538261 , -0.28493066, -0.20322749, -0.04353681;
    m_rl_swinging_x_acceleration.resize(15);
    m_rl_swinging_x_acceleration << -0.02192488, -0.01108213,  0.02291717,  0.10723446,  0.00037244,
            0.01561971,  0.15274073,  0.17767487, -0.00477221,  0.11477326,
            0.30707139, -0.04984643,  0.10415217, -0.11340734, -0.20365848;
    m_rl_swinging_y_acceleration.resize(15);
    m_rl_swinging_y_acceleration << -0.02504178, -0.01850434,  0.02779902,  0.00108434,  0.10323362,
            0.12011877, -0.12400546,  0.52261692,  0.12969648,  0.03984607,
            -0.08978501,  0.55321892,  0.36904738, -0.12580072, 0.04183551;

    // RR models coeff. in swing and support mode
    m_rr_support_x_acceleration.resize(15);
    m_rr_support_x_acceleration << -0.04592321,  0.05992726,  0.00444163, -0.05940514, -0.00592916,
            -0.0015923 , -0.26720297, -0.2507139 , -0.09690001, -0.12898225,
            -0.00549156, -0.03475759,  0.75317466, -0.35805392, -0.00632505;
    m_rr_support_y_acceleration.resize(15);
    m_rr_support_y_acceleration << 0.0467644 , -0.11682995, -0.03738772, -0.01554283, -0.04175731,
            -0.04767361, -0.41178566, -0.0363657 , -0.0592284 , -0.14906208,
            0.03646865, -0.01642791,  0.06434424,  1.08297395, 0.13677799;
    m_rr_swinging_x_acceleration.resize(15);
    m_rr_swinging_x_acceleration << -1.22117311e-02, -5.41029885e-03, -2.07880169e-02,
            1.05229334e-01,  1.08279160e-04, -1.92118599e-03,
            -1.36632511e-01, -3.90171366e-03, -3.90766990e-02,
            -7.50224517e-02,  1.98140007e-01,  2.07535397e-01,
            4.06975811e-01, -2.03351791e-02, -0.10951978;
    m_rr_swinging_y_acceleration.resize(15);
    m_rr_swinging_y_acceleration << 0.01218653,  0.00575799,  0.04918521, -0.00116033,  0.10172032,
            0.10784985, -0.11050414, -0.0343263 ,  0.26946199,  0.23847801,
            -0.36744035, -0.1401431 ,  0.12656233,  0.56553021, -0.10135209;
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
//        ROS_INFO_STREAM("Predicting FR/RL swinging");
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
//        ROS_INFO_STREAM("Predicting FL/RR swinging");
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
//    ROS_INFO_STREAM("Next Velocity: " << p_action.x * p_nextVelocity << ", " << p_action.y * p_nextVelocity << ", "
//                                      << p_action.theta * p_nextVelocity);
//    ROS_INFO_STREAM(
//            "Previous Velocity: " << p_action.x * p_previousVelocity << ", " << p_action.y * p_previousVelocity << ", "
//                                  << p_action.theta * p_previousVelocity);

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

//    // Publish predicted footstep and CoM
//    int j = 0;
//
//    // Populate array
//    visualization_msgs::Marker l_footCommonMarker;
//    l_footCommonMarker.header.stamp = ros::Time::now();
//    l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//    l_footCommonMarker.type = 2;
//    l_footCommonMarker.action = 0;
//    l_footCommonMarker.lifetime = ros::Duration(0.1);
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
//    ros::Duration(0.12).sleep();
}