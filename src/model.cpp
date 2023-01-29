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

    // Set models coefficients (real robot)
    if (SIMULATION) {
        setModelsCoefficientsSimulation();
    }
    else {
        setModelsCoefficientsReal();
    }


}

/**
 * Destructor
 */
Model::~Model() = default;

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of continuous
 * velocity commands for the real robot;
 */
void Model::setModelsCoefficientsReal() {
    /**
     * Models to predict CoM and feet displacements.
     */
    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x.resize(12);
    m_fr_rl_com_x << 0.06332597,  0.02703497,  0.1960978,  0.15615165,  0.09147864,
            -0.43828674,  0.35268758,  0.21791736, -0.20224054, -0.33295142,
            -0.29193987, 0.05706894;
    m_fr_rl_com_y.resize(12);
    m_fr_rl_com_y << 0.00139705,  0.00794286, -0.04676512, -0.0122436 ,  0.0483176 ,
            -0.14921831, -0.40602063,  0.09230524, -0.32213948,  0.27410517,
            0.28576526, 0.15204344;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(12);
    m_fl_rr_com_x << 0.07166076,  0.02760293,  0.16344876,  0.12077152, -0.10377703,
            -0.72439479,  0.4567756 ,  0.65285092, -0.21270105, -0.45067008,
            -0.21489902, 0.26707376;
    m_fl_rr_com_y.resize(12);
    m_fl_rr_com_y << 0.00585145, -0.00599027,  0.06448369,  0.03400156, -0.44946822,
            -0.22689174,  0.1565868 , -0.1036284 ,  0.15511946, -0.04615985,
            -0.32311168, 0.02368865;

    // FL models coefficients
    m_fl_swinging_x.resize(12);
    m_fl_swinging_x << 0.13425642,  0.04440987,  0.31318025, -0.82261151, -0.1726768 ,
            -1.28514097,  0.69416782,  0.72024664, -0.03341558, -0.61090385,
            -0.54472856, 0.58157249;
    m_fl_swinging_y.resize(12);
    m_fl_swinging_y << 0.00759202, -0.01038824,  0.09155511,  0.21189849, -1.72705035,
            0.05052868,  0.52961249, -0.54124066,  0.27099776, -0.26092137,
            -0.32862838, -0.05724075;

    // FR models coefficients
    m_fr_swinging_x.resize(12);
    m_fr_swinging_x << 0.12058061,  0.04052996,  0.37333293,  0.01966696,  0.10775982,
            -1.54042714,  0.41545651,  0.28988817, -0.03052201, -0.76595059,
            -0.88297853, 0.169984;
    m_fr_swinging_y.resize(12);
    m_fr_swinging_y << -0.00185444,  0.01563072, -0.08602507, -0.54807157,  0.30734842,
            -0.47104925, -1.62064904,  0.38076756, -0.2293023 ,  0.97497215,
            0.70241665, 0.46365326;

    // RL models coefficients
    m_rl_swinging_x.resize(12);
    m_rl_swinging_x << 0.11970358,  0.04103   ,  0.37303573,  0.00550732,  0.10453207,
            -1.5313616 ,  0.42473327,  0.26879397, -0.0166233 , -0.75687411,
            -0.87923592, 0.16842458;
    m_rl_swinging_y.resize(12);
    m_rl_swinging_y << -0.00331044,  0.01588474, -0.08395149, -0.53993221,  0.298245  ,
            -0.47393506, -1.6319909 ,  0.37783795, -0.21633675,  0.95435072,
            0.69305827, 0.45335082;

    // RR models coefficients
    m_rr_swinging_x.resize(12);
    m_rr_swinging_x << 0.13300849,  0.0453301 ,  0.31194444, -0.83304333, -0.19228761,
            -1.29134474,  0.6951624 ,  0.73610529, -0.0432848 , -0.60054927,
            -0.54900332, 0.59548225;
    m_rr_swinging_y.resize(12);
    m_rr_swinging_y << 0.00713925, -0.01037364,  0.09397095,  0.21292658, -1.72895593,
            0.04134238,  0.53610303, -0.54010135,  0.27526456, -0.25713117,
            -0.32073478, -0.05246776;

    /**
     * Models to predict next CoM velocity.
     */
    m_fr_rl_com_velocity.resize(12);
    m_fr_rl_com_velocity << 0.36266859,  0.22558791,  1.01144484, -0.4131313 , -0.75598252,
            -0.59342166,  1.43732982, -0.09615676, -0.53061334, -3.81047013,
            -3.87418707, -0.91324077;
    m_fl_rr_com_velocity.resize(12);
    m_fl_rr_com_velocity << 0.40888028,  0.24568807,  0.81220226,  0.08365514, -1.27276491,
            -3.88550679,  2.9413424 ,  0.165419  ,  0.55311668, -1.44982485,
            -1.33304894, 0.88128811;
}

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of continuous
 * velocity commands for the simulated robot;
 */
void Model::setModelsCoefficientsSimulation() {
    /**
     * Models to predict CoM and feet displacements.
     */
    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x.resize(12);
    m_fr_rl_com_x << 0.26798353,  0.1411866 ,  0.0034254 , -0.26421108, -0.12315839,
            -0.15470753, -0.58805423,  0.44947799,  0.27569299, -0.46356286,
            0.375402, 0.05782257;
    m_fr_rl_com_y.resize(12);
    m_fr_rl_com_y << -0.02788519,  0.02285328,  0.00231819,  0.03030785,  0.03458423,
            -0.02304854,  0.02020893,  0.00394446, -0.0448641 , -0.01926709,
            0.06255304, 0.00727056;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(12);
    m_fl_rr_com_x << 0.26327099,  0.14361449,  0.00636443, -0.32471669,  0.77742042,
            -0.29108908, -0.12423961, -0.37063209, -0.172853  ,  0.62908985,
            -0.34215119, 0.08769401;
    m_fl_rr_com_y.resize(12);
    m_fl_rr_com_y << 0.02601374, -0.01946098, -0.00443268,  0.03354043, -0.00348309,
            -0.00522878,  0.05403825, -0.01548992,  0.0463616 , -0.03288831,
            -0.03115343, -0.02351051;

    // FL models coefficients
    m_fl_swinging_x.resize(12);
    m_fl_swinging_x << 0.05641654,  0.15631046,  0.0120269 , -0.72743333,  0.57153082,
            0.25773774,  0.18065822, -0.29872819, -0.17041045, -0.0634581 ,
            -0.4132508, -0.05559401;
    m_fl_swinging_y.resize(12);
    m_fl_swinging_y << -1.62818227e-02, -2.53123444e-02,  4.55112055e-04,
            5.87401869e-02, -9.53254386e-01, -5.30178244e-01,
            -2.90697786e-02,  6.46311107e-01,  4.28985133e-01,
            -1.06088315e-01,  1.47810391e-01, 0.35043307;

    // FR models coefficients
    m_fr_swinging_x.resize(12);
    m_fr_swinging_x << 0.02940453,  0.15429298,  0.02150346,  0.22646398, -0.35517057,
            -0.77631345, -0.22728933, -0.03639591,  0.32571008, -0.24878486,
            0.21317364, 0.06906874;
    m_fr_swinging_y.resize(12);
    m_fr_swinging_y << 0.01615353,  0.02741666, -0.00157497,  0.52101965,  0.04749743,
            -0.08922193, -1.06503854,  0.10659531,  0.11019611, -0.63289466,
            0.48824533, -0.34698529;

    // RL models coefficients
    m_rl_swinging_x.resize(12);
    m_rl_swinging_x << 0.03024467,  0.15427875,  0.02165259,  0.21335346, -0.33438107,
            -0.76926163, -0.26676026, -0.04172628,  0.34390123, -0.23147201,
            0.22110347, 0.06490906;
    m_rl_swinging_y.resize(12);
    m_rl_swinging_y << 0.01728412,  0.0275103 , -0.00141114,  0.51586399,  0.03997077,
            -0.09886605, -1.07949542,  0.12058875,  0.12088729, -0.63406076,
            0.49657029, -0.34081477;

    // RR models coefficients
    m_rr_swinging_x.resize(12);
    m_rr_swinging_x << 0.05674911,  0.15590937,  0.01131618, -0.70891987,  0.61493807,
            0.26590464,  0.152007  , -0.29232421, -0.16986644, -0.08828293,
            -0.42674721, -0.07820096;
    m_rr_swinging_y.resize(12);
    m_rr_swinging_y << -1.47349209e-02, -2.55660078e-02, -3.47223456e-04,
            6.32050029e-02, -9.27105999e-01, -5.36895165e-01,
            -3.51363357e-02,  6.54055405e-01,  4.20547669e-01,
            -1.07819052e-01,  1.37503324e-01, 0.34842818;

    /**
     * Models to predict next CoM velocity.
     */
    m_fr_rl_com_velocity.resize(12);
    m_fr_rl_com_velocity << -0.5038789 ,  0.6790296 ,  1.12470442, -1.09632737,  0.4291266 ,
            -0.80260592, -0.31084915,  0.7855237 , -0.18593647, -0.66790006,
            0.30944715, 0.48800521;
    m_fl_rr_com_velocity.resize(12);
    m_fl_rr_com_velocity << -0.32050175,  0.78641588,  0.69849847,  0.21611672,  0.25653276,
            -0.1169588 ,  0.1666966 , -0.79250885,  0.4213433 , -0.26584696,
            -0.33278697, -0.41933941;
}

/**
 * Motion prediction for the first step.
 *
 * @param p_plannedHorizon
 * @param p_previousVelocityX
 * @param p_previousVelocityY
 * @param p_previousAngularVelocity
 * @param p_nextVelocityX
 * @param p_nextVelocityY
 * @param p_nextAngularVelocity
 * @param p_currentFeetConfiguration
 * @param p_predictions
 */
void Model::motionPrediction(uint p_plannedHorizon,
                             double p_previousVelocityX,
                             double p_previousVelocityY,
                             double p_previousAngularVelocity,
                             double p_nextVelocityX,
                             double p_nextVelocityY,
                             double p_nextAngularVelocity,
                             const double &p_baseVelocity,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             std::vector<double> &p_predictions) {
    Eigen::VectorXd l_modelInput(12);
    l_modelInput << p_previousVelocityX,
                    p_nextVelocityX,
                    p_baseVelocity,
                    p_currentFeetConfiguration.flCoM.x,
                    p_currentFeetConfiguration.flCoM.y,
                    p_currentFeetConfiguration.frCoM.x,
                    p_currentFeetConfiguration.frCoM.y,
                    p_currentFeetConfiguration.rlCoM.x,
                    p_currentFeetConfiguration.rlCoM.y,
                    p_currentFeetConfiguration.rrCoM.x,
                    p_currentFeetConfiguration.rrCoM.y,
                    1;

    ROS_DEBUG_STREAM("Input: " << l_modelInput);

    if (p_currentFeetConfiguration.fr_rl_swinging) {
        // CoM prediction
        p_predictions[0] = m_fr_rl_com_x * l_modelInput;
        p_predictions[1] = 0.0;

        // FL prediction
        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        // FR prediction
        p_predictions[4] = m_fr_swinging_x * l_modelInput;
        p_predictions[5] = 0.0;

        // RL prediction
        p_predictions[6] = m_rl_swinging_x * l_modelInput;
        p_predictions[7] = 0.0;

        // RR prediction
        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;

        // Theta (CoM) prediction
        p_predictions[10] = 0.0;
    }
    else {
        // CoM prediction
        p_predictions[0] = m_fl_rr_com_x * l_modelInput;
        p_predictions[1] = 0.0;

        // FL prediction
        p_predictions[2] = m_fl_swinging_x * l_modelInput;
        p_predictions[3] = 0.0;

        // FR prediction
        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        // RL prediction
        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

        // RR prediction
        p_predictions[8] = m_rr_swinging_x * l_modelInput;
        p_predictions[9] = 0.0;

        // Theta (CoM) prediction
        p_predictions[10] = 0.0;
    }

    // Predicted CoM velocity
    p_predictions[11] = velocityPrediction(p_previousVelocityX,
                                           p_previousVelocityY,
                                           p_previousAngularVelocity,
                                           p_nextVelocityX,
                                           p_nextVelocityY,
                                           p_nextAngularVelocity,
                                           p_baseVelocity,
                                           p_currentFeetConfiguration);
}

/**
 * Motion prediction for second step onwards.
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
double Model::velocityPrediction(double p_previousVelocityX,
                                 double p_previousVelocityY,
                                 double p_previousAngularVelocity,
                                 double p_nextVelocityX,
                                 double p_nextVelocityY,
                                 double p_nextAngularVelocity,
                                 double p_baseVelocity,
                                 const FeetConfiguration &p_currentFeetConfiguration) {
    Eigen::VectorXd l_modelInput(12);
    l_modelInput << p_previousVelocityX,
                    p_nextVelocityX,
                    p_baseVelocity,
                    p_currentFeetConfiguration.flCoM.x,
                    p_currentFeetConfiguration.flCoM.y,
                    p_currentFeetConfiguration.frCoM.x,
                    p_currentFeetConfiguration.frCoM.y,
                    p_currentFeetConfiguration.rlCoM.x,
                    p_currentFeetConfiguration.rlCoM.y,
                    p_currentFeetConfiguration.rrCoM.x,
                    p_currentFeetConfiguration.rrCoM.y,
                    1;

    ROS_DEBUG_STREAM("Velocity Input: " << l_modelInput);

    if (p_currentFeetConfiguration.fr_rl_swinging) {
        return m_fr_rl_com_velocity * l_modelInput;
    }
    else {
        return m_fl_rr_com_velocity * l_modelInput;
    }
}

/**
  * Compute new CoM in world frame.
  *
  * @param p_predictedCoMVelocity
  * @param p_predictedCoMDisplacementX
  * @param p_predictedCoMDisplacementY
  * @param p_predictedCoMDisplacementTheta,
  * @param p_currentWorldCoordinatesCoM
  * @param p_newWorldCoordinatesCoM
  */
void Model::computeNewCoM(const double p_predictedCoMDisplacementX,
                          const double p_predictedCoMDisplacementY,
                          const double p_predictedCoMDisplacementTheta,
                          const double p_predictedCoMVelocity,
                          const World3D &p_currentWorldCoordinatesCoM,
                          World3D &p_newWorldCoordinatesCoM) {
    // Get rotation matrix of 
    // the base w.r.t world
    Eigen::Quaterniond q;
    q.x() = p_currentWorldCoordinatesCoM.q.x();
    q.y() = p_currentWorldCoordinatesCoM.q.y();
    q.z() = p_currentWorldCoordinatesCoM.q.z();
    q.w() = p_currentWorldCoordinatesCoM.q.w();
    Eigen::Matrix3d RWorldBase = q.normalized().toRotationMatrix();

    // CoM displacement
    Eigen::Vector3d l_displacementCoMFrame{p_predictedCoMDisplacementX, p_predictedCoMDisplacementY, 0.0};

    // Map displacement
    Eigen::Vector3d l_displacementMapFrame = RWorldBase.transpose() * l_displacementCoMFrame;

    // Update predicted CoM velocity
    p_newWorldCoordinatesCoM.v = p_predictedCoMVelocity;

    // Update CoM position in world frame
    p_newWorldCoordinatesCoM.x = p_currentWorldCoordinatesCoM.x + l_displacementMapFrame(0);
    p_newWorldCoordinatesCoM.y = p_currentWorldCoordinatesCoM.y + l_displacementMapFrame(1);

    ROS_DEBUG_STREAM("CoM position: " << p_currentWorldCoordinatesCoM.x << ", " << p_currentWorldCoordinatesCoM.y);
    ROS_DEBUG_STREAM("CoM position2: " << p_newWorldCoordinatesCoM.x << ", " << p_newWorldCoordinatesCoM.y);

    // Compute quaternion representation of predicted rotation
    tf2::Quaternion l_velocityCommandQuaternion;
    //l_velocityCommandQuaternion.setRPY(0, 0, p_predictedCoMDisplacementTheta);

    // Apply predicted quaternion rotation to
    // current CoM's rotation in world frame
//     tf2::Quaternion l_newCoMRotation = p_currentWorldCoordinatesCoM.q * l_velocityCommandQuaternion;
//     l_newCoMRotation.normalize();
//     p_newWorldCoordinatesCoM.q = l_newCoMRotation;
    p_newWorldCoordinatesCoM.q = p_currentWorldCoordinatesCoM.q;
}

/**
 * Compute the new feet configuration.
 *
 * @param p_predictions
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeNewFeetConfiguration(const World3D &p_newWorldCoordinatesCoM,
                                        const std::vector<double> &p_predictions,
                                        const FeetConfiguration &p_currentFeetConfiguration,
                                        FeetConfiguration &p_newFeetConfiguration) {
    // Get rotation matrix of
    // the base w.r.t world
    Eigen::Quaterniond q;
    q.x() = p_newWorldCoordinatesCoM.q.x();
    q.y() = p_newWorldCoordinatesCoM.q.y();
    q.z() = p_newWorldCoordinatesCoM.q.z();
    q.w() = p_newWorldCoordinatesCoM.q.w();
    Eigen::Matrix3d RWorldBase = q.normalized().toRotationMatrix();

    // Transform feet prediction 
    // from CoM to world frame
    std::vector<double> l_feetPredictionWorldFrame;
    Eigen::Vector3d l_feetPredictionCoMFrame;
    for (int x = 0; x < 4; x++) {
        l_feetPredictionCoMFrame << p_predictions[2 * x + 2], p_predictions[2 * x + 3], 0.0;
        Eigen::Vector3d l_footDisplacementWorldFrame = RWorldBase.transpose() * l_feetPredictionCoMFrame;
        l_feetPredictionWorldFrame.push_back(l_footDisplacementWorldFrame(0));
        l_feetPredictionWorldFrame.push_back(l_footDisplacementWorldFrame(1));

        ROS_DEBUG_STREAM("CoM (x): " << l_feetPredictionCoMFrame(0) << ". CoM (y): " << l_feetPredictionCoMFrame(1));
        ROS_DEBUG_STREAM("Map (x): " << l_footDisplacementWorldFrame(0) << ". Map(y): " << l_footDisplacementWorldFrame(1));
    }

    // Feet poses in world frame
    p_newFeetConfiguration.flMap.x = p_currentFeetConfiguration.flMap.x + l_feetPredictionWorldFrame[0];
    p_newFeetConfiguration.flMap.y = p_currentFeetConfiguration.flMap.y + l_feetPredictionWorldFrame[1];

    p_newFeetConfiguration.frMap.x = p_currentFeetConfiguration.frMap.x + l_feetPredictionWorldFrame[2];
    p_newFeetConfiguration.frMap.y = p_currentFeetConfiguration.frMap.y + l_feetPredictionWorldFrame[3];

    p_newFeetConfiguration.rlMap.x = p_currentFeetConfiguration.rlMap.x + l_feetPredictionWorldFrame[4];
    p_newFeetConfiguration.rlMap.y = p_currentFeetConfiguration.rlMap.y + l_feetPredictionWorldFrame[5];

    p_newFeetConfiguration.rrMap.x = p_currentFeetConfiguration.rrMap.x + l_feetPredictionWorldFrame[6];
    p_newFeetConfiguration.rrMap.y = p_currentFeetConfiguration.rrMap.y + l_feetPredictionWorldFrame[7];

    // Feet poses in CoM frame
    p_newFeetConfiguration.flCoM.x = std::abs(p_newFeetConfiguration.flMap.x - p_newWorldCoordinatesCoM.x);
    p_newFeetConfiguration.flCoM.y = std::abs(p_newFeetConfiguration.flMap.y - p_newWorldCoordinatesCoM.y);

    p_newFeetConfiguration.frCoM.x = std::abs(p_newFeetConfiguration.frMap.x - p_newWorldCoordinatesCoM.x);
    p_newFeetConfiguration.frCoM.y = -std::abs(p_newFeetConfiguration.frMap.y - p_newWorldCoordinatesCoM.y);

    p_newFeetConfiguration.rlCoM.x = -std::abs(p_newFeetConfiguration.rlMap.x - p_newWorldCoordinatesCoM.x);
    p_newFeetConfiguration.rlCoM.y = std::abs(p_newFeetConfiguration.rlMap.y - p_newWorldCoordinatesCoM.y);

    p_newFeetConfiguration.rrCoM.x = -std::abs(p_newFeetConfiguration.rrMap.x - p_newWorldCoordinatesCoM.x);
    p_newFeetConfiguration.rrCoM.y = -std::abs(p_newFeetConfiguration.rrMap.y - p_newWorldCoordinatesCoM.y);

    ROS_DEBUG_STREAM("Pose: " << p_newWorldCoordinatesCoM.x << ", " << p_newWorldCoordinatesCoM.y);
    ROS_DEBUG_STREAM("FL: " << p_newFeetConfiguration.flMap.x << ", " << p_newFeetConfiguration.flMap.y);
    ROS_DEBUG_STREAM("FR: " << p_newFeetConfiguration.frMap.x << ", " << p_newFeetConfiguration.frMap.y);
    ROS_DEBUG_STREAM("RL: " << p_newFeetConfiguration.rlMap.x << ", " << p_newFeetConfiguration.rlMap.y);
    ROS_DEBUG_STREAM("RR: " << p_newFeetConfiguration.rrMap.x << ", " << p_newFeetConfiguration.rrMap.y << "\n\n");

    // Change swinging sequence
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;
}

/**
 * Predicts new robot state given
 * the previous state and a new
 * velocity.
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
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    std::vector<double> l_predictions(12);
    motionPrediction(p_plannedFootstep,
                     p_action.x * p_previousVelocity,
                     p_action.y * p_previousVelocity,
                     p_action.theta * p_previousVelocity,
                     p_action.x * p_nextVelocity,
                     p_action.y * p_nextVelocity,
                     p_action.theta * p_nextVelocity,
                     p_currentWorldCoordinatesCoM.v,
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
                                    << l_predictions[10] <<","
                                    << l_predictions[11] <<"\n");

    computeNewCoM(l_predictions[0],
                  l_predictions[1],
                  l_predictions[10],
                  l_predictions[11],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    computeNewFeetConfiguration(p_newWorldCoordinatesCoM,
                                l_predictions,
                                p_currentFeetConfiguration,
                                p_newFeetConfiguration);

    //  // Publish predicted CoM and feet poses
    //  int j = 0;
    //  visualization_msgs::Marker l_footCommonMarker;
    //  l_footCommonMarker.header.stamp = ros::Time::now();
    //  l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    //  l_footCommonMarker.type = 2;
    //  l_footCommonMarker.action = 0;
    //  l_footCommonMarker.lifetime = ros::Duration(0.5);
    //  l_footCommonMarker.pose.orientation.x = p_currentWorldCoordinatesCoM.q.x();
    //  l_footCommonMarker.pose.orientation.y = p_currentWorldCoordinatesCoM.q.y();
    //  l_footCommonMarker.pose.orientation.z = p_currentWorldCoordinatesCoM.q.z();
    //  l_footCommonMarker.pose.orientation.w = p_currentWorldCoordinatesCoM.q.w();
    //  l_footCommonMarker.scale.x = 0.05;
    //  l_footCommonMarker.scale.y = 0.035;
    //  l_footCommonMarker.scale.z = 0.035;
    //  l_footCommonMarker.color.r = 0;
    //  l_footCommonMarker.color.g = 0;
    //  l_footCommonMarker.color.b = 1;
    //  l_footCommonMarker.color.a = 1;

    //  visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
    //  l_CoMMarker.id = j++;
    //  l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    //  l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
    //  l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
    //  l_CoMMarker.pose.position.z = 0.170;

    //  visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
    //  l_flFootMarker.id = j++;
    //  l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
    //  l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
    //  l_flFootMarker.pose.position.z = 0.170;

    //  visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
    //  l_frFootMarker.id = j++;
    //  l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
    //  l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
    //  l_frFootMarker.pose.position.z = 0.170;

    //  visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
    //  l_rlFootMarker.id = j++;
    //  l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
    //  l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
    //  l_rlFootMarker.pose.position.z = 0.170;

    //  visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
    //  l_rrFootMarker.id = j++;
    //  l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
    //  l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
    //  l_rrFootMarker.pose.position.z = 0.170;

    //  visualization_msgs::MarkerArray l_pathFeetConfiguration;
    //  l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
    //  l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
    //  l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
    //  l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
    //  l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

    //  m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
    //  ros::Duration(0.5).sleep();
}