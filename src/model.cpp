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
    m_fr_rl_com_x.resize(11);
    m_fr_rl_com_x << 0.04422648,  0.36155565, -0.11439759, -0.21091345, -0.46140704,
            0.48245202,  0.38302874, -0.1803193 , -0.71142217, -0.62789302, 0.08529418;
    m_fr_rl_com_y.resize(11);
    m_fr_rl_com_y << 0.00779179, -0.04878178, -0.01757122,  0.05513316, -0.15511127,
            -0.4163938 ,  0.09376421, -0.3365921 ,  0.29582221,  0.30262735, 0.16242028;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(11);
    m_fl_rr_com_x << 0.05392864,  0.33456285,  0.32488612, -0.0820275 , -1.04721027,
            0.82887982,  0.3047756 ,  0.09489397, -0.50316411, -0.2949946, 0.18984772;
    m_fl_rr_com_y.resize(11);
    m_fl_rr_com_y << -0.00337644,  0.08655258,  0.06640072, -0.43785057, -0.28252446,
            0.22204231, -0.14148514,  0.18071435, -0.05246979, -0.34718979, 0.01860129;

    // FL models coefficients
    m_fl_swinging_x.resize(11);
    m_fl_swinging_x << 0.09527837,  0.62307263, -0.48934844, -0.1312738 , -1.90407128,
            1.37103624,  0.11439445,  0.48734909, -0.69368284, -0.68833573, 0.47322866;
    m_fl_swinging_y.resize(11);
    m_fl_swinging_y << -0.0081548 ,  0.12739258,  0.30876009, -1.71355152, -0.07925677,
            0.63989057, -0.56430549,  0.30499927, -0.32142618, -0.36211739, -0.0671177;

    // FR models coefficients
    m_fr_swinging_x.resize(11);
    m_fr_swinging_x << 0.0772539 ,  0.66546657, -0.48114255, -0.45338576, -1.64850384,
            0.64718163,  0.61471063, -0.018211  , -1.41160375, -1.48807261, 0.26133981;
    m_fr_swinging_y.resize(11);
    m_fr_swinging_y << 0.0153408 , -0.10082556, -0.54858947,  0.331158  , -0.48198103,
            -1.66376876,  0.37445464, -0.24880407,  1.04370394,  0.76600611, 0.48440744;

    // RL models coefficients
    m_rl_swinging_x.resize(11);
    m_rl_swinging_x << 0.07556805,  0.67523518, -0.49831388, -0.47343816, -1.68095667,
            0.65376566,  0.67023807, -0.03008145, -1.42690924, -1.4928123, 0.28810221;
    m_rl_swinging_y.resize(11);
    m_rl_swinging_y << 0.01536223, -0.10135935, -0.54734916,  0.33300104, -0.48339889,
            -1.66303995,  0.38378271, -0.24996121,  1.05226731,  0.77443481, 0.49013598;

    // RR models coefficients
    m_rr_swinging_x.resize(11);
    m_rr_swinging_x << 0.09558352,  0.62090045, -0.50914729, -0.13227248, -1.89426221,
            1.34743554,  0.11467514,  0.48284013, -0.67868982, -0.65857396, 0.48087558;
    m_rr_swinging_y.resize(11);
    m_rr_swinging_y << -0.00811884,  0.12795483,  0.29975576, -1.71326575, -0.06978923,
            0.63645976, -0.57856001,  0.31339186, -0.31169629, -0.3608843, -0.06971812;

    /**
     * Models to predict next CoM velocity.
     */
    m_com_velocity.resize(11);
    m_fr_rl_com_velocity << 0.49175065, 0.58153937, 0, 0,
            0,0,0,0,0,0, -0.01958978;
    m_fl_rr_com_velocity.resize(11);
    m_fl_rr_com_velocity << 0.49175065, 0.58153937, 0, 0,
            0,0,0,0,0,0, -0.01958978;
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
    m_fr_rl_com_x.resize(11);
    m_fr_rl_com_x << 0.14195434,  0.29309914, -0.45063771,  0.10792488, -0.35124081,
            -0.27716803,  0.63653376,  0.45524618, -0.2922709 , -0.43105432, 0.12798102;
    m_fr_rl_com_y.resize(11);
    m_fr_rl_com_y << 0.02240725, -0.01769868,  0.04851553,  0.0044649 ,  0.00978158,
            -0.07848464, -0.01821843, -0.03730028, -0.07432889,  0.18059606, -0.01957075;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(11);
    m_fl_rr_com_x << 0.14497326,  0.27579546, -0.46409283,  0.30298774, -0.49893243,
            -0.27786822, -0.1428397 ,  0.63605616,  0.7484066 , -0.47387858, 0.18265735;
    m_fl_rr_com_y.resize(11);
    m_fl_rr_com_y << -0.0187296 ,  0.01060315, -0.00248636, -0.1425632 , -0.03449454,
            0.03747548,  0.05862977,  0.16866319, -0.00718056, -0.01268418, 0.02055811;

    // FL models coefficients
    m_fl_swinging_x.resize(11);
    m_fl_swinging_x << 0.30013909,  0.36931784, -1.1074553 ,  0.93583121, -0.25726387,
            -0.11980933, -0.50802773,  0.56461053,  0.61182688, -0.97648846, 0.03803627;
    m_fl_swinging_y.resize(11);
    m_fl_swinging_y << -0.04428962, -0.00720189,  0.0721136 , -1.0658317 , -0.55454471,
            -0.00109622,  0.70293325,  0.54989502, -0.1292977 ,  0.14070075, 0.36168857;

    // FR models coefficients
    m_fr_swinging_x.resize(11);
    m_fr_swinging_x << 0.2937814 ,  0.37239737, -0.23818396, -0.16423643, -1.08254897,
            -0.68770499,  0.56144116,  0.83909571, -0.61687197, -0.22815406, 0.11685523;
    m_fr_swinging_y.resize(11);
    m_fr_swinging_y << 0.0498393 ,  0.00436791,  0.54752218,  0.05112793, -0.08545294,
            -1.14838131,  0.10240832,  0.10149859, -0.71089934,  0.6389606, -0.36486827;

    // RL models coefficients
    m_rl_swinging_x.resize(11);
    m_rl_swinging_x << 0.29476012,  0.37255638, -0.19666099, -0.20529423, -1.10887963,
            -0.67182016,  0.59328337,  0.8695958 , -0.6709506 , -0.1866163, 0.11607556;
    m_rl_swinging_y.resize(11);
    m_rl_swinging_y << 0.04987723,  0.00214641,  0.55488705,  0.05641943, -0.10095723,
            -1.14232925,  0.11468043,  0.10034111, -0.71062025,  0.63335682, -0.3599297;

    // RR models coefficients
    m_rr_swinging_x.resize(11);
    m_rr_swinging_x << 0.3000257 ,  0.36589414, -1.16892414,  0.9394715 , -0.29496693,
            -0.16138798, -0.44886004,  0.60326015,  0.67842782, -0.98761891, 0.08390283;
    m_rr_swinging_y.resize(11);
    m_rr_swinging_y << -0.04434898, -0.00771166,  0.07051247, -1.07733049, -0.56825544,
            0.0163967 ,  0.71544876,  0.55445835, -0.1263772 ,  0.14038913, 0.37286927;

    /**
     * Models to predict next CoM velocity.
     */
    m_fr_rl_com_velocity.resize(11);
    m_fr_rl_com_velocity << 0.85510601,  0.17584364, -0.54060372,  0.10019051, -0.33629804,
            -0.18362215,  0.36374963,  0.01261887,  0.07632279,  0.11875604, 0.31037846;
    m_fl_rr_com_velocity.resize(11);
    m_fl_rr_com_velocity << 0.86909078,  0.16767237, -0.34461438,  0.58227706, -0.38741709,
            -0.44716001,  0.02843422,  0.05172457,  0.33606895, -0.04124653, 0.12983126;
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
    Eigen::VectorXd l_modelInput(11);
    l_modelInput << p_nextVelocityX,
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
    Eigen::VectorXd l_modelInput(11);
    l_modelInput << p_nextVelocityX,
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

//      // Publish predicted CoM and feet poses
//      int j = 0;
//      visualization_msgs::Marker l_footCommonMarker;
//      l_footCommonMarker.header.stamp = ros::Time::now();
//      l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//      l_footCommonMarker.type = 2;
//      l_footCommonMarker.action = 0;
//      l_footCommonMarker.lifetime = ros::Duration(0.5);
//      l_footCommonMarker.pose.orientation.x = p_currentWorldCoordinatesCoM.q.x();
//      l_footCommonMarker.pose.orientation.y = p_currentWorldCoordinatesCoM.q.y();
//      l_footCommonMarker.pose.orientation.z = p_currentWorldCoordinatesCoM.q.z();
//      l_footCommonMarker.pose.orientation.w = p_currentWorldCoordinatesCoM.q.w();
//      l_footCommonMarker.scale.x = 0.05;
//      l_footCommonMarker.scale.y = 0.035;
//      l_footCommonMarker.scale.z = 0.035;
//      l_footCommonMarker.color.r = 0;
//      l_footCommonMarker.color.g = 0;
//      l_footCommonMarker.color.b = 1;
//      l_footCommonMarker.color.a = 1;
//
//      visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
//      l_CoMMarker.id = j++;
//      l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//      l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
//      l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
//      l_CoMMarker.pose.position.z = 0.170;
//
//      visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
//      l_flFootMarker.id = j++;
//      l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
//      l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
//      l_flFootMarker.pose.position.z = 0.170;
//
//      visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//      l_frFootMarker.id = j++;
//      l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
//      l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
//      l_frFootMarker.pose.position.z = 0.170;
//
//      visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//      l_rlFootMarker.id = j++;
//      l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
//      l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
//      l_rlFootMarker.pose.position.z = 0.170;
//
//      visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//      l_rrFootMarker.id = j++;
//      l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
//      l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
//      l_rrFootMarker.pose.position.z = 0.170;
//
//      visualization_msgs::MarkerArray l_pathFeetConfiguration;
//      l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
//      l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
//      l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
//      l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
//      l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);
//
//      m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
//      ros::Duration(2).sleep();
}