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
    /**
     * Models to predict CoM and feet displacements.
     */
    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x.resize(12);
    m_fr_rl_com_x << 0.05448516,  0.03441826,  0.19691618,
                     0.45987373, -0.05635762, -0.17578773,
                     0.46648442, -0.0430613 , -0.40712269,
                     -0.61398136, 0.18153147, -0.0994127;
    m_fr_rl_com_y.resize(12);
    m_fr_rl_com_y << -0.00104521,  0.00979397, -0.04572748,
                     0.00508817, -0.01354416, -0.2291856,
                     -0.38698806,  0.15567586, -0.32311293,
                     0.23375873, 0.33399223, 0.19049178;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(12);
    m_fl_rr_com_x << 0.06023137,  0.03652947,  0.1716086,
                     0.46280515, -0.26706655, -0.52760397,
                     0.80584152,  0.48917598, -0.50779652,
                     -0.76653621, -0.09781971, 0.13002072;
    m_fl_rr_com_y.resize(12);
    m_fl_rr_com_y << 0.00505268, -0.00745314,  0.07240949,
                     0.16522746, -0.64905582, -0.20585345,
                     0.08712672, -0.09534711,  0.27037509,
                     -0.12442457, -0.18104688, -0.01191284;

    // FL models coefficients
    m_fl_swinging_x.resize(12);
    m_fl_swinging_x << 0.11394667,  0.05909815,  0.32815105,
                       -0.06512937, -0.70631468, -0.68068313,
                       1.08502347,  0.19066487, -0.3042283,
                       -1.3301189 , -0.27135917, 0.11362224;
    m_fl_swinging_y.resize(12);
    m_fl_swinging_y << 0.00914275, -0.0131736 ,  0.11119442,
                       0.29735921, -1.82704987, -0.23875157,
                       0.24198908, -0.2493276 ,  0.54834196,
                       -0.22923156, -0.29622512, 0.01833887;

    // FR models coefficients
    m_fr_swinging_x.resize(12);
    m_fr_swinging_x << 0.10525315,  0.05222282,  0.36930306,
                       0.40712723, -0.06413178, -1.04118673,
                       0.84753741, -0.22153804, -0.45491052,
                       -1.08686161, -0.40807668, -0.06698039;
    m_fr_swinging_y.resize(12);
    m_fr_swinging_y << -0.0040224 ,  0.01843992, -0.10350222,
                       -0.09015949,  0.11398807, -0.42075525,
                       -1.25346152,  0.27389504, -0.62840245,
                       0.58040351, 0.71929535, 0.33831753;

    // RL models coefficients
    m_rl_swinging_x.resize(12);
    m_rl_swinging_x << 0.10645284,  0.05138495,  0.36248151,
                       0.50433318,  0.02051393, -1.11204187,
                       0.91719296, -0.15139501, -0.51204497,
                       -1.14385788, -0.45906862, -0.07120493;
    m_rl_swinging_y.resize(12);
    m_rl_swinging_y << -0.00353922,  0.01844719, -0.10540354,
                       -0.1108641 ,  0.10607341, -0.42432812,
                       -1.27190947,  0.27157736, -0.61660869,
                       0.60438143, 0.7296827, 0.34816895;

    // RR models coefficients
    m_rr_swinging_x.resize(12);
    m_rr_swinging_x << 0.11311664,  0.0600422,  0.32767699,
                       -0.1213129 , -0.69495377, -0.73444584,
                       1.15241851,  0.24377274, -0.34590998,
                       -1.27758647, -0.29590616, 0.17665898;
    m_rr_swinging_y.resize(12);
    m_rr_swinging_y << 0.00838896, -0.01300398,  0.11285895,
                       0.29776656, -1.83328916, -0.25452871,
                       0.24340342, -0.23804424,  0.55672439,
                       -0.22715981, -0.30655798, 0.02405718;

    /**
     * Models to predict next CoM velocity.
     */
    m_fr_rl_com_velocity.resize(12);
    m_fr_rl_com_velocity << 0.28667625,  0.28739869,  0.999083,
                            -0.89113917, -0.71046337, -0.24828993,
                            2.2453202 , -0.49038979, -0.39022772,
                            -3.23929642, -2.8776404, -0.62540223;
    m_fl_rr_com_velocity.resize(12);
    m_fl_rr_com_velocity << 0.31694185,  0.32779365,  0.86578869,
                            1.69154918, -2.69788237, -1.85089924,
                            2.46023385, -1.65959182,  1.11302375,
                            -2.76930195, -0.64663195, -0.68732977;
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

    ROS_INFO_STREAM("Input: " << l_modelInput);

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
    ROS_INFO_STREAM("Predictions: " << l_predictions[0] << ", "
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