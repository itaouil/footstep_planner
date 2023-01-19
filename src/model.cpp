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
    m_fr_rl_com_x.resize(11);
    m_fr_rl_com_x << 0.3413327 ,  0.04330229, -0.00892772, -0.13436354, -0.45046356,
            0.42385022,  0.39755945, -0.18969188, -0.66459745, -0.5523526, 0.0658942;
    m_fr_rl_com_y.resize(11);
    m_fr_rl_com_y << -0.04183188,  0.00881484, -0.03864055,  0.05035712, -0.13322402,
            -0.39830968,  0.07347725, -0.31758539,  0.26979435,  0.28527234, 0.14868676;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(11);
    m_fl_rr_com_x << 0.32067152,  0.0487076 ,  0.21003707, -0.08485246, -0.92080453,
            0.69307246,  0.23624223,  0.09224192, -0.42274909, -0.22896073, 0.18334296;
    m_fl_rr_com_y.resize(11);
    m_fl_rr_com_y << 0.07696628, -0.00455455,  0.02086856, -0.45442287, -0.24427243,
            0.18229701, -0.13405219,  0.1802141 , -0.02508452, -0.31299782, 0.03107825;

    // FL models coefficients
    m_fl_swinging_x.resize(11);
    m_fl_swinging_x << 0.61037315,  0.08302818, -0.65020145, -0.16353701, -1.64521276,
            1.09568879, -0.06585305,  0.5517719 , -0.54658792, -0.5340484, 0.42390254;
    m_fl_swinging_y.resize(11);
    m_fl_swinging_y << 0.10584819, -0.00840725,  0.14967247, -1.7420368 ,  0.03483904,
            0.56763245, -0.58127223,  0.27081265, -0.19349724, -0.33035011, -0.02407202;

    // FR models coefficients
    m_fr_swinging_x.resize(11);
    m_fr_swinging_x << 0.64755647,  0.07058201, -0.31313665, -0.28663073, -1.59341278,
            0.59278647,  0.64928919, -0.05817972, -1.37145254, -1.37606334, 0.21734779;
    m_fr_swinging_y.resize(11);
    m_fr_swinging_y << -0.08948509,  0.01621373, -0.58559904,  0.32244687, -0.44066079,
            -1.59453324,  0.32992371, -0.23782513,  0.99771837,  0.69641863, 0.4598784;

    // RL models coefficients
    m_rl_swinging_x.resize(11);
    m_rl_swinging_x << 0.65167322,  0.07066066, -0.33970894, -0.30481623, -1.57784872,
            0.568529  ,  0.64116947, -0.07782225, -1.36626907, -1.38156732, 0.22028612;
    m_rl_swinging_y.resize(11);
    m_rl_swinging_y << -0.08680267,  0.01583431, -0.5954909 ,  0.31846572, -0.4310003 ,
            -1.5929267 ,  0.32394818, -0.2425585 ,  0.99573987,  0.68836777, 0.45815307;

    // RR models coefficients
    m_rr_swinging_x.resize(11);
    m_rr_swinging_x << 0.60561131,  0.08311347, -0.6445211 , -0.16686697, -1.6557399 ,
            1.09562388, -0.02769487,  0.529682  , -0.55934387, -0.54432458, 0.43338959;
    m_rr_swinging_y.resize(11);
    m_rr_swinging_y << 0.10677755, -0.00857562,  0.1584875 , -1.73643864,  0.03599106,
            0.5685165 , -0.58385727,  0.27482395, -0.19653561, -0.3306087, -0.02902543;

    /**
     * Models to predict next CoM velocity.
     */
    m_fr_rl_com_velocity.resize(11);
    m_fr_rl_com_velocity << 1.86877324,  0.31594136, -1.48811733, -1.92292572, -0.88482433,
            1.88067609,  1.16274509, -0.81975005, -5.71904961, -5.39427136, -0.68953185;
    m_fl_rr_com_velocity.resize(11);
    m_fl_rr_com_velocity << 1.73371804,  0.36594976,  0.35635102, -1.38628841, -4.93901498,
            4.13528431, -2.31033018,  2.38689846, -0.94311919, -1.14797215, 0.53724605;
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
    ROS_DEBUG_STREAM("Planned footstep horizon " << p_plannedHorizon);

    Eigen::VectorXd l_modelInput(11);
    l_modelInput << p_baseVelocity,
                    p_nextVelocityX,
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
    Eigen::VectorXd l_modelInput(11);
    l_modelInput << p_baseVelocity,
                    p_nextVelocityX,
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