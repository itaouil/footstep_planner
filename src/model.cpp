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
    m_fr_rl_com_x.resize(11);
    m_fr_rl_com_x << 0.34814042,  0.0443974 ,  0.02248182,
                     -0.24593676, -0.54733808, 0.37333974,
                     0.50504116, -0.11442048, -0.74744383,
                     -0.55015726, 0.08721679;
    m_fr_rl_com_y.resize(11);
    m_fr_rl_com_y << -0.04252885,  0.00895371, -0.06737622,
                     0.04238722, -0.11797932, -0.36156175,
                     0.04167309, -0.32009066,  0.29072492,
                     0.26253723, 0.15225123;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(11);
    m_fl_rr_com_x << 0.32672647,  0.05213033,  0.20859677,
                     -0.18385445, -0.89602334, 0.78082436,
                     0.16502125,  0.223455, -0.40618345,
                     -0.28057083, 0.16403635;
    m_fl_rr_com_y.resize(11);
    m_fl_rr_com_y << 0.077717, -0.00297991,  0.03867121,
                     -0.4555994 , -0.25568115, 0.22871646,
                     -0.10917902,  0.1900128, -0.02037374,
                     -0.3275084, 0.04002941;

    // FL models coefficients
    m_fl_swinging_x.resize(11);
    m_fl_swinging_x << 0.60810309,  0.08946696, -0.64548272,
                       -0.24221458, -1.58522499, 1.27026251,
                       -0.13346972,  0.75813714, -0.5174049,
                       -0.59862431, 0.39566204;
    m_fl_swinging_y.resize(11);
    m_fl_swinging_y << 0.1062739 , -0.00554221,  0.19705682,
                       -1.73468649,  0.05026411, 0.6478325,
                       -0.55860172,  0.30023867, -0.18480923,
                       -0.35795317, -0.02881693;

    // FR models coefficients
    m_fr_swinging_x.resize(11);
    m_fr_swinging_x << 0.64400628,  0.07445803, -0.18182764,
                       -0.5062394 , -1.83066712, 0.492878,
                       0.94443075,  0.00901013, -1.46220206,
                       -1.41000119, 0.29835093;
    m_fr_swinging_y.resize(11);
    m_fr_swinging_y << -0.08129458,  0.01506163, -0.60353,
                       0.3436308 , -0.44050002, -1.43951442,
                       0.34428192, -0.29550614,  0.97168819,
                       0.58536166, 0.47195019;

    // RL models coefficients
    m_rl_swinging_x.resize(11);
    m_rl_swinging_x << 0.6447439,  0.07364073, -0.1513184,
                       -0.50363216, -1.83730424, 0.49870859,
                       0.94485061,  0.06602175, -1.50236821,
                       -1.39676942, 0.27742248;
    m_rl_swinging_y.resize(11);
    m_rl_swinging_y << -0.08188215,  0.01504879, -0.61338043,
                       0.34914438, -0.42098793, -1.44751393,
                       0.32292546, -0.30260626,  0.98173945,
                       0.58349991, 0.46572397;

    // RR models coefficients
    m_rr_swinging_x.resize(11);
    m_rr_swinging_x << 0.60894533,  0.09002252, -0.6212247,
                       -0.26217805, -1.5968552, 1.26515921,
                       -0.11898317,  0.76859891, -0.5312728,
                       -0.60180949, 0.39276814;
    m_rr_swinging_y.resize(11);
    m_rr_swinging_y << 0.10576521, -0.00536826,  0.20810501,
                       -1.7244661,  0.04869788, 0.65568708,
                       -0.5520065,  0.30937826, -0.19285911,
                       -0.35322993, -0.03234559 ;
}

/**
 * Motion prediction.
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
void Model::motionPrediction(double p_previousVelocityX,
                             double p_previousVelocityY,
                             double p_previousAngularVelocity,
                             double p_nextVelocityX,
                             double p_nextVelocityY,
                             double p_nextAngularVelocity,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             std::vector<double> &p_predictions) {
    Eigen::VectorXd l_modelInput(11);
    l_modelInput << p_odomVelocityState.linear.x,
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
}

/**
  * Compute new CoM in world frame.
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
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    std::vector<double> l_predictions(11);

    motionPrediction(p_action.x * p_previousVelocity,
                     p_action.y * p_previousVelocity,
                     p_action.theta * p_previousVelocity,
                     p_action.x * p_nextVelocity,
                     p_action.y * p_nextVelocity,
                     p_action.theta * p_nextVelocity,
                     p_odomVelocityState,
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
                                    << l_predictions[10] <<"\n");

    computeNewCoM(l_predictions[0],
                  l_predictions[1],
                  l_predictions[10],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    computeNewFeetConfiguration(p_newWorldCoordinatesCoM,
                                l_predictions,
                                p_currentFeetConfiguration,
                                p_newFeetConfiguration);

    // // Publish predicted CoM and feet poses
    // int j = 0;
    // visualization_msgs::Marker l_footCommonMarker;
    // l_footCommonMarker.header.stamp = ros::Time::now();
    // l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    // l_footCommonMarker.type = 2;
    // l_footCommonMarker.action = 0;
    // l_footCommonMarker.lifetime = ros::Duration(0.5);
    // l_footCommonMarker.pose.orientation.x = p_currentWorldCoordinatesCoM.q.x();
    // l_footCommonMarker.pose.orientation.y = p_currentWorldCoordinatesCoM.q.y();
    // l_footCommonMarker.pose.orientation.z = p_currentWorldCoordinatesCoM.q.z();
    // l_footCommonMarker.pose.orientation.w = p_currentWorldCoordinatesCoM.q.w();
    // l_footCommonMarker.scale.x = 0.05;
    // l_footCommonMarker.scale.y = 0.035;
    // l_footCommonMarker.scale.z = 0.035;
    // l_footCommonMarker.color.r = 0;
    // l_footCommonMarker.color.g = 1;
    // l_footCommonMarker.color.b = 0;
    // l_footCommonMarker.color.a = 0.7;

    // visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
    // l_CoMMarker.id = j++;
    // l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    // l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
    // l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
    // l_CoMMarker.pose.position.z = 0.170;

    // visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
    // l_flFootMarker.id = j++;
    // l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
    // l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
    // l_flFootMarker.pose.position.z = 0.170;

    // visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
    // l_frFootMarker.id = j++;
    // l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
    // l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
    // l_frFootMarker.pose.position.z = 0.170;

    // visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
    // l_rlFootMarker.id = j++;
    // l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
    // l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
    // l_rlFootMarker.pose.position.z = 0.170;

    // visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
    // l_rrFootMarker.id = j++;
    // l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
    // l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
    // l_rrFootMarker.pose.position.z = 0.170;

    // visualization_msgs::MarkerArray l_pathFeetConfiguration;
    // l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
    // l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
    // l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
    // l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
    // l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

    // m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
    // ros::Duration(2).sleep();
}