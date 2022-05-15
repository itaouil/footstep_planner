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

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of continuous
 * velocity commands.
 */
void Model::setModelsCoefficients() {
    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x.resize(21);
    m_fr_rl_com_x << 9.59733942e-03,  1.84429939e-02,  1.58683948e-02,
            1.21596511e-01, -3.22463364e-02,  1.55011567e-04,
            1.96935440e-02, -4.24067395e-02,  3.53699519e-02,
            1.46701020e-03,  1.02717710e-01, -3.98563621e-02,
            9.24792547e-02,  1.90660254e-02,  2.05498477e-01,
            -1.26494972e-01,  2.09232937e-01, -1.30029779e-01,
            7.17009213e-02,  1.56106851e-02, -0.00468173;

    m_fr_rl_com_y.resize(21);
    m_fr_rl_com_y << 2.34042170e-02,  5.48680900e-03, -1.66045839e-02,
            -2.74308950e-02,  1.51090057e-01,  2.42060292e-04,
            -6.06831151e-03,  4.80238777e-02,  1.12309088e-01,
            9.23914769e-03,  8.21521957e-03,  4.18897975e-02,
            8.54691304e-03, -7.58887254e-03,  7.55532076e-02,
            2.58800108e-01,  8.00942563e-02,  2.22827815e-01,
            -2.30548156e-02, -1.00312683e-02, -0.00333011;

    m_fr_rl_com_theta.resize(21);
    m_fr_rl_com_theta << -0.00661641,  0.00117121,  0.08456548,  0.00155278, -0.00260304,
            0.02916036, -0.0070565 ,  0.01170454, -0.04534085, -0.01038279,
            -0.08539721,  0.05447219,  0.01954089, -0.04428893,  0.00016834,
            0.00976488, -0.03258159, -0.0114951 ,  0.01034275, -0.00616154, 0.0008823;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(21);
    m_fl_rr_com_x << 1.95381409e-03, -1.59354717e-02, -2.51731596e-02,
            1.20968452e-01,  3.53030037e-02, -6.28179889e-05,
            3.21341453e-02,  2.95578674e-02,  4.76921099e-02,
            -1.87995344e-03, -3.59223846e-03,  6.36765154e-02,
            2.15180751e-01, -2.06471610e-03,  8.61543005e-02,
            -8.13064732e-02,  7.57800803e-02, -9.67839525e-02,
            1.91382153e-01,  3.40313460e-03, -0.00462337;
    m_fl_rr_com_y.resize(21);
    m_fl_rr_com_y << -2.57526733e-02,  1.15113842e-02, -3.01514800e-02,
            3.32356307e-02,  1.52764565e-01, -1.56411481e-04,
            3.89400513e-02,  1.59985544e-02, -1.22343026e-02,
            -2.17108622e-03, -6.09363857e-02,  7.48597376e-02,
            7.78185885e-02,  2.27045865e-01, -4.15819576e-02,
            1.78980234e-02, -3.82764959e-02,  3.82597761e-03,
            7.10120050e-02,  2.23208244e-01, 0.00103609;
    m_fl_rr_com_theta.resize(21);
    m_fl_rr_com_theta << 0.00343642,  0.0027297 ,  0.07613084, -0.00175182, -0.00254914,
            0.02825271, -0.00225602, -0.00306501, -0.00593673,  0.00566103,
            0.05009655,  0.07933354,  0.03741864, -0.00270922,  0.02961297,
            0.00568369,  0.0221484 , -0.0010134 ,  0.03138302,  0.00456071, -0.00081345;

    // FL models coefficients
    m_fl_swinging_x.resize(21);
    m_fl_swinging_x << 5.68668948e-02, -4.19676720e-03, -3.15967180e-02,
            2.16065034e-01,  2.52761234e-02, -1.08586754e-02,
            2.96815332e-02,  1.56788615e-02,  2.19362357e-02,
            4.74159167e-03,  7.71192507e-02, -1.49228318e-04,
            -2.48351882e-02,  1.92184409e-01, -2.15339425e-02,
            -6.03416658e-02,  7.89134234e-03, -2.98951341e-02,
            -1.89917122e-03,  1.53707379e-01, -0.00696629;
    m_fl_swinging_y.resize(21);
    m_fl_swinging_y << -0.01285211,  0.06617754, -0.00037633,  0.02153827,  0.22228335,
            0.02251766,  0.02361662,  0.02958602, -0.03437496, -0.00405193,
            -0.0938188 ,  0.09303467,  0.00615358, -0.10310938, -0.0719153 ,
            -0.08101959, -0.05691183, -0.04875078,  0.02864874, -0.1214641, -0.0006146;

    // FR models coefficients
    m_fr_swinging_x.resize(21);
    m_fr_swinging_x << 0.05574284,  0.01435029,  0.0313532 ,  0.22243261, -0.02589252,
            0.01058334,  0.00516258, -0.0394547 ,  0.0185532 ,  0.00051325,
            0.11906083,  0.00169464,  0.02944117,  0.06573215, -0.15231285,
            -0.07912396, -0.12269827, -0.08068744,  0.0202072 ,  0.04245839, -0.00703511;
    m_fr_swinging_y.resize(21);
    m_fr_swinging_y << 0.01651946,  0.0461886 , -0.0250572 , -0.01670357,  0.23982355,
            -0.02373036,  0.02814047,  0.05140617,  0.10980504,  0.01656614,
            0.04823836, -0.03509313, -0.09667832, -0.08705281, -0.11000973,
            -0.02117264, -0.09498864, -0.02625809, -0.10019543, -0.0908613, -0.00115294;

    // RL models coefficients
    m_rl_swinging_x.resize(21);
    m_rl_swinging_x << 0.06208483,  0.01374186, -0.0018572 ,  0.21806331, -0.01954891,
            -0.01085404,  0.00516673, -0.03726974, -0.00749375,  0.00050479,
            0.13682024, -0.07633658,  0.02634592,  0.060389  , -0.13001305,
            -0.10581256, -0.11743465, -0.08473655,  0.04003359,  0.04432071, -0.00895347;
    m_rl_swinging_y.resize(21);
    m_rl_swinging_y << 0.01651946,  0.0461886 , -0.0250572 , -0.01670357,  0.23982355,
            -0.02373036,  0.02814047,  0.05140617,  0.10980504,  0.01656614,
            0.04823836, -0.03509313, -0.09667832, -0.08705281, -0.11000973,
            -0.02117264, -0.09498864, -0.02625809, -0.10019543, -0.0908613, -0.00115294;

    // RR models coefficients
    m_rr_swinging_x.resize(21);
    m_rr_swinging_x << 0.06344051, -0.00716245,  0.00201564,  0.21204235,  0.01966099,
            0.01090911,  0.02709588,  0.0213779 ,  0.00310731,  0.00835518,
            0.07521112,  0.07586045, -0.00420039,  0.19430828, -0.01179764,
            -0.05298584,  0.01149527, -0.04887089, -0.00773726,  0.16839448, -0.00897636;
    m_rr_swinging_y.resize(21);
    m_rr_swinging_y << -0.0155568 ,  0.0643704 , -0.03810496,  0.02307837,  0.22379678,
            -0.02321892,  0.03002271,  0.02417401, -0.00313337, -0.01086804,
            -0.10451044, -0.00223342,  0.01722882, -0.10173892, -0.09181096,
            -0.06068971, -0.05344539, -0.03952463,  0.01821698, -0.13989253, -0.00233286;
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
void Model::prediction(double p_previousVelocityX,
                       double p_previousVelocityY,
                       double p_previousAngularVelocity,
                       double p_nextVelocityX,
                       double p_nextVelocityY,
                       double p_nextAngularVelocity,
                       const geometry_msgs::Twist &p_odomVelocityState,
                       const FeetConfiguration &p_currentFeetConfiguration,
                       std::vector<double> &p_predictions) {
    // Common input to all models
    Eigen::VectorXd l_modelInput(21);
    l_modelInput << p_previousVelocityX,
            p_previousVelocityY,
            p_previousAngularVelocity,
            p_nextVelocityX,
            p_nextVelocityY,
            p_nextAngularVelocity,
            p_odomVelocityState.linear.x,
            p_odomVelocityState.linear.y,
            p_odomVelocityState.linear.z,
            p_odomVelocityState.angular.x,
            p_odomVelocityState.angular.y,
            p_odomVelocityState.angular.z,
            p_currentFeetConfiguration.flCoM.x,
            p_currentFeetConfiguration.flCoM.y,
            p_currentFeetConfiguration.frCoM.x,
            p_currentFeetConfiguration.frCoM.y,
            p_currentFeetConfiguration.rlCoM.x,
            p_currentFeetConfiguration.rlCoM.y,
            p_currentFeetConfiguration.rrCoM.x,
            p_currentFeetConfiguration.rrCoM.y,
            1;

    ROS_DEBUG_STREAM("FL CoM Pose: " << p_currentFeetConfiguration.flCoM.x << ", " << p_currentFeetConfiguration.flCoM.y);
    ROS_DEBUG_STREAM("FR CoM Pose: " << p_currentFeetConfiguration.frCoM.x << ", " << p_currentFeetConfiguration.frCoM.y);
    ROS_DEBUG_STREAM("RL CoM Pose: " << p_currentFeetConfiguration.rlCoM.x << ", " << p_currentFeetConfiguration.rlCoM.y);
    ROS_DEBUG_STREAM("RR CoM Pose: " << p_currentFeetConfiguration.rrCoM.x << ", " << p_currentFeetConfiguration.rrCoM.y);

    // FR/RL are swinging
    if (p_currentFeetConfiguration.fr_rl_swinging) {
        p_predictions[0] = m_fr_rl_com_x * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y * l_modelInput;

        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        p_predictions[4] = m_fr_swinging_x * l_modelInput;
        p_predictions[5] = m_fr_swinging_y * l_modelInput;

        p_predictions[6] = m_rl_swinging_x * l_modelInput;
        p_predictions[7] = m_rl_swinging_y * l_modelInput;

        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;

        p_predictions[10] = m_fr_rl_com_theta * l_modelInput;
    }
        // FL/RR are swinging
    else {
        p_predictions[0] = m_fl_rr_com_x * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y * l_modelInput;

        p_predictions[2] = m_fl_swinging_x * l_modelInput;
        p_predictions[3] = m_fl_swinging_y * l_modelInput;

        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

        p_predictions[8] = m_rr_swinging_x * l_modelInput;
        p_predictions[9] = m_rr_swinging_y * l_modelInput;

        p_predictions[10] = m_fl_rr_com_theta * l_modelInput;
    }
}

/**
  * Compute new CoM in world coordinates.
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

    // Get yaw rotation in quaternion form
    tf2::Quaternion l_velocityCommandQuaternion;
    l_velocityCommandQuaternion.setRPY(0, 0, p_predictedCoMDisplacementTheta);

    // Apply rotation command rotation to CoM quaternion
    tf2::Quaternion l_newCoMRotation = p_currentWorldCoordinatesCoM.q * l_velocityCommandQuaternion;
    l_newCoMRotation.normalize();

    // Set new CoM rotation
    p_newWorldCoordinatesCoM.q = l_newCoMRotation;
}

/**
 * Compute new CoM feet configuration
 * using the newly computed map feet
 * configuration and kinematic transforms.
 *
 * @param p_predictions
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeNewFeetConfiguration(const World3D &p_newWorldCoordinatesCoM,
                                        const std::vector<double> &p_predictions,
                                        const FeetConfiguration &p_currentFeetConfiguration,
                                        FeetConfiguration &p_newFeetConfiguration) {
    // Map poses
    p_newFeetConfiguration.flMap.x = p_currentFeetConfiguration.flMap.x + p_predictions[2];
    p_newFeetConfiguration.flMap.y = p_currentFeetConfiguration.flMap.y + p_predictions[3];

    p_newFeetConfiguration.frMap.x = p_currentFeetConfiguration.frMap.x + p_predictions[4];
    p_newFeetConfiguration.frMap.y = p_currentFeetConfiguration.frMap.y + p_predictions[5];

    p_newFeetConfiguration.rlMap.x = p_currentFeetConfiguration.rlMap.x + p_predictions[6];
    p_newFeetConfiguration.rlMap.y = p_currentFeetConfiguration.rlMap.y + p_predictions[7];

    p_newFeetConfiguration.rrMap.x = p_currentFeetConfiguration.rrMap.x + p_predictions[8];
    p_newFeetConfiguration.rrMap.y = p_currentFeetConfiguration.rrMap.y + p_predictions[9];

    // CoM Poses
    p_newFeetConfiguration.flCoM.x = p_newFeetConfiguration.flMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.flCoM.y = p_newFeetConfiguration.flMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.frCoM.x = p_newFeetConfiguration.frMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.frCoM.y = p_newFeetConfiguration.frMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.rlCoM.x = p_newFeetConfiguration.rlMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.rlCoM.y = p_newFeetConfiguration.rlMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.rrCoM.x = p_newFeetConfiguration.rrMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.rrCoM.y = p_newFeetConfiguration.rrMap.y - p_newWorldCoordinatesCoM.y;

//    ROS_INFO_STREAM("Planner: CoM (MAP) " << p_newWorldCoordinatesCoM.x << ", " << p_newWorldCoordinatesCoM.y);
//
//
//    ROS_INFO_STREAM("Model: FL (MAP) " << p_newFeetConfiguration.flMap.x << ", " << p_newFeetConfiguration.flMap.y);
//    ROS_INFO_STREAM("Model: FL (COM) " << p_newFeetConfiguration.flCoM.x << ", " << p_newFeetConfiguration.flCoM.y << "\n");
//
//    ROS_INFO_STREAM("Model: FR (MAP) " << p_newFeetConfiguration.frMap.x << ", " << p_newFeetConfiguration.frMap.y);
//    ROS_INFO_STREAM("Model: FL (COM) " << p_newFeetConfiguration.frCoM.x << ", " << p_newFeetConfiguration.frCoM.y << "\n");
//
//    ROS_INFO_STREAM("Model: RL (MAP) " << p_newFeetConfiguration.rlMap.x << ", " << p_newFeetConfiguration.rlMap.y);
//    ROS_INFO_STREAM("Model: RL (COM) " << p_newFeetConfiguration.rlCoM.x << ", " << p_newFeetConfiguration.rlCoM.y << "\n");
//
//    ROS_INFO_STREAM("Model: RR (MAP) " << p_newFeetConfiguration.rrMap.x << ", " << p_newFeetConfiguration.rrMap.y);
//    ROS_INFO_STREAM("Model: RR (COM) " << p_newFeetConfiguration.rrCoM.x << ", " << p_newFeetConfiguration.rrCoM.y << "\n");

    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;
}

/**
 * Predicts new feet configuration using
 * the learnt models and extracts new CoM
 * from them.
 *
 * @param p_previousVelocity
 * @param p_nextVelocity
 * @param p_action
 * @param p_currentWorldCoordinatesCoM
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 * @param p_newWorldCoordinatesCoM
 */
void Model::predictNextState(double p_previousVelocity,
                             double p_nextVelocity,
                             const Action &p_action,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    // Predict feet and CoM displacements
    std::vector<double> l_predictions(11);
    prediction(p_action.x * p_previousVelocity,
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

    // Compute new CoM
    computeNewCoM(l_predictions[0],
                  l_predictions[1],
                  l_predictions[10],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    // Compute new feet configuration
    computeNewFeetConfiguration(p_newWorldCoordinatesCoM,
                                l_predictions,
                                p_currentFeetConfiguration,
                                p_newFeetConfiguration);

    // Change swinging feet pair
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

    // Publish predicted CoM and feet poses
    int j = 0;
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
    l_footCommonMarker.scale.x = 0.04;
    l_footCommonMarker.scale.y = 0.04;
    l_footCommonMarker.scale.z = 0.04;
    l_footCommonMarker.color.r = 0;
    l_footCommonMarker.color.g = 1;
    l_footCommonMarker.color.b = 0;
    l_footCommonMarker.color.a = 0.7;

    visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
    l_CoMMarker.id = j++;
    l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x - 0.33118;
    l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
    l_CoMMarker.pose.position.z = 0;

    visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
    l_flFootMarker.id = j++;
    l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
    l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
    l_flFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
    l_frFootMarker.id = j++;
    l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
    l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
    l_frFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
    l_rlFootMarker.id = j++;
    l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
    l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
    l_rlFootMarker.pose.position.z = 0;

    visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
    l_rrFootMarker.id = j++;
    l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
    l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
    l_rrFootMarker.pose.position.z = 0;

    visualization_msgs::MarkerArray l_pathFeetConfiguration;
    l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
    l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
    l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

    m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
    ros::Duration(1).sleep();
}