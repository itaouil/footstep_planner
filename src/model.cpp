/*
 * model.cpp
 *
 *  Created on: Aug 24, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include <model.hpp>

/**
 * Constructor
 */
Model::Model(ros::NodeHandle& p_nh):
    m_nh(p_nh), m_listener(m_buffer)
{
    // Feet configuration marker array publisher
    m_feetConfigurationPublisher = m_nh.advertise<visualization_msgs::MarkerArray>(TARGET_FEET_CONFIGURATION_MARKERS_TOPIC, 1);

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
 * the prediction process.
 */
void Model::setModelsCoefficients()
{
    // CoM models coeff. when FR/RL swinging
    m_fr_rl_com_x.resize(12);
    m_fr_rl_com_x << 1.14402016e-01, -1.46263600e-04, -9.49649521e-03,
                     1.04827938e+00, -4.08979242e-02, -1.17332787e-01,
                     1.28137632e-01, -6.63037331e-02, -1.08698563e-01,
                     2.30694608e-01,  1.45460953e-01, -0.07340656;

    m_fr_rl_com_y.resize(12);
    m_fr_rl_com_y << -0.00441734,  0.01496484,  0.01807551,
                     -0.10798794, 1.49287406, -0.12222931,
                     -0.25313078,  0.11289323, -0.20699757,
                     0.16328088, 0.13514436, -0.06957352;

    // CoM models coeff. when FF/RR swinging
    m_fl_rr_com_x.resize(12);
    m_fl_rr_com_x << 7.30663428e-02, -2.27413626e-04,  3.97576644e-03,
                     -2.86325282e-01, -1.87534084e-01,  8.91002469e-01,
                     9.34386382e-02,  2.38419311e-01, -1.22664880e-01,
                     -4.17338480e-02,  2.22460533e-01, 0.02626072;
    m_fl_rr_com_y.resize(12);
    m_fl_rr_com_y << 3.03046899e-03,  1.08289150e-03,  2.15625398e-03,
                     2.45150480e-01, -2.25395201e-01,  7.67700181e-02,
                     1.53923435e+00, -5.94327229e-02, -1.59224712e-01,
                     -2.34142349e-01, -2.60578393e-01, 0.08364543;

    // FL models coeff. in swinging and support mode
    m_fl_support_x.resize(12);
    m_fl_support_x << -1.11334355e-01, -2.94643116e-06,  9.28212369e-03,
                      -1.03928824e+00,  4.86196355e-02,  1.08500023e-01,
                      -1.25932689e-01,  8.29840164e-02,  1.01528087e-01,
                      -2.46714446e-01, -1.56741651e-01, 0.07220234;
    m_fl_support_y.resize(12);
    m_fl_support_y << 0.00441734, -0.01496484, -0.01807551,
                      0.10798794, -1.49287406, 0.12222931,
                      0.25313078, -0.11289323, 0.20699757,
                      -0.16328088, -0.13514436, 0.06957352;
    m_fl_swinging_x.resize(12);
    m_fl_swinging_x << 6.08753071e-02,  6.28929714e-05, -2.90609271e-03,
                      -1.17379331e+00, -3.66375512e-04, -1.14463525e-01,
                      1.28952202e-03,  3.68869541e-01,  6.65510669e-02,
                      5.20313892e-02, -4.48244769e-03, 0.35277527;
    m_fl_swinging_y.resize(12);
    m_fl_swinging_y << 0.00928936,  0.00972964,  0.0033553,
                       -0.02043487, -1.34256373, 0.30067711,
                       0.29503976, -0.36566688,  0.26411568,
                       0.07108397, 0.0303334, 0.04983661;

    // FR models coeff. in swinging and support mode
    m_fr_support_x.resize(12);
    m_fr_support_x << -7.33666490e-02,  1.23537575e-04, -4.05961029e-03,
                      2.54617103e-01,  1.73145475e-01, -8.90993046e-01,
                      -9.48097770e-02, -2.45213436e-01,  1.20721749e-01,
                      7.15458719e-02, -2.06105471e-01, -0.00941408;
    m_fr_support_y.resize(12);
    m_fr_support_y << -3.03046899e-03, -1.08289150e-03, -2.15625398e-03,
                      -2.45150480e-01,  2.25395201e-01, -7.67700181e-02,
                      -1.53923435e+00,  5.94327229e-02,  1.59224712e-01,
                      2.34142349e-01,  2.60578393e-01, -0.08364543;
    m_fr_swinging_x.resize(12);
    m_fr_swinging_x << 9.79712587e-02, -5.12065744e-04,  1.03718990e-02,
                       -6.55896646e-03,  2.10925679e-02, -8.42801966e-01,
                       -4.32587407e-02, -1.65537385e-01,  9.22188799e-02,
                       1.56522035e-01, -5.77347141e-02, 0.12855594;
    m_fr_swinging_y.resize(12);
    m_fr_swinging_y << 0.00214255,  0.02432448,  0.01115724,
                       0.05466127,  0.40734676, 0.103314,
                       -1.28684035, -0.11026708,  0.02444531,
                       -0.10043746, 0.01717326, -0.31804721;

    // RL models coeff. in swinging and support mode
    m_rl_support_x.resize(12);
    m_rl_support_x << -4.95600125e-02,  8.32433424e-04,  3.15635013e-03,
                      2.99094621e-01, -1.24864268e-01, -3.04043468e-02,
                      -2.93021902e-02, -1.29873127e+00,  8.10235198e-03,
                      1.46893566e-01,  1.46861209e-01, -0.34473587;
    m_rl_support_y.resize(12);
    m_rl_support_y << 2.11720198e-03, -8.64614775e-04,  8.20007542e-03,
                      1.84174058e-01,  8.12516416e-02,  2.45078567e-01,
                      -1.81466644e-01, -1.77584919e-01, -1.21201957e+00,
                      -1.62941795e-01,  4.46850808e-01, 0.01254846;
    m_rl_swinging_x.resize(12);
    m_rl_swinging_x << 1.05996590e-01, -1.30723107e-04, -5.80474201e-03,
                       1.45774468e-01, -7.53264487e-02,  1.31225867e-01,
                       3.42267667e-03, -1.12053691e+00,  3.60937062e-02,
                       -4.27068491e-02,  3.39991018e-02, -0.37406442;
    m_rl_swinging_y.resize(12);
    m_rl_swinging_y << -0.00511665,  0.02283127, -0.01346415,
                       0.2578356 ,  0.15286922, -0.15610528,
                       0.04144732,  0.14905619, -1.20841892,
                       -0.17332232, 0.25331673, 0.15898015;

    // RR models coeff. in swinging and support mode
    m_rr_support_x.resize(12);
    m_rr_support_x << -1.07763836e-01,  5.41972110e-04, -1.32055395e-02,
                      -4.17642560e-01, -2.22802510e-01, -5.01796630e-02,
                      1.01830777e-01,  2.51733303e-01, -1.55618734e-01,
                      -7.77373991e-01,  2.97758251e-01, 0.0474829;
    m_rr_support_y.resize(12);
    m_rr_support_y << -0.00406419, -0.01535347,  0.02499548,
                      -0.19024051, -0.25483827, -0.08481939,
                      0.19661478,  0.07660149,  0.26224184,
                      0.17758845, -1.35331986, -0.03545181;
    m_rr_swinging_x.resize(12);
    m_rr_swinging_x << 5.92483902e-02,  5.98820152e-04,  3.74287507e-03,
                       -2.71333921e-01, -1.55467217e-01,  1.02390882e-01,
                       1.85833887e-01,  2.11099138e-01, -1.34491619e-01,
                       -8.86573251e-01,  1.74333171e-01, -0.07321299;
    m_rr_swinging_y.resize(12);
    m_rr_swinging_y << 9.11557581e-04,  7.96752491e-03, -6.12761024e-03,
                       -1.69531771e-01, -3.69152908e-02, -8.70984474e-02,
                       1.09659174e-01,  7.69277098e-02,  3.90956364e-01,
                       1.45873682e-01, -1.22612609e+00, -0.09027069;
}

/**
 * Compute CoM and feet displacements
 * predictions using the learnt model
 * coefficients.
 *
 * @param p_velocityX
 * @param p_velocityY
 * @param p_angularVelocity
 * @param p_currentFeetConfiguration
 * @param p_predictions
 */
void Model::computePredictedDisplacements(const double p_velocityX,
                                          const double p_velocityY,
                                          const double p_angularVelocity,
                                          const FeetConfiguration &p_currentFeetConfiguration,
                                          std::vector<double> &p_predictions)
{
    ROS_INFO("Compute predictions.");

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
    if (p_currentFeetConfiguration.fr_rl_swinging)
    {
        p_predictions[0] = m_fr_rl_com_x * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y * l_modelInput;

        p_predictions[2] = m_fl_support_x * l_modelInput;
        p_predictions[3] = m_fl_support_y * l_modelInput;

        p_predictions[4] = m_fr_swinging_x * l_modelInput;
        p_predictions[5] = m_fr_swinging_y * l_modelInput;

        p_predictions[6] = m_rl_swinging_x * l_modelInput;
        p_predictions[7] = m_rl_swinging_y * l_modelInput;

        p_predictions[8] = m_rr_support_x * l_modelInput;
        p_predictions[9] = m_rr_support_y * l_modelInput;
    }
    // FL/RR are swinging
    else
    {
        p_predictions[0] = m_fl_rr_com_x * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y * l_modelInput;

        p_predictions[2] = m_fl_swinging_x * l_modelInput;
        p_predictions[3] = m_fl_swinging_y * l_modelInput;

        p_predictions[4] = m_fr_support_x * l_modelInput;
        p_predictions[5] = m_fr_support_y * l_modelInput;

        p_predictions[6] = m_rl_support_x * l_modelInput;
        p_predictions[7] = m_rl_support_y * l_modelInput;

        p_predictions[8] = m_rr_swinging_x * l_modelInput;
        p_predictions[9] = m_rr_swinging_y * l_modelInput;
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
                          const World2D &p_currentWorldCoordinatesCoM,
                          World2D &p_newWorldCoordinatesCoM)
{
    ROS_INFO_STREAM("Model: COM movement in x: " << p_predictedCoMDisplacementX);
    ROS_INFO_STREAM("Model: COM movement in y: " << p_predictedCoMDisplacementY);

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
    if (p_angularVelocity != 0)
    {
        // Get yaw rotation in quaternion form
        tf2::Quaternion l_velocityCommandQuaternion;
        l_velocityCommandQuaternion.setRPY(0, 0, p_angularVelocity * TIMESTAMP);

        // Apply rotation command rotation to CoM quaternion
        tf2::Quaternion l_newCoMRotation = p_currentWorldCoordinatesCoM.q * l_velocityCommandQuaternion;
        l_newCoMRotation.normalize();

        // Set new CoM rotation
        p_newWorldCoordinatesCoM.q = l_newCoMRotation;
    }
    else
    {
        p_newWorldCoordinatesCoM.q = p_currentWorldCoordinatesCoM.q;
    }

    ROS_INFO_STREAM("Model: new CoM (" << p_newWorldCoordinatesCoM.x << ", "
                                            << p_newWorldCoordinatesCoM.y << ", "
                                            << p_newWorldCoordinatesCoM.q.x() << ","
                                            << p_newWorldCoordinatesCoM.q.y() << ","
                                            << p_newWorldCoordinatesCoM.q.z() << ","
                                            << p_newWorldCoordinatesCoM.q.w() << ")");
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
                                           FeetConfiguration &p_newFeetConfiguration)
{
    ROS_INFO_STREAM("Model: Prev FL CoM: " << p_currentFeetConfiguration.flCoM.x << ", " << p_currentFeetConfiguration.flCoM.y);
    ROS_INFO_STREAM("Model: Prev FR CoM: " << p_currentFeetConfiguration.frCoM.x << ", " << p_currentFeetConfiguration.frCoM.y);
    ROS_INFO_STREAM("Model: Prev RL CoM: " << p_currentFeetConfiguration.rlCoM.x << ", " << p_currentFeetConfiguration.rlCoM.y);
    ROS_INFO_STREAM("Model: Prev RR CoM: " << p_currentFeetConfiguration.rrCoM.x << ", " << p_currentFeetConfiguration.rrCoM.y << "\n");

    for (auto &prediction: p_relativeStepPredictions)
        ROS_INFO_STREAM(prediction);

    p_newFeetConfiguration.flCoM.x = p_currentFeetConfiguration.flCoM.x + p_relativeStepPredictions[0];
    p_newFeetConfiguration.flCoM.y = p_currentFeetConfiguration.flCoM.y + p_relativeStepPredictions[1];

    p_newFeetConfiguration.frCoM.x = p_currentFeetConfiguration.frCoM.x + p_relativeStepPredictions[2];
    p_newFeetConfiguration.frCoM.y = p_currentFeetConfiguration.frCoM.y + p_relativeStepPredictions[3];

    p_newFeetConfiguration.rlCoM.x = p_currentFeetConfiguration.rlCoM.x + p_relativeStepPredictions[4];
    p_newFeetConfiguration.rlCoM.y = p_currentFeetConfiguration.rlCoM.y + p_relativeStepPredictions[5];

    p_newFeetConfiguration.rrCoM.x = p_currentFeetConfiguration.rrCoM.x + p_relativeStepPredictions[6];
    p_newFeetConfiguration.rrCoM.y = p_currentFeetConfiguration.rrCoM.y + p_relativeStepPredictions[7];

    ROS_INFO_STREAM("Model: New FL CoM: " << p_newFeetConfiguration.flCoM.x << ", " << p_newFeetConfiguration.flCoM.y);
    ROS_INFO_STREAM("Model: New FR CoM: " << p_newFeetConfiguration.frCoM.x << ", " << p_newFeetConfiguration.frCoM.y);
    ROS_INFO_STREAM("Model: New RL CoM: " << p_newFeetConfiguration.rlCoM.x << ", " << p_newFeetConfiguration.rlCoM.y);
    ROS_INFO_STREAM("Model: New RR CoM: " << p_newFeetConfiguration.rrCoM.x << ", " << p_newFeetConfiguration.rrCoM.y << "\n");
}

/**
 * Predicts new feet configuration using
 * the learnt models and extracts new CoM
 * from them.
 *
 * @param p_velocity
 * @param p_action
 * @param p_currentWorldCoordinatesCoM
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 * @param p_newWorldCoordinatesCoM
 */
void Model::predictNewConfiguration(double p_velocity,
                                    const Action &p_action,
                                    const World2D &p_currentWorldCoordinatesCoM,
                                    const FeetConfiguration &p_currentFeetConfiguration,
                                    FeetConfiguration &p_newFeetConfiguration,
                                    World2D &p_newWorldCoordinatesCoM)
{
    // Compute predicted CoM and feet movements
    // using linear model coefficients
    std::vector<double> l_predictions(10);
    computePredictedDisplacements(p_action.x * p_velocity,
                                  p_action.y * p_velocity,
                                  p_action.theta * p_velocity,
                                  p_currentFeetConfiguration,
                                  l_predictions);

    // Compute new CoM
    computeNewCoM(p_action.theta * p_velocity,
                  l_predictions[0],
                  l_predictions[1],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    // Compute feet configuration w.r.t CoM
    std::vector<double> l_relativeStepPredictions = std::vector<double>(l_predictions.begin() + 2,
                                                                        l_predictions.end());
    computeNewCoMFeetConfiguration(l_relativeStepPredictions,
                                   p_currentFeetConfiguration,
                                   p_newFeetConfiguration);

    // Change swinging feet pair
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

//    int j = 0;
//
//    // Populate array
//    visualization_msgs::Marker l_footCommonMarker;
//    l_footCommonMarker.header.stamp = ros::Time::now();
//    l_footCommonMarker.header.frame_id = ROBOT_REFERENCE_FRAME;
//    l_footCommonMarker.type = 2;
//    l_footCommonMarker.action = 0;
//    l_footCommonMarker.lifetime = ros::Duration(4);
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
//    l_flFootMarker.pose.position.x = p_newFeetConfiguration.flCoM.x;
//    l_flFootMarker.pose.position.y = p_newFeetConfiguration.flCoM.y;
//    l_flFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//    l_frFootMarker.id = j++;
//    l_frFootMarker.pose.position.x = p_newFeetConfiguration.frCoM.x;
//    l_frFootMarker.pose.position.y = p_newFeetConfiguration.frCoM.y;
//    l_frFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//    l_rlFootMarker.id = j++;
//    l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlCoM.x;
//    l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlCoM.y;
//    l_rlFootMarker.pose.position.z = 0;
//
//    visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//    l_rrFootMarker.id = j++;
//    l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrCoM.x;
//    l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrCoM.y;
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
//    ros::Duration(5).sleep();
}