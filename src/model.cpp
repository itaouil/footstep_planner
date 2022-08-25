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
    m_fr_rl_com_x.resize(15);
    m_fr_rl_com_x << 1.01320462e-01, -5.55111512e-17,  0.00000000e+00,
            7.43222931e-02, -1.11022302e-16, -8.32667268e-17,
            3.97155487e-01, -3.72202880e-01,  2.32936105e-01,
            2.62902628e-01, -2.28457695e-01,  1.98860449e-01,
            2.83006097e-01,  2.34632940e-02, -0.07492087;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << -4.67354517e-03, -6.63531730e-17,  1.38777878e-17,
            -5.44197758e-03,  0.00000000e+00,  0.00000000e+00,
            -8.17415655e-02,  1.30031316e-01, -1.43943314e-01,
            -3.87208199e-01,  1.52531755e-01, -1.51384581e-01,
            8.57435717e-02,  2.08865560e-01, 0.09721618;

    m_fr_rl_com_theta.resize(15);
    m_fr_rl_com_theta << 2.14266772e-02, -1.38777878e-17, -1.38777878e-17,
            -1.24650599e-02,  3.12250226e-17,  4.16333634e-17,
            -6.40214008e-02,  1.79224563e-02, -1.16235715e-01,
            -2.67890125e-01,  1.20880304e-01,  1.84015317e-01,
            -4.19143537e-03, -1.01382016e-01, 0.00209039;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 2.55474889e-01, -9.02056208e-17,  1.11022302e-16,
            -1.06432936e-01,  5.55111512e-17,  2.77555756e-17,
            8.54200014e-02, -2.20566290e-04,  2.32940456e-01,
            3.82258293e-01,  6.24058844e-01, -7.79810273e-02,
            -2.03161529e-01, -2.67317612e-01, 0.06021609;

    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << -2.05499085e-02, -1.21430643e-17,  9.02056208e-17,
            5.85800789e-04, -2.77555756e-17,  4.16333634e-17,
            9.40067846e-02, -2.85924504e-01, -2.42484540e-02,
            8.04912927e-02,  6.40799915e-02,  1.07140517e-01,
            -1.48100711e-01, -1.59688105e-01, -0.02693472;

    m_fl_rr_com_theta.resize(15);
    m_fl_rr_com_theta << -5.42888016e-02,  6.93889390e-18,  9.71445147e-17,
            3.78201854e-02,  1.11022302e-16,  1.66533454e-16,
            2.16411521e-01, -3.78116420e-01,  3.31331453e-01,
            2.77408716e-01, -2.69011456e-01, -3.36199324e-01,
            -1.97770972e-01,  7.38964679e-02, -0.10753502;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 2.75128539e-01,  5.55111512e-17, -2.22044605e-16,
            -4.90190824e-02, -8.32667268e-17, -2.22044605e-16,
            -9.77484332e-01, -6.74978202e-02,  5.39225677e-01,
            2.85760039e-01,  7.26984017e-01,  4.74330926e-02,
            -2.64292811e-01, -2.17202346e-01, 0.23595124;

    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << -4.69329271e-03,  1.48752538e-16,  3.46944695e-17,
            -3.12448784e-02, -6.93889390e-17, -5.55111512e-17,
            9.93743494e-02, -8.24554160e-01,  1.16979790e-01,
            2.88328439e-01, -1.64376979e-03,  7.86025764e-02,
            -2.16598531e-01, -2.25832176e-01, -0.00487445;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << 2.27634714e-01, -2.77555756e-16,  5.55111512e-17,
            6.15357962e-02, -1.11022302e-16,  0.00000000e+00,
            4.61998135e-01, -3.42917102e-01, -7.81238121e-01,
            1.94020531e-01, -2.98059631e-01,  4.26317499e-01,
            3.84603496e-01, -1.58825466e-01, 0.0945918;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << 2.89293859e-02,  0.00000000e+00,  5.55111512e-17,
            -3.77057926e-02,  0.00000000e+00,  1.11022302e-16,
            -3.14136573e-01,  5.33129593e-01, -1.38895066e-01,
            -5.97387596e-01,  1.77289318e-01, -9.82415530e-01,
            3.21649951e-01,  4.47731778e-01, 0.28026587;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << 2.06035416e-01, -5.55111512e-17, -1.11022302e-16,
            8.86744746e-02, -2.22044605e-16, -2.22044605e-16,
            5.69183801e-01, -4.27866060e-01,  2.87200561e-01,
            2.95710415e-01, -1.36025528e+00,  4.56892225e-01,
            2.62223078e-01, -1.98319082e-01, -0.4818469;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y <<  2.89293859e-02,  0.00000000e+00,  5.55111512e-17,
            -3.77057926e-02,  0.00000000e+00,  1.11022302e-16,
            -3.14136573e-01,  5.33129593e-01, -1.38895066e-01,
            -5.97387596e-01,  1.77289318e-01, -9.82415530e-01,
            3.21649951e-01,  4.47731778e-01, 0.28026587;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << 2.82719982e-01,  1.66533454e-16,  4.44089210e-16,
            -5.96725602e-02,  1.38777878e-16,  1.66533454e-16,
            1.95097037e-01, -2.44264969e-01,  5.88833096e-01,
            3.54226281e-01,  6.99170463e-01,  1.58623184e-01,
            -1.45174788e+00, -3.27025609e-01, -0.37073426;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y << -1.34607285e-02,  1.31838984e-16,  1.70002901e-16,
            -2.99588697e-02, -5.55111512e-17, -2.08166817e-16,
            9.11113195e-02, -3.05566666e-01, -8.33596010e-03,
            2.95800332e-01,  1.29373490e-01,  1.73964119e-01,
            -2.01566050e-01, -8.01167950e-01, -0.08681985;
}

//void Model::setModelsCoefficients() {
//    // CoM models coefficients when FR/RL are swinging
//    m_fr_rl_com_x.resize(15);
//    m_fr_rl_com_x << -2.49115792e-02,  3.33066907e-16,  1.38777878e-15,
//            1.06215824e-01, -6.66133815e-16,  0.00000000e+00,
//            -1.11129215e+00,  1.34543096e+00,  1.30790539e-01,
//            3.15836432e+00, -5.50468783e-01, -3.20066543e+00,
//            1.70392303e+00, -1.21131622e+00, 1.0240498;
//
//    m_fr_rl_com_y.resize(15);
//    m_fr_rl_com_y << -1.15981497e-02,  8.97719399e-17,  2.63677968e-16,
//            2.98301295e-02,  8.84708973e-17,  1.38777878e-17,
//            -6.78566946e-02,  2.49849749e-01,  5.94595147e-02,
//            3.40419972e-01, -8.44270366e-02, -7.42223009e-01,
//            4.48011952e-02,  1.78176584e-01, 0.11919596;
//
//    m_fr_rl_com_theta.resize(15);
//    m_fr_rl_com_theta << -3.39744268e-04, -2.03287907e-17, -2.86229374e-17,
//            3.17509570e-03, -1.73472348e-17,  1.38777878e-17,
//            -1.71314943e-02,  3.76120708e-02, -1.62190847e-02,
//            -4.17056488e-02,  3.04005453e-02,  5.89117229e-02,
//            1.57648707e-02, -3.31176267e-02, 0.00157585;
//
//    // CoM models coefficients when FL/RR are swinging
//    m_fl_rr_com_x.resize(15);
//    m_fl_rr_com_x << 1.49425716e-02,  8.88178420e-16,  2.22044605e-16,
//            1.13322692e-01, -1.11022302e-15, -2.22044605e-16,
//            -8.32254517e-01, -7.47692972e-01,  1.01401612e+00,
//            -3.37744098e-01, -5.69206891e-01,  4.44833782e-01,
//            7.40231915e-01,  6.16007568e-01, 0.07213049;
//
//    m_fl_rr_com_y.resize(15);
//    m_fl_rr_com_y << 2.01061822e-03, -1.59594560e-16, -5.55111512e-17,
//            -2.84083652e-02,  4.00721123e-16,  4.16333634e-17,
//            3.32538690e-02, -2.22784381e-02, -2.83648172e-01,
//            1.73305070e-01,  3.09533288e-01,  3.58611466e-01,
//            -5.30169914e-02, -4.81369075e-01, 0.04840229;
//
//    m_fl_rr_com_theta.resize(15);
//    m_fl_rr_com_theta << -1.34623669e-03,  6.93889390e-17,  2.08166817e-17,
//            -3.22547567e-03, -1.73472348e-16,  2.77555756e-17,
//            -4.10206047e-02, -7.35003910e-03,  1.27384461e-01,
//            6.50635021e-02, -1.16517707e-01, -7.54674972e-02,
//            2.92143317e-02,  3.90481477e-02, -0.02347458;
//
//    // FL models coefficients
//    m_fl_swinging_x.resize(15);
//    m_fl_swinging_x << -1.65212809e-02, -4.44089210e-16, -5.55111512e-17,
//            3.05471858e-01, -1.66533454e-16, -1.38777878e-16,
//            -1.82220424e+00, -3.72930764e-01,  6.34679512e-02,
//            -7.21308518e-01,  7.22890998e-01,  5.01791567e-01,
//            7.69727237e-01,  5.56016887e-01, 0.79274179;
//
//    m_fl_swinging_y.resize(15);
//    m_fl_swinging_y << 6.28315932e-02,  7.28583860e-16,  3.88578059e-16,
//            -6.72277297e-02,  9.43689571e-16, -3.46944695e-16,
//            5.05572846e-01, -5.06277613e-01, -1.38884434e+00,
//            -9.34526724e-02,  1.19384983e+00,  1.03736099e+00,
//            -4.16524238e-01, -8.76821954e-01, 0.24082866;
//
//    // FR models coefficients
//    m_fr_swinging_x.resize(15);
//    m_fr_swinging_x << -4.75679792e-02,  4.44089210e-16,  1.33226763e-15,
//            2.97867469e-01, -9.99200722e-16,  2.77555756e-16,
//            -1.31718635e+00,  1.44027172e+00, -7.37392382e-01,
//            2.57336397e+00, -6.04126651e-01, -2.86334806e+00,
//            2.19850757e+00, -1.09605516e+00, 1.29014651;
//    m_fr_swinging_y.resize(15);
//    m_fr_swinging_y << -4.62821480e-02,  3.50414142e-16,  1.52655666e-16,
//            4.94955553e-02,  1.62370117e-15, -8.88178420e-16,
//            2.20836762e+00, -3.95564950e-02, -3.16987161e-01,
//            3.02671675e-01,  2.66564441e-01, -1.44223855e+00,
//            -2.10061209e+00,  8.78044488e-01, -0.62304166;
//
//    // RL models coefficients
//    m_rl_swinging_x.resize(15);
//    m_rl_swinging_x << -4.99745487e-02,  4.44089210e-16,  2.10942375e-15,
//            3.00266016e-01, -5.55111512e-16,  8.60422844e-16,
//            -1.48786725e+00,  1.44747550e+00,  2.85823133e-01,
//            2.64756152e+00, -1.62870421e+00, -2.97738482e+00,
//            2.36998032e+00, -1.12206195e+00, 0.86803981;
//    m_rl_swinging_y.resize(15);
//    m_rl_swinging_y <<  -4.62821480e-02,  3.50414142e-16,  1.52655666e-16,
//            4.94955553e-02,  1.62370117e-15, -8.88178420e-16,
//            2.20836762e+00, -3.95564950e-02, -3.16987161e-01,
//            3.02671675e-01,  2.66564441e-01, -1.44223855e+00,
//            -2.10061209e+00,  8.78044488e-01, -0.62304166;
//
//    // RR models coefficients
//    m_rr_swinging_x.resize(15);
//    m_rr_swinging_x << -2.12722702e-02, -2.22044605e-16,  5.55111512e-17,
//            3.07815893e-01,  1.11022302e-16, -3.05311332e-16,
//            -7.84851657e-01, -4.60585789e-01, -3.94081915e-02,
//            -6.49115861e-01,  8.24611757e-01,  4.68265821e-01,
//            -2.69368127e-01,  6.63583128e-01, 0.34665231;
//    m_rr_swinging_y.resize(15);
//    m_rr_swinging_y << 2.94278278e-02,  1.28716482e-15,  5.96744876e-16,
//            -4.54636556e-02,  1.15359111e-15, -1.24900090e-16,
//            4.15687426e-01,  1.82894312e-01, -1.48269095e+00,
//            2.84728612e-01,  1.35800803e+00,  8.38018878e-01,
//            -3.72736013e-01, -1.61966451e+00, 0.22938063;
//}

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

    ROS_DEBUG_STREAM("Model input: " << l_modelInput);
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
    l_footCommonMarker.scale.x = 0.05;
    l_footCommonMarker.scale.y = 0.035;
    l_footCommonMarker.scale.z = 0.035;
    l_footCommonMarker.color.r = 0;
    l_footCommonMarker.color.g = 1;
    l_footCommonMarker.color.b = 0;
    l_footCommonMarker.color.a = 0.7;

    visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
    l_CoMMarker.id = j++;
    l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
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