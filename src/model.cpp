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
    m_fr_rl_com_x << 1.45274660e-02,  1.11022302e-16,  5.55111512e-17,
            1.21480029e-01,  0.00000000e+00, -2.22044605e-16,
            4.59183036e-01, -1.08546195e+00, -2.72558095e-01,
            5.21936188e-01, -3.38240962e-01,  1.65537374e-01,
            -1.89407942e-02,  8.96852488e-01, 0.16325123;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << 3.20974179e-03,  1.19695920e-16, -2.42861287e-17,
            -5.61711439e-03,  1.11022302e-16,  5.55111512e-17,
            2.21675430e-02,  9.67680434e-02, -1.22439462e-01,
            -3.63380029e-01,  2.00918606e-01, -4.12373699e-01,
            6.38717439e-02,  1.09094404e-01, 0.09900231;

    m_fr_rl_com_theta.resize(15);
    m_fr_rl_com_theta << -1.61920617e-03, -2.42861287e-17, -3.12250226e-17,
            2.26519780e-03,  6.93889390e-18,  1.38777878e-17,
            -6.60284389e-02,  1.05302352e-01, -2.36640580e-02,
            -2.15212000e-02, -4.06046357e-02,  2.23379477e-02,
            3.11566249e-02, -8.97357333e-02, -0.0110588;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 4.52958116e-02,  5.55111512e-17, -5.55111512e-17,
            1.17559859e-01, -2.49800181e-16,  2.77555756e-17,
            -3.76058725e-01,  3.65030691e-02,  5.07825311e-01,
            9.24184008e-01, -3.14873512e-01, -5.99854750e-01,
            -4.17385907e-01, -8.97012315e-01, -0.14768165;

    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << -4.09720989e-03, -4.68375339e-17, -6.96057795e-17,
            -2.65430680e-04,  0.00000000e+00, -1.11022302e-16,
            -9.78155604e-02, -6.39772941e-01,  5.51243049e-02,
            2.10072324e-01, -5.94327029e-02,  2.27566328e-01,
            4.61211123e-02, -2.35416061e-01, 0.05630789;

    m_fl_rr_com_theta.resize(15);
    m_fl_rr_com_theta << -2.06496524e-03,  1.73472348e-18, -4.33680869e-18,
            -5.77681429e-03, -1.73472348e-17, -2.77555756e-17,
            -7.33784330e-03, -1.05762075e-01,  3.86233177e-02,
            7.52059702e-02, -1.12792307e-02, -2.50944653e-02,
            4.56671218e-02,  1.07287847e-01, 0.04288928;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 6.85519221e-02, -1.11022302e-16, -3.33066907e-16,
            2.01715553e-01, -2.49800181e-16,  3.60822483e-16,
            -1.41099410e+00, -3.68749740e-02,  6.51826194e-01,
            1.42784351e+00, -4.64835955e-01, -5.19789116e-01,
            -7.89292004e-01, -1.46466502e+00, -0.07434123;

    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << -9.35006898e-03, -7.45931095e-17, -1.00613962e-16,
            4.71894356e-04, -2.22044605e-16, -1.11022302e-16,
            -1.91268911e-01, -1.70553196e+00,  8.97027490e-02,
            5.33735407e-01, -1.53124653e-01,  5.13288013e-01,
            1.71236294e-02, -5.86363472e-01, 0.13553003;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << 2.05675074e-02, -1.11022302e-16, -1.11022302e-16,
            2.19132907e-01, -1.11022302e-16,  1.11022302e-16,
            4.68616241e-01, -1.47294861e+00, -1.32181760e+00,
            8.81263120e-01, -7.38832561e-01,  1.72107747e-01,
            -4.90660517e-02,  1.05655314e+00, 0.42105654;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << 8.16716731e-03, -9.36750677e-17,  1.31838984e-16,
            -9.47637391e-03,  0.00000000e+00, -1.66533454e-16,
            9.70951400e-02,  3.76339322e-01, -1.20083170e-01,
            -7.77032352e-01,  2.26838811e-01, -1.45441731e+00,
            1.00942212e-02,  3.14341060e-01, 0.13845567;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << 2.05539721e-02, -1.11022302e-16,  1.11022302e-16,
            2.19522760e-01,  0.00000000e+00,  4.44089210e-16,
            4.68019098e-01, -1.48467052e+00, -3.52324776e-01,
            9.16546126e-01, -1.71138679e+00,  2.04332147e-01,
            -5.79293634e-02,  1.01039420e+00, -0.07144999;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y <<  8.16716731e-03, -9.36750677e-17,  1.31838984e-16,
            -9.47637391e-03,  0.00000000e+00, -1.66533454e-16,
            9.70951400e-02,  3.76339322e-01, -1.20083170e-01,
            -7.77032352e-01,  2.26838811e-01, -1.45441731e+00,
            1.00942212e-02,  3.14341060e-01, 0.13845567;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << 6.95139108e-02, -1.11022302e-16,  0.00000000e+00,
            2.02025576e-01, -3.05311332e-16, -3.33066907e-16,
            -4.76977358e-01, -6.90331502e-02,  6.54003083e-01,
            1.47679972e+00, -4.75345412e-01, -5.30164930e-01,
            -1.75380690e+00, -1.52964208e+00, -0.55095271;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y <<  -1.41321835e-02,  4.51028104e-17, -6.24500451e-17,
            -2.83917303e-03, -1.11022302e-16, -1.11022302e-16,
            -1.27478279e-01, -1.08764974e+00,  6.10081889e-02,
            5.25566231e-01, -5.53263506e-02,  5.00222634e-01,
            1.41043188e-02, -1.07131716e+00, 0.01381127;
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