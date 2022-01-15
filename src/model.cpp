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
    // CoM models coeff. when FR/RL are swinging
    m_fr_rl_com_x_continuous.resize(18);
    m_fr_rl_com_x_continuous << 1.67703889e-01, -4.57877553e-02, -9.52577939e-02,
            1.83052067e-04, -2.31385764e-03, 7.61571999e-02,
            3.56845376e-03, -3.63856299e-03, 1.30516849e-01,
            -7.53428691e-02, 4.18659096e-01, 2.35614874e-01,
            6.96479953e-02, 1.80526670e-01, 1.37323753e-02,
            1.57726113e-01, 1.30269533e-01, 0.0045297;

    m_fr_rl_com_y_continuous.resize(18);
    m_fr_rl_com_y_continuous << -0.02743412, 0.20984167, 0.14934019, -0.00067403, -0.00179722,
            -0.11849514, -0.00245629, 0.03057901, -0.18730101, 0.26729459,
            -0.3715912, 0.05218765, 0.12329882, 0.05822368, 0.24019054,
            0.06380863, -0.00982783, -0.00276858;

    // CoM models coeff. when FL/RR are swinging
    m_fl_rr_com_x_continuous.resize(18);
    m_fl_rr_com_x_continuous << 0.1583681, 0.04025599, 0.04330785, 0.00056129, 0.00300335,
            0.06321915, -0.00138429, 0.0037453, -0.06826527, 0.30538243,
            -0.0686407, 0.11815135, -0.2891667, 0.10949485, -0.15844472,
            0.21392442, 0.02842542, -0.02424664;
    m_fl_rr_com_y_continuous.resize(18);
    m_fl_rr_com_y_continuous << 0.01937409, 0.19852968, 0.06716851, -0.00271488, -0.00098713,
            0.0955618, -0.00339751, -0.0140157, -0.10065062, 0.04418614,
            0.13699184, -0.0732032, -0.14528297, -0.09966058, -0.0176591,
            -0.05120565, 0.30746906, -0.02464283;

    // FL models coeff.
    m_fl_swinging_x_continuous.resize(18);
    m_fl_swinging_x_continuous << 0.3128645, 0.03854193, 0.06476222, -0.12521594, -0.02748448,
            0.02078511, -0.00565055, -0.01748714, -0.06027348, -0.48875333,
            -0.03628026, 0.13002127, -0.186551, 0.07204461, -0.09984394,
            0.03203002, -0.0295187, 0.09315323;
    m_fl_swinging_y_continuous.resize(18);
    m_fl_swinging_y_continuous << 0.00788368, 0.34168152, 0.08299261, 0.01703037, -0.13684924,
            0.16085338, 0.00797252, -0.04573592, -0.08847974, 0.08675361,
            -0.58730763, -0.0840307, 0.00871576, -0.08967907, -0.06865873,
            -0.0795744, -0.0317027, 0.06396557;

    // FR models coeff.
    m_fr_swinging_x_continuous.resize(18);
    m_fr_swinging_x_continuous << 0.32839572, -0.04714625, -0.11133982, -0.13585002, 0.02782288,
            0.026687, 0.01109038, -0.0081996, 0.12479007, -0.16405499,
            0.39720701, -0.5720436, 0.02019725, 0.01433763, 0.09571478,
            0.20154869, 0.04758636, 0.14757465;
    m_fr_swinging_y_continuous.resize(18);
    m_fr_swinging_y_continuous << -0.02131934, 0.35384246, 0.17069471, -0.019309, -0.15073623,
            -0.21713835, 0.00990064, 0.06069949, -0.16760545, 0.45731106,
            -0.15124097, 0.03922841, -0.45237282, 0.07406216, -0.18157712,
            -0.06872352, -0.00711227, -0.13183908;

    // RL models coeff.
    m_rl_swinging_x_continuous.resize(18);
    m_rl_swinging_x_continuous << 0.32740427, -0.05142719, -0.08826239, -0.13679047, 0.02924439,
            0.02948881, 0.01085036, -0.0059351, 0.1182178, -0.27703602,
            0.40254209, -0.13025778, 0.04832223, -0.45960441, 0.04459591,
            0.31404968, 0.07177345, -0.00203714;
    m_rl_swinging_y_continuous.resize(18);
    m_rl_swinging_y_continuous << -0.01804344, 0.35453721, 0.12839727, -0.01905046, -0.15119163,
            -0.21614303, 0.01034208, 0.06386109, -0.15750954, 0.34054433,
            -0.17589365, 0.09582321, -0.09387381, 0.00225342, -0.58275445,
            0.02364924, -0.03416471, 0.01608741;

    // RR models coeff.
    m_rr_swinging_x_continuous.resize(18);
    m_rr_swinging_x_continuous << 0.3080201, 0.04652164, 0.0542705, -0.12590866, -0.02915219,
            0.03601599, -0.00724633, -0.01551572, -0.07517701, -0.03526347,
            -0.13263748, 0.08920247, -0.25307634, 0.15897949, -0.14097096,
            -0.44989631, 0.04945942, -0.07495486;
    m_rr_swinging_y_continuous.resize(18);
    m_rr_swinging_y_continuous << 0.00048411, 0.34334255, 0.05344302, 0.01748407, -0.13906107,
            0.1623064, 0.00998895, -0.04684145, -0.08470929, 0.03884878,
            -0.15501406, -0.02713333, 0.01583715, -0.077703, -0.09518424,
            -0.00421451, -0.4838934, -0.06104703;
}

/**
 * Populates the respective model
 * coefficients vectors required for
 * the prediction process of discontinuous
 * velocity commands.
 */
void Model::setDiscontinuousModelsCoefficients() {
    // CoM models coeff. when FR/RL swinging
    m_fr_rl_com_x_acceleration.resize(21);
    m_fr_rl_com_x_acceleration << 0.0262345, -0.11382684, 0.03319261, 0.06064813, 0.00522371,
            0.05978128, 0.01230941, -0.01193984, -0.05066692, -0.00278379,
            0.03058728, 0.06760297, -0.0148463, 0.59954507, 0.32433232,
            0.49977088, -0.38565946, -0.00744623, 0.64167684, 0.76648772, 0.10096746;

    m_fr_rl_com_y_acceleration.resize(21);
    m_fr_rl_com_y_acceleration << -0.04060758, 0.14562743, 0.0891601, 0.01735542, 0.04756675,
            0.02126936, -0.00226834, 0.02202925, 0.06451431, 0.01405608,
            -0.00778946, -0.08310267, 0.02748841, 0.09426949, -0.16040192,
            0.28857145, 0.04880829, 0.12883396, 0.11371643, -0.40684101, 0.01054885;

    // CoM models coeff. when FL/RR swinging
    m_fl_rr_com_x_acceleration.resize(21);
    m_fl_rr_com_x_acceleration << 0.02388812, 0.12072007, -0.00153214, 0.06135513, -0.00516689,
            -0.04768366, 0.01284948, 0.00480223, -0.09878998, 0.02392297,
            0.04972781, -0.07514635, 0.27739971, -0.43901943, 0.02963638,
            -0.75670172, 0.58245151, -0.76950869, -0.37429812, -0.17543078, 0.02949681;
    m_fl_rr_com_y_acceleration.resize(21);
    m_fl_rr_com_y_acceleration << 3.46228577e-02, 1.53876199e-01, 1.22390167e-01,
            -1.52389410e-02, 5.39452813e-02, 1.92226499e-02,
            2.92674471e-03, 2.13090530e-02, -5.20527349e-04,
            6.64204068e-03, 2.35308163e-02, -1.22337553e-01,
            1.45084021e-01, 2.14312430e-01, -2.69302963e-02,
            3.63619804e-02, -7.21408304e-02, -5.25549936e-01,
            -6.71586928e-02, 1.20483259e-01, 0.01716566;

    // FL models coeff. in swing and support mode
    m_fl_swinging_x_acceleration.resize(21);
    m_fl_swinging_x_acceleration << 0.02178396, 0.14332711, -0.00989652, 0.16901972, -0.00309409,
            -0.07326355, -0.0966734, -0.02417165, -0.11635287, 0.02519874,
            0.04600941, -0.10278288, -0.21603318, -0.30698127, -0.42649669,
            -0.56464856, 1.3958494, -1.05240091, -0.95479862, -0.4269248, 0.32133342;
    m_fl_swinging_y_acceleration.resize(21);
    m_fl_swinging_y_acceleration << 0.04249836, 0.1745788, 0.10035121, -0.01451949, 0.15749015,
            0.06894747, 0.02260231, -0.10489863, -0.02254711, 0.00690144,
            0.02645862, -0.0943537, 0.2461572, -0.30928044, -0.19689506,
            0.42849008, -0.07621504, -0.57697278, -0.07445186, -0.25892667, 0.13019219;

    // FR models coeff. in swing and support mode
    m_fr_swinging_x_acceleration.resize(21);
    m_fr_swinging_x_acceleration << 0.03760407, -0.01866219, 0.01107833, 0.00220064, 0.00213474,
            -0.00256029, -0.13249545, 0.0317378, -0.03228565, 0.00917183,
            -0.02490379, 0.03488127, -0.16984764, -0.08476807, -0.15184468,
            -0.17669569, -0.20647698, 0.1179173, 0.24807438, 0.01661002, 0.05068404;
    m_fr_swinging_y_acceleration.resize(21);
    m_fr_swinging_y_acceleration << -1.68690166e-02, 2.73038271e-02, 7.81023867e-02,
            -4.60286229e-05, 2.78327872e-03, 3.41755255e-02,
            -2.02517670e-02, -1.39881555e-01, 1.46411543e-03,
            1.82165000e-02, 2.05150792e-02, 4.00658246e-02,
            3.87069073e-02, 9.45557587e-02, 3.49018682e-02,
            -5.30972187e-02, -1.38802050e-01, -4.43088221e-01,
            2.15749997e-01, 6.31968191e-02, 0.0666916;

    // RL models coeff. in swing and support mode
    m_rl_swinging_x_acceleration.resize(21);
    m_rl_swinging_x_acceleration << 0.04476194, -0.01580006, -0.06352495, 0.00278718, 0.00561405,
            -0.02228349, -0.13120207, 0.02592039, 0.00579645, 0.00647071,
            0.00106013, -0.00304598, -0.54001061, -0.17243847, -0.28179652,
            -0.19093308, -0.01642727, 0.24672256, 0.58195364, 0.25602468, 0.32551454;
    m_rl_swinging_y_acceleration.resize(21);
    m_rl_swinging_y_acceleration << -1.68690166e-02, 2.73038271e-02, 7.81023867e-02,
            -4.60286229e-05, 2.78327872e-03, 3.41755255e-02,
            -2.02517670e-02, -1.39881555e-01, 1.46411543e-03,
            1.82165000e-02, 2.05150792e-02, 4.00658246e-02,
            3.87069073e-02, 9.45557587e-02, 3.49018682e-02,
            -5.30972187e-02, -1.38802050e-01, -4.43088221e-01,
            2.15749997e-01, 6.31968191e-02, 0.0666916;

    // RR models coeff. in swing and support mode
    m_rr_swinging_x_acceleration.resize(21);
    m_rr_swinging_x_acceleration << 0.02955762, 0.1432621, -0.04151896, 0.16966638, -0.00349037,
            -0.07295244, -0.09448333, -0.02616144, -0.11079691, 0.02199614,
            0.0346485, -0.11885317, 0.30968237, -0.44383705, -0.5538524,
            -0.46948078, 1.41342596, -1.12849994, -1.52378962, -0.29375091, 0.16684358;
    m_rr_swinging_y_acceleration.resize(21);
    m_rr_swinging_y_acceleration << 0.03638912, 0.18701459, 0.0954422, -0.01548988, 0.1574662,
            -0.00257799, 0.02134739, -0.10280393, -0.02216174, 0.00889261,
            0.03276828, -0.07616043, 0.20511595, 0.09373271, -0.29935101,
            0.24284354, 0.08618278, -0.59123267, -0.04102567, -0.72838794, 0.03393695;
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
                                           const geometry_msgs::Twist &p_odomVelocityState,
                                           const FeetConfiguration &p_currentFeetConfiguration,
                                           std::vector<double> &p_predictions) {
    // Common input to all models
    Eigen::VectorXd l_modelInput(18);
    l_modelInput << p_velocityX,
            p_velocityY,
            p_angularVelocity,
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

    // FR/RL are swinging
    if (p_currentFeetConfiguration.fr_rl_swinging) {
        p_predictions[0] = m_fr_rl_com_x_continuous * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y_continuous * l_modelInput;

        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        p_predictions[4] = m_fr_swinging_x_continuous * l_modelInput;
        p_predictions[5] = m_fr_swinging_y_continuous * l_modelInput;

        p_predictions[6] = m_rl_swinging_x_continuous * l_modelInput;
        p_predictions[7] = m_rl_swinging_y_continuous * l_modelInput;

        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;
    }
        // FL/RR are swinging
    else {
        p_predictions[0] = m_fl_rr_com_x_continuous * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y_continuous * l_modelInput;

        p_predictions[2] = m_fl_swinging_x_continuous * l_modelInput;
        p_predictions[3] = m_fl_swinging_y_continuous * l_modelInput;

        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

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

    // FR/RL are swinging
    if (p_currentFeetConfiguration.fr_rl_swinging) {
        p_predictions[0] = m_fr_rl_com_x_acceleration * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y_acceleration * l_modelInput;

        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        p_predictions[4] = m_fr_swinging_x_acceleration * l_modelInput;
        p_predictions[5] = m_fr_swinging_y_acceleration * l_modelInput;

        p_predictions[6] = m_rl_swinging_x_acceleration * l_modelInput;
        p_predictions[7] = m_rl_swinging_y_acceleration * l_modelInput;

        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;
    }
        // FL/RR are swinging
    else {
        p_predictions[0] = m_fl_rr_com_x_acceleration * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y_acceleration * l_modelInput;

        p_predictions[2] = m_fl_swinging_x_acceleration * l_modelInput;
        p_predictions[3] = m_fl_swinging_y_acceleration * l_modelInput;

        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

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
}

/**
 * Compute new CoM feet configuration
 * using the newly computed map feet
 * configuration and kinematic transforms.
 *
 * @param p_absoluteFootstepPredictions
 * @param p_currentFeetConfiguration
 * @param p_newFeetConfiguration
 */
void Model::computeNewFeetConfiguration(const World3D &p_newWorldCoordinatesCoM,
                                        const std::vector<double> &p_absoluteFootstepPredictions,
                                        const FeetConfiguration &p_currentFeetConfiguration,
                                        FeetConfiguration &p_newFeetConfiguration) {
    // Map poses
    p_newFeetConfiguration.flMap.x += p_absoluteFootstepPredictions[0];
    p_newFeetConfiguration.flMap.y += p_absoluteFootstepPredictions[1];

    p_newFeetConfiguration.frMap.x += p_absoluteFootstepPredictions[2];
    p_newFeetConfiguration.frMap.y += p_absoluteFootstepPredictions[3];

    p_newFeetConfiguration.rlMap.x += p_absoluteFootstepPredictions[4];
    p_newFeetConfiguration.rlMap.y += p_absoluteFootstepPredictions[5];

    p_newFeetConfiguration.rrMap.x += p_absoluteFootstepPredictions[6];
    p_newFeetConfiguration.rrMap.y += p_absoluteFootstepPredictions[7];

    // CoM Poses
    p_newFeetConfiguration.flCoM.x = p_newWorldCoordinatesCoM.x - p_newFeetConfiguration.flMap.x;
    p_newFeetConfiguration.flCoM.y = p_newWorldCoordinatesCoM.y - p_newFeetConfiguration.flMap.y;

    p_newFeetConfiguration.frCoM.x = p_newWorldCoordinatesCoM.x - p_newFeetConfiguration.frMap.x;
    p_newFeetConfiguration.frCoM.y = p_newWorldCoordinatesCoM.y - p_newFeetConfiguration.frMap.x;

    p_newFeetConfiguration.rlCoM.x = p_newWorldCoordinatesCoM.x - p_newFeetConfiguration.rlMap.x;
    p_newFeetConfiguration.rlCoM.y = p_newWorldCoordinatesCoM.y - p_newFeetConfiguration.rlMap.x;

    p_newFeetConfiguration.rrCoM.x = p_newWorldCoordinatesCoM.x - p_newFeetConfiguration.rrMap.x;
    p_newFeetConfiguration.rrCoM.y = p_newWorldCoordinatesCoM.y - p_newFeetConfiguration.rrMap.x;

    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;
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
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    // Predict feet and CoM displacements
    std::vector<double> l_predictions(10);
    if (!p_accelerating) {
        predictContinuousDisplacements(p_action.x * p_previousVelocity,
                                       p_action.y * p_previousVelocity,
                                       p_action.theta * p_previousVelocity,
                                       p_odomVelocityState,
                                       p_currentFeetConfiguration,
                                       l_predictions);
    } else {
        predictDiscontinuousDisplacements(p_action.x * p_previousVelocity,
                                          p_action.y * p_previousVelocity,
                                          p_action.theta * p_previousVelocity,
                                          p_action.x * p_nextVelocity,
                                          p_action.y * p_nextVelocity,
                                          p_action.theta * p_nextVelocity,
                                          p_odomVelocityState,
                                          p_currentFeetConfiguration,
                                          l_predictions);
    }

    // Compute new CoM
    computeNewCoM(p_action.theta * p_nextVelocity,
                  l_predictions[0],
                  l_predictions[1],
                  p_currentWorldCoordinatesCoM,
                  p_newWorldCoordinatesCoM);

    // Compute new feet configuration
    std::vector<double> l_absoluteFootstepPredictions = std::vector<double>(l_predictions.begin() + 2,
                                                                            l_predictions.end());
    computeNewFeetConfiguration(p_newWorldCoordinatesCoM,
                                l_absoluteFootstepPredictions,
                                p_currentFeetConfiguration,
                                p_newFeetConfiguration);

    // Change swinging feet pair
    p_newFeetConfiguration.fr_rl_swinging = !p_currentFeetConfiguration.fr_rl_swinging;

    // Publish predicted footstep and CoM
//    int j = 0;

//    // Populate array
//    visualization_msgs::Marker l_footCommonMarker;
//    l_footCommonMarker.header.stamp = ros::Time::now();
//    l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//    l_footCommonMarker.type = 2;
//    l_footCommonMarker.action = 0;
//    l_footCommonMarker.lifetime = ros::Duration(0.5);
//    l_footCommonMarker.pose.orientation.x = 0;
//    l_footCommonMarker.pose.orientation.y = 0;
//    l_footCommonMarker.pose.orientation.z = 0;
//    l_footCommonMarker.pose.orientation.w = 1;
//    l_footCommonMarker.scale.x = 0.035;
//    l_footCommonMarker.scale.y = 0.035;
//    l_footCommonMarker.scale.z = 0.035;
//    l_footCommonMarker.color.r = 0;
//    l_footCommonMarker.color.g = 1;
//    l_footCommonMarker.color.b = 0;
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
//    ros::Duration(1).sleep();
}