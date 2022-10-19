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
    m_fr_rl_com_x.resize(15);
    m_fr_rl_com_x << 1.55476542e-02,  2.22044605e-16, -2.22044605e-16,
            1.24370237e-01,  2.22044605e-16,  4.44089210e-16,
            6.66308105e-01, -4.34299860e-01, -1.48564026e-01,
            5.89749574e-01, -5.80109199e-01,  6.61961669e-01,
            -3.40262714e-01,  7.81067377e-01, -0.21192527;

    m_fr_rl_com_y.resize(15);
    m_fr_rl_com_y << 1.08352146e-03,  6.67868538e-17, -5.20417043e-18,
            1.18881486e-03, -1.11022302e-16,  0.00000000e+00,
            -9.88107745e-03,  9.09466687e-02,  8.36384168e-04,
            -8.93606112e-02,  8.58723839e-02, -6.01935483e-01,
            7.68458080e-02,  1.60597778e-01, 0.12018495;

    m_fr_rl_com_theta.resize(15);
    m_fr_rl_com_theta << -5.25721471e-04, -1.90819582e-17,  2.42861287e-17,
            2.48761318e-03,  0.00000000e+00,  3.46944695e-18,
            -1.06575947e-02,  2.72193463e-02, -2.84351892e-03,
            -3.15264338e-04, -7.44108469e-02, -4.90586961e-02,
            -4.42046174e-02, -1.03480378e-03, -0.02405147;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x.resize(15);
    m_fl_rr_com_x << 2.13060674e-02,  1.66533454e-16, -1.66533454e-16,
            1.26510523e-01,  1.11022302e-16,  0.00000000e+00,
            8.81433170e-01, -1.09894961e+00,  1.34764628e-01,
            1.73731183e+00,  3.24921666e-01, -1.97849489e-01,
            -1.57214009e+00, -5.48487795e-02, -0.18221264;

    m_fl_rr_com_y.resize(15);
    m_fl_rr_com_y << -1.01817056e-03, -1.76941795e-16, -1.37043155e-16,
            5.94789356e-03, -2.77555756e-16, -5.55111512e-17,
            1.61757065e-01, -4.34254858e-01,  5.05352570e-02,
            3.59328902e-01, -9.18808358e-02,  7.59846965e-02,
            -2.79053041e-01, -2.39756934e-01, -0.08715053;

    m_fl_rr_com_theta.resize(15);
    m_fl_rr_com_theta << 4.96150901e-04,  2.08166817e-17,  6.07153217e-17,
            -2.79959932e-03,  0.00000000e+00, -2.77555756e-17,
            -8.56784551e-02,  2.30646244e-02,  1.68399940e-02,
            -7.07108868e-02,  3.06016991e-02, -1.79665741e-01,
            1.92624350e-01,  1.11434322e-01, 0.09874808;

    // FL models coefficients
    m_fl_swinging_x.resize(15);
    m_fl_swinging_x << 4.20600309e-02, -5.55111512e-16, -1.11022302e-16,
            2.06952476e-01, -2.22044605e-16,  2.22044605e-16,
            3.17740200e-01, -1.42997682e+00, -1.28564077e-01,
            2.34331471e+00,  6.11250579e-01,  1.97573088e-01,
            -2.38618624e+00, -2.55289131e-01, -0.07062906;

    m_fl_swinging_y.resize(15);
    m_fl_swinging_y << -8.70312999e-04,  2.16840434e-17,  9.71445147e-17,
            1.25034194e-02,  1.11022302e-16,  0.00000000e+00,
            3.26485833e-01, -1.38608334e+00,  1.56238251e-01,
            7.70901464e-01, -2.30168468e-01,  2.78559425e-01,
            -4.94983307e-01, -4.44662655e-01, -0.11971733;

    // FR models coefficients
    m_fr_swinging_x.resize(15);
    m_fr_swinging_x << 3.05846348e-02,  0.00000000e+00,  0.00000000e+00,
            2.13265380e-01,  0.00000000e+00,  2.22044605e-16,
            8.12179238e-01, -4.70675267e-01, -1.13492826e+00,
            1.05306748e+00, -1.00196696e+00,  8.87622596e-01,
            -5.27874134e-01,  6.71587336e-01, -0.14272959;
    m_fr_swinging_y.resize(15);
    m_fr_swinging_y << 2.57320771e-03, -1.35308431e-16, -6.09321621e-17,
            6.59747348e-03,  2.22044605e-16,  0.00000000e+00,
            -7.98642419e-02,  3.73805608e-01,  1.69780011e-01,
            -7.97532303e-02,  5.46915760e-03, -1.79611612e+00,
            1.47892327e-01,  3.34829672e-01, 0.2355151;

    // RL models coefficients
    m_rl_swinging_x.resize(15);
    m_rl_swinging_x << 3.07931536e-02,  3.33066907e-16,  1.11022302e-16,
            2.11103962e-01, -2.22044605e-16,  4.44089210e-16,
            8.68981546e-01, -6.23198792e-01, -3.33866232e-01,
            9.00839440e-01, -1.86107306e+00,  1.18015837e+00,
            -6.34753381e-01,  6.54572837e-01, -0.63735996;
    m_rl_swinging_y.resize(15);
    m_rl_swinging_y << 2.57320771e-03, -1.35308431e-16, -6.09321621e-17,
            6.59747348e-03,  2.22044605e-16,  0.00000000e+00,
            -7.98642419e-02,  3.73805608e-01,  1.69780011e-01,
            -7.97532303e-02,  5.46915760e-03, -1.79611612e+00,
            1.47892327e-01,  3.34829672e-01, 0.2355151;

    // RR models coefficients
    m_rr_swinging_x.resize(15);
    m_rr_swinging_x << 3.74494867e-02,  6.66133815e-16,  3.33066907e-16,
            2.17375414e-01,  0.00000000e+00,  4.44089210e-16,
            1.37199999e+00, -1.70127875e+00,  4.60088837e-02,
            2.32024043e+00,  4.44512538e-01,  2.87099424e-01,
            -3.47622289e+00, -3.02327973e-01, -0.67954112;
    m_rr_swinging_y.resize(15);
    m_rr_swinging_y << -9.68396761e-04,  1.02348685e-16, -1.11022302e-16,
            6.15428461e-03, -2.22044605e-16,  1.11022302e-16,
            1.58101117e-01, -5.44408332e-01,  2.84881050e-02,
            5.56203805e-01, -4.63776740e-02,  2.41075975e-01,
            -2.55065105e-01, -7.60707747e-01, -0.11239619;

    // CoM models coefficients when FR/RL are swinging
    m_fr_rl_com_x_fs.resize(19);
    m_fr_rl_com_x_fs << 4.59125776e-03,  2.22044605e-16,  2.58907479e-16,
            4.79423471e-02, -2.63677968e-16,  2.77555756e-16,
            2.47476264e-01, -1.84531234e-01, -8.53056614e-02,
            -1.21467889e-02,  5.81296570e-01,  1.13816556e-01,
            -1.31624245e-01,  3.57560521e-01,  1.78773100e-01,
            2.30522418e-02, -7.33081009e-01,  1.51249337e-01, -0.19377041;

    m_fr_rl_com_y_fs.resize(19);
    m_fr_rl_com_y_fs << 2.41247703e-03, -2.81892565e-18,  7.45931095e-17,
            9.96083816e-03,  9.02056208e-17, -1.38777878e-17,
            -3.14860635e-02,  1.76816607e-02,  6.27707848e-03,
            -2.58757166e-03, -8.45705059e-03,  2.66990732e-02,
            -1.70024764e-03, -8.90444265e-02, -9.54108190e-03,
            -5.16319865e-01,  1.37448775e-01,  2.43330452e-01, 0.11878082;

    m_fr_rl_com_theta_fs.resize(19);
    m_fr_rl_com_theta_fs << -6.76730810e-04, -2.60208521e-18,  2.25514052e-17,
            5.11027226e-04,  0.00000000e+00, -1.73472348e-17,
            4.95308808e-03, -2.65707866e-03, -8.93438168e-03,
            -1.26937253e-02, -4.10816351e-02,  2.45681242e-02,
            -7.08142790e-03, -3.92508888e-02, -5.60843590e-02,
            -1.51652134e-02, -3.43587932e-02,  2.16107425e-03, -0.01683231;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x_fs.resize(19);
    m_fl_rr_com_x_fs << 7.24552341e-03,  2.49800181e-16,  1.76941795e-16,
            6.41195343e-02,  5.55111512e-17, -2.63677968e-16,
            2.27727958e-01,  2.11152332e-01, -9.54239812e-02,
            2.09440299e-03,  1.49950923e-01, -6.50530451e-01,
            -3.87961824e-01,  6.57983614e-01,  2.10521581e-01,
            5.67005503e-01, -2.26922012e-01,  4.26763477e-01, 0.21400358;

    m_fl_rr_com_y_fs.resize(19);
    m_fl_rr_com_y_fs << -3.27346856e-03,  1.26634814e-16,  7.97972799e-17,
            -4.41215809e-03,  1.56125113e-17,  4.16333634e-17,
            4.74372504e-02,  2.72256278e-02, -4.95738171e-03,
            -1.59052453e-03,  1.05211971e-01, -4.17793035e-01,
            -5.32452419e-02,  2.30569673e-01, -1.19047986e-01,
            1.36511055e-01, -9.09861445e-02, -1.72150833e-01, -0.02391585;

    m_fl_rr_com_theta_fs.resize(19);
    m_fl_rr_com_theta_fs << 1.50704664e-03, -1.21430643e-17, -1.64798730e-17,
            6.99620378e-04, -2.08166817e-17, -3.46944695e-17,
            -8.87330895e-03, -1.65762421e-02,  7.51065314e-03,
            -7.87796758e-03, -4.39059199e-02, -1.73056479e-02,
            1.36747489e-02, -2.48819752e-02,  5.26059308e-02,
            -1.77695465e-01,  1.23857129e-01,  8.83107424e-02, 0.08482643;

    // FL models coefficients
    m_fl_swinging_x_fs.resize(19);
    m_fl_swinging_x_fs << 1.40279092e-02, -4.99600361e-16, -9.71445147e-16,
            1.08621104e-01,  2.77555756e-16, -1.94289029e-16,
            3.97684829e-01,  3.52009734e-01, -1.12523521e-01,
            -7.14378290e-04, -6.47822537e-01, -8.93822283e-01,
            -8.25165590e-01,  5.76822396e-01,  2.17618000e-01,
            1.09877543e+00, -3.45350203e-01,  6.29666151e-01, 0.46885229;

    m_fl_swinging_y_fs.resize(19);
    m_fl_swinging_y_fs << -5.89003645e-03, -4.16333634e-17,  3.40005801e-16,
            -5.01935848e-03, -1.24900090e-16, -1.56125113e-16,
            8.36258980e-02,  4.36105605e-02, -1.67269514e-02,
            -7.71144734e-03,  1.82965128e-01, -1.36326557e+00,
            -1.93187865e-02,  4.66495023e-01, -2.96620343e-01,
            4.93508728e-01, -1.24692982e-01, -3.12202613e-01, -0.01502302;

    // FR models coefficients
    m_fr_swinging_x_fs.resize(19);
    m_fr_swinging_x_fs << 8.93720918e-03,  4.99600361e-16,  1.04083409e-15,
            8.47085793e-02,  4.85722573e-16,  2.77555756e-16,
            4.20077834e-01, -3.05615483e-01, -1.10729480e-01,
            -3.07059451e-02,  6.66301589e-01,  3.51358889e-01,
            -1.23157699e+00,  6.22568436e-01,  3.96842119e-01,
            -1.32578032e-01, -1.16502090e+00, -6.67094116e-02, -0.00445306;
    m_fr_swinging_y_fs.resize(19);
    m_fr_swinging_y_fs << 5.34008225e-03,  1.58727198e-16, -4.00287442e-16,
            2.04006133e-02, -5.55111512e-16, -2.77555756e-16,
            -4.67513343e-02,  3.45903178e-02,  1.00038210e-02,
            -3.39628381e-03, -7.25649871e-02,  2.66127522e-01,
            1.53824492e-01, -6.69291976e-02, -1.25680840e-01,
            -1.64627201e+00,  2.21972692e-01,  4.46878327e-01, 0.23246413;

    // RL models coefficients
    m_rl_swinging_x_fs.resize(19);
    m_rl_swinging_x_fs << 7.79802189e-03,  6.66133815e-16, -1.73472348e-16,
            8.26464943e-02, -1.45716772e-16, -3.88578059e-16,
            4.34497675e-01, -3.14847826e-01, -1.05995296e-01,
            -3.45420950e-02,  7.06510111e-01,  2.43376966e-01,
            -1.82229567e-01,  7.04716335e-01, -6.32295167e-01,
            -3.87117578e-02, -1.25555562e+00, -2.01462757e-01, -0.56056053;
    m_rl_swinging_y_fs.resize(19);
    m_rl_swinging_y_fs << 5.34008225e-03,  1.58727198e-16, -4.00287442e-16,
            2.04006133e-02, -5.55111512e-16, -2.77555756e-16,
            -4.67513343e-02,  3.45903178e-02,  1.00038210e-02,
            -3.39628381e-03, -7.25649871e-02,  2.66127522e-01,
            1.53824492e-01, -6.69291976e-02, -1.25680840e-01,
            -1.64627201e+00,  2.21972692e-01,  4.46878327e-01, 0.23246413;

    // RR models coefficients
    m_rr_swinging_x_fs.resize(19);
    m_rr_swinging_x_fs << 1.24992855e-02,  1.11022302e-16, -6.93889390e-17,
            1.05618429e-01, -1.11022302e-16,  5.82867088e-16,
            4.21684803e-01,  3.68385216e-01, -1.24842067e-01,
            9.90899905e-03,  2.51371537e-01, -9.90138125e-01,
            -7.75263939e-01,  6.59148238e-01,  8.36876079e-02,
            1.21563528e+00, -1.22723600e+00,  6.06083898e-01, -0.01714463;
    m_rr_swinging_y_fs.resize(19);
    m_rr_swinging_y_fs << -3.98897869e-03,  3.46944695e-17, -1.56125113e-16,
            -5.13190653e-03,  2.94902991e-17,  1.64798730e-16,
            5.52404659e-02,  2.60914675e-02, -5.86537091e-03,
            -4.98036713e-03,  7.92422074e-02, -5.33540420e-01,
            -1.03239751e-01,  3.94104769e-01, -7.07802915e-02,
            3.50998996e-01, -2.38010157e-02, -6.82391788e-01, -0.03352679;
}

/**
 * Footstep prediction for first step.
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
void Model::predictFirstStep(double p_previousVelocityX,
                             double p_previousVelocityY,
                             double p_previousAngularVelocity,
                             double p_nextVelocityX,
                             double p_nextVelocityY,
                             double p_nextAngularVelocity,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             std::vector<double> &p_predictions) {
    Eigen::VectorXd l_modelInput(19);
    l_modelInput << p_previousVelocityX,
                    p_previousVelocityY,
                    p_previousAngularVelocity,
                    p_nextVelocityX,
                    p_nextVelocityY,
                    p_nextAngularVelocity,
                    p_odomVelocityState.linear.x,
                    p_odomVelocityState.linear.y,
                    p_odomVelocityState.linear.z,
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

    ROS_DEBUG_STREAM("Input: " << l_modelInput);

    if (p_currentFeetConfiguration.fr_rl_swinging) {
        p_predictions[0] = m_fr_rl_com_x_fs * l_modelInput;
        p_predictions[1] = m_fr_rl_com_y_fs * l_modelInput;

        p_predictions[2] = 0.0;
        p_predictions[3] = 0.0;

        p_predictions[4] = m_fr_swinging_x_fs * l_modelInput;
        p_predictions[5] = m_fr_swinging_y_fs * l_modelInput;

        p_predictions[6] = m_rl_swinging_x_fs * l_modelInput;
        p_predictions[7] = m_rl_swinging_y_fs * l_modelInput;

        p_predictions[8] = 0.0;
        p_predictions[9] = 0.0;

        p_predictions[10] = m_fr_rl_com_theta_fs * l_modelInput;
    }
    else {
        p_predictions[0] = m_fl_rr_com_x_fs * l_modelInput;
        p_predictions[1] = m_fl_rr_com_y_fs * l_modelInput;

        p_predictions[2] = m_fl_swinging_x_fs * l_modelInput;
        p_predictions[3] = m_fl_swinging_y_fs * l_modelInput;

        p_predictions[4] = 0.0;
        p_predictions[5] = 0.0;

        p_predictions[6] = 0.0;
        p_predictions[7] = 0.0;

        p_predictions[8] = m_rr_swinging_x_fs * l_modelInput;
        p_predictions[9] = m_rr_swinging_y_fs * l_modelInput;

        p_predictions[10] = m_fl_rr_com_theta_fs * l_modelInput;
    }
}

/**
 * Footstep prediction for after second step
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
void Model::predictOnwardSteps(double p_previousVelocityX,
                               double p_previousVelocityY,
                               double p_previousAngularVelocity,
                               double p_nextVelocityX,
                               double p_nextVelocityY,
                               double p_nextAngularVelocity,
                               const FeetConfiguration &p_currentFeetConfiguration,
                               std::vector<double> &p_predictions) {
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

    ROS_DEBUG_STREAM("Input: " << l_modelInput);

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
    // Rotation of CoM w.r.t World
    geometry_msgs::TransformStamped l_R_W_C;
    l_R_W_C.header.stamp = ros::Time::now();
    l_R_W_C.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_R_W_C.transform.translation.x = 0;
    l_R_W_C.transform.translation.y = 0;
    l_R_W_C.transform.translation.z = 0;
    l_R_W_C.transform.rotation.x = p_currentWorldCoordinatesCoM.q.x();
    l_R_W_C.transform.rotation.y = p_currentWorldCoordinatesCoM.q.y();
    l_R_W_C.transform.rotation.z = p_currentWorldCoordinatesCoM.q.z();
    l_R_W_C.transform.rotation.w = p_currentWorldCoordinatesCoM.q.w();

    // Predicted CoM displacement (in CoM frame)
    geometry_msgs::PointStamped l_displacementRobotFrame;
    l_displacementRobotFrame.header.stamp = ros::Time::now();
    l_displacementRobotFrame.header.frame_id = ROBOT_REFERENCE_FRAME;
    l_displacementRobotFrame.point.x = p_predictedCoMDisplacementX;
    l_displacementRobotFrame.point.y = p_predictedCoMDisplacementY;
    l_displacementRobotFrame.point.z = 0;

    // Transform CoM displacement to world frame
    geometry_msgs::PointStamped l_displacementMapFrame;
    tf2::doTransform(l_displacementRobotFrame, l_displacementMapFrame, l_R_W_C);

    // Update CoM position in world frame
    p_newWorldCoordinatesCoM.x = p_currentWorldCoordinatesCoM.x + l_displacementMapFrame.point.x;
    p_newWorldCoordinatesCoM.y = p_currentWorldCoordinatesCoM.y + l_displacementMapFrame.point.y;

    // Compute quaternion representation of predicted rotation
    tf2::Quaternion l_velocityCommandQuaternion;
    //l_velocityCommandQuaternion.setRPY(0, 0, p_predictedCoMDisplacementTheta);
    l_velocityCommandQuaternion.setRPY(0, 0, 0);

    // Apply predicted quaternion rotation to
    // current CoM's rotation in world frame
    tf2::Quaternion l_newCoMRotation = p_currentWorldCoordinatesCoM.q * l_velocityCommandQuaternion;
    l_newCoMRotation.normalize();
    p_newWorldCoordinatesCoM.q = l_newCoMRotation;
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
    // Rotation of CoM w.r.t World
    geometry_msgs::TransformStamped l_R_W_C;
    l_R_W_C.header.stamp = ros::Time::now();
    l_R_W_C.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_R_W_C.transform.translation.x = 0;
    l_R_W_C.transform.translation.y = 0;
    l_R_W_C.transform.translation.z = 0;
    l_R_W_C.transform.rotation.x = p_newWorldCoordinatesCoM.q.x();
    l_R_W_C.transform.rotation.y = p_newWorldCoordinatesCoM.q.y();
    l_R_W_C.transform.rotation.z = p_newWorldCoordinatesCoM.q.z();
    l_R_W_C.transform.rotation.w = p_newWorldCoordinatesCoM.q.w();

    // Transform predicted footstep predictions from CoM to world frame
    std::vector<double> l_feetPredictionWorldFrame;
    geometry_msgs::PointStamped l_feetPredictionCoMFrame;
    l_feetPredictionCoMFrame.header.stamp = ros::Time::now();
    l_feetPredictionCoMFrame.header.frame_id = ROBOT_REFERENCE_FRAME;
    for (int x = 0; x < 4; x++) {
        l_feetPredictionCoMFrame.point.x = p_predictions[2 * x + 2];
        l_feetPredictionCoMFrame.point.y = p_predictions[2 * x + 3];
        l_feetPredictionCoMFrame.point.z = 0;

        geometry_msgs::PointStamped l_footDisplacementWorldFrame;
        tf2::doTransform(l_feetPredictionCoMFrame, l_footDisplacementWorldFrame, l_R_W_C);

        l_feetPredictionWorldFrame.push_back(l_footDisplacementWorldFrame.point.x);
        l_feetPredictionWorldFrame.push_back(l_footDisplacementWorldFrame.point.y);
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
    p_newFeetConfiguration.flCoM.x = p_newFeetConfiguration.flMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.flCoM.y = p_newFeetConfiguration.flMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.frCoM.x = p_newFeetConfiguration.frMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.frCoM.y = p_newFeetConfiguration.frMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.rlCoM.x = p_newFeetConfiguration.rlMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.rlCoM.y = p_newFeetConfiguration.rlMap.y - p_newWorldCoordinatesCoM.y;

    p_newFeetConfiguration.rrCoM.x = p_newFeetConfiguration.rrMap.x - p_newWorldCoordinatesCoM.x;
    p_newFeetConfiguration.rrCoM.y = p_newFeetConfiguration.rrMap.y - p_newWorldCoordinatesCoM.y;

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

    if (p_plannedFootstep == 0) {
        predictFirstStep(p_action.x * p_previousVelocity,
                         p_action.y * p_previousVelocity,
                         p_action.theta * p_previousVelocity,
                         p_action.x * p_nextVelocity,
                         p_action.y * p_nextVelocity,
                         p_action.theta * p_nextVelocity,
                         p_odomVelocityState,
                         p_currentFeetConfiguration,
                         l_predictions);
    }
    else {
        predictOnwardSteps(p_action.x * p_previousVelocity,
                           p_action.y * p_previousVelocity,
                           p_action.theta * p_previousVelocity,
                           p_action.x * p_nextVelocity,
                           p_action.y * p_nextVelocity,
                           p_action.theta * p_nextVelocity,
                           p_currentFeetConfiguration,
                           l_predictions);
    }

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

//     // Publish predicted CoM and feet poses
//     int j = 0;
//     visualization_msgs::Marker l_footCommonMarker;
//     l_footCommonMarker.header.stamp = ros::Time::now();
//     l_footCommonMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//     l_footCommonMarker.type = 2;
//     l_footCommonMarker.action = 0;
//     l_footCommonMarker.lifetime = ros::Duration(1);
//     l_footCommonMarker.pose.orientation.x = 0;
//     l_footCommonMarker.pose.orientation.y = 0;
//     l_footCommonMarker.pose.orientation.z = 0;
//     l_footCommonMarker.pose.orientation.w = 1;
//     l_footCommonMarker.scale.x = 0.05;
//     l_footCommonMarker.scale.y = 0.035;
//     l_footCommonMarker.scale.z = 0.035;
//     l_footCommonMarker.color.r = 0;
//     l_footCommonMarker.color.g = 1;
//     l_footCommonMarker.color.b = 0;
//     l_footCommonMarker.color.a = 0.7;

//     visualization_msgs::Marker l_CoMMarker = l_footCommonMarker;
//     l_CoMMarker.id = j++;
//     l_CoMMarker.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
//     l_CoMMarker.pose.position.x = p_newWorldCoordinatesCoM.x;
//     l_CoMMarker.pose.position.y = p_newWorldCoordinatesCoM.y;
//     l_CoMMarker.pose.position.z = 0;

//     visualization_msgs::Marker l_flFootMarker = l_footCommonMarker;
//     l_flFootMarker.id = j++;
//     l_flFootMarker.pose.position.x = p_newFeetConfiguration.flMap.x;
//     l_flFootMarker.pose.position.y = p_newFeetConfiguration.flMap.y;
//     l_flFootMarker.pose.position.z = 0;

//     visualization_msgs::Marker l_frFootMarker = l_footCommonMarker;
//     l_frFootMarker.id = j++;
//     l_frFootMarker.pose.position.x = p_newFeetConfiguration.frMap.x;
//     l_frFootMarker.pose.position.y = p_newFeetConfiguration.frMap.y;
//     l_frFootMarker.pose.position.z = 0;

//     visualization_msgs::Marker l_rlFootMarker = l_footCommonMarker;
//     l_rlFootMarker.id = j++;
//     l_rlFootMarker.pose.position.x = p_newFeetConfiguration.rlMap.x;
//     l_rlFootMarker.pose.position.y = p_newFeetConfiguration.rlMap.y;
//     l_rlFootMarker.pose.position.z = 0;

//     visualization_msgs::Marker l_rrFootMarker = l_footCommonMarker;
//     l_rrFootMarker.id = j++;
//     l_rrFootMarker.pose.position.x = p_newFeetConfiguration.rrMap.x;
//     l_rrFootMarker.pose.position.y = p_newFeetConfiguration.rrMap.y;
//     l_rrFootMarker.pose.position.z = 0;

//     visualization_msgs::MarkerArray l_pathFeetConfiguration;
//     l_pathFeetConfiguration.markers.push_back(l_CoMMarker);
//     l_pathFeetConfiguration.markers.push_back(l_flFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_frFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_rlFootMarker);
//     l_pathFeetConfiguration.markers.push_back(l_rrFootMarker);

//     m_feetConfigurationPublisher.publish(l_pathFeetConfiguration);
//     ros::Duration(0.5).sleep();
}