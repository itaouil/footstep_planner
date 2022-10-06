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
    m_fr_rl_com_x_fs << 3.01098992e-03,  1.11022302e-16, -4.51461785e-16,
            4.85310703e-02,  2.77555756e-17,  3.05311332e-16,
            2.45196956e-01, -1.89389592e-01, -8.37240174e-02,
            -9.40708692e-03,  5.84766386e-01,  1.13082145e-01,
            -1.65785784e-01,  3.82109468e-01,  2.05946283e-01,
            -7.33059247e-02, -7.16729785e-01,  1.46746379e-01, -0.16013304;

    m_fr_rl_com_y_fs.resize(19);
    m_fr_rl_com_y_fs << 2.60691052e-03,  6.89552582e-17,  3.46944695e-18,
            9.92550939e-03,  6.24500451e-17,  0.00000000e+00,
            -2.94379662e-02,  1.94459681e-02,  6.32811296e-03,
            -1.97583367e-03, -1.45592293e-02,  2.81001228e-02,
            -1.21214529e-02, -8.80658394e-02,  1.12480163e-02,
            -5.10392854e-01,  1.39140951e-01,  2.33920664e-01, 0.12644459;

    m_fr_rl_com_theta_fs.resize(19);
    m_fr_rl_com_theta_fs << -1.32065556e-03, -3.46944695e-18,  1.58293517e-17,
            8.11823510e-04, -1.04083409e-17, -2.60208521e-17,
            3.98787577e-03, -1.45926963e-03, -9.17793844e-03,
            -1.27845579e-02, -3.91451023e-02,  1.85896781e-02,
            -1.71862459e-02, -3.77782802e-02, -5.12678478e-02,
            -1.24143929e-02, -3.33577086e-02,  8.25124816e-03, -0.01187962;

    // CoM models coefficients when FL/RR are swinging
    m_fl_rr_com_x_fs.resize(19);
    m_fl_rr_com_x_fs << 6.24956899e-03, -4.16333634e-16, -5.41233725e-16,
            6.46220854e-02, -2.49800181e-16, -2.77555756e-17,
            2.26828730e-01,  2.12128004e-01, -9.70759294e-02,
            4.29185264e-03,  1.70845654e-01, -6.61786550e-01,
            -4.04253097e-01,  6.27735444e-01,  2.26359565e-01,
            5.90631606e-01, -2.60205595e-01,  4.13282682e-01, 0.20086454;

    m_fl_rr_com_y_fs.resize(19);
    m_fl_rr_com_y_fs << -3.63391487e-03, -7.63278329e-17,  2.08166817e-17,
            -4.68245973e-03, -2.42861287e-17, -2.42861287e-17,
            4.75312681e-02,  2.61110947e-02, -5.41402135e-03,
            -1.89320671e-03,  6.16196328e-02, -4.17975767e-01,
            -5.26513254e-02,  2.40837789e-01, -1.23418601e-01,
            1.40058160e-01, -5.23193666e-02, -1.58710496e-01, -0.0019254;

    m_fl_rr_com_theta_fs.resize(19);
    m_fl_rr_com_theta_fs << 1.54282739e-03,  3.29597460e-17,  4.51028104e-17,
            6.18510112e-04,  4.16333634e-17,  2.08166817e-17,
            -8.86762594e-03, -1.57089658e-02,  8.05450823e-03,
            -8.58213052e-03, -4.81682217e-02, -2.40438015e-02,
            6.46129245e-03, -3.60983450e-02,  6.18872146e-02,
            -1.65519686e-01,  1.30063405e-01,  9.96318393e-02, 0.09096875;

    // FL models coefficients
    m_fl_swinging_x_fs.resize(19);
    m_fl_swinging_x_fs << 1.32199891e-02, -1.66533454e-16, -7.28583860e-16,
            1.06751476e-01,  2.77555756e-16,  2.22044605e-16,
            4.05532445e-01,  3.50557055e-01, -1.18391707e-01,
            3.43127594e-03, -6.12407620e-01, -7.98817854e-01,
            -8.54166439e-01,  5.68200403e-01,  2.30295517e-01,
            1.10309585e+00, -3.42897935e-01,  6.74314531e-01, 0.46393625;

    m_fl_swinging_y_fs.resize(19);
    m_fl_swinging_y_fs << -5.70985567e-03, -1.04950770e-16,  1.59594560e-16,
            -5.68337852e-03, -4.99600361e-16,  5.96744876e-16,
            8.40215606e-02,  4.45183373e-02, -1.76397393e-02,
            -4.26489947e-03,  1.86683441e-01, -1.33350920e+00,
            -2.15422871e-02,  4.89043853e-01, -2.93881223e-01,
            4.67061873e-01, -1.26250522e-01, -3.17432370e-01, -0.01334711;

    // FR models coefficients
    m_fr_swinging_x_fs.resize(19);
    m_fr_swinging_x_fs << 8.24643093e-03,  5.55111512e-17, -1.52655666e-16,
            8.26598512e-02, -5.68989300e-16, -7.21644966e-16,
            4.27169669e-01, -3.14064556e-01, -1.02580723e-01,
            -3.09172832e-02,  5.81209113e-01,  3.31519794e-01,
            -1.26318592e+00,  5.56872261e-01,  4.41857281e-01,
            -1.06494247e-01, -1.09412261e+00,  1.99126907e-02, 0.05516289;
    m_fr_swinging_y_fs.resize(19);
    m_fr_swinging_y_fs << 5.34008225e-03,  1.58727198e-16, -4.00287442e-16,
            2.04006133e-02, -5.55111512e-16, -2.77555756e-16,
            -4.67513343e-02,  3.45903178e-02,  1.00038210e-02,
            -3.39628381e-03, -7.25649871e-02,  2.66127522e-01,
            1.53824492e-01, -6.69291976e-02, -1.25680840e-01,
            -1.64627201e+00,  2.21972692e-01,  4.46878327e-01, 0.23246413;

    // RL models coefficients
    m_rl_swinging_x_fs.resize(19);
    m_rl_swinging_x_fs << 6.70153587e-03,  5.55111512e-17, -2.08166817e-16,
            8.12906344e-02,  1.66533454e-16,  6.66133815e-16,
            4.39505252e-01, -3.16213419e-01, -1.06330695e-01,
            -3.59055901e-02,  6.80581830e-01,  2.44727390e-01,
            -1.82501870e-01,  6.59572934e-01, -6.18893091e-01,
            9.40809559e-03, -1.24605793e+00, -6.56648744e-02, -0.5422321;
    m_rl_swinging_y_fs.resize(19);
    m_rl_swinging_y_fs << 5.34008225e-03,  1.58727198e-16, -4.00287442e-16,
            2.04006133e-02, -5.55111512e-16, -2.77555756e-16,
            -4.67513343e-02,  3.45903178e-02,  1.00038210e-02,
            -3.39628381e-03, -7.25649871e-02,  2.66127522e-01,
            1.53824492e-01, -6.69291976e-02, -1.25680840e-01,
            -1.64627201e+00,  2.21972692e-01,  4.46878327e-01, 0.23246413;

    // RR models coefficients
    m_rr_swinging_x_fs.resize(19);
    m_rr_swinging_x_fs << 1.51205756e-02,  2.77555756e-16, -1.23338839e-15,
            1.08068001e-01, -6.66133815e-16, -4.44089210e-16,
            4.12603725e-01,  3.71686144e-01, -1.20124124e-01,
            1.19504478e-02,  4.04933217e-01, -9.05609229e-01,
            -6.91478523e-01,  7.16406254e-01,  3.52799426e-02,
            1.10976451e+00, -1.39439115e+00,  5.18007415e-01, -0.13208212;
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
    // Regression input
     Eigen::VectorXd l_modelInput(19);
     l_modelInput << p_previousVelocityX,
             p_previousVelocityY,
             p_previousAngularVelocity,
             p_nextVelocityX,
             p_nextVelocityY,
             p_nextAngularVelocity,
             p_odomVelocityState.linear.x,
             p_odomVelocityState.linear.y,
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
        // FL/RR are swinging
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
    //l_velocityCommandQuaternion.setRPY(0, 0, p_predictedCoMDisplacementTheta);
    l_velocityCommandQuaternion.setRPY(0, 0, 0);

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
void Model::predictNextState(uint p_plannedFootstep,
                             double p_previousVelocity,
                             double p_nextVelocity,
                             const Action &p_action,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const World3D &p_currentWorldCoordinatesCoM,
                             const FeetConfiguration &p_currentFeetConfiguration,
                             FeetConfiguration &p_newFeetConfiguration,
                             World3D &p_newWorldCoordinatesCoM) {
    // Predict feet and CoM displacements
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
//     ros::Duration(0.3).sleep();
}