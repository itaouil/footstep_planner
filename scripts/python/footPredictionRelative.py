#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

# Imports
import rospy
import joblib
import numpy as np
from aliengo_navigation.srv import FootstepPrediction, FootstepPredictionResponse

# Path where models reside
path = "/home/itaouil/workspace/aliengo_ws/src/aliengo_navigation/models/protocol3/"

# Load models for CoM
model_fl_rr_com_x = joblib.load(path + "com/model_fl_rr_com_x.sav")
model_fl_rr_com_y = joblib.load(path + "com/model_fl_rr_com_y.sav")
model_fr_rl_com_x = joblib.load(path + "com/model_fr_rl_com_x.sav")
model_fr_rl_com_y = joblib.load(path + "com/model_fr_rl_com_y.sav")

# Load models for each foot
model_fl_relative_swinging_x = joblib.load(path + "relative/model_fl_relative_swinging_x.sav")
model_fl_relative_swinging_y = joblib.load(path + "relative/model_fl_relative_swinging_y.sav")
model_fl_relative_support_x = joblib.load(path + "relative/model_fl_relative_support_x.sav")
model_fl_relative_support_y = joblib.load(path + "relative/model_fl_relative_support_y.sav")
model_fr_relative_swinging_x = joblib.load(path + "relative/model_fr_relative_swinging_x.sav")
model_fr_relative_swinging_y = joblib.load(path + "relative/model_fr_relative_swinging_y.sav")
model_fr_relative_support_x = joblib.load(path + "relative/model_fr_relative_support_x.sav")
model_fr_relative_support_y = joblib.load(path + "relative/model_fr_relative_support_y.sav")
model_rl_relative_swinging_x = joblib.load(path + "relative/model_rl_relative_swinging_x.sav")
model_rl_relative_swinging_y = joblib.load(path + "relative/model_rl_relative_swinging_y.sav")
model_rl_relative_support_x = joblib.load(path + "relative/model_rl_relative_support_x.sav")
model_rl_relative_support_y = joblib.load(path + "relative/model_rl_relative_support_y.sav")
model_rr_relative_swinging_x = joblib.load(path + "relative/model_rr_relative_swinging_x.sav")
model_rr_relative_swinging_y = joblib.load(path + "relative/model_rr_relative_swinging_y.sav")
model_rr_relative_support_x = joblib.load(path + "relative/model_rr_relative_support_x.sav")
model_rr_relative_support_y = joblib.load(path + "relative/model_rr_relative_support_y.sav")


def predict(req):
    print("Prediction request received: \n", req, "\n")

    # Input for models
    input_ = np.array([req.x_velocity,
                       req.y_velocity,
                       req.theta_velocity,
                       req.fl_x,
                       req.fl_y,
                       req.fr_x,
                       req.fr_y,
                       req.rl_x,
                       req.rl_y,
                       req.rr_x,
                       req.rr_y]).reshape(1, -1)

    # FR/RL swinging
    if req.fr_rl_swinging:
        # Predicted CoM movement
        predicted_com_x_step = model_fr_rl_com_x.predict(input_)
        predicted_com_y_step = model_fr_rl_com_y.predict(input_)

        # Predict FL footstep
        predicted_fl_x_step = model_fl_relative_support_x.predict(input_)
        predicted_fl_y_step = model_fl_relative_support_y.predict(input_)

        # Predict FR footstep
        predicted_fr_x_step = model_fr_relative_swinging_x.predict(input_)
        predicted_fr_y_step = model_fr_relative_swinging_y.predict(input_)

        # Predict RL footstep
        predicted_rl_x_step = model_rl_relative_swinging_x.predict(input_)
        predicted_rl_y_step = model_rl_relative_swinging_y.predict(input_)

        # Predict RR footstep
        predicted_rr_x_step = model_rr_relative_support_x.predict(input_)
        predicted_rr_y_step = model_rr_relative_support_y.predict(input_)

        print("FR/RL predictions:\n", [predicted_com_x_step,
                                       predicted_com_y_step,
                                       predicted_fl_x_step,
                                       predicted_fl_y_step,
                                       predicted_fr_x_step,
                                       predicted_fr_y_step,
                                       predicted_rl_x_step,
                                       predicted_rl_y_step,
                                       predicted_rr_x_step,
                                       predicted_rr_y_step])

        return FootstepPredictionResponse([predicted_com_x_step,
                                           predicted_com_y_step,
                                           predicted_fl_x_step,
                                           predicted_fl_y_step,
                                           predicted_fr_x_step,
                                           predicted_fr_y_step,
                                           predicted_rl_x_step,
                                           predicted_rl_y_step,
                                           predicted_rr_x_step,
                                           predicted_rr_y_step])
    else:
        # Predicted CoM movement
        predicted_com_x_step = model_fl_rr_com_x.predict(input_)
        predicted_com_y_step = model_fl_rr_com_y.predict(input_)

        # Predict FL footstep
        predicted_fl_x_step = model_fl_relative_swinging_x.predict(input_)
        predicted_fl_y_step = model_fl_relative_swinging_y.predict(input_)

        # Predict FR footstep
        predicted_fr_x_step = model_fr_relative_support_x.predict(input_)
        predicted_fr_y_step = model_fr_relative_support_y.predict(input_)

        # Predict RL footstep
        predicted_rl_x_step = model_rl_relative_support_x.predict(input_)
        predicted_rl_y_step = model_rl_relative_support_y.predict(input_)

        # Predict RR footstep
        predicted_rr_x_step = model_rr_relative_swinging_x.predict(input_)
        predicted_rr_y_step = model_rr_relative_swinging_y.predict(input_)

        print("FL/RR predictions:\n", [predicted_com_x_step,
                                       predicted_com_y_step,
                                       predicted_fl_x_step,
                                       predicted_fl_y_step,
                                       predicted_fr_x_step,
                                       predicted_fr_y_step,
                                       predicted_rl_x_step,
                                       predicted_rl_y_step,
                                       predicted_rr_x_step,
                                       predicted_rr_y_step])

        return FootstepPredictionResponse([predicted_com_x_step,
                                           predicted_com_y_step,
                                           predicted_fl_x_step,
                                           predicted_fl_y_step,
                                           predicted_fr_x_step,
                                           predicted_fr_y_step,
                                           predicted_rl_x_step,
                                           predicted_rl_y_step,
                                           predicted_rr_x_step,
                                           predicted_rr_y_step])


def handle_request():
    # Create ROS node
    rospy.init_node('footstep_prediction_server')

    # Callback function
    s = rospy.Service('footstep_prediction', FootstepPrediction, predict)

    print("Service server is spinning...")

    # Spin it
    rospy.spin()


if __name__ == "__main__":
    handle_request()