#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

# Imports
import rospy
import joblib
import numpy as np
from aliengo_navigation.srv import FootstepPrediction, FootstepPredictionResponse

# Path where models reside
path = "/home/itaouil/workspace/aliengo_ws/src/aliengo_navigation/models/protocol3/"

# Load models for each foot
model_fl_x = joblib.load(path + "model_fl_x.sav")
model_fl_y = joblib.load(path + "model_fl_y.sav")
model_fr_x = joblib.load(path + "model_fr_x.sav")
model_fr_y = joblib.load(path + "model_fr_y.sav")
model_rl_x = joblib.load(path + "model_rl_x.sav")
model_rl_y = joblib.load(path + "model_rl_y.sav")
model_rr_x = joblib.load(path + "model_rr_x.sav")
model_rr_y = joblib.load(path + "model_rr_y.sav")


def predict(req):
    # Velocities in the request
    x_velocity = req.x_velocity
    y_velocity = req.y_velocity
    theta_velocity = req.theta_velocity

    print("Prediction request received: \n", req, "\n")

    # Predict FL foot
    predicted_fl_x = model_fl_x.predict(np.array([x_velocity, y_velocity, 0, theta_velocity, req.fl_x, req.fr_x]).reshape(1, -1))
    predicted_fl_y = model_fl_y.predict(np.array([x_velocity, y_velocity, 0, theta_velocity, req.fl_y, req.fr_y]).reshape(1, -1))

    # Predict FR foot
    predicted_fr_x = model_fr_x.predict(np.array([x_velocity, y_velocity, 0, theta_velocity, req.fl_x, req.fr_x]).reshape(1, -1))
    predicted_fr_y = model_fr_y.predict(np.array([x_velocity, y_velocity, 0, theta_velocity, req.fl_y, req.fr_y]).reshape(1, -1))

    # Predict RL foot
    predicted_rl_x = model_rl_x.predict(np.array([x_velocity, y_velocity, 0, theta_velocity, req.rl_x, req.rr_x]).reshape(1, -1))
    predicted_rl_y = model_rl_y.predict(np.array([x_velocity, y_velocity, 0, theta_velocity, req.rl_y, req.rr_y]).reshape(1, -1))

    # Predict RR foot
    predicted_rr_x = model_rr_x.predict(np.array([x_velocity, y_velocity, 0, theta_velocity, req.rl_x, req.rr_x]).reshape(1, -1))
    predicted_rr_y = model_rr_y.predict(np.array([x_velocity, y_velocity, 0, theta_velocity, req.rl_y, req.rr_y]).reshape(1, -1))

    # print([predicted_fl_x,
    #        predicted_fl_y,
    #        predicted_fr_x,
    #        predicted_fr_y,
    #        predicted_rl_x,
    #        predicted_rl_y,
    #        predicted_rr_x,
    #        predicted_rr_y], "\n")

    return FootstepPredictionResponse([predicted_fl_x,
                                      predicted_fl_y,
                                      predicted_fr_x,
                                      predicted_fr_y,
                                      predicted_rl_x,
                                      predicted_rl_y,
                                      predicted_rr_x,
                                      predicted_rr_y])


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