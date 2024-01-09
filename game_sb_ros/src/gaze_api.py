import argparse
import logging
import pathlib
import warnings
import rospkg
import os

import torch
import numpy as np
import cv2

from omegaconf import DictConfig, OmegaConf
import ptgaze
from ptgaze.api_gaze import APIGaze
from ptgaze.utils import (check_path_all, download_dlib_pretrained_model,
                    download_ethxgaze_model, download_mpiifacegaze_model,
                    download_mpiigaze_model, expanduser_all,
                    generate_dummy_camera_params)

CONFIG_PATH = os.path.join(rospkg.RosPack().get_path('game_sb_ros'),
                           'src', 'configs', 'api-eth-xgaze.yaml')

class GazeAPI:

    def __init__(self) -> None:
        config = OmegaConf.load(CONFIG_PATH)
        package_root = pathlib.Path(ptgaze.__file__).parent.resolve()
        config.PACKAGE_ROOT = package_root.as_posix()
        print('loaded config')
        
        expanduser_all(config)
        if config.gaze_estimator.use_dummy_camera_params:
            generate_dummy_camera_params(config)
        OmegaConf.set_readonly(config, True)

        if config.face_detector.mode == 'dlib':
            download_dlib_pretrained_model()
        if config.mode == 'ETH-XGaze':
            download_ethxgaze_model()

        check_path_all(config)
        self.api = APIGaze(config)


    def run(self, image) -> list:
        face_gazes = self.api.run(image)
        return face_gazes
     

    def gaze_vector_to_angle(self, vector: np.ndarray) -> np.ndarray:
        assert vector.shape == (3, )
        x, y, z = vector
        pitch = np.arcsin(-y)
        yaw = np.arctan2(-x, -z)
        return np.array([pitch, yaw])
