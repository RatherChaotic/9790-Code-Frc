#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#


from wpilib.cameraserver import CameraServer
import robotpy_apriltag
from ntcore import NetworkTableInstance
import cv2
import wpilib
import wpilib.cameraserver


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Your image processing code will be launched via a stub that will set up logging and initialize NetworkTables
        # to talk to your robot code.
        # https://robotpy.readthedocs.io/en/stable/vision/roborio.html#important-notes

        wpilib.CameraServer.launch("vision.py:main")