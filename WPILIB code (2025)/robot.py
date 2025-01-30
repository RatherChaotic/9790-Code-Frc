# TODO: insert robot code here

#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.drive
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig
from pathplannerlib.auto import PathPlannerAuto
import rev
import cv2
import robotpy_apriltag


kY = 4 #Y Button
kX = 3 #X Button
kB = 2 #B Button
kA = 1 #A Button
kLB = 5 #LB Button
kRB = 6 #RB Button
kBack = 7 #Back Button
kStart = 8 #Start Button
kRT = 9 #Right Trigger Button


def robotPeriodic(self):
    # update the dashboard mechanism's state
    self.elevator.setLength(
    self.kElevatorMinimumLength + self.elevatorEncoder.getDistance()
    )
    self.wrist.setAngle(self.wristPot.get())

    # MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
# off the root node or another ligament object
    self.elevator = self.root.appendLigament(
    "elevator", self.kElevatorMinimumLength, 90
    )
    self.wrist = self.elevator.appendLigament(
    "wrist", 0.5, 90, 6, wpilib.Color8Bit(wpilib.Color.kPurple)
    )

    # post the mechanism to the dashboard
    wpilib.SmartDashboard.putData("Mech2d", self.mech)

class MyRobot(wpilib.TimedRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""


        self.controller1 = wpilib.XboxController(0)
        self.controller2 = wpilib.XboxController(1)
        self.config = RobotConfig.fromGUISettings()
        self.timer = wpilib.Timer()
        self.lf_motor = wpilib.PWMSparkMax(1)
        self.lr_motor = wpilib.PWMSparkMax(2)
        self.rf_motor = wpilib.PWMSparkMax(3)
        self.rr_motor = wpilib.PWMSparkMax(4)

        l_motor = wpilib.MotorControllerGroup(self.lf_motor, self.lr_motor)
        r_motor = wpilib.MotorControllerGroup(self.rf_motor, self.rr_motor)

        l_motor.setInverted(True)

        AutoBuilder.configureHolonomic(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPLTVController(0.02), # PPLTVController is the built in path following controller for differential drive trains
            self.config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )


        self.drive = wpilib.drive.DifferentialDrive(l_motor, r_motor)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def getAutonomousCommand(self):
        return PathPlannerAuto('New Auto')

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        if self.timer.get() < 2.0:
            # Drive forwards half speed, make sure to turn input squaring off
            self.drive.arcadeDrive(-0.5, 0, squareInputs=False)
        else:
            self.drive.stopMotor()  # Stop robot

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""
         #TEST THIS LATER (MOVEMENT ADJUSTMENTS)
        #drive motors
        rightTrigger = self.controller1.getRightTriggerAxis()    #New (test) code
        #RightY = self.joystick.getRightY()    #original code
        LeftX = self.controller1.getLeftX()
        leftTrigger = self.controller1.getLeftTriggerAxis()

        #exponential movement
        if(rightTrigger > 0):
            rightTrigger = (rightTrigger**-4)*-1
        else:
            rightTrigger = (rightTrigger**4)

        if(leftTrigger > 0):
            rightTrigger = (rightTrigger**0)
        else:
            rightTrigger = (rightTrigger**4)*-1

        if(rightTrigger == 0):
            leftTrigger = (leftTrigger**4)*-1
        else:
            leftTrigger = (leftTrigger**4)*1
        
        if(LeftX < 0):
            LeftX = (LeftX**4)*-1
        else:
            LeftX = (LeftX**4)

        rightTrigger = rightTrigger *1.5        
        leftTrigger = leftTrigger *.9  
        LeftX = LeftX *.9     
        #this makes it turn slower
        if((rightTrigger < 0.1 and rightTrigger > -0.1) and (LeftX >= 0.5 or LeftX <= -0.5)):
            LeftX = LeftX * 0.66
        elif((LeftX < 0.1 and LeftX > -0.1) and (rightTrigger >= 0.5 or rightTrigger <=-0.5)):
            rightTrigger = rightTrigger * 0.66

        if((leftTrigger < 0.1 and leftTrigger > -0.1) and (LeftX >= 0.5 or LeftX <= -0.5)):
            LeftX = LeftX * 0.66
        elif((LeftX < 0.1 and LeftX > -0.1) and (leftTrigger >= 0.5 or leftTrigger <=-0.5)):
            leftTrigger = leftTrigger * 0.66

        self.drive.arcadeDrive(rightTrigger, LeftX, leftTrigger) # new arcade input
        self.adjustIntake()

    def disabledInit(self) -> None:
        # This just makes sure that our simulation code knows that the motor is off
        self.lf_motor.set(0)
        self.lr_motor.set(0)
        self.rf_motor.set(0)
        self.rr_motor.set(0)


    def adjustIntake(self):
        rt = self.controller2.getRightBumper()
        lt = self.controller2.getLeftBumper()



