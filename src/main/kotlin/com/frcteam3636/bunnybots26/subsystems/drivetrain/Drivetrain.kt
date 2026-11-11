package com.frcteam3636.bunnybots26.subsystems.drivetrain

import com.frcteam3636.bunnybots26.robot.Robot
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.wpilibj2.command.Subsystem
import com.frcteam3636.bunnybots26.Constants
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj2.command.Command
import java.util.*
import kotlin.math.*


object Drivetrain : Subsystem {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> DrivetrainIOSim()
        Robot.Model.COMPETITION -> DrivetrainIOReal.fromKrakenSwerve()
    }

    val leftConfig = SparkMaxConfig().apply {
        SparkMaxConfig().apply {
            idleMode(SparkBaseConfig.IdleMode.kBrake)
        }
        inverted(true)
    }
    val rightConfig = SparkMaxConfig().apply{
        SparkMaxConfig().apply {
            idleMode(SparkBaseConfig.IdleMode.kBrake)
        }
        inverted(false)
    }


    private var leftmotor1 = SparkMax(Constants.Drivetrain.MOTOR_LEFT_1, SparkLowLevel.MotorType.kBrushless).apply{
        configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }
    private var leftmotor2 = SparkMax(Constants.Drivetrain.MOTOR_LEFT_2, SparkLowLevel.MotorType.kBrushless).apply{
        configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    private var rightmotor1 = SparkMax(Constants.Drivetrain.MOTOR_RIGHT_1, SparkLowLevel.MotorType.kBrushless).apply{
        configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }
    private var rightmotor2 = SparkMax(Constants.Drivetrain.MOTOR_RIGHT_2, SparkLowLevel.MotorType.kBrushless).apply{
        configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }


    private var drivetrain = MecanumDrive(
                            leftmotor1,
                            leftmotor2,
                            rightmotor1,
                            rightmotor2
    )

    fun driveWithController(controller: XboxController): Command =
        run {
            var drive = 0.0
            var turn = 0.0

            if (abs(controller.leftX) >= 0.10) {
                drive = Constants.Drivetrain.TURNING_SENSITIVITY * controller.leftX
            }

            if (abs(controller.rightY) >= 0.10) {
                turn = Constants.Drivetrain.DRIVING_SENSITIVITY * controller.rightY
            }

            drivetrain.driveCartesian()
        }


}