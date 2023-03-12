package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Sensors")
public class SensorExamples extends OpMode {

    ColorSensor colorSensor;
    IMU imu;
    IMU.Parameters imuParameters;
    AnalogInput potentiometer;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(imuParameters);

        potentiometer = hardwareMap.analogInput.get("potentiometer");

    }

    private double getPotentiometerAngle() {
        return potentiometer.getVoltage() * 81.8;
    }

    @Override
    public void loop() {
        telemetry.addLine("Color Sensor");
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("Alpha: ", colorSensor.alpha());
        telemetry.addLine();
        telemetry.addLine("IMU");
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        telemetry.addData("Yaw", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("xAngular Vel", angularVelocity.xRotationRate);
        telemetry.addData("yAngular Vel", angularVelocity.yRotationRate);
        telemetry.addData("zAngular Vel", angularVelocity.zRotationRate);
        telemetry.addLine();
        telemetry.addLine("Potentiometer");
        telemetry.addData("Angle", getPotentiometerAngle());
        telemetry.addData("Voltage", potentiometer.getVoltage());
    }
}
