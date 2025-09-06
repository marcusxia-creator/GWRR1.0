package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class TeleOp_Drive extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

    Drive drive = new drive(hardwareMap);

    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    waitForStart();

    while (!isStopRequested()) {
        drive.set(
                new Pose2d(
                        -gamepad1.right_stick_y * 0.5,
                        -gamepad1.right_stick_x * 0.5,
                        -gamepad1.left_stick_x * 0.5
                )
            );
        }
    }


    public class Drive {
        private DcMotorEx leftFront, leftRear, rightRear, rightFront;
        private List<DcMotorEx> motors;
        private GoBildaPinpointDriver pinpoint;

        public Drive(HardwareMap hardwareMap) {
            leftFront = hardwareMap.get(DcMotorEx.class, "FL_Motor");
            leftRear = hardwareMap.get(DcMotorEx.class, "BL_Motor");
            rightRear = hardwareMap.get(DcMotorEx.class, "BR_Motor");
            rightFront = hardwareMap.get(DcMotorEx.class, "FR_Motor");

            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);
            }

            if (RUN_USING_ENCODER) {
                setMode(RUN_USING_ENCODER);
            }

            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

            /// Setup pinpoint
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
            pinpoint.setEncoderResolution(19.89436789f, DistanceUnit.MM);
            pinpoint.setOffsets(-149.225, -131.2468, DistanceUnit.MM);

        }

        public void setMode(DcMotor.RunMode runMode) {
            for (DcMotorEx motor : motors) {
                motor.setMode(runMode);
            }
        }


        public void setWeightedDrivePower(Pose2d drivePower) {
            Pose2d vel = drivePower;

            if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                    + Math.abs(drivePower.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                        + VY_WEIGHT * Math.abs(drivePower.getY())
                        + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

                vel = new Pose2d(
                        VX_WEIGHT * drivePower.getX(),
                        VY_WEIGHT * drivePower.getY(),
                        OMEGA_WEIGHT * drivePower.getHeading()
                ).div(denom);
            }

            setDrivePower(vel);
        }

        @Override
        public void setMotorPowers(double v, double v1, double v2, double v3) {
            leftFront.setPower(v);
            leftRear.setPower(v1);
            rightRear.setPower(v2);
            rightFront.setPower(v3);
        }
    }



}
