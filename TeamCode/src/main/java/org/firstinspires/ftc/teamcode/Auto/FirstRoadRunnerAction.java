package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="First RR1.0 Actions")
public class FirstRoadRunnerAction extends LinearOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        Pose2d initialPose = new Pose2d(0,0,0);
        TrajectoryActionBuilder driveForwards = drive.actionBuilder(initialPose)
                        .lineToX(0);
        TrajectoryActionBuilder intakeSamples = driveForwards.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(-4,45), Math.toRadians(90))
                        .lineToX(12)
                        .afterDisp(12,new ServoAction(robot, 0.5))
                        .lineToXConstantHeading(10)
                        .stopAndAdd(new ServoAction(robot, 0.2));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(64)
                        .stopAndAdd(new PatientServoAction(robot,0))
                        .stopAndAdd(new PatientServoAction(robot, 0.5))
                        .lineToX(0)
                        .stopAndAdd(new PatientServoAction(robot, 1.0))
                        .waitSeconds(1)
                        .build());

    }

    public class ServoAction implements Action {
        RobotHardware robot;
        double position;

        public ServoAction(RobotHardware robot, double p)
        {
            this.robot = robot;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.intakeClawServo.setPosition(position);
            return false;
        }
    }

    public class PatientServoAction implements Action {
        RobotHardware robot;
        double position;
        ElapsedTime timer;
        boolean hasInitialized = false;

        public PatientServoAction(RobotHardware robot, double p)
        {
            this.robot = robot;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null || !hasInitialized){
                hasInitialized = true;
                timer = new ElapsedTime();
                robot.depositClawServo.setPosition(position);
                robot.liftMotorLeft.setPower(0);     // only run once
            }
            // do we need to keep running?
            robot.liftMotorRight.setPower(0.1);
            return timer.seconds() < 3;
            //return motor.getPosition() >= targetPosition;
        }
        /**
        public Action cool (Robot robot){
            return new cool(robot);
        }
         */
    }
}
