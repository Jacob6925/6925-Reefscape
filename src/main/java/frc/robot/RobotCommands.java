package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BallIntakeSubsys;
import frc.robot.subsystems.BallIntakeSubsys.BallIntakeSpeed;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ElevatorSubsys.ElevatorPosition;
import frc.robot.subsystems.PipeIntakeSubsys;
import frc.robot.subsystems.PipeIntakeSubsys.PipeIntakeSpeed;
import frc.robot.subsystems.WristSubsys;
import frc.robot.subsystems.WristSubsys.WristSetpoint;

public final class RobotCommands {
    private static ElevatorSubsys elevatorSubsys;
    private static PipeIntakeSubsys pipeIntakeSubsys;
    private static BallIntakeSubsys ballIntakeSubsys;
    private static WristSubsys wristSubsys;
    
    public static void init(ElevatorSubsys elevatorSubsys, PipeIntakeSubsys pipeIntakeSubsys, BallIntakeSubsys ballIntakeSubsys, WristSubsys wristSubsys) {
        RobotCommands.elevatorSubsys = elevatorSubsys;
        RobotCommands.pipeIntakeSubsys = pipeIntakeSubsys;
        RobotCommands.ballIntakeSubsys = ballIntakeSubsys;
        RobotCommands.wristSubsys = wristSubsys;
    }

    public static Command setSpeedPipeAndBall(PipeIntakeSpeed speed) {
        return new ParallelCommandGroup(
            pipeIntakeSubsys.setSpeedCommand(speed),
            ballIntakeSubsys.setSpeedFromPipeSpeedCommand(speed)
        );
    }

    public static Command resetElevator() {
        return new SequentialCommandGroup(
            wristSubsys.goTo(WristSetpoint.HOLDING_POS),
            new WaitCommand(0.25),
            elevatorSubsys.goTo(ElevatorPosition.MIN_HEIGHT)
        );
    }

    public static Command elevatorLevelActions(ElevatorPosition elevatorPosition, WristSetpoint wristSetpoint, double waitTime) {
        return new SequentialCommandGroup(
            elevatorSubsys.goTo(elevatorPosition),
            new WaitCommand(waitTime),
            wristSubsys.goTo(wristSetpoint)
        );
    }

    public static Command elevatorLevelActions(ElevatorPosition elevatorPosition, WristSetpoint wristSetpoint) {
        return elevatorLevelActions(elevatorPosition, wristSetpoint, 0.75);
    }

    public static Command moveElevAndIntakeBall(ElevatorPosition elevatorPosition) {
        return new SequentialCommandGroup(
            elevatorLevelActions(elevatorPosition, WristSetpoint.REMOVE_ALGAE, 0.75),
            new WaitCommand(0.5),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.INTAKE)
        );
    }

    public static Command resetIntakesAndElevator() {
        return new SequentialCommandGroup(
            pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.OFF),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF),
            resetElevator()
        );
    }

    public static Command humanPlayerActionsCommand() {
        return new SequentialCommandGroup(
            elevatorSubsys.goTo(ElevatorPosition.HUMAN_PLAYER_INTAKE),
            new WaitCommand(0.5),
            wristSubsys.goTo(WristSetpoint.HUMAN_PLAYER_INTAKE),
            setSpeedPipeAndBall(PipeIntakeSpeed.INTAKE)
        );
    }

    public static Command ejectIntakes(boolean pipe, boolean ball) {
        Command ejectCommand;
        if (pipe && ball) {
            ejectCommand = setSpeedPipeAndBall(PipeIntakeSpeed.EJECT);
        } else if (pipe) {
            ejectCommand = pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.OFF);
        } else {
            ejectCommand = ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF);
        }
        return new SequentialCommandGroup(
            ejectCommand,
            new WaitCommand(0.5),
            setSpeedPipeAndBall(PipeIntakeSpeed.OFF)
        );
    }

    public static Command liftAndShoot() {
        return new SequentialCommandGroup(
            // change to elevator actions to add wrist point
            elevatorSubsys.goTo(ElevatorPosition.MAX_HEIGHT),
            new WaitCommand(1),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.EJECT),
            new WaitCommand(2),
            elevatorSubsys.goTo(ElevatorPosition.MIN_HEIGHT),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF)
        );
    }

    public static BooleanSupplier elevatorAtSetpoint(ElevatorPosition goal) {
        return () -> Math.abs(elevatorSubsys.getMotorRotations() - goal.rotations) < 1;
    }

    public static Command outtakeBall(ElevatorPosition elevatorPosition) {
        return new SequentialCommandGroup(
            elevatorSubsys.goTo(elevatorPosition),
            new WaitCommand(1),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.EJECT)
        );
    }
}