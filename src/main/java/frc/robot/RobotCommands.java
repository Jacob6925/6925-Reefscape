package frc.robot;

import java.util.EventListener;

import edu.wpi.first.wpilibj2.command.Command;
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
    private final ElevatorSubsys elevatorSubsys;
    private final PipeIntakeSubsys pipeIntakeSubsys;
    private final BallIntakeSubsys ballIntakeSubsys;
    private final WristSubsys wristSubsys;
    
    public RobotCommands(ElevatorSubsys elevatorSubsys, PipeIntakeSubsys pipeIntakeSubsys, BallIntakeSubsys ballIntakeSubsys, WristSubsys wristSubsys) {
        this.elevatorSubsys = elevatorSubsys;
        this.pipeIntakeSubsys = pipeIntakeSubsys;
        this.ballIntakeSubsys = ballIntakeSubsys;
        this.wristSubsys = wristSubsys;
    }

    public Command setSpeedPipeAndBall(PipeIntakeSpeed speed) {
        return new SequentialCommandGroup(
            pipeIntakeSubsys.setSpeedCommand(speed),
            ballIntakeSubsys.setSpeedFromPipeSpeedCommand(speed)
        );
    }

    public Command resetElevator() {
        return new SequentialCommandGroup(
            wristSubsys.goTo(WristSetpoint.START_POS),
            new WaitCommand(0.25),
            elevatorSubsys.goTo(ElevatorPosition.MIN_HEIGHT)
        );
    }

    public Command elevatorLevelActions(ElevatorPosition elevatorPosition, WristSetpoint wristSetpoint) {
        return new SequentialCommandGroup(
            elevatorSubsys.goTo(elevatorPosition),
            new WaitCommand(0.5),
            wristSubsys.goTo(wristSetpoint)
        );
    }

    public Command moveElevAndIntakeBall(ElevatorPosition elevatorPosition) {
        return new SequentialCommandGroup(
            elevatorLevelActions(elevatorPosition, WristSetpoint.REMOVE_ALGAE).until(() -> Math.abs(elevatorSubsys.getMotorRotations() - elevatorPosition.rotations) > 1),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.INTAKE)
        );
    }

    public Command resetIntakesAndElevator() {
        return new SequentialCommandGroup(
            pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.OFF),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF),
            resetElevator()
        );
    }

    public Command humanPlayerActionsCommand() {
        return new SequentialCommandGroup(
            elevatorSubsys.goTo(ElevatorPosition.HUMAN_PLAYER_INTAKE),
            new WaitCommand(0.5),
            wristSubsys.goTo(WristSetpoint.HUMAN_PLAYER_INTAKE)
        ).andThen(
            pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.INTAKE),
            ballIntakeSubsys.setSpeedFromPipeSpeedCommand(PipeIntakeSpeed.INTAKE)
        );
    }
}