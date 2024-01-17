package frc.robot.OI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.CommandFactory;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.TwistCommand;
import frc.robot.commands.XCommand;

public class OperatorInterface {

        private final Joystick driveJoystick = new Joystick(RobotConstants.Ports.CONTROLLER.JOYSTICK);

        public OperatorInterface(CommandFactory commandFactory, RobotContainer robotContainer) {

                new JoystickButton(driveJoystick, 1).whileTrue(new TwistCommand());

                new JoystickButton(driveJoystick,11).whileTrue(commandFactory.gyroResetCommand());
        
                new JoystickButton(driveJoystick, 3).whileTrue((new XCommand()));        }
}