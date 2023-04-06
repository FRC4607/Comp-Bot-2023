package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem responsible for the LED Lights.
 */
public class IndicatorSubsystem extends SubsystemBase {

    /**
     * Enumerator that defines the current segment of the game.
     */
    public enum GamePeriod {
        DISABLED,
        AUTO,
        TELEOP,
        ENDGAME
    }

    /**
     * Enumerator that defines the alliance color.
     */
    public static enum AllianceColor {
        NONE(0),
        RED(1),
        BLUE(2);

        public final int m_value;

        private AllianceColor(int value) {
            m_value = value;
        }
    }

    /**
     * Enumerator that defines the requested item.
     */
    public enum PieceIndicatorState {
        NONE(0),
        CONE(1),
        CUBE(2);

        public final int m_value;

        private PieceIndicatorState(int value) {
            m_value = value;
        }
    }

    private GamePeriod m_previousGamePeriod = GamePeriod.DISABLED;
    private AllianceColor m_previousAllianceColor = AllianceColor.NONE;
    private PieceIndicatorState m_previousIndicatorState = PieceIndicatorState.NONE;
    private PieceIndicatorState m_currentIndicatorState = PieceIndicatorState.NONE;

    private boolean m_endgameLEDHigh = true;

    private NetworkTableEntry m_allianceColorEntry;
    private StringPublisher m_pieceEntry;
    private DoublePublisher m_timeEntry;

    private int m_refreshCounter;
    private int m_effectTimer;

    // private final AddressableLED m_elementLED1;
    // private final AddressableLED m_elementLED2;
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(52);
    private AddressableLED m_indicatorLED;
    private AddressableLEDBuffer m_indicatorLEDBuffer = new AddressableLEDBuffer(38);

    /**
     * The subsystem responsible for the LED lights.
     */
    public IndicatorSubsystem() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmsInfo = inst.getTable("FMSInfo");
        m_allianceColorEntry = fmsInfo.getEntry("IsRedAlliance");

        NetworkTable colorTable = inst.getTable("Boat");
        m_pieceEntry = colorTable.getStringTopic("Gas").publish();
        m_timeEntry = colorTable.getDoubleTopic("GasTime").publish();

        m_indicatorLED = new AddressableLED(0);
        m_indicatorLED.setLength(m_indicatorLEDBuffer.getLength());
        m_indicatorLED.start();

        // m_elementLED1 = new AddressableLED(1);
        // m_elementLED1.setLength(m_ledBuffer.getLength());
        // m_elementLED1.start();

        // m_elementLED2 = new AddressableLED(2);
        // m_elementLED2.setLength(m_ledBuffer.getLength());
        // m_elementLED2.start();
        

    }

    /**
     * The method used to change the color of the forward-mounted LED strips.
     *
     * @param request The enumerator that defines the requested game piece.
     */
    public void setIndicator(PieceIndicatorState request) {
        m_currentIndicatorState = request;
    }

    @Override
    public void periodic() {
        AllianceColor newAllianceColor = m_allianceColorEntry.getBoolean(false) ? AllianceColor.RED
                : AllianceColor.BLUE;

        GamePeriod newGamePeriod;

        if (DriverStation.isAutonomousEnabled()) {
            newGamePeriod = GamePeriod.AUTO;
        } else if (DriverStation.isTeleopEnabled()) {
            newGamePeriod = DriverStation.getMatchTime() > 30 ? GamePeriod.TELEOP : GamePeriod.ENDGAME;
        } else {
            newGamePeriod = GamePeriod.DISABLED;
        }

        boolean indicatorLEDsUpdated = false;
        if (m_previousIndicatorState.m_value != m_currentIndicatorState.m_value) {
            int[] rgb;
            switch (m_currentIndicatorState) {
                case CONE:
                    rgb = new int[] { 235, 255, 0 };
                    break;

                case CUBE:
                    rgb = new int[] { 66, 6, 156 };
                    break;

                case NONE:
                    rgb = newAllianceColor.m_value == AllianceColor.RED.m_value ? new int[] { 255, 0, 0 }
                            : new int[] { 0, 0, 255 };
                    break;

                default:
                    rgb = newAllianceColor.m_value == AllianceColor.RED.m_value ? new int[] { 255, 0, 0 }
                            : new int[] { 0, 0, 255 };
                    break;
            }

            for (int i = 0; i < m_indicatorLEDBuffer.getLength(); i++) {
                m_indicatorLEDBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
            }

            m_indicatorLED.setData(m_indicatorLEDBuffer);

            m_previousIndicatorState = m_currentIndicatorState;
            indicatorLEDsUpdated = true;
        }

        if (m_previousAllianceColor.m_value != newAllianceColor.m_value) {
            int[] rgb = newAllianceColor.m_value == AllianceColor.RED.m_value ? new int[] { 255, 0, 0 }
                    : new int[] { 0, 0, 255 };

            // for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            // m_ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
            // }
            // m_elementLED1.setData(m_indicatorLEDBuffer);
            // m_elementLED2.setData(m_indicatorLEDBuffer);

            if (!indicatorLEDsUpdated && m_previousIndicatorState.m_value == PieceIndicatorState.NONE.m_value) {
                for (int i = 0; i < m_indicatorLEDBuffer.getLength(); i++) {
                    m_indicatorLEDBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
                }
                m_indicatorLED.setData(m_indicatorLEDBuffer);
            }

            m_previousAllianceColor = newAllianceColor;
        }

        m_refreshCounter++;

        if (m_refreshCounter > 50) {
            m_refreshCounter = 0;

            // if (newGamePeriod == GamePeriod.ENDGAME) {
            // int[] rgb = newAllianceColor == AllianceColor.RED ? new int[] {
            // m_endgameLEDHigh ? 50 : 255, 0, 0 }
            // : new int[] { 0, 0, m_endgameLEDHigh ? 50 : 255 };
            // m_endgameLEDHigh = !m_endgameLEDHigh;

            // for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            // m_ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
            // }

            // // m_elementLED1.setData(m_ledBuffer);
            // // m_elementLED2.setData(m_ledBuffer);
            // }

        }
        m_pieceEntry.accept(m_currentIndicatorState.name());
        m_timeEntry.accept(DriverStation.getMatchTime());
    }
}
