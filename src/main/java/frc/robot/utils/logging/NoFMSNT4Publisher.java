package frc.robot.utils.logging;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;

/**
 * NoFMSNT4Publisher — publishes AdvantageKit log data over NetworkTables ONLY
 * when the robot is NOT connected to the field (FMS).
 *
 * ============================================================================
 * WHY THIS EXISTS (read me, future students!)
 * ============================================================================
 * During a competition match, the field's Wi-Fi network is shared by EVERY
 * team playing at the same time. The robot normally streams its whole log
 * (hundreds of values, 50 times per second) to the driver station over NT.
 * On the field that traffic fights with the Driver Station connection and
 * other teams' robots for limited bandwidth — for zero benefit, since nobody
 * can watch dashboards mid-match anyway.
 *
 * The .wpilog file on the RoboRIO keeps recording NO MATTER WHAT — so we lose
 * nothing by pausing the live stream. After the match, pull the log from the
 * RIO (or let it upload) and everything is there.
 *
 * Pattern borrowed directly from Team 6328 (Mechanical Advantage), whose
 * class of the same name does exactly this.
 * ============================================================================
 */
public class NoFMSNT4Publisher implements LogDataReceiver {
  private final NetworkTable akitTable;
  private LogTable lastTable = new LogTable(0);
  private final IntegerPublisher timestampPublisher;
  private final Map<String, GenericPublisher> publishers = new HashMap<>();
  private final Map<String, String> units = new HashMap<>();

  /** Creates a new NoFMSNT4Publisher. */
  public NoFMSNT4Publisher() {
    akitTable = NetworkTableInstance.getDefault().getTable("/AdvantageKit");
    timestampPublisher =
        akitTable.getIntegerTopic(LogDataReceiver.timestampKey.substring(1))
            .publish(PubSubOption.sendAll(true));
  }

  public void putTable(LogTable table) {
    // THE WHOLE POINT: skip all publishing while attached to FMS.
    // The WPILOG file writer still receives every cycle regardless.
    if (DriverStation.isFMSAttached()) {
      return;
    }

    // Send timestamp
    timestampPublisher.set(table.getTimestamp(), table.getTimestamp());

    // Get old and new data
    Map<String, LogValue> newMap = table.getAll(false);
    Map<String, LogValue> oldMap = lastTable.getAll(false);

    // Encode new/changed fields
    for (Map.Entry<String, LogValue> field : newMap.entrySet()) {
      // Check if field has changed
      LogValue newValue = field.getValue();
      if (newValue.equals(oldMap.get(field.getKey()))) {
        continue;
      }

      // Create publisher if necessary
      String key = field.getKey().substring(1);
      String unit = field.getValue().unitStr;
      GenericPublisher publisher = publishers.get(key);
      if (publisher == null) {
        publisher =
            akitTable
                .getTopic(key)
                .genericPublish(field.getValue().getNT4Type(), PubSubOption.sendAll(true));
        publishers.put(key, publisher);

        // Set initial unit
        if (unit != null) {
          akitTable.getTopic(key).setProperty("unit", "\"" + unit + "\"");
          units.put(key, unit);
        }
      }

      // Check if unit changed
      if (unit != null && !unit.equals(units.get(key))) {
        akitTable.getTopic(key).setProperty("unit", "\"" + unit + "\"");
        units.put(key, unit);
      }

      // Write new data
      switch (field.getValue().type) {
        case Raw:
          publisher.setRaw(field.getValue().getRaw(), table.getTimestamp());
          break;
        case Boolean:
          publisher.setBoolean(field.getValue().getBoolean(), table.getTimestamp());
          break;
        case BooleanArray:
          publisher.setBooleanArray(field.getValue().getBooleanArray(), table.getTimestamp());
          break;
        case Integer:
          publisher.setInteger(field.getValue().getInteger(), table.getTimestamp());
          break;
        case IntegerArray:
          publisher.setIntegerArray(field.getValue().getIntegerArray(), table.getTimestamp());
          break;
        case Float:
          publisher.setFloat(field.getValue().getFloat(), table.getTimestamp());
          break;
        case FloatArray:
          publisher.setFloatArray(field.getValue().getFloatArray(), table.getTimestamp());
          break;
        case Double:
          publisher.setDouble(field.getValue().getDouble(), table.getTimestamp());
          break;
        case DoubleArray:
          publisher.setDoubleArray(field.getValue().getDoubleArray(), table.getTimestamp());
          break;
        case String:
          publisher.setString(field.getValue().getString(), table.getTimestamp());
          break;
        case StringArray:
          publisher.setStringArray(field.getValue().getStringArray(), table.getTimestamp());
          break;
      }
    }

    // Update last table
    lastTable = table;
  }
}
