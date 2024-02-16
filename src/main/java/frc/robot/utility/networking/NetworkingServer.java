package frc.robot.utility.networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.networking.types.NetworkingPose;
import frc.robot.utility.networking.types.NetworkingTag;
import frc.robot.utility.networking.types.NetworkingVector;

/**
 * UDP-based networking server that replaces NetworkTable
 */
public class NetworkingServer implements Runnable {
    private static final int MAX_PACKET_SIZE = 1024;
    private static final int DEFAULT_PORT = 5800;

    private final int port;
    private Thread thread;
    private DatagramSocket socket;
    private List<NetworkingObserver> observers = new ArrayList<NetworkingObserver>();

    /**
     * Networking server constructor
     * 
     * @param port Server port
     */
    public NetworkingServer(int port) {
        this.port = port;

        thread = new Thread(this, "ComputeNetworkingServer");
        thread.start();
    }

    /**
     * Networking server constructor (with port 5800)
     */
    public NetworkingServer() {
        this(DEFAULT_PORT);
    }

    /**
     * Listener method to specified call name
     * 
     * @param objectName Call name
     * @param call       Call method (with value)
     */
    public void subscribe(String objectName, NetworkingCall call) {
        observers.add(new NetworkingObserver(objectName, call));
    }

    /**
     * Listener method to specified call name with reply
     * 
     * @param objectName Call name
     * @param replyName  Reply name
     * @param call       Call method (with value)
     */
    public void subscribe(String objectName, String replyName, NetworkingCall call) {
        observers.add(new NetworkingObserver(objectName, replyName, call));
    }

    public void run() {
        try {
            socket = new DatagramSocket(port);
        } catch (SocketException err) {
            SmartDashboard.putString("server error", err.getMessage());
        }

        DatagramPacket packet = null;
        byte[] buffer = new byte[MAX_PACKET_SIZE];

        while (true) {
            packet = new DatagramPacket(buffer, buffer.length);

            try {
                socket.receive(packet);
            } catch (IOException err) {
                SmartDashboard.putString("server error", err.getMessage());
            }

            char packetType = getPacketType(buffer);

            switch (packetType) {
                case 'v': // 3D Vector
                    NetworkingVector vec = parseVector(buffer, packet.getLength());
                    updateValue(vec.objectName, vec.vector);
                    break;
                case 'p': // Pose (Two 3D vectors)
                    NetworkingPose pose = parsePose(buffer, packet.getLength());
                    updateValue(pose.objectName, pose);
                    break;
                case 't': // Apriltag (Pose + ID)
                    NetworkingTag tag = parseTag(buffer, packet.getLength());
                    updateValue(tag.objectName, tag);
                    break;
            }
        }
    }

    private char getPacketType(byte[] data) {
        return bytesToChar(Arrays.copyOfRange(data, 0, 1));
    }

    private <T> void updateValue(String name, T value) {
        for (NetworkingObserver observer : observers) {
            if (observer.objectName.equals(name)) {
                observer.update(value);
                return;
            }
        }
    }

    private NetworkingVector parseVector(byte[] data, int length) {
        if (data == null) {
            return null;
        }

        double x = bytesToDouble(Arrays.copyOfRange(data, 8, 16)); // 8 - 15: x of vector (double)
        double y = bytesToDouble(Arrays.copyOfRange(data, 16, 24)); // 16 - 23: y of vector (double)
        double z = bytesToDouble(Arrays.copyOfRange(data, 24, 32)); // 24 - 31: z of vector (double)
        String name = bytesToString(Arrays.copyOfRange(data, 32, length)); // 32 - N: name

        return new NetworkingVector(name, new Translation3d(x, y, z));
    }

    private NetworkingPose parsePose(byte[] data, int length) {
        if (data == null) {
            return null;
        }

        double x = bytesToDouble(Arrays.copyOfRange(data, 8, 16)); // 8 - 15: x of vector (double)
        double y = bytesToDouble(Arrays.copyOfRange(data, 16, 24)); // 16 - 23: y of vector (double)
        double z = bytesToDouble(Arrays.copyOfRange(data, 24, 32)); // 24 - 31: z of vector (double)

        double roll = bytesToDouble(Arrays.copyOfRange(data, 32, 40)); // 32- 39: roll of vector (double)
        double pitch = bytesToDouble(Arrays.copyOfRange(data, 40, 48)); // 40 - 47: pitch of vector (double)
        double yaw = bytesToDouble(Arrays.copyOfRange(data, 48, 56)); // 48 - 55: yaw of vector (double)

        String name = bytesToString(Arrays.copyOfRange(data, 56, length)); // 56 - N: name

        return new NetworkingPose(name, new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    private NetworkingTag parseTag(byte[] data, int length) {
        if (data == null) {
            return null;
        }

        double x = bytesToDouble(Arrays.copyOfRange(data, 8, 16)); // 8 - 15: x of vector (double)
        double y = bytesToDouble(Arrays.copyOfRange(data, 16, 24)); // 16 - 23: y of vector (double)
        double z = bytesToDouble(Arrays.copyOfRange(data, 24, 32)); // 24 - 31: z of vector (double)

        double roll = bytesToDouble(Arrays.copyOfRange(data, 32, 40)); // 32- 39: roll of vector (double)
        double pitch = bytesToDouble(Arrays.copyOfRange(data, 40, 48)); // 40 - 47: pitch of vector (double)
        double yaw = bytesToDouble(Arrays.copyOfRange(data, 48, 56)); // 48 - 55: yaw of vector (double)

        int id = bytesToInt(Arrays.copyOfRange(data, 56, 64)); // 56 - 63: id of tag
        String name = bytesToString(Arrays.copyOfRange(data, 64, length)); // 32 - N: name

        return new NetworkingTag(name, new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw), id);
    }

    // private void serializePacketData(byte[] packet, String name, Translation3d
    // vector) {
    // byte[] x = doubleToBytes(vector.getX());
    // byte[] y = doubleToBytes(vector.getY());
    // byte[] z = doubleToBytes(vector.getZ());
    // //byte[] nameData = name.getBytes();

    // int doubleLength = x.length;

    // System.arraycopy(x, 0, packet, 0, doubleLength);
    // System.arraycopy(y, 0, packet, doubleLength, doubleLength);
    // System.arraycopy(z, 0, packet, doubleLength * 2, doubleLength);
    // //System.arraycopy(nameData, 0, packet, doubleLength * 3, nameData.length);
    // }

    // private NetworkingVector parsePacketData(byte[] data, int length) {
    // if (data == null) {
    // return null;
    // }

    // double vecX = bytesToDouble(Arrays.copyOfRange(data, 0, 8)); // 0 - 7: x of
    // vector (double)
    // double vecY = bytesToDouble(Arrays.copyOfRange(data, 8, 16)); // 8 - 15: y of
    // vector (double)
    // double vecZ = bytesToDouble(Arrays.copyOfRange(data, 16, 24)); // 16 - 23: z
    // of vector (double)

    // Translation3d vector = new Translation3d(vecX, vecY, vecZ);

    // String objectName = "";
    // try {
    // objectName = new String(data, 24, length - 24, "UTF-8"); // 24 - end: object
    // name (string)
    // } catch (UnsupportedEncodingException err) {
    // SmartDashboard.putString("server error", err.getMessage());
    // }

    // return new NetworkingVector(objectName.replaceAll("[^\\x20-\\x7E]", ""),
    // vector);
    // }

    // private byte[] doubleToBytes(double d) {
    // ByteBuffer byteBuffer = ByteBuffer.allocate(Double.BYTES);
    // byteBuffer.putDouble(d);
    // return byteBuffer.array();
    // }

    private double bytesToDouble(byte[] buffer) {
        return ByteBuffer.wrap(buffer).getDouble();
    }

    private char bytesToChar(byte[] buffer) {
        return (char)buffer[0];
    }

    private String bytesToString(byte[] buffer) {
        return new String(buffer, StandardCharsets.UTF_8).replaceAll("[^\\x20-\\x7E]", "");
    }

    private int bytesToInt(byte[] buffer) {
        return ByteBuffer.wrap(buffer).getInt();
    }
}