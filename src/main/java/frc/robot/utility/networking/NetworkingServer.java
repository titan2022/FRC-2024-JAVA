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
import frc.robot.utility.networking.types.NetworkingPose;
import frc.robot.utility.networking.types.NetworkingTag;
import frc.robot.utility.networking.types.NetworkingVector;

/**
 * UDP-based networking server that replaces NetworkTable
 */
public class NetworkingServer implements Runnable {
    private static final int MAX_PACKET_SIZE = 128;
    private static final int DEFAULT_PORT = 5804;

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
            System.out.println(err.getMessage());
        }

        DatagramPacket packet = null;
        byte[] buffer = new byte[MAX_PACKET_SIZE];

        while (true) {
            packet = new DatagramPacket(buffer, buffer.length);

            try {
                socket.receive(packet);
            } catch (IOException err) {
                System.out.println(err.getMessage());
            }

            // for (int i = 0; i < 72; i++) {
            //     System.out.print(String.format("%8s", Integer.toBinaryString(buffer[i] & 0xFF)).replace(' ', '0') + " ");
            // }
            // System.out.println("\r\n");

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

        String name = bytesToString(Arrays.copyOfRange(data, 1, 16));
        double x = bytesToDouble(Arrays.copyOfRange(data, 16, 24));
        double y = bytesToDouble(Arrays.copyOfRange(data, 24, 32));
        double z = bytesToDouble(Arrays.copyOfRange(data, 32, 40));

        return new NetworkingVector(name, new Translation3d(x, y, z));
    }

    private NetworkingPose parsePose(byte[] data, int length) {
        if (data == null) {
            return null;
        }

        String name = bytesToString(Arrays.copyOfRange(data, 1, 16));
        double x = bytesToDouble(Arrays.copyOfRange(data, 16, 24));
        double y = bytesToDouble(Arrays.copyOfRange(data, 24, 32));
        double z = bytesToDouble(Arrays.copyOfRange(data, 32, 40));

        double roll = bytesToDouble(Arrays.copyOfRange(data, 40, 48));
        double pitch = bytesToDouble(Arrays.copyOfRange(data, 48, 56));
        double yaw = bytesToDouble(Arrays.copyOfRange(data, 56, 64));

        return new NetworkingPose(name, new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    private NetworkingTag parseTag(byte[] data, int length) {
        if (data == null) {
            return null;
        }

        String name = bytesToString(Arrays.copyOfRange(data, 1, 16));
        double x = bytesToDouble(Arrays.copyOfRange(data, 16, 24));
        double y = bytesToDouble(Arrays.copyOfRange(data, 24, 32));
        double z = bytesToDouble(Arrays.copyOfRange(data, 32, 40));
        
        double roll = bytesToDouble(Arrays.copyOfRange(data, 40, 48));
        double pitch = bytesToDouble(Arrays.copyOfRange(data, 48, 56));
        double yaw = bytesToDouble(Arrays.copyOfRange(data, 56, 64));

        int id = bytesToInt(Arrays.copyOfRange(data, 64, 68));

        return new NetworkingTag(name, new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw), id);
    }

    private byte[] changeEndian(byte[] buffer) {
        for(int i = 0; i < buffer.length / 2; i++)
        {
            byte temp = buffer[i];
            buffer[i] = buffer[buffer.length - i - 1];
            buffer[buffer.length - i - 1] = temp;
        }
        return buffer;
    }

    private double bytesToDouble(byte[] buffer) {
        return ByteBuffer.wrap(changeEndian(buffer)).getDouble();
    }

    private int bytesToInt(byte[] buffer) {
        return ByteBuffer.wrap(changeEndian(buffer)).getInt();
    }

    private char bytesToChar(byte[] buffer) {
        return (char)buffer[0];
    }

    private String bytesToString(byte[] buffer) {
        return new String(buffer, StandardCharsets.UTF_8).replaceAll("\0", "");
    }
}