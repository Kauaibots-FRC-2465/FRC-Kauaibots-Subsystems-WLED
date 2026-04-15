package frc.kauaibots.subsystems.wled;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.util.Arrays;
import java.io.IOException;

public final class AnimationUDPs {
    private final DatagramPacket[] packets;
    private final int packetsPerFrame;
    private final double[] frameDisplayDurationsSeconds;

    AnimationUDPs(
            DatagramPacket[] packets,
            int packetsPerFrame,
            double[] frameDisplayDurationsSeconds) {
        this.packets = copyPackets(packets);
        this.packetsPerFrame = packetsPerFrame;
        this.frameDisplayDurationsSeconds = Arrays.copyOf(
                frameDisplayDurationsSeconds,
                frameDisplayDurationsSeconds.length);
    }

    public int getPacketCount() {
        return packets.length;
    }

    public int getPacketsPerFrame() {
        return packetsPerFrame;
    }

    public int getFrameCount() {
        if (packetsPerFrame <= 0) {
            return 0;
        }
        return packets.length / packetsPerFrame;
    }

    public double getFrameDisplayDurationSeconds(int frameIndex) {
        return frameDisplayDurationsSeconds[frameIndex];
    }

    public void sendFrame(DatagramSocket socket, int frameIndex) throws IOException {
        int frameStartPacketIndex = frameIndex * packetsPerFrame;
        for (int packetOffset = 0; packetOffset < packetsPerFrame; packetOffset++) {
            socket.send(packets[frameStartPacketIndex + packetOffset]);
        }
    }

    byte[] getPacketDataCopy(int packetIndex) {
        return Arrays.copyOf(packets[packetIndex].getData(), packets[packetIndex].getLength());
    }

    boolean isEmpty() {
        return packets.length == 0
                || packetsPerFrame <= 0
                || frameDisplayDurationsSeconds.length == 0;
    }

    private static DatagramPacket[] copyPackets(DatagramPacket[] packets) {
        DatagramPacket[] copiedPackets = new DatagramPacket[packets.length];
        for (int i = 0; i < packets.length; i++) {
            copiedPackets[i] = copyPacket(packets[i]);
        }
        return copiedPackets;
    }

    private static DatagramPacket copyPacket(DatagramPacket packet) {
        byte[] payload = Arrays.copyOf(packet.getData(), packet.getLength());
        return new DatagramPacket(payload, payload.length, packet.getAddress(), packet.getPort());
    }
}
