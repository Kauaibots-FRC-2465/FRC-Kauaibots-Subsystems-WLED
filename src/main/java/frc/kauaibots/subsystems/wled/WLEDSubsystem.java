package frc.kauaibots.subsystems.wled;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Arrays;

import javax.imageio.ImageIO;

import com.madgag.gif.fmsware.GifDecoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WLEDSubsystem extends SubsystemBase {
    private static final int CHANNELS = 4;

    private final WLEDConfig config;

    private final DatagramSocket socket;

    private ActivePlayback activePlayback;

    public static final class PreparedPlaybackPackets {
        private final DatagramPacket[] packets;
        private final int packetsPerFrame;
        private final double[] frameDisplayDurationsSeconds;

        private PreparedPlaybackPackets(
                DatagramPacket[] packets,
                int packetsPerFrame,
                double[] frameDisplayDurationsSeconds) {
            this.packets = packets;
            this.packetsPerFrame = packetsPerFrame;
            this.frameDisplayDurationsSeconds = frameDisplayDurationsSeconds;
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

        byte[] getPacketDataCopy(int packetIndex) {
            return Arrays.copyOf(packets[packetIndex].getData(), packets[packetIndex].getLength());
        }

        private boolean isEmpty() {
            return packets.length == 0
                    || packetsPerFrame <= 0
                    || frameDisplayDurationsSeconds.length == 0;
        }
    }

    private static final class ActivePlayback {
        private final PreparedPlaybackPackets playback;
        private final Timer frameTimer = new Timer();
        private int currentFrameIndex;
        private int lastSentFrameIndex;

        private ActivePlayback(PreparedPlaybackPackets playback) {
            this.playback = playback;
            this.currentFrameIndex = 0;
            this.lastSentFrameIndex = -1;
            this.frameTimer.start();
            this.frameTimer.reset();
        }
    }

    private static final class FrameData {
        private final byte[][][] pixels;
        private final double displayDurationSeconds;

        private FrameData(byte[][][] pixels, double displayDurationSeconds) {
            this.pixels = pixels;
            this.displayDurationSeconds = displayDurationSeconds;
        }
    }

    public WLEDSubsystem(WLEDConfig config) {
        this.config = config;

        DatagramSocket createdSocket = null;
        try {
            createdSocket = new DatagramSocket();
        } catch (SocketException e) {
            DriverStation.reportError("WLED socket initialization failed: " + e.getMessage(), false);
        }
        socket = createdSocket;
        activePlayback = null;
    }

    @Override
    public void periodic() {
        if (socket == null || activePlayback == null || activePlayback.playback.isEmpty()) {
            return;
        }

        double frameDurationSeconds =
                activePlayback.playback.getFrameDisplayDurationSeconds(activePlayback.currentFrameIndex);

        if (activePlayback.currentFrameIndex != activePlayback.lastSentFrameIndex) {
            int packetsPerFrame = activePlayback.playback.getPacketsPerFrame();
            int frameStartPacketIndex = activePlayback.currentFrameIndex * packetsPerFrame;
            for (int packetOffset = 0; packetOffset < packetsPerFrame; packetOffset++) {
                DatagramPacket packet = activePlayback.playback.packets[frameStartPacketIndex + packetOffset];
                try {
                    socket.send(packet);
                } catch (IOException e) {
                    DriverStation.reportError("WLED packet send failed: " + e.getMessage(), false);
                    return;
                }
            }
            activePlayback.lastSentFrameIndex = activePlayback.currentFrameIndex;
        }

        if (frameDurationSeconds <= 0.0 || activePlayback.frameTimer.hasElapsed(frameDurationSeconds)) {
            activePlayback.currentFrameIndex++;
            if (activePlayback.currentFrameIndex >= activePlayback.playback.getFrameCount()) {
                activePlayback.currentFrameIndex = 0;
            }
            activePlayback.frameTimer.restart();
        }
    }

    public PreparedPlaybackPackets prepareMarquee(String path) {
        try {
            BufferedImage image = readImage(path);
            byte[][][] imageData = getImageData(image);
            int frameCount = image.getWidth();
            return buildPlaybackPackets(
                    frameCount,
                    frameIndex -> new FrameData(extractWrappedFrame(imageData, frameIndex), 0.0));
        } catch (BadImageFormatException | IOException e) {
            return reportPreparationFailure("marquee", path, e);
        }
    }

    public PreparedPlaybackPackets prepareHorizontalAnimationStrip(String path) {
        try {
            BufferedImage image = readImage(path);
            int realWidth = image.getWidth();
            if (realWidth % config.width() != 0) {
                throw new BadImageFormatException(
                        "Animation width must be an integer multiple of " + config.width() + " pixels.");
            }
            byte[][][] imageData = getImageData(image);
            int frameCount = realWidth / config.width();
            return buildPlaybackPackets(
                    frameCount,
                    frameIndex -> new FrameData(extractClippedFrame(imageData, frameIndex * config.width()), 0.0));
        } catch (BadImageFormatException | IOException e) {
            return reportPreparationFailure("animation", path, e);
        }
    }

    public PreparedPlaybackPackets prepareImage(String path) {
        try {
            BufferedImage image = readImage(path);
            byte[][][] imageData = getImageData(image);
            DatagramPacket[] packets = buildPacketsForFrame(extractClippedFrame(imageData, 0));
            return new PreparedPlaybackPackets(packets, packets.length, new double[] {0.0});
        } catch (BadImageFormatException | IOException e) {
            return reportPreparationFailure("image", path, e);
        }
    }

    public PreparedPlaybackPackets prepareGIF(String path) {
        File gifFile = new File(path);
        try (InputStream inputStream = gifFile.toURI().toURL().openStream()) {
            GifDecoder gifDecoder = new GifDecoder();
            int status = gifDecoder.read(inputStream);
            if (status == GifDecoder.STATUS_OPEN_ERROR) {
                throw new BadImageFormatException("GIF file could not be opened.");
            }
            if (status != GifDecoder.STATUS_OK) {
                throw new BadImageFormatException("GIF could not be decoded.");
            }

            int frameCount = gifDecoder.getFrameCount();
            if (frameCount <= 0) {
                throw new BadImageFormatException("GIF contains no frames.");
            }

            validateGifDimensions(gifDecoder.getFrame(0), "GIF logical screen");
            return buildPlaybackPackets(frameCount, frameIndex -> buildGifFrame(gifDecoder, frameIndex));
        } catch (BadImageFormatException | IOException e) {
            return reportPreparationFailure("gif", path, e);
        }
    }

    public Command showMarquee(String path) {
        PreparedPlaybackPackets playbackPackets = prepareMarquee(path);
        return runOnce(() -> setActivePlayback(playbackPackets)).ignoringDisable(true);
    }

    public Command showHorizontalAnimationStrip(String path) {
        PreparedPlaybackPackets playbackPackets = prepareHorizontalAnimationStrip(path);
        return runOnce(() -> setActivePlayback(playbackPackets)).ignoringDisable(true);
    }

    public Command showImage(String path) {
        PreparedPlaybackPackets playbackPackets = prepareImage(path);
        return runOnce(() -> setActivePlayback(playbackPackets)).ignoringDisable(true);
    }

    public Command showGIF(String path) {
        PreparedPlaybackPackets playbackPackets = prepareGIF(path);
        return runOnce(() -> setActivePlayback(playbackPackets)).ignoringDisable(true);
    }

    public void setActivePlayback(PreparedPlaybackPackets playbackPackets) {
        if (playbackPackets == null || playbackPackets.isEmpty()) {
            activePlayback = null;
            return;
        }

        activePlayback = new ActivePlayback(playbackPackets);
    }

    private PreparedPlaybackPackets buildPlaybackPackets(int frameCount, FrameBuilder frameBuilder)
            throws IOException, BadImageFormatException {
        if (frameCount <= 0) {
            return new PreparedPlaybackPackets(new DatagramPacket[0], 0, new double[0]);
        }

        FrameData firstFrameData = frameBuilder.buildFrame(0);
        DatagramPacket[] firstFramePackets = buildPacketsForFrame(firstFrameData.pixels);
        int packetsPerFrame = firstFramePackets.length;
        DatagramPacket[] packets = new DatagramPacket[frameCount * packetsPerFrame];
        double[] frameDisplayDurationsSeconds = new double[frameCount];
        System.arraycopy(firstFramePackets, 0, packets, 0, packetsPerFrame);
        frameDisplayDurationsSeconds[0] = firstFrameData.displayDurationSeconds;

        int packetIndex = packetsPerFrame;
        for (int frameIndex = 1; frameIndex < frameCount; frameIndex++) {
            FrameData frameData = frameBuilder.buildFrame(frameIndex);
            DatagramPacket[] framePackets = buildPacketsForFrame(frameData.pixels);
            System.arraycopy(framePackets, 0, packets, packetIndex, packetsPerFrame);
            frameDisplayDurationsSeconds[frameIndex] = frameData.displayDurationSeconds;
            packetIndex += packetsPerFrame;
        }

        return new PreparedPlaybackPackets(packets, packetsPerFrame, frameDisplayDurationsSeconds);
    }

    private DatagramPacket[] buildPacketsForFrame(byte[][][] matrix) throws UnknownHostException {
        int packetsPerFrame = (matrix.length + config.rowsPerPacket() - 1) / config.rowsPerPacket();
        DatagramPacket[] packets = new DatagramPacket[packetsPerFrame];
        InetAddress address = InetAddress.getByName(config.controllerAddress());

        for (int packetIndex = 0; packetIndex < packetsPerFrame; packetIndex++) {
            byte[] payload = new byte[10 + (config.width() * config.rowsPerPacket() * CHANNELS)];
            payload[0] = 0b01000001;
            payload[1] = 0b00000001;
            payload[2] = 0b00011011;
            payload[3] = 0b00000001;

            int offset = config.width() * config.rowsPerPacket() * CHANNELS * packetIndex;
            payload[4] = (byte) ((offset >> 24) & 0xFF);
            payload[5] = (byte) ((offset >> 16) & 0xFF);
            payload[6] = (byte) ((offset >> 8) & 0xFF);
            payload[7] = (byte) (offset & 0xFF);

            int dataLength = config.width() * config.rowsPerPacket() * CHANNELS;
            payload[8] = (byte) ((dataLength >> 8) & 0xFF);
            payload[9] = (byte) (dataLength & 0xFF);

            int bufferOffset = 9;
            for (int y = 0; y < config.rowsPerPacket(); y++) {
                for (int x = 0; x < config.width(); x++) {
                    int sourceY = y + (packetIndex * config.rowsPerPacket());
                    if (sourceY < matrix.length) {
                        payload[++bufferOffset] = matrix[sourceY][x][1];
                        payload[++bufferOffset] = matrix[sourceY][x][2];
                        payload[++bufferOffset] = matrix[sourceY][x][3];
                        payload[++bufferOffset] = matrix[sourceY][x][0];
                    }
                }
            }

            packets[packetIndex] = new DatagramPacket(payload, payload.length, address, config.ddpPort());
        }

        return packets;
    }

    private BufferedImage readImage(String path) throws IOException, BadImageFormatException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Unsupported image format or file not found.");
        }
        if (image.getHeight() < config.height()) {
            throw new BadImageFormatException("Image height must be at least " + config.height() + " pixels.");
        }
        return image;
    }

    private FrameData buildGifFrame(GifDecoder gifDecoder, int frameIndex) throws BadImageFormatException {
        BufferedImage frame = gifDecoder.getFrame(frameIndex);
        validateGifDimensions(frame, "GIF frame " + frameIndex);
        return new FrameData(getImageData(frame), gifDecoder.getDelay(frameIndex) / 1000.0);
    }

    private void validateGifDimensions(BufferedImage frame, String subject) throws BadImageFormatException {
        if (frame == null) {
            throw new BadImageFormatException(subject + " is unavailable.");
        }
        if (frame.getWidth() != config.width() || frame.getHeight() != config.height()) {
            throw new BadImageFormatException(
                    subject + " must be exactly " + config.width() + "x" + config.height() + " pixels.");
        }
    }

    private byte[][][] getImageData(BufferedImage image) {
        int realWidth = image.getWidth();
        byte[][][] buffer = new byte[config.height()][realWidth][CHANNELS];

        for (int w = 0; w < realWidth; ++w) {
            for (int h = 0; h < config.height(); ++h) {
                int hexcode = image.getRGB(w, h);
                int red = (hexcode >> 16) & 0xFF;
                int green = (hexcode >> 8) & 0xFF;
                int blue = hexcode & 0xFF;
                int white = Math.min(Math.min(red, green), blue);

                red -= white;
                green -= white;
                blue -= white;

                buffer[h][w][0] = (byte) (white & 0xFF);
                buffer[h][w][1] = (byte) (red & 0xFF);
                buffer[h][w][2] = (byte) (green & 0xFF);
                buffer[h][w][3] = (byte) (blue & 0xFF);
            }
        }
        return buffer;
    }

    private byte[][][] extractWrappedFrame(byte[][][] imageData, int startX) {
        int realWidth = imageData[0].length;
        byte[][][] frame = new byte[config.height()][config.width()][CHANNELS];

        for (int y = 0; y < config.height(); y++) {
            for (int x = 0; x < config.width(); x++) {
                int sourceX = (startX + x) % realWidth;
                copyPixel(imageData, frame, y, sourceX, x);
            }
        }

        return frame;
    }

    private byte[][][] extractClippedFrame(byte[][][] imageData, int startX) {
        int realWidth = imageData[0].length;
        byte[][][] frame = new byte[config.height()][config.width()][CHANNELS];

        for (int y = 0; y < config.height(); y++) {
            for (int x = 0; x < config.width(); x++) {
                int sourceX = startX + x;
                if (sourceX < realWidth) {
                    copyPixel(imageData, frame, y, sourceX, x);
                }
            }
        }

        return frame;
    }

    private void copyPixel(byte[][][] source, byte[][][] destination, int y, int sourceX, int destinationX) {
        destination[y][destinationX][0] = source[y][sourceX][0];
        destination[y][destinationX][1] = source[y][sourceX][1];
        destination[y][destinationX][2] = source[y][sourceX][2];
        destination[y][destinationX][3] = source[y][sourceX][3];
    }

    private PreparedPlaybackPackets reportPreparationFailure(String mode, String path, Exception exception) {
        DriverStation.reportError(
                "WLED " + mode + " preparation failed for " + path + ": " + exception.getMessage(),
                false);
        return new PreparedPlaybackPackets(new DatagramPacket[0], 0, new double[0]);
    }

    @FunctionalInterface
    private interface FrameBuilder {
        FrameData buildFrame(int frameIndex) throws IOException, BadImageFormatException;
    }

    private static class BadImageFormatException extends Exception {
        private static final long serialVersionUID = 1L;

        private BadImageFormatException(String message) {
            super(message);
        }
    }
}
