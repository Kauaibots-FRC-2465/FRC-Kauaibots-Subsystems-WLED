package frc.kauaibots.subsystems.wled;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.UnknownHostException;

import javax.imageio.ImageIO;

import com.madgag.gif.fmsware.GifDecoder;

import edu.wpi.first.wpilibj.DriverStation;

final class AnimationSupport {
    private static final int CHANNELS = 4;

    private final WLEDConfig config;
    private InetAddress cachedControllerAddress;

    AnimationSupport(WLEDConfig config) {
        this.config = config;
        this.cachedControllerAddress = null;
    }

    AnimationUDPs buildPlaybackPackets(int frameCount, FrameBuilder frameBuilder)
            throws IOException, BadImageFormatException {
        if (frameCount <= 0) {
            return emptyPlayback();
        }

        FrameData firstFrameData = frameBuilder.buildFrame(0);
        DatagramPacket[] firstFramePackets = buildPacketsForFrame(firstFrameData.pixels());
        int packetsPerFrame = firstFramePackets.length;
        DatagramPacket[] packets = new DatagramPacket[frameCount * packetsPerFrame];
        double[] frameDisplayDurationsSeconds = new double[frameCount];
        System.arraycopy(firstFramePackets, 0, packets, 0, packetsPerFrame);
        frameDisplayDurationsSeconds[0] = firstFrameData.displayDurationSeconds();

        int packetIndex = packetsPerFrame;
        for (int frameIndex = 1; frameIndex < frameCount; frameIndex++) {
            FrameData frameData = frameBuilder.buildFrame(frameIndex);
            DatagramPacket[] framePackets = buildPacketsForFrame(frameData.pixels());
            System.arraycopy(framePackets, 0, packets, packetIndex, packetsPerFrame);
            frameDisplayDurationsSeconds[frameIndex] = frameData.displayDurationSeconds();
            packetIndex += packetsPerFrame;
        }

        return new AnimationUDPs(packets, packetsPerFrame, frameDisplayDurationsSeconds);
    }

    AnimationUDPs buildSingleFramePlayback(byte[][][] pixels)
            throws UnknownHostException {
        DatagramPacket[] packets = buildPacketsForFrame(pixels);
        return new AnimationUDPs(packets, packets.length, new double[] {0.0});
    }

    AnimationUDPs emptyPlayback() {
        return new AnimationUDPs(new DatagramPacket[0], 0, new double[0]);
    }

    AnimationUDPs reportAnimationFailure(String mode, String path, Exception exception) {
        DriverStation.reportError(
                "WLED " + mode + " animation failed for " + path + ": " + exception.getMessage(),
                false);
        return emptyPlayback();
    }

    BufferedImage readImage(String path) throws IOException, BadImageFormatException {
        BufferedImage image = ImageIO.read(new File(path));
        if (image == null) {
            throw new BadImageFormatException("Unsupported image format or file not found.");
        }
        if (image.getHeight() < config.height()) {
            throw new BadImageFormatException("Image height must be at least " + config.height() + " pixels.");
        }
        return image;
    }

    FrameData buildGifFrame(GifDecoder gifDecoder, int frameIndex) throws BadImageFormatException {
        BufferedImage frame = gifDecoder.getFrame(frameIndex);
        validateGifDimensions(frame, "GIF frame " + frameIndex);
        return new FrameData(getImageData(frame), gifDecoder.getDelay(frameIndex) / 1000.0);
    }

    void validateGifDimensions(BufferedImage frame, String subject) throws BadImageFormatException {
        if (frame == null) {
            throw new BadImageFormatException(subject + " is unavailable.");
        }
        if (frame.getWidth() != config.width() || frame.getHeight() != config.height()) {
            throw new BadImageFormatException(
                    subject + " must be exactly " + config.width() + "x" + config.height() + " pixels.");
        }
    }

    byte[][][] getImageData(BufferedImage image) {
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

    byte[][][] extractWrappedFrame(byte[][][] imageData, int startX) {
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

    byte[][][] extractClippedFrame(byte[][][] imageData, int startX) {
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

    private DatagramPacket[] buildPacketsForFrame(byte[][][] matrix) throws UnknownHostException {
        int packetsPerFrame = (matrix.length + config.rowsPerPacket() - 1) / config.rowsPerPacket();
        DatagramPacket[] packets = new DatagramPacket[packetsPerFrame];
        InetAddress address = getControllerAddress();

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

    private InetAddress getControllerAddress() throws UnknownHostException {
        if (cachedControllerAddress == null) {
            cachedControllerAddress = InetAddress.getByName(config.controllerAddress());
        }
        return cachedControllerAddress;
    }

    private void copyPixel(byte[][][] source, byte[][][] destination, int y, int sourceX, int destinationX) {
        destination[y][destinationX][0] = source[y][sourceX][0];
        destination[y][destinationX][1] = source[y][sourceX][1];
        destination[y][destinationX][2] = source[y][sourceX][2];
        destination[y][destinationX][3] = source[y][sourceX][3];
    }

    record FrameData(byte[][][] pixels, double displayDurationSeconds) {}

    @FunctionalInterface
    interface FrameBuilder {
        FrameData buildFrame(int frameIndex) throws IOException, BadImageFormatException;
    }
}
