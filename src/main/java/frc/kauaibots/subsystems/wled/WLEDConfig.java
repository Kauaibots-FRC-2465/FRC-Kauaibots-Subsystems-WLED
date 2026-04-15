package frc.kauaibots.subsystems.wled;

import java.util.Objects;

public record WLEDConfig(
        String controllerAddress,
        int ddpPort,
        int width,
        int height,
        int rowsPerPacket) {
    public WLEDConfig {
        if (Objects.requireNonNull(controllerAddress, "controllerAddress").isBlank()) {
            throw new IllegalArgumentException("controllerAddress must not be blank.");
        }
        if (ddpPort <= 0 || ddpPort > 65535) {
            throw new IllegalArgumentException("ddpPort must be between 1 and 65535.");
        }
        if (width <= 0) {
            throw new IllegalArgumentException("width must be positive.");
        }
        if (height <= 0) {
            throw new IllegalArgumentException("height must be positive.");
        }
        if (rowsPerPacket <= 0) {
            throw new IllegalArgumentException("rowsPerPacket must be positive.");
        }
    }
}
