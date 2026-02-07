package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Kalman Filter for tracking target position (basket/goal) in field coordinates
 * Fuses AprilTag vision measurements with Pedro Pathing odometry
 *
 * State: [targetX, targetY, velocityX, velocityY]
 * Measurements: Vision (yaw, range) converted to Cartesian OR Odometry (goal position)
 */
@Config
public class KalmanFilter {
    // Tunable parameters (FTC Dashboard)
    public static double Q_POSITION = 0.1;      // cm^2 - process noise position
    public static double Q_VELOCITY = 0.5;      // (cm/s)^2 - process noise velocity
    public static double R_VISION_X = 25.0;     // cm^2 - vision measurement noise
    public static double R_VISION_Y = 25.0;
    public static double R_ODOMETRY_X = 100.0;  // cm^2 - odometry measurement noise
    public static double R_ODOMETRY_Y = 100.0;
    public static double MAHALANOBIS_THRESHOLD = 9.21;  // Chi-squared 99% confidence

    // State vector: [targetX, targetY, vx, vy]
    private double[] state;

    // Covariance matrix P (4x4)
    private double[][] P;

    // Timing
    private long lastUpdateTimeMs;

    // Debug
    private double[] lastInnovation;
    private int outlierCount;

    /**
     * Initialize filter with goal position
     */
    public KalmanFilter(double goalX, double goalY) {
        state = new double[]{goalX, goalY, 0, 0};

        // Initial covariance - high uncertainty
        P = new double[4][4];
        P[0][0] = 100.0;  // X position
        P[1][1] = 100.0;  // Y position
        P[2][2] = 10.0;   // X velocity
        P[3][3] = 10.0;   // Y velocity

        lastUpdateTimeMs = System.currentTimeMillis();
        lastInnovation = new double[2];
        outlierCount = 0;
    }

    /**
     * Prediction step - call every loop
     * @param currentTimeMs Current system time in milliseconds
     */
    public void predict(long currentTimeMs) {
        double dt = (currentTimeMs - lastUpdateTimeMs) / 1000.0;
        lastUpdateTimeMs = currentTimeMs;

        // Clamp dt to prevent large jumps
        dt = Math.max(0.001, Math.min(0.1, dt));

        // State transition matrix F (constant velocity model)
        double[][] F = {
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };

        // X_pred = F * X
        double[] X_pred = matrixVectorMultiply(F, state);

        // P_pred = F * P * F^T + Q
        double[][] FP = matrixMultiply4x4(F, P);
        double[][] FPFt = matrixMultiply4x4Transpose(FP, F);

        // Add process noise Q (diagonal matrix)
        FPFt[0][0] += Q_POSITION;
        FPFt[1][1] += Q_POSITION;
        FPFt[2][2] += Q_VELOCITY;
        FPFt[3][3] += Q_VELOCITY;

        state = X_pred;
        P = FPFt;
    }

    /**
     * Update with vision measurement (polar to Cartesian conversion)
     * @param robotX Robot X position (cm)
     * @param robotY Robot Y position (cm)
     * @param robotHeading Robot heading (radians)
     * @param visionYawDeg Vision yaw angle (degrees)
     * @param visionRangeCm Vision range (cm)
     * @return true if accepted, false if outlier rejected
     */
    public boolean updateVision(double robotX, double robotY, double robotHeading,
                                double visionYawDeg, double visionRangeCm) {
        // Sanity checks - reject unreasonable measurements
        if (visionRangeCm > 300.0 || Math.abs(visionYawDeg) > 80.0) {
            outlierCount++;
            return false;
        }

        // Convert polar (yaw, range) to Cartesian (targetX, targetY)
        double yawRad = Math.toRadians(visionYawDeg);
        double targetX = robotX + visionRangeCm * Math.cos(robotHeading + yawRad);
        double targetY = robotY + visionRangeCm * Math.sin(robotHeading + yawRad);

        double[] z = {targetX, targetY};

        // Observation matrix H (observe position only, not velocity)
        double[][] H = {
            {1, 0, 0, 0},
            {0, 1, 0, 0}
        };

        // Measurement noise R
        double[][] R = {
            {R_VISION_X, 0},
            {0, R_VISION_Y}
        };

        return updateStep(z, H, R);
    }

    /**
     * Update with odometry measurement (goal position fallback)
     * @param goalX Goal X position (cm)
     * @param goalY Goal Y position (cm)
     */
    public void updateOdometry(double goalX, double goalY) {
        double[] z = {goalX, goalY};

        double[][] H = {
            {1, 0, 0, 0},
            {0, 1, 0, 0}
        };

        double[][] R = {
            {R_ODOMETRY_X, 0},
            {0, R_ODOMETRY_Y}
        };

        updateStep(z, H, R);
    }

    /**
     * Common update step with outlier rejection (Mahalanobis distance)
     */
    private boolean updateStep(double[] z, double[][] H, double[][] R) {
        // Innovation: y = z - H * X
        double[] HX = matrixVectorMultiply(H, state);
        double[] innovation = {z[0] - HX[0], z[1] - HX[1]};

        // Innovation covariance: S = H * P * H^T + R
        double[][] HP = matrixMultiply2x4(H, P);
        double[][] HPHt = matrixMultiply2x4Transpose(HP, H);
        double[][] S = {
            {HPHt[0][0] + R[0][0], HPHt[0][1] + R[0][1]},
            {HPHt[1][0] + R[1][0], HPHt[1][1] + R[1][1]}
        };

        // Mahalanobis distance check (outlier rejection)
        double[][] S_inv = matrixInverse2x2(S);
        double d_squared = innovation[0] * (S_inv[0][0] * innovation[0] + S_inv[0][1] * innovation[1])
                         + innovation[1] * (S_inv[1][0] * innovation[0] + S_inv[1][1] * innovation[1]);

        if (d_squared > MAHALANOBIS_THRESHOLD) {
            outlierCount++;
            return false;  // Reject outlier
        }

        // Kalman gain: K = P * H^T * S^{-1}
        double[][] PHt = matrixTranspose4x2(P, H);  // 4x2
        double[][] K = matrixMultiply4x2(PHt, S_inv);  // 4x2 matrix

        // State update: X = X + K * innovation
        for (int i = 0; i < 4; i++) {
            state[i] += K[i][0] * innovation[0] + K[i][1] * innovation[1];
        }

        // Covariance update: P = (I - K * H) * P
        double[][] KH = matrixMultiply4x2_2x4(K, H);  // 4x4
        double[][] I_KH = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
            }
        }
        P = matrixMultiply4x4(I_KH, P);

        lastInnovation = innovation;
        return true;
    }

    /**
     * Get filtered target position
     * @return [targetX, targetY]
     */
    public double[] getEstimatedPosition() {
        return new double[]{state[0], state[1]};
    }

    /**
     * Get innovation (measurement residual) for diagnostics
     */
    public double[] getInnovation() {
        return lastInnovation;
    }

    /**
     * Get number of outliers rejected
     */
    public int getOutlierCount() {
        return outlierCount;
    }

    /**
     * Reset filter to new goal position
     */
    public void reset(double goalX, double goalY) {
        state[0] = goalX;
        state[1] = goalY;
        state[2] = 0;
        state[3] = 0;

        P = new double[4][4];
        P[0][0] = 100.0;
        P[1][1] = 100.0;
        P[2][2] = 10.0;
        P[3][3] = 10.0;

        outlierCount = 0;
        lastInnovation = new double[2];
    }

    // ========== MATRIX OPERATIONS ==========

    /**
     * 2x2 matrix inversion (for innovation covariance S)
     */
    private double[][] matrixInverse2x2(double[][] M) {
        double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        if (Math.abs(det) < 1e-10) {
            // Singular matrix - return identity (fallback)
            return new double[][]{{1, 0}, {0, 1}};
        }
        double invDet = 1.0 / det;
        return new double[][]{
            {M[1][1] * invDet, -M[0][1] * invDet},
            {-M[1][0] * invDet, M[0][0] * invDet}
        };
    }

    /**
     * 4x4 matrix multiplication: C = A * B
     */
    private double[][] matrixMultiply4x4(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    /**
     * 4x4 * 4x4^T: C = A * B^T
     */
    private double[][] matrixMultiply4x4Transpose(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[j][k];  // Note: B[j][k] instead of B[k][j]
                }
            }
        }
        return C;
    }

    /**
     * 2x4 matrix multiplication: C = A * B (A is 2x4, B is 4x4, result is 2x4)
     */
    private double[][] matrixMultiply2x4(double[][] A, double[][] B) {
        double[][] C = new double[2][4];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 4; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    /**
     * 2x4 * 2x4^T: C = A * B^T (result is 2x2)
     */
    private double[][] matrixMultiply2x4Transpose(double[][] A, double[][] B) {
        double[][] C = new double[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[j][k];  // Note: B[j][k] instead of B[k][j]
                }
            }
        }
        return C;
    }

    /**
     * 4x4 * 2x4^T: C = P * H^T (result is 4x2)
     */
    private double[][] matrixTranspose4x2(double[][] P, double[][] H) {
        double[][] C = new double[4][2];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    C[i][j] += P[i][k] * H[j][k];  // H^T means H[j][k]
                }
            }
        }
        return C;
    }

    /**
     * 4x2 * 2x2: C = A * B (result is 4x2)
     */
    private double[][] matrixMultiply4x2(double[][] A, double[][] B) {
        double[][] C = new double[4][2];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 2; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    /**
     * 4x2 * 2x4: C = K * H (result is 4x4)
     */
    private double[][] matrixMultiply4x2_2x4(double[][] K, double[][] H) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 2; k++) {
                    C[i][j] += K[i][k] * H[k][j];
                }
            }
        }
        return C;
    }

    /**
     * Matrix-vector multiplication: result = M * v
     */
    private double[] matrixVectorMultiply(double[][] M, double[] v) {
        int rows = M.length;
        int cols = M[0].length;
        double[] result = new double[rows];
        for (int i = 0; i < rows; i++) {
            result[i] = 0;
            for (int j = 0; j < cols; j++) {
                result[i] += M[i][j] * v[j];
            }
        }
        return result;
    }
}
