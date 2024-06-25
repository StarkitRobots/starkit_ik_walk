import numpy as np

class AnglesConverter:

    def fusedYawOfRotmat(self, R: np.ndarray) -> float:
        fusedYaw = 0.0
        trace = R[0][0] + R[1][1] + R[2][2]

        if trace >= 0.0:
            fusedYaw = 2.0 * np.arctan2(R[1][0] - R[0][1], 1.0 + trace)

        elif R[2][2] >= R[1][1] and R[2][2] >= R[0][0]:
            fusedYaw = 2.0 * np.arctan2(1.0 - R[0][0] - R[1][1] + R[2][2], R[1][0] - R[0][1])

        elif R[1][1] >= R[0][0]:
            fusedYaw = 2.0 * np.arctan2(R[2][1] + R[1][2], R[0][2] - R[2][0])

        else:
            fusedYaw = 2.0 * np.arctan2(R[0][2] + R[2][0], R[2][1] - R[1][2])

        if fusedYaw > np.pi:    fusedYaw -= 2.0 * np.pi
        if fusedYaw <= -np.pi:  fusedYaw += 2.0 * np.pi

        return fusedYaw 

    def euler2rotmat(self, yaw: float, pitch: float, roll: float) -> np.ndarray:

        # calculate trigonometry once and use variables then
        # c - cos, s - sin
        # psi - psi = yaw, th - theta = pitch, phi - phi = roll
        # https://github.com/AIS-Bonn/rot_conv_lib/tree/master

        cpsi = np.cos(yaw)
        spsi = np.sin(yaw)

        cth = np.cos(pitch)
        sth = np.sin(pitch)

        cphi = np.cos(roll)
        sphi = np.sin(roll)

        return np.array(
            [
                [cpsi * cth, cpsi * sth * sphi - spsi * cphi, cpsi * sth * cphi + spsi * sphi],
                [spsi * cth, spsi * sth * sphi + cpsi * cphi, spsi * sth * cphi - cpsi * sphi],
                [      -sth,                      cth * sphi,                      cth * cphi],
            ]
        )

    def euler2fused(self, yaw: float, pitch: float, roll: float) -> tuple:
        R = self.euler2rotmat(yaw, pitch, roll)

        fusedYaw = self.fusedYawOfRotmat(R)
        fusedPitch = pitch
        fusedRoll = np.arcsin(np.clip(R[2][1], -1.0, 1.0))
        fusedHemi = int(R[2][2] >= 0.0)

        return fusedYaw, fusedPitch, fusedRoll, fusedHemi