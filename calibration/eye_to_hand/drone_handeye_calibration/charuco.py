"""Shared ChArUco board detection and pose-estimation helpers."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraModel:
    matrix: np.ndarray
    distortion: np.ndarray
    width: int
    height: int
    frame_id: str


@dataclass
class Detection:
    marker_ids: list[int]
    corner_count: int
    coverage: float
    reprojection_rms_px: float | None
    rvec: np.ndarray | None
    tvec: np.ndarray | None
    annotated: np.ndarray


def make_board(
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
) -> tuple[cv2.aruco.CharucoBoard, cv2.aruco.CharucoDetector]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length_m,
        marker_length_m,
        dictionary,
    )
    detector_parameters = cv2.aruco.DetectorParameters()
    detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(
        board,
        cv2.aruco.CharucoParameters(),
        detector_parameters,
    )
    return board, detector


def detect_board(
    image_bgr: np.ndarray,
    board: cv2.aruco.CharucoBoard,
    detector: cv2.aruco.CharucoDetector,
    camera: CameraModel | None,
) -> Detection:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    annotated = image_bgr.copy()
    marker_id_list: list[int] = []
    if marker_ids is not None and len(marker_ids):
        marker_id_list = [int(value) for value in marker_ids.reshape(-1)]
        cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)

    corner_count = 0 if charuco_ids is None else int(len(charuco_ids))
    coverage = 0.0
    reprojection_rms_px: float | None = None
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None
    if corner_count:
        cv2.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids)
        points = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
        if len(points) >= 3:
            coverage = float(cv2.contourArea(cv2.convexHull(points))) / float(
                image_bgr.shape[0] * image_bgr.shape[1]
            )

    can_solve_pose = (
        camera is not None
        and corner_count >= 4
        and not board.checkCharucoCornersCollinear(charuco_ids)
    )
    if can_solve_pose:
        object_points, image_points = board.matchImagePoints(charuco_corners, charuco_ids)
        success, solved_rvec, solved_tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera.matrix,
            camera.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if success:
            rvec = solved_rvec.reshape(3)
            tvec = solved_tvec.reshape(3)
            projected, _ = cv2.projectPoints(
                object_points,
                solved_rvec,
                solved_tvec,
                camera.matrix,
                camera.distortion,
            )
            residual = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
            reprojection_rms_px = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
            cv2.drawFrameAxes(
                annotated,
                camera.matrix,
                camera.distortion,
                solved_rvec,
                solved_tvec,
                0.05,
                2,
            )

    label = (
        f"markers={len(marker_id_list)} corners={corner_count} "
        f"coverage={coverage * 100:.1f}%"
    )
    if reprojection_rms_px is not None:
        label += f" reproj={reprojection_rms_px:.2f}px"
    cv2.rectangle(
        annotated,
        (8, 8),
        (min(annotated.shape[1] - 8, 720), 42),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        annotated,
        label,
        (16, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return Detection(
        marker_ids=marker_id_list,
        corner_count=corner_count,
        coverage=coverage,
        reprojection_rms_px=reprojection_rms_px,
        rvec=rvec,
        tvec=tvec,
        annotated=annotated,
    )


def decode_color(message) -> np.ndarray:
    encoding = message.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported color encoding: {message.encoding}")
    packed = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height,
        message.step,
    )
    image = packed[:, : message.width * channels].reshape(
        message.height,
        message.width,
        channels,
    )
    conversions = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
    }
    conversion = conversions.get(encoding)
    return image.copy() if conversion is None else cv2.cvtColor(image, conversion)


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Return an xyzw quaternion for a proper 3x3 rotation matrix."""
    matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    quaternion = np.empty(4, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion[3] = 0.25 * scale
        quaternion[0] = (matrix[2, 1] - matrix[1, 2]) / scale
        quaternion[1] = (matrix[0, 2] - matrix[2, 0]) / scale
        quaternion[2] = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            quaternion[3] = (matrix[2, 1] - matrix[1, 2]) / scale
            quaternion[0] = 0.25 * scale
            quaternion[1] = (matrix[0, 1] + matrix[1, 0]) / scale
            quaternion[2] = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            quaternion[3] = (matrix[0, 2] - matrix[2, 0]) / scale
            quaternion[0] = (matrix[0, 1] + matrix[1, 0]) / scale
            quaternion[1] = 0.25 * scale
            quaternion[2] = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            quaternion[3] = (matrix[1, 0] - matrix[0, 1]) / scale
            quaternion[0] = (matrix[0, 2] + matrix[2, 0]) / scale
            quaternion[1] = (matrix[1, 2] + matrix[2, 1]) / scale
            quaternion[2] = 0.25 * scale
    return quaternion / np.linalg.norm(quaternion)
