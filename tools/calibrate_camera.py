import numpy as np
import cv2
import glob
import os
import argparse
import PyKDL
import math
import random
from termcolor import cprint
import sys
import threading
import time


class Spinner(object):
    def __init__(self, sleep_time=0.1, auto_start=True):
        self.sleep_time = sleep_time
        self.thread = threading.Thread(target=self.loop)
        self.active = False
        if auto_start:
            self.start()

    def loop(self):
        spinner = self.spinning_cursor()
        while self.active:
            sys.stdout.write(spinner.next())
            sys.stdout.flush()
            time.sleep(self.sleep_time)
            sys.stdout.write('\b')

    def start(self):
        self.active = True
        self.thread.start()

    def stop(self):
        self.active = False
        self.thread.join()

    def spinning_cursor(self):
        while True:
            for cursor in '|/-\\':
                yield cursor


def printArrow():
    print("||\n||\n||\n\/")


def KDLToMat(frame):
    mat = np.eye(4, dtype=float)
    for i in range(0, 3):
        for j in range(0, 3):
            mat[i, j] = frame.M[i, j]
    mat[0, 3] = frame.p.x()
    mat[1, 3] = frame.p.y()
    mat[2, 3] = frame.p.z()
    return mat


def MatToKDL(mat):
    frame = PyKDL.Frame()
    for i in range(0, 3):
        for j in range(0, 3):
            frame.M[i, j] = mat[i, j]
    frame.p = PyKDL.Vector(
        mat[0, 3],
        mat[1, 3],
        mat[2, 3]
    )
    return frame


def KDLToCv(frame):
    rot = np.array([
        [frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
        [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
        [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]
    ]
    )
    Rvec, _ = cv2.Rodrigues(rot)
    Tvec = np.array([frame.p.x(), frame.p.y(), frame.p.z()])

    return Rvec, Tvec


def KDLFromArray(chunks, fmt='XYZQ'):
    if fmt == 'XYZQ':
        frame = PyKDL.Frame()
        frame.p = PyKDL.Vector(
            chunks[0], chunks[1], chunks[2]
        )
        q = np.array([chunks[3],
                      chunks[4],
                      chunks[5],
                      chunks[6]])
        q = q / np.linalg.norm(q)
        frame.M = PyKDL.Rotation.Quaternion(q[0], q[1], q[2], q[3])
    return frame


def singleRowPrint(reference_image, camera_matrix, camera_distortions):
    data = [
        reference_image.shape[1],
        reference_image.shape[0],
        camera_matrix[0, 0],
        camera_matrix[1, 1],
        camera_matrix[0, 2],
        camera_matrix[1, 2]
    ]
    data.extend(camera_distortions.ravel().tolist())
    return np.array(data)


def optimizationFunction(x, robot_poses, pattern_poses, offset_factor=[0.0, 0.0, 0.0]):
    e = 0.0

    T_find = KDLFromArray(x)
    T_k = PyKDL.Frame(PyKDL.Vector(
        offset_factor[0], offset_factor[1], offset_factor[2]
    ))

    for i in range(0, len(robot_poses)):
        for j in range(0, len(robot_poses)):
            if i != j:
                T1_i = robot_poses[i]
                T2_i = pattern_poses[i]
                T1_j = robot_poses[j]
                T2_j = pattern_poses[j]
                Tr_i = ((T1_i * T_find) * T2_i) * T_k
                Tr_j = ((T1_j * T_find) * T2_j) * T_k
                pr_i = Tr_i.p
                pr_j = Tr_j.p
                # rot_i = Tr_i[0:3,0:3]
                # rot_j = Tr_j[0:3,0:3]
                # qa = rotm2quat(rot_i)';
                # qd = rotm2quat(rot_j)';
                diff = pr_j - pr_i
                e = e + diff.Norm()**2
    return e


def optimizationRefinementFunction(x, robot_poses, pattern_poses, offset_factor=[0.0, 0.0, 0.0]):
    e = 0.0

    initial_guess = KDLFromArray(x)
    measurements = []
    for i, robot_pose in enumerate(robot_poses):
        camera_pose = robot_pose * initial_guess
        chessboard_pose = pattern_poses[i]
        chessboard_pose = camera_pose * chessboard_pose

        measurements.append(np.append(
            np.array([
                chessboard_pose.p.x(),
                chessboard_pose.p.y(),
                chessboard_pose.p.z()
            ]), np.asarray(
                chessboard_pose.M.GetRPY()) * 180.0 / math.pi))
    measurements = np.array(measurements)

    vx = np.var(measurements[:, 0])
    vy = np.var(measurements[:, 1])
    vz = np.var(measurements[:, 2])
    vroll = np.var(measurements[:, 3])
    vpitch = np.var(measurements[:, 4])
    vyaw = np.var(measurements[:, 5])
    return vx + vy + vz + vroll + vpitch + vyaw


def bundleAdjustment(x, mean_chessboard_bose, chessboard_calibrator_undistorted):

    T_find = KDLFromArray(x)
    e = 0.0
    for index in range(0, len(chessboard_calibrator_undistorted.images)):
        img = chessboard_calibrator_undistorted.images[index]
        img_pts = chessboard_calibrator_undistorted.chessboard_image_points[index]

        num = chessboard_calibrator_undistorted.w * chessboard_calibrator_undistorted.h

        cv2.drawChessboardCorners(img, (chessboard_calibrator_undistorted.w,
                                        chessboard_calibrator_undistorted.h), img_pts, True)

        # print("IMAGE_POINTS")
        # print img_pts
        # cv2.imshsow("img", img)

        camera_pose = chessboard_calibrator_undistorted.robot_poses[index] * T_find
        Rvec, Tvec = KDLToCv(camera_pose.Inverse())
        world_points = chessboard_calibrator_undistorted.chessboard_world_points[index]

        world_points = np.hstack((world_points, np.ones((num, 1))))

        world_points = np.matmul(mean_chessboard_bose,
                                 world_points.T).astype(np.float32)
        # print("WORLD POINTS")
        world_points = world_points[:3, :].T
        # print(world_points[:, 0:5], world_points.shape, world_points.dtype)

        proj_points, _ = cv2.projectPoints(world_points, Rvec, Tvec, chessboard_calibrator_undistorted.camera_matrix,
                                           chessboard_calibrator_undistorted.camera_distortions)

        proj_points = np.array(proj_points)
        # print("COMPUTED IMAGE_POINTS")
        # print(proj_points, proj_points.shape)
        img2 = img.copy()
        cv2.drawChessboardCorners(img2, (chessboard_calibrator_undistorted.w,
                                         chessboard_calibrator_undistorted.h), proj_points, True)

        diff = np.linalg.norm(np.square(img_pts - proj_points))
        # print("DIFFERENCE", diff)
        # cv2.imshow("img2", img2)
        # cv2.waitKey(0)
        e += diff
    # print("E:", e)
    return e


class ChessboardCalibrator(object):

    def __init__(self, w, h, square_size_meter):
        self.w = w
        self.h = h
        self.square_size_meter = square_size_meter
        self.chessboard_points = np.zeros((w * h, 3), np.float32)
        self.chessboard_points[:, :2] = np.mgrid[0:w,
                                                 0:h].T.reshape(-1, 2) * square_size_meter

        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.kernel_size = 11

        self.images = []
        self.gray_images = []
        self.undistored_images = []
        self.chessboard_world_points = []
        self.chessboard_image_points = []
        self.camera_matrix = np.eye(3, dtype=float)
        self.camera_matrix_refined = np.eye(3, dtype=float)
        self.camera_distortions = np.zeros((5,), dtype=float)
        self.chessboard_poses = []
        self.robot_poses = []

    def consumeImage(self, img, color_image=True):
        if color_image:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        ret, corners = cv2.findChessboardCorners(gray, (self.w, self.h), None)
        if ret:
            self.images.append(img.copy())
            self.gray_images.append(gray.copy())
            self.chessboard_world_points.append(self.chessboard_points)
            corners2 = cv2.cornerSubPix(
                gray, corners, (self.kernel_size, self.kernel_size), (-1, -1), self.criteria)
            self.chessboard_image_points.append(corners2)
            return corners2
        return None

    def calibrate(self, undistortion_alpha=1.0):
        if len(self.images) > 0:
            ret, self.camera_matrix, self.camera_distortions, rvecs, tvecs = cv2.calibrateCamera(
                self.chessboard_world_points,
                self.chessboard_image_points,
                self.gray_images[0].shape[::-1],
                None, None
            )

            self.chessboard_poses = []
            for i, t in enumerate(tvecs):
                rvec = rvecs[i]
                R, _ = cv2.Rodrigues(rvec)
                T = np.hstack((R, t))
                T = np.vstack((T, np.array([0, 0, 0, 1.0])))
                self.chessboard_poses.append(MatToKDL(T))

            self.refineCameraMatrix(alpha=undistortion_alpha)
            self.buildUndistortedImages()
            return True
        return False

    def refineCameraMatrix(self, alpha=1):
        if len(self.images) > 0:
            img = self.images[0]
            h,  w = img.shape[:2]
            self.camera_matrix_refined, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix,
                self.camera_distortions,
                (w, h), alpha, (w, h)
            )

    def buildUndistortedImages(self):
        for img in self.images:
            dst = cv2.undistort(img,
                                self.camera_matrix,
                                self.camera_distortions,
                                None,
                                self.camera_matrix_refined
                                )
            self.undistored_images.append(dst)

    def saveUndistortedImages(self, output_folder, extension="png", zero_fill=5):
        for i, img in enumerate(chessboard_calibrator.undistored_images):
            output_file = os.path.join(output_folder, str(
                i).zfill(zero_fill) + "." + extension)
            cv2.imwrite(output_file, img)

    def setRobotPoses(self, poses):
        if len(poses) != len(self.images):
            print("Error! Number of Robot Poses is different from the number of images")
            return
        self.robot_poses = poses


# frame = KDLFromArray(np.array([0.148919685995928,	0.00695950033291103,	-0.186177553121779,
#                                0.700711784018802,	-0.713247286081620,	-0.0117472159865630,	0.0119711131878521]))
# print np.asarray(frame.M.GetRPY()) * 180.0 / math.pi

#######################################
# Arguments
#######################################
ap = argparse.ArgumentParser()
ap.add_argument("--image_folder", required=True, help="Image Folder")
ap.add_argument("--image_extension", default='png', type=str)
ap.add_argument("--output_folder", default='', type=str)
ap.add_argument("--chessboard_size", default='6x9', type=str)
ap.add_argument("--chessboard_square_size", default=0.0261, type=float)
ap.add_argument("--save_undistorted", action='store_true')
ap.add_argument("--compute_extrinsics", action='store_true')
ap.add_argument("--undistortion_alpha", default=1.0, type=float)
ap.add_argument('--initial_guess', nargs='*', default=[0.0, 0.0, -0.09, 0.707388044876899, -0.7068249569936026,
                                                       0.0005628637715330792, 0.0005633121735972125], help="'x y z a b c d'")
ap.add_argument('--translation_bounds', nargs='*',
                default=[-0.3, 0.3, -0.3, 0.3, -0.3, 0.3])

ap.add_argument('--offset_factor', nargs='*',
                default=[0.0, 0.0, 0.0])
ap.add_argument('--max_iterations', default=10000000, type=int)
ap.add_argument('--optimization_debug', default=0, type=int)


args = vars(ap.parse_args())

#######################################
# Creates Calibrator
#######################################
w, h = map(int, args['chessboard_size'].split('x'))
sqs = args['chessboard_square_size']
chessboard_calibrator = ChessboardCalibrator(w, h, sqs)

#######################################
# Load Images Files
#######################################
images_files = sorted(
    glob.glob(
        os.path.join(args['image_folder'], '*.' + args['image_extension'])
    )
)


#######################################
# Feed with images
#######################################
discarded = []
for i, fname in enumerate(images_files):
    img = cv2.imread(fname)
    if chessboard_calibrator.consumeImage(img) is None:
        discarded.append(i)

#######################################
# Images check
#######################################
if len(chessboard_calibrator.images) == 0:
    cprint("No images found!")
    sys.exit(0)

#######################################
# Calibrate
#######################################
cprint("{}\n Calibration...\n {}".format("=" * 50, "=" * 50), color="yellow")
sp = Spinner()
chessboard_calibrator.calibrate(
    undistortion_alpha=args['undistortion_alpha']
)
sp.stop()

printArrow()

#######################################
# Output options
#######################################
np.set_printoptions(precision=6, suppress=True, linewidth=np.inf)


reference_image = chessboard_calibrator.images[0]
first_color = "red"
separator_width = 50
cprint("First Calibration {}".format("=" * separator_width), first_color)

cprint("\nCamera Matrix", first_color)
print(chessboard_calibrator.camera_matrix)

cprint("\nCamera Distortions", first_color)
print(chessboard_calibrator.camera_distortions)

cprint("\nSingle row", first_color)
print(np.array2string(singleRowPrint(reference_image, chessboard_calibrator.camera_matrix,
                                     chessboard_calibrator.camera_distortions), separator=","))

printArrow()

first_color = "green"
cprint("\nRefined Calibration {}".format("=" * separator_width), first_color)

cprint("\nCamera Matrix", first_color)
print(chessboard_calibrator.camera_matrix_refined)

cprint("\nCamera Distortions", first_color)
print(chessboard_calibrator.camera_distortions)

cprint("\nSingle row", first_color)
print(np.array2string(singleRowPrint(reference_image, chessboard_calibrator.camera_matrix_refined,
                                     chessboard_calibrator.camera_distortions), separator=","))

printArrow()

# print(chessboard_calibrator.chessboard_poses[-1])


# img1 = chessboard_calibrator.gray_images[10]
# img2 = cv2.cvtColor(
#     chessboard_calibrator.undistored_images[10], cv2.COLOR_BGR2GRAY)
# diff = img1 - img2
# whole = np.hstack((img1, img2, diff))
# cv2.imshow("und", whole)

# cv2.waitKey(0)


output_folder = args['output_folder']
if args['save_undistorted'] and len(output_folder) > 0:
    chessboard_calibrator.saveUndistortedImages(
        output_folder,
        extension=args['image_extension']
    )


if args['compute_extrinsics']:
    #######################################
    # Load Robot Poses
    #######################################
    poses_files = sorted(
        glob.glob(
            os.path.join(args['image_folder'], '*.txt')
        )
    )

    robot_poses = []
    for i, pose_file in enumerate(poses_files):
        if i not in discarded:
            pose_data = np.loadtxt(pose_file)
            robot_poses.append(KDLFromArray(pose_data))

    #######################################
    # Creates Undistorted Chessboard Calibrator
    #######################################
    chessboard_calibrator_undistorted = ChessboardCalibrator(w, h, sqs)
    for uimg in chessboard_calibrator.undistored_images:
        chessboard_calibrator_undistorted.consumeImage(uimg)

    cprint("{}\n Compute Chessboard Poses...\n {}".format(
        "=" * 50, "=" * 50), color="yellow")
    sp = Spinner()
    chessboard_calibrator_undistorted.calibrate()
    sp.stop()

    #######################################
    # Set Robot Poses
    #######################################
    chessboard_calibrator.setRobotPoses(robot_poses)
    chessboard_calibrator_undistorted.setRobotPoses(robot_poses)

    # initial_guess = np.array(
    #     #[0.138465, -0.001308, -0.164206, -1.676834, 1.721383, 0.024899, -0.053322]
    #     #[0.141472,-0.003309,-0.167088,-1.676989, 1.721503, 0.032095,-0.054912]
    #     [0.141472, -0.003309, -0.167088, -1.676989, 1.721503, 0.032095, -0.054912]
    # )

    initial_guess = np.array(args['initial_guess'][0].split(" "), dtype=float)

    initial_guess_frame = KDLFromArray(initial_guess)
    print("INITIAL GUESS", np.asarray(
        initial_guess_frame.M.GetRPY()) * 180.0 / math.pi)

    print("VERIFICATION")
    measurements = []
    chessboard_poses = []
    camera_poses = []
    row_sum = np.zeros((7,), dtype=np.float32)
    for i, robot_pose in enumerate(chessboard_calibrator_undistorted.robot_poses):
        camera_pose = robot_pose * initial_guess_frame
        camera_poses.append(camera_pose)
        chessboard_pose = chessboard_calibrator_undistorted.chessboard_poses[i]
        chessboard_pose = camera_pose * chessboard_pose
        chessboard_poses.append(KDLToMat(chessboard_pose))

        # print("Pose", i, chessboard_pose.p, np.asarray(
        #       chessboard_pose.M.GetRPY())*180.0/math.pi)

        row = np.append(
            np.array([
                chessboard_pose.p.x(),
                chessboard_pose.p.y(),
                chessboard_pose.p.z()
            ]), np.asarray(
                chessboard_pose.M.GetQuaternion()))
        row_sum += row
        measurements.append(row)

    measurements = np.array(measurements)

    row_sum = row_sum / measurements.shape[0]
    mean_chessboard_bose = KDLToMat(KDLFromArray(row_sum))
    print("MEasurements")
    print(measurements)
    print("MEan")
    print(row_sum)

    cv2.imshow("img", img)
    cv2.waitKey(0)
    bundleAdjustment(initial_guess, mean_chessboard_bose,
                     chessboard_calibrator_undistorted)

    # sys.exit(0)

    from scipy.optimize import minimize
    x0 = initial_guess  # map(float, np.array(args['initial_guess']))
    translation_bounds = map(float, args['translation_bounds'])
    offset_factor = map(float, args['offset_factor'])

    bounds = [
        (translation_bounds[0], translation_bounds[1]),
        (translation_bounds[2], translation_bounds[3]),
        (translation_bounds[4], translation_bounds[5]),
        (-np.inf, np.inf),
        (-np.inf, np.inf),
        (-np.inf, np.inf),
        (-np.inf, np.inf)
    ]

    def cb(x):
        print("Minimization", x)

    cprint("{}\n Exstrinsics Computation...\n {}".format(
        "=" * 50, "=" * 50), color="yellow")

    cprint("Initial Guess: {}".format(x0), 'blue')
    cprint("Bounds: {}".format(bounds), 'blue')
    cprint("Offset Factor: {}".format(offset_factor), 'blue')
    sp = Spinner()
    # res = minimize(bundleAdjustment, x0,
    #                args=(mean_chessboard_bose,
    #                      chessboard_calibrator_undistorted),
    #                method='L-BFGS-B', bounds=bounds, options={'maxiter': args['max_iterations'], 'disp': args['optimization_debug']})

    res = minimize(optimizationFunction, x0,
                   args=(chessboard_calibrator_undistorted.robot_poses,
                         chessboard_calibrator_undistorted.chessboard_poses,
                         offset_factor),
                   method='L-BFGS-B', bounds=bounds, options={'maxiter': args['max_iterations'], 'disp': args['optimization_debug']})

    sp.stop()

    first_color = "blue"
    cprint("\nExtrinsics {}".format("=" * separator_width), first_color)
    cprint("\nCamera Frame: [X,Y,Z,QX,QY,QZ,QW]", first_color)
    print(np.array2string(res.x, separator=","))

    cprint("\nCamera Frame: KDL", first_color)
    frame = KDLFromArray(res.x)
    print(frame)

    cprint("\nCamera Frame orientation: [Roll Pitch Yaw] deg", first_color)
    print(np.asarray(frame.M.GetRPY()) * 180 / math.pi)
