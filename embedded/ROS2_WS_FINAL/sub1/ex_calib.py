
import numpy as np
import cv2
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 2368,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.4+0.1,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0., # meter
    "Y": 0,
    "Z":  0.8,
    "YAW": 0, # deg
    "PITCH": 0.0,
    "ROLL": 0
}



def rotationMtx(yaw, pitch, roll):
    
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R


def translationMtx(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M


def transformMTX_lidar2cam(params_lidar, params_cam):
    '''
    transformMTX_lidar2cam 내 좌표 변환행렬 로직 순서
    1. params에서 라이다와 카메라 센서들의 자세, 위치 정보를 뽑기.
    2. 라이다에서 카메라 위치까지 변환하는 translation 행렬을 정의
    3. 카메라의 자세로 맞춰주는 rotation 행렬을 정의.
    4. 위의 두 행렬을 가지고 최종 라이다-카메라 변환 행렬을 정의.
    '''    
    lidar_yaw, lidar_pitch, lidar_roll = [np.deg2rad(params_lidar.get(i)) for i in (["YAW","PITCH","ROLL"])]
    cam_yaw, cam_pitch, cam_roll = [np.deg2rad(params_cam.get(i)) for i in (["YAW","PITCH","ROLL"])]
    
    #Relative position of lidar w.r.t cam
    lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
    cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

    x_rel = cam_pos[0] - lidar_pos[0]
    y_rel = cam_pos[1] - lidar_pos[1]
    z_rel = cam_pos[2] - lidar_pos[2]

    R_T = np.matmul(rotationMtx(lidar_yaw, lidar_pitch, lidar_roll).T, translationMtx(-x_rel, -y_rel, -z_rel).T)
    R_T = np.matmul(R_T, rotationMtx(cam_yaw, cam_pitch, cam_roll))
    R_T = np.matmul(R_T, rotationMtx(np.deg2rad(-90.), 0., 0.))
    R_T = np.matmul(R_T, rotationMtx(0, 0., np.deg2rad(-90.)))
    
    #rotate and translate the coordinate of a lidar
    R_T = R_T.T 
    
    print('r : \n')

    print(R_T[:3,:3])

    print('t : \n')

    print(R_T[:3,3])

    return R_T

def project2img_mtx(params_cam):
    
    """
    project2img_mtx 내 projection 행렬 계산 로직 순서
    1. params에서 카메라의 width, height, fov를 가져와서 focal length를 계산.
    2. 카메라의 파라메터로 이미지 프레임 센터를 계산.
    3. Projection 행렬을 계산 

    """
    # focal lengths
    fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    #the center of image
    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2
    
    #transformation matrix from 3D to 2D
    R_f = np.array([[fc_x,  0,      cx],
                    [0,     fc_y,   cy]])

    return R_f    


def draw_pts_img(img, xi, yi):
    '''
    place the lidar points into numpy arrays in order to make intensity map
    \n img : source image
    \n xi, yi : point pixel 
    '''
    point_np = img

    #Left Lane
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)
    return point_np

class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):

        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):
        
        xyz_c = np.matmul(np.concatenate([xyz_p, np.ones((xyz_p.shape[0], 1))], axis=1), self.RT.T)

        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

        xyz_c = xyz_c.T

        xc, yc, zc = xyz_c[0,:].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))

        xyi = xyi[0:2,:].T

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass
        
        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi

    



class SensorCalib(Node):

    def __init__(self):
        super().__init__(node_name='ex_calib')

        self.subs_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback, 10)

        self.subs_img = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.xyz, self.R, self.intens = None, None, None
        self.img = None

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):

        self.R = np.array(msg.ranges)
        self.intens = np.array(msg.intensities)

        x = self.R*np.cos(np.linspace(0, 2*np.pi, 360))
        y = self.R*np.sin(np.linspace(0, 2*np.pi, 360))
        z = np.zeros_like(x)

        self.xyz = np.concatenate([
            x.reshape([-1, 1]),
            y.reshape([-1, 1]),
            z.reshape([-1, 1])
        ], axis=1)

    def timer_callback(self):

        if self.xyz is not None and self.img is not None :
            
            xyz_p = self.xyz[np.where(self.xyz[:, 0]>=0)]

            print(xyz_p)

            intens_p = self.intens.reshape([-1,1])
            intens_p = intens_p[np.where(self.xyz[:, 0]>=0)]

            xyz_c = self.l2c_trans.transform_lidar2cam(xyz_p)

            xy_i = self.l2c_trans.project_pts2img(xyz_c, crop=True)

            img_l2c = draw_pts_img(self.img, xy_i[:, 0].astype(np.int32),
                                            xy_i[:, 1].astype(np.int32))
                                                
            cv2.imshow("Lidar2Cam", img_l2c)
            cv2.waitKey(1)

        else:
            pass


def main(args=None):

    rclpy.init(args=args)

    calibrator = SensorCalib()

    rclpy.spin(calibrator)


if __name__ == '__main__':

    main()