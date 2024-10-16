#!/home/zju/anaconda3/envs/yolov8D3/bin/python

from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import find_and_connect, confirm_capture_3d

from ultralytics import YOLO

import numpy as np
import cv2


class Mapping2DImageToDepthMap(object):
    def __init__(self, point_cloud_file: str="UntexturedPointCloud.pcd"):
        self.camera = Camera()
        self.point_cloud_file = point_cloud_file

    def contains(self, row, col, roi):
        # return roi[1] <= row <= roi[1] + roi[3] and roi[0] <= col <= roi[0] + roi[2]
        return roi[1] <= row <= roi[3] and roi[0] <= col <= roi[2]

    def generate_texture_mask(self, color: Color2DImage, roi1: ROI):
        mask = GrayScale2DImage()
        height = color.height()
        width = color.width()
        mask.resize(width, height)

        for i in range(height):
            for j in range(width):
                if self.contains(i, j, roi1):
                    mask.at(i, j).gray = 255
        return mask

    def mapping_2d_image_to_depth_map(self):
        # capture frame
        frame_all_2d_3d = Frame2DAnd3D()
        show_error(self.camera.capture_2d_and_3d(frame_all_2d_3d))
        color = frame_all_2d_3d.frame_2d().get_color_image()
        # print(type(color.data()), type(color.data()[0]), len(color.data()))
        # print(color.data().shape)
        np_image = color.data()
        self.roi = self.get_roi_from_yolo(np_image)
        if self.roi == -1:
            self.roi = (color.width() / 4, color.height() / 4,
                        color.width() * 3 / 4, color.height() *3 / 4)
        print("yolomech roi: ", self.roi)

        file_name = "color.png"
        cv2.imwrite(file_name, color.data())
        print("Capture and save the 2D image: {}".format(file_name))
        depth = frame_all_2d_3d.frame_3d().get_depth_map()
        intrinsics = CameraIntrinsics()
        show_error(self.camera.get_camera_intrinsics(intrinsics))

        # roi1 = (color.width() * 3 / 5, color.height() * 3 / 5,
        #         color.width() / 2, color.height() / 2)
        # roi2 = (color.width() * 2 / 5, color.height() * 2 /
        #         5, color.width() / 2, color.height() / 2)
        #
        #  Generate a mask of the following shape:
        #   ______________________________
        #  |                              |
        #  |                              |
        #  |   *****************          |
        #  |   *****************          |
        #  |   ************************   |
        #  |   ************************   |
        #  |          *****************   |
        #  |          *****************   |
        #  |                              |
        #  |______________________________|
        #
        # mask = self.generate_texture_mask(color, roi1, roi2)
        # mask = self.generate_texture_mask(color, self.roi)

        points_xyz = UntexturedPointCloud()
        print(depth.width())
        # show_error(get_point_cloud_after_mapping(
        #     depth, mask, intrinsics, points_xyz))
        self.convert_depth_map_to_point_cloud(depth, intrinsics, points_xyz)
        show_error(
            Frame3D.save_point_cloud(points_xyz, FileFormat_PCD, self.point_cloud_file))
        print("Save the untextured point cloud to file:", self.point_cloud_file)
        return 0

    def convert_depth_map_to_point_cloud(self, depth: DepthMap, intrinsics: CameraIntrinsics, xyz: UntexturedPointCloud):
        xyz.resize(self.roi[2] - self.roi[0], self.roi[3] - self.roi[1])
        j = 0
        for i in range(depth.width() * depth.height()):
            row = int(i / depth.width())
            col = int(i - row * depth.width())
            if self.roi[1] <= row < self.roi[3] and self.roi[0] <= col < self.roi[2]:
                xyz[j].z = depth[i].z / 1000
                xyz[j].x = float((col - intrinsics.depth.camera_matrix.cx)
                                * depth[i].z / intrinsics.depth.camera_matrix.fx / 1000)
                xyz[j].y = float((row - intrinsics.depth.camera_matrix.cy)
                                * depth[i].z / intrinsics.depth.camera_matrix.fy / 1000)
                j += 1
        

    def get_roi_from_yolo(self, color_image, padding: int=230):
        model = YOLO("./weights/best.pt")
        # color_image_3c = np.expand_dims(color_image, axis=2).repeat(3, axis=2)
        results = model.predict(color_image, save=False, show_conf=False)
        results[0].show()

        # find the charge box
        boxes = results[0].boxes.data.numpy()
        index_array = np.argwhere(boxes[:,-1] == 0)
    
        if np.size(index_array) == 0:
            print("Failed to detect the charge box")
            return -1

        index = index_array[0, 0]
        charge_box = boxes[index, 0:4].astype(dtype=np.int16).tolist()  # [x, y, x, y]

        return (charge_box[0] - padding, charge_box[1] - padding, \
                charge_box[2] + padding, charge_box[3] + padding)


    def main(self):
        if find_and_connect(self.camera):
            if not confirm_capture_3d():
                return
            flag = self.mapping_2d_image_to_depth_map()
            self.camera.disconnect()
            print("Disconnected from the camera successfully.")


if __name__ == '__main__':
    a = Mapping2DImageToDepthMap()
    a.main()
