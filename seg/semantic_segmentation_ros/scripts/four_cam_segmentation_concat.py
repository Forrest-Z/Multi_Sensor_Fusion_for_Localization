#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# System libs

import os
import sys
from distutils.version import LooseVersion
# Numerical libs
import numpy as np
import torch
import torch.nn as nn
from scipy.io import loadmat
from PIL import Image as PIL_Image
import csv
# ROS libs
import rospy
from sensor_msgs.msg import Image
import message_filters
# Our libs
from mit_semseg.dataset import imresize, img_transform, round2nearest_multiple
from mit_semseg.models import ModelBuilder, SegmentationModule
from mit_semseg.utils import colorEncode
from mit_semseg.lib.nn import async_copy_to
from mit_semseg.lib.utils import as_numpy
from mit_semseg.config import cfg


# 分割的类
class ROSSegmentation:
    # 初始化中解析参数并建立网络
    def __init__(self, cfg):
        # 参数解析
        self.cfg = cfg
        self.target_widths = None
        self.target_heights = None
        self.segSize = None

        # 配置GPU
        torch.cuda.set_device(self.cfg.gpu_id)

        # 配置编码器和解码器，这里直接调用了MIT库的API
        self.net_encoder = ModelBuilder.build_encoder(arch=self.cfg.MODEL.arch_encoder,
                                                      fc_dim=self.cfg.MODEL.fc_dim,
                                                      weights=self.cfg.MODEL.weights_encoder)
        self.net_decoder = ModelBuilder.build_decoder(arch=self.cfg.MODEL.arch_decoder,
                                                      fc_dim=self.cfg.MODEL.fc_dim,
                                                      num_class=self.cfg.DATASET.num_class,
                                                      weights=self.cfg.MODEL.weights_decoder, use_softmax=True)

        # 配置分割网络模型
        self.segmentation_module = SegmentationModule(self.net_encoder, self.net_decoder,
                                                      crit=nn.NLLLoss(ignore_index=-1))

        self.segmentation_module.cuda()
        self.segmentation_module.eval()

        # 发布图片
        self.seg1_pub = rospy.Publisher(
            "seg1/color/image_raw", Image, queue_size=10)
        self.seg2_pub = rospy.Publisher(
            "seg2/color/image_raw", Image, queue_size=10)
        self.seg3_pub = rospy.Publisher(
            "seg3/color/image_raw", Image, queue_size=10)
        self.seg4_pub = rospy.Publisher(
            "seg4/color/image_raw", Image, queue_size=10)

        # 订阅图片
        self.img1_sub = message_filters.Subscriber(
            'camera1/color/image_raw', Image)
        self.img2_sub = message_filters.Subscriber(
            'camera2/color/image_raw', Image)
        self.img3_sub = message_filters.Subscriber(
            'camera3/color/image_raw', Image)
        self.img4_sub = message_filters.Subscriber(
            'camera4/color/image_raw', Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.img1_sub, self.img2_sub, self.img3_sub, self.img4_sub], 10, 0.1)
        self.sync.registerCallback(self.callback)

        rospy.loginfo("Initialization Done. Running Inference .....")

    def set_size(self, img):
        ori_width, ori_height = img.size

        target_width_list = []
        target_height_list = []
        for this_short_size in self.cfg.DATASET.imgSizes:
            # calculate target height and width
            scale = min(this_short_size / float(min(ori_height, ori_width)),
                        self.cfg.DATASET.imgMaxSize / float(max(ori_height, ori_width)))
            target_height, target_width = int(
                ori_height * scale), int(ori_width * scale)

            # to avoid rounding in network
            target_width = round2nearest_multiple(
                target_width, self.cfg.DATASET.padding_constant)
            target_height = round2nearest_multiple(
                target_height, self.cfg.DATASET.padding_constant)

            target_width_list.append(target_width)
            target_height_list.append(target_height)

        self.target_widths = tuple(target_width_list)
        self.target_heights = tuple(target_height_list)
        self.segSize = (ori_height, ori_width)

        # 图像预处理，把图片修改成同一尺寸并转到torch中
        # test文件中使用pytorch的dataloader实现的

    def preprocess(self, img_arr):
        img = PIL_Image.fromarray(img_arr).convert('RGB')

        if (not self.target_widths) and (not self.target_heights):
            self.set_size(img)

        img_resized_list = []
        for t_width, t_height in zip(self.target_widths, self.target_heights):
            # resize images
            img_resized = imresize(img, (t_width, t_height), interp='bilinear')

            # image transform, to torch float tensor 3xHxW
            img_resized = img_transform(img_resized)
            img_resized = torch.unsqueeze(img_resized, 0)
            img_resized_list.append(img_resized)

        output = dict()
        output['img_ori'] = np.array(img)
        output['img_data'] = [x.contiguous() for x in img_resized_list]

        return output

    def callback(self, msg1, msg2, msg3, msg4):
        # rosmsg->numpy.具体看官方教程
        # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        img1_arr = np.frombuffer(msg1.data, dtype=np.uint8).reshape(
            msg1.height, msg1.width, -1)
        img2_arr = np.frombuffer(msg2.data, dtype=np.uint8).reshape(
            msg2.height, msg2.width, -1)
        img3_arr = np.frombuffer(msg3.data, dtype=np.uint8).reshape(
            msg3.height, msg3.width, -1)
        img4_arr = np.frombuffer(msg4.data, dtype=np.uint8).reshape(
            msg4.height, msg4.width, -1)
        # numpy拼接
        img_arr_concat = np.concatenate(
            (img1_arr, img2_arr, img3_arr, img4_arr), axis=1)

        # 把numpy的图像转成一个tensor的词典
        # 在test中是通过pytorch的datasetload实现的
        data_dict = self.preprocess(img_arr_concat)

        # 分割
        with torch.no_grad():
            scores = torch.zeros(1, cfg.DATASET.num_class,
                                 self.segSize[0], self.segSize[1])
            scores = async_copy_to(scores, self.cfg.gpu_id)

            for img_tensor in data_dict['img_data']:
                feed_dict = dict()
                feed_dict['img_data'] = img_tensor
                feed_dict = async_copy_to(feed_dict, self.cfg.gpu_id)

                pred_tmp = self.segmentation_module(
                    feed_dict, segSize=self.segSize)
                scores = scores + pred_tmp / len(self.cfg.DATASET.imgSizes)

            _, pred = torch.max(scores, dim=1)
            pred = as_numpy(pred.squeeze(0).cpu())

        # 颜色编码
        # TODO:不进行图像编码，而是把分割的结果直接通过话题发布
        pred_color = colorEncode(pred, colors).astype(np.uint8)

        # 把分割结果分隔开
        pred1, pred2, pred3, pred4 = np.hsplit(pred_color, 4)

        out_msg1 = Image()
        out_msg2 = Image()
        out_msg3 = Image()
        out_msg4 = Image()

        out_msg1.header, out_msg1.encoding = msg1.header, "rgb8"
        out_msg1.height, out_msg1.width, out_msg1.step = pred1.shape[0], pred1.shape[1], pred1.shape[1] * 3
        out_msg1.data = pred1.tobytes()

        out_msg2.header, out_msg2.encoding = msg2.header, "rgb8"
        out_msg2.height, out_msg2.width, out_msg2.step = pred2.shape[0], pred2.shape[1], pred2.shape[1] * 3
        out_msg2.data = pred2.tobytes()

        out_msg3.header, out_msg3.encoding = msg3.header, "rgb8"
        out_msg3.height, out_msg3.width, out_msg3.step = pred3.shape[0], pred3.shape[1], pred3.shape[1] * 3
        out_msg3.data = pred3.tobytes()

        out_msg4.header, out_msg4.encoding = msg4.header, "rgb8"
        out_msg4.height, out_msg4.width, out_msg4.step = pred4.shape[0], pred4.shape[1], pred4.shape[1] * 3
        out_msg4.data = pred4.tobytes()

        self.seg1_pub.publish(out_msg1)
        self.seg2_pub.publish(out_msg2)
        self.seg3_pub.publish(out_msg3)
        self.seg4_pub.publish(out_msg4)

        return


if __name__ == '__main__':
    assert LooseVersion(torch.__version__) >= LooseVersion(
        '0.4.0'), 'PyTorch>=0.4.0 is required'

    # 初始化ROS节点
    rospy.init_node("semantic_segmentation_ros", anonymous=True)

    gpu_id = int(rospy.get_param("~gpu_id"))
    # 颜色编码文件
    color150_mat_filepath = rospy.get_param("~color150_mat_filepath")
    objects150_csv_filepath = rospy.get_param("~objects150_csv_filepath")
    # 训练模型
    model_ckpt_dir = rospy.get_param("~model_ckpt_dir")
    # 模型参数
    cfg_file_path = rospy.get_param("~cfg_filepath")

    colors = loadmat(color150_mat_filepath)['colors']
    names = {}
    with open(objects150_csv_filepath) as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            names[int(row[0])] = row[5].split(";")[0]

    # cfg是yacs的参数配置节点，下面根据参数文件给这个cfg节点追加参数配置
    cfg.merge_from_file(cfg_file_path)
    opts = ["gpu_id", gpu_id, "DIR", model_ckpt_dir]
    cfg.merge_from_list(opts)

    rospy.loginfo("Loaded configuration file {}".format(cfg_file_path))

    cfg.MODEL.arch_encoder = cfg.MODEL.arch_encoder.lower()
    cfg.MODEL.arch_decoder = cfg.MODEL.arch_decoder.lower()

    cfg.MODEL.weights_encoder = os.path.join(
        cfg.DIR, 'encoder_' + cfg.TEST.checkpoint)
    cfg.MODEL.weights_decoder = os.path.join(
        cfg.DIR, 'decoder_' + cfg.TEST.checkpoint)

    assert os.path.exists(cfg.MODEL.weights_encoder) and os.path.exists(
        cfg.MODEL.weights_decoder), "checkpoint does not exitst!"

    try:
        # 建立一个对象，把参数传cfg入
        seg_obj = ROSSegmentation(cfg)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down Node!!!")
        sys.exit(0)
