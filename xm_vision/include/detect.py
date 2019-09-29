import torch

from torch.autograd import Variable
import sys
sys.path.append("/home/domestic/ssd_pytorch")
from detection import *
from ssd_net_vgg import *
from voc0712 import *
import torch.nn as nn
import numpy as np
import cv2
import utils

class Detector:
    net = ''

    def __init__(self, model):
        if torch.cuda.is_available():
            torch.set_default_tensor_type('torch.cuda.FloatTensor')
        self.net = SSD()    # initialize SSD
        self.net = torch.nn.DataParallel(self.net)
        self.net.train(mode=False)
        # self.net.load_state_dict(torch.load('./weights/ssd300_VOC_60000.pth',map_location=lambda storage, loc: storage))
        self.net.load_state_dict(torch.load(model,map_location=lambda storage, loc: storage))

    def Detect(self, image):
        img_id = 60
        # image = cv2.imread('./pic/00000.png', cv2.IMREAD_COLOR)
        x = cv2.resize(image, (300, 300)).astype(np.float32)
        x -= (104.0, 117.0, 123.0)
        x = x.astype(np.float32)
        x = x[:, :, ::-1].copy()
        # plt.imshow(x)
        x = torch.from_numpy(x).permute(2, 0, 1)
        xx = Variable(x.unsqueeze(0))     # wrap tensor in Variable
        if torch.cuda.is_available():
            xx = xx.cuda()
        y = self.net(xx)
        softmax = nn.Softmax(dim=-1)
        detect = Detect(config.class_num, 0, 200, 0.01, 0.45)
        priors = utils.default_prior_box()

        loc,conf = y
        loc = torch.cat([o.view(o.size(0), -1) for o in loc], 1)
        conf = torch.cat([o.view(o.size(0), -1) for o in conf], 1)

        detections = detect(
            loc.view(loc.size(0), -1, 4),
            softmax(conf.view(conf.size(0), -1,config.class_num)),
            torch.cat([o.view(-1, 4) for o in priors], 0)
        ).data

        labels = VOC_CLASSES
        top_k=10

        # plt.imshow(rgb_image)  # plot the image for matplotlib

        # scale each detection back up to the image
        result_all = []
        scale = torch.Tensor(image.shape[1::-1]).repeat(2)
        for i in range(detections.size(1)):
            j = 0
            while detections[0,i,j,0] >= 0.4:
                result_single = []
                score = detections[0,i,j,0]
                label_name = labels[i-1]
                pt = (detections[0,i,j,1:]*scale).cpu().numpy()
                j+=1

                result_single.append(label_name)
                result_single.append(score)
                result_single.append(pt[0])
                result_single.append(pt[1])
                result_single.append(pt[2])
                result_single.append(pt[3])
                result_all.append(result_single)

                #display_txt = '%s: %.2f'%(label_name, score)
                #coords = (pt[0], pt[1]), pt[2]-pt[0]+1, pt[3]-pt[1]+1
                #color = colors_tableau[i]
                #cv2.rectangle(image,(pt[0],pt[1]), (pt[2],pt[3]), color, 2)
                #cv2.putText(image, display_txt, (int(pt[0]), int(pt[1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, 8)
        return result_all

# detector = Detector("/home/domestic/ssd_pytorch/weights/ssd300_VOC_60000.pth")
# image = cv2.imread('/home/domestic/ssd_pytorch/pic/00000.png', cv2.IMREAD_COLOR)
# detections = detector.Detect(image)
# print(detections)
# if detections[0][0] == "car":
#     print("1111")
