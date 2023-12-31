#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from uav_interface.uav import UAV

uav = UAV(uav_id=1)

def start():
    rospy.init_node('any_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    image = uav.camera.get_img_cam()
    model = cv2.dnn.readNet('models-ai/midas-small.onnx')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    blob = cv2.dnn.blobFromImage(image, 1/255., (256, 256), (123.675, 116.28, 103.53), True, False)
    model.setInput(blob)
    output = model.forward()[0]
    output = cv2.resize(output, (image.shape[1], image.shape[0]))
    output = cv2.normalize(output, None, 0, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imshow('image', image)
    cv2.imshow('Depth Map', output)
    cv2.waitKey(0)

start()