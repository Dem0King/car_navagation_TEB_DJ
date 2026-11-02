#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import time
import sys
import re
import base64
import json
import openai
import numpy as np
from openai import OpenAI

from PIL import Image, ImageFont, ImageDraw
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Bool

################################################################################
# 用户调整部分
################################################################################

KEY = 'c828ce960ee1403b9fb0a201e78a8ba8.DTsGVzHb775SL9Qp'                        # 大模型APIKEY
API_URL = 'https://open.bigmodel.cn/api/paas/v4'                          # 大模型API标准端点（Coding Plan订阅者可改为 /api/coding/paas/v4）

# 模型选择说明（根据官方文档 2026-10）：
#   glm-5v-turbo  : 视觉多模态模型。【支持彻底关闭深度思考】（thinking.type="disabled" 有效）
#                   不会强制思考，响应速度快，适合数学题识别这类简单视觉问答。
#   glm-5.3-flash : 原生多模态 Flash 版。【不支持关闭思考】，只能通过 reasoning_effort=low 降低强度。
#   glm-4.6v      : 另一款支持关闭思考的视觉模型，可作备选。
#   glm-5.2       : 纯文本模型，不支持图像输入，不要用于当前视觉任务。
MODEL = 'glm-5v-turbo'                       # 使用 glm-5v-turbo 视觉模型，可彻底关闭深度思考

# 深度思考控制：
#   对于 glm-5v-turbo / glm-4.6v / glm-5.2 等：THINKING_TYPE="disabled" → 彻底关闭深度思考，无思考过程，响应最快
#   对于 glm-5.3 / glm-5.3-flash：只能用 THINKING_TYPE="enabled" + REASONING_EFFORT 调强度，无法关闭
THINKING_TYPE = "disabled"                   # 彻底关闭深度思考（模型直接输出答案，无 reasoning_content）
# REASONING_EFFORT = "low"                   # 仅 glm-5.3 系列需要此档位；当前模型已关闭思考，本参数无效
# 调用大模型前的助手提示词
PROMPT = """
Task Description: 
I need a problem-solving system that can recognise mathematical information in images and output numerical results. 

Specific requirements are as follows: 

Problem-Solving Task:

If the image contains a math problem, calculate the answer and output the result. 

Output Format: 

The output must be in pure numeric form, without any other text or explanation. Do not describe the image content or output the system's thought or reasoning process. 

Example: 

Input Image: 1+1 
Output: 2
"""

###################################################################################################

# 识别到图像后发布话题Flag
ar_pub = rospy.Publisher('/vres_flag', Bool, queue_size=1)
last_memory_time = 0        # 识别成功后记录时间戳，超过记忆时间清除识别结果

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")  # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

    # If the byte order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # Convert to BGR if the encoding is not already BGR
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None

    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg


def yi_vision_api(_PROMPT=PROMPT, img_path='/home/abot/temp/vl_now.jpg'):

    client = OpenAI(
        api_key = KEY,
        base_url = API_URL
    )
    
    # 编码为base64数据
    with open(img_path, 'rb') as image_file:
        image = 'data:image/jpeg;base64,' + base64.b64encode(image_file.read()).decode('utf-8')
    
    # 向大模型发起请求
    # 根据智谱官方文档 2026-10：
    #   glm-5.3 / glm-5.3-flash 系列：不支持关闭思考，传 thinking.type=disabled 会 400 报错
    #       -> 必须使用 thinking.type="enabled"，并通过 reasoning_effort="low" 来降低思考强度减少延迟
    #   glm-5.2 及以下：支持 thinking.type="disabled" 真正关闭深度思考
    request_kwargs = {
        "model": MODEL,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": _PROMPT},
                    {"type": "image_url", "image_url": {"url": image}}
                ]
            }
        ]
    }

    # 判断模型版本，组装正确的 thinking / reasoning_effort 参数
    model_lower = MODEL.lower()
    is_glm53_series = ("glm-5.3" in model_lower) or ("glm-5v" in model_lower and ("turbo" not in model_lower))

    try:
        if is_glm53_series:
            # glm-5.3 / glm-5.3-flash / glm-5.3-flashx：必须启用思考，只可调低推理强度
            completion = client.chat.completions.create(
                **request_kwargs,
                thinking={"type": "enabled"},
                reasoning_effort=REASONING_EFFORT
            )
        else:
            # glm-5.2 及以下：可根据 THINKING_TYPE 关闭或开启思考
            completion = client.chat.completions.create(
                **request_kwargs,
                extra_body={"thinking": {"type": THINKING_TYPE}}
            )
    except TypeError:
        # 兼容部分旧版 SDK 不接受关键字参数的情况，走 extra_body 回退
        try:
            extra = {"thinking": {"type": THINKING_TYPE if not is_glm53_series else "enabled"}}
            if is_glm53_series:
                extra["reasoning_effort"] = REASONING_EFFORT
            completion = client.chat.completions.create(
                **request_kwargs,
                extra_body=extra
            )
        except Exception as e2:
            rospy.logwarn(f'带思考参数的请求失败（{e2}），回退到普通请求...')
            completion = client.chat.completions.create(**request_kwargs)
    except Exception as e:
        rospy.logwarn(f'API 请求异常（{e}），回退到普通对话请求...')
        completion = client.chat.completions.create(**request_kwargs)

    # 解析大模型返回结果
    # 注意：启用思考模式后，模型回复可能包含 reasoning_content（思考过程），
    #       必须取 message.content 才是最终答案，不要把思考过程当结果。
    msg = completion.choices[0].message
    result_str = (msg.content or "").strip()
    result = str(result_str)

    # 如果开启了调试，把 reasoning_content（思考过程）打印出来便于排查（可选，不影响结果）
    # reasoning = getattr(msg, "reasoning_content", None)
    # if reasoning:
    #     rospy.loginfo(f'[模型思考过程]: {reasoning[:200]}...')

    return result

def top_view_shot(image_msg):
    global im_flag
    '''
    这里接收来自话题/usb_cam/image_raw的ROS图像格式的消息，并保存图像，是否拍照用的参数服务器，然后设置参数就行，注意要加命名空间路径
    '''
    # 将ROS图像消息转换为OpenCV格式
    img_bgr = imgmsg_to_cv2(image_msg)
    # 从参数服务器获取im_flag的值
    im_flag = rospy.get_param('/abot_vlm_node/im_flag', 255)

    if im_flag != 255:
        # 保存图像
        # rospy.loginfo('保存至temp/vl_now.jpg')
        cv2.imwrite('/home/abot/temp/vl_now.jpg', img_bgr)
        # 将im_flag重置为255
        rospy.set_param('/abot_vlm_node/im_flag', 255)

        # 调用视觉大模型API，返回结果存储到res中
        res = yi_vision_api()

        # 正则表达式公式，提取返回结果中的数值
        pattern = r'3[1-3]|4[0-2]|49|5[0-1]'
        numbers = re.findall(pattern, res)
        if len(numbers) != 0:
            data = int(numbers[-1])
        else:
            data = -0xFFFF  # 错误代码：-65535 

        # 更新参数服务器参数
        rospy.set_param('/abot_vlm_node/im_flog', data)                         # 更新识别到的结果
        rospy.set_param('/abot_vlm_node/im_flog_memory_time', time.time())      # 更新记忆时间

        print('大模型调用成功！')
        print(f'原始输出: {res} | 整理输出: {data}')


def main():
    global im_flag
    rospy.init_node('abot_vlm_node', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw', ROSImage, top_view_shot)
    rospy.loginfo('视觉大模型模块导入成功！')
    rospy.loginfo('准备识别...')

    # 从参数服务器获取im_flag的值
    im_flag = rospy.get_param('/abot_vlm_node/im_flag', 255)
    
    # 参考这种赋值方式，注意加入命名空间路径
    # rospy.set_param('/abot_vlm_node/im_flag', 1)
    
    rospy.spin()

if __name__ == '__main__':
    main()
