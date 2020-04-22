# -*- coding: utf-8 -*-

import math

# 在该文件里面配置3 dof运动平台的各种参数，单位 mm
CYLIDER_HEIGHT = 400.0
MAX_CYLIDER_LENGTH = 35.0 # 总行程的一半
FB_LENGTH = 560.0
LR_LENGTH = 560.0
# 最大俯仰角采用了近似的手段
MAX_PITCH_ANGLE = int(math.atan(MAX_CYLIDER_LENGTH * 2 / FB_LENGTH) * 180.0 / math.pi)
MAX_ROLL_ANGLE = int(math.atan(MAX_CYLIDER_LENGTH * 2 / LR_LENGTH) * 180.0 / math.pi)
