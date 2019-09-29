## 注意
需要装：
libfreenect2的python版本和cpp版本驱动（如果还用kinect2的话）
darkent-yolov3（建议用github上一个叫AlexeyAB的，他的可以直接生成动态链接库）
opencv
cuda9
cudnn
其他的缺啥装啥吧

xm_vision应放于Ros的工作空间里，比如catkin_ws/src中
八月份的比赛使用到的文件主要放于xm_vision/src中：object_all.py object_detect.py people_identify.py people_tracking.cpp
所有的py文件都需要手动设置成可执行文件
项目使用方法：
## GPSR：
rosrun xm_vision object_detect.py
rosservice call /get_position "object_name: ' ' people_id: 0"
object_name为需要进行识别的物体名字，包括人，people_id对应的是不同项目的id号，在现有的项目中，people_id为0的时候，是进行物体的识别，id为1的时候，是进行单个人的识别，id为2的时候是同时进行两个人的识别，id为3的时候，是进行打电话的人的识别

## 多人辨识：
rosrun xm_vision people_identify.py
rosservice call /get_position "object_name: 'person' people_id: "
object_name只能是person，id号为-1的时候进行所有人的识别，id为xx（状态机传递）的时候会进行特定人的识别，xx对应的是人的id号

people_identify2.py文件中当给的id号为非-1的时候（状态机给的是4）会进行一次性所有特定人的识别，这个需要看状态机的需求，如果他们决定识别特定人要一个一个识别就用people_identify.py好了

## Object_all.py（忘了项目名字了）
rosrun xm_vision object_all.py
这个是进行所有物体的识别，会生成out_all.txt文件，该文件里是识别到的物体名称，目前是按照一定的顺序（忘了从左还是从右了），从上至下（因为可能有两层物品）的顺序写入的（方便机械臂抓取）

## 跟随（导航）
rosrun xm_vision people_tracking
跟随项目

四月份的项目里还有餐厅项目和性别识别的项目，餐厅项目使用的是dinging_room.cpp文件，性别识别是gender.cpp（如果我没记错的话），餐厅项目使用的方式和GPSR项目差不过，性别识别用的是虹软的SDK，具体使用方法它官方SDK说明文档里面有，可以换成百度的或者其他的
印象中做性别识别项目的时候突然说还要进行站坐的检测，这个调用的应该是百度的SDK，都是在线的，但具体的代码部分忘了

xm_vision中的代码是主体结构，它调用的所有函数均在Vision文件中
Vision/Function里面，Kinect.py文件用于摄像头的打开关闭、存图片、计算实际距离，Processor.py用于调用网络模型进行时别并返回坐标等，darknet.py是改写过的yolov3的python版本的检测文件，Find_people.py文件用于多人辨识项目

Vision/Result中存放的是检测结果

Vision/models存放网络模型及其必要的配置文件

Vision/Frame存放实时图片，Kinect.py写得不好，图片的调用都是先写到本地然后再读取进行识别的，但在cpp版本中是可以直接传图片进行识别的，这个地方需要改

Vision/Camera和Vison/Processor文件里存放的都是Kincet.py和Processor.py文件对应的cpp版本，都是通过生成动态链接库进行调用的，其中Camera/build/中有一个可执行文件take_photos，这个是拍数据集用的，拍出来的照片放在Vision/Data文件中

其他的应该没什么了，如果有问题再说吧
建议代码重写吧
如果因为主线程子线程的问题觉得python写不好的话，也可以改回cpp