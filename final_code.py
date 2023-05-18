# !/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.color_pub = rospy.Publisher('/rotate_cmd', Header, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # default: STOP
            
            # {(b,g,r) : 개수} 를 저장하는 dictionary
            color_dict = dict()
            color_dict.clear()
            
            x = image.shape[0]
            y = image.shape[1]
            start = 0.05
            end = 0.95
			
            # 상하좌우 5% 잘라내고 시작하기
            for i in range(round(x*start),round(x*end),10):
                for j in range(round(y*start),round(y*end),10):
                    
                    # 적당히 그룹화 (num=15일 때)
                    # 0~14->0 / 15~29->15 / 30~44->30 / 45~59 -> 45
                    num = 15
                    b = image[i][j][0]//num*num
                    g = image[i][j][1]//num*num
                    r = image[i][j][2]//num*num
                
                    # 이미 한 번 나왔던 (b,g,r)이면 개수 1 추가
                    if (b,g,r) in color_dict:
                        color_dict[(b,g,r)]+=1
                        # 가장 많이 나온 색깔 찾기
                    # 처음 나온 (b,g,r) 이면 개수=1로 딕셔너리에 추가        
                    else:
                        color_dict[(b,g,r)]=1
            
            sorted_dict = sorted(color_dict.items(), key = lambda item: item[1], reverse = True)
            # blue +1 : 반시계  // red -1 : 시계
            
            color_names={
                (255,255,255):'White',
                (-30,-30,-30):'Black',
                (0,0,255):'Red',
                (0,100,255):'Orange',
                (0,255,0):'Green',
                (255,0,0):'Blue',
                (0,220,220):'Yellow',
                (255,0,255):'Pink',
                (255,128,0):'SkyBlue'
            }
            
            # 가장 가까운 색깔 찾기
            
            color_cnt = dict()
            color_cnt.clear()
            
            color_cnt['White']=0
            color_cnt['Black']=0
            color_cnt['Red']=0
            color_cnt['Green']=0
            color_cnt['Blue']=0
            color_cnt['Yellow']=0
            color_cnt['Pink']=0
            color_cnt['SkyBlue']=0  
            color_cnt['Orange']=0
            
            black_basket = 250
            basket = 0

            for ((b,g,r),z) in sorted_dict:
                min_dist = 10000000
                for key in color_names:
                    dist = (b-key[0])**2+(g-key[1])**2+(r-key[2])**2
                    if dist < min_dist:
                        min_dist = dist
                        min_color = color_names[key]
                        
                color_cnt[min_color] += color_dict[(b,g,r)]
                
                if min_color!='Black':
                    basket += z
                    if basket>=1000:
                        break
                else:
                    if black_basket>z:
                        black_basket-=z
                    else:
                        black_basket=0
                        basket += z-black_basket

            color_cnt['Blue']+=color_cnt['SkyBlue']
            color_cnt['SkyBlue']=0
            color_cnt['Red']+=color_cnt['Orange']
            color_cnt['Orange']=0
            color_cnt['Black']-=200

            most = (sorted(color_cnt.items(), key = lambda item: item[1], reverse = True)[0])[0]
            if most=='Red':
                msg.frame_id = '-1'
            elif most=='Blue':
                msg.frame_id = '+1'
                
            self.color_pub.publish(msg)

        except CvBridgeError as e:
            print(e)


    def rospy_shutdown(self, signal, frame):
        rospy.signal_shutdown("shut down")
        sys.exit(0)

if __name__ == '__main__':
    detector = DetermineColor()
    rospy.init_node('CompressedImages1', anonymous=False)
    rospy.spin()
