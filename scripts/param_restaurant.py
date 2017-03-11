#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#RCJ2016 Restaurant用パラメータのROSノード
#
#author: Yutaro ISHIDA
#date: 16/03/17
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *
from common_param import *


#--------------------------------------------------
#メイン関数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])


    #全デバッグモードを使う
    rospy.set_param('/param/dbg/sm/all', 0) #bool    

    #選択されたデバッグモードを使う
    if rospy.get_param('/param/dbg/sm/all') == 0:
        rospy.set_param('/param/dbg/sm/flow', 0) #全ての機能なしでsmを流す bool
        rospy.set_param('/param/dbg/sm/stepin', 0) #ステート入口ごとにキー入力を促す bool
        rospy.set_param('/param/dbg/sm/stepout', 0) #ステート出口ごとにキー入力を促す bool
        rospy.set_param('/param/dbg/speech/onlyspeech', 0) #音声認識のみデバッグ bool
        rospy.set_param('/param/dbg/speech/ssynlog', 0) #音声合成の文章をデバッグ表示 bool
    #else以下は触らない
    else:
        rospy.set_param('/param/dbg/sm/flow', 1) #bool
        rospy.set_param('/param/dbg/sm/stepin', 1) #bool
        rospy.set_param('/param/dbg/sm/stepout', 1) #bool
        rospy.set_param('/param/dbg/speech/onlyspeech', 1) #bool
        rospy.set_param('/param/dbg/speech/ssynlog', 1) #bool


    rospy.set_param('/param/table/cnt', 0) #何個のテーブルの位置を覚えたか int
    rospy.set_param('/param/table/num', 0) #今覚えるべきテーブルの番号 int
    rospy.set_param('/param/table/lor', '') #今覚えるべきテーブルの方向(オペレータ視点で) string left right
    rospy.set_param('/param/table/pos', [{'x':0.0, 'y':0.0, 'yaw':0.0}, #テーブル1
                                         {'x':0.0, 'y':0.0, 'yaw':0.0}, #テーブル2
                                         {'x':0.0, 'y':0.0, 'yaw':0.0}, #テーブル3
                                         {'x':0.0, 'y':0.0, 'yaw':0.0}, #テーブルA
                                         {'x':0.0, 'y':0.0, 'yaw':0.0}, #テーブルB
                                         {'x':0.0, 'y':0.0, 'yaw':0.0}]) #テーブルC テーブルの位置 #float


    rospy.set_param('/param/kitchen/lor', '') #キッチンの方向(オペレータ視点で) string left right
    rospy.set_param('/param/kitchen/pos', {'x':0.0, 'y':0.0, 'yaw':0.0}) #キッチンの位置 float


    rospy.set_param('/param/order/cnt', 0) #何回オーダを取りに行ったか int
    rospy.set_param('/param/order/table', [0, 0]) #オーダを取ったテーブルの番号 int
    rospy.set_param('/param/order/obj', [[0, 0, 0, 0], [0, 0, 0, 0]]) #各テーブルのオーダのオブジェクトid int


    rospy.set_param('/param/delivery/table', 0) #何番のテーブルに配達に行くか int


    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        main_rate.sleep()
