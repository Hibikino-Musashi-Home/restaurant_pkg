#! /usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#RCJ2016 Restaurant用音声認識ActionのROSノード
#
#author: Takahiro TSUCHIDA
#date: 16/03/17
#--------------------------------------------------


#-----------speech recognition------------------------
from __future__ import print_function
import socket
from contextlib import closing
import commands

import re
import csv
import numpy as np
from copy import deepcopy
#-----------speech recognition------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *


rospy.sleep(5) #paramノードが立ち上がるまで待つ


#--------------------------------------------------
#--------------------------------------------------
class SpeechRec(object):
    #--------------------------------------------------
    #--------------------------------------------------
    def __init__(self, julius_bufsize, julius_sock, RecgDicts):
        self._speech_rec_action_server = actionlib.SimpleActionServer('speech_rec_action', SpeechRecAction, execute_cb = self.speech_rec)
        self._speech_rec_action_server.start()
        self.julius_bufsize = julius_bufsize
        self.julius_sock = julius_sock
        self.RecgDicts = RecgDicts
        self.order_counter = 0


    #--------------------------------------------------
    #--------------------------------------------------
    def speech_rec(self, goal):
        #--------------------------------------------------
        #追跡開始の呼びかけを認識する
        #--------------------------------------------------
        if goal.speech_rec_goal == 'SRec_StartFollow':
            if rospy.get_param('/param/dbg/sm/flow') == 0:                
                commonf_speech_single('ウェイターは私に話しかけて下さい。')
                while 1:
                    text = self.voice2text()
                    flag = self.returnFlag('CMD', text)
                    if text == '':
                        pass
                    elif flag == 'start':
                        break
                    else:
                        commonf_speech_single('すみません。聞き取れませんでした。')

            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)


        #--------------------------------------------------
        #追跡停止の呼びかけを認識する
        #--------------------------------------------------
        elif goal.speech_rec_goal == 'FollowWaiter':
            if rospy.get_param('/param/dbg/sm/flow') == 0:
                while 1:
                    text = self.voice2text()
                    flag = self.returnFlag('CMD', text)
                    if text == '':
                        pass
                    elif flag == 'stop':
                        commonf_speech_single('止まります')
                        break
                    else:
                        commonf_speech_single('すみません。聞き取れませんでした。')

            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)


        #--------------------------------------------------
        #テーブル番号と位置を認識する
        #--------------------------------------------------
        elif goal.speech_rec_goal == 'SRec_TablePos':
            if rospy.get_param('/param/dbg/sm/flow') == 0:
                commonf_speech_single('どちらがなにテーブルですか？')
                while 1:
                    ok_flag = 0
                    table_k, table_v = 0, 0
                    pos_k, pos_v = 0, 0
                    text = self.voice2text()
                    for table_key, table_value in self.RecgDicts['TABLE'].items():
                        if table_key in text:
                            table_k = table_key
                            table_v = table_value
                            break
                    for pos_key, pos_value in self.RecgDicts['POS'].items():
                        if pos_key in text:
                            pos_k = pos_key
                            pos_v = pos_value
                            break
                    if table_k and pos_k:
                        commonf_speech_single(pos_k+'が'+table_k+'でいいですか？')
                        while 1:
                            text = self.voice2text()
                            flag = self.returnFlag('CMD', text)
                            if text == '':
                                pass
                            elif flag == 'ok':
                                ok_flag = 1
                                commonf_speech_single('覚えます。')
                                rospy.set_param('/param/table/num', table_v) #テーブル番号
                                rospy.set_param('/param/table/lor', pos_v) #テーブル位置 left right
                                break
                            elif flag == 'ng':
                                commonf_speech_single('もう一度どちらがなにテーブルか教えてください。')
                                break
                            else:
                                commonf_speech_single('すみません。聞き取れませんでした。')
                    elif text == '':
                        pass
                    else:
                        commonf_speech_single('すみません。聞き取れませんでした。')

                    if ok_flag == 1:
                        break

            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)


        #--------------------------------------------------
        #キッチンの方向を認識する
        #--------------------------------------------------
        elif goal.speech_rec_goal == 'SRec_WhichSideIsKitchen':
            commonf_speech_single('キッチンの方向を教えてください。')
            while 1:
                ok_flag = 0
                pos_k, pos_v = 0, 0
                text = self.voice2text()
                for pos_key, pos_value in self.RecgDicts['POS'].items():
                    if pos_key in text:
                        pos_k = pos_key
                        pos_v = pos_value
                        break
                if pos_k:
                    commonf_speech_single(pos_k+'でいいですか？')
                    while 1:
                        text = self.voice2text()
                        flag = self.returnFlag('CMD', text)
                        if text == '':
                            pass
                        elif flag == 'ok':
                            ok_flag = 1
                            commonf_speech_single('覚えます。')
                            rospy.set_param('/param/kitchen/lor', pos_v) #キッチン位置 left right
                            break
                        elif flag == 'ng':
                            commonf_speech_single('もう一度キッチンの方向を教えてください')
                            break
                        else:
                            commonf_speech_single('すみません。聞き取れませんでした。')
                elif text == '':
                    pass
                else:
                    commonf_speech_single('すみません。聞き取れませんでした。')

                if ok_flag == 1:
                    break


            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)

        #--------------------------------------------------
        #オーダをとるテーブル番号を認識
        #--------------------------------------------------
        elif goal.speech_rec_goal == 'SRec_WhichTableToAttend':
            if rospy.get_param('/param/dbg/sm/flow') == 0:
                commonf_speech_single('なにテーブルのオーダーを取ってきますか？')
                while 1:
                    table_k = 0
                    table_v = 0
                    for_break_flag = 0
                    order_start_flag = 0
                    text = self.voice2text()
                    for table_key, table_value in self.RecgDicts['TABLE'].items():
                        if table_key in text:
                            table_k = table_key
                            table_v = table_value
                            commonf_speech_single(table_k+'でいいですか？')
                            while 1:
                                text = self.voice2text()
                                flag = self.returnFlag('CMD', text)
                                if text == '':
                                    pass
                                elif flag == 'ok':
                                    commonf_speech_single(table_k+'でオーダーをとります。')
                                    table_num = rospy.get_param('/param/order/table')
                                    table_num[int(rospy.get_param('/param/order/cnt'))] = table_v 
                                    rospy.set_param('/param/order/table', table_num)
                                    for_break_flag = 1
                                    order_start_flag = 1
                                    break
                                elif flag == 'ng':
                                    for_break_flag = 1
                                    commonf_speech_single('なにテーブルのオーダーを取ってきますか？')
                                    break
                                else:
                                    commonf_speech_single('すみません。聞き取れませんでした。')
                        if for_break_flag == 1:
                            break
                    else:
                        commonf_speech_single('すみません。聞き取れませんでした。')

                    if text == '':
                        pass

                    if order_start_flag == 1:
                        break

            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)


        #--------------------------------------------------
        #オーダーを認識
        #--------------------------------------------------
        elif goal.speech_rec_goal == 'SRec_Order':
            if rospy.get_param('/param/dbg/sm/flow') == 0:
                commonf_speech_single('開始音のあとに、ドリンクは、グリーンティを下さい。のように。フードは、カップヌードルとチップスターを下さい。のように言って下さい。確認の問いかけには、それであってるよ。間違ってるよ。で答えて下さい。ご注文をどうぞ。')

                order_cnt = rospy.get_param('/param/order/cnt')
                order_table = rospy.get_param('/param/order/table')
                order_obj = rospy.get_param('/param/order/obj')
                cnt = 0
                order_obj_list = list()

                while 1:
                    order_ok_flag, order_ng_flag = 0, 0
                    order_end_flag = 0
                    text = self.voice2text()
                    drink_k, drink_v = 0, 0
                    for drink_key, drink_value in self.RecgDicts['DRINK'].items():
                        if drink_key in text:
                            drink_k = drink_key
                            drink_v = drink_value
                            commonf_speech_single(drink_k+'でいいですか？')
                            while 1:
                                text = self.voice2text()
                                flag = self.returnFlag('CMD', text)
                                if text == '':
                                    pass
                                elif flag == 'ok':
                                    #DBに追加
                                    order_obj[order_cnt][cnt] = drink_v
                                    cnt += 1
                                    order_obj_list.append(drink_k)
                                    order_ok_flag = 1
                                    break
                                elif flag == 'ng':
                                    commonf_speech_single('もう一度注文を言ってください。')
                                    order_ng_flag = 1
                                    break
                                else:
                                    commonf_speech_single('すみません。聞き取れませんでした。')
                        if order_ok_flag == 1 or order_ng_flag == 1:
                            drink_k, drink_v = 0, 0
                            break
                    else:
                        food_k, food_v = list(), list()
                        for food_key, food_value in self.RecgDicts['FOOD'].items():
                            if food_key in text:
                                food_k.append(food_key)
                                food_v.append(food_value)
                        if len(food_k) == 1:
                            commonf_speech_single(food_k[0]+'と、もうひとつは、なんですか？')
                            while 1:
                                text = self.voice2text()
                                flag_tmp = self.returnFlag('CMD', text)
                                if flag_tmp == 'ng':
                                    commonf_speech_single('もう一度注文を言ってください。')
                                    break
                                for food_key, food_value in self.RecgDicts['FOOD'].items():
                                    if food_key in text:
                                        food_k.append(food_key)
                                        food_v.append(food_value)
                                        break
                                else:
                                    commonf_speech_single('すみません。聞き取れませんでした。')
                                if len(food_k) == 2:
                                    break
                        if len(food_k) == 2:
                            commonf_speech_single(food_k[0]+'と'+food_k[1]+'でいいですか？')
                            while 1:
                                text = self.voice2text()
                                flag = self.returnFlag('CMD', text)
                                if text == '':
                                    pass
                                elif flag == 'ok':
                                    order_obj[order_cnt][cnt] = food_v[0]
                                    order_obj[order_cnt][cnt+1] = food_v[1]
                                    cnt += 2
                                    order_obj_list.append(food_k[0])
                                    order_obj_list.append(food_k[1])
                                    order_ok_flag = 1
                                    break
                                elif flag == 'ng':
                                    commonf_speech_single('もう一度注文を言ってください。')
                                    order_ng_flag = 1
                                    break
                                else:
                                    commonf_speech_single('すみません。聞き取れませんでした。')
                        if order_ok_flag == 1 or order_ng_flag == 1:
                            food_k, food_v = list(), list()
                            #break

                    if order_ok_flag == 1:
                        item_counts = ['０', '１', '２', '３', '４', '５', '６', '７', '８']
                        item_count = item_counts[len(order_obj_list)]
                        orders = '、'.join(order_obj_list)
                        commonf_speech_single('以上でいいですか')
                        while 1:
                            text = self.voice2text()
                            flag = self.returnFlag('CMD', text)
                            if text == '':
                                pass
                            elif flag == 'ok':
                                kakunin = 'かしこまりました、' + orders + 'を持ってきます。'
                                commonf_speech_single(kakunin)
                                order_end_flag = 1
                                break
                            elif flag == 'ng':
                                if order_obj[order_cnt][2] != 0:
                                    kakunin = 'これ以上注文できません、' + orders + 'を持ってきます。'
                                    commonf_speech_single(kakunin)
                                    order_end_flag = 1
                                    break
                                else:
                                    commonf_speech_single('追加のご注文をどうぞ。')
                                    break
                            else:
                                commonf_speech_single('すみません。聞き取れませんでした。')
                    if text == '':
                        pass

                    if order_end_flag == 1:
                        break
                print(order_obj)
                rospy.set_param('/param/order/obj', order_obj)

            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)


        #--------------------------------------------------
        #--------------------------------------------------
        else:
            rospy.logwarn('[speech_rec]: ステートに対する処理が記述されていません')
            result = SpeechRecResult(speech_rec_result = False)
            self._speech_rec_action_server.set_succeeded(result)


    #--------------------------------------------------
    #Juliusから返ってきた出力（sock,bufsize,XML形式）から、文章部分の抽出を行う関数
    #-------------------------------------------------- 
    def voice2text(self):
        #sentence = ''
        #以追加socket削除するため以下2行を追加
        socket = self.julius_sock
        commonf_actionf_sound_effect_multi('speech_rec')
        rospy.sleep(0.5)
        os.system('amixer -c 2 sset "Mic" 80%')
        #rospy.sleep(0.5)
        recv_data = socket.recv(self.julius_bufsize)
        recv_data = ''
        #print('------ speech rec start ------------')
        while True:
            #socket = self.julius_sock
            recv_data += socket.recv(self.julius_bufsize)
            #sentence_start = re.findall(r'<s>', recv_data)
            sentence_end = re.findall(r'</s>', recv_data)
            if sentence_end:
                sentence = ''
                matchs = re.findall(r'<WHYPO WORD=".*?"', recv_data)
                for match in matchs:
                    s = match[13:-1]
                    sentence += s
                out_sentence = sentence.strip()
                break
        os.system('amixer -c 2 sset "Mic" 0%')
        print(out_sentence)
        return out_sentence


    #--------------------------------------------------
    #認識した文章に対応するフラグを返す
    #--------------------------------------------------  
    def returnFlag(self, state, text):
        if text == '':
            return 'error'
        RecgDict = self.RecgDicts[state]
        for k, v in RecgDict.items():
            if text.find(k) >= 0:
                return v
        else:
            return 0


#--------------------------------------------------
#メイン関数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])


    #初期設定
    #-----------speech recognition------------------------
    julius_host = 'localhost'
    julius_port = 10500
    julius_bufsize = 4096 * 4

    julius_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    julius_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, julius_bufsize)
    julius_sock.connect((julius_host, julius_port))
    RecgDicts = {
        'TABLE': {'１番テーブル': 1, '２番テーブル': 2, '３番テーブル': 3, 'Ａテーブル': 4, 'Ｂテーブル': 5, 'Ｃテーブル': 6},
        'POS': {'右': 'right', '左': 'left'},
        #'DRINK': {'グリーンティ': 1, 'オレンジジュース': 2, 'ブラウンティ': 3,
        #    'ジャパニーズティ': 4, 'レッドティ': 5, 'レモンティ': 6,
        #    'ストロベリージュース': 7},
        #'FOOD': {'カップスター': 8, 'カップヌードル': 9, 'シーフードヌードル': 10,
        #    'コリアンスープ': 11, 'エッグスープ': 12, 'オニオンドレッシング': 13,
        #    'ジャパニーズドレッシング': 14, 'チップスター': 15, 'プリングルス': 16,
        #    'ロングポテト': 17, 'ブルーポテト': 18, 'レッドポテト': 19, 'スティックポテト': 20},
        'CMD': {'エクシアちょっときて': 'start', 'ここで止まって': 'stop', 'それであってるよ': 'ok', '間違ってるよ': 'ng'}
    }
    objs = rospy.get_param('/param/obj/db')
    RecgDicts['DRINK'] = dict([[obj['obj_name_j'].encode('utf-8'), obj['obj_id']] for i, obj in enumerate(objs) if obj['obj_class'] == u'drink'])
    RecgDicts['FOOD'] = dict([[obj['obj_name_j'].encode('utf-8'), obj['obj_id']] for i, obj in enumerate(objs) if obj['obj_class'] == u'food' or obj['obj_class'] == u'snack'])
    #-----------speech recognition------------------------


    speech_rec = SpeechRec(julius_bufsize, julius_sock, RecgDicts)


    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        main_rate.sleep()
