#!/usr/bin/env python

from esiaf_pocketsphinx.pocketsphinx_wrapper import Wrapper
import pyesiaf
import rospy
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

# config
import yaml
import sys


nodename = 'esiaf_pocketsphinx'

# initialize rosnode
rospy.init_node(nodename)
roscpp_init(nodename, [])

# read config
rospy.loginfo('Loading config...')
argv = sys.argv
if len(argv) < 2:
    rospy.logerr('Need path to configfile as first parameter!')
    exit('1')
path_to_config = argv[1]
data = yaml.safe_load(open(path_to_config))

rospy.loginfo('Creating pocketsphinx instance...')

wrapper = Wrapper(nodename=nodename)

rospy.loginfo('Creating esiaf handler...')
handler = pyesiaf.Esiaf_Handler('pocketsphinx', pyesiaf.NodeDesignation.SpeechRec, sys.argv)

rospy.loginfo('Setting up esiaf...')
esiaf_format = pyesiaf.EsiafAudioFormat()
esiaf_format.rate = pyesiaf.Rate.RATE_16000
esiaf_format.bitrate = pyesiaf.Bitrate.BIT_INT_16_SIGNED
esiaf_format.endian = pyesiaf.Endian.LittleEndian
esiaf_format.channels = 1

esiaf_audio_info = pyesiaf.EsiafAudioTopicInfo()
esiaf_audio_info.topic = data['esiaf_input_topic']
esiaf_audio_info.allowedFormat = esiaf_format

rospy.loginfo('adding input topic...')


def input_callback(**kwargs):
    wrapper.add_audio_data(**kwargs)


def vad_finished_callback():
    wrapper.vad_finished_callback()


handler.add_input_topic(esiaf_audio_info, input_callback)
handler.add_vad_finished_callback(data['esiaf_input_topic'], vad_finished_callback)
handler.start_esiaf()

rospy.loginfo('Pocketsphinx ready!')
rospy.spin()

handler.quit_esiaf()
