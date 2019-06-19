#!/usr/bin/env python

from esiaf_pocketsphinx.pocketsphinx_wrapper import Wrapper
import pyesiaf
import rospy

# config
import yaml
import sys


nodename = 'esiaf_pocketsphinx'

# initialize rosnode
rospy.init_node(nodename)
pyesiaf.roscpp_init(nodename, [])

# read config
rospy.loginfo('Loading config...')
argv = sys.argv
if len(argv) < 2:
    rospy.logerr('Need path to configfile as first parameter!')
    exit('1')
path_to_config = argv[1]
data = yaml.safe_load(open(path_to_config))

rospy.loginfo('Creating pocketsphinx instance...')

wrapper = Wrapper(nodename=nodename, **data)

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


def input_callback(audio, timeStamps):
    wrapper.add_audio_data(audio, timeStamps)


def vad_finished_callback():
    wrapper.vad_finished_callback()


handler.add_input_topic(esiaf_audio_info, input_callback)
rospy.loginfo('input topic added')
handler.add_vad_finished_callback(esiaf_audio_info, vad_finished_callback)
rospy.loginfo('vad callback added')
handler.start_esiaf()

rospy.loginfo('Pocketsphinx ready!')
rospy.spin()

handler.quit_esiaf()
