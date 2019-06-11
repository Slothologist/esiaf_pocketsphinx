from pocketsphinx import Pocketsphinx
import signal
from esiaf_ros.msg import RecordingTimeStamps, AugmentedAudio, SpeechHypothesis, SpeechInfo
import rospy
import StringIO

def msg_from_string(msg, data):
    msg.deserialize(data)


def msg_to_string(msg):
    buf = StringIO.StringIO()
    msg.serialize(buf)
    return buf.getvalue()


class Wrapper(Pocketsphinx):
    def __init__(self, **kwargs):
        signal.signal(signal.SIGINT, self.stop)

        self.no_search = kwargs.pop('no_search', False)
        self.full_utt = kwargs.pop('full_utt', False)
        nodename = kwargs.pop('nodename')

        self.start = None
        self.finish = None

        self.speech_publisher = rospy.Publisher(nodename + '/' + 'SpeechRec', SpeechInfo, queue_size=10)

        super(Wrapper, self).__init__(**kwargs)

    def stop(self, *args, **kwargs):
        raise StopIteration

    def vad_finished_callback(self):
        self.end_utt()
        result = ''
        if self.hyp():
            result = self.hypothesis()
        rospy.loginfo('understood: \'' + str(result) + '\'')

        hypo = SpeechHypothesis()
        hypo.recognizedSpeech = result
        hypo.probability = 1.0

        time = RecordingTimeStamps()
        time.start = self.start
        time.finish = self.finish

        speechInfo = SpeechInfo()
        speechInfo.hypotheses = [hypo]
        speechInfo.duration = time

        self.speech_publisher.publish(speechInfo)

        self.start = None
        self.finish = None


    def add_audio_data(self, audio_data, recording_timestamps):
        _recording_timestamps = RecordingTimeStamps()
        msg_from_string(_recording_timestamps, recording_timestamps)
        rospy.loginfo('got audio!')
        if not self.start:
            self.start = _recording_timestamps.start
            self.start_utt()
        self.finish = _recording_timestamps.finish
        bytearray = audio_data.tobytes()
        self.process_raw(bytearray, self.no_search, self.full_utt)