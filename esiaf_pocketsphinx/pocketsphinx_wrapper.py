from pocketsphinx import Pocketsphinx
import signal
from esiaf_ros.msg import RecordingTimeStamps, AugmentedAudio, SpeechHypothesis, SpeechInfo
import rospy


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
        result = None
        if self.hyp():
            result = self.__str__

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
        if not self.start:
            self.start = recording_timestamps.start
        self.finish = recording_timestamps.finish
        bytearray = audio_data.tobytes()
        self.process_raw(bytearray, self.no_search, self.full_utt)