from pocketsphinx import Jsgf, Decoder, get_model_path
import signal
from esiaf_ros.msg import RecordingTimeStamps, AugmentedAudio, SpeechHypothesis, SpeechInfo
import rospy
import StringIO
import os, sys

def msg_from_string(msg, data):
    msg.deserialize(data)


def msg_to_string(msg):
    buf = StringIO.StringIO()
    msg.serialize(buf)
    return buf.getvalue()


class Wrapper():
    def __init__(self, **kwargs):
        signal.signal(signal.SIGINT, self.stop)

        model_path = get_model_path()

        kwargs = {x: os.path.expandvars(kwargs[x]) for x in kwargs}

        nodename = kwargs.pop('nodename')
        grammar_file = kwargs.pop('grammar_file', None)
        grammar_rule = kwargs.pop('grammar_rule', None)
        grammar_name = kwargs.pop('grammar_name', None)

        kwargs.pop('esiaf_input_topic')

        if kwargs.get('dic') is not None and kwargs.get('dict') is None:
            kwargs['dict'] = kwargs.pop('dic')

        if kwargs.get('hmm') is None:
            kwargs['hmm'] = os.path.join(model_path, 'en-us')

        if kwargs.get('lm') is None:
            kwargs['lm'] = os.path.join(model_path, 'en-us.lm.bin')

        if kwargs.get('dict') is None:
            kwargs['dict'] = os.path.join(model_path, 'cmudict-en-us.dict')

        if kwargs.pop('verbose', False) is False:
            if sys.platform.startswith('win'):
                kwargs['logfn'] = 'nul'
            else:
                kwargs['logfn'] = '/dev/null'

        config = Decoder.default_config()

        for key, value in kwargs.items():
            if isinstance(value, bool):
                config.set_boolean('-{}'.format(key), value)
            elif isinstance(value, int):
                config.set_int('-{}'.format(key), value)
            elif isinstance(value, float):
                config.set_float('-{}'.format(key), value)
            elif isinstance(value, str):
                config.set_string('-{}'.format(key), value)

        self.decoder = Decoder(config)

        if grammar_file and grammar_rule and grammar_name:
            jsgf = Jsgf(grammar_file)
            rule = jsgf.get_rule(grammar_name + '.' + grammar_rule)
            fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
            self.decoder.set_fsg(grammar_name, fsg)
            self.decoder.set_search(grammar_name)

        self.start = None
        self.finish = None

        self.speech_publisher = rospy.Publisher(nodename + '/' + 'SpeechRec', SpeechInfo, queue_size=10)

    def stop(self, *args, **kwargs):
        raise StopIteration

    def hypothesis(self):
        hyp = self.decoder.hyp()
        if hyp:
            return hyp.hypstr
        else:
            return ''

    def vad_finished_callback(self):
        self.decoder.end_utt()
        result = ''
        if self.decoder.hyp():
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
            self.decoder.start_utt()
        self.finish = _recording_timestamps.finish
        bytearray = audio_data.tobytes()
        self.decoder.process_raw(bytearray, False, False)