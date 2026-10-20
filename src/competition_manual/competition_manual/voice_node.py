"""Offline wake word and one-way manual lock for the competition handoff."""

import json
import audioop
import os
import queue
import subprocess
import threading
import time

import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from vosk import KaldiRecognizer, Model


STAGE_TIMEOUT_SEC = 5.0


class VoiceNode(Node):
    def __init__(self):
        super().__init__('competition_voice')
        self.declare_parameter('model_path', '')
        self.declare_parameter('wake_sound', '')
        self.declare_parameter('auto_sound', '')
        self.declare_parameter('audio_device_index', -1)
        self.declare_parameter('audio_output_device', 'plughw:CARD=Device,DEV=0')
        self.declare_parameter('use_array_wake', True)
        self.model_path = str(self.get_parameter('model_path').value)
        self.wake_sound = str(self.get_parameter('wake_sound').value)
        self.auto_sound = str(self.get_parameter('auto_sound').value)
        self.audio_output_device = str(self.get_parameter('audio_output_device').value)
        self.use_array_wake = bool(self.get_parameter('use_array_wake').value)
        if not os.path.isfile(os.path.join(self.model_path, 'am', 'final.mdl')):
            raise RuntimeError(f'Vosk model is missing: {self.model_path}')

        self.audio = pyaudio.PyAudio()
        index = int(self.get_parameter('audio_device_index').value)
        if index < 0:
            inputs = [i for i in range(self.audio.get_device_count())
                      if self.audio.get_device_info_by_index(i).get('maxInputChannels', 0) > 0]
            array_inputs = [i for i in inputs
                            if 'xfmdpv0018' in self.audio.get_device_info_by_index(i).get('name', '').lower()]
            usb_inputs = [i for i in inputs
                          if 'usb' in self.audio.get_device_info_by_index(i).get('name', '').lower()]
            index = (array_inputs or usb_inputs or inputs or [-1])[0]
        if index < 0:
            raise RuntimeError('no microphone input device found')

        self.stream = None
        for rate, channels in ((16000, 1), (48000, 1), (44100, 1),
                               (48000, 2), (44100, 2)):
            try:
                self.stream = self.audio.open(format=pyaudio.paInt16, channels=channels,
                                              rate=rate, input=True, input_device_index=index,
                                              frames_per_buffer=2048, start=False)
                self.input_rate = rate
                self.channels = channels
                break
            except (OSError, ValueError):
                continue
        if self.stream is None:
            raise RuntimeError(f'cannot open microphone device {index}')
        self.resample_state = None
        self.recognizer = KaldiRecognizer(Model(self.model_path), 16000)
        self.audio_queue = queue.Queue(maxsize=40)
        self.running = True
        self.listening = False
        self.capture_thread = None
        self.stage = 'disabled'
        self.stage_deadline = None
        self.auto_requested = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.lock_pub = self.create_publisher(Bool, '/competition_manual/auto_requested', qos)
        self.create_subscription(Bool, '/competition_manual/voice_enable',
                                 self.on_voice_enable, qos)
        if self.use_array_wake:
            self.create_subscription(Bool, '/awake_node/awake_flag', self.on_array_wake, 10)
        self.create_timer(0.05, self.process_audio)
        wake_source = ('array /awake_node/awake_flag + Vosk fallback'
                       if self.use_array_wake else 'Vosk 徐总')
        self.get_logger().info(
            f'voice ready: microphone {index}, {self.input_rate} Hz, '
            f'{self.channels} channel(s) -> 16000 Hz mono; '
            f'waiting for Start; wake source: {wake_source}')

    def on_voice_enable(self, message):
        if not message.data or self.listening or self.auto_requested:
            return
        self.clear_audio()
        self.recognizer.Reset()
        self.resample_state = None
        try:
            self.stream.start_stream()
        except OSError as error:
            self.get_logger().error(f'cannot start microphone: {error}')
            return
        self.listening = True
        self.stage = 'waiting_wake'
        self.stage_deadline = time.monotonic() + STAGE_TIMEOUT_SEC
        self.capture_thread = threading.Thread(target=self.capture, daemon=True)
        self.capture_thread.start()
        self.get_logger().info('Start pressed: waiting 5 seconds for 徐总')

    def clear_audio(self):
        while True:
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                return

    def enter_auto_wait(self, reason):
        if self.stage != 'waiting_wake':
            return
        self.stage = 'waiting_auto'
        self.stage_deadline = time.monotonic() + STAGE_TIMEOUT_SEC
        self.clear_audio()
        self.recognizer.Reset()
        self.get_logger().info(f'{reason}: 干嘛; waiting 5 seconds for 自动模式')
        self.play(self.wake_sound)

    def complete_auto(self, reason):
        if self.stage != 'waiting_auto' or self.auto_requested:
            return
        self.stage = 'done'
        self.stage_deadline = None
        self.auto_requested = True
        self.listening = False
        self.lock_pub.publish(Bool(data=True))
        self.get_logger().warn(f'{reason}: 好的; gamepad locked; timed forward requested')
        self.play(self.auto_sound)

    def check_timeout(self):
        if not self.listening or self.stage_deadline is None:
            return
        if time.monotonic() < self.stage_deadline:
            return
        if self.stage == 'waiting_wake':
            self.enter_auto_wait('徐总 timeout')
        elif self.stage == 'waiting_auto':
            self.complete_auto('自动模式 timeout')

    def on_array_wake(self, message):
        if not message.data or not self.listening or self.stage != 'waiting_wake':
            return
        self.enter_auto_wait('array wake word: 徐总')

    def capture(self):
        while self.running and self.listening:
            try:
                data = self.stream.read(2048, exception_on_overflow=False)
                if self.channels == 2:
                    data = audioop.tomono(data, 2, 0.5, 0.5)
                if self.input_rate != 16000:
                    data, self.resample_state = audioop.ratecv(
                        data, 2, 1, self.input_rate, 16000, self.resample_state)
                if self.audio_queue.full():
                    self.audio_queue.get_nowait()
                self.audio_queue.put_nowait(data)
            except (OSError, queue.Empty, queue.Full):
                time.sleep(0.05)
        try:
            if self.stream.is_active():
                self.stream.stop_stream()
        except OSError as error:
            self.get_logger().warn(f'cannot stop microphone: {error}')

    def play(self, path):
        if not os.path.isfile(path):
            self.get_logger().error(f'audio file missing: {path}')
            return
        try:
            subprocess.Popen(['aplay', '-q', '-D', self.audio_output_device, path],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except OSError as error:
            self.get_logger().error(f'cannot play audio: {error}')

    def process_text(self, text, partial=False):
        words = text.replace(' ', '')
        if words and not partial:
            self.get_logger().info(f'recognized: {words}')
        if not words or not self.listening or self.auto_requested:
            return
        if self.stage == 'waiting_wake' and '徐总' in words:
            self.enter_auto_wait('Vosk wake word: 徐总')
        elif self.stage == 'waiting_auto' and '自动模式' in words:
            self.complete_auto('voice command: 自动模式')

    def process_audio(self):
        if not self.listening:
            return
        for _ in range(8):
            try:
                data = self.audio_queue.get_nowait()
            except queue.Empty:
                break
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                self.process_text(result.get('text', ''))
            else:
                result = json.loads(self.recognizer.PartialResult())
                self.process_text(result.get('partial', ''), partial=True)
            if not self.listening:
                return
        self.check_timeout()

    def destroy_node(self):
        self.running = False
        self.listening = False
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=1.0)
        if self.stream is not None:
            if self.stream.is_active():
                self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
