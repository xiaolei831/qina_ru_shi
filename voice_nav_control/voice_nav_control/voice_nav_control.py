import json
import math
import time
from typing import Callable, Dict, Optional

import rclpy
import serial
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class SpeechSerial:
    def __init__(self, device: str, baud_rate: int) -> None:
        self._serial = serial.Serial(device, baud_rate, timeout=0.05)
        self._rx_buffer = bytearray()

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def read_frame(self) -> Optional[Dict[str, int]]:
        count = self._serial.in_waiting
        if count > 0:
            self._rx_buffer.extend(self._serial.read(count))

        while len(self._rx_buffer) >= 2:
            header_index = self._rx_buffer.find(b'\xaa\x55')
            if header_index < 0:
                last_byte = self._rx_buffer[-1:]
                self._rx_buffer.clear()
                if last_byte == b'\xaa':
                    self._rx_buffer.extend(last_byte)
                return None

            if header_index > 0:
                del self._rx_buffer[:header_index]

            if len(self._rx_buffer) < 5:
                return None

            if self._rx_buffer[4] != 0xFB:
                del self._rx_buffer[0]
                continue

            frame = {
                'type': int(self._rx_buffer[2]),
                'code': int(self._rx_buffer[3]),
            }
            del self._rx_buffer[:5]
            return frame

        return None

    def voice_write(self, broadcast_code: int) -> None:
        command = bytes([0xAA, 0x55, 0xFF, int(broadcast_code), 0xFB])
        self._serial.write(command)
        time.sleep(0.005)
        self._serial.reset_input_buffer()


class VoiceNavControlNode(Node):
    def __init__(self, node_name: str = 'voice_nav_control') -> None:
        super().__init__(node_name)

        self.declare_parameter('speech_port', '/dev/ttyUSB1')
        self.declare_parameter('speech_baud_rate', 115200)
        self.declare_parameter('poll_period_sec', 0.1)
        self.declare_parameter('command_cooldown_sec', 1.0)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('task_status_topic', '/voice_nav_task')
        self.declare_parameter('delivery_task_status_topic', '/medicine_delivery_task')
        self.declare_parameter('delivery_record_topic', '/medicine_delivery_record')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pickup_wait_sec', 1.0)
        self.declare_parameter('delivery_complete_wait_sec', 5.0)
        self.declare_parameter('task_complete_hold_sec', 3.0)
        self.declare_parameter('pick_timeout_sec', 180.0)
        self.declare_parameter('pick_command_repeat_sec', 2.0)
        self.declare_parameter('place_timeout_sec', 30.0)
        self.declare_parameter('place_command_delay_sec', 0.2)
        self.declare_parameter('return_rest_area_after_delivery', True)
        self.declare_parameter('enable_voice_broadcast', True)
        self.declare_parameter('medicine_pick_enable_topic', '/medicine_pick_enable')
        self.declare_parameter('target_medicine_topic', '/target_medicine')
        self.declare_parameter('medicine_pick_state_topic', '/medicine_pick_state')
        self.declare_parameter('arm_state_topic', '/arm_state')
        self.declare_parameter('medicine_place_command', 'rotate_put')
        self.declare_parameter('arm_phase_topic', '/arm_phase')

        self.declare_parameter('code_room_101', 19)
        self.declare_parameter('code_room_102', 20)
        self.declare_parameter('code_room_103', 21)
        self.declare_parameter('code_room_104', 32)
        self.declare_parameter('code_nav_room_101', 10)
        self.declare_parameter('code_nav_room_102', 11)
        self.declare_parameter('code_nav_room_103', 12)
        self.declare_parameter('code_nav_room_104', 13)
        self.declare_parameter('code_rest_area', 33)
        self.declare_parameter('code_medicine_lianhua', 14)
        self.declare_parameter('code_medicine_ointment', 16)
        self.declare_parameter('code_medicine_vitamin', 15)
        self.declare_parameter('code_medicine_yinchi', 17)

        self.declare_parameter('broadcast_ask_medicine', 0)
        self.declare_parameter('broadcast_arrived_pharmacy', 61)
        self.declare_parameter('broadcast_picked_medicine_a', 62)
        self.declare_parameter('broadcast_start_pick', 0)
        self.declare_parameter('broadcast_pick_failed', 0)
        self.declare_parameter('broadcast_start_room_101', 63)
        self.declare_parameter('broadcast_start_room_102', 64)
        self.declare_parameter('broadcast_start_room_103', 66)
        self.declare_parameter('broadcast_start_room_104', 67)
        self.declare_parameter('broadcast_arrived_rest_area', 68)
        self.declare_parameter('broadcast_arrived_room_101', 73)
        self.declare_parameter('broadcast_arrived_room_102', 88)
        self.declare_parameter('broadcast_arrived_room_103', 94)
        self.declare_parameter('broadcast_arrived_room_104', 95)
        self.declare_parameter('broadcast_nav_arrived_room_101', 35)
        self.declare_parameter('broadcast_nav_arrived_room_102', 36)
        self.declare_parameter('broadcast_nav_arrived_room_103', 37)
        self.declare_parameter('broadcast_nav_arrived_room_104', 38)

        self.declare_parameter('medicine_lianhua.name', '莲花清瘟胶囊')
        self.declare_parameter('medicine_lianhua.target', '莲花清瘟胶囊')
        self.declare_parameter('medicine_lianhua.broadcast_selected', 0)
        self.declare_parameter('medicine_ointment.name', '乳膏')
        self.declare_parameter('medicine_ointment.target', '乳膏')
        self.declare_parameter('medicine_ointment.broadcast_selected', 0)
        self.declare_parameter('medicine_vitamin.name', '维生素胶囊')
        self.declare_parameter('medicine_vitamin.target', '维生素胶囊')
        self.declare_parameter('medicine_vitamin.broadcast_selected', 0)
        self.declare_parameter('medicine_yinchi.name', '银翘解毒片')
        self.declare_parameter('medicine_yinchi.target', '银翘解毒片')
        self.declare_parameter('medicine_yinchi.broadcast_selected', 0)

        for prefix in (
            'pharmacy',
            'rest_area',
            'room_101_door',
            'room_102_door',
            'room_103_door',
            'room_104_door',
            'room_101_delivery',
            'room_102_delivery',
            'room_103_delivery',
            'room_104_delivery',
            'room_101',
            'room_102',
            'room_103',
            'room_104',
            'point_1',
            'point_2',
            'point_3',
            'point_4',
        ):
            self.declare_parameter(f'{prefix}.x', 0.0)
            self.declare_parameter(f'{prefix}.y', 0.0)
            self.declare_parameter(f'{prefix}.yaw', 0.0)

        self.speech_port = self.get_str('speech_port')
        self.speech_baud_rate = self.get_int('speech_baud_rate')
        poll_period_sec = self.get_float('poll_period_sec')
        self.command_cooldown_sec = self.get_float('command_cooldown_sec')
        self.map_frame = self.get_str('map_frame')
        self.pickup_wait_sec = self.get_float('pickup_wait_sec')
        self.delivery_complete_wait_sec = self.get_float('delivery_complete_wait_sec')
        self.task_complete_hold_sec = self.get_float('task_complete_hold_sec')
        self.pick_timeout_sec = self.get_float('pick_timeout_sec')
        self.pick_command_repeat_sec = self.get_float('pick_command_repeat_sec')
        self.place_timeout_sec = self.get_float('place_timeout_sec')
        self.place_command_delay_sec = self.get_float('place_command_delay_sec')
        self.medicine_place_command = self.get_str('medicine_place_command')
        self.return_rest_area_after_delivery = self.get_bool('return_rest_area_after_delivery')
        self.enable_voice_broadcast = self.get_bool('enable_voice_broadcast')

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            self.get_str('goal_topic'),
            10,
        )
        task_status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.task_status_publishers = [
            self.create_publisher(String, topic, task_status_qos)
            for topic in self.unique_topics([
                self.get_str('task_status_topic'),
                self.get_str('delivery_task_status_topic'),
            ])
        ]
        self.delivery_record_publisher = self.create_publisher(
            String,
            self.get_str('delivery_record_topic'),
            10,
        )
        self.pick_enable_publisher = self.create_publisher(
            Bool,
            self.get_str('medicine_pick_enable_topic'),
            10,
        )
        self.target_medicine_publisher = self.create_publisher(
            String,
            self.get_str('target_medicine_topic'),
            10,
        )
        self.arm_state_publisher = self.create_publisher(
            String,
            self.get_str('arm_state_topic'),
            10,
        )
        self.pick_state_subscription = self.create_subscription(
            String,
            self.get_str('medicine_pick_state_topic'),
            self.on_pick_state,
            10,
        )
        self.arm_phase_subscription = self.create_subscription(
            String,
            self.get_str('arm_phase_topic'),
            self.on_arm_phase,
            10,
        )
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.get_str('nav_action_name'),
        )

        self.pharmacy_pose = self.load_pose('pharmacy')
        self.rest_area_pose = self.load_pose('rest_area')
        self.delivery_commands = self.build_delivery_commands()
        self.direct_navigation_commands = self.build_direct_navigation_commands()
        self.medicine_commands = self.build_medicine_commands()
        self.rest_area_code = self.get_int('code_rest_area')

        self.last_command_code: Optional[int] = None
        self.last_command_time = self.get_clock().now() - Duration(seconds=60)
        self.speech_serial: Optional[SpeechSerial] = None
        self.last_serial_open_attempt = 0.0
        self.active_goal_handle = None
        self.current_task = ''
        self.task_token = 0
        self.task_status_seq = 0
        self.pending_delivery: Optional[Dict[str, object]] = None
        self.pick_waiting_token: Optional[int] = None
        self.pick_waiting_delivery: Optional[Dict[str, object]] = None
        self.pick_waiting_medicine: Optional[Dict[str, object]] = None
        self.pick_deadline = None
        self.last_pick_command_publish_time = 0.0
        self.place_waiting_token: Optional[int] = None
        self.place_waiting_delivery: Optional[Dict[str, object]] = None
        self.place_waiting_medicine: Optional[Dict[str, object]] = None
        self.place_deadline = None
        self.current_task_info: Dict[str, object] = self.make_idle_task_status()
        self.delivery_record_tokens = set()

        self.ensure_speech_serial(force=True)
        self.timer = self.create_timer(poll_period_sec, self.poll_speech_device)
        self.pick_watch_timer = self.create_timer(0.5, self.check_pick_timeout)
        self.publish_pick_enable(False)
        self.publish_task_status()
        self.get_logger().info('voice navigation control node started')

    def destroy_node(self):
        if self.speech_serial is not None:
            self.speech_serial.close()
        return super().destroy_node()

    def get_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def get_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def get_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def get_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    @staticmethod
    def unique_topics(topics) -> list:
        result = []
        seen = set()
        for topic in topics:
            topic = str(topic).strip()
            if not topic or topic in seen:
                continue
            seen.add(topic)
            result.append(topic)
        return result

    def make_idle_task_status(self) -> Dict[str, object]:
        return {
            'active': False,
            'status': 'idle',
            'state': '待分配',
            'target': '护士站',
            'target_room': '',
            'medicine': '暂无',
            'medicine_id': '',
            'task_id': '--',
            'task_type': 'idle',
            'task_name': '',
            'stage': 'idle',
            'current_step': '等待语音任务',
            'step_index': 0,
            'total_steps': 0,
            'progress': 0.0,
            'route': [],
            'pick_state': 'idle',
            'arm_phase': 'idle',
            'command_code': 0,
            'room_command_code': 0,
            'medicine_command_code': 0,
            'message': '等待语音任务',
            'error': '',
            'started_at': 0.0,
            'finished_at': 0.0,
            'seq': 0,
            'updated_at': time.time(),
        }

    def enrich_task_status(self) -> None:
        stage = str(self.current_task_info.get('stage', 'idle'))
        task_type = str(self.current_task_info.get('task_type', 'idle'))
        if stage == 'failed':
            self.current_task_info['status'] = 'failed'
            self.current_task_info['current_step'] = '任务异常'
            return
        if stage == 'completed':
            self.current_task_info['status'] = 'completed'
            self.current_task_info['progress'] = 1.0
            self.current_task_info['current_step'] = '任务完成'
            if not self.current_task_info.get('finished_at'):
                self.current_task_info['finished_at'] = time.time()
            return

        if task_type == 'delivery':
            if stage == 'room_arrived':
                step_key = 'room_arrived'
            elif stage.startswith('room_door_'):
                step_key = 'room_door'
            elif stage.startswith('room_inside_'):
                step_key = 'room_inside'
            else:
                step_key = stage
            steps = [
                ('await_medicine', '等待药品名称'),
                ('pharmacy', '导航到药房'),
                ('pharmacy_arrived', '到达药房并播报'),
                ('picking', '视觉抓取药品'),
                ('room_door', '导航到病房门口'),
                ('room_inside', '进入病房'),
                ('room_arrived', '到达病房内并播报'),
                ('placing', '机械臂放下药品'),
                ('placing_done', '药品已放下'),
            ]
            if self.return_rest_area_after_delivery:
                steps.append(('rest_area', '返回休息区'))
            self.apply_step_progress(step_key, steps)
            return

        if task_type == 'navigation':
            step_key = 'nav_room_door' if stage.startswith('nav_room_door_') else stage
            step_key = 'navigation' if step_key.startswith('nav_room_inside_') else step_key
            self.apply_step_progress(
                step_key,
                [('nav_room_door', '导航到病房门口'), ('navigation', '进入目标病房')],
            )
            return

        if task_type == 'return_rest_area':
            self.apply_step_progress(stage, [('rest_area', '返回休息区')])
            return

        self.current_task_info['status'] = 'idle'
        self.current_task_info['current_step'] = '等待语音任务'
        self.current_task_info['step_index'] = 0
        self.current_task_info['total_steps'] = 0
        self.current_task_info['progress'] = 0.0

    def apply_step_progress(self, step_key: str, steps) -> None:
        total_steps = len(steps)
        index = 0
        label = str(self.current_task_info.get('state', '执行中'))
        for i, (key, step_label) in enumerate(steps, start=1):
            if key == step_key:
                index = i
                label = step_label
                break

        self.current_task_info['status'] = 'running'
        self.current_task_info['current_step'] = label
        self.current_task_info['step_index'] = index
        self.current_task_info['total_steps'] = total_steps
        self.current_task_info['progress'] = round(index / total_steps, 3) if index else 0.0

    def publish_task_status(self) -> None:
        self.enrich_task_status()
        self.task_status_seq += 1
        self.current_task_info['seq'] = self.task_status_seq
        self.current_task_info['updated_at'] = time.time()
        msg = String()
        msg.data = json.dumps(self.current_task_info, ensure_ascii=False, separators=(',', ':'))
        for publisher in self.task_status_publishers:
            publisher.publish(msg)

    def update_task_status(self, token: int, **fields: object) -> None:
        if not self.is_current_task(token):
            return
        self.current_task_info.update(fields)
        self.publish_task_status()

    def fail_task(self, token: int, message: str) -> None:
        if not self.is_current_task(token):
            return
        self.current_task_info.update({
            'active': False,
            'status': 'failed',
            'state': '异常',
            'stage': 'failed',
            'pick_state': 'failed',
            'message': message,
            'error': message,
            'finished_at': time.time(),
        })
        self.publish_task_status()
        self.publish_pick_enable(False)
        self.current_task = ''
        self.active_goal_handle = None
        self.pending_delivery = None
        self.clear_pick_wait()
        self.clear_place_wait()
        self.publish_arm_state('no_msg')

    def load_pose(self, prefix: str, *fallback_prefixes: str) -> Dict[str, float]:
        pose = {
            'x': self.get_float(f'{prefix}.x'),
            'y': self.get_float(f'{prefix}.y'),
            'yaw': self.get_float(f'{prefix}.yaw'),
        }
        if not self.pose_is_zero(pose):
            return pose

        for fallback_prefix in fallback_prefixes:
            fallback = {
                'x': self.get_float(f'{fallback_prefix}.x'),
                'y': self.get_float(f'{fallback_prefix}.y'),
                'yaw': self.get_float(f'{fallback_prefix}.yaw'),
            }
            if not self.pose_is_zero(fallback):
                return fallback

        return pose

    @staticmethod
    def pose_is_zero(pose: Dict[str, float]) -> bool:
        return (
            abs(pose['x']) < 1e-9 and
            abs(pose['y']) < 1e-9 and
            abs(pose['yaw']) < 1e-9
        )

    def build_delivery_commands(self) -> Dict[int, Dict[str, object]]:
        return {
            self.get_int('code_room_101'): {
                'room': '101',
                'door_pose': self.load_pose('room_101_door', 'room_101', 'point_1'),
                'pose': self.load_pose('room_101_delivery', 'room_101', 'point_1'),
                'start_broadcast': self.get_int('broadcast_start_room_101'),
                'arrived_broadcast': self.get_int('broadcast_arrived_room_101'),
            },
            self.get_int('code_room_102'): {
                'room': '102',
                'door_pose': self.load_pose('room_102_door', 'room_102', 'point_2'),
                'pose': self.load_pose('room_102_delivery', 'room_102', 'point_2'),
                'start_broadcast': self.get_int('broadcast_start_room_102'),
                'arrived_broadcast': self.get_int('broadcast_arrived_room_102'),
            },
            self.get_int('code_room_103'): {
                'room': '103',
                'door_pose': self.load_pose('room_103_door', 'room_103', 'point_3'),
                'pose': self.load_pose('room_103_delivery', 'room_103', 'point_3'),
                'start_broadcast': self.get_int('broadcast_start_room_103'),
                'arrived_broadcast': self.get_int('broadcast_arrived_room_103'),
            },
            self.get_int('code_room_104'): {
                'room': '104',
                'door_pose': self.load_pose('room_104_door', 'room_104', 'point_4'),
                'pose': self.load_pose('room_104_delivery', 'room_104', 'point_4'),
                'start_broadcast': self.get_int('broadcast_start_room_104'),
                'arrived_broadcast': self.get_int('broadcast_arrived_room_104'),
            },
        }

    def build_direct_navigation_commands(self) -> Dict[int, Dict[str, object]]:
        return {
            self.get_int('code_nav_room_101'): {
                'room': '101',
                'door_pose': self.load_pose('room_101_door', 'room_101', 'point_1'),
                'pose': self.load_pose('room_101_delivery', 'room_101', 'point_1'),
                'arrived_broadcast': self.get_int('broadcast_nav_arrived_room_101'),
            },
            self.get_int('code_nav_room_102'): {
                'room': '102',
                'door_pose': self.load_pose('room_102_door', 'room_102', 'point_2'),
                'pose': self.load_pose('room_102_delivery', 'room_102', 'point_2'),
                'arrived_broadcast': self.get_int('broadcast_nav_arrived_room_102'),
            },
            self.get_int('code_nav_room_103'): {
                'room': '103',
                'door_pose': self.load_pose('room_103_door', 'room_103', 'point_3'),
                'pose': self.load_pose('room_103_delivery', 'room_103', 'point_3'),
                'arrived_broadcast': self.get_int('broadcast_nav_arrived_room_103'),
            },
            self.get_int('code_nav_room_104'): {
                'room': '104',
                'door_pose': self.load_pose('room_104_door', 'room_104', 'point_4'),
                'pose': self.load_pose('room_104_delivery', 'room_104', 'point_4'),
                'arrived_broadcast': self.get_int('broadcast_nav_arrived_room_104'),
            },
        }

    def build_medicine_commands(self) -> Dict[int, Dict[str, object]]:
        medicines = {
            self.get_int('code_medicine_lianhua'): {
                'name': self.get_str('medicine_lianhua.name'),
                'target': self.get_str('medicine_lianhua.target'),
                'broadcast_selected': self.get_int('medicine_lianhua.broadcast_selected'),
            },
            self.get_int('code_medicine_ointment'): {
                'name': self.get_str('medicine_ointment.name'),
                'target': self.get_str('medicine_ointment.target'),
                'broadcast_selected': self.get_int('medicine_ointment.broadcast_selected'),
            },
            self.get_int('code_medicine_vitamin'): {
                'name': self.get_str('medicine_vitamin.name'),
                'target': self.get_str('medicine_vitamin.target'),
                'broadcast_selected': self.get_int('medicine_vitamin.broadcast_selected'),
            },
            self.get_int('code_medicine_yinchi'): {
                'name': self.get_str('medicine_yinchi.name'),
                'target': self.get_str('medicine_yinchi.target'),
                'broadcast_selected': self.get_int('medicine_yinchi.broadcast_selected'),
            },
        }
        return {
            int(code): medicine
            for code, medicine in medicines.items()
            if int(code) > 0 and str(medicine['target']).strip()
        }

    def ensure_speech_serial(self, force: bool = False) -> bool:
        if self.speech_serial is not None:
            return True

        now = time.monotonic()
        if not force and now - self.last_serial_open_attempt < 3.0:
            return False

        self.last_serial_open_attempt = now
        try:
            self.speech_serial = SpeechSerial(self.speech_port, self.speech_baud_rate)
            self.get_logger().info(f'opened speech serial {self.speech_port}')
            return True
        except serial.SerialException as exc:
            self.get_logger().error(f'failed to open speech serial {self.speech_port}: {exc}')
            return False

    def poll_speech_device(self) -> None:
        if not self.ensure_speech_serial():
            return

        try:
            frame = self.speech_serial.read_frame()
        except serial.SerialException as exc:
            self.get_logger().error(f'speech serial read failed: {exc}')
            self.speech_serial.close()
            self.speech_serial = None
            return

        if frame is None:
            return

        frame_type = int(frame['type'])
        command_code = int(frame['code'])
        if frame_type != 0x00:
            self.get_logger().debug(
                f'ignore non-command speech frame type=0x{frame_type:02X} code={command_code}'
            )
            return

        now = self.get_clock().now()
        if (
            self.last_command_code == command_code and
            (now - self.last_command_time).nanoseconds / 1e9 < self.command_cooldown_sec
        ):
            return

        self.last_command_code = command_code
        self.last_command_time = now
        self.handle_command(command_code)

    def handle_command(self, command_code: int) -> None:
        medicine = self.medicine_commands.get(command_code)
        if medicine is not None:
            self.handle_medicine_command(medicine, command_code)
            return

        delivery = self.delivery_commands.get(command_code)
        if delivery is not None:
            self.start_delivery_request(delivery, command_code)
            return

        navigation = self.direct_navigation_commands.get(command_code)
        if navigation is not None:
            self.start_direct_room_navigation(navigation, command_code)
            return

        if command_code == self.rest_area_code:
            self.start_return_rest_area(command_code)
            return

        self.get_logger().warning(f'unmapped speech command code: {command_code}')

    def start_delivery_request(self, delivery: Dict[str, object], command_code: int) -> None:
        room = str(delivery['room'])
        token = self.begin_task(
            f'delivery_room_{room}',
            task_type='delivery',
            target=f'{room}病房',
            target_room=room,
            medicine='待确认',
            medicine_id='',
            command_code=command_code,
            state='等待药品名称',
            stage='await_medicine',
            route=['药房', f'{room}病房门口', f'{room}病房', '休息区'] if self.return_rest_area_after_delivery
            else ['药房', f'{room}病房门口', f'{room}病房'],
        )
        self.pending_delivery = {
            'delivery': delivery,
            'token': token,
            'room_command_code': int(command_code),
        }
        self.publish_pick_enable(False)
        self.broadcast(self.get_int('broadcast_ask_medicine'))
        self.update_task_status(
            token,
            message=f'已选择{room}病房，等待语音选择药品',
        )
        self.get_logger().info(
            f'received room {room} delivery command {command_code}, wait medicine command'
        )

    def handle_medicine_command(self, medicine: Dict[str, object], command_code: int) -> None:
        if self.pending_delivery is None:
            if self.current_task:
                self.get_logger().warning(
                    f'ignore medicine command {command_code}; current task is not awaiting medicine'
                )
                return
            self.get_logger().warning(
                f'received medicine command {command_code} without pending delivery room'
            )
            self.current_task_info = self.make_idle_task_status()
            self.current_task_info['message'] = '请先说目标病房，再说药品名称'
            self.publish_task_status()
            self.broadcast(self.get_int('broadcast_ask_medicine'))
            return

        token = int(self.pending_delivery['token'])
        if not self.is_current_task(token):
            return

        delivery = self.pending_delivery['delivery']
        self.pending_delivery = None
        room = str(delivery['room'])
        medicine_name = str(medicine['name'])
        medicine_target = str(medicine['target'])
        self.update_task_status(
            token,
            state='前往药房',
            stage='pharmacy',
            target='药房',
            target_room=room,
            medicine=medicine_name,
            medicine_id=medicine_target,
            command_code=int(command_code),
            medicine_command_code=int(command_code),
            pick_state='idle',
            message=f'已确认药品：{medicine_name}，前往药房',
        )
        self.broadcast(int(medicine.get('broadcast_selected', 0)))
        self.get_logger().info(
            f'received medicine command {command_code}: {medicine_name} -> {medicine_target}; go pharmacy'
        )
        self.navigate_to(
            self.pharmacy_pose,
            token,
            'pharmacy',
            lambda: self.on_arrived_pharmacy(delivery, medicine, token),
        )

    def start_direct_room_navigation(self, navigation: Dict[str, object], command_code: int) -> None:
        self.pending_delivery = None
        self.clear_pick_wait()
        self.clear_place_wait()
        self.publish_pick_enable(False)
        room = str(navigation['room'])
        token = self.begin_task(
            f'navigate_room_{room}',
            task_type='navigation',
            target=f'{room}病房',
            target_room=room,
            medicine='暂无',
            medicine_id='',
            command_code=command_code,
            state='导航中',
            stage=f'nav_room_door_{room}',
            route=[f'{room}病房门口', f'{room}病房'],
        )
        self.update_task_status(
            token,
            message=f'正在导航到{room}病房门口',
        )
        self.get_logger().info(f'received direct navigation command {command_code}: room {room}')
        self.navigate_to(
            navigation['door_pose'],  # type: ignore[arg-type]
            token,
            f'nav_room_door_{room}',
            lambda: self.on_arrived_direct_room_door(navigation, token),
        )

    def on_arrived_direct_room_door(self, navigation: Dict[str, object], token: int) -> None:
        if not self.is_current_task(token):
            return

        room = str(navigation['room'])
        self.update_task_status(
            token,
            state='进入病房',
            stage=f'nav_room_inside_{room}',
            target=f'{room}病房',
            target_room=room,
            message=f'已到达{room}病房门口，继续进入病房',
        )
        self.get_logger().info(f'direct navigation arrived room {room} door, entering room')
        self.navigate_to(
            navigation['pose'],  # type: ignore[arg-type]
            token,
            f'nav_room_inside_{room}',
            lambda: self.on_arrived_direct_room(navigation, token),
        )

    def on_arrived_direct_room(self, navigation: Dict[str, object], token: int) -> None:
        if not self.is_current_task(token):
            return

        room = str(navigation['room'])
        self.update_task_status(
            token,
            active=False,
            state='已完成',
            stage='completed',
            target=f'{room}病房',
            target_room=room,
            message=f'已到达{room}病房',
        )
        self.broadcast(int(navigation['arrived_broadcast']))
        self.get_logger().info(f'direct navigation arrived room {room}')
        self.call_later(self.task_complete_hold_sec, lambda: self.finish_task(token))

    def start_return_rest_area(self, command_code: int) -> None:
        self.pending_delivery = None
        self.clear_pick_wait()
        self.clear_place_wait()
        self.publish_pick_enable(False)
        token = self.begin_task(
            'return_rest_area',
            task_type='return_rest_area',
            target='休息区',
            target_room='',
            medicine='暂无',
            medicine_id='',
            command_code=command_code,
            state='返回休息区',
            stage='rest_area',
            route=['休息区'],
        )
        self.get_logger().info(f'received return rest area command {command_code}')
        self.navigate_to(
            self.rest_area_pose,
            token,
            'rest_area',
            lambda: self.on_arrived_rest_area(token),
        )

    def begin_task(
        self,
        name: str,
        task_type: str,
        target: str,
        target_room: str,
        medicine: str,
        medicine_id: str,
        command_code: int,
        state: str,
        stage: str,
        route=None,
    ) -> int:
        self.task_token += 1
        self.current_task = name
        self.cancel_active_goal()
        self.clear_pick_wait()
        self.clear_place_wait()
        self.publish_arm_state('no_msg')
        self.current_task_info = {
            'active': True,
            'status': 'running',
            'state': state,
            'target': target,
            'target_room': target_room,
            'medicine': medicine,
            'medicine_id': medicine_id,
            'task_id': str(self.task_token),
            'task_type': task_type,
            'task_name': name,
            'stage': stage,
            'current_step': state,
            'step_index': 0,
            'total_steps': 0,
            'progress': 0.0,
            'route': list(route or []),
            'pick_state': 'idle',
            'arm_phase': 'idle',
            'command_code': int(command_code),
            'room_command_code': int(command_code) if task_type == 'delivery' else 0,
            'medicine_command_code': 0,
            'message': '',
            'error': '',
            'started_at': time.time(),
            'finished_at': 0.0,
        }
        self.publish_task_status()
        return self.task_token

    def is_current_task(self, token: int) -> bool:
        return token == self.task_token and self.current_task != ''

    def finish_task(self, token: int) -> None:
        if self.is_current_task(token):
            self.current_task = ''
            self.active_goal_handle = None
            self.pending_delivery = None
            self.clear_pick_wait()
            self.clear_place_wait()
            self.publish_pick_enable(False)
            self.publish_arm_state('no_msg')
            self.current_task_info = self.make_idle_task_status()
            self.current_task_info['message'] = '任务完成，等待语音任务'
            self.publish_task_status()

    def navigate_to(
        self,
        target: Dict[str, float],
        token: int,
        stage: str,
        on_success: Callable[[], None],
    ) -> None:
        if self.pose_is_zero(target):
            self.get_logger().warning(f'{stage} target is still all zeros; calibrate params first')

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            message = 'navigate_to_pose action server is unavailable'
            self.get_logger().error(message)
            self.fail_task(token, message)
            return

        pose = self.make_pose(target)
        self.goal_publisher.publish(pose)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result_future: self.on_goal_response(result_future, token, stage, on_success)
        )
        self.get_logger().info(
            f'send navigation stage={stage} x={target["x"]:.3f} y={target["y"]:.3f} '
            f'yaw={target["yaw"]:.3f}'
        )

    def make_pose(self, target: Dict[str, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        # Use latest available TF. Stamping with "now" can be a few milliseconds
        # ahead of the TF buffer on this robot and make Nav2 abort immediately.
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = float(target['x'])
        pose.pose.position.y = float(target['y'])
        pose.pose.orientation.z = math.sin(float(target['yaw']) / 2.0)
        pose.pose.orientation.w = math.cos(float(target['yaw']) / 2.0)
        return pose

    def on_goal_response(
        self,
        future,
        token: int,
        stage: str,
        on_success: Callable[[], None],
    ) -> None:
        if not self.is_current_task(token):
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            message = f'navigation goal response failed for {stage}: {exc}'
            self.get_logger().error(message)
            self.fail_task(token, message)
            return

        if goal_handle is None or not goal_handle.accepted:
            message = f'navigation goal rejected for {stage}'
            self.get_logger().warning(message)
            self.fail_task(token, message)
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self.on_goal_result(done_future, token, stage, on_success)
        )
        self.get_logger().info(f'navigation goal accepted for {stage}')

    def on_goal_result(
        self,
        future,
        token: int,
        stage: str,
        on_success: Callable[[], None],
    ) -> None:
        if not self.is_current_task(token):
            return

        try:
            result = future.result()
        except Exception as exc:
            message = f'navigation result failed for {stage}: {exc}'
            self.get_logger().error(message)
            self.fail_task(token, message)
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'navigation stage succeeded: {stage}')
            on_success()
        else:
            message = f'navigation stage {stage} ended with status {result.status}'
            self.get_logger().warning(message)
            self.fail_task(token, message)

    def cancel_active_goal(self) -> None:
        if self.active_goal_handle is None:
            return

        cancel_future = self.active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.on_cancel_done)
        self.active_goal_handle = None

    def on_cancel_done(self, future) -> None:
        response = future.result()
        if response is not None:
            self.get_logger().info('previous navigation goal cancel requested')

    def on_arrived_pharmacy(
        self,
        delivery: Dict[str, object],
        medicine: Dict[str, object],
        token: int,
    ) -> None:
        if not self.is_current_task(token):
            return

        self.update_task_status(
            token,
            state='到达药房',
            stage='pharmacy_arrived',
            target='药房',
            message='已到达药房，准备启动视觉抓取',
        )
        self.broadcast(self.get_int('broadcast_arrived_pharmacy'))
        self.call_later(
            self.pickup_wait_sec,
            lambda: self.start_medicine_pick(delivery, medicine, token),
        )

    def start_medicine_pick(
        self,
        delivery: Dict[str, object],
        medicine: Dict[str, object],
        token: int,
    ) -> None:
        if not self.is_current_task(token):
            return

        medicine_name = str(medicine['name'])
        medicine_target = str(medicine['target'])
        self.pick_waiting_token = token
        self.pick_waiting_delivery = delivery
        self.pick_waiting_medicine = medicine
        self.pick_deadline = self.get_clock().now() + Duration(seconds=self.pick_timeout_sec)
        self.last_pick_command_publish_time = 0.0
        self.update_task_status(
            token,
            state='视觉抓取中',
            stage='picking',
            target='药房',
            target_room=str(delivery['room']),
            medicine=medicine_name,
            medicine_id=medicine_target,
            pick_state='picking',
            message=f'正在抓取{medicine_name}',
        )
        self.broadcast(self.get_int('broadcast_start_pick'))
        self.publish_pick_command(medicine_target, True)
        self.get_logger().info(
            f'start medicine pick target={medicine_target} room={delivery["room"]}'
        )

    def on_medicine_pick_complete(
        self,
        delivery: Dict[str, object],
        medicine: Dict[str, object],
        token: int,
    ) -> None:
        if not self.is_current_task(token):
            return

        self.publish_pick_enable(False)
        self.clear_pick_wait()
        medicine_name = str(medicine['name'])
        self.broadcast(self.get_int('broadcast_picked_medicine_a'))
        self.broadcast(int(delivery['start_broadcast']))
        self.update_task_status(
            token,
            state='配送中',
            stage=f'room_door_{delivery["room"]}',
            target=f'{delivery["room"]}病房门口',
            target_room=str(delivery['room']),
            medicine=medicine_name,
            medicine_id=str(medicine['target']),
            pick_state='pick_complete',
            message=f'{medicine_name}抓取完成，前往{delivery["room"]}病房门口',
        )
        self.navigate_to(
            delivery['door_pose'],  # type: ignore[arg-type]
            token,
            f'room_door_{delivery["room"]}',
            lambda: self.on_arrived_room_door(delivery, medicine, token),
        )

    def on_arrived_room_door(
        self,
        delivery: Dict[str, object],
        medicine: Dict[str, object],
        token: int,
    ) -> None:
        if not self.is_current_task(token):
            return

        medicine_name = str(medicine['name'])
        self.update_task_status(
            token,
            state='进入病房',
            stage=f'room_inside_{delivery["room"]}',
            target=f'{delivery["room"]}病房',
            target_room=str(delivery['room']),
            medicine=medicine_name,
            medicine_id=str(medicine['target']),
            pick_state='pick_complete',
            message=f'已到达{delivery["room"]}病房门口，继续进入病房',
        )
        self.get_logger().info(f'room {delivery["room"]} door arrived, entering room')
        self.navigate_to(
            delivery['pose'],  # type: ignore[arg-type]
            token,
            f'room_inside_{delivery["room"]}',
            lambda: self.on_arrived_room(delivery, medicine, token),
        )

    def on_arrived_room(
        self,
        delivery: Dict[str, object],
        medicine: Dict[str, object],
        token: int,
    ) -> None:
        if not self.is_current_task(token):
            return

        medicine_name = str(medicine['name'])
        self.update_task_status(
            token,
            state='已到达病房',
            stage='room_arrived',
            target=f'{delivery["room"]}病房',
            medicine=medicine_name,
            medicine_id=str(medicine['target']),
            message=f'{medicine_name}已送达{delivery["room"]}病房',
        )
        self.broadcast(int(delivery['arrived_broadcast']))
        self.get_logger().info(
            f'room {delivery["room"]} arrived, start medicine placement'
        )
        self.start_medicine_place(delivery, medicine, token)

    def start_medicine_place(
        self,
        delivery: Dict[str, object],
        medicine: Dict[str, object],
        token: int,
    ) -> None:
        if not self.is_current_task(token):
            return

        medicine_name = str(medicine['name'])
        self.place_waiting_token = token
        self.place_waiting_delivery = delivery
        self.place_waiting_medicine = medicine
        self.place_deadline = self.get_clock().now() + Duration(seconds=self.place_timeout_sec)
        self.update_task_status(
            token,
            state='放置药品',
            stage='placing',
            target=f'{delivery["room"]}病房',
            target_room=str(delivery['room']),
            medicine=medicine_name,
            medicine_id=str(medicine['target']),
            pick_state='placing',
            message=f'正在放下{medicine_name}',
        )
        self.publish_arm_state('no_msg')
        self.call_later(
            self.place_command_delay_sec,
            lambda: self.publish_place_command(token),
        )

    def publish_place_command(self, token: int) -> None:
        if not self.is_current_task(token) or self.place_waiting_token != token:
            return

        command = self.medicine_place_command.strip()
        if not command:
            self.on_medicine_place_complete(token)
            return

        self.publish_arm_state(command)
        self.get_logger().info(f'sent medicine place command: {command}')

    def on_medicine_place_complete(self, token: int) -> None:
        if not self.is_current_task(token):
            return
        delivery = self.place_waiting_delivery
        medicine = self.place_waiting_medicine
        if delivery is None or medicine is None:
            return

        medicine_name = str(medicine['name'])
        self.clear_place_wait()
        self.publish_delivery_record(delivery, medicine, token)
        self.update_task_status(
            token,
            state='药品已放下',
            stage='placing_done',
            target=f'{delivery["room"]}病房',
            target_room=str(delivery['room']),
            medicine=medicine_name,
            medicine_id=str(medicine['target']),
            pick_state='placed',
            message=f'{medicine_name}已放下，准备返回',
        )
        self.publish_arm_state('no_msg')
        if self.return_rest_area_after_delivery:
            self.call_later(
                self.delivery_complete_wait_sec,
                lambda: self.return_to_rest_area_after_delivery(token),
            )
        else:
            self.call_later(
                self.delivery_complete_wait_sec,
                lambda: self.complete_delivery_at_room(token),
            )

    def complete_delivery_at_room(self, token: int) -> None:
        if not self.is_current_task(token):
            return

        self.update_task_status(
            token,
            active=False,
            state='已完成',
            stage='completed',
            message='送药任务完成',
        )
        self.call_later(self.task_complete_hold_sec, lambda: self.finish_task(token))

    def publish_delivery_record(
        self,
        delivery: Dict[str, object],
        medicine: Dict[str, object],
        token: int,
    ) -> None:
        if token in self.delivery_record_tokens:
            return
        self.delivery_record_tokens.add(token)

        now = time.time()
        room = str(delivery['room'])
        medicine_name = str(medicine['name'])
        record = {
            'task_id': str(token),
            'task_type': 'delivery',
            'medicine': medicine_name,
            'medicine_id': str(medicine['target']),
            'room': room,
            'target_room': room,
            'status': '已放下药品',
            'state': '已放下药品',
            'message': f'{medicine_name}已送达{room}病房并放下',
            'room_command_code': int(self.current_task_info.get('room_command_code', 0)),
            'medicine_command_code': int(self.current_task_info.get('medicine_command_code', 0)),
            'delivered_at': now,
            'timestamp_ms': int(now * 1000),
        }
        msg = String()
        msg.data = json.dumps(record, ensure_ascii=False, separators=(',', ':'))
        self.delivery_record_publisher.publish(msg)
        self.get_logger().info(
            f'published delivery record: medicine={medicine_name} room={room} status=已放下药品'
        )

    def return_to_rest_area_after_delivery(self, token: int) -> None:
        if not self.is_current_task(token):
            return

        self.get_logger().info('returning to rest area after delivery')
        self.update_task_status(token, state='返回休息区', stage='rest_area', target='休息区')
        self.navigate_to(
            self.rest_area_pose,
            token,
            'rest_area',
            lambda: self.on_arrived_rest_area(token),
        )

    def on_arrived_rest_area(self, token: int) -> None:
        if not self.is_current_task(token):
            return

        self.update_task_status(
            token,
            active=False,
            state='已完成',
            stage='completed',
            target='休息区',
            message='送药任务完成，已返回休息区',
        )
        self.broadcast(self.get_int('broadcast_arrived_rest_area'))
        self.get_logger().info('returned to rest area')
        self.call_later(self.task_complete_hold_sec, lambda: self.finish_task(token))

    def publish_pick_enable(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.pick_enable_publisher.publish(msg)

    def publish_arm_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.arm_state_publisher.publish(msg)

    def publish_pick_command(self, medicine_target: str, enabled: bool) -> None:
        target_msg = String()
        target_msg.data = medicine_target
        self.target_medicine_publisher.publish(target_msg)
        self.publish_pick_enable(enabled)
        self.last_pick_command_publish_time = time.monotonic()

    def clear_pick_wait(self) -> None:
        self.pick_waiting_token = None
        self.pick_waiting_delivery = None
        self.pick_waiting_medicine = None
        self.pick_deadline = None
        self.last_pick_command_publish_time = 0.0

    def clear_place_wait(self) -> None:
        self.place_waiting_token = None
        self.place_waiting_delivery = None
        self.place_waiting_medicine = None
        self.place_deadline = None

    def check_pick_timeout(self) -> None:
        if self.pick_waiting_token is not None:
            token = int(self.pick_waiting_token)
            if not self.is_current_task(token):
                self.publish_pick_enable(False)
                self.clear_pick_wait()
                return

            if (
                self.pick_waiting_medicine is not None and
                time.monotonic() - self.last_pick_command_publish_time >= self.pick_command_repeat_sec
            ):
                self.publish_pick_command(str(self.pick_waiting_medicine['target']), True)

            if self.pick_deadline is not None and self.get_clock().now() > self.pick_deadline:
                self.broadcast(self.get_int('broadcast_pick_failed'))
                self.fail_task(token, '药品视觉抓取超时')
                return

        if self.place_waiting_token is not None:
            token = int(self.place_waiting_token)
            if not self.is_current_task(token):
                self.clear_place_wait()
                return

            if self.place_deadline is not None and self.get_clock().now() > self.place_deadline:
                self.fail_task(token, '药品放置超时')

    def on_pick_state(self, msg: String) -> None:
        state = self.pick_state_from_message(msg.data)
        if not state:
            return

        if self.place_waiting_token is not None:
            self.handle_place_state(state)
            return

        if self.pick_waiting_token is None:
            return

        token = int(self.pick_waiting_token)
        if not self.is_current_task(token):
            return

        if state in ('picking', 'lifting', 'placing'):
            messages = {
                'picking': '机械臂正在抓取药品',
                'lifting': '机械臂正在抬起药品',
                'placing': '机械臂正在放置药品',
            }
            self.update_task_status(
                token,
                pick_state=state,
                message=messages[state],
            )
            return

        if state in ('pick_complete', 'picked'):
            delivery = self.pick_waiting_delivery
            medicine = self.pick_waiting_medicine
            if delivery is None or medicine is None:
                return
            self.on_medicine_pick_complete(delivery, medicine, token)
            return

        if state in ('failed', 'pick_failed'):
            self.broadcast(self.get_int('broadcast_pick_failed'))
            self.fail_task(token, '药品视觉抓取失败')

    def handle_place_state(self, state: str) -> None:
        if self.place_waiting_token is None:
            return

        token = int(self.place_waiting_token)
        if not self.is_current_task(token):
            self.clear_place_wait()
            return

        if state == 'placing':
            self.update_task_status(
                token,
                pick_state='placing',
                message='机械臂正在放下药品',
            )
            return

        if state in ('placed', 'place_complete', 'put_complete'):
            self.on_medicine_place_complete(token)
            return

        if state in ('failed', 'place_failed', 'put_failed'):
            self.fail_task(token, '药品放置失败')

    def on_arm_phase(self, msg: String) -> None:
        if self.current_task == '':
            return
        phase = msg.data.strip() or 'unknown'
        if self.current_task_info.get('arm_phase') == phase:
            return
        self.current_task_info['arm_phase'] = phase
        if self.current_task_info.get('stage') in ('picking', 'placing'):
            phase_label = {
                'observe': '机械臂观察药品',
                'pick_down': '机械臂下探抓取',
                'pick_up': '机械臂抓取抬起',
                'pick_done': '机械臂抬起完成',
                'place': '机械臂放置药品',
            }.get(phase)
            if phase_label:
                self.current_task_info['message'] = phase_label
        self.publish_task_status()

    @staticmethod
    def pick_state_from_message(payload: str) -> str:
        text = payload.strip()
        if not text:
            return ''
        if text.startswith('{') and text.endswith('}'):
            try:
                data = json.loads(text)
                return str(data.get('state', '')).strip()
            except json.JSONDecodeError:
                return ''
        return text

    def broadcast(self, code: int) -> None:
        if int(code) <= 0:
            return
        if not self.enable_voice_broadcast:
            return
        if not self.ensure_speech_serial():
            return
        try:
            self.speech_serial.voice_write(code)
            self.get_logger().info(f'speech broadcast code {code}')
        except serial.SerialException as exc:
            self.get_logger().error(f'speech serial write failed: {exc}')
            self.speech_serial.close()
            self.speech_serial = None

    def call_later(self, delay_sec: float, callback: Callable[[], None]) -> None:
        if delay_sec <= 0.0:
            callback()
            return

        holder = {}

        def wrapped() -> None:
            timer = holder.get('timer')
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            callback()

        holder['timer'] = self.create_timer(delay_sec, wrapped)


def run(args=None, node_name: str = 'voice_nav_control') -> None:
    rclpy.init(args=args)
    node = VoiceNavControlNode(node_name=node_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    run(args=args)
