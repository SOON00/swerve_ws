import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import numpy as np
import math
import tf2_ros

class OmniParallelLineWaiter(Node):
    def __init__(self):
        super().__init__('omni_parallel_line_waiter')
        
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.last_print_time = 0.0
        
        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.is_heading_locked = None 
        self.last_target_yaw = None
        self.w_curr_ema = None 
        self.w_next_ema = None 
        
        self.has_aligned_to_single_corridor = True 
        
        self.get_logger().info("=========================================")
        self.get_logger().info("[단일 통로 1회성 정렬] 평행선 헤딩 제어 노드")
        self.get_logger().info("=========================================")

    def set_heading_mode(self, lock_heading, update_target, target_yaw):
        if self.is_heading_locked == lock_heading and not update_target:
            return 

        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/controller_server/set_parameters 서비스 대기 중...')
            return

        req = SetParameters.Request()
        p1 = Parameter(name="FollowPath.PathAngleCritic.cost_weight", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE))
        p2 = Parameter(name="FollowPath.HeadingLockCritic.cost_weight", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE))
        params = [p1, p2]

        if lock_heading:
            p1.value.double_value = 0.0
            p2.value.double_value = 5.0  # (필요시 이 값을 15.0 등으로 올려서 고정력을 높일 수 있습니다)
            state_msg = "헤딩 고정 ON"
        else:
            p1.value.double_value = 0.0
            p2.value.double_value = 0.0
            state_msg = "헤딩 고정 OFF"

        if update_target and target_yaw is not None:
            p3 = Parameter(name="FollowPath.HeadingLockCritic.locked_heading", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE))
            p3.value.double_value = float(target_yaw)
            params.append(p3)
            state_msg += f" | 타겟 헤딩 갱신: {target_yaw:.2f} rad"

        req.parameters = params
        future = self.param_client.call_async(req)
        future.add_done_callback(lambda fut: self.param_set_callback(fut, state_msg))
        
        self.is_heading_locked = lock_heading
        if update_target and target_yaw is not None:
            self.last_target_yaw = target_yaw

    def param_set_callback(self, future, state_msg):
        try:
            future.result()
            self.get_logger().info(f"파라미터 업데이트: {state_msg}")
        except Exception as e:
            self.get_logger().error(f"업데이트 실패: {e}")

    def scan_cb(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_print_time < 0.5: 
            return

        try:
            t = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, rclpy.time.Time())
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        except Exception as e:
            self.get_logger().warn("TF 오류 대기 중...", throttle_duration_sec=2.0)
            return

        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        
        valid = (ranges > msg.range_min) & (ranges < 5.0) & ~np.isinf(ranges) & ~np.isnan(ranges)
        if not np.any(valid): return

        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])

        search_angles = np.deg2rad(np.arange(0, 180, 2))
        cos_a = np.cos(search_angles)
        sin_a = np.sin(search_angles)

        r_projections = np.outer(x, cos_a) + np.outer(y, sin_a)
        bins = np.arange(-5.0, 5.0, 0.05)
        
        all_counts = []
        max_peaks = []
        for i in range(len(search_angles)):
            counts, _ = np.histogram(r_projections[:, i], bins=bins)
            all_counts.append(counts)
            max_peaks.append(np.max(counts))

        best_idx1 = np.argmax(max_peaks)
        best_idx2 = (best_idx1 + 45) % 90

        w1, _, _ = self.extract_width(all_counts[best_idx1], bins)
        w2, _, _ = self.extract_width(all_counts[best_idx2], bins)

        valid_widths = []
        if w1 is not None and 0.4 <= w1 <= 2.0:
            valid_widths.append((w1, best_idx1 * 2))
        if w2 is not None and 0.4 <= w2 <= 2.0:
            valid_widths.append((w2, best_idx2 * 2))

        if len(valid_widths) > 0:
            print(f"\n========================================")
            if len(valid_widths) == 1:
                print("[단일 통로] 코너 통과 완료 / 직진 중")
            else:
                print("[교차로] 코너 진입 / 다음 통로 감지")

        # =========================================================
        # 1. 단일 통로 로직 (코너 극복 후 1회성 정렬)
        # =========================================================
        if len(valid_widths) == 1:
            w_single, angle_single = valid_widths[0]
            
            if self.w_curr_ema is None:
                self.w_curr_ema = w_single
            else:
                if self.w_next_ema is not None and abs(w_single - self.w_next_ema) < abs(w_single - self.w_curr_ema):
                    self.get_logger().info("단일 통로 진입: 다음 통로가 현재 통로로 전환되었습니다.")
                    self.w_curr_ema = w_single
                    self.w_next_ema = None
                    self.has_aligned_to_single_corridor = False 
                else:
                    self.w_curr_ema = 0.6 * self.w_curr_ema + 0.4 * w_single 

            print(f"[현재] 통로 폭: {self.w_curr_ema:.2f}m")
            print("========================================\n")

            if not self.has_aligned_to_single_corridor:
                # 1. 벽과 평행한 두 로컬 방향(+90도, -90도) 산출
                local_yaw_1 = math.radians(angle_single + 90.0)
                local_yaw_2 = math.radians(angle_single - 90.0)
                
                # 2. 로컬 0도(전방)와 가장 가까운 평행선 방향을 선택
                norm_yaw_1 = math.atan2(math.sin(local_yaw_1), math.cos(local_yaw_1))
                norm_yaw_2 = math.atan2(math.sin(local_yaw_2), math.cos(local_yaw_2))
                best_local_yaw = norm_yaw_1 if abs(norm_yaw_1) < abs(norm_yaw_2) else norm_yaw_2
                
                # 3. 로봇의 현재 글로벌 각도에 더하여 타겟 계산 (부호 반전, 180도 보정 완전 제거됨)
                target_yaw_calc = robot_yaw + best_local_yaw
                target_yaw_calc = math.atan2(math.sin(target_yaw_calc), math.cos(target_yaw_calc))

                # 4. [Pi Ambiguity 해결] 180도 부근에서 제어기 360도 스핀 방지
                if abs(abs(target_yaw_calc) - math.pi) < 0.1:
                    if robot_yaw >= 0:
                        target_yaw_calc = abs(target_yaw_calc)
                    else:
                        target_yaw_calc = -abs(target_yaw_calc)
                elif abs(target_yaw_calc) < 0.001:
                    target_yaw_calc = 0.0

                # 5. 파라미터 업데이트 
                if self.w_curr_ema <= 1.15:
                    self.set_heading_mode(lock_heading=True, update_target=True, target_yaw=target_yaw_calc)
                else:
                    self.set_heading_mode(lock_heading=True, update_target=False, target_yaw=None)

                self.has_aligned_to_single_corridor = True
            
            self.last_print_time = current_time

        # =========================================================
        # 2. 교차로 통로 로직 (코너 진입 전 / 턴 중)
        # =========================================================
        elif len(valid_widths) == 2:
            self.has_aligned_to_single_corridor = False 
            
            (wa, angle_a), (wb, angle_b) = valid_widths
            
            if self.w_curr_ema is None:
                if wa < wb:
                    self.w_curr_ema, self.w_next_ema = wa, wb
                else:
                    self.w_curr_ema, self.w_next_ema = wb, wa
            else:
                if abs(wa - self.w_curr_ema) < abs(wb - self.w_curr_ema):
                    c_raw, n_raw = wa, wb
                else:
                    c_raw, n_raw = wb, wa
                
                self.w_curr_ema = 0.6 * self.w_curr_ema + 0.4 * c_raw
                if self.w_next_ema is None:
                    self.w_next_ema = n_raw
                else:
                    self.w_next_ema = 0.6 * self.w_next_ema + 0.4 * n_raw

            if self.w_curr_ema: print(f"[현재] 통로 폭: {self.w_curr_ema:.2f}m")
            if self.w_next_ema: print(f"[다음] 통로 폭: {self.w_next_ema:.2f}m")
            print("========================================\n")

            if self.w_next_ema is not None:
                desired_lock = (self.w_next_ema > 1.15)
                
                if self.is_heading_locked is not None:
                    if self.is_heading_locked:
                        desired_lock = not (self.w_next_ema < 1.10)
                    else:
                        desired_lock = (self.w_next_ema > 1.20)

                self.set_heading_mode(lock_heading=desired_lock, update_target=False, target_yaw=None)

            self.last_print_time = current_time
        else:
            self.get_logger().info("시야 확보 대기 중...", throttle_duration_sec=2.0)

    def extract_width(self, counts, bins):
        peaks = []
        for j in range(1, len(counts)-1):
            if counts[j] > counts[j-1] and counts[j] > counts[j+1] and counts[j] > 15:
                peaks.append((counts[j], bins[j]))
        
        if len(peaks) >= 2:
            peaks.sort(key=lambda x: x[0], reverse=True)
            r1 = peaks[0][1]
            r2 = peaks[1][1]
            return abs(r1 - r2), min(r1, r2), max(r1, r2)
        return None, None, None

def main():
    rclpy.init()
    node = OmniParallelLineWaiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()