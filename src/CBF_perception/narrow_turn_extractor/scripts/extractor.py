import yaml
import cv2
import numpy as np
import os


class MapData:
    def __init__(self, map_yaml_path):
        self.map_yaml_path = map_yaml_path
        self._load_yaml()
        self._load_image()
        self._build_occupancy_grid()
        self._compute_distance_map()

    def _load_yaml(self):
        with open(self.map_yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        self.image_path = meta['image']
        if not os.path.isabs(self.image_path):
            self.image_path = os.path.join(
                os.path.dirname(self.map_yaml_path),
                self.image_path
            )

        self.resolution = meta['resolution']
        self.origin = meta['origin']
        self.occupied_thresh = meta.get('occupied_thresh', 0.65)
        self.free_thresh = meta.get('free_thresh', 0.25)
        self.negate = meta.get('negate', 0)

        # print("[INFO] Map YAML loaded")
        # print(f"  image: {self.image_path}")
        # print(f"  resolution: {self.resolution}")
        # print(f"  origin: {self.origin}")

    def _load_image(self):
        self.image = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise RuntimeError("Failed to load map image")

        if self.negate:
            self.image = 255 - self.image

        # print("[INFO] Map image loaded")
        # print(f"  size: {self.image.shape}")

    def _build_occupancy_grid(self):
        norm = self.image.astype(np.float32) / 255.0
        self.occupied = norm < self.occupied_thresh
        self.free = norm > self.free_thresh

        # print("[INFO] Occupancy grid built")
        # print(f"  occupied cells: {np.sum(self.occupied)}")
        
    def _compute_distance_map(self):
        inv_occupied = (~self.occupied).astype(np.uint8)
        self.dist_map = cv2.distanceTransform(inv_occupied, cv2.DIST_L2, 3)

    def pixel_to_world(self, px, py):
        x = self.origin[0] + px * self.resolution
        y = self.origin[1] + (self.image.shape[0] - py) * self.resolution
        return x, y

    def world_to_pixel(self, x, y):
        px = int((x - self.origin[0]) / self.resolution)
        py = int(self.image.shape[0] - (y - self.origin[1]) / self.resolution)
        return px, py

    def get_binary_image(self):
        binary_img = np.zeros_like(self.image, dtype=np.uint8)
        binary_img[self.occupied] = 255
        binary_img[self.free] = 0

        return binary_img

    def extract_corner_points(self, boundary_img, epsilon_px=6.0, merge_dist=5):
        contours, _ = cv2.findContours(boundary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        corner_sets = []

        for cnt in contours:
            if len(cnt) < 40:
                continue

            approx = cv2.approxPolyDP(cnt, epsilon_px, closed=False)
            corners = [tuple(p[0]) for p in approx]
            # 근접 코너 병합
            corners = self.merge_close_corners(corners, merge_dist)
            if len(corners) >= 2:
                corner_sets.append(corners)

        return corner_sets
    
    def merge_close_corners(self, corners, min_dist=5):
        merged = []
        for x, y in corners:
            too_close = False
            for mx, my in merged:
                if np.hypot(mx - x, my - y) < min_dist:
                    too_close = True
                    break
            if not too_close:
                merged.append((x, y))
        return merged

    def visualize_corners(self, binary_img, corner_sets):
        overlay = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        for corners in corner_sets:
            for x, y in corners:
                cv2.circle(overlay, (x, y), 4, (0, 0, 255), -1)
        cv2.imshow("Corners", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    def line_pixels(self, x1, y1, x2, y2):
        line_img = np.zeros_like(self.image, dtype=np.uint8)
        cv2.line(line_img, (x1, y1), (x2, y2), 255, 1)
        ys, xs = np.where(line_img > 0)
        return list(zip(xs, ys))

    def line_distance_sum(self, x1, y1, x2, y2):
        pixels = self.line_pixels(x1, y1, x2, y2)
        if len(pixels) == 0:
            return np.inf
        total = 0.0
        for x, y in pixels:
            total += self.dist_map[y, x]
        return total

    def visualize_lines_by_distance(self, binary_img, corner_sets, max_distance_sum=50.0):
        overlay = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        line_count = 0

        for corners in corner_sets:
            N = len(corners)
            for i in range(N):
                for j in range(i + 1, N):
                    x1, y1 = corners[i]
                    x2, y2 = corners[j]
                    dist_sum = self.line_distance_sum(x1, y1, x2, y2)

                    if dist_sum <= max_distance_sum:
                        cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        line_count += 1

                        if x2 != x1:
                            a = (y2 - y1) / (x2 - x1)
                            b = y1 - a * x1
                            print(f"Line: y = {a:.3f}x + {b:.3f}  (points: ({x1},{y1})-({x2},{y2}))")
                        else:
                            print(f"Line: x = {x1}  (points: ({x1},{y1})-({x2},{y2}))")

        print(f"\nTotal lines: {line_count}\n")

        cv2.imshow("Distance Filtered Lines", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    map_yaml_path = os.path.abspath(
        os.path.join(script_dir, "..", "maps", "cbf_map.yaml")
    )

    map_data = MapData(map_yaml_path)
    binary_img = map_data.get_binary_image()
    corner_sets = map_data.extract_corner_points(binary_img, epsilon_px=6.0, merge_dist=5)

    # map_data.visualize_corners(binary_img, corner_sets)

    map_data.visualize_lines_by_distance(binary_img, corner_sets, max_distance_sum=50.0)


if __name__ == "__main__":
    main()