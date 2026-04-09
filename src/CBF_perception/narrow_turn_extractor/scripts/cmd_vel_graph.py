import yaml
import matplotlib.pyplot as plt

# ---------- load yaml ----------
with open("cmd_vel.yaml", "r") as f:
    docs = list(yaml.safe_load_all(f))

# ---------- extract data ----------
vx, vy, wz = [], [], []

for d in docs:
    if d is None:
        continue

    if "linear" not in d or "angular" not in d:
        continue
    if "x" not in d["linear"] or "y" not in d["linear"]:
        continue
    if "z" not in d["angular"]:
        continue

    vx.append(d["linear"]["x"])
    vy.append(-d["linear"]["y"])
    wz.append(d["angular"]["z"])

if len(vx) == 0:
    raise RuntimeError("유효한 twist 데이터가 없습니다. YAML 파일을 확인하세요.")

t = range(len(vx))

# ---------- plot ----------
plt.figure(figsize=(10, 5))
plt.plot(t, vx, label="linear.x")
plt.plot(t, vy, label="linear.y")
plt.plot(t, wz, label="angular.z")
plt.legend()
plt.xlabel("time step")
plt.ylabel("value")
plt.title("Twist Log")
plt.grid(True)
plt.tight_layout()
plt.show()
