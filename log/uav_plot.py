import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Đọc file CSV
df = pd.read_csv("uav_log.csv", delim_whitespace=False)  # hoặc sep='\t' nếu là file TSV

# Lấy dữ liệu mong muốn
x_des = df["x_desired"].to_numpy()
y_des = df["y_desired"].to_numpy()
z_des = df["z_desired"].to_numpy()

# Lấy dữ liệu thực tế
x_act = df["x_actual"].to_numpy()
y_act = df["y_actual"].to_numpy()
z_act = df["z_actual"].to_numpy()

# Tạo figure 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Vẽ quỹ đạo mong muốn (xanh lá)
ax.plot(x_des, y_des, z_des, label="Quỹ đạo mong muốn", color='green', linestyle='-', linewidth=2)

# Vẽ quỹ đạo thực tế (đỏ)
ax.plot(x_act, y_act, z_act, label="Quỹ đạo thực tế", color='red', linestyle='-', linewidth=1.5)

# Điểm takeoff
ax.scatter(x_des[0], y_des[0], z_des[0]-2, color='blue', s=50, marker='o')

# Điểm start
ax.scatter(x_des[0], y_des[0], z_des[0], color='green', s=50, marker='o')

# Điểm goal
ax.scatter(x_des[-1], y_des[-1], z_des[-1], color='red', s=50, marker='o')

# Thêm nhãn văn bản
ax.text(x_des[0] + 0.3, y_des[0] + 0.3, z_des[0] - 2 + 0.3, "Takeoff", color='blue', fontsize=10)
ax.text(x_des[0] + 0.3, y_des[0] + 0.3, z_des[0] + 0.3, "Start", color='green', fontsize=10)
ax.text(x_des[-1] + 0.3, y_des[-1] + 0.3, z_des[-1] + 0.3, "Goal", color='red', fontsize=10)

# Thiết lập biểu đồ
ax.set_title("So sánh quỹ đạo UAV")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.legend()
ax.grid(True)

# Giới hạn bản đồ
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 10)

plt.tight_layout()
plt.show()

