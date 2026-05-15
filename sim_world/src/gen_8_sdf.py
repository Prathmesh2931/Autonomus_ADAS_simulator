import math

segments = []
seg_id = 0

# ============================================================
# PARAMETERS
# ============================================================

road_w = 3.0

white_w = 0.08
yellow_w = 0.06

seg_len = 0.35

road_z = 0.001
mark_z = 0.003

samples = 260

# Scale of S-track
scale_x = 8.0
scale_y = 10.0

# ============================================================
# MATERIALS
# ============================================================

ROAD_MAT = """
<ambient>0.14 0.14 0.14 1</ambient>
<diffuse>0.14 0.14 0.14 1</diffuse>
"""

WHITE_MAT = """
<ambient>1 1 1 1</ambient>
<diffuse>1 1 1 1</diffuse>
"""

YELLOW_MAT = """
<ambient>1 1 0 1</ambient>
<diffuse>1 1 0 1</diffuse>
"""

# ============================================================
# ADD SEGMENT
# ============================================================

def add_seg(x, y, yaw, width, color, z):

    global seg_id

    if color == "road":
        mat = ROAD_MAT
    elif color == "white":
        mat = WHITE_MAT
    else:
        mat = YELLOW_MAT

    sdf = f"""
    <model name="seg_{seg_id:05d}">
      <static>true</static>

      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>

      <link name="link">

        <collision name="collision">

          <geometry>
            <box>
              <size>{seg_len:.4f} {width:.4f} 0.002</size>
            </box>
          </geometry>

        </collision>

        <visual name="visual">

          <geometry>
            <box>
              <size>{seg_len:.4f} {width:.4f} 0.002</size>
            </box>
          </geometry>

          <material>
            {mat}
          </material>

        </visual>

      </link>
    </model>
    """

    segments.append(sdf)
    seg_id += 1

# ============================================================
# DRAW ROAD CROSS SECTION
# ============================================================

def draw_section(x, y, yaw):

    # normal vector
    nx = -math.sin(yaw)
    ny =  math.cos(yaw)

    # road body
    add_seg(
        x, y,
        yaw,
        road_w,
        "road",
        road_z
    )

    # left white line
    add_seg(
        x + 0.9 * nx,
        y + 0.9 * ny,
        yaw,
        white_w,
        "white",
        mark_z
    )

    # right white line
    add_seg(
        x - 0.9 * nx,
        y - 0.9 * ny,
        yaw,
        white_w,
        "white",
        mark_z
    )

    # center yellow
    add_seg(
        x,
        y,
        yaw,
        yellow_w,
        "yellow",
        mark_z
    )

# ============================================================
# GENERATE "S" CENTERLINE
# ============================================================

# Parametric equation:
#
# x = A * sin(t)
# y = B * cos(t/2)
#
# Produces smooth S-shaped track
#
# Humanity spent centuries inventing calculus
# only to eventually use it for toy-car roads in Gazebo.
# Beautiful, honestly.

prev_x = None
prev_y = None

for i in range(samples):

    t = (i / (samples - 1)) * 2.6 * math.pi - 1.3 * math.pi

    x = scale_x * math.sin(t)

    y = scale_y * math.sin(t / 2)

    if prev_x is not None:

        dx = x - prev_x
        dy = y - prev_y

        yaw = math.atan2(dy, dx)

        draw_section(x, y, yaw)

    prev_x = x
    prev_y = y

# ============================================================
# SDF HEADER
# ============================================================

header = """<?xml version="1.0" ?>
<sdf version="1.6">

  <world name="s_track_world">

    <!-- PHYSICS -->

    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics"/>

    <!-- SENSORS -->

    <plugin
      filename="libignition-gazebo-sensors-system.so"
      name="ignition::gazebo::systems::Sensors">

      <render_engine>ogre2</render_engine>

    </plugin>

    <!-- SCENE -->

    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster"/>

    <!-- USER COMMANDS -->

    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands"/>

    <!-- SUN -->

    <light type="directional" name="sun">

      <cast_shadows>true</cast_shadows>

      <pose>0 0 30 0 0 0</pose>

      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>

      <direction>-0.5 0.1 -0.9</direction>

    </light>

    <!-- GROUND -->

    <model name="ground_plane">

      <static>true</static>

      <link name="link">

        <collision name="collision">

          <geometry>

            <plane>
              <normal>0 0 1</normal>
              <size>120 120</size>
            </plane>

          </geometry>

        </collision>

        <visual name="visual">

          <geometry>

            <plane>
              <normal>0 0 1</normal>
              <size>120 120</size>
            </plane>

          </geometry>

          <material>

            <ambient>0.08 0.08 0.08 1</ambient>
            <diffuse>0.08 0.08 0.08 1</diffuse>

          </material>

        </visual>

      </link>

    </model>

"""

footer = """
  </world>
</sdf>
"""

# ============================================================
# WRITE FILE
# ============================================================

sdf = header + "\n".join(segments) + footer

with open("s_track_world.sdf", "w") as f:
    f.write(sdf)

print(f"Generated {seg_id} segments")
print("Saved: s_track_world.sdf")