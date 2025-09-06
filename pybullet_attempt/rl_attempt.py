import gymnasium as gym
import pybullet as p
import pybullet_data
import numpy as np
import math
import os
from stable_baselines3 import PPO

class HexabotEnv(gym.Env):
    """
    A custom Gymnasium environment for the PyBullet hexabot with video recording.
    """
    def __init__(self, record_video=False):
        super(HexabotEnv, self).__init__()

        self.record_video = record_video
        self.episode_count = 0  # Initialize episode counter
        
        # Connect to PyBullet in GUI mode for visualization
        # self.physicsClient = p.connect(p.GUI)
        self.physicsClient = p.connect(p.GUI, options='--width=960 --height=1010')
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        # Define the action and observation spaces
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(24,), dtype=np.float32)
        obs_dim = 3 + 4 + 24 + 24
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

        # Load the robot and ground plane
        self.planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF("urdf/hexabot_v2.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

        # Create a directory to save videos
        if self.record_video and not os.path.exists("training_videos"):
            os.makedirs("training_videos")

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Increment the episode counter
        self.episode_count += 1
        
        # Stop any previous video recording and start a new one
        if self.record_video:
            # Stop any previous video
            if hasattr(self, 'video_log_id'):
                p.stopStateLogging(self.video_log_id)
            
            # Use the internal counter to name the video file
            video_filename = f"training_videos/episode_{self.episode_count}.mp4"
            self.video_log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_filename)
        
        # Reset robot position and orientation
        p.resetBasePositionAndOrientation(self.robotId, [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

        # Reset all joints to a neutral position
        for joint_index in range(p.getNumJoints(self.robotId)):
            p.resetJointState(self.robotId, joint_index, 0)

        observation = self._get_observation()
        info = {}
        return observation, info

    def _get_observation(self):
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robotId)
        joint_states = p.getJointStates(self.robotId, range(p.getNumJoints(self.robotId)))
        joint_positions = [s[0] for s in joint_states]
        joint_velocities = [s[1] for s in joint_states]

        obs = np.concatenate([
            np.array(base_pos),
            np.array(base_orn),
            np.array(joint_positions),
            np.array(joint_velocities)
        ])
        return obs

    def _calculate_reward(self, prev_base_pos, current_base_pos, base_orn):
        forward_reward = current_base_pos[0] - prev_base_pos[0]
        roll, pitch, _ = p.getEulerFromQuaternion(base_orn)
        stability_penalty = -abs(roll) - abs(pitch)
        reward_multiplier = 10
        reward = reward_multiplier*forward_reward + stability_penalty
        # print(reward)
        return reward

    def step(self, action):
        prev_base_pos, _ = p.getBasePositionAndOrientation(self.robotId)
        amplitude_rad = math.radians(30)
        target_joint_angles = action * amplitude_rad

        p.setJointMotorControlArray(
            bodyUniqueId=self.robotId,
            jointIndices=range(p.getNumJoints(self.robotId)),
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_joint_angles,
            forces=[500] * 24
        )

        p.stepSimulation()
        p.setRealTimeSimulation(0)
        
        observation = self._get_observation()
        current_base_pos, current_base_orn = p.getBasePositionAndOrientation(self.robotId)

        reward = self._calculate_reward(prev_base_pos, current_base_pos, current_base_orn)
        
        done = current_base_pos[2] < 0.2
        info = {}

        return observation, reward, done, False, info

    def render(self, mode="human"):
        pass

    def close(self):
        if self.record_video and hasattr(self, 'video_log_id'):
            p.stopStateLogging(self.video_log_id)
        p.disconnect()

# --- Training Script ---
if __name__ == '__main__':
    # Set record_video to True to enable recording
    env = HexabotEnv(record_video=True)

    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./hexabot_tensorboard_log/")
    print("Starting training...")
    model.learn(total_timesteps=1000000)
    print("Training finished.")
    model.save("hexabot_ppo_model")
    print("Model saved as hexabot_ppo_model.zip")
    env.close()