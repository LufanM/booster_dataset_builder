import numpy as np
import tqdm
import os

N_TRAIN_EPISODES = 10
N_VAL_EPISODES = 10

EPISODE_LENGTH = 5


def create_fake_episode(path):
    episode = []
    for step in range(EPISODE_LENGTH):
        episode.append({
            'image': np.asarray(np.random.rand(720, 1280, 3) * 255, dtype=np.uint8),
            'l_wrist_image': np.asarray(np.random.rand(424, 240, 3) * 255, dtype=np.uint8),
            'r_wrist_image': np.asarray(np.random.rand(424, 240, 3) * 255, dtype=np.uint8),
            'joint_pos': np.asarray(np.random.rand(14), dtype=np.float32),
            'eef_pos': np.asarray(np.random.rand(14), dtype=np.float32),
            'action': np.asarray(np.random.rand(14), dtype=np.float32),
            'language_instruction': 'dummy instruction',
        })
    np.save(path, episode)


# create fake episodes for train and validation
print("Generating train examples...")
os.makedirs('data/train', exist_ok=True)
for i in tqdm.tqdm(range(N_TRAIN_EPISODES)):
    create_fake_episode(f'data/train/episode_{i}.npy')

print("Generating val examples...")
os.makedirs('data/val', exist_ok=True)
for i in tqdm.tqdm(range(N_VAL_EPISODES)):
    create_fake_episode(f'data/val/episode_{i}.npy')

print('Successfully created example data!')
