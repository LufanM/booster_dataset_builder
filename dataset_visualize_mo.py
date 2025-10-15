import numpy as np
import tensorflow_datasets as tfds
from PIL import Image
# from IPython import display
import os

def as_gif(images, path='temp.gif'):
  # Render the images as the gif:
  images[0].save(path, save_all=True, append_images=images[1:], duration=1000, loop=0)
  gif_bytes = open(path,'rb').read()
  return gif_bytes


dataset_name = 'jaco_play' # @param ['fractal20220817_data', 'kuka', 'bridge', 'taco_play', 'jaco_play', 'berkeley_cable_routing', 'roboturk', 'nyu_door_opening_surprising_effectiveness', 'viola', 'berkeley_autolab_ur5', 'toto', 'language_table', 'columbia_cairlab_pusht_real', 'stanford_kuka_multimodal_dataset_converted_externally_to_rlds', 'nyu_rot_dataset_converted_externally_to_rlds', 'stanford_hydra_dataset_converted_externally_to_rlds', 'austin_buds_dataset_converted_externally_to_rlds', 'nyu_franka_play_dataset_converted_externally_to_rlds', 'maniskill_dataset_converted_externally_to_rlds', 'furniture_bench_dataset_converted_externally_to_rlds', 'cmu_franka_exploration_dataset_converted_externally_to_rlds', 'ucsd_kitchen_dataset_converted_externally_to_rlds', 'ucsd_pick_and_place_dataset_converted_externally_to_rlds', 'austin_sailor_dataset_converted_externally_to_rlds', 'austin_sirius_dataset_converted_externally_to_rlds', 'bc_z', 'usc_cloth_sim_converted_externally_to_rlds', 'utokyo_pr2_opening_fridge_converted_externally_to_rlds', 'utokyo_pr2_tabletop_manipulation_converted_externally_to_rlds', 'utokyo_saytap_converted_externally_to_rlds', 'utokyo_xarm_pick_and_place_converted_externally_to_rlds', 'utokyo_xarm_bimanual_converted_externally_to_rlds', 'robo_net', 'berkeley_mvp_converted_externally_to_rlds', 'berkeley_rpt_converted_externally_to_rlds', 'kaist_nonprehensile_converted_externally_to_rlds', 'stanford_mask_vit_converted_externally_to_rlds', 'tokyo_u_lsmo_converted_externally_to_rlds', 'dlr_sara_pour_converted_externally_to_rlds', 'dlr_sara_grid_clamp_converted_externally_to_rlds', 'dlr_edan_shared_control_converted_externally_to_rlds', 'asu_table_top_converted_externally_to_rlds', 'stanford_robocook_converted_externally_to_rlds', 'eth_agent_affordances', 'imperialcollege_sawyer_wrist_cam', 'iamlab_cmu_pickup_insert_converted_externally_to_rlds', 'uiuc_d3field', 'utaustin_mutex', 'berkeley_fanuc_manipulation', 'cmu_food_manipulation', 'cmu_play_fusion', 'cmu_stretch', 'berkeley_gnm_recon', 'berkeley_gnm_cory_hall', 'berkeley_gnm_sac_son']
display_key = 'image'

b = tfds.builder_from_directory(f"~/tensorflow_datasets/example_dataset/1.0.0")
ds = b.as_dataset(split='train') # 具体可以根据需求改
output_dir = f'{dataset_name}_videos'

os.makedirs(output_dir, exist_ok=True)

instructions_file_path = os.path.join(output_dir, f'{dataset_name}.txt')
state_file_path = os.path.join(output_dir, f'state.txt')
# 遍历数据集
for idx, episode in enumerate(ds):
    # 为每个视频创建一个文件夹
    video_folder = os.path.join(output_dir, f'video_{idx}')
    os.makedirs(video_folder, exist_ok=True)

    # 提取该视频的所有帧
    frames = episode['steps']

    # 遍历每一帧并保存
    state_list = []
    for frame_idx, step in enumerate(frames):
    	# 每个数据集image的特征名字不一样，具体要看数据集下载好后的 features.json 文件中对应的字段是什么
        # image = step['observation'][image] # fractal20220817_data
        # image = step['observation']["agentview_rgb"] # viola
        image = step['observation']["image"] # bridge

        # discout = step['discount']
        # print(f"discout is: {discout}")

        # 获取自然语言指令，具体要看数据集下载好后的 features.json 文件对应的字段是什么
        # natural_language_instruction = step["language_instruction"].numpy().decode('utf-8') # for ucsd、berkeley_fanuc_manipulation
        # natural_language_instruction = step['observation']["natural_language_instruction"].numpy().decode('utf-8') 

        state_list.append(step['observation']["state"])
        # state_list.append(step['observation']["joint_pos"])

        # 将图像转换为 PIL 格式
        image_pil = Image.fromarray(image.numpy())

        # 保存图像，文件名格式为 frame_{frame_idx}.png
        output_path = os.path.join(video_folder, f'frame_{frame_idx}.png')
        image_pil.save(output_path)

    with open(state_file_path, 'a') as f:
        f.write(f"state {idx}: {state_list}\n")

    with open(instructions_file_path, 'a') as f:
        f.write(f"Video {idx} Instruction: {idx}\n")

    print(f"第 {idx} 个视频的所有帧已保存到: {video_folder}, 该视频共有{frame_idx + 1}帧")

print("所有视频的帧提取完成。")