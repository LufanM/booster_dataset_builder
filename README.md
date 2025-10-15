# Booster Dataset builder

该仓库的主要用于将**booster T1 7dof**的遥操作数据实时保存下来，并生成RLDS格式的数据集。支持两种方式的生成：

* 直接实时保存数据：一种运行脚本`booster_dataset/create_booster_data_realtime.py`生成`.npy`格式的数据；

* 基于Ros2bag实时保存数据：首先ros2 bag的方式生成一个数据包，然后运行脚本`ros2_data_process/rosbag_2_npy.py`将ros2bag数据包转成`.npy`格式。


然后，`.npy`数据格式通过tfds build生成RLDS的数据集，生成方式参考下面**RLDS Dataset Conversion**章节。

# 1 依赖安装

默认您已经安装了`Booster Teleop`的环境，这样`create_booster_data_realtime.py`的环境就配置好了

# 2 数据保存

## 2.1 直接实时保存数据

主要流程为:

1. 启动手腕相机，默认相机启动包已经安装于运控板下的`~/ros2_ws/src`

   ```bash
   cd ~/ros2
   source ./install/setup.bash
   ros2 launch orbbec_camera multi_camera.launch.py
   ```

   相机参数，如分辨率修改请参考[OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)

2. 运行脚本`booster_dataset/create_booster_data_realtime.py`生成`.npy`格式的数据 。

   ```bash
   python booster_dataset/create_booster_data_realtime.py
   ```

3. `.npy`数据通过tfds build生成RLDS的数据集。使用tfds build时，需使用相应的一个环境，参考下面**RLDS Dataset Conversion**章节的**3.1** 和 **3.2**。当然您也可以将它转换成自己需要的格式，`.npy`数据的格式定义在`create_booster_data_realtime.py`，可根据需要修改，当然需保证与机器人发出的数据保持一一对应，详情请仔细阅读下面**RLDS Dataset Conversion**章节的**3.3**。

## 2.2 基于Ros2bag实时保存数据

**务必于感知板运行**，且默认已启动了手腕相机

* 进入感知板

  

* 实时保存数据

  ```bash
  cd ~/Workspace
  source  booster_robotics_sdk_ros2/booster_ros2_interface/install/setup.bash
  ros2 bag record topics /camera_right/color/image_raw     /camera_left/color/image_raw /camera/camera/color/image_raw /low_state
  ```

* 运行脚本`ros2_data_process/rosbag_2_npy.py`将ros2 bag转换成.npy数据，ros2 bag的数据文件需放置于`rosbag_2_npy.py`同级文件夹中。转换后会在统计文件的`./data`生成.npy格式数据，然后拷贝至`booster_dataset/data/`的`train`或者`val`文件夹中，以便进行RLDS格式转换。

  ```bash
  python ros2_data_process/rosbag_2_npy.py --bag_path=./your_rosbag2_file_name --output_dir=./
  ```

* 最后进行RLDS转换，保证`booster_dataset/data/`的`train`合`val`都有episode文件再进行转换，转换参考下面**RLDS Dataset Conversion**章节的**3.1** 和 **3.2**。



# 3 RLDS Dataset Conversion

This is based on [**RLDS Dataset**](https://github.com/kpertsch/rlds_dataset_builder) repo,  demonstrates how to convert an existing dataset into RLDS format for X-embodiment experiment integration.
It provides an example for converting a dummy dataset to RLDS. To convert your own dataset, **fork** this repo and modify the example code for your dataset following the steps below.

> **需要注意的是：**`create_booster_data_realtime.py`里面的step_data数据格式需与`booster_dataset_dataset_builder.py`中的step数据格式一一对应，如标签`image`，如果生成.npy时时height×width=720×1280的格式，那么step里面的shape需为(720, 1280, 3)，如标签`joint_pos`的维度，在.npy生成时的维度与shape里面注明的维度一致，这里7dof双臂的话为14，否则tfds build时会报错。若使用ros2bag也是同样的原理，保证数据格式一一对应。

## 3.1 Installation

First create a conda environment using the provided environment.yml file (use `environment_ubuntu.yml` or `environment_macos.yml` depending on the operating system you're using):

```
conda env create -f environment_ubuntu.yml
```

Then activate the environment using:

```
conda activate rlds_env
```

If you want to manually create an environment, the key packages to install are `tensorflow`, 
`tensorflow_datasets`, `tensorflow_hub`, `apache_beam`, `matplotlib`, `plotly` and `wandb`.


## 3.2 Run Example RLDS Dataset Creation

Before modifying the code to convert your own dataset, run the provided example dataset creation script to ensure
everything is installed correctly. Run the following lines to create some dummy data and convert it to RLDS.

```
cd example_dataset
python3 create_example_data.py
tfds build  # tfds build --overwrite ：Convert your dataset according to the rules of *-dataset_builder.py
```

This should create a new dataset in `~/tensorflow_datasets/example_dataset`. Please verify that the example
conversion worked before moving on.


## 3.3 Converting your Own Dataset to RLDS

Now we can modify the provided example to convert your own data. Follow the steps below:

1. **Rename Dataset**: Change the name of the dataset folder from `example_dataset` to the name of your dataset (e.g. robo_net_v2), 
   also change the name of `example_dataset_dataset_builder.py` by replacing `example_dataset` with your dataset's name (e.g. robo_net_v2_dataset_builder.py)
   and change the class name `ExampleDataset` in the same file to match your dataset's name, using camel case instead of underlines (e.g. RoboNetV2).

2. **Modify Features**: Modify the data fields you plan to store in the dataset. You can find them in the `_info()` method
   of the `ExampleDataset` class. Please add **all** data fields your raw data contains, i.e. please add additional features for 
   additional cameras, audio, tactile features etc. If your type of feature is not demonstrated in the example (e.g. audio),
   you can find a list of all supported feature types [here](https://www.tensorflow.org/datasets/api_docs/python/tfds/features?hl=en#classes).
   You can store step-wise info like camera images, actions etc in `'steps'` and episode-wise info like `collector_id` in `episode_metadata`.
   Please don't remove any of the existing features in the example (except for `wrist_image` and `state`), since they are required for RLDS compliance.
   Please add detailed documentation what each feature consists of (e.g. what are the dimensions of the action space etc.).
   Note that we store `language_instruction` in every step even though it is episode-wide information for easier downstream usage (if your dataset
   does not define language instructions, you can fill in a dummy string like `pick up something`).

3. **Modify Dataset Splits**: The function `_split_generator()` determines the splits of the generated dataset (e.g. training, validation etc.).
   If your dataset defines a train vs validation split, please provide the corresponding information to `_generate_examples()`, e.g. 
   by pointing to the corresponding folders (like in the example) or file IDs etc. If your dataset does not define splits,
   remove the `val` split and only include the `train` split. You can then remove all arguments to `_generate_examples()`.

4. **Modify Dataset Conversion Code**: Next, modify the function `_generate_examples()`. Here, your own raw data should be 
   loaded, filled into the episode steps and then yielded as a packaged example. Note that the value of the first return argument,
   `episode_path` in the example, **is only used as a sample ID in the dataset and can be set to any value** that is connected to the 
   particular stored episode, or any other random value. Just ensure to avoid using the same ID twice.

5. **Provide Dataset Description**: Next, add a bibtex citation for your dataset in `CITATIONS.bib` and add a short description
   of your dataset in `README.md` inside the dataset folder. You can also provide a link to the dataset website and please add a
   few example trajectory images from the dataset for visualization.

6. **Add Appropriate License**: Please add an appropriate license to the repository. 
   Most common is the [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) license -- 
   you can copy it from [here](https://github.com/teamdigitale/licenses/blob/master/CC-BY-4.0).

That's it! You're all set to run dataset conversion. Inside the dataset directory, run:

```
tfds build --overwrite
```

The command line output should finish with a summary of the generated dataset (including size and number of samples). 
Please verify that this output looks as expected and that you can find the generated `tfrecord` files in `~/tensorflow_datasets/<name_of_your_dataset>`.


### Parallelizing Data Processing

By default, dataset conversion is single-threaded. If you are parsing a large dataset, you can use parallel processing.
For this, replace the last two lines of `_generate_examples()` with the commented-out `beam` commands. This will use 
Apache Beam to parallelize data processing. Before starting the processing, you need to install your dataset package 
by filling in the name of your dataset into `setup.py` and running `pip install -e .`

Then, make sure that no GPUs are used during data processing (`export CUDA_VISIBLE_DEVICES=`) and run:

```
tfds build --overwrite --beam_pipeline_options="direct_running_mode=multi_processing,direct_num_workers=10"
```

You can specify the desired number of workers with the `direct_num_workers` argument.

## 3.4 Visualize Converted Dataset

To verify that the data is converted correctly, please run the data visualization script from the base directory:

```
python3 dataset_visualize_mo.py
```

Change the direct in `b = tfds.builder_from_directory(f"~/tensorflow_datasets/example_dataset/1.0.0")`to your own converted dataset directory.

## 3.5 Add Transform for Target Spec

For X-embodiment training we are using specific inputs / outputs for the model: input is a single RGB camera, output
is an 8-dimensional action, consisting of end-effector position and orientation, gripper open/close and a episode termination
action.

The final step in adding your dataset to the training mix is to provide a transform function, that transforms a step
from your original dataset above to the required training spec. Please follow the two simple steps below:

1. **Modify Step Transform**: Modify the function `transform_step()` in `example_transform/transform.py`. The function 
   takes in a step from your dataset above and is supposed to map it to the desired output spec. The file contains a detailed
   description of the desired output spec.

2. **Test Transform**: We provide a script to verify that the resulting __transformed__ dataset outputs match the desired
   output spec. Please run the following command: `python3 test_dataset_transform.py <name_of_your_dataset>`

If the test passes successfully, you are ready to upload your dataset!

## 3.6 Upload Your Data

We provide a Google Cloud bucket that you can upload your data to. First, install `gsutil`, the Google cloud command 
line tool. You can follow the installation instructions [here](https://cloud.google.com/storage/docs/gsutil_install).

Next, authenticate your Google account with:

```
gcloud auth login
```

This will open a browser window that allows you to log into your Google account (if you're on a headless server, 
you can add the `--no-launch-browser` flag). Ideally, use the email address that
you used to communicate with Karl, since he will automatically grant permission to the bucket for this email address. 
If you want to upload data with a different email address / google account, please shoot Karl a quick email to ask 
to grant permissions to that Google account!

After logging in with a Google account that has access permissions, you can upload your data with the following 
command:

```
gsutil -m cp -r ~/tensorflow_datasets/<name_of_your_dataset> gs://xembodiment_data
```

This will upload all data using multiple threads. If your internet connection gets interrupted anytime during the upload
you can just rerun the command and it will resume the upload where it was interrupted. You can verify that the upload
was successful by inspecting the bucket [here](https://console.cloud.google.com/storage/browser/xembodiment_data).

The last step is to commit all changes to this repo and send Karl the link to the repo.

**Thanks a lot for contributing your data! :)**
