import os
from huggingface_hub import HfApi

api = HfApi(token=os.getenv("HF_TOKEN"))
api.upload_folder(
    folder_path="/opt/unitree_robotics/xr_teleoperate/teleop/utils/data/<>",
    repo_id="ShonH/<>",
    repo_type="dataset",
)
