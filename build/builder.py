# builder.py
import subprocess
import os
import shutil

class Builder:
    def __init__(self):
        self.root_dir = os.getcwd()
        self.build_dir = os.path.join(self.root_dir, 'build')
        self.result_dir = os.path.join(self.root_dir, 'result')
        os.makedirs(self.result_dir, exist_ok=True)

    def build_docker(self, version, robot):
        # prepare result/<version>_<robot>
        target = f"{version}_{robot}"
        target_dir = os.path.join(self.result_dir, target)
        if os.path.exists(target_dir):
            shutil.rmtree(target_dir)
        os.makedirs(target_dir)
        # copy base Dockerfile
        src_df = os.path.join(self.build_dir, 'ROS2', f'Dockerfile.{version}')
        dst_df = os.path.join(target_dir, 'Dockerfile')
        shutil.copy(src_df, dst_df)
        # copy robot script
        src_robot = os.path.join(self.build_dir, 'ROBOT', f'{robot}.sh')
        dst_robot = os.path.join(target_dir, f'{robot}.sh')
        shutil.copy(src_robot, dst_robot)
        # copy DDS implementation folder
        src_dds = os.path.join(self.build_dir, 'DDS', f'build.sh')
        dst_dds = os.path.join(target_dir, f'build.sh')
        shutil.copy(src_dds, dst_dds)
        # build Docker image
        tag = f"fuzzer_{version}_{robot}"
        print(f"Building DDS image: {tag}")
        subprocess.run([
            'docker', 'build',
            '--build-arg', f'TARGET_ROBOT={robot}',
            '-t', tag,
            target_dir
        ], check=True)