# builder.py
import subprocess
import os
import shutil

BUILD_MEMORY = '8g'

class Builder:
    def __init__(self):
        self.root_dir = os.getcwd()
        self.build_dir = os.path.join(self.root_dir, 'build')
        self.result_dir = os.path.join(self.root_dir, 'result')
        os.makedirs(self.result_dir, exist_ok=True)

    def build_docker(self, version, robot):
        tag = f"fuzzer_{version}_{robot}"
        '''
        existing = subprocess.run(
            ['docker', 'images', '-q', tag],
            capture_output=True, text=True
        ).stdout.strip()
        if existing:
            print(f"[Builder] Image '{tag}' already exists (id={existing}); skipping build.")
            return
        '''
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

        # build Docker image
        print(f"[Builder] Building DDS image: {tag}")
        subprocess.run([
            'docker', 'build',
            f'--memory={BUILD_MEMORY}',
            f'--memory-swap={BUILD_MEMORY}',
            '--build-arg', f'TARGET_ROBOT={robot}',
            '-t', tag,
            target_dir
        ], check=True)
