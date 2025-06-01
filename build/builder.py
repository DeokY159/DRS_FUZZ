# builder.py
import subprocess
import os
import shutil

BUILD_MEMORY = '8g'

class Builder:
    def __init__(self):
        self.root_dir = os.getcwd()
        self.build_dir = os.path.join(self.root_dir, 'build')
        self.cache_dir = os.path.join(self.build_dir, 'cache')
        os.makedirs(self.cache_dir, exist_ok=True)

    def build_docker(self, version, robot, headless=False):
        tag = f"fuzzer_{version}_{robot}_headless" if headless else f"fuzzer_{version}_{robot}"
        
        existing = subprocess.run(
            ['docker', 'images', '-q', tag],
            capture_output=True, text=True
        ).stdout.strip()
        if existing:
            print(f"[Builder] Image '{tag}' already exists (id={existing}); skipping build.")
            return
        
        # prepare cache/<version>_<robot>
        target = f"{version}_{robot}"
        target_dir = os.path.join(self.cache_dir, target)
        if os.path.exists(target_dir):
            shutil.rmtree(target_dir)
        os.makedirs(target_dir)

        # copy base Dockerfile
        src_df = os.path.join(self.build_dir, 'ROS2', f'Dockerfile.{version}')
        dst_df = os.path.join(target_dir, 'Dockerfile')
        shutil.copy(src_df, dst_df)

        # determine robot script (headless-aware)
        robot_script_name = f"{robot}.headless.sh" if headless else f"{robot}.sh"
        src_robot = os.path.join(self.build_dir, 'ROBOT', robot_script_name)
        dst_robot = os.path.join(target_dir, robot_script_name)

        if not os.path.exists(src_robot):
            raise FileNotFoundError(f"[Builder] Robot script not found: {src_robot}")
        
        shutil.copy(src_robot, dst_robot)

        # build Docker image
        print(f"[Builder] Building DDS image: {tag} (headless={headless})")
        subprocess.run([
            'docker', 'build',
            f'--memory={BUILD_MEMORY}',
            f'--memory-swap={BUILD_MEMORY}',
            '--build-arg', f'TARGET_ROBOT={robot_script_name}',
            '-t', tag,
            target_dir
        ], check=True)
