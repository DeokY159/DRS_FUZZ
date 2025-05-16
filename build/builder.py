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
        tag = f"fuzzer_{version}_{robot}"
        print(f"Building DDS image: {tag}")
        subprocess.run([
            'docker', 'build',
            f'--memory={BUILD_MEMORY}',
            f'--memory-swap={BUILD_MEMORY}',
            '--build-arg', f'TARGET_ROBOT={robot}',
            '-t', tag,
            target_dir
        ], check=True)
    
    def run_docker(self, version, robot, container_name='test'):
        """
        xhost 설정 후, DISPLAY 포워딩과 --privileged 옵션으로 컨테이너를 실행합니다.
        Docker network 'drs_fuzz'를 bridge 모드로 생성하고 지정합니다.
        """
        image_tag = f"fuzzer_{version}_{robot}"
        network_name = "drs_fuzz"
        subnet = "172.21.0.0/16"
        container_ip = "172.21.0.2"

        print(f"[Builder] Granting X server access: xhost +local:root")
        subprocess.run(['xhost', '+local:root'], check=True)

        # Try creating network only if it doesn't exist
        result = subprocess.run(
            ['docker', 'network', 'ls', '--filter', f'name=^{network_name}$', '--format', '{{.Name}}'],
            capture_output=True, text=True
        )

        if network_name not in result.stdout.splitlines():
            print(f"[Builder] Creating Docker network: {network_name}")
            subprocess.run([
                'docker', 'network', 'create',
                '--driver', 'bridge',
                f'--subnet={subnet}',
                network_name
            ], check=True)
        else:
            print(f"[Builder] Docker network '{network_name}' already exists. Reusing.")


        # Run container with the created network
        cmd = [
            'docker', 'run',
            '-it', '--privileged',
            '-e', f'DISPLAY={os.environ.get("DISPLAY")}',
            '-v', '/tmp/.X11-unix:/tmp/.X11-unix',
            '--net', network_name,
            '--ip', container_ip,
            '--name', container_name,
            image_tag
        ]
        print(f"[Builder] Running container '{container_name}' from image '{image_tag}' with network '{network_name}'")
        subprocess.run(cmd, check=True)
        print(f"[Builder] Container '{container_name}' exited")