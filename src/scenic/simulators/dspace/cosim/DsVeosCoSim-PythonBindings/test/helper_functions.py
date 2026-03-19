from pathlib import Path
from typing import Tuple, List, Union
import os
import platform
import re
import subprocess
import tempfile


def find_windows_veos_installation(version: str = "") -> Path:
    """
    Find the installation path of VEOS on Windows.

    Args:
        version (str, optional): The version of VEOS. Default is "VeosPlayer.Application".

    Returns:
        Path: The path to the VEOS installation directory.

    Raises:
        RuntimeError: If the installation is not found.
    """
    import winreg

    version = "." + version or version
    clsid = winreg.QueryValue(
        winreg.HKEY_CLASSES_ROOT, f"VeosPlayer.Application{version}\\CLSID"
    )
    local_path = Path(
        winreg.QueryValue(winreg.HKEY_CLASSES_ROOT, f"CLSID\\{clsid}\\LocalServer32")
    )

    return local_path.parent


def find_veos_bin_directory(version: str = "") -> str:
    """
    Find the installation bin directory of VEOS.

    Args:
        version (str): The version of VEOS.

    Returns:
        Path: The path to the VEOS bin directory.

    Raises:
        FileNotFoundError: If the installation is not found.
    """
    veos_bin = None
    if platform.system() == "Windows":
        veos_bin = find_windows_veos_installation(version=version)
    else:
        version = version.replace("-", "").lower()
        veos_bin = f"/opt/dspace/veos{version}/bin"
    if not os.path.isdir(veos_bin):
        raise FileNotFoundError(f"Could not find installation root for VEOS {version}")
    return veos_bin


def executed_in_azure() -> bool:
    """
    Determines if process is run in Azure pipeline.

    Returns:
        bool: True if run in Azure pipeline.
    """
    if any(key.startswith("AZP_") for key in os.environ.keys()):
        return True
    else:
        return False


def run_process(
    full_path: Union[None, str],
    args: Union[None, List[str]] = None,
    cwd: Union[None, str] = None,
    timeout: int = 60,
) -> subprocess.CompletedProcess:
    """
    Run a process with optional arguments.

    Args:
        full_path (str): The full path to the executable. Default is None.
        args (List[str], optional): List of arguments to pass to the process. Default is None.
        cwd (str, optional): The current working directory. Default is None.
        timeout (int, optional): The timeout for the executed command. Default is 60 s.

    Returns:
        CompletedProcess: The completed process.

    Raises:
        subprocess.TimeoutExpired: If the process times out.
    """
    if args:
        command = [full_path] + args
    else:
        command = [full_path]
    task = subprocess.run(
        command,
        cwd=cwd,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        encoding="utf-8",
        timeout=timeout,
    )

    if task.returncode:
        raise RuntimeError(
            f"Command returned non-zero exit status {task.returncode}:\n{task.stdout}\n{task.stderr}"
        )

    return task


def open_process(
    full_path: Union[None, str],
    args: Union[None, List[str]] = None,
    cwd: Union[None, str] = None,
) -> subprocess.Popen:
    """
    Run a process with optional arguments.

    Args:
        full_path (str): The full path to the executable. Default is None.
        args (List[str], optional): List of arguments to pass to the process. Default is None.
        cwd (str, optional): The current working directory. Default is None.
        timeout (int, optional): The timeout for the executed command. Default is 60 s.

    Returns:
        Popen: The Popen instance.

    Raises:
        subprocess.TimeoutExpired: If the process times out.
    """
    if args:
        command = [full_path] + args
    else:
        command = [full_path]
    return subprocess.Popen(
        command,
        cwd=cwd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        encoding="utf-8",
    )


def create_temp_dir() -> str:
    """
    Create a temporary directory.

    Returns:
        str: The path to the created temporary directory.
    """
    temp_dir = tempfile.mkdtemp()
    return temp_dir


def get_temp_dir() -> str:
    """
    Get path to temporary directory.

    Returns:
        str: The path to the temporary directory.
    """
    return tempfile.gettempdir()


def veos_build_container(
    veos_path, container_path: Path, output_file: Path, target="", clean_up=True
):
    if not target:
        if platform.system() == "Windows":
            target = "HostPC64/GCC"
        else:
            target = "HostPCLinux64/GCC"
    args = [
        "build",
        os.path.splittext(container_path)[1:],
        f'"{container_path}"',
        "--output-file",
        f'"{output_file}"',
        "--target",
        target,
    ]
    if clean_up:
        args.append("--clean-after-successful-build")

    return run_process(veos_path, args)


def veos_model_import(
    veos_path, osa_file: Path, json_file: Path, create_new=False
) -> subprocess.CompletedProcess:
    args = ["model", "import", f'"{osa_file}"', "-p", f'"{json_file}"']
    if create_new:
        args.append("-n")
        
    return run_process(veos_path, args)


def veos_config_any(veos_path: str, **kargs) -> subprocess.CompletedProcess:
    args = ["sim", "config"]
    if kargs:
        for key, value in kargs.items():
            args.append(f"--{key.replace('_', '-')}")
            args.append(f"{value}")

    return run_process(veos_path, args)


def veos_config_acceleration_factor(
    veos_path: str, acceleration_factor: float
) -> subprocess.CompletedProcess:
    args = ["sim", "config", "--acceleration-factor", f"{acceleration_factor}"]
    return run_process(veos_path, args)


def initialize_veos(veos_path: str, timeout: int = 60) -> subprocess.CompletedProcess:
    args = ["sim", "up", "--timeout", str(timeout)]
    return run_process(veos_path, args)


def load_veos_osa(
    veos_path: str, osa_path: str, timeout: int = 60
) -> subprocess.CompletedProcess:
    args = ["sim", "load", f'"{osa_path}"', "--timeout", str(timeout)]
    return run_process(veos_path, args)


def start_veos_simulation(
    veos_path: str, timeout: int = 60
) -> subprocess.CompletedProcess:
    args = ["sim", "start", "--timeout", str(timeout)]
    return run_process(veos_path, args)


def stop_veos_simulation(
    veos_path: str, timeout: int = 60
) -> subprocess.CompletedProcess:
    args = ["sim", "stop", "--timeout", str(timeout)]
    return run_process(veos_path, args)


def unload_veos_osa(veos_path: str, timeout: int = 60) -> subprocess.CompletedProcess:
    args = ["sim", "unload", "--timeout", str(timeout)]
    return run_process(veos_path, args)


def deinitialize_veos(veos_path: str, timeout: int = 60) -> subprocess.CompletedProcess:
    args = ["sim", "down", "--timeout", str(timeout)]
    return run_process(veos_path, args)


def get_eth_interface(name: str) -> Tuple[str, str, str]:
    """
    Get index, name, and type of ethernet interface by name.

    Args:
        name (str): The name of the interface.

    Returns:
        Tuple[str, str, str]: The index, name, and type of ethernet interface.
    """
    if platform.system() == "Windows":
        args = ["-Command", f"$(Get-NetAdapter -Name '{name}').ComponentID"]
        driver = run_process("powershell", args=args).stdout.strip()
        args = ["-Command", f"$(Get-NetAdapter -Name '{name}').InterfaceIndex"]
        index = run_process("powershell", args=args).stdout.strip()
        args = ["-Command", f"$(Get-NetIPAddress -InterfaceIndex '{index}').IPAddress"]
        ip = run_process("powershell", args=args).stdout.strip()
    else:
        args = ["address", "show", f"{name}"]
        process = run_process("ip", args)
        index = re.match(r"^(\d+): ", process.stdout).group(1)
        ip = re.search(
            r"inet (\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})\/", process.stdout, re.MULTILINE
        ).group(1)
        with open(f"/sys/class/net/{name}/tun_flags") as f:
            flags = int(f.read().strip(), 16)
        driver = "tap" if flags & 0x0002 else ""
    return index, ip, driver


def validate_eth_interface(name: str, ip: str, driver: str) -> bool:
    """
    Validate that a ethernet interface has the correct IP address and type.

    Args:
        name (str): The name of the ethernet interface to validate.
        ip (str): The IP address to be validated.
        driver (str): The type of the interface.

    Returns:
        bool: True if IP address and driver matches.
    """
    _, _ip, _driver = get_eth_interface(name)
    return ip == _ip and driver == _driver


def veos_model_autoconnect_signals(
    veos_path, osa_file: Path
) -> subprocess.CompletedProcess:
    args = ["model", "connect", "--autoconnect-signals", osa_file]
    return run_process(veos_path, args)
