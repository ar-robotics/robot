import subprocess

for i in subprocess.check_output(["ps", "-fA"]).decode().splitlines():
    if "python" in i:
        data = i.split()
        pid = data[1]
        path = data[8]

        if "vr_linker" in path:
            subprocess.run(["kill", "-9", pid])
            print(f"killed {pid}")
