# About ros_translator

`ros_translator` is a tool that converts ROS message definitions to other formats, current supported output are

- [typescript](ros_translator/typescript/README.md)
- pydantic

```
usage: ros_translator [-h] -t TARGET -o OUTDIR pkgs [pkgs ...]

positional arguments:
  pkgs                  ROS2 packages to generate files from

optional arguments:
  -h, --help            show this help message and exit
  -t TARGET, --target TARGET
                        target language
  -o OUTDIR, --outdir OUTDIR
                        Output directory
```

# Running tests

Source a ros distro

```bash
. /opt/ros/foxy/setup.bash
```

Run tests

```bash
npm test
```
