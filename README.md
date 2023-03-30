# Autoware Signage

## Tested environments

| OS           | python     | ros           |
| ------------ | ---------- | ------------- |
| Ubuntu 20.04 | python3.8  | ros2 galactic |
| Ubuntu 22.04 | python3.10 | ros2 humble   |

## setup

### Install Autoware

refer to here
<https://autowarefoundation.github.io/autoware-documentation/main/installation/>

### setup

```bash
source {AUTOWARE_PATH}/install/setup.bash
bash setup.sh
```

IF you wont to use other FMS Domain, change environment variable.

```bash
export FMS_URL=fms.web.auto
```

## start

```bash
source {AUTOWARE_PATH}/install/setup.bash
bash start.sh
```

## rebuild

```bash
colcon build
```

## License

voice：jtalk
