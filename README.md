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

### Environment variables

The following environment variables must be set before launching:

- `FMS_URL`: FMS domain (e.g. set by your operations team)
- `AUTOWARE_IP`: IP address of the Autoware PC running the FMS gateway
- `AUTOWARE_PORT`: TCP port of the FMS gateway on the Autoware PC

On production vehicles these are provisioned by the `x2_signage_env_setup` role
in [`autoware_ecu_system_setup`](https://github.com/tier4/autoware_ecu_system_setup),
which writes them into `autoware.env`. The signage service then sources
`/opt/autoware/services/set-autoware-env/setup.sh` before launch.

For local development, export them in your shell before running `start.sh`:

```bash
export FMS_URL=<your-fms-domain>
export AUTOWARE_IP=<autoware-pc-ip>
export AUTOWARE_PORT=<autoware-pc-port>
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
