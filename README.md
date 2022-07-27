# FieldOperatorApplication

## 確認環境

- Ubuntu20.04
- Python3.5
- Qt5.9
- ros galactic

## setup

### Install Autoware

下記参照
<https://github.com/tier4/AutowareArchitectureProposal.proj>

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

音声：jtalk
