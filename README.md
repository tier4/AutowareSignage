# FieldOperatorApplication

## 確認環境
- Ubuntu20.04
- Python3.5
- Qt5.9
- ros foxy

## setup

### Install Autoware
下記参照
https://github.com/tier4/AutowareArchitectureProposal.proj

### setup FOA
```
cd GSM8Signage
source {AUTOWARE_PATH}/install/setup.bash
bash setup.sh
```


## start
```
source {AUTOWARE_PATH}/install/setup.bash
bash start.sh
```

## rebuild
```
cd GSM8Signage
colcon build
```
