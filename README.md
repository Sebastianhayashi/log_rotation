# README


## Usage

```
colcon build 

./test.sh
```

## Parameter

### `LOG_DIR`

Log file path , default in `$HOME/.ros/log`

### `LOG_FILE`

this is log file name

### `BACKUP_LOG_PATTERN`

this a back up log file name,in default like: my_node.1.log, my_node.2.log

### `sleep`

 this is a parameter for waiting ros node to generate log. 

sleep 10 -> waiting 10 second
sleep 15 -> waiting 15 second

**You can change the path in test.sh file**