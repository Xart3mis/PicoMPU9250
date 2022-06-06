cd ~/Documents/PicoMPU9250
python tools/pyboard.py -d /dev/ttyACM0 -f cp src/main.py :
python tools/pyboard.py -d /dev/ttyACM0 -f cp src/lib/*.py :lib/
python tools/pyboard.py -d /dev/ttyACM0 $1