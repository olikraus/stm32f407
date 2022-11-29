sh /home/kraus/.arduino15/packages/STMicroelectronics/tools/STM32Tools/2.1.1/stm32CubeProg.sh 2 ./tmp/*.bin -g -tm 2000
sleep 1
stty -F /dev/ttyACM0 sane 115200 && cat /dev/ttyACM0
#minicom -D /dev/ttyACM0  -b 9600
