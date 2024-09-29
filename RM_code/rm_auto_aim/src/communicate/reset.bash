while true; 
do
    lsusb | sed 's/:/ /g' | awk '{print $2, $4}' | sed 's/ /\//g' | xargs -I {} sudo usbreset /dev/bus/usb/{}
    sleep 5
    sudo chmod 777 /dev/ttyACM0
    bash ./communicate/shell/run.bash
done
