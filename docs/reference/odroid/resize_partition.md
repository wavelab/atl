# Odroid Partition Resize

On a freshly flashed Odriod with Ubuntu image, it is most likely the image only
utilizes 4GB of the eMMC or SD card you're using. To resize the paritition run
the following script:





save the script as `resize.sh`, give it execution rights with:

    chmod +x resize.sh

then run the script by:

    sudo ./resize.sh
