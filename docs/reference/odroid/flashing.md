# Odroid Flashing

The following will detail how to flash a linux distro onto the eMMC or MicroSD
and boot.

1.) Uncompress the `.xz` file

    unxz file.image.xz

Note: this replaces the `.xz` file to `.img`

2.) Write the image to eMMC or MicroSD with `dd`

    sudo dd if=</path/to/img> of=</dev/of/card/> bs=1M conv=fsync
    sync; sync; sync;

The `sync` command is very important, as it flushes all cached data, this
command has to be executed multiple times to ensure all data is all flushed.

NOTE: Many SD-to-MicroSD adapter don't work correctly. You need to connect the
flash storage to your USB reader directly.
