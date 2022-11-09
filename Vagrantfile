Vagrant.configure("2") do |config|
    config.ssh.username = "SINDy-PX4"
    config.vm.box = "ubuntu/xenial64"
    config.vm.provision "shell", path: "PATH_TO_INSTALL_SCRIPT"
    config.vm.synced_folder ".", "/home/vagrant/project"
    config.vm.provider "vmware" do |vb|
      vb.memory = 2048  # building cctool requires 2GB of RAM
      vb.customize ["modifyvm", :id, "--usb", "on"]
      vb.customize ["modifyvm", :id, "--usbehci", "on"]
      vb.customize ["usbfilter", "add", "0", 
        "--target", :id,
        "--name", "CC Debugger",
        "--manufacturer", "Texas Instruments",
        "--product", "CC Debugger"]
      vb.customize ["usbfilter", "add", "0",
        "--target", :id,
        "--name", "TTL232R-3V3",
        "--manufacturer", "FTDI",
        "--product", "TTL232R-3V3"]
    end
  end