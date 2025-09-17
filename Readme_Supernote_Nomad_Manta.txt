COMPILING the u-boot_Nomad_Manta:
  cd /home/name/build/ 
git clone git@github.com:Supernote-Ratta/u-boot_Nomad_Manta.git
git clone git@github.com:Supernote-Ratta/prebuilts_Nomad_Manta.git
git@github.com:Supernote-Ratta/rkbin_Nomad_Manta.git

mv prebuilts_Nomad_Manta/ prebuilts
mv rkbin_Nomad_Manta/ rkbin

cd u-boot_Nomad_Manta/

 ./make.sh rk3566-multidtb-eink
