ICONSURL=https://github.com/matiasbavera/romi-dashboard-icons/archive/master.zip
[ -d "public/assets/icons/" ] && rm -rf public/assets/icons
mkdir -p public/assets/icons
wget --no-check-certificate --content-disposition $ICONSURL
unzip romi-dashboard-icons-master.zip
cp -r romi-dashboard-icons-master/* public/assets/icons/ 
rm -rf romi-dashboard-icons-master
rm romi-dashboard-icons-master.zip
