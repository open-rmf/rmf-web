ICONPATH=rmf_demos_ws/src/rmf/rmf_demos/rmf_demo_maps/maps/office/icons
[ -d "public/assets/icons/" ] && rm -rf public/assets/icons
cp -r $ICONPATH/ public/assets/icons/