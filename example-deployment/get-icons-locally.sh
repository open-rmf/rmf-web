#!/bin/bash
if [[ -z "$HOME_RMF_WEB" ]]; then
    echo "Must provide HOME_RMF_WEB in environment" 
    return 2
fi


if [[ -z "$ICONS_FOLDER" ]]; then
    echo "Must provide and icon folder" 
    return 2
fi

cd $HOME_RMF_WEB/packages/dashboard
echo -e "{\"path\":\"$ICONS_FOLDER\"}" > .resources.json
node ./scripts/setup/get-icons.js
cd -